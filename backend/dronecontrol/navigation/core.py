import asyncio
import numpy as np
from abc import ABC, abstractmethod
from enum import Enum, auto

from ..utils import dist_ned, relative_gps, heading_ned, heading_gps, offset_from_gps
from backend import dronecontrol


class WayPointType(Enum):
    # Type of setpoint              # Expected data structure
    POS_NED = auto()                # array [pos_n, pos_e, pos_d, yaw]
    POS_VEL_NED = auto()            # array [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, yaw]
    POS_VEL_ACC_NED = auto()        # array [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, yaw]
    VEL_NED = auto()                # array [vel_n, vel_e, vel_d, yaw]
    POS_GLOBAL = auto()             # array [lat, long, amsl, yaw]


class Waypoint:
    def __init__(self, waypoint_type: WayPointType,
                 pos: np.ndarray | None = None,
                 vel: np.ndarray | None = None,
                 acc: np.ndarray | None = None,
                 gps: np.ndarray | None = None,
                 yaw: float | None = None):
        # Internal data structure, form [x, y, z, xvel, yvel, zvel, xacc, yacc, zacc, lat, long, amsl, yaw]
        self._array: np.ndarray[float] = np.empty((13,))

        self._array[:] = None
        self._array[-1] = yaw
        match waypoint_type:
            case WayPointType.POS_NED:
                self._array[:3] = pos
            case WayPointType.POS_VEL_NED:
                self._array[:3] = pos
                self._array[3:6] = vel
            case WayPointType.POS_VEL_ACC_NED:
                self._array[:3] = pos
                self._array[3:6] = vel
                self._array[6:9] = acc
            case WayPointType.POS_GLOBAL:
                self._array[9:12] = gps
            case WayPointType.VEL_NED:
                self._array[3:6] = vel
        self.type = waypoint_type

    def __str__(self):
        return f"{self.type}, {self.pos}, {self.vel}, {self.acc}, {self.gps}, {self.yaw}"

    @property
    def pos(self):
        return self._array[:3]

    @pos.setter
    def pos(self, new_pos: np.ndarray):
        self._array[:3] = new_pos

    @property
    def vel(self):
        return self._array[3:6]

    @vel.setter
    def vel(self, new_vel: np.ndarray):
        self._array[3:6] = new_vel

    @property
    def acc(self):
        return self._array[6:9]

    @acc.setter
    def acc(self, new_acc: np.ndarray):
        self._array[6:9] = new_acc

    @property
    def gps(self):
        return self._array[9:12]

    @gps.setter
    def gps(self, new_gps: np.ndarray):
        self._array[9:12] = new_gps

    @property
    def yaw(self):
        return self._array[-1]

    @yaw.setter
    def yaw(self, new_yaw: float):
        self._array[-1] = new_yaw

    def distance(self, other: "Waypoint"):
        return dist_ned(self.pos, other.pos)

    def heading_ned(self, other: "Waypoint"):
        return heading_ned(self.pos, other.pos)

    def heading_gps(self, other: "Waypoint"):
        return heading_gps(self.gps, other.gps)

    def shift_gps(self, north: float, east: float, up: float) -> "Waypoint":
        """Returns a new waypoint, offset by north, east and up from this waypoint. Note that the yaw is kept."""
        new_gps = relative_gps(north, east, up, *self.gps)
        return Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray(new_gps), yaw=self.yaw)

    def offset_gps(self, initial: "Waypoint", target: "Waypoint"):
        """ Creates a vector between two waypoints and then creates a new Waypoint offset from this
        waypoint by the same distance and heading."""
        new_gps = offset_from_gps(self.gps, initial.gps, target.gps)
        return Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray(new_gps), yaw=self.yaw)


class TrajectoryGenerator(ABC):
    """ Abstract base class for trajectory generators."""

    CAN_DO_GPS = False
    WAYPOINT_TYPES = set()
    """ These determine the type of intermediate waypoints a trajectory generator may produce"""

    def __init__(self, drone: "dronecontrol.drone.Drone", logger, waypoint_type, use_gps=False):
        """

        Should be called at the end of subclass constructors.

        :param drone:
        :param logger:
        :param use_gps:
        """
        assert waypoint_type in self.WAYPOINT_TYPES, (f"Invalid waypoint type {waypoint_type} "
                                                      f"for trajectory generator {self.__class__.__name__}")
        self.drone = drone
        self.logger = logger
        self.waypoint_type = waypoint_type
        self.use_gps = use_gps
        self.target_position: Waypoint | None = None

    def set_target(self, waypoint: Waypoint):
        """ Sets the target position that we will try to fly towards.
        """
        self.target_position = waypoint

    @abstractmethod
    async def create_trajectory(self) -> None:
        """ Function that performs whatever calculations are initially necessary to be able to produce waypoints.
        This function may be quite slow. """

    @abstractmethod
    def next(self) -> Waypoint:
        """ The next waypoint, once the follower algorithm asks for another one.

        This function must execute quickly, as it might be called with a high frequency during flight. Trajectory
        followers should call it when they think they have reached the current waypoint. Should return None if the
        trajectory generator hasn't produced any waypoints or has run out."""


class TrajectoryFollower(ABC):
    """ Abstract Base class to "follow" a given trajectory and maintain position at waypoints.

    A trajectory follower can work with different types of waypoints, but must be able to process WayPoinType.POS_NED,
    as that is the default case.
    """

    CAN_DO_GPS = False
    SETPOINT_TYPES = set()
    WAYPOINT_TYPES = set()

    def __init__(self, drone: "dronecontrol.drone.Drone", logger, dt, setpoint_type: WayPointType):
        assert setpoint_type in self.SETPOINT_TYPES, (f"Invalid setpoint type {setpoint_type} "
                                                      f"for trajectory follower {self.__class__.__name__}")
        assert setpoint_type in drone.VALID_SETPOINT_TYPES, (f"Invalid setpoint type {setpoint_type} "
                                                             f"for drone {drone.__class__.__name__}")
        self.logger = logger
        self.drone = drone
        self.setpoint_type = setpoint_type
        self.dt = dt  # How often to send setpoints to the FC
        self.current_waypoint: Waypoint | None = None
        self._active = False
        self._following_task: asyncio.Coroutine | None = None

    def activate(self):
        if self._active:
            raise RuntimeWarning("Can't activate trajectory follower, it is already active.")
        else:
            self._active = True
            self._following_task = asyncio.create_task(self.follow())

    async def deactivate(self):
        if not self._active:
            raise RuntimeWarning("Can't deactivate trajectory follower, because it isn't active.")
        else:
            self.logger.debug("Trajectory follower deactivating...")
            self._active = False
            await self._following_task
            self._following_task = None

    @property
    def is_active(self):
        return self._active

    async def follow(self):
        """ Follows waypoints produced from a trajectory generator by sending setpoints to the drone FC.

        Requests a new waypoint from the TG when get_next_waypoint returns True. If the TG does not produce a waypoint,
        holds position instead.
        :return:
        """
        # Use current position as dummy waypoint in case trajectory generator can't produce any yet.
        # TODO: A timer or something so we don't spam the log with "still using current position"
        dummy_waypoint = Waypoint(WayPointType.POS_NED, pos=self.drone.position_ned,
                                  vel=np.zeros((3,)), yaw=self.drone.attitude[2])
        have_waypoints = False
        using_current_position = False
        waypoint = dummy_waypoint
        while self.is_active:
            try:
                if self.get_next_waypoint():
                    self.logger.debug("Getting new waypoint from trajectory generator...")
                    waypoint = self.drone.trajectory_generator.next()
                    if not waypoint:
                        if not using_current_position:
                            self.logger.debug(f"No waypoints, current position: {self.drone.position_ned}")
                            dummy_waypoint = Waypoint(WayPointType.POS_NED, pos=self.drone.position_ned,
                                                      yaw=self.drone.attitude[2])
                            if have_waypoints:
                                self.logger.debug("Generator no longer producing waypoints, using current position")
                                # If we had waypoints, but lost them, use the current position as a dummy waypoint
                                have_waypoints = False
                                using_current_position = True
                            else:  # Never had a waypoint
                                self.logger.debug("Don't have any waypoints from the generator yet, using current position")
                                using_current_position = True
                        else:
                            self.logger.debug("Still using current position...")
                            waypoint = dummy_waypoint
                    else:
                        have_waypoints = True
                        using_current_position = False
                    self.current_waypoint = waypoint
                await self.set_setpoint(waypoint)
                await asyncio.sleep(self.dt)
            except Exception as e:
                self.logger.error("Encountered an exception during following algorithm:", repr(e))
                self.logger.debug(repr(e), exc_info=True)

    @abstractmethod
    def get_next_waypoint(self) -> bool:
        """ Function that determines when to get the next waypoint from the trajectory generator.

        TrajectoryGenerator.next() is called during the follow loop when this function returns True. It should always
        return True if we don't have a waypoint already."""

    @abstractmethod
    async def set_setpoint(self, waypoint):
        """ Function that determines the next setpoint required to get to the target waypoint. This function is called
        once every dt seconds using either the next waypoint from the trajectory generator or the drones current
        position.

        :return:
        """

    def close(self):
        if self._following_task:
            self._following_task.cancel()
