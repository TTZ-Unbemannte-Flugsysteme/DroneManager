import asyncio
import datetime
from collections import deque
import math
import os.path
from enum import Enum, auto
import threading
import platform
import time
from subprocess import Popen, DEVNULL
from concurrent.futures import ProcessPoolExecutor
from abc import ABC, abstractmethod

import numpy as np

from mavsdk import System
from mavsdk.telemetry import FlightMode as MAVSDKFlightMode
from mavsdk.telemetry import FixType as MAVSDKFixType
from mavsdk.telemetry import StatusTextType
from mavsdk.action import ActionError, OrbitYawBehavior
from mavsdk.offboard import PositionNedYaw, PositionGlobalYaw, VelocityNedYaw, AccelerationNed, OffboardError
from mavsdk.manual_control import ManualControlError
from mavsdk.camera import CameraError

from dronecontrol.utils import dist_ned, dist_gps, relative_gps, heading_ned, heading_gps, offset_from_gps
from dronecontrol.utils import parse_address, common_formatter, get_free_port
from dronecontrol.mavpassthrough import MAVPassthrough
from dronecontrol.GMP3 import GMP3, GMP3Config

import logging

_cur_dir = os.path.dirname(os.path.abspath(__file__))
logdir = os.path.abspath("./logs")
os.makedirs(logdir, exist_ok=True)
_mav_server_file = os.path.join(_cur_dir, "mavsdk_server_bin.exe")


# TODO: Separate activate/deactivate for follower algorithm, currently can only be activated by move/flyto and cannot
#  be deactivated at all.
# TODO: Have a look at the entire connection procedure, make some diagrams, plan everything out and refactor any issues
#  nicely
# TODO: Add possibility for fly_to or other "do at" commands to accept waypoints

# TODO: Set Fence functions here, in DM and app
# TODO: Allow multiple fences

FlightMode = MAVSDKFlightMode
FixType = MAVSDKFixType


class Battery:
    def __init__(self):
        self.id = None
        self.remaining = math.nan
        self.consumed = math.nan
        self.voltage = math.nan
        self.temperature = math.nan

    def __str__(self):
        return f"Remain: {self.remaining}   Consumed: {self.consumed}   V: {self.voltage}   T: {self.temperature}"


class WayPointType(Enum):
    # Type of setpoint              # Expected data structure
    POS_NED = auto()                # array [pos_n, pos_e, pos_d, yaw]
    POS_VEL_NED = auto()            # array [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, yaw]
    POS_VEL_ACC_NED = auto()        # array [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, yaw]
    VEL_NED = auto()                # array [vel_n, vel_e, vel_d, yaw]
    POS_GLOBAL = auto()             # array [lat, long, amsl, yaw]


class Drone(ABC, threading.Thread):

    VALID_FLIGHTMODES = set()
    VALID_SETPOINT_TYPES = set()

    def __init__(self, name, *args, log_to_file=True, **kwargs):
        threading.Thread.__init__(self)
        self.name = name
        self.drone_addr = None
        self.drone_ip = None
        self.action_queue: deque[tuple[asyncio.Coroutine, asyncio.Future]] = deque()
        self.current_action: asyncio.Task | None = None
        self.should_stop = threading.Event()
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        self.logging_handlers = []
        self.log_to_file = log_to_file
        if self.log_to_file:
            log_file_name = f"drone_{self.name}_{datetime.datetime.now()}"
            log_file_name = log_file_name.replace(":", "_").replace(".", "_") + ".log"
            file_handler = logging.FileHandler(os.path.join(logdir, log_file_name))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(common_formatter)
            self.add_handler(file_handler)

        self.position_update_rate: float = 10
        self.fence: Fence | None = None
        self.trajectory_generator: TrajectoryGenerator | None = None
        self.trajectory_follower: TrajectoryFollower | None = None

        self.is_paused = False
        self.mav_conn: MAVPassthrough | None = None
        self.start()
        asyncio.create_task(self._task_scheduler())

    def run(self):
        while not self.should_stop:
            pass

    async def _task_scheduler(self):
        while True:
            while len(self.action_queue) > 0:
                if self.is_paused:
                    await asyncio.sleep(0.1)
                else:
                    action, fut = self.action_queue.popleft()
                    self.current_action = asyncio.create_task(action)
                    try:
                        result = await self.current_action
                        fut.set_result(result)
                        self.current_action = None
                    except asyncio.CancelledError:
                        pass
                    except Exception as e:
                        fut.set_exception(e)
            else:
                await asyncio.sleep(0.1)

    def schedule_task(self, coro) -> asyncio.Future:
        fut = asyncio.get_running_loop().create_future()
        self.action_queue.append((coro, fut))
        return fut

    def execute_task(self, coro) -> asyncio.Future:
        self.clear_queue()
        self.cancel_action()
        return self.schedule_task(coro)

    def add_handler(self, handler):
        self.logger.addHandler(handler)
        self.logging_handlers.append(handler)

    @abstractmethod
    async def stop_execution(self):
        """ Stops the thread. This function should be called at the end of any implementing function.

        :return:
        """
        self.should_stop.set()

    def pause(self):
        """ Pause task execution by setting self.is_paused to True.

        Note that it is not possible to "pause" what the drone is doing in a general way. What "pausing" a task does or
        if a task can even be paused depends on the specific task and implementation. Subclasses must define and
        implement this behaviour themselves.
        However, pausing is always possible between tasks, and this is the default behaviour for subclasses that do not
        implement any of their own: When paused, drones will finish their current task and then wait until unpaused
        before beginning the next task."""
        self.is_paused = True
        self.logger.debug("Pausing...")

    def resume(self):
        """ Resume executing tasks. """
        self.is_paused = False
        self.logger.debug("Resuming...")

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @property
    @abstractmethod
    def is_armed(self) -> bool:
        pass

    @property
    @abstractmethod
    def flightmode(self) -> FlightMode:
        pass

    @property
    @abstractmethod
    def in_air(self) -> bool:
        pass

    @property
    def autopilot(self) -> str:
        return self.mav_conn.drone_autopilot

    @property
    @abstractmethod
    def fix_type(self) -> FixType:
        pass

    @property
    @abstractmethod
    def position_global(self) -> np.ndarray:
        """

        :return: Array with the GPS coordinates [latitude, longitude, AMSL]
        """
        pass

    @property
    @abstractmethod
    def position_ned(self) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def velocity(self) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def attitude(self) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def batteries(self) -> dict[int, Battery]:
        pass

    @abstractmethod
    async def connect(self, drone_addr, *args, **kwargs):
        pass

    @abstractmethod
    async def disconnect(self, force=False) -> bool:
        pass

    @abstractmethod
    async def arm(self) -> bool:
        pass

    @abstractmethod
    async def disarm(self) -> bool:
        pass

    @abstractmethod
    async def takeoff(self, altitude=2.0) -> bool:
        """ Takes off to the specified altitude above current position.

        Note that altitude is positive.

        :param altitude: Takeoff altitude above.
        :return:
        """
        pass

    @abstractmethod
    async def change_flight_mode(self, flightmode) -> bool:
        pass

    def is_at_pos(self, target_pos, tolerance=0.25) -> bool:
        """

        :param target_pos: Array with target position. If a yaw is also passed (i.e. array length 4), it is ignored.
        :param tolerance: How close we have to be to the target position to be considered "at" it.
        :return:
        """
        cur_pos = self.position_ned
        if dist_ned(cur_pos, target_pos[:3]) < tolerance:
            return True
        return False

    def is_at_heading(self, target_heading, tolerance=1) -> bool:
        target_heading = (target_heading + 180) % 360 - 180
        cur_heading = self.attitude[2]
        if abs(cur_heading - target_heading) < tolerance:
            return True
        return False

    def is_at_gps(self, target_gps, tolerance=0.25) -> bool:
        if dist_gps(target_gps, self.position_global[:3]) < tolerance:
            return True
        return False

    @abstractmethod
    async def yaw_to(self, x, y, z, target_yaw, yaw_rate, tolerance) -> bool:
        pass

    @abstractmethod
    async def spin_at_rate(self, yaw_rate, duration, direction="cw") -> bool:
        pass

    def set_fence(self, fence_type: type["Fence"], *args, **kwargs):
        self.fence = fence_type(*args, **kwargs)

    def check_waypoint(self, waypoint: "Waypoint"):
        """ Check if a waypoint is valid and within any geofence (if such a fence is set)"""
        try:
            waypoint_valid = not self.fence or (self.fence and self.fence.check_waypoint_compatible(waypoint))
        except Exception:
            waypoint_valid = False
        return waypoint_valid

    @abstractmethod
    async def set_setpoint(self, setpoint: "Waypoint") -> bool:
        pass

    @abstractmethod
    async def fly_to(self, x=None, y=None, z=None, lat=None, long=None, amsl=None, yaw=None, tolerance=0.25):
        """ Fly to the specified position.

        :param x:
        :param y:
        :param z:
        :param lat:
        :param long:
        :param amsl:
        :param yaw:
        :param tolerance:
        :return:
        """
        pass

    @abstractmethod
    async def move(self, north, east, down, yaw, use_gps=True, tolerance=0.25):
        """ Move from the current position by the specified distances.

        :param north:
        :param east:
        :param down:
        :param yaw:
        :param use_gps:
        :param tolerance:
        :return:
        """
        pass

    @abstractmethod
    async def orbit(self, radius, velocity, latitude, longitude, amsl) -> bool:
        pass

    @abstractmethod
    async def land(self) -> bool:
        pass

    @abstractmethod
    async def stop(self) -> bool:
        pass

    @abstractmethod
    async def kill(self) -> bool:
        pass

    def clear_queue(self) -> None:
        """ Clears the action queue.

        Does not cancel the current action.

        :return:
        """
        self.logger.debug("Clearing action queue")
        self.action_queue.clear()

    def cancel_action(self) -> None:
        """ Cancels the current action task

        :return:
        """
        if self.current_action:
            self.logger.debug("Cancelling current action!")
            self.current_action.cancel()


class DroneMAVSDK(Drone):

    VALID_FLIGHTMODES = {"hold", "offboard", "return", "land", "takeoff", "position", "altitude"}
    # This attribute is for checking which flight modes can be changed into manually
    VALID_SETPOINT_TYPES = {WayPointType.POS_NED,
                            WayPointType.POS_VEL_NED,
                            WayPointType.POS_VEL_ACC_NED,
                            WayPointType.VEL_NED,
                            WayPointType.POS_GLOBAL}
    # What type of trajectory setpoints this classes fly_<> commands can follow. This limits what Trajectory generators
    # can be used.

    def __init__(self, name, mavsdk_server_address: str | None = None, mavsdk_server_port: int = 50051):
        super().__init__(name)
        self.system: System | None = None
        self.server_addr = mavsdk_server_address
        self.server_port = mavsdk_server_port
        self.gcs_system_id = None
        self.gcs_component_id = None
        self.drone_system_id = None           # Populated during connection process
        self.drone_component_id = None
        self._server_process: Popen | None = None
        self._is_connected: bool = False
        self._is_armed: bool = False
        self._flightmode: FlightMode = FlightMode.UNKNOWN
        self._in_air: bool = False
        self._gps_info: FixType | None = None
        self._position_g: np.ndarray = np.zeros((4,), dtype=np.double)
        # Latitude, Longitude, AMSL, Relative altitude to takeoff
        self._position_ned: np.ndarray = np.zeros((3,))     # NED
        self._velocity: np.ndarray = np.zeros((3,))         # NED
        self._attitude: np.ndarray = np.zeros((3,))         # Roll, pitch and yaw, with positives right up and right.
        self._heading: float = math.nan
        self._batteries: dict[int, Battery] = {}
        self._running_tasks = []

        # How often (per second) we request position information from the drone. The same interval is used by path
        # planning algorithms for their time resolution.
        self.position_update_rate = 5

        self.mav_conn = MAVPassthrough(loggername=f"{name}_MAVLINK", log_messages=False)

        #self.trajectory_generator = StaticWaypoints(self, self.logger, WayPointType.POS_NED)
        self.trajectory_generator = GMP3Gen(self, 1/self.position_update_rate, self.logger, use_gps=False)
        self.trajectory_follower = DirectSetpointFollower(self, self.logger, 1/self.position_update_rate,
                                                          WayPointType.POS_VEL_NED)
        #self.trajectory_follower = VelocityControlFollower(self, self.logger, 1/self.position_update_rate)

        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized Drone {self.name}, {self.__class__.__name__}:\n   {attr_string}")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_armed(self) -> bool:
        return self._is_armed

    @property
    def flightmode(self) -> FlightMode:
        return self._flightmode

    @property
    def in_air(self) -> bool:
        return self._in_air

    @property
    def fix_type(self) -> FixType:
        return self._gps_info

    @property
    def position_global(self) -> np.ndarray:
        return self._position_g

    @property
    def position_ned(self) -> np.ndarray:
        return self._position_ned

    @property
    def velocity(self) -> np.ndarray:
        return self._velocity

    @property
    def attitude(self) -> np.ndarray:
        return self._attitude

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def batteries(self) -> dict[int, Battery]:
        return self._batteries

    async def connect(self, drone_address, system_id=0, component_id=0) -> bool:
        # If we are on windows, we can't rely on the MAVSDK to have the binary installed.
        # If we use serial, loc is the path and appendix the baudrate, if we use udp it is IP and port
        self.gcs_system_id = system_id
        self.gcs_component_id = component_id
        scheme, loc, appendix = parse_address(string=drone_address)
        self.drone_addr = f"{scheme}://{loc}:{appendix}"
        self.logger.debug(f"Connecting to drone {self.name} @ {self.drone_addr}")
        if self.mav_conn:
            mavsdk_passthrough_port = get_free_port()
            mavsdk_passthrough_string = f"udp://:{mavsdk_passthrough_port}"
            passthrough_gcs_string = f"127.0.0.1:{mavsdk_passthrough_port}"
        else:
            if scheme == "serial":
                mavsdk_passthrough_string = f"serial://{loc}"
            else:
                mavsdk_passthrough_string = f"{scheme}://:{appendix}"

        try:
            if self.server_addr is None:
                self.logger.debug(f"Starting up own MAVSDK Server instance with app port {self.server_port} and remote "
                                  f"connection {mavsdk_passthrough_string}")
            if self.server_addr is None and platform.system() == "Windows":
                self._server_process = Popen(f"{_mav_server_file} -p {self.server_port} {mavsdk_passthrough_string}",
                                             stdout=DEVNULL, stderr=DEVNULL)
                self.server_addr = "127.0.0.1"
            self.system = System(mavsdk_server_address=self.server_addr, port=self.server_port,
                                 sysid=system_id, compid=component_id)

            connected = asyncio.create_task(self.system.connect(system_address=mavsdk_passthrough_string))

            # Create passthrough
            if self.mav_conn:
                # Wait to try and make sure that the mavsdk server has started before booting up passthrough
                await asyncio.sleep(0.5)
                self.logger.debug(
                    f"Connecting passthrough to drone @{loc}:{appendix} and MAVSDK server @ {passthrough_gcs_string}")
                self.mav_conn.connect_drone(loc, appendix, scheme=scheme)
                self.mav_conn.connect_gcs(passthrough_gcs_string)

                while not self.mav_conn.connected_to_drone() or not self.mav_conn.connected_to_gcs():
                    self.logger.debug(f"Waiting on passthrough to connect. "
                                      f"Drone: {self.mav_conn.connected_to_drone()}, "
                                      f"GCS: {self.mav_conn.connected_to_gcs()}")
                    await asyncio.sleep(0.1)
                self.logger.debug("Connected passthrough!")
                self.drone_system_id = self.mav_conn.drone_system
                self.drone_component_id = self.mav_conn.drone_component

            await connected

            async for state in self.system.core.connection_state():
                if state.is_connected:
                    await self._configure_message_rates()
                    await self._schedule_update_tasks()
                    self.logger.debug("Connected!")
                    return True
        except Exception as e:
            self.logger.debug(f"Exception during connection: {repr(e)}", exc_info=True)
        return False

    async def disconnect(self, force=False):
        self.clear_queue()
        self.cancel_action()
        if force or not self._is_armed:
            if force:
                self.logger.debug("Force disconnecting from drone...")
            await self.stop_execution()
            return True
        else:
            self.logger.warning("Can't disconnect from an armed drone!")
            return False

    async def _schedule_update_tasks(self) -> None:
        self._running_tasks.append(asyncio.create_task(self._connect_check()))
        self._running_tasks.append(asyncio.create_task(self._arm_check()))
        self._running_tasks.append(asyncio.create_task(self._flightmode_check()))
        self._running_tasks.append(asyncio.create_task(self._inair_check()))
        self._running_tasks.append(asyncio.create_task(self._gps_check()))
        self._running_tasks.append(asyncio.create_task(self._g_pos_check()))
        self._running_tasks.append(asyncio.create_task(self._vel_rpos_check()))
        self._running_tasks.append(asyncio.create_task(self._att_check()))
        self._running_tasks.append(asyncio.create_task(self._battery_check()))
        self._running_tasks.append(asyncio.create_task(self._status_check()))

    async def _configure_message_rates(self) -> None:
        try:
            await self.system.telemetry.set_rate_position(self.position_update_rate)
            await self.system.telemetry.set_rate_position_velocity_ned(self.position_update_rate)
            await self.system.telemetry.set_rate_attitude_euler(self.position_update_rate)
        except Exception as e:
            self.logger.warning(f"Couldn't set message rate!")
            self.logger.debug(f"{repr(e)}", exc_info=True)

    async def _connect_check(self):
        if self.mav_conn:
            while True:
                self._is_connected = self.mav_conn.connected_to_drone() and self.mav_conn.connected_to_gcs()
                await asyncio.sleep(1 / self.position_update_rate)
        else:
            async for state in self.system.core.connection_state():
                self._is_connected = state.is_connected

    async def _arm_check(self):
        async for arm in self.system.telemetry.armed():
            self._is_armed = arm

    async def _flightmode_check(self):
        async for flightmode in self.system.telemetry.flight_mode():
            self._flightmode = flightmode

    async def _inair_check(self):
        async for in_air in self.system.telemetry.in_air():
            self._in_air = in_air

    async def _gps_check(self):
        async for gps in self.system.telemetry.gps_info():
            self._gps_info = gps.fix_type

    async def _g_pos_check(self):
        async for pos in self.system.telemetry.position():
            self._position_g[0] = pos.latitude_deg
            self._position_g[1] = pos.longitude_deg
            self._position_g[2] = pos.absolute_altitude_m
            self._position_g[3] = pos.relative_altitude_m

    async def _vel_rpos_check(self):
        async for pos_vel in self.system.telemetry.position_velocity_ned():
            self._velocity[0] = pos_vel.velocity.north_m_s
            self._velocity[1] = pos_vel.velocity.east_m_s
            self._velocity[2] = pos_vel.velocity.down_m_s
            new_pos = np.array([pos_vel.position.north_m, pos_vel.position.east_m, pos_vel.position.down_m])
            self._position_ned = new_pos

    async def _att_check(self):
        async for att in self.system.telemetry.attitude_euler():
            self._attitude[0] = att.roll_deg
            self._attitude[1] = att.pitch_deg
            self._attitude[2] = att.yaw_deg

    async def _heading_check(self):
        async for heading in self.system.telemetry.heading():
            self._heading = heading.heading_deg

    async def _battery_check(self):
        async for battery in self.system.telemetry.battery():
            battery_id = battery.id
            if battery_id in self._batteries:
                own_battery = self._batteries[battery_id]
            else:
                own_battery = Battery()
                own_battery.id = battery_id
                self._batteries[battery_id] = own_battery
            own_battery.consumed = battery.capacity_consumed_ah
            own_battery.remaining = battery.remaining_percent
            own_battery.voltage = battery.voltage_v
            own_battery.temperature = battery.temperature_degc

    async def _status_check(self):
        async for message in self.system.telemetry.status_text():
            if message.type is StatusTextType.DEBUG:
                self.logger.debug(f"{message.text}")
            elif message.type in [StatusTextType.INFO, StatusTextType.NOTICE]:
                self.logger.info(f"{message.text}")
            elif message.type is StatusTextType.WARNING:
                self.logger.warning(f"{message.text}")
            else:
                self.logger.error(f"{message.text}")

    async def arm(self):
        timeout = 5
        self.logger.info("Arming!")
        await super().arm()
        result = await self._error_wrapper(self.system.action.arm, ActionError)
        if result and not isinstance(result, Exception):
            start_time = time.time()
            while not self.is_armed:
                await asyncio.sleep(1 / self.position_update_rate)
                if time.time() - start_time > timeout:
                    self.logger.warning("Arming timed out!")
                    return False
            self.logger.info("Armed!")
        else:
            self.logger.warning("Couldn't arm!")
        return result

    async def disarm(self):
        timeout = 5
        self.logger.info("Disarming!")
        if self.trajectory_follower.is_active:
            await self.trajectory_follower.deactivate()
        await super().disarm()
        result = await self._error_wrapper(self.system.action.disarm, ActionError)
        if result and not isinstance(result, Exception):
            start_time = time.time()
            while self.is_armed:
                await asyncio.sleep(1 / self.position_update_rate)
                if time.time() - start_time > timeout:
                    self.logger.warning("Disarming timed out!")
                    return False
            self.logger.info("Disarmed!")
        else:
            self.logger.warning("Couldn't disarm!")
        return result

    def _can_takeoff(self):
        if not self.is_armed:
            raise RuntimeError("Can't take off without being armed!")

    async def takeoff(self, altitude=2.0) -> bool:
        """

        :param altitude:
        :return:
        """
        if self.trajectory_follower.is_active:
            await self.trajectory_follower.deactivate()
        return await self._takeoff_using_offboard(altitude=altitude)

    def _get_pos_ned_yaw(self):
        pos_yaw = np.zeros((4,))
        pos_yaw[:3] = self.position_ned
        pos_yaw[3] = self.attitude[2]
        return pos_yaw

    async def _takeoff_using_takeoffmode(self, altitude=2.0):
        """

        :param altitude: Currently ignored.
        :return:
        """
        self.logger.info("Trying to take off...")
        await super().takeoff(altitude=altitude)
        self._can_takeoff()
        result = await self._error_wrapper(self.system.action.takeoff, ActionError)
        if isinstance(result, Exception):
            self.logger.warning("Couldn't takeoff!")
        while self.flightmode is not FlightMode.TAKEOFF:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Taking off!")
        while self.flightmode is FlightMode.TAKEOFF:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Completed takeoff!")
        return True

    async def _takeoff_using_offboard(self, altitude=2.0, tolerance=0.25):
        """

        :param altitude:
        :param tolerance:
        :return:
        """
        self.logger.info(f"Trying to take off to {altitude}m in offboard mode...")
        await super().takeoff(altitude=altitude)
        self._can_takeoff()
        target_pos_yaw = self._get_pos_ned_yaw()
        target_pos_yaw[2] = target_pos_yaw[2] - altitude
        await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=target_pos_yaw[:3], yaw=target_pos_yaw[3]))
        if self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")
        self.logger.info("Taking off!")
        while True:
            if self.is_at_pos(target_pos_yaw, tolerance=tolerance):
                self.logger.info(f"Takeoff completed!")
                return True
            await asyncio.sleep(1 / self.position_update_rate)

    async def change_flight_mode(self, flightmode: str, timeout: float = 5):
        self.logger.info(f"Changing flight mode to {flightmode}")
        await super().change_flight_mode(flightmode)
        start_time = time.time()
        if flightmode == "hold":
            result = await self._error_wrapper(self.system.action.hold, ActionError)
            target_flight_mode = FlightMode.HOLD
        elif flightmode == "offboard":
            result = await self._error_wrapper(self.system.offboard.start, OffboardError)
            target_flight_mode = FlightMode.OFFBOARD
        elif flightmode == "return":
            result = await self._error_wrapper(self.system.action.return_to_launch, ActionError)
            target_flight_mode = FlightMode.RETURN_TO_LAUNCH
        elif flightmode == "land":
            result = await self._error_wrapper(self.system.action.land, ActionError)
            target_flight_mode = FlightMode.LAND
        elif flightmode == "takeoff":
            self._can_takeoff()
            result = await self._error_wrapper(self.system.action.takeoff, ActionError)
            target_flight_mode = FlightMode.TAKEOFF
        elif flightmode == "position":
            result = await self._error_wrapper(self.system.manual_control.start_position_control, ManualControlError)
            target_flight_mode = FlightMode.POSCTL
        elif flightmode == "altitude":
            result = await self._error_wrapper(self.system.manual_control.start_altitude_control, ManualControlError)
            target_flight_mode = FlightMode.ALTCTL
        else:
            raise KeyError(f"{flightmode} is not a valid flightmode!")
        if isinstance(result, Exception):
            self.logger.warning(f"Couldn't change flight mode due to exception {repr(result)}")
            return False
        elif not result:
            self.logger.warning("Couldn't change flight mode due to a programmatic impossibility!")
            return False
        else:
            while self.flightmode != target_flight_mode and time.time() < start_time + timeout:
                await asyncio.sleep(1/self.position_update_rate)
            if self.flightmode != target_flight_mode:
                self.logger.warning("Drone accepted command, but flight mode change timed out! "
                                    "Possible connection issue.")
                return False
            else:
                self.logger.info(f"New flight mode {self.flightmode}!")
                return True

    async def set_setpoint(self, setpoint: "Waypoint"):
        setpoint_type = setpoint.type
        if setpoint_type == WayPointType.POS_NED:
            point_ned_yaw = PositionNedYaw(*setpoint.pos, setpoint.yaw)
            return await self._error_wrapper(self.system.offboard.set_position_ned, OffboardError, point_ned_yaw)
        elif setpoint_type == WayPointType.POS_VEL_NED:
            point_ned_yaw = PositionNedYaw(*setpoint.pos, setpoint.yaw)
            velocity_ned_yaw = VelocityNedYaw(*setpoint.vel, setpoint.yaw)
            return await self._error_wrapper(self.system.offboard.set_position_velocity_ned, OffboardError,
                                             point_ned_yaw, velocity_ned_yaw)
        elif setpoint_type == WayPointType.POS_VEL_ACC_NED:
            yaw = setpoint.yaw
            point_ned_yaw = PositionNedYaw(*setpoint.pos, yaw)
            velocity_ned_yaw = VelocityNedYaw(*setpoint.vel, yaw)
            acc_ned = AccelerationNed(*setpoint.acc)
            return await self._error_wrapper(self.system.offboard.set_position_velocity_acceleration_ned,
                                             OffboardError,
                                             point_ned_yaw,
                                             velocity_ned_yaw,
                                             acc_ned)
        elif setpoint_type == WayPointType.VEL_NED:
            vel_yaw = VelocityNedYaw(*setpoint.vel, setpoint.yaw)
            return await self._error_wrapper(self.system.offboard.set_velocity_ned, OffboardError, vel_yaw)
        elif setpoint_type == WayPointType.POS_GLOBAL:
            latitude, longitude, amsl = setpoint.gps
            alt_type = PositionGlobalYaw.AltitudeType.AMSL
            position = PositionGlobalYaw(lat_deg=latitude, lon_deg=longitude, alt_m=amsl,
                                         yaw_deg=setpoint.yaw, altitude_type=alt_type)
            return await self._error_wrapper(self.system.offboard.set_position_global, OffboardError, position)
        else:
            raise RuntimeError("Invalid SetPointType!")

    def _can_do_in_air_commands(self):
        # TODO: Figure out how to do this with ardupilot. Currently the in_air detection seems very poor
        if self.autopilot == "ardupilot":
            return True
        if not self.is_armed or not self.in_air:
            return False
        return True

    async def yaw_to(self, x, y, z, target_yaw, yaw_rate=30, tolerance=2):
        """ Move to the point x,y,z, yawing to the target heading as you do so at the specified rate.

        To spin in place pass current position to x,y,z. Pausable.

        :param x:
        :param y:
        :param z:
        :param target_yaw: Heading as a degree fom -180 to 180, right positive, 0 forward.
        :param yaw_rate:
        :param tolerance: How close we have to get to the heading before this function returns.
        :return:
        """
        # Add 180, take modulo 360 and subtract 180 to get proper range
        if self.trajectory_follower.is_active:
            await self.trajectory_follower.deactivate()
        og_yaw = self.attitude[2]
        dif_yaw = (target_yaw - og_yaw + 180) % 360 - 180
        time_required = abs(dif_yaw / yaw_rate)
        n_steps = math.ceil(time_required * self.position_update_rate)
        step_size = dif_yaw/n_steps
        pos = np.asarray([x, y, z], dtype=float)
        for i in range(n_steps):
            if not self.is_paused:
                yaw = og_yaw + step_size*(i+1)
                await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=pos, yaw=yaw))
            await asyncio.sleep(1 / self.position_update_rate)
        while not self.is_at_heading(target_heading=target_yaw, tolerance=tolerance):
            await asyncio.sleep(1 / self.position_update_rate)
        return True

    async def spin_at_rate(self, yaw_rate, duration, direction="cw"):
        """ Spin in place at the given rate for the given duration.

        Pausable.

        :param yaw_rate:
        :param duration:
        :param direction:
        :return:
        """
        await super().spin_at_rate(yaw_rate, duration, direction=direction)
        if self.trajectory_follower.is_active:
            await self.trajectory_follower.deactivate()
        pos = self.position_ned
        og_yaw = self.attitude[2]
        freq = 10
        n_steps = math.ceil(duration * freq)
        step_size = yaw_rate / freq
        if direction == "ccw":
            step_size = - step_size
        for i in range(n_steps):
            if not self.is_paused:
                yaw = og_yaw + step_size*(i+1)
                await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=pos, yaw=yaw))
            await asyncio.sleep(1/freq)

    async def fly_to(self, x=None, y=None, z=None,
                     lat=None, long=None, amsl=None, yaw=None,
                     tolerance=0.25, put_into_offboard=True):
        """ Fly to a specified point in offboard mode. Uses trajectory generators and followers to get there.

        If both GPS and local coordinates are provided we use the GPS coordinates and local coordinates are ignored.

        :param x:
        :param y:
        :param z:
        :param lat:
        :param long:
        :param amsl:
        :param yaw:
        :param tolerance:
        :param put_into_offboard:
        :return:
        """
        # Check that we have one full set of coordinates and are in a flyable state
        if not self._can_do_in_air_commands():
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        assert ((x is not None and y is not None and z is not None)
                or (lat is not None, long is not None, amsl is not None)), \
            "Must provide a full set of either NED or GPS coordinates!"

        # Check that we have a trajectory generator and follower who are compatible with each other and the drone
        assert (self.trajectory_follower is not None
                and self.trajectory_follower.setpoint_type in self.VALID_SETPOINT_TYPES)
        assert (self.trajectory_generator is not None
                and self.trajectory_generator.waypoint_type in self.trajectory_follower.WAYPOINT_TYPES)

        # Use GPS if GPS coordinates are provided
        use_gps = lat is not None and long is not None and amsl is not None

        # Maintain current yaw if no yaw is provided
        if yaw is None:
            yaw = self.attitude[2]

        # Create setpoints for current position to allow drone to go into offboard mode
        if use_gps:
            self.logger.info(f"Flying to Lat: {lat} Long: {long} AMSL: {amsl} facing {yaw} with tolerance {tolerance}")
            cur_lat, cur_long, cur_amsl, cur_atl = self.position_global
            cur_yaw = self.attitude[2]
            await self.set_setpoint(Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray([cur_lat, cur_long, cur_amsl]),
                                             yaw=cur_yaw))
        else:
            self.logger.info(f"Flying to N: {x} E: {y} D: {z} facing {yaw} with tolerance {tolerance}")
            cur_pos = self.position_ned
            cur_yaw = self.attitude[2]
            await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=cur_pos, yaw=cur_yaw))

        if put_into_offboard and self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")

        # Determine target waypoint and send it to trajectory generator
        if use_gps:
            if not self.trajectory_generator.CAN_DO_GPS or not self.trajectory_follower.CAN_DO_GPS:
                raise RuntimeError("Trajectory generator can't use GPS coordinates!")
            self.trajectory_generator.use_gps = True
            target = np.asarray([lat, long, amsl])
            target_waypoint = Waypoint(WayPointType.POS_GLOBAL, gps=target, yaw=yaw)
        else:
            self.trajectory_generator.use_gps = False
            target = np.asarray([x, y, z])
            target_waypoint = Waypoint(WayPointType.POS_NED, pos=target, yaw=yaw)
        if self.check_waypoint(target_waypoint):
            self.trajectory_generator.set_target(target_waypoint)
        else:
            self.logger.warning("Can't fly to target position due to conflict with area fence!")
            return False

        # Create trajectory and activate follower algorithm if not already active
        self.logger.debug("Creating trajectory...")
        have_trajectory = await self.trajectory_generator.create_trajectory()
        if not have_trajectory:
            self.logger.warning("The trajectory generator couldn't generate a trajectory!")
            return False
        if not self.trajectory_follower.is_active:
            self.logger.debug("Starting follower algorithm...")
            self.trajectory_follower.activate()

        while True:
            # Check if we have arrived at target waypoint
            if use_gps:
                reached = (self.is_at_gps(target, tolerance=tolerance)
                           and self.is_at_heading(yaw, tolerance=1))
            else:
                reached = self.is_at_pos(target, tolerance=tolerance) and self.is_at_heading(yaw, tolerance=1)

            # Print message and stop if we have reached waypoint
            if reached:
                self.logger.info("Reached target position!")
                return True
            await asyncio.sleep(1 / self.position_update_rate)

    async def move(self, north, east, down, yaw, use_gps=True, tolerance=0.25):
        target_x = None
        target_y = None
        target_z = None
        target_lat = None
        target_long = None
        target_amsl = None
        if use_gps:
            cur_lat, cur_long, cur_alt, cur_ata = self.position_global
            target_lat, target_long, target_amsl = relative_gps(north, east, -down, cur_lat, cur_long, cur_alt)
        else:
            cur_x, cur_y, cur_z = self.position_ned
            target_x = cur_x + north
            target_y = cur_y + east
            target_z = cur_z + down
        target_yaw = self.attitude[2] + yaw
        return await self.fly_to(x=target_x, y=target_y, z=target_z,
                                 lat=target_lat, long=target_long, amsl=target_amsl,
                                 yaw=target_yaw, put_into_offboard=True, tolerance=tolerance)

    async def orbit(self, radius, velocity, center_lat, center_long, amsl):
        await super().orbit(radius, velocity, center_lat, center_long, amsl)
        if self.trajectory_follower.is_active:
            await self.trajectory_follower.deactivate()
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        yaw_behaviour = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER
        await self._error_wrapper(self.system.action.do_orbit, ActionError, radius, velocity, yaw_behaviour,
                                  center_lat, center_long, amsl)

    async def land(self):
        self.logger.info("Trying to land...")
        if self.trajectory_follower.is_active:
            await self.trajectory_follower.deactivate()
        await super().land()
        return await self._land_using_offbord_mode()

    async def _land_using_offbord_mode(self, error_thresh=0.00001, min_time=1):
        self.logger.info("Landing!")
        ema_alt_error = 0
        going_down = True
        old_alt = self.position_ned[2]
        start_time = time.time()
        target_pos = self._get_pos_ned_yaw()
        await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=target_pos[:3], yaw=target_pos[3]))
        if self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")
        update_freq = 2
        try:
            await self.system.telemetry.set_rate_position_velocity_ned(self.position_update_rate)
            update_freq = self.position_update_rate
        except Exception as e:
            self.logger.debug(f"Couldn't set message rate: {repr(e)}", exc_info=True)
        while going_down:
            cur_alt = self.position_ned[2]
            ema_alt_error = (cur_alt - old_alt) + 0.33 * ema_alt_error
            if ema_alt_error < error_thresh and time.time() > start_time + min_time:
                going_down = False
            old_alt = cur_alt
            target_pos[2] = cur_alt + 0.5
            await self.set_setpoint(Waypoint(WayPointType.POS_NED, pos=target_pos[:3], yaw=target_pos[3]))
            await asyncio.sleep(1/update_freq)
        self.logger.info("Landed!")
        return True

    async def _land_using_landmode(self):
        result = await self._error_wrapper(self.system.action.land, ActionError)
        if isinstance(result, Exception):
            self.logger.warning("Couldn't go into land mode")
        while self.flightmode is not FlightMode.LAND:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Landing!")
        while self.in_air:
            await asyncio.sleep(1 / self.position_update_rate)
        self.logger.info("Landed!")
        await self.change_flight_mode("hold")
        return True

    async def stop_execution(self):
        """ Stops all coroutines, closes all connections, etc.

        :return:
        """
        if self.trajectory_follower:
            if self.trajectory_follower.is_active:
                await self.trajectory_follower.deactivate()
            self.trajectory_follower.close()
        if self.mav_conn:
            await self.mav_conn.stop()
            del self.mav_conn
        self.system.__del__()
        if self._server_process:
            self._server_process.terminate()
        for handler in self.logging_handlers:
            self.logger.removeHandler(handler)
        await super().stop_execution()

    async def stop(self):
        # Override whatever else is going on and land
        self.clear_queue()
        self.cancel_action()
        if not self.is_connected:
            return True
        await self.land()
        await self.disarm()
        await super().stop()
        return True

    async def kill(self):
        await self._error_wrapper(self.system.action.kill, ActionError)
        await super().kill()
        return True

    async def _error_wrapper(self, func, error_type, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except error_type as e:
            self.logger.error(e._result.result_str)
            return False
        return True

# Camera Stuff #########################################################################################################

    async def prepare(self):
        await self._error_wrapper(self.system.camera.prepare, CameraError)

    async def take_picture(self, ir=True, vis=True):
        flags = 0
        if ir:
            flags += 1
        if vis:
            flags += 8
        await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=2000,
                                          param3=1.0,
                                          param5=int(flags),
                                          )

    async def start_video(self, ir=True, vis=True):
        flags = 0
        if ir:
            flags += 2
        if vis:
            flags += 4
        await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=2500,
                                          param1=int(flags),
                                          param2=2,
                                          )

    async def stop_video(self):
        await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=2501, )

    async def get_settings(self):
        await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=521, )
        await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=522,
                                          param1=1)

    async def set_zoom(self, zoom):
        await self.mav_conn.send_cmd_long(target_system=self.drone_system_id, target_component=100, cmd=203,
                                          param2=zoom,
                                          param5=0)


class Waypoint:
    def __init__(self, waypoint_type: WayPointType,
                 pos: np.ndarray | None = None,
                 vel: np.ndarray | None = None,
                 acc: np.ndarray | None = None,
                 gps: np.ndarray | None = None,
                 yaw: float | np.ndarray = 0.0):
        self._array = np.empty(
            (13,))  # Internal data structure, form [x, y, z, xvel, yvel, zvel, xacc, yacc, zacc, lat, long, amsl, yaw]
        self._array[:] = np.nan
        self._array[-1] = yaw
        self.type = waypoint_type
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

    def __str__(self):
        return f"{self.type}, {self.pos}, {self.vel}, {self.acc}, {self.gps}, {self.yaw}"

    @property
    def pos(self):
        return self._array[:3]

    @property
    def vel(self):
        return self._array[3:6]

    @property
    def acc(self):
        return self._array[6:9]

    @property
    def gps(self):
        return self._array[9:12]

    @property
    def yaw(self):
        return self._array[-1]

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
        """ Creates a vector between the initial and target waypoint and then creates a new Waypoint offset from this
        one by the same distance and heading."""
        new_gps = offset_from_gps(self.gps, initial.gps, target.gps)
        return Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray(new_gps), yaw=self.yaw)


##################################################################################################
# Trajectory Generators ##########################################################################
##################################################################################################

class TrajectoryGenerator(ABC):
    """ Abstract base class for trajectory generators."""

    CAN_DO_GPS = False
    WAYPOINT_TYPES = set()
    """ These determine the type of intermediate waypoints a trajectory generator may produce"""

    def __init__(self, drone: Drone, logger, waypoint_type, *args, use_gps=False, **kwargs):
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
        This function may be quite slow.
        This function should check that the trajectory stays within the fence defined by drone.fence"""

    @abstractmethod
    def next(self) -> Waypoint:
        """ The next waypoint, once the follower algorithm asks for another one.

        This function must execute quickly, as it might be called with a high frequency during flight. Trajectory
        followers should call it when they think they have reached the current waypoint. Should return None if the
        trajectory generator hasn't produced any waypoints or has run out."""


class StaticWaypoints(TrajectoryGenerator):
    """ Simply sends the target waypoint as static setpoints.
    """

    CAN_DO_GPS = True
    WAYPOINT_TYPES = {WayPointType.POS_NED, WayPointType.POS_GLOBAL}

    def __init__(self, drone, logger, waypoint_type, use_gps=False):
        super().__init__(drone, logger, waypoint_type, use_gps=use_gps)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory generator {self.__class__.__name__}:\n   {attr_string}")

    async def create_trajectory(self):
        if self.drone.check_waypoint(self.target_position):
            return True
        else:
            return False

    def next(self):
        """ Should return None if the generator isn't ready to produce waypoints yet."""
        return self.target_position


class GMP3Gen(TrajectoryGenerator):

    # TODO: GMP3 currently doesn't run with less than 2 obstacles
    # TODO: Altitude and yaw. Both are currently just set immediately when we get a new target

    WAYPOINT_TYPES = {WayPointType.POS_VEL_NED}
    CAN_DO_GPS = False

    def __init__(self, drone, dt, logger, use_gps=False, ):
        super().__init__(drone, logger, waypoint_type=WayPointType.POS_VEL_NED, use_gps=use_gps)
        self.GMP3_PARAMS = {
            "maxit": 100,
            "alpha": 0.8,
            "wdamp": 1,
            "delta": 0.01,
            "vx_max": 0.5,
            "vy_max": 0.5,
            "Q11": 0.7,
            "Q22": 0.7,
            "Q12": 0.01,
            "dt": dt,
            "obstacles": [
                (-3, -1, 0.5),
                (0, 1, 0.75),
                (3, 0, 2.0/3.0)
            ],
        }
        self.config = GMP3Config(**self.GMP3_PARAMS)
        self.gmp3 = GMP3(self.config)
        self.waypoints = None
        self.valid_path = False
        self.start_time = None
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory generator {self.__class__.__name__}:\n   {attr_string}")

    async def create_trajectory(self):
        try:
            self.logger.info("Calculating path...")
            cur_x, cur_y, _ = self.drone.position_ned
            target_x, target_y, _ = self.target_position.pos
            with ProcessPoolExecutor(max_workers=2) as executor:
                self.waypoints = await asyncio.get_running_loop().run_in_executor(executor, _calculate_path, cur_x,
                                                                                  cur_y, target_x, target_y, self.gmp3)
            valid = True
            for waypoint in self.waypoints:
                t, x, y, xdot, ydot = waypoint
                if not self.drone.check_waypoint(Waypoint(WayPointType.POS_VEL_NED,
                                                 pos=np.asarray([x, y, self.target_position.pos[2]]),
                                                 vel=np.asarray([xdot, ydot, 0]),
                                                 yaw=self.target_position.yaw)):
                    self.logger.debug(f"Generated waypoint {waypoint} is invalid")
                    valid = False
                    break
            if valid:
                self.logger.info("Found path!")
                self.logger.debug(f"Generated {len(self.waypoints)} waypoints: {self.waypoints}")
                self.start_time = time.time_ns()/1e9
                self.valid_path = True
                return True
            else:
                self.logger.warning(f"No valid path, generated trajectory violates waypoint constraints "
                                    f"(probably fence)!")
                self.valid_path = False
                return False
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.valid_path = False
            return False

    def next(self) -> Waypoint | None:
        if not self.valid_path:
            return None
        current_waypoint = None
        for wp in self.waypoints:
            if time.time_ns()/1e9 <= self.start_time + wp[0]:
                current_waypoint = wp
                break
        if current_waypoint is None:
            return None
        t, x, y, xdot, ydot = current_waypoint
        waypoint = Waypoint(WayPointType.POS_VEL_NED,
                            pos=np.asarray([x, y, self.target_position.pos[2]]),
                            vel=np.asarray([xdot, ydot, 0]),
                            yaw=self.target_position.yaw)
        return waypoint


def _calculate_path(cur_x, cur_y, target_x, target_y, gmp3):
    gmp3.calculate((cur_x, cur_y), (target_x, target_y))
    ts = gmp3.t
    xs = gmp3.x
    ys = gmp3.y
    xdots = gmp3.xdot
    ydots = gmp3.ydot
    waypoints = list(zip(ts, xs, ys, xdots, ydots))
    return waypoints


##################################################################################################
# Trajectory Followers ###########################################################################
##################################################################################################

class TrajectoryFollower(ABC):
    """ Abstract Base class to "follow" a given trajectory and maintain position at waypoints.

    A trajectory follower can work with different types of waypoints, but must be able to process WayPointType.POS_NED,
    as that is the default case, used if the trajectory generator fails to produce waypoints for some reason.
    """

    CAN_DO_GPS = False
    SETPOINT_TYPES = set()
    WAYPOINT_TYPES = set()

    def __init__(self, drone: Drone, logger, dt, setpoint_type: WayPointType):
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
                        if have_waypoints and not using_current_position:
                            self.logger.debug("Generator no longer producing waypoints, using current position")
                            # If we had waypoints, but lost them, use the current position as a dummy waypoint
                            have_waypoints = False
                            using_current_position = True
                        elif not have_waypoints and not using_current_position:  # Never had a waypoint
                            self.logger.debug("Don't have any waypoints from the generator yet, using current position")
                            using_current_position = True
                        else:
                            self.logger.debug("Still using current position...")
                        if using_current_position:
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
        position. This function should check that the trajectory stays within the fence defined by drone.fence.

        :return:
        """

    def close(self):
        if self._following_task:
            self._following_task.cancel()


class DirectSetpointFollower(TrajectoryFollower):

    CAN_DO_GPS = True

    SETPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_GLOBAL,
                      WayPointType.VEL_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED}

    WAYPOINT_TYPES = {WayPointType.POS_NED,
                      WayPointType.POS_GLOBAL,
                      WayPointType.VEL_NED,
                      WayPointType.POS_VEL_NED,
                      WayPointType.POS_VEL_ACC_NED}

    def __init__(self, drone: Drone, logger, dt, setpoint_type):
        super().__init__(drone, logger, dt, setpoint_type)
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory follower {self.__class__.__name__}:\n   {attr_string}")

    def get_next_waypoint(self) -> bool:
        return True

    async def set_setpoint(self, waypoint):
        if not self.drone.fence or (self.drone.fence and self.drone.fence.check_waypoint_compatible(waypoint)):
            await self.drone.set_setpoint(waypoint)


class VelocityControlFollower(TrajectoryFollower):
    """ Flies directly toward the waypoint facing towards it along the way. Turning towards the target yaw happens
    after we reach the waypoint. Control happens only through velocity setpoints.

    Currently very WIP, drifts off as soon as target positions are reached.
    """
    # TODO: Figure out better way to handle yaw rate

    SETPOINT_TYPES = {WayPointType.VEL_NED}
    WAYPOINT_TYPES = {WayPointType.POS_NED}
    CAN_DO_GPS = False

    def __init__(self, drone, logger, dt, max_vel_h=1.0, max_vel_z=0.5, max_acc_h=0.5, max_acc_z=0.25, max_yaw_rate=60):
        super().__init__(drone, logger, dt, setpoint_type=WayPointType.VEL_NED)
        self.max_vel_h = max_vel_h
        self.max_vel_z = max_vel_z
        self.max_acc_h = max_acc_h
        self.max_acc_z = max_acc_z
        self.max_yaw_rate = max_yaw_rate

        self.fudge_yaw = 1
        self.fudge_xy = 1
        self.fudge_z = 1

        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory follower {self.__class__.__name__}:\n   {attr_string}")

    def get_next_waypoint(self) -> bool:
        return (self.current_waypoint is None or
                self.drone.is_at_pos(self.current_waypoint.pos)
                and self.drone.is_at_heading(self.current_waypoint.yaw))

    async def set_setpoint(self, waypoint):
        """ Always move towards target. Accelerates if we are slower than the max speed and have space to accelerate,
        keep speed if we are at max velocity and still some distance away from target, decelerate when we approach
        target.

        :return:
        """
        if not (self.drone.fence and self.drone.fence.check_waypoint_compatible(waypoint)):
            return False
        # Yaw
        target_yaw = waypoint.yaw
        if self.drone.is_at_pos(waypoint.pos, tolerance=1):
            temp_yaw_target = target_yaw
        else:
            temp_yaw_target = heading_ned(self.drone.position_ned, waypoint.pos)
        cur_yaw = self.drone.attitude[2]
        dif_yaw = (temp_yaw_target - cur_yaw + 180) % 360 - 180
        step_size = self.max_yaw_rate * self.dt * self.fudge_yaw
        if abs(dif_yaw) < step_size:
            yaw = temp_yaw_target
        else:
            if dif_yaw > 0:
                yaw = cur_yaw + step_size
            else:
                yaw = cur_yaw - step_size

        # Vertical movement
        cur_z = self.drone.position_ned[2]
        cur_speed_z = abs(self.drone.velocity[2])
        dist_z = abs(waypoint.pos[2] - cur_z)
        speed_z_lim = min(math.sqrt(abs(2 * self.max_acc_z * dist_z)), self.max_vel_z)
        speed_z = min(cur_speed_z + self.max_acc_z * self.dt * self.fudge_z, speed_z_lim)
        vel_z = speed_z if waypoint.pos[2] - cur_z > 0 else -speed_z  # Speed is not velocity -> manually set sign

        # Horizontal
        cur_xy = self.drone.position_ned[:2]
        dist_xy = dist_ned(waypoint.pos[:2], cur_xy)
        cur_vel_xy = self.drone.velocity[:2]
        cur_speed_xy = np.sqrt(cur_vel_xy.dot(cur_vel_xy))
        dist_xy_v = waypoint.pos[:2] - cur_xy
        speed_xy_limit = min(math.sqrt(abs(2 * self.max_acc_h * dist_xy)), self.max_vel_h)
        speed_xy = min(cur_speed_xy + self.max_acc_h * self.dt * self.fudge_xy, speed_xy_limit)
        dir_xy = math.atan2(dist_xy_v[1], dist_xy_v[0])
        vel_x = math.cos(dir_xy) * speed_xy
        vel_y = math.sin(dir_xy) * speed_xy

        vel_yaw_setpoint = Waypoint(WayPointType.VEL_NED, vel=np.asarray([vel_x, vel_y, vel_z]), yaw=yaw)
        await self.drone.set_setpoint(vel_yaw_setpoint)


##################################################################################################
# Fences #########################################################################################
##################################################################################################

class Fence(ABC):
    """ Abstract base class for geo-fence type classes and methods.

    """
    def __init__(self, *args, **kwargs):
        self.active = True

    @abstractmethod
    def check_waypoint_compatible(self, point: Waypoint) -> bool:
        """ Should return True if the waypoint fits within the fence, and false otherwise.

        "Fits within" is taken broadly here, a fence can be inclusive, exclusive, around a dynamic obstacle, or
         anything else. As long as True is returned when the waypoint is "good" and False otherwise, it works."""
        pass


class RectLocalFence(Fence):
    """ Class for rectangular fences in the local coordinate frame.

    Works by defining five limits: north upper and lower, east upper and lower, height. Waypoints will only be accepted
    if they use local (NED) coordinates and lie within the box between these five limits.
    The north lower limit should be lower than north upper, and the same for east.
    Note that height should be positive.
    """
    def __init__(self, north_lower, north_upper, east_lower, east_upper, height):
        super().__init__()
        assert north_lower < north_upper and east_lower < east_upper, \
            "Lower fence limits must be less than the upper ones!"
        self.north_lower = north_lower
        self.north_upper = north_upper
        self.east_lower = east_lower
        self.east_upper = east_upper
        self.height = height

    def check_waypoint_compatible(self, point: Waypoint):
        if self.active and point.type in [WayPointType.POS_NED, WayPointType.POS_VEL_NED, WayPointType.POS_VEL_ACC_NED]:
            coord_north, coord_east, coord_down = point.pos
            if (self.north_lower < coord_north < self.north_upper
                    and self.east_lower < coord_east < self.east_upper
                    and -coord_down < self.height):
                return True
        return False

    def __str__(self):
        return (f"{self.__class__.__name__}, with limits {self.north_lower, self.north_upper}, "
                f"{self.east_lower, self.east_upper} and {self.height}")
