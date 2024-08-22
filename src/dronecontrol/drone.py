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

from dronecontrol.utils import dist_ned, dist_gps, relative_gps, parse_address, common_formatter, get_free_port
from dronecontrol.mavpassthrough import MAVPassthrough
from dronecontrol.gimbal import Gimbal

import logging

_cur_dir = os.path.dirname(os.path.abspath(__file__))
logdir = os.path.abspath("./logs")
os.makedirs(logdir, exist_ok=True)
_mav_server_file = os.path.join(_cur_dir, "mavsdk_server_bin.exe")

# TODO: Have a look at the entire connection procedure, make some diagrams, plan everything out and refactor
# TODO: health info


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


class SetPointType(Enum):
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
        self.trajectory_generator: TrajectoryGenerator | None = None

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
        """ Resume the current task. """
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
    async def takeoff(self, altitude=-2.0) -> bool:
        """ Takes off to the specified altitude.

        Note that negative for up, i.e. the altitude should be -2 to take off to 2m above ground.

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

    def is_at_gps(self, target_lat, target_long, target_amsl, tolerance=0.25) -> bool:
        cur_lat, cur_long, cur_asml, cur_atl = self.position_global
        if dist_gps(target_lat, target_long, target_amsl, cur_lat, cur_long, cur_asml) < tolerance:
            return True
        return False

    @abstractmethod
    async def yaw_to(self, x, y, z, target_yaw, yaw_rate, tolerance) -> bool:
        pass

    @abstractmethod
    async def spin_at_rate(self, yaw_rate, duration, direction="cw") -> bool:
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
    VALID_SETPOINT_TYPES = {SetPointType.POS_NED,
                            SetPointType.POS_VEL_NED,
                            SetPointType.POS_VEL_ACC_NED,
                            SetPointType.VEL_NED,
                            SetPointType.POS_GLOBAL}
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
        self._position_g: np.ndarray = np.zeros((4,), dtype=np.double)  # Latitude, Longitude, AMSL, Relative altitude to takeoff
        self._position_ned: np.ndarray = np.zeros((3,))     # NED
        self._velocity: np.ndarray = np.zeros((3,))         # NED
        self._attitude: np.ndarray = np.zeros((3,))         # Roll, pitch and yaw, with positives right up and right.
        self._heading: float = math.nan
        self._batteries: dict[int, Battery] = {}
        self._running_tasks = []
        self.position_update_rate = 5                     # How often (per second) go-to-position commands compute if they have arrived.

        self._max_position_discontinuity = - math.inf

        self.mav_conn = MAVPassthrough(loggername=f"{name}_MAVLINK", log_messages=True)
        self.trajectory_generator = StaticWaypoints(self, 1 / self.position_update_rate, self.logger)

        self.gimbal = None

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
                await asyncio.sleep(0.5)  # Wait to try and make sure that the mavsdk server has started before booting up passthrough
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
                    self.gimbal = Gimbal(self.logger, self)
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
            shift = dist_ned(self._position_ned, new_pos)
            if shift > self._max_position_discontinuity:
                self._max_position_discontinuity = shift
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

    async def takeoff(self, altitude=-2.0) -> bool:
        """

        Note that altitude is negative for  up.

        :param altitude:
        :return:
        """
        return await self._takeoff_using_offboard(altitude=altitude)

    def _get_pos_ned_yaw(self):
        pos_yaw = np.zeros((4,))
        pos_yaw[:3] = self.position_ned
        pos_yaw[3] = self.attitude[2]
        return pos_yaw

    async def _takeoff_using_takeoffmode(self, altitude=-2.5):
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

    async def _takeoff_using_offboard(self, altitude=-2.5, tolerance=0.25):
        self.logger.info(f"Trying to take off to {-altitude}m in offboard mode...")
        await super().takeoff(altitude=altitude)
        self._can_takeoff()
        target_pos_yaw = self._get_pos_ned_yaw()
        target_pos_yaw[2] = altitude
        await self.set_setpoint_pos_ned(target_pos_yaw)
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
        result = False
        target_flight_mode = None
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
                self.logger.warning("Drone accepted command, but flight mode change timed out! Possible connection issue.")
                return False
            else:
                self.logger.info(f"New flight mode {self.flightmode}!")
                return True

    async def set_setpoint_pos_ned(self, setpoint):
        # point should be a numpy array of size (4, ) for north, east, down, yaw, with yaw in degrees.
        point_ned_yaw = PositionNedYaw(*setpoint)
        return await self._error_wrapper(self.system.offboard.set_position_ned, OffboardError, point_ned_yaw)

    async def set_setpoint_pos_vel_ned(self, setpoint):
        point_ned_yaw = PositionNedYaw(*setpoint[:3], setpoint[-1])
        velocity_ned_yaw = VelocityNedYaw(*setpoint[3:])
        return await self._error_wrapper(self.system.offboard.set_position_velocity_ned, OffboardError,
                                         point_ned_yaw, velocity_ned_yaw)

    async def set_setpoint_pos_vel_acc_ned(self, setpoint):
        yaw = setpoint[-1]
        point_ned_yaw = PositionNedYaw(*setpoint[:3], yaw)
        velocity_ned_yaw = VelocityNedYaw(*setpoint[3:6], yaw)
        acc_ned = AccelerationNed(*setpoint[6:9])
        return await self._error_wrapper(self.system.offboard.set_position_velocity_acceleration_ned,
                                         OffboardError,
                                         point_ned_yaw,
                                         velocity_ned_yaw,
                                         acc_ned)

    async def set_setpoint_vel_ned(self, setpoint):
        vel_yaw = VelocityNedYaw(*setpoint)
        return await self._error_wrapper(self.system.offboard.set_velocity_ned, OffboardError, vel_yaw)

    async def set_setpoint_gps(self, setpoint):
        latitude, longitude, amsl, yaw = setpoint
        alt_type = PositionGlobalYaw.AltitudeType.AMSL
        position = PositionGlobalYaw(lat_deg=latitude, lon_deg=longitude, alt_m=amsl,
                                     yaw_deg=yaw, altitude_type=alt_type)
        return await self._error_wrapper(self.system.offboard.set_position_global, OffboardError, position)

    def _can_do_in_air_commands(self):
        # TODO: Figure out how to do this with ardupilot. Currently the in_air detection is very poor
        if self.autopilot == "ardupilot":
            return True
        if not self.is_armed or not self.in_air:
            return False

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
        og_yaw = self.attitude[2]
        dif_yaw = (target_yaw - og_yaw + 180) % 360 - 180
        time_required = abs(dif_yaw / yaw_rate)
        n_steps = math.ceil(time_required * self.position_update_rate)
        step_size = dif_yaw/n_steps
        pos_yaw = np.asarray([x, y, z, 0], dtype=float)
        for i in range(n_steps):
            if not self.is_paused:
                pos_yaw[3] = og_yaw + step_size*(i+1)
                await self.set_setpoint_pos_ned(pos_yaw)
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
        x, y, z = self.position_ned
        pos_yaw = np.asarray([x, y, z, 0], dtype=float)
        freq = 10
        n_steps = math.ceil(duration * freq)
        step_size = yaw_rate / freq
        if direction == "ccw":
            step_size = - step_size
        for i in range(n_steps):
            if not self.is_paused:
                pos_yaw[3] = step_size*(i+1)
                await self.set_setpoint_pos_ned(pos_yaw)
            await asyncio.sleep(1/freq)

    async def fly_to(self, x=None, y=None, z=None,
                     lat=None, long=None, amsl=None, yaw=None,
                     tolerance=0.25, put_into_offboard=True):
        """ Fly to a specified point in offboard mode.

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

        # Check that we have a trajectory generator whose output we can process
        assert self.trajectory_generator is not None and self.trajectory_generator.setpoint_type in self.VALID_SETPOINT_TYPES

        # Use GPS if GPS coordinates are provided
        use_gps = lat is not None and long is not None and amsl is not None

        # Maintain current yaw if no yaw is provided
        if yaw is None:
            yaw = self.attitude[2]

        if use_gps:
            self.logger.info(f"Flying to Lat: {lat} Long: {long} AMSL: {amsl} facing {yaw} with tolerance {tolerance}")
            cur_lat, cur_long, cur_amsl, cur_atl = self.position_global
            cur_yaw = self.attitude[2]
            await self.set_setpoint_gps([cur_lat, cur_long, cur_amsl, cur_yaw])
        else:
            self.logger.info(f"Flying to N: {x} E: {y} D: {z} facing {yaw} with tolerance {tolerance}")
            cur_pos_yaw = np.zeros((4,))
            cur_pos_yaw[:3] = self.position_ned
            cur_pos_yaw[3] = self.attitude[2]
            await self.set_setpoint_pos_ned(cur_pos_yaw)

        if put_into_offboard and self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")

        if use_gps:
            if not self.trajectory_generator.CAN_DO_GPS:
                raise RuntimeError("Trajectory generator can't use GPS coordinates!")
            self.trajectory_generator.use_gps = True
            self.trajectory_generator.set_target(np.asarray([lat, long, amsl, yaw]))
        else:
            self.trajectory_generator.use_gps = False
            self.trajectory_generator.set_target(np.asarray([x, y, z, yaw]))

        while True:
            if not self.is_paused:
                # Get and set a new setpoint to go to
                setpoint = self.trajectory_generator.next_setpoint()
                # Set appropriate setpoint
                match self.trajectory_generator.setpoint_type:
                    case SetPointType.POS_NED:
                        await self.set_setpoint_pos_ned(setpoint)
                    case SetPointType.POS_VEL_NED:
                        await self.set_setpoint_pos_vel_ned(setpoint)
                    case SetPointType.POS_VEL_ACC_NED:
                        await self.set_setpoint_pos_vel_acc_ned(setpoint)
                    case SetPointType.VEL_NED:
                        await self.set_setpoint_vel_ned(setpoint)
                    case SetPointType.POS_GLOBAL:
                        await self.set_setpoint_gps(setpoint)
                    case _:
                        raise RuntimeError(f"Invalid generator setpoint type {self.trajectory_generator.setpoint_type}! Must "
                                           f"be one of {self.VALID_SETPOINT_TYPES}")

                # Check if we have arrived at target waypoint
                if use_gps:
                    reached = (self.is_at_gps(lat, long, amsl, tolerance=tolerance)
                               and self.is_at_heading(yaw, tolerance=1))
                else:
                    reached = self.is_at_pos([x, y, z], tolerance=tolerance) and self.is_at_heading(yaw, tolerance=1)

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
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        yaw_behaviour = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER
        await self._error_wrapper(self.system.action.do_orbit, ActionError, radius, velocity, yaw_behaviour,
                                  center_lat, center_long, amsl)

    async def land(self):
        self.logger.info("Trying to land...")
        await super().land()
        return await self._land_using_offbord_mode()

    async def _land_using_offbord_mode(self, error_thresh=0.00001, min_time=1):
        self.logger.info("Landing!")
        ema_alt_error = 0
        going_down = True
        old_alt = self.position_ned[2]
        start_time = time.time()
        target_pos = self._get_pos_ned_yaw()
        await self.set_setpoint_pos_ned(target_pos)
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
                break
            old_alt = cur_alt
            target_pos[2] = cur_alt + 0.5
            await self.set_setpoint_pos_ned(target_pos)
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

# Gimbal Stuff #########################################################################################################

    def log_status(self):
        self.gimbal.log_status()

    async def take_control(self):
        await self.gimbal.take_control()

    async def release_control(self):
        await self.gimbal.release_control()

    async def set_gimbal_angles(self, roll, pitch, yaw):
        await self.gimbal.set_gimbal_angles(roll, pitch, yaw)

    async def point_gimbal_at(self, lat, long, amsl):
        await self.gimbal.point_gimbal_at(lat, long, amsl)

    async def point_gimbal_at_relative(self, x, y, z):
        await self.gimbal.point_gimbal_at_relative(x, y, z)

    async def set_gimbal_mode(self, mode):
        await self.gimbal.set_gimbal_mode(mode)

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
        #await self._error_wrapper(self.system.camera.take_photo, CameraError)

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


##################################################################################################
# Trajectory Generators ##########################################################################
##################################################################################################

class TrajectoryGenerator(ABC):

    CAN_DO_GPS = False
    SETPOINT_TYPES = set()

    def __init__(self, drone: Drone, dt, logger, use_gps=False, setpointtype: SetPointType = None):
        """

        Should be called at the end of subclass constructors.

        :param drone:
        :param dt:
        :param logger:
        :param use_gps:
        :param setpointtype:
        """
        assert setpointtype in self.SETPOINT_TYPES, (f"Invalid setpoint type {setpointtype} "
                                                     f"for trajectory generator {self.__class__.__name__}")
        self.drone = drone
        self.dt = dt
        self.logger = logger
        self.use_gps = use_gps
        self.setpoint_type = setpointtype
        self.target_position: np.ndarray | None = None

    @property
    @abstractmethod
    def is_ready(self) -> bool:
        """ Indicates if the trajectory generator is ready to produce setpoints."""
        pass

    def set_target(self, point: np.ndarray):
        """ Sets the target position that we will try to fly towards.

        :param point: Target waypoint. Should be an array with shape (4,), where the first three entries are the
            position (FRD, NED or LAT/LONG/AMSL) and the last is the target yaw
        :return:
        """
        self.target_position = point

    @abstractmethod
    def next_setpoint(self) -> np.ndarray:
        pass


class StaticWaypoints(TrajectoryGenerator):
    """ Simply sends the target waypoint as static setpoints.
    """

    CAN_DO_GPS = True
    SETPOINT_TYPES = {SetPointType.POS_NED, SetPointType.POS_GLOBAL}

    def __init__(self, drone, dt, logger, use_gps=False, setpointtype=SetPointType.POS_NED):
        super().__init__(drone, dt, logger=logger, use_gps=use_gps, setpointtype=setpointtype)
        self._is_ready = True
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory generator {self.__class__.__name__}:\n   {attr_string}")

    @property
    def is_ready(self) -> bool:
        return self._is_ready

    def next_setpoint(self) -> np.ndarray:
        if self.use_gps:
            self.setpoint_type = SetPointType.POS_GLOBAL
        else:
            self.setpoint_type = SetPointType.POS_NED
        return self.target_position


class DirectFlightFacingForward(TrajectoryGenerator):
    """ Flies directly toward the waypoint facing towards it along the way. Turning towards the target yaw happens
    after we reach the waypoint. Control happens only through velocity setpoints.

    Currently very WIP, drifts off as soon as target positions are reached.
    """
    # TODO: Figure out better way to handle yaw rate

    SETPOINT_TYPES = {SetPointType.VEL_NED}
    CAN_DO_GPS = False

    def __init__(self, drone, dt, logger, use_gps=False, setpointtype=SetPointType.VEL_NED,
                 max_vel_h=1.0, max_vel_z=0.5, max_acc_h=0.5, max_acc_z=0.25, max_yaw_rate=20, ):
        super().__init__(drone, dt, logger=logger, use_gps=use_gps, setpointtype=setpointtype)
        self.max_vel_h = max_vel_h
        self.max_vel_z = max_vel_z
        self.max_acc_h = max_acc_h
        self.max_acc_z = max_acc_z
        self.max_yaw_rate = max_yaw_rate

        self.fudge_yaw = 1
        self.fudge_xy = 1
        self.fudge_z = 1

        self._is_ready = True
        attr_string = "\n   ".join(["{}: {}".format(key, value) for key, value in self.__dict__.items()])
        self.logger.debug(f"Initialized trajectory generator {self.__class__.__name__}:\n   {attr_string}")

    @property
    def is_ready(self) -> bool:
        return self._is_ready

    def next_setpoint(self):
        """ Always move towards target. Accelerates if we are slower than the max speed and have space to accelerate,
        keep speed if we are at max velocity and still some distance away from target, decelerate when we approach
        target.

        :return:
        """
        # Yaw
        target_yaw = self.target_position[3]
        if self.drone.is_at_pos(self.target_position[:3], tolerance=1):
            temp_yaw_target = target_yaw
        else:
            temp_yaw_target = math.atan2(self.target_position[1] - self.drone.position_ned[1],
                                         self.target_position[0] - self.drone.position_ned[0]) / math.pi * 180
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
        dist_z = abs(self.target_position[2] - cur_z)
        speed_z_lim = min(math.sqrt(abs(2 * self.max_acc_z * dist_z)), self.max_vel_z)
        speed_z = min(cur_speed_z + self.max_acc_z * self.dt * self.fudge_z, speed_z_lim)
        vel_z = speed_z if self.target_position[2] - cur_z > 0 else -speed_z

        # Horizontal
        cur_xy = self.drone.position_ned[:2]
        dist_xy = dist_ned(self.target_position[:2], cur_xy)
        cur_vel_xy = self.drone.velocity[:2]
        cur_speed_xy = np.sqrt(cur_vel_xy.dot(cur_vel_xy))
        dist_xy_v = self.target_position[:2] - cur_xy
        speed_xy_limit = min(math.sqrt(abs(2 * self.max_acc_h * dist_xy)), self.max_vel_h)
        speed_xy = min(cur_speed_xy + self.max_acc_h * self.dt * self.fudge_xy, speed_xy_limit)
        dir_xy = math.atan2(dist_xy_v[1], dist_xy_v[0])
        vel_x = math.cos(dir_xy) * speed_xy
        vel_y = math.sin(dir_xy) * speed_xy

        vel_yaw_setpoint = np.asarray([vel_x, vel_y, vel_z, yaw])
        return vel_yaw_setpoint
