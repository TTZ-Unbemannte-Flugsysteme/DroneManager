import asyncio
import datetime
from collections import deque
import math
import os.path
from enum import Enum, auto
import threading
import haversine
import platform
import time
from subprocess import Popen, DEVNULL
from abc import ABC, abstractmethod

import numpy as np
from urllib.parse import urlparse, urlunparse

from typing import Dict, Deque, Tuple

from mavsdk import System
from mavsdk.telemetry import FlightMode, FixType, StatusTextType
from mavsdk.action import ActionError, OrbitYawBehavior
from mavsdk.offboard import PositionNedYaw, PositionGlobalYaw, VelocityNedYaw, AccelerationNed, OffboardError
from mavsdk.manual_control import ManualControlError

from mavpassthrough import MAVPassthrough

import logging

_cur_dir = os.path.dirname(os.path.abspath(__file__))
logdir = os.path.abspath("./logs")
os.makedirs(logdir, exist_ok=True)
_mav_server_file = os.path.join(_cur_dir, "mavsdk_server_bin.exe")

common_formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")

# TODO: Fix yaw_to so it moves the correct direction
# TODO: change_flight_mode scheduling, more flightmodes
# TODO: health info
# TODO: Fix the dummy drone so it actually works again


def dist_ned(pos1, pos2):
    return np.sqrt(np.sum((pos1 - pos2) ** 2, axis=0))


def dist_gps(lat1, long1, alt1, lat2, long2, alt2):
    dist_horiz = haversine.haversine((lat1, long1), (lat2, long2), unit=haversine.Unit.METERS)
    dist_alt = alt1 - alt2
    return math.sqrt(dist_horiz*dist_horiz + dist_alt*dist_alt)


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
        self.action_queue: Deque[Tuple[asyncio.Coroutine, asyncio.Future]] = deque()
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
        self.is_paused = False
        self.start()
        asyncio.create_task(self._task_scheduler())

    def run(self):
        while not self.should_stop:
            pass

    def __del__(self):
        self.stop_execution()

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

    async def stop_execution(self):
        """ Stops all tasks, closes all connections, etc.

        :return:
        """
        pass

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
    def batteries(self) -> Dict[int, Battery]:
        pass

    @abstractmethod
    async def connect(self, drone_addr):
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
        pass

    @abstractmethod
    async def orbit(self, radius, velocity, latitude, longitude, amsl) -> bool:
        pass

    @abstractmethod
    async def land(self) -> bool:
        pass

    @abstractmethod
    async def stop(self) -> bool:
        """
        This function should be called at the end of the implementing function.

        :return:
        """
        for handler in self.logging_handlers:
            self.logger.removeHandler(handler)
        return True

    @abstractmethod
    async def kill(self) -> bool:
        """
        This function should be called at the end of the implementing function.

        :return:
        """
        for handler in self.logging_handlers:
            self.logger.removeHandler(handler)
        return True

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


def parse_address(string=None, scheme=None, host=None, port=None, return_string=False):
    """ Used to ensure that udp://:14540, udp://localhost:14540 and udp://127.0.0.1:14540 are recognized as equivalent.

    Missing elements from the string or the other entries are replaced with defaults. These are udp, empty host and
    50051 for the scheme, host and port, respectively.
    """
    if string is None and scheme is None and host is None and port is None:
        raise RuntimeError("Cant parse an address without either a url or a host/port pair!")
    if string:
        parse_drone_addr = urlparse(string)
        scheme = parse_drone_addr.scheme
        host = parse_drone_addr.hostname
        port = parse_drone_addr.port
    if scheme is None:
        scheme = "udp"
    if host is None:
        host = ""
    elif host == "localhost":
        host = ""
    elif host == "127.0.0.1":
        host = ""
    if port is None:
        port = 50051
    if return_string:
        return urlunparse((scheme, ":".join([host, str(port)]), "", "", "", ""))
    else:
        return scheme, host, port


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

    def __init__(self, name, mavsdk_server_address: str | None = None, mavsdk_server_port: int = 50051, compid=190):
        Drone.__init__(self, name)
        self.compid = compid
        self.system: System | None = None
        self.server_addr = mavsdk_server_address
        self.server_port = mavsdk_server_port
        self._server_process: Popen | None = None
        self._is_connected: bool = False
        self._is_armed: bool = False
        self._flightmode: FlightMode = FlightMode.UNKNOWN
        self._in_air: bool = False
        self._gps_info: FixType | None = None
        self._position_g: np.ndarray = np.zeros((4,), dtype=np.double)  # Latitude, Longitude, AMSL, Relative altitude
        self._position_ned: np.ndarray = np.zeros((3,))     # NED
        self._velocity: np.ndarray = np.zeros((3,))         # NED
        self._attitude: np.ndarray = np.zeros((3,))         # Roll, pitch and yaw, with positives right up and right.
        self._heading: float = math.nan
        self._batteries: Dict[int, Battery] = {}
        self._running_tasks = []
        self._position_update_freq = 10                     # How often (per second) go-to-position commands compute if they have arrived.

        self._max_position_discontinuity = - math.inf
        self._passthrough = MAVPassthrough(loggername=f"{name}_MAVLINK", log_messages=False)
        self.trajectory_gen = StaticWaypoints(self, 1/self._position_update_freq)

    def __del__(self):
        self.system.__del__()
        super().__del__()

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
    def batteries(self) -> Dict[int, Battery]:
        return self._batteries

    async def connect(self, drone_address) -> bool:
        # If we are on windows, we can't rely on the MAVSDK to have the binary installed.
        scheme, ip, port = parse_address(string=drone_address, return_string=False)
        assert scheme == "udp"
        self.drone_addr = f"{scheme}://{ip}:{port}"
        self.logger.debug(f"Connecting to drone {self.name} @ {self.drone_addr}")
        if self._passthrough:
            mavsdk_passthrough_port = port + 1000
        else:
            mavsdk_passthrough_port = port
        mavsdk_passthrough_string = f"{scheme}://:{mavsdk_passthrough_port}"
        passthrough_gcs_string = f"127.0.0.1:{mavsdk_passthrough_port}"

        try:
            if self.server_addr is None:
                self.logger.debug(f"Starting up own MAVSDK Server instance with app port {self.server_port} and remote "
                                  f"connection {mavsdk_passthrough_string}")
            if self.server_addr is None and platform.system() == "Windows":
                self._server_process = Popen(f".\\mavsdk_server_bin.exe -p {self.server_port} {mavsdk_passthrough_string}",
                                             stdout=DEVNULL, stderr=DEVNULL)
                self.server_addr = "127.0.0.1"
            self.system = System(mavsdk_server_address=self.server_addr, port=self.server_port, compid=self.compid)

            _, self.server_addr, self.server_port = parse_address(scheme="udp",
                                                                  host=self.server_addr,
                                                                  port=self.server_port,
                                                                  return_string=False)
            connected = asyncio.create_task(self.system.connect(system_address=mavsdk_passthrough_string))

            # Create passthrough
            if self._passthrough:
                await asyncio.sleep(0.5)  # Wait to try and make sure that the mavsdk server has started before booting up passthrough
                self.logger.debug(
                    f"Connecting passthrough to drone @{ip}:{port} and MAVSDK server @ {passthrough_gcs_string}")
                self._passthrough.connect_drone(ip, port)
                self._passthrough.connect_gcs(passthrough_gcs_string)

                while not self._passthrough.connected_to_drone() or not self._passthrough.connected_to_gcs():
                    self.logger.debug(f"Waiting on passthrough to connect. "
                                      f"Drone: {self._passthrough.connected_to_drone()}, "
                                      f"GCS: {self._passthrough.connected_to_gcs()}")
                    await asyncio.sleep(0.1)
                self.logger.debug("Connected passthrough!")

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
            await self.system.telemetry.set_rate_position(self._position_update_freq)
            await self.system.telemetry.set_rate_position_velocity_ned(self._position_update_freq)
            await self.system.telemetry.set_rate_attitude_euler(self._position_update_freq)
        except Exception as e:
            self.logger.warning(f"Couldn't set message rate!")
            self.logger.debug(f"{repr(e)}", exc_info=True)

    async def _connect_check(self):
        if self._passthrough:
            while True:
                self._is_connected = self._passthrough.connected_to_drone() and self._passthrough.connected_to_gcs()
                await asyncio.sleep(1/self._position_update_freq)
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
        result = await self._action_error_wrapper(self.system.action.arm)
        if result and not isinstance(result, Exception):
            start_time = time.time()
            while not self.is_armed:
                await asyncio.sleep(1 / self._position_update_freq)
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
        result = await self._action_error_wrapper(self.system.action.disarm)
        if result and not isinstance(result, Exception):
            start_time = time.time()
            while self.is_armed:
                await asyncio.sleep(1 / self._position_update_freq)
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
        # TODO: Change takeoff altitude by changing PX4 param before going into takeoff mode
        self.logger.info("Trying to take off...")
        await super().takeoff(altitude=altitude)
        self._can_takeoff()
        result = await self._action_error_wrapper(self.system.action.takeoff)
        if isinstance(result, Exception):
            self.logger.warning("Couldn't takeoff!")
        while self.flightmode is not FlightMode.TAKEOFF:
            await asyncio.sleep(1/self._position_update_freq)
        self.logger.info("Taking off!")
        while self.flightmode is FlightMode.TAKEOFF:
            await asyncio.sleep(1/self._position_update_freq)
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
            await asyncio.sleep(1 / self._position_update_freq)

    async def change_flight_mode(self, flightmode: str):
        self.logger.info(f"Changing flight mode to {flightmode}")
        await super().change_flight_mode(flightmode)
        result = False
        if flightmode == "hold":
            result = await self._action_error_wrapper(self.system.action.hold)
        elif flightmode == "offboard":
            result = await self._offboard_error_wrapper(self.system.offboard.start)
        elif flightmode == "return":
            result = await self._action_error_wrapper(self.system.action.return_to_launch)
        elif flightmode == "land":
            result = await self._action_error_wrapper(self.system.action.land)
        elif flightmode == "takeoff":
            await self._can_takeoff()
            result = await self._action_error_wrapper(self.system.action.takeoff)
        elif flightmode == "position":
            result = await self.__manual_control_error_wrapper(self.system.manual_control.start_position_control())
        elif flightmode == "altitude":
            result = await self.__manual_control_error_wrapper(self.system.manual_control.start_altitude_control())
        else:
            raise KeyError(f"{flightmode} is not a valid flightmode!")
        if result and not isinstance(result, Exception):
            self.logger.info(f"New flight mode {flightmode}!")
        else:
            self.logger.warning("Couldn't change flight mode!")
        return result

    async def set_setpoint_pos_ned(self, setpoint):
        # point should be a numpy array of size (4, ) for north, east, down, yaw, with yaw in degrees.
        point_ned_yaw = PositionNedYaw(*setpoint)
        return await self._offboard_error_wrapper(self.system.offboard.set_position_ned, point_ned_yaw)

    async def set_setpoint_pos_vel_ned(self, setpoint):
        point_ned_yaw = PositionNedYaw(*setpoint[:3], setpoint[-1])
        velocity_ned_yaw = VelocityNedYaw(*setpoint[3:])
        return await self._offboard_error_wrapper(self.system.offboard.set_position_velocity_ned,
                                                  point_ned_yaw, velocity_ned_yaw)

    async def set_setpoint_pos_vel_acc_ned(self, setpoint):
        yaw = setpoint[-1]
        point_ned_yaw = PositionNedYaw(*setpoint[:3], yaw)
        velocity_ned_yaw = VelocityNedYaw(*setpoint[3:6], yaw)
        acc_ned = AccelerationNed(*setpoint[6:9])
        return await self._offboard_error_wrapper(self.system.offboard.set_position_velocity_acceleration_ned,
                                                  point_ned_yaw,
                                                  velocity_ned_yaw,
                                                  acc_ned)

    async def set_setpoint_vel_ned(self, setpoint):
        vel_yaw = VelocityNedYaw(*setpoint)
        return await self._offboard_error_wrapper(self.system.offboard.set_velocity_ned, vel_yaw)

    async def set_setpoint_gps(self, setpoint):
        latitude, longitude, amsl, yaw = setpoint
        alt_type = PositionGlobalYaw.AltitudeType.AMSL
        position = PositionGlobalYaw(lat_deg=latitude, lon_deg=longitude, alt_m=amsl,
                                     yaw_deg=yaw, altitude_type=alt_type)
        return await self._offboard_error_wrapper(self.system.offboard.set_position_global, position)

    def _can_do_in_air_commands(self):
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")

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
        n_steps = math.ceil(time_required*self._position_update_freq)
        step_size = dif_yaw/n_steps
        pos_yaw = np.asarray([x, y, z, 0], dtype=float)
        for i in range(n_steps):
            if not self.is_paused:
                pos_yaw[3] = og_yaw + step_size*(i+1)
                await self.set_setpoint_pos_ned(pos_yaw)
            await asyncio.sleep(1/self._position_update_freq)
        return self.is_at_heading(target_heading=target_yaw, tolerance=tolerance)

    async def spin_at_rate(self, yaw_rate, duration, direction="cw"):
        """ Spin in place at the given rate for the given duration.

        Pausable

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
        self._can_do_in_air_commands()
        assert ((x is not None and y is not None and z is not None)
                or (lat is not None, long is not None, amsl is not None)), \
            "Must provide a full set of either NED or GPS coordinates!"

        # Check that we have a trajectory generator whose output we can process
        assert self.trajectory_gen is not None and self.trajectory_gen.setpoint_type in self.VALID_SETPOINT_TYPES

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
            if not self.trajectory_gen.CAN_DO_GPS:
                raise RuntimeError("Trajectory generator can't use GPS coordinates!")
            self.trajectory_gen.use_gps = True
            self.trajectory_gen.set_target(np.asarray([lat, long, amsl, yaw]))
        else:
            self.trajectory_gen.use_gps = False
            self.trajectory_gen.set_target(np.asarray([x, y, z, yaw]))

        while True:
            if not self.is_paused:
                # Get and set a new setpoint to go to
                setpoint = self.trajectory_gen.next_setpoint()
                # Set appropriate setpoint
                match self.trajectory_gen.setpoint_type:
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
                        raise RuntimeError(f"Invalid generator setpoint type {self.trajectory_gen.setpoint_type}! Must "
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
            await asyncio.sleep(1/self._position_update_freq)

    async def orbit(self, radius, velocity, center_lat, center_long, amsl):
        await super().orbit(radius, velocity, center_lat, center_long, amsl)
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        yaw_behaviour = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER
        await self._action_error_wrapper(self.system.action.do_orbit, radius, velocity, yaw_behaviour, center_lat,
                                         center_long, amsl)

    async def land(self):
        self.logger.info("Trying to land...")
        await super().land()
        return await self._land_using_offbord_mode()

    async def _land_using_offbord_mode(self, error_thresh=0.0001, min_time=1):
        # TODO: Check that we have reached the ground in a more robust way
        self.logger.info("Landing!")
        ema_alt_error = 0
        going_down = True
        old_pos = self.position_ned
        start_time = time.time()
        target_pos = self._get_pos_ned_yaw()
        target_pos[2] += 0.1
        await self.set_setpoint_pos_ned(target_pos)
        if self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")
        update_freq = 2
        try:
            await self.system.telemetry.set_rate_position_velocity_ned(self._position_update_freq)
            update_freq = self._position_update_freq
        except Exception as e:
            self.logger.debug(f"Couldn't set message rate: {repr(e)}", exc_info=True)
        while going_down:
            cur_pos = self.position_ned
            ema_alt_error = dist_ned(cur_pos, old_pos) + 0.33 * ema_alt_error
            if ema_alt_error < error_thresh and time.time() > start_time + min_time:
                break
            old_pos = cur_pos.copy()
            cur_alt = self.position_ned[2]
            target_pos[2] = cur_alt + 0.5
            await self.set_setpoint_pos_ned(target_pos)
            await asyncio.sleep(1/update_freq)
        self.logger.info("Landed!")
        return True

    async def _land_using_landmode(self):
        result = await self._action_error_wrapper(self.system.action.land)
        if isinstance(result, Exception):
            self.logger.warning("Couldn't go into land mode")
        while self.flightmode is not FlightMode.LAND:
            await asyncio.sleep(1 / self._position_update_freq)
        self.logger.info("Landing!")
        while self.in_air:
            await asyncio.sleep(1 / self._position_update_freq)
        self.logger.info("Landed!")
        await self.change_flight_mode("hold")
        return True

    async def stop_execution(self):
        if self._passthrough:
            await self._passthrough.stop()
            del self._passthrough
        self.system.__del__()
        if self._server_process:
            self._server_process.terminate()
        for handler in self.logging_handlers:
            self.logger.removeHandler(handler)

    async def stop(self):
        # Override whatever else is going on and land
        self.clear_queue()
        self.cancel_action()
        if not self.is_connected:
            return True
        await self.land()
        await self.disarm()
        await self.stop_execution()
        await super().stop()
        return True

    async def kill(self):
        await self._action_error_wrapper(self.system.action.kill)
        await self.stop_execution()
        await super().kill()
        return True

    # MAVSDK Error wrapping functions

    async def _action_error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except ActionError as e:
            self.logger.error(e._result.result_str)
            return False
        return True

    async def _offboard_error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except OffboardError as e:
            self.logger.error(e._result.result_str)
            return False
        return True

    async def __manual_control_error_wrapper(self, func, *args, **kwargs):
        try:
            await asyncio.wait_for(func(*args, **kwargs), timeout=5)
        except ManualControlError as e:
            self.logger.error(e._result.result_str)
            return False
        return True


class TrajectoryGenerator(ABC):

    CAN_DO_GPS = False
    SETPOINT_TYPES = set()

    def __init__(self, drone: Drone, dt, use_gps=False, setpointtype: SetPointType = None):
        assert setpointtype in self.SETPOINT_TYPES, (f"Invalid setpoint type {setpointtype} "
                                                     f"for trajectory generator {self.__class__.__name__}")
        self.drone = drone
        self.dt = dt
        self.use_gps = use_gps
        self.setpoint_type = setpointtype
        self.target_position: np.ndarray | None = None

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

    def __init__(self, drone, dt, use_gps=False, setpointtype=SetPointType.POS_NED):
        super().__init__(drone, dt, use_gps=use_gps, setpointtype=setpointtype)

    def next_setpoint(self) -> np.ndarray:
        if self.use_gps:
            self.setpoint_type = SetPointType.POS_GLOBAL
        return self.target_position


class DirectFlightFacingForward(TrajectoryGenerator):
    """ Flies directly toward the waypoint facing towards it along the way. Turning towards the target yaw happens
    after we reach the waypoint. Control happens only through velocity setpoints.
    """
    # TODO: Implement GPS
    # TODO: Figure out better way to handle yaw rate

    SETPOINT_TYPES = {SetPointType.VEL_NED}
    CAN_DO_GPS = False

    def __init__(self, drone, dt, use_gps=False, setpointtype=SetPointType.VEL_NED,
                 max_vel_h=1.0, max_vel_z=0.5, max_acc_h=0.5, max_acc_z=0.25, max_yaw_rate=20, ):
        super().__init__(drone, dt, use_gps=use_gps, setpointtype=setpointtype)
        self.max_vel_h = max_vel_h
        self.max_vel_z = max_vel_z
        self.max_acc_h = max_acc_h
        self.max_acc_z = max_acc_z
        self.max_yaw_rate = max_yaw_rate

        self.fudge_yaw = 1#2.5
        self.fudge_xy = 1#8
        self.fudge_z = 1

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
