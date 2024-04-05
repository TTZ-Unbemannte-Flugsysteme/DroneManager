import asyncio
from collections import deque
import math
import os.path
import random
import threading
import haversine
import platform
from subprocess import Popen, DEVNULL
from abc import ABC, abstractmethod

import numpy as np
from urllib.parse import urlparse, urlunparse

from typing import Dict, Deque

from mavsdk import System
from mavsdk.telemetry import FlightMode, FixType, StatusText, StatusTextType
from mavsdk.action import ActionError, OrbitYawBehavior
from mavsdk.offboard import PositionNedYaw, PositionGlobalYaw, OffboardError

import logging

_cur_dir = os.path.dirname(os.path.abspath(__file__))
_mav_server_file = os.path.join(_cur_dir, "mavsdk_server_bin.exe")

# TODO: health info


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


class Drone(ABC, threading.Thread):

    VALID_FLIGHTMODES = []

    def __init__(self, name, *args, **kwargs):
        threading.Thread.__init__(self)
        self.name = name
        self.drone_addr = None
        self.action_queue: Deque[asyncio.Task] = deque()
        self.current_action: asyncio.Task | None = None
        self.should_stop = threading.Event()
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        self.start()
        asyncio.create_task(self._task_scheduler())

    def run(self):
        while not self.should_stop:
            pass

    async def _task_scheduler(self):
        while True:
            while len(self.action_queue) > 0:
                self.current_action = asyncio.create_task(self.action_queue.popleft())
                await self.current_action
            else:
                await asyncio.sleep(0.5)

    def schedule_task(self, coro):
        self.action_queue.append(coro)

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
    async def takeoff(self) -> bool:
        pass

    @abstractmethod
    async def change_flight_mode(self, flightmode) -> bool:
        pass

    @abstractmethod
    async def fly_to_point(self, point: np.ndarray, tolerance=0.5) -> bool:
        pass

    @abstractmethod
    async def fly_to_gps(self, latitude, longitude, altitude, yaw, tolerance=0.5) -> bool:
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

    async def clear_queue(self) -> None:
        """ Clears the action queue
        Does not cancel
        """
        self.action_queue.clear()

    async def cancel_action(self) -> None:
        if self.current_action:
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
    elif host == "0.0.0.0":
        host = ""
    if port is None:
        port = 50051
    if return_string:
        return urlunparse((scheme, ":".join([host, str(port)]), "", "", "", ""))
    else:
        return scheme, host, port


class DroneMAVSDK(Drone):

    VALID_FLIGHTMODES = ["hold", "offboard", "return", "land", "takeoff"]
    # This attribute is for checking which flight modes can be changed into manually

    def __init__(self, name, mavsdk_server_address: str | None = None, mavsdk_server_port: int = 50051, compid=160):
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
        self._position_update_freq = 20                     # How often (per second) go-to-position commands compute if they have arrived.

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
        if self.server_addr is None and platform.system() == "Windows":
            self._server_process = Popen(f".\\mavsdk_server_bin.exe -p {self.server_port} {drone_address}",
                                         stdout=DEVNULL, stderr=DEVNULL)
            self.server_addr = "127.0.0.1"
        self.system = System(mavsdk_server_address=self.server_addr, port=self.server_port, compid=self.compid)
        _, self.server_addr, self.server_port = parse_address(scheme="udp",
                                                              host=self.server_addr,
                                                              port=self.server_port)
        drone_address = parse_address(string=drone_address, return_string=True)
        self.drone_addr = drone_address
        await self.system.connect(system_address=self.drone_addr)
        async for state in self.system.core.connection_state():
            if state.is_connected:
                await self._schedule_update_tasks()
                return True
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

    async def _connect_check(self):
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
            self._position_ned[0] = pos_vel.position.north_m
            self._position_ned[1] = pos_vel.position.east_m
            self._position_ned[2] = pos_vel.position.down_m

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
            if message.type == StatusTextType.DEBUG:
                self.logger.debug(message.text)
            elif message.type in [StatusTextType.INFO, StatusTextType.NOTICE]:
                self.logger.info(message.text)
            elif message.type == StatusTextType.WARNING:
                self.logger.warning(message.text)
            else:
                self.logger.error(message.text)

    async def arm(self):
        await super().arm()
        if self.current_action:
            self.current_action.cancel()
        return await self._action_error_wrapper(self.system.action.arm)

    async def disarm(self):
        await super().disarm()
        return await self._action_error_wrapper(self.system.action.disarm)

    async def takeoff(self):
        await super().takeoff()
        if not self.is_armed:
            raise RuntimeError("Can't take off without being armed!")
        await self._action_error_wrapper(self.system.action.takeoff)
        while self.flightmode is FlightMode.TAKEOFF:
            await asyncio.sleep(self._position_update_freq)
        return True

    async def change_flight_mode(self, flightmode: str):
        await super().change_flight_mode(flightmode)
        if flightmode == "hold":
            return await self._action_error_wrapper(self.system.action.hold)
        elif flightmode == "offboard":
            return await self._offboard_error_wrapper(self.system.offboard.start)
        elif flightmode == "return":
            return await self._action_error_wrapper(self.system.action.return_to_launch)
        elif flightmode == "land":
            return await self._action_error_wrapper(self.system.action.land)
        elif flightmode == "takeoff":
            return await self._action_error_wrapper(self.system.action.takeoff)
        else:
            raise KeyError(f"{flightmode} is not a valid flightmode!")

    async def _set_setpoint_ned(self, point):
        # point should be a numpy array of size (4,) for north, east, down, yaw, with yaw in degrees.
        point_ned_yaw = PositionNedYaw(*point)
        await self._offboard_error_wrapper(self.system.offboard.set_position_ned, point_ned_yaw)
        return True

    async def _set_setpoint_gps(self, latitude, longitude, alt_rel, yaw):
        alt_type = PositionGlobalYaw.AltitudeType.AMSL
        position = PositionGlobalYaw(lat_deg=latitude, lon_deg=longitude, alt_m=alt_rel,
                                     yaw_deg=yaw, altitude_type=alt_type)
        await self._offboard_error_wrapper(self.system.offboard.set_position_global, position)
        return True

    async def fly_to_point(self, target_pos: np.ndarray, tolerance=0.5):
        await super().fly_to_point(target_pos, tolerance)
        # Do set_setpoint, but also put into offboard mode if we are not already in it?
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        await self._set_setpoint_ned(target_pos)
        if self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")
        while True:
            cur_pos = self.position_ned
            if dist_ned(cur_pos, target_pos[:3]) < tolerance:
                return True
            else:
                await asyncio.sleep(1/self._position_update_freq)

    async def fly_to_gps(self, latitude, longitude, amsl, yaw, tolerance=0.5):
        await super().fly_to_gps(latitude, longitude, amsl, yaw=yaw, tolerance=tolerance)
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        await self._action_error_wrapper(self.system.action.goto_location, latitude, longitude, amsl, yaw)
        while True:
            while True:
                lat1, long1, alt1 = np.take(self.position_global, [0, 1, 2])
                if dist_gps(lat1, long1, alt1, latitude, longitude, amsl) < tolerance:
                    return True
                else:
                    await asyncio.sleep(1 / self._position_update_freq)

    async def orbit(self, radius, velocity, center_lat, center_long, amsl):
        await super().orbit(radius, velocity,center_lat, center_long, amsl)
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        yaw_behaviour = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER
        await self._action_error_wrapper(self.system.action.do_orbit, radius, velocity, yaw_behaviour, center_lat,
                                         center_long, amsl)

    async def land(self):
        await super().land()
        await self._action_error_wrapper(self.system.action.land)
        return True

    def _stop_tasks(self):
        for task in self._running_tasks:
            task.cancel()

    async def stop(self):
        await super().stop()
        # Override whatever else is going on and land?
        Warning("The stop command currently lands the drone and then disarms it")
        if not self.is_connected:
            return True
        await self.land()
        while self.in_air:
            await asyncio.sleep(0.1)
        await self.disarm()
        self._stop_tasks()
        if self._server_process:
            self._server_process.terminate()
        return True

    async def kill(self):
        await self._action_error_wrapper(self.system.action.kill)
        self._stop_tasks()
        if self._server_process:
            self._server_process.terminate()
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


class DummyMAVDrone(Drone):

    VALID_FLIGHTMODES = ["hold", "offboard", "return", "land", "takeoff"]

    def __init__(self, name, mavsdk_server_address: str = "localhost", mavsdk_server_port: int = 50050):
        super().__init__(name)
        _, self.server_addr, self.server_port = parse_address(scheme="udp", host=mavsdk_server_address,
                                                              port=mavsdk_server_port)
        self._is_connected = False
        self._is_armed = False
        self._flightmode = None
        self._in_air = False
        asyncio.create_task(self._fail_offboard())

    @property
    def is_connected(self):
        return self._is_connected

    @property
    def is_armed(self):
        return self._is_armed

    @property
    def flightmode(self):
        return self._flightmode

    @property
    def in_air(self):
        return self._in_air

    async def _fail_offboard(self):
        while True:
            until_next = random.uniform(10, 15)
            duration = random.uniform(3, 5)
            if self.flightmode == "OFFBOARD":
                self._flightmode = "HOLD"
                await asyncio.sleep(duration)
                await self.change_flight_mode("offboard")
            await asyncio.sleep(until_next)

    async def connect(self, drone_addr):
        drone_address = parse_address(string=drone_addr, return_string=True)
        self.drone_addr = drone_address
        con_delay = random.uniform(1, 6)
        con_success = random.uniform(0, 1)
        con_success_rate = 0.8
        await asyncio.sleep(con_delay)
        if con_success < con_success_rate:
            self._is_connected = True
            return True
        else:
            return False

    async def arm(self):
        await asyncio.sleep(random.uniform(0.1, 1))
        if self.is_armed:
            return False
        self._is_armed = True
        return True

    async def disarm(self):
        await asyncio.sleep(random.uniform(0.1, 1))
        if self.is_armed:
            self._is_armed = False
            return True
        return False

    async def takeoff(self):
        if not self.is_armed:
            raise RuntimeError("Can't takeoff while disarmed!")
        if self.in_air:
            raise RuntimeError("Already in the air!")
        self._flightmode = "TAKEOFF"
        self._in_air = True
        await asyncio.sleep(random.uniform(1, 3))
        return True

    async def change_flight_mode(self, flightmode):
        await asyncio.sleep(random.uniform(0.1, 3))
        if flightmode == "hold":
            self._flightmode = "HOLD"
        elif flightmode == "offboard":
            self._flightmode = "OFFBOARD"
        elif flightmode == "return":
            self._flightmode = "RTB"
        elif flightmode == "land":
            await self.land()
        elif flightmode == "takeoff":
            await self.takeoff()
        else:
            raise KeyError(f"{flightmode} is not a valid flightmode!")
        return True

    async def fly_to_point(self, point: np.ndarray, tolerance=0.5):
        await asyncio.sleep(random.uniform(1, 10))
        if not self.is_armed:
            raise RuntimeError("Can't fly an unarmed drone!")
        return True

    async def land(self):
        if not self.is_armed:
            raise RuntimeError("Can't land an unarmed drone!")
        if not self.in_air:
            raise RuntimeError("Already landed!")
        self._flightmode = "LAND"
        await asyncio.sleep(random.uniform(3, 10))
        self._in_air = False
        return True

    async def stop(self):
        await self.land()
        while self.in_air:
            await asyncio.sleep(0.1)
        return True

    async def kill(self):
        self._is_connected = False
        self._is_armed = False
        self._in_air = False
        self._flightmode = None
