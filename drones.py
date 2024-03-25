import asyncio
import math
import random
import threading
from abc import ABC, abstractmethod

import numpy as np
from urllib.parse import urlparse, urlunparse

from typing import Dict

from mavsdk import System
from mavsdk.telemetry import FlightMode, FixType
from mavsdk.action import ActionError
from mavsdk.offboard import PositionNedYaw, OffboardError


# TODO: Implement a mission manager/queue thing.
# TODO: Use the modes to check completion of command maybe?
# TODO: health info

class Battery:
    def __init__(self):
        self.id = None
        self.remaining = math.nan
        self.consumed = math.nan
        self.voltage = math.nan
        self.temperature = math.nan

    def __str__(self):
        return f"Remain: {self.remaining}   Consumed: {self.consumed}   V: {self.voltage}   T: {self.temperature}"


class Drone(ABC):

    VALID_FLIGHTMODES = []

    def __init__(self, name, *args, **kwargs):
        self.name = name
        self.drone_addr = None

    @property
    @abstractmethod
    def is_connected(self):
        pass

    @property
    @abstractmethod
    def is_armed(self):
        pass

    @property
    @abstractmethod
    def flightmode(self):
        pass

    @property
    @abstractmethod
    def in_air(self):
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
    async def arm(self):
        pass

    @abstractmethod
    async def disarm(self):
        pass

    @abstractmethod
    async def takeoff(self):
        pass

    @abstractmethod
    async def change_flight_mode(self, flightmode):
        pass

    @abstractmethod
    async def fly_to_point(self, point: np.ndarray, tolerance=0.5):
        pass

    @abstractmethod
    async def land(self):
        pass

    @abstractmethod
    async def stop(self):
        pass

    @abstractmethod
    async def kill(self):
        pass


# MAVSDK Error wrapping functions

async def _action_error_wrapper(func, *args, **kwargs):
    try:
        await func(*args, **kwargs)
    except ActionError as e:
        print(e._result.result_str)
        return False
    return True


async def _offboard_error_wrapper(func, *args, **kwargs):
    try:
        await func(*args, **kwargs)
    except OffboardError as e:
        print(e._result.result_str)
        return False
    return True


def parse_address(string=None, scheme=None, host=None, port=None, return_string=False):
    """ Used to ensure that udp://:14540, udp://localhost:14540 and udp://127.0.0.1:14540 are recognized as equivalent.

    Missing elements from the string or the other entries are replaced with defaults. These are udp, empty host and 50051, respectively.
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


class DroneMAVSDK(Drone, threading.Thread):

    VALID_FLIGHTMODES = ["hold", "offboard", "return", "land", "takeoff"]
    # This attribute is for checking which flight modes can be changed into manually

    def __init__(self, name, mavsdk_server_address: str | None = None, mavsdk_server_port: int = 50051, compid=160):
        threading.Thread.__init__(self)
        Drone.__init__(self, name)
        self.compid = compid
        self.system = None
        self.server_addr = mavsdk_server_address
        self.server_port = mavsdk_server_port
        self._is_connected: bool = False
        self._is_armed: bool = False
        self._flightmode: FlightMode = FlightMode.UNKNOWN
        self._in_air: bool = False
        self._gps_info: FixType | None = None
        self._position_g: np.ndarray = np.zeros((4,), dtype=np.double)  # Latitude, Longitude, AMSL, Relative altitude
        self._position_ned: np.ndarray = np.zeros((3,))     # NED
        self._velocity: np.ndarray = np.zeros((3,))         # NED
        self._attitude: np.ndarray = np.zeros((3,))         # Roll, pitch and yaw, with positives right up and right.
        self._batteries: Dict[int, Battery] = {}
        self.start()

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
    def batteries(self) -> Dict[int, Battery]:
        return self._batteries

    async def connect(self, drone_address) -> bool:
        print(self.server_addr, self.server_port, drone_address)
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
        asyncio.create_task(self._connect_check())
        asyncio.create_task(self._arm_check())
        asyncio.create_task(self._flightmode_check())
        asyncio.create_task(self._inair_check())
        asyncio.create_task(self._gps_check())
        asyncio.create_task(self._g_pos_check())
        asyncio.create_task(self._vel_rpos_check())
        asyncio.create_task(self._att_check())
        asyncio.create_task(self._battery_check())

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

    async def arm(self):
        return await _action_error_wrapper(self.system.action.arm)

    async def disarm(self):
        return await _action_error_wrapper(self.system.action.disarm)

    async def takeoff(self):
        if not self.is_armed:
            raise RuntimeError("Can't take off without being armed!")
        return await _action_error_wrapper(self.system.action.takeoff)

    async def change_flight_mode(self, flightmode: str):
        if flightmode == "hold":
            return await _action_error_wrapper(self.system.action.hold)
        elif flightmode == "offboard":
            return await _offboard_error_wrapper(self.system.offboard.start)
        elif flightmode == "return":
            return await _action_error_wrapper(self.system.action.return_to_launch)
        elif flightmode == "land":
            return await _action_error_wrapper(self.system.action.land)
        elif flightmode == "takeoff":
            return await _action_error_wrapper(self.system.action.takeoff)
        else:
            raise KeyError(f"{flightmode} is not a valid flightmode!")

    async def set_setpoint(self, point: np.ndarray):
        # point should be a numpy array of size (4,) for north, east, down, yaw, with yaw in degrees.
        point_ned_yaw = PositionNedYaw(*point)
        await _offboard_error_wrapper(self.system.offboard.set_position_ned, point_ned_yaw)
        return True

    async def fly_to_point(self, target_pos: np.ndarray, tolerance=0.5):
        # Do set_setpoint, but also put into offboard mode if we are not already in it?
        if not self.is_armed or not self.in_air:
            raise RuntimeError("Can't fly a landed or unarmed drone!")
        await self.set_setpoint(target_pos)
        if self._flightmode != FlightMode.OFFBOARD:
            await self.change_flight_mode("offboard")
        while True:
            cur_pos = self.position_ned
            if np.sqrt(np.sum((target_pos[:3]-cur_pos)**2, axis=0)) < tolerance:
                return True
            else:
                await asyncio.sleep(0.05)

    async def fly_circle(self, velocity, radius, angle, direction):
        raise NotImplementedError

    async def land(self):
        await _action_error_wrapper(self.system.action.land)
        return True

    async def stop(self):
        # Override whatever else is going on and land?
        Warning("The stop command currently lands the drone and then disarms it")
        if not self.is_connected:
            return True
        await self.land()
        while self.in_air:
            await asyncio.sleep(0.1)
        await self.disarm()
        return True

    async def kill(self):
        await _action_error_wrapper(self.system.action.kill)
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

    async def fly_circle(self, velocity, radius, angle, direction):
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
