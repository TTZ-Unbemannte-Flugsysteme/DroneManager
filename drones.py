import asyncio
import numpy as np
import random

from abc import ABC, abstractmethod

from mavsdk import System


# TODO: Implement a mission manager/queue thing.

class Drone(ABC):

    def __init__(self, name, *args, **kwargs):
        self.name = name

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
    async def land(self):
        pass

    @abstractmethod
    async def stop(self):
        pass

    @abstractmethod
    async def kill(self):
        pass


class DroneMAVSDK(Drone):

    def __init__(self, name, mavsdk_server_address: str | None = None, mavsdk_server_port: int = 50051, ):
        super().__init__(name)
        self.drone_addr = None
        self.server_addr = mavsdk_server_address
        self.server_port = mavsdk_server_port
        self.system = System(mavsdk_server_address=self.server_addr, port=self.server_port)
        self._is_connected = False
        self._is_armed = False
        self._flightmode = "None"

    @property
    def is_connected(self):
        return self._is_connected

    @property
    def is_armed(self):
        return self._is_armed

    @property
    def flightmode(self):
        return self._flightmode

    async def connect(self, drone_address: str = "udp://:14540"):
        self.drone_addr = drone_address
        await self.system.connect(system_address=self.drone_addr)
        async for state in self.system.core.connection_state():
            if state.is_connected:
                await self._schedule_update_tasks()
                return True
        return False

    async def _schedule_update_tasks(self):
        asyncio.create_task(self._connect_check())
        asyncio.create_task(self._arm_check())
        asyncio.create_task(self._flightmode_check())

    async def _connect_check(self):
        async for state in self.system.core.connection_state():
            self._is_connected = state.is_connected

    async def _arm_check(self):
        async for arm in self.system.telemetry.armed():
            self._is_armed = arm

    async def _flightmode_check(self):
        async for flightmode in self.system.telemetry.flight_mode():
            self._flightmode = flightmode.name

    async def arm(self):
        if not self.is_armed:
            try:
                await self.system.action.arm()
                return True
            except Exception as e:  # add exception later
                print(e)
        else:
            return False

    async def disarm(self):
        if self.is_armed:
            try:
                await self.system.action.disarm()
                return True
            except Exception as e:  # add exception later
                print(e)
        else:
            return False

    async def offboard(self):
        await self.system.offboard.start()
        return True

    async def fly_to_point(self, point: np.ndarray, tolerance=0.5):
        raise NotImplementedError

    async def fly_circle(self, velocity, radius, angle, direction):
        raise NotImplementedError

    async def land(self):
        await self.system.action.land()
        return True

    async def stop(self):
        # land, disarm then stop?
        Warning("The stop command currently lands the drone")
        await self.land()
        return True

    async def kill(self):
        raise NotImplementedError


class DummyMAVDrone(Drone):
    def __init__(self, name, mavsdk_server_address: str = "localhost", mavsdk_server_port: int = 50050):
        super().__init__(name)
        self.server_addr = mavsdk_server_address
        self.server_port = mavsdk_server_port
        self.connection_address = None
        self._is_connected = False
        self._is_armed = False
        self._flightmode = None
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

    async def _fail_offboard(self):
        while True:
            until_next = random.uniform(10, 15)
            duration = random.uniform(3, 5)
            if self.flightmode == "Offboard":
                self._flightmode = "Manual"
                await asyncio.sleep(duration)
                await self.offboard()
            await asyncio.sleep(until_next)

    async def connect(self, connection_address):
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

    async def offboard(self):
        if not self.is_armed:
            raise RuntimeError("Can't offboard a disarmed drone!")
        self._flightmode = "Offboard"
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
        await asyncio.sleep(random.uniform(1, 10))
        return True

    async def stop(self):
        #if self.is_armed:
        await self.land()
        return True

    async def kill(self):
        raise NotImplementedError
