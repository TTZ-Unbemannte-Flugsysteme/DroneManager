import asyncio
import numpy as np
import random
import mock

from dronecontrol import DroneManager, Drone


# DUMMY STUFF #

class DummyDrone(Drone):
    def __init__(self, name, mavsdk_server_address: str = "localhost", mavsdk_server_port: int = 50050):
        self.name = name
        self.server_addr = mavsdk_server_address
        self.server_port = mavsdk_server_port
        self.connection_address = None
        self.is_connected = False
        self.is_armed = False
        self.dummy_offboard = False
        asyncio.create_task(self._fail_offboard())

    async def _fail_offboard(self):
        while True:
            until_next = random.uniform(10, 15)
            duration = random.uniform(3, 5)
            if self.offboard_mode:
                self.offboard_mode = False
                await asyncio.sleep(duration)
                await self.offboard()
            await asyncio.sleep(until_next)

    async def connect(self, connection_address):
        con_delay = random.uniform(1, 6)
        con_success = random.uniform(0, 1)
        con_success_rate = 0.8
        await asyncio.sleep(con_delay)
        if con_success < con_success_rate:
            self.is_connected = True
            return True
        else:
            return False

    @property
    async def is_offboard(self):
        return self.dummy_offboard

    async def arm(self):
        await asyncio.sleep(random.uniform(0.1, 1))
        if self.is_armed:
            return False
        self.is_armed = True
        return True

    async def disarm(self):
        await asyncio.sleep(random.uniform(0.1, 1))
        if self.is_armed:
            self.is_armed = False
            return True
        return False

    async def offboard(self):
        if not self.is_armed:
            raise RuntimeError("Can't offboard a disarmed drone!")
        self.dummy_offboard = True
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
        await asyncio.sleep(random.uniform(1, 10))
        if not self.is_armed:
            raise RuntimeError("Can't land an unarmed drone!")
        return True

    async def stop(self):
        if self.is_armed:
            await self.land()
            await self.disarm()
        return True

    async def kill(self):
        raise NotImplementedError


# END DUMMY STUFF #

if __name__ == "__main__":
    with mock.patch('dronecontrol.Drone', DummyDrone):
        app = DroneManager()
        app.run()
