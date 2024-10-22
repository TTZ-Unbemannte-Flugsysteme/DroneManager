import asyncio
from collections import OrderedDict
import numpy as np
from dronecontrol.drone import Waypoint, WayPointType, Drone

from dronecontrol.plugins import Plugin

# TODO: Think of better ways to do this. Issues with current implementation: Duelling commands, drones might be in
#  multiple formations, etc...
#   Possible alternative implementations: A trajectory follower algorithm, a hook on move commands, simply following
#   leader with an offset


class FormationsPlugin(Plugin):
    PREFIX = "form"

    def __init__(self, dm, logger):
        super().__init__(dm, logger)
        self.cli_commands = {
            "offset": self.set_offset,
            "add": self.add_drones,
            "remove": self.remove_drones,
            "fly": self.fly_to,
            "status": self.status,
        }
        self.drones: OrderedDict[str, Drone] = OrderedDict()
        self.offset = np.empty((3,))
        self.offset[:] = np.nan
        self.minimum_distance = 3
        self.background_functions = [
        ]

    @property
    def valid_formation(self):
        return np.sqrt(self.offset.dot(self.offset)) > self.minimum_distance and not np.isnan(self.offset).any()

    async def start(self):
        self.logger.debug("Starting formations plugin...")

    async def set_offset(self, offset_north: float, offset_east: float, offset_up: float):
        self.offset[0] = offset_north
        self.offset[1] = offset_east
        self.offset[2] = offset_up
        self.logger.info(f"Formation offset set to {self.offset}")

    async def add_drones(self, names: list[str]):
        for name in names:
            assert name in self.dm.drones, f"No drone named {name}"
        for name in names:
            self.drones[name] = self.dm.drones[name]
        await self.status()

    async def remove_drones(self, names: list[str]):
        for name in names:
            try:
                self.drones.pop(name)
            except KeyError:
                pass
        await self.status()

    async def status(self):
        self.logger.info(f"Drones {", ".join(self.drones.keys())} currently flying in formation with offset {self.offset}.")
        if not self.valid_formation:
            self.logger.warning(f"Formation currently does not have valid offsets, flying is impossible.")

    async def fly_to(self, lat: float, long: float, amsl: float):
        formation_waypoint = Waypoint(WayPointType.POS_GLOBAL, gps=np.asarray([lat, long, amsl]))
        assert self.valid_formation
        self.logger.info(f"Formation flying to {lat, long, amsl}")
        tasks = []
        for i, name in enumerate(self.drones):
            drone_gps = formation_waypoint.offset_gps(*self.offset*i).gps
            self.logger.debug(f"Sending {name} to {drone_gps}")
            tasks.append(self.drones[name].fly_to(lat=drone_gps[0], long=drone_gps[1], amsl=drone_gps[2]))
        await asyncio.gather(*tasks)
        self.logger.info("Formation has reached position!")

    async def close(self):
        """ Removes all drones from formation"""
        await self.remove_drones(list(self.drones.keys()))
