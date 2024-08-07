import asyncio
import math
from typing import Dict

from mavsdk.gimbal import GimbalError
from mavsdk.gimbal import ControlMode as MAVControlMode
from mavsdk.gimbal import GimbalMode as MAVGimbalMode

from dronecontrol.plugins import Plugin
from dronecontrol.utils import relative_gps


# TODO: Support multiple gimbals per drone
#   Currently looks like it might be supported, but actually isn't. Will have to do this without MAVSDK.

ControlMode = MAVControlMode
GimbalMode = MAVGimbalMode


class GimbalPlugin(Plugin):
    
    PREFIX = "gmbl"

    def __init__(self, dm, logger):
        super().__init__(dm, logger)
        self.cli_commands = {
            "add": self.add_gimbals,
            "remove": self.remove_gimbal,
            "control": self.take_control,
            "release": self.release_control,
            "angles": self.set_gimbal_angles,
            "point": self.point_gimbal_at,
            "mode": self.set_gimbal_mode,
            "status": self.status,
        }
        self.background_functions = [
        ]
        self.gimbals: Dict[str, Dict[int, Gimbal]] = {}  # Dictionary with drone names as keys and list of gimbals on that drone as values
        self.running_tasks = set()

    def add_gimbals(self, name: str):
        """ Add Gimbals from/for a given drone to the plugin"""
        drone = self.dm.drones[name]
        self.gimbals[name] = {0: Gimbal(self.logger, drone)}

    def remove_gimbal(self, name: str, gimbal_id: int):
        """ Remove a gimbal from the plugin"""
        gimbal = self.gimbals[name].pop(gimbal_id)
        gimbal.close()
        del gimbal

    def status(self, name: str, gimbal_id: int):
        self.gimbals[name][gimbal_id].log_status()

    async def take_control(self, name: str, gimbal_id: int):
        await self.gimbals[name][gimbal_id].take_control()

    async def release_control(self, name: str, gimbal_id: int):
        await self.gimbals[name][gimbal_id].release_control()

    async def set_gimbal_angles(self, name: str, gimbal_id: int, roll: float, pitch: float, yaw: float):
        return await self.gimbals[name][gimbal_id].set_gimbal_angles(roll, pitch, yaw)

    async def point_gimbal_at(self, name: str, gimbal_id: int, x1: float, x2: float, x3: float, relative: bool = False):
        if relative:
            return await self.gimbals[name][gimbal_id].point_gimbal_at_relative(x1, x2, x3)
        else:
            return await self.gimbals[name][gimbal_id].point_gimbal_at(x1, x2, x3)

    async def set_gimbal_mode(self, name: str, gimbal_id: int, mode: str):
        return await self.gimbals[name][gimbal_id].set_gimbal_mode(mode)


class Gimbal:

    def __init__(self, logger, drone):
        self.logger = logger
        self.drone = drone

        self.roll = math.nan
        self.pitch = math.nan
        self.yaw = math.nan
        self.primary_control = (math.nan, math.nan)
        self.secondary_control = (math.nan, math.nan)
        self.running_tasks = set()
        self._start_background_tasks()

    def _start_background_tasks(self):
        self.running_tasks.add(asyncio.create_task(self._check_gimbal_attitude()))
        self.running_tasks.add(asyncio.create_task(self._check_gimbal_control()))

    def close(self):
        for task in self.running_tasks:
            task.cancel()

    @property
    def in_control(self):
        return (self.drone.gcs_system_id == self.primary_control[0] and
                self.drone.gcs_component_id == self.primary_control[1])

    async def _check_gimbal_attitude(self):
        async for attitude in self.drone.system.gimbal.attitude():
            rpy = attitude.euler_angle_forward
            self.roll = rpy.roll_deg
            self.pitch = rpy.pitch_deg
            self.yaw = rpy.yaw_deg

    async def _check_gimbal_control(self):
        async for gimbal_control in self.drone.system.gimbal.control():
            self.primary_control = (gimbal_control.sysid_primary_control, gimbal_control.compid_primary_control)
            self.secondary_control = (gimbal_control.sysid_secondary_control, gimbal_control.compid_secondary_control)
            self.logger.debug(f"Gimbal control: {gimbal_control.control_mode}, "
                              f"P: {gimbal_control.sysid_primary_control, gimbal_control.compid_primary_control}, "
                              f"S: {gimbal_control.sysid_secondary_control, gimbal_control.compid_secondary_control}")

    def log_status(self):
        self.logger.info(f"Gimbal control P:{self.primary_control}, S: {self.secondary_control}, Roll: {self.roll}, "
                         f"Pitch: {self.pitch}, Yaw: {self.yaw}")
            
    async def take_control(self):
        self.logger.info("Taking control of gimbal...")
        await self._error_wrapper(self.drone.system.gimbal.take_control, ControlMode.PRIMARY)

    async def release_control(self):
        self.logger.info("Releasing control of gimbal...")
        await self._error_wrapper(self.drone.system.gimbal.release_control, ControlMode.PRIMARY)

    async def point_gimbal_at(self, lat, long, amsl):
        return await self._error_wrapper(self.drone.system.gimbal.set_roi_location, lat, long, amsl)

    async def point_gimbal_at_relative(self, x, y, z):
        lat, long, amsl = relative_gps(x, y, z, *self.drone.position_global[:3])
        return await self.point_gimbal_at(lat, long, amsl)

    async def set_gimbal_angles(self, roll, pitch, yaw):
        await self._error_wrapper(self.drone.system.gimbal.set_angles, roll, pitch, yaw)

    async def set_gimbal_mode(self, mode):
        assert mode in ["follow", "lock"]
        if mode == "follow":
            await self._error_wrapper(self.drone.system.gimbal.set_mode, GimbalMode.YAW_FOLLOW)
        elif mode == "lock":
            await self._error_wrapper(self.drone.system.gimbal.set_mode, GimbalMode.YAW_LOCK)

    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except GimbalError as e:
            self.logger.error(e._result.result_str)
            return False
        return True
