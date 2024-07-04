import math

from mavsdk.gimbal import GimbalError, ControlMode

from dronecontrol.plugins import DronePlugin
from dronecontrol.utils import relative_gps


class Gimbal(DronePlugin):
    
    PREFIX = "GMBL"

    def __init__(self, logger, drone):
        super().__init__(logger, drone)
        self.drone = drone
        self.cli_commands = {
            "control": self.take_control,
            "release": self.release_control,
            "set-angles": self.set_gimbal_angles,
        }
        self.background_functions = [
            self._check_gimbal_attitude(),
            self._check_gimbal_control(),
        ]
        self.roll = math.nan
        self.pitch = math.nan
        self.yaw = math.nan
        self.primary_control = (0, 0)
        self.secondary_control = (0, 0)

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
            self.logger.debug(f"Gimbal control: {gimbal_control.control_mode} "
                              f"P:{gimbal_control.sysid_primary_control}:{gimbal_control.compid_primary_control}, "
                              f"S:{gimbal_control.sysid_secondary_control}:{gimbal_control.compid_secondary_control}")
            
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
        
    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except GimbalError as e:
            self.logger.error(e._result.result_str)
            return False
        return True
