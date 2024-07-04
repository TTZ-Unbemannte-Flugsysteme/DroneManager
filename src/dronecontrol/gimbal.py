from mavsdk.gimbal import GimbalError, ControlMode

from dronecontrol.plugins import Plugin
from dronecontrol.utils import relative_gps


class Gimbal(Plugin):
    
    PREFIX = "GMBL"

    def __init__(self, dm, logger):
        super().__init__(dm, logger)
        self.gimbals = {}
        self.cli_commands = {
            "control": {"func": self.take_control, "help": "Take control of the gimbal on a drone.",
                        "args": [
                                {"arg": "name", "type": str, "help": "Which drones gimbal to take over."},
                            ]
                        },
            "angles": {"func": self.set_gimbal_angles, "help": "Set the gimbal to specific angles.",
                       "args": [
                               {"arg": "name", "type": str, "help": "Which drones gimbal will have its angles changed."},
                               {"arg": "roll", "type": float, "help": "Desired roll angle."},
                               {"arg": "pitch", "type": float, "help": "Desired pitch angle."},
                               {"arg": "yaw", "type": float, "help": "Desired yaw angle."},
                           ]
                       },
        }
        self.background_functions = [
            self._check_gimbal_attitude,
            self._check_gimbal_control,
        ]

    async def _check_gimbal_attitude(self, name):
        drone = self.dm.drones[name]
        async for attitude in drone.system.gimbal.attitude():
            rpy = attitude.euler_angle_forward
            self.logger.debug(f"Gimbal attitude: R:{rpy.roll_deg}, P: {rpy.pitch_deg}, Y: {rpy.yaw_deg}")

    async def _check_gimbal_control(self, name):
        drone = self.dm.drones[name]
        async for gimbal_control in drone.system.gimbal.control():
            self.logger.debug(f"Gimbal control: {gimbal_control.control_mode} "
                              f"P:{gimbal_control.sysid_primary_control}:{gimbal_control.compid_primary_control}, "
                              f"S:{gimbal_control.sysid_secondary_control}:{gimbal_control.compid_secondary_control}")
            
    async def take_control(self, name):
        self.logger.info("Taking control of gimbal...")
        drone = self.dm.drones[name]
        await self._error_wrapper(drone.system.gimbal.take_control, ControlMode.PRIMARY)

    async def point_gimbal_at(self, name, lat, long, amsl):
        drone = self.dm.drones[name]
        return await self._error_wrapper(drone.system.gimbal.set_roi_location, lat, long, amsl)

    async def point_gimbal_at_relative(self, name, x, y, z):
        drone = self.dm.drones[name]
        lat, long, amsl = relative_gps(x, y, z, *drone.position_global[:3])
        return await self.point_gimbal_at(name, lat, long, amsl)

    async def set_gimbal_angles(self, name, roll, pitch, yaw):
        drone = self.dm.drones[name]
        await self._error_wrapper(drone.system.gimbal.set_angles, roll, pitch, yaw)
        
    async def _error_wrapper(self, func, *args, **kwargs):
        try:
            await func(*args, **kwargs)
        except GimbalError as e:
            self.logger.error(e._result.result_str)
            return False
        return True
