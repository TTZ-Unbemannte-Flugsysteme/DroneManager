import asyncio
import math
import numpy as np
from typing import Dict, List, Tuple
from collections import OrderedDict

from dronecontrol import DroneManager

# TODO: Move the qualify and redcross functions from app.py to here somehow
#   TODO: How to do the parsing?  ->  Add a "add parser" hook to the app? Pass the subparsers object to RedCross?
#   TODO: How to do the functions?  ->  ???

# TODO: Fix stage 4 completion check (Fires during stage 5 for some reason?)


class WayPoint:
    def __init__(self, x, y, z, yaw, in_circle, offset_altitudes):
        self.x = x
        self.y = y
        self.z = z
        self.heading = yaw
        self.in_circle = in_circle
        self.offset_altitudes = offset_altitudes


class DemoDrone:
    def __init__(self, name, init_pos):
        self.name = name
        self.waypoint = init_pos
        self.launch_pos = None  # Set after takeoff


class RedCross:
    # TODO: Checks to prevent flying in the wrong stage (only allow current stage +1 and reset to stage 0(landed) or
    #  stage 1(at start positions)

    STAGE2_WP = [
        WayPoint(0, 3, -2, 0, False, True),
        WayPoint(3, 3, -2, 0, False, True),
        WayPoint(3, 3, -2, 90, False, False)
    ]

    STAGE3_WP = [
        WayPoint(3, 37, -2, 90, False, False),
        WayPoint(3, 37, -2, 0, False, True),
        WayPoint(21, 37, -2, 0, False, True),
        WayPoint(21, 37, -2, -90, False, False),
        WayPoint(21, 20, -2, -90, False, False),
    ]

    STAGE4_WP = [
        WayPoint(21, 20, -2, -90, False, True),
        WayPoint(26, 18, -2, -90, True, True),
    ]

    STAGE5_WP = [
        WayPoint(20, 20, -2, -90, False, True),
        WayPoint(3, 3, -2, -90, False, True),
        WayPoint(3, 3, -2, 0, False, True),
    ]

    def __init__(self, logger, dm: DroneManager):
        self.dm = dm
        self.drones: Dict[str, DemoDrone] = OrderedDict()
        self.logger = logger

        # formation parameters
        self.altitude_step = 1  # minimum 1m offset between adjacent drones
        self.max_alt_offset = 2  # Highest drone at most this height above the lowest
        self.base_altitude = 2  # Currently only used for takeoff

    def remove(self, name):
        try:
            self.drones.pop(name)
        except KeyError:
            self.logger.warning(f"No drone named {name} in this demo!")

    def add(self, name):
        assert name in self.dm.drones, f"Not connected to a drone with name {name}"
        if name in self.drones:
            pass  # Don't add the same drone twice
        else:
            x, y, z = self.dm.drones[name].position_ned
            yaw = self.dm.drones[name].attitude[2]
            self.drones[name] = DemoDrone(name,  (x, y, z, yaw))

    def current_drone_list(self):
        return [drone.name for name, drone in self.drones.items()]

    def formation_line(self, waypoint: WayPoint) -> List[Tuple[float, float, float, float]]:
        # Returns a list with position_yaw coordinates for each drone
        forward = 4
        right = 0
        altitudes = self.altitudes(waypoint.offset_altitudes)
        position_yaw_local_drones = [(waypoint.x + forward*i,
                                      waypoint.y + right*i,
                                      waypoint.z + altitudes[i],
                                      waypoint.heading) for i in range(len(self.drones))]
        return position_yaw_local_drones

    def formation_circle(self, waypoint: WayPoint) -> List[Tuple[float, float, float, float]]:
        # TODO: calculate radius so we have 4m between drones on circumference, with min diameter 4m
        indices = list(range(len(self.drones)))
        if len(self.drones) == 3:
            indices = [2, 0, 1]
        elif len(self.drones) == 4:
            indices = [3, 0, 2, 1]
        elif len(self.drones) == 5:
            indices = [4, 3, 0, 1, 2]
        circle_radius = 5  # Adjust radius size as needed
        angle_offset = 2*math.pi/len(self.drones)
        altitudes = self.altitudes(waypoint.offset_altitudes)
        formation_offsets = [(circle_radius * math.sin(i*angle_offset),
                              circle_radius * math.cos(i*angle_offset),
                              altitudes[alt],
                              -(angle_offset*i*180/math.pi + 90)) for alt, i in enumerate(indices)]
        position_yaw_local_drones = [(waypoint.x + formation_offsets[i][0],
                                      waypoint.y + formation_offsets[i][1],
                                      waypoint.z + formation_offsets[i][2],
                                      formation_offsets[i][3]) for i in range(len(self.drones))]
        return position_yaw_local_drones

    def formation(self, waypoint) -> List[Tuple[float, float, float, float]]:
        if waypoint.in_circle:
            return self.formation_circle(waypoint)
        else:
            return self.formation_line(waypoint)

    def _are_at_coordinates(self):
        drones_at_coordinates = []
        for i, name in enumerate(self.drones):
            drone = self.dm.drones[name]
            at_pos = drone.is_at_pos(np.asarray(self.drones[name].waypoint[:3]), tolerance=0.2)
            at_heading = drone.is_at_heading(self.drones[name].waypoint[3], tolerance=1)
            if at_pos and at_heading:
                drones_at_coordinates.append(True)
            else:
                drones_at_coordinates.append(False)
        return drones_at_coordinates

    def altitudes(self, offset_altitudes):
        if offset_altitudes:
            altitudes = [-self.altitude_step*i for i in range(self.max_alt_offset//self.altitude_step + 1)]
        else:
            altitudes = [0 for _ in range(self.max_alt_offset//self.altitude_step + 1)]
        n_pattern = len(self.drones) // len(altitudes) + 1
        return altitudes*n_pattern  # Repeat the altitude pattern so that we have enough entries

    async def fly_stage(self, waypoints):
        for waypoint in waypoints:
            coordinates = self.formation(waypoint)

            yaw_rate = 20
            # Spin first
            coros = []
            for i, name in enumerate(self.drones):
                drone = self.dm.drones[name]
                x, y, z = drone.position_ned
                new_heading = coordinates[i][3]
                coros.append(drone.yaw_to(x, y, z, new_heading, yaw_rate=yaw_rate, tolerance=2))
            try:
                results = await asyncio.wait_for(asyncio.gather(*coros, return_exceptions=True), timeout=30)
            except TimeoutError:
                self.logger.error(f"Heading Change timed out!")
                return False
            for i, name in enumerate(self.drones):
                result = results[i]
                if isinstance(result, Exception):
                    self.logger.error(f"Drone {name} couldn't complete this heading change!")
                    self.logger.debug(repr(result), exc_info=True)
                    return False

            # Make x,y,z moves
            for i, name in enumerate(self.drones):
                drone = self.dm.drones[name]
                self.drones[name].waypoint = coordinates[i]
                await drone.set_waypoint_ned(coordinates[i])
            # Check that all drones have reached the waypoint before proceeding
            while not all(self._are_at_coordinates()):
                await asyncio.sleep(0.1)
        return True

    async def stage_1(self):
        self.logger.info(f"Starting stage 1 with {self.current_drone_list()}")
        takeoff_offsets = self.altitudes(offset_altitudes=True)
        try:
            results = await self.dm.arm(self.drones.keys(), schedule=True)
            for result in results:
                if not result or isinstance(result, Exception):
                    self.logger.warning("Failed to arm all drones!")
                    return False
            takeoff_altitudes = [(-self.base_altitude + takeoff_offsets[i]) for i, name in enumerate(self.drones)]
            self.logger.info(f"Takeoff altitudes: {takeoff_altitudes}")
            await asyncio.gather(*[self.dm.drones[name].takeoff(altitude=takeoff_altitudes[i])
                                   for i, name in enumerate(self.drones)])
            for name in self.drones:
                x, y, z = self.dm.drones[name].position_ned
                yaw = self.dm.drones[name].attitude[2]
                self.drones[name].launch_pos = np.asarray([x, y, z, yaw])
            self.logger.info("Stage 1 complete!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)

    async def stage_2(self):
        try:
            self.logger.info(f"Starting stage 2 with {self.current_drone_list()}")
            await self.fly_stage(self.STAGE2_WP)
            self.logger.info("Stage 2 complete!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)

    async def stage_3(self):
        try:
            self.logger.info(f"Starting stage 3 with {self.current_drone_list()}")
            await self.fly_stage(self.STAGE3_WP)
            self.logger.info("Found body!")
            self.logger.info("Stage 3 complete!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)

    async def stage_4(self):
        try:
            self.logger.info(f"Starting stage 4 with {self.current_drone_list()}")
            await self.fly_stage(self.STAGE4_WP)
            self.logger.info("Stage 4 complete!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)

    async def stage_5(self):
        try:
            self.logger.info(f"Starting stage 5 with {self.current_drone_list()}")
            await self.fly_stage(self.STAGE5_WP)
            self.logger.info("Stage 5 complete!")
            # Fly back to launch pos
            for name in self.drones:
                drone = self.dm.drones[name]
                self.drones[name].waypoint = self.drones[name].launch_pos
                await drone.set_waypoint_ned(self.drones[name].launch_pos)
            # Check that all drones have reached the waypoint before proceeding
            while not all(self._are_at_coordinates()):
                await asyncio.sleep(0.1)
            await self.dm.land(self.drones.keys())
            self.logger.info("All drones landed!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)
