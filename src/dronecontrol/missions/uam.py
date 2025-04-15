import asyncio
import enum
import collections
import math
import time

import numpy as np

from dronecontrol.plugins.mission import Mission
from dronecontrol.utils import dist_ned
from dronecontrol.navigation.core import Waypoint, WayPointType

# TODO: Add fence to every drone (where has fence class gone?)
# TODO: Take circling function and put it into drone orbit function


class UAMStages(enum.Enum):
    Uninitialized = enum.auto()
    Start = enum.auto()
    SearchSingle = enum.auto()
    SearchGroup = enum.auto()
    POIFound = enum.auto()
    Observation = enum.auto()
    Return = enum.auto()


class FakeBattery:

    TIME_SCALE = 180

    def __init__(self):
        self.level = 1.0  # float from 0 to 1.0
        self.critical_level = 0.33

    @property
    def battery_low(self):
        return self.level <= self.critical_level


class UAMMission(Mission):

    def __init__(self, name, dm, logger):
        super().__init__(name, dm, logger)
        self.drones = collections.OrderedDict()

        mission_cli_commands = {
            "reset": self.reset,
            "set": self.set_start,
            "singlesearch": self.single_search,
            "rtb": self.rtb,
            "groupsearch": self.group_search,
        }
        self.cli_commands.update(mission_cli_commands)
        self.background_functions = [
            self._stage_managing_function(),
            self._battery_drainer(),
        ]

        # Static parameters for mission definition
        self.n_drones_required = 3
        self.current_stage = UAMStages.Uninitialized
        self.flight_area = [-3.75, 3.75, -1.75, 1.75, 2]
        self.search_space = [-3.5, 3.5, -1.5, 1.5]
        self.start_positions_y = [-1.5, 0, 1.5]  # TODO: Make this dynamic somehow
        self.start_position_x = 3.5
        self.start_yaw = 180
        self.flight_altitude = 3  # in meters, positive for up
        self.poi_position = [-2, -0.25, -self.flight_altitude]  # in NED
        self.update_rate = 5  # Mission state is checked and progressed this often per second.

        # SingleSearch Parameters
        self.single_search_forward_leg = 1  # in meters

        # Observation Stage Parameters
        self.observation_diameter = 5  # in meters
        self.circling_speed = 0.5  # in m/s
        self._circling_speed_angular = math.pi * 2 / (math.pi*self.observation_diameter / self.circling_speed)
        self._swap_altitude = 2  # The height that the drones will do the swap at.

        # Dynamic attributes, each stage must appropriate set these
        self.drone_tasks = set()  # Keeps track of all the stage functions
        self.found_poi = None  # The drone that found the POI, if one found the POI
        self.observing_drone = None  # The drone that is currently observing the POI

        self.flying_drones = set()  # List of names
        self.batteries: dict[str, FakeBattery] = {}  # Dict with batteries for each drone. Will be drained while drones are flying

        # TODO: Implement this: self.TESTING = False  # If true, none of the stage transitions happen automatically.
        # TODO: Figure out some way of asking for "confirmation" of a step
        # TODO: Reset battery somehow?

    async def _stage_managing_function(self):
        # Check the current stage every so often and cancel functions/start new functions when the stage changes
        # We manage the behaviour of the drones through these stages
        old_stage = self.current_stage
        n_steps = 0
        while True:
            try:
                if self.current_stage is not old_stage:
                    # Cancel all current drone tasks
                    for task in self.drone_tasks:
                        if isinstance(task, asyncio.Task):
                            task.cancel()
                    # Tell the drones to maintain their current positions, unless we are in Start or Uninitialized
                    if self.current_stage is not UAMStages.Start or self.current_stage is not UAMStages.Uninitialized:
                        for drone in self.drones:
                            self.dm.drones[drone].fly_to(local=self.dm.drones[drone].position_ned)
                    # Start new drone tasks for the new stage
                    new_tasks = set()
                    if self.current_stage is UAMStages.SearchSingle:
                        new_tasks.add(asyncio.create_task(self._single_search()))
                        new_tasks.add(asyncio.create_task(self._check_found_poi()))
                    elif self.current_stage is UAMStages.POIFound:
                        new_tasks.add(asyncio.create_task(self.poi_found()))
                    elif self.current_stage is UAMStages.Observation:
                        new_tasks.add(asyncio.create_task(self.observation()))
                    elif self.current_stage is UAMStages.Return:
                        new_tasks.add(asyncio.create_task(self._rtb()))
                    elif self.current_stage is UAMStages.SearchGroup:
                        new_tasks.add(asyncio.create_task(self._group_search()))
                        new_tasks.add(asyncio.create_task(self._check_found_poi()))
                    self.drone_tasks.update(new_tasks)
                    self._running_tasks.update(new_tasks)
                    old_stage = self.current_stage
                n_steps += 1
                if n_steps % 10 == 0:
                    self.logger.info(f"Flying drones: {self.flying_drones}")
                    self.logger.info(f"Battery levels: {[self.batteries[drone].level for drone in self.drones]}")
                await asyncio.sleep(1/self.update_rate)
            except asyncio.CancelledError:
                self.logger.debug("Cancelling stage-managing function")
                return
            except Exception as e:
                self.logger.error("Stage managing function encountered an exception!")
                self.logger.debug(repr(e), exc_info=True)

    async def _battery_drainer(self):
        while True:
            try:
                await asyncio.sleep(1/self.update_rate)
                for drone_name in self.drones:
                    if drone_name in self.flying_drones:
                        battery = self.batteries[drone_name]
                        battery.level -= 1 / FakeBattery.TIME_SCALE / self.update_rate
                        if battery.level < battery.critical_level / 2:
                            battery.level = battery.critical_level / 2
                    else:
                        self.batteries[drone_name].level = 1.0
            except asyncio.CancelledError:
                return
            except Exception as e:
                self.logger.error("Encountered an exception in the battery managing function!")
                self.logger.debug(repr(e), exc_info=True)

    async def _check_found_poi(self):
        # For each drone, check if we are within the search radius of the POI
        # TODO: Implement a better check, instead of distance to POI, compute vision cone and check POI in it
        while True:
            try:
                await asyncio.sleep(1 / self.update_rate)
                for drone in self.drones:
                    current_pos = self.dm.drones[drone].position_ned
                    if dist_ned(current_pos, self.poi_position) < 1:
                        self.found_poi = drone
                        self.current_stage = UAMStages.POIFound
                        self.logger.info(f"{drone} found POI!")
            except asyncio.CancelledError:
                self.logger.debug("Cancelling poi check function")
            except Exception as e:
                self.logger.error(f"The POI checker function encountered an exception!")
                self.logger.debug(repr(e), exc_info=True)
            await asyncio.sleep(1/self.update_rate)

    async def single_search(self):
        self.logger.info("Starting search with single drone!")
        self.current_stage = UAMStages.SearchSingle

    async def _single_search(self):
        # Do the search pattern ( Stage SingleSearch. If we find POI -> Stage POIFound, else RTB)
        flying_drone = list(self.drones.keys())[0]
        assert self.ready()
        try:
            # Do the thing
            # Arm, takeoff
            armed = await self.dm.arm([flying_drone])
            takeoff = False
            if armed:
                takeoff = await self.dm.takeoff([flying_drone], altitude=self.flight_altitude)
                self.flying_drones.add(flying_drone)
            if not armed or not takeoff:
                self.logger.warning("Couldn't start the single search pattern due to denied arming or takeoff!")
                return False

            # Do the pattern
            for repeat in range(1, math.floor((self.search_space[1] - self.search_space[0]) / self.single_search_forward_leg)):
                x_pos_new = self.search_space[1] - repeat*self.single_search_forward_leg
                if repeat % 2 == 0:
                    y_pos_current = self.search_space[3]
                    y_pos_new = self.search_space[2]
                    side_yaw = -90
                else:
                    y_pos_current = self.search_space[2]
                    y_pos_new = self.search_space[3]
                    side_yaw = 90
                # fly forward
                await self.dm.fly_to(flying_drone,
                                     local=[x_pos_new, y_pos_current, -self.flight_altitude],
                                     yaw=-180, tol=0.25, schedule=False)
                # switch sides
                await self.dm.fly_to(flying_drone,
                                     local=[x_pos_new, y_pos_new, -self.flight_altitude],
                                     yaw=side_yaw, tol=0.25, schedule=False)
            await asyncio.sleep(3)
            # Reached end of search pattern without finding POI, RTB
            self.current_stage = UAMStages.Return
        except asyncio.CancelledError:
            self.logger.debug("Cancelling single search function")
        except Exception as e:
            self.logger.error("Encountered an exception in the single search function!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def poi_found(self):
        # Send the drone that found the POI to a position on the circle, send everybody else back.
        # Start in POI, end in observation
        # Theta is the angle on the circle, with theta = 0 at y = 0, x = observation radius / 2
        # We approach the circle by flying directly to the closest point on it, while pointing at it.
        poi_tasks = []
        try:
            for i, drone in enumerate(self.flying_drones):
                if drone == self.found_poi:
                    poi_tasks.append(asyncio.create_task(self._poi_task(drone)))
                else:
                    poi_tasks.append(asyncio.create_task(self.dm.fly_to(drone,
                                                                        local=[self.start_position_x,
                                                                               self.start_positions_y[i],
                                                                               -self.flight_altitude],
                                                                        yaw=self.start_yaw,
                                                                        tol=0.25,
                                                                        schedule=False)))
                    poi_tasks.append(asyncio.create_task(self.dm.land([drone], schedule=True)))
            await asyncio.gather(*poi_tasks)
            self.flying_drones = {self.found_poi}
            self.observing_drone = self.found_poi
            self.found_poi = None
            self.current_stage = UAMStages.Observation
        except asyncio.CancelledError:
            self.logger.debug("Cancelling poi_found function")
        except Exception as e:
            self.logger.error("An exception occurred in the POI function!")
            self.logger.debug(repr(e), exc_info=True)

    async def group_search(self):
        self.logger.info("Starting search with group of drones!")
        self.current_stage = UAMStages.SearchGroup

    async def _group_search(self):
        # Do the search pattern (Stage group stage during, then go to POIFound)
        assert self.ready()
        try:
            armed = await self.dm.arm(self.drones)
            takeoff = [False]
            if all(armed):
                self.flying_drones.update(self.drones.keys())
                takeoff = await self.dm.takeoff(self.drones, altitude=self.flight_altitude)
            if not (all(armed) and all(takeoff)):
                self.logger.warning("Couldn't arm or takeoff with all drones! Stopping mission")
                tasks = []
                for drone in self.drones:
                    tasks.append(asyncio.create_task(self.dm.land([drone], schedule=True)))
                await asyncio.gather(*tasks)
                return False
            # Do the pattern (i.e. just fly forward)
            end_positions = []
            for i, _ in enumerate(self.drones):
                end_positions.append([self.search_space[0], self.start_positions_y[i], -self.flight_altitude])

            coros = [self.dm.fly_to(drone, local=end_positions[i], yaw=180) for i, drone in enumerate(self.drones)]
            await asyncio.gather(*coros)
            await asyncio.sleep(3)
            # Reached end of pattern without finding POI
            self.current_stage = UAMStages.Return
        except Exception as e:
            self.logger.error("Encountered an exception in the group search function!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def observation(self):
        # Do the observation stage, with battery swap and everything (Stage observation throughout).
        # There should already be one drone observing the POI, saved in self.observing_drone
        assert self.ready()
        self.logger.info("Starting long-term observation phase!")
        try:
            # Start circling the observation drone
            observe_task = asyncio.create_task(self._observation_circling(self.observing_drone))
            self.drone_tasks.add(observe_task)
            while True:
                if self.batteries[self.observing_drone].battery_low:
                    self.logger.info("Observing drone battery going low!")
                    # Pick the drone back at base with the highest battery
                    max_battery = -1
                    drone_with_max_battery = None
                    for drone in self.drones:
                        if drone == self.observing_drone:
                            continue
                        if self.batteries[drone].level > max_battery:
                            max_battery = self.batteries[drone].level
                            drone_with_max_battery = drone
                    swap_drone = drone_with_max_battery
                    # Launch the new drone
                    await self.dm.arm([swap_drone])
                    # TODO: What to do if the drone doesn't arm?
                    await self.dm.takeoff([swap_drone], altitude=self._swap_altitude)
                    self.flying_drones.add(swap_drone)

                    swap_tasks = []
                    # Stop the circling of the observing drone, send the new drone to the point opposite the circle.
                    observe_task.cancel()
                    #observ_stop = asyncio.create_task(self.dm.fly_to(self.observing_drone, local=observ_cur_position))
                    #swap_tasks.append(observ_stop)
                    # Determine position for new drone
                    dx = self.poi_position[0] - self.dm.drones[self.observing_drone].position_ned[0]
                    dy = self.poi_position[1] - self.dm.drones[self.observing_drone].position_ned[1]
                    new_theta = (math.atan2(dy, dx) + math.pi * 3) % (math.pi * 2) # Angle of current drone + pi
                    target_yaw = ((new_theta + math.pi) % (math.pi * 2)) * 180 / math.pi  # Opposite angle in degrees
                    x_pos = math.cos(new_theta) * self.observation_diameter / 2 + self.poi_position[0]
                    y_pos = math.sin(new_theta) * self.observation_diameter / 2 + self.poi_position[1]
                    await self.dm.fly_to(swap_drone, local=[x_pos, y_pos, -self._swap_altitude], yaw=target_yaw,
                                         schedule=False, tol=0.25)
                    # New drone as arrived, send old drone back to base
                    for i, drone in enumerate(self.drones):
                        if drone == self.observing_drone:
                            observ_cur_position = self.dm.drones[self.observing_drone].position_ned
                            observ_cur_position[2] = -self._swap_altitude
                            height_change_task = asyncio.create_task(self.dm.fly_to(self.observing_drone,
                                                                                    local=observ_cur_position,
                                                                                    schedule=True))
                            rtb_task = asyncio.create_task(self.dm.fly_to(self.observing_drone,
                                                                          local=[self.start_position_x,
                                                                                 self.start_positions_y[i],
                                                                                 -self._swap_altitude],
                                                                          yaw=self.start_yaw, tol=0.25,
                                                                          schedule=True))
                            land_task = asyncio.create_task(self.dm.land([self.observing_drone], schedule=True))
                            swap_tasks.append(height_change_task)
                            swap_tasks.append(rtb_task)
                            swap_tasks.append(land_task)
                    # Once old drone is back at base, update attributes, start circling new drone
                    await asyncio.gather(*swap_tasks)
                    self.flying_drones.remove(self.observing_drone)
                    self.observing_drone = swap_drone
                    observe_task = asyncio.create_task(self._observation_circling(self.observing_drone))
                    self.drone_tasks.add(observe_task)

                await asyncio.sleep(1/self.update_rate)
        except asyncio.CancelledError:
            self.logger.debug("Cancelling observation function")
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def rtb(self):
        self.logger.info("Returning to base!")
        self.current_stage = UAMStages.Return

    async def _rtb(self):
        """ Return to base stage.

        Returns all drones still out in the field back to their start positions"""
        assert self.ready()
        try:
            # Do the thing
            tasks = []
            for i, drone in enumerate(self.drones):
                if drone in self.flying_drones:
                    tasks.append(asyncio.create_task(self.dm.fly_to(drone,
                                                                    local=[self.start_position_x,
                                                                           self.start_positions_y[i],
                                                                           -self.flight_altitude],
                                                                    yaw=self.start_yaw, tol=0.25, schedule=True)))
                    tasks.append(asyncio.create_task(self.dm.land([drone], schedule=True)))
            await asyncio.gather(*tasks)
            self.flying_drones = set()
            self.current_stage = UAMStages.Start
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def _poi_task(self, drone):
        # Fly to POI and start circling
        # Determine vector from POI center to drone, send drone to that point, wait short beauty pause,
        # then circle.
        dx = self.poi_position[0] - self.dm.drones[drone].position_ned[0]
        dy = self.poi_position[1] - self.dm.drones[drone].position_ned[1]
        theta = (math.atan2(dy, dx) + math.pi * 2) % (math.pi * 2)  # x-axis is forward, y-axis is right
        target_yaw = ((theta + math.pi) % (math.pi * 2)) * 180 / math.pi  # Opposite angle in degrees
        x_pos = math.cos(theta) * self.observation_diameter / 2 + self.poi_position[0]
        y_pos = math.sin(theta) * self.observation_diameter / 2 + self.poi_position[1]
        await self.dm.fly_to(drone, local=[x_pos, y_pos, -self.flight_altitude], yaw=target_yaw, schedule=False,
                             tol=0.25)
        # Beauty Pause
        await asyncio.sleep(1)
        # Start circling: Have to add this to drone_tasks, so it gets cancelled and replaced with the proper obs task
        circle_task = self._observation_circling(drone)
        self.drone_tasks.add(asyncio.create_task(circle_task))

    async def _observation_circling(self, drone):
        # Slowly fly in a circle, pointing inwards.
        # Theta is the angle on the circle, with theta = 0 at y = 0
        # We approach the circle by flying directly to the closest point on it, while pointing at it.
        try:
            start_time = time.time()
            dx = self.poi_position[0] - self.dm.drones[drone].position_ned[0]
            dy = self.poi_position[1] - self.dm.drones[drone].position_ned[1]
            start_theta = (math.atan2(dy, dx) + math.pi * 2) % (math.pi * 2)  # Do this in 0-2pi for easy modulo math
            self.dm.drones[drone].trajectory_follower.deactivate()
            target_pos = np.zeros((3,))
            target_pos[2] = -self.flight_altitude
            while True:
                target_theta = (start_theta + self._circling_speed_angular * (time.time() - start_time)) % (math.pi * 2)
                target_pos[0] = math.cos(target_theta) * self.observation_diameter / 2 + self.poi_position[0]
                target_pos[1] = math.sin(target_theta) * self.observation_diameter / 2 + self.poi_position[1]
                target_yaw = ((target_theta + math.pi) % (math.pi * 2)) * 180 / math.pi  # Opposite angle in degrees
                # Have to convert target yaw from 0-360 to -180-180
                if target_yaw > 180:
                    target_yaw = target_yaw - 360
                waypoint = Waypoint(WayPointType.POS_NED, pos=target_pos, yaw=target_yaw)
                await self.dm.drones[drone].set_setpoint(waypoint)
                #tmp = asyncio.create_task(self.dm.fly_to(drone, local=[x_pos, y_pos, -self.flight_altitude],
                #                                         yaw=target_yaw, schedule=False, tol=0.25))
                await asyncio.sleep(1/self.update_rate)
        except asyncio.CancelledError:
                self.logger.debug("Cancelling observation function")
        except Exception as e:
            self.logger.error("Exception in circling function!")
            self.logger.debug(repr(e), exc_info=True)

    async def reset(self):
        self.logger.info("Resetting drones to start positions.")
        for task in self.drone_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()
        self.current_stage = UAMStages.Uninitialized
        #if len(self.drones) != self.n_drones_required:
        #    self.logger.info("Not enough drones to start mission!")
        #    return False
        if not len(self.drones) > 0:
            self.logger.warning("Can't fly a mission without any drones!")
            return False
        # Land all drones in case there are any in the air
        self.logger.info("Landing all drones")
        await asyncio.gather(*[self.dm.land([drone]) for drone in self.drones])
        self.logger.info("Flying all drones to start positions")
        for i, drone in enumerate(self.drones):
            await self.dm.arm([drone])
            await self.dm.takeoff([drone], altitude=self.flight_altitude)
            await self.dm.fly_to(drone, local=[self.start_position_x, self.start_positions_y[i], -self.flight_altitude],
                                 yaw=self.start_yaw, tol=0.25, schedule=True)
            await self.dm.land([drone])
        self.current_stage = UAMStages.Start

    async def set_start(self):
        """ Set the current stage to the start stage.

        This should be called/used when the drones are all setup at their starting positions already."""
        self.logger.info("Set to Start stage!")
        self.current_stage = UAMStages.Start

    async def status(self):
        self.logger.info(f"Mission {self.PREFIX} of type {self.__class__.__name__}: In stage {self.current_stage.name} "
                         f"with drones {list(self.drones.keys())}. Ready: {self.ready()}")

    def ready(self):
        drones_ready = all([self.mission_ready(drone) for drone in self.drones])
        return drones_ready and self.current_stage is not UAMStages.Uninitialized and len(self.drones) > 0

    async def add_drones(self, names: list[str]):
        self.logger.info(f"Adding drones {names} to mission!")
        self.current_stage = UAMStages.Uninitialized
        if len(self.drones) + len(names) > self.n_drones_required:
            self.logger.warning(f"Can't add this many drones to this kind of mission! These missions require "
                                f"exactly {self.n_drones_required} drones")
            return False
        for name in names:
            try:
                self.drones[name] = self.dm.drones[name]
                self.batteries[name] = FakeBattery()
            except KeyError:
                self.logger.error(f"No drone named {name}")

    async def remove_drones(self, names: list[str]):
        for name in names:
            try:
                self.drones.pop(name)
            except KeyError:
                self.logger.warning(f"No drone named {name}")

    def mission_ready(self, drone):
        # Check connection
        # TODO: What else should we check? Depends on stage...
        # Can't safely disarm in offboard due to landing issue, but do we want to stay armed whole demo?
        # Can't rearm
        return self.dm.drones[drone].is_connected
