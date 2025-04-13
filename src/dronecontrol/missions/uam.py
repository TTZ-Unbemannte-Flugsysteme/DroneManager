import asyncio
import enum
import collections
import math
import time

from dronecontrol.plugins.mission import Mission
from dronecontrol.utils import dist_ned
from dronecontrol.navigation.core import Waypoint, WayPointType

# TODO: Mission scripting (take from battery_swap.py)
# TODO: Ensure mission plan matches flight area
# TODO: Add fence to every drone (where has fence class gone?)


class UAMStages(enum.Enum):
    Uninitialized: enum.auto()
    Start: enum.auto()
    SearchSingle: enum.auto()
    SearchGroup: enum.auto()
    POIFound: enum.auto()
    Observation: enum.auto()
    Return: enum.auto()


class UAMMission(Mission):

    def __init__(self, name, dm, logger):
        super().__init__(name, dm, logger)
        self.drones = collections.OrderedDict()

        mission_cli_commands = {
            "reset": self.reset,
            # TODO
        }
        self.cli_commands.update(mission_cli_commands)
        self.background_functions = [
            self._check_found_poi(),
            self._stage_managing_function()
        ]

        # Static parameters for mission definition
        self.n_drones_required = 3
        self.current_stage = UAMStages.Uninitialized
        self.flight_area = [-3.75, 3.75, -1.75, 1.75, 2]
        self.search_space = [-3.5, 3.5, -1.5, 1.5]
        self.start_positions_y = [-1.5, 0, 1.5]  # TODO: Make this dynamic somehow
        self.start_position_x = 3.5
        self.start_yaw = 180
        self.poi_position = [-2, -0.5]
        self.flight_altitude = 1  # in meters, positive for up
        self.update_rate = 5  # Mission state is checked and progressed this often per second.

        # SingleSearch Parameters
        self.single_search_forward_leg = 1  # in meters

        # Observation Stage Parameters
        self.observation_diameter = 1  # in meters
        self.circling_speed = 0.2  # in m/s
        self._circling_speed_angular = math.pi * 2 / (math.pi*self.observation_diameter / self.circling_speed)

        # Dynamic attributes, each stage must appropriate set these
        self.drone_tasks = set()  # Keeps track of all the stage functions
        self.found_poi = None  # The drone that found the POI, if one found the POI
        self.observing_drone = None  # The drone that is currently observing the POI

        # TODO: Implement this: self.TESTING = False  # If true, none of the stage transitions happen automatically.
        # TODO: Figure out some way of asking for "confirmation" of a step

    async def _stage_managing_function(self):
        # Check the current stage every so often and cancel functions/start new functions when the stage changes
        # We manage the behaviour of the drones through these stages
        old_stage = self.current_stage
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
                    if self.current_stage is UAMStages.SearchSingle:
                        self.drone_tasks.add(asyncio.create_task(self.single_search()))
                    elif self.current_stage is UAMStages.POIFound:
                        self.drone_tasks.add(asyncio.create_task(self.poi_found()))
                    elif self.current_stage is UAMStages.Observation:
                        self.drone_tasks.add(asyncio.create_task(self.observation()))
                    elif self.current_stage is UAMStages.Return:
                        self.drone_tasks.add(asyncio.create_task(self.rtb()))
                    elif self.current_stage is UAMStages.SearchGroup:
                        self.drone_tasks.add(asyncio.create_task(self.group_search()))
                    old_stage = self.current_stage
                await asyncio.sleep(1/self.update_rate)
            except asyncio.CancelledError:
                self.logger.debug("Cancelling stage-managing function")
            except Exception as e:
                self.logger.error("Stage managing function encountered an exception!")
                self.logger.debug(repr(e), exc_info=True)

    async def _check_found_poi(self):
        # For each drone, check if we are within the search radius of the POI
        # TODO: Better check: Not just distance but also facing of the drone. Optimally, at flight altitude
        while self.current_stage is UAMStages.SearchSingle or self.current_stage is UAMStages.SearchGroup:
            try:
                for drone in self.drones:
                    current_pos = self.dm.drones[drone].position_ned
                    if dist_ned(current_pos, self.poi_position) < 0.75:
                        self.found_poi = drone
                        self.current_stage = UAMStages.POIFound
            except asyncio.CancelledError:
                self.logger.debug("Cancelling poi check function")
            except Exception as e:
                self.logger.error(f"The POI checker function encountered an exception!")
                self.logger.debug(repr(e), exc_info=True)

    async def single_search(self):
        # Do the search pattern ( Stage SingleSearch. If we find POI -> Stage POIFound, else RTB)
        flying_drone = self.drones[0]
        assert self.ready()
        assert self.current_stage is UAMStages.Start
        self.current_stage = UAMStages.SearchSingle
        try:
            # Do the thing
            # Arm, takeoff
            await self.dm.arm([flying_drone])
            await self.dm.takeoff([flying_drone])

            # Do the pattern
            for repeat in range(1, math.floor((self.search_space[1] - self.search_space[0]) / self.single_search_forward_leg)):
                x_pos_new = self.search_space[0] - repeat*self.single_search_forward_leg
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
                                     yaw=-180, tol=0.25, schedule=True)
                # switch sides
                await self.dm.fly_to(flying_drone,
                                     local=[x_pos_new, y_pos_new, -self.flight_altitude],
                                     yaw=side_yaw, tol=0.25, schedule=True)
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
            for i, drone in enumerate(self.drones):
                if drone == self.found_poi:
                    poi_tasks.append(asyncio.create_task(self._poi_task(drone)))
                else:
                    poi_tasks.append(asyncio.create_task(self.dm.fly_to(drone,
                                                                        local=[self.start_position_x,
                                                                               self.start_positions_y[i],
                                                                               -self.flight_altitude],
                                                                        yaw=self.start_yaw,
                                                                        tol=0.25,
                                                                        schedule=True)))
                    poi_tasks.append(asyncio.create_task(self.dm.land([drone], schedule=True)))
            await asyncio.gather(*poi_tasks)
            self.observing_drone = self.found_poi
            self.found_poi = None
            self.current_stage = UAMStages.Observation
        except asyncio.CancelledError:
            self.logger.debug("Cancelling poi_found function")
        except Exception as e:
            self.logger.error("An exception occurred in the POI function!")
            self.logger.debug(repr(e), exc_info=True)

    async def group_search(self):
        # Do the search pattern (Stage group stage during, then go to POIFound)
        assert self.ready()
        assert self.current_stage is UAMStages.Start
        self.current_stage = UAMStages.SearchGroup
        try:
            await self.dm.arm(self.drones)
            await self.dm.takeoff(self.drones)
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
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def observation(self):
        # Do the observation stage, with battery swap and everything (Stage observation throughout).
        # There should already be one drone observing the POI, saved in self.observing_drone
        assert self.ready()
        assert self.current_stage is UAMStages.POIFound
        self.current_stage = UAMStages.Observation
        try:
            # Start circling the observation drone
            observe_task = asyncio.create_task(self._observation_circling(self.observing_drone))
            self.drone_tasks.add(observe_task)
            while True:
                # TODO: Do the battery swap thing
                pass
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def rtb(self):
        # TODO: All of it
        # Return to base
        assert self.ready()
        assert self.current_stage is UAMStages.Observation
        self.current_stage = UAMStages.Return
        try:
            # Do the thing
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
        await self.dm.fly_to(drone, local=[x_pos, y_pos, self.flight_altitude], yaw=target_yaw, schedule=False,
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
        start_time = time.time()
        dx = self.poi_position[0] - self.dm.drones[drone].position_ned[0]
        dy = self.poi_position[1] - self.dm.drones[drone].position_ned[1]
        start_theta = (math.atan2(dy, dx) + math.pi * 2) % (math.pi * 2)  # Do this in 0-2pi for easy modulo math
        while True:
            target_theta = start_theta + self._circling_speed_angular * (time.time() - start_time)
            x_pos = math.cos(target_theta) * self.observation_diameter / 2 + self.poi_position[0]
            y_pos = math.sin(target_theta) * self.observation_diameter / 2 + self.poi_position[1]
            target_yaw = ((target_theta + math.pi) % (math.pi * 2)) * 180 / math.pi  # Opposite angle in degrees
            await self.dm.fly_to(drone, local=[x_pos, y_pos, self.flight_altitude], yaw=target_yaw, schedule=False,
                                 tol=0.25)

    async def autonomous_search(self):
        # Do group search and automatically go into observation, but rtb still manually
        assert self.ready()
        assert self.current_stage is UAMStages.Start
        try:
            await self.group_search()
            await self.poi_found()
            await self.observation()
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def reset(self):
        for task in self.drone_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()
        self.current_stage = UAMStages.Uninitialized
        if len(self.drones) != self.n_drones_required:
            self.logger.info("Not enough drones to start mission!")
            return False
        # Land all drones in case there are any in the air
        await asyncio.gather(*[self.dm.land([drone]) for drone in self.drones])
        for i, drone in self.drones:
            await self.dm.arm([drone])
            await self.dm.takeoff([drone], altitude=1)
            await self.dm.fly_to(drone, local=[self.start_position_x, self.start_positions_y[i], -self.flight_altitude],
                                 yaw=self.start_yaw, tol=0.25, schedule=True)
            await self.dm.land([drone])
        self.current_stage = UAMStages.Start

    def status(self):
        self.logger.info(f"Mission {self.PREFIX} of type {self.__class__.__name__}: In stage {self.current_stage.name} "
                         f"with drones {self.drones}. Ready: {self.ready()}")

    def ready(self):
        drones_ready = all([self.mission_ready(drone) for drone in self.drones])
        return drones_ready and self.current_stage is not UAMStages.Uninitialized

    async def add_drones(self, names: list[str]):
        if len(self.drones) + len(names) > self.n_drones_required:
            self.logger.warning(f"Can't add this many drones to this kind of mission! These missions require "
                                f"exactly {self.n_drones_required} drones")
            return False
        for name in names:
            try:
                self.drones[name] = self.dm.drones[name]
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
        # TODO: What else should we check? Depends on stage, but generally, drones should be in offboard mode.
        # Can't safely disarm in offboard due to landing issue, but do we want to stay armed whole demo?
        # Can't rearm
        return self.dm.drones[drone].is_connected

    async def close(self):
        for task in self.drone_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()
        await super().close()
