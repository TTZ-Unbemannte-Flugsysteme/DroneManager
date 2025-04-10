import asyncio
import enum
import collections
import math

from dronecontrol.plugins.mission import Mission

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
        self.n_drones_required = 3
        self.current_stage = UAMStages.Uninitialized
        self.flight_area = [-3.75, 3.75, -1.75, 1.75, 2]

        self.start_positions = [-1.5, 0, 1.5]
        self.observe_center = (-3.5, -0.5)
        self.flight_altitude = 1  # in meters, positive for up

        self.search_space = [-3.5, 3.5, -1.5, 1.5]
        self.single_search_forward_leg = 1

    async def reset(self):
        self.current_stage = UAMStages.Uninitialized
        if len(self.drones) != self.n_drones_required:
            self.logger.info("Not enough drones to start mission!")
            return False
        # Land all drones in case there are any in the air
        await asyncio.gather(*[self.dm.land([drone]) for drone in self.drones])
        for i, drone in self.drones:
            await self.dm.arm([drone])
            await self.dm.takeoff([drone], altitude=1)
            await self.dm.fly_to(drone, local=[self.search_space[1], self.start_positions[i], -self.flight_altitude],
                                 yaw=0, tol=0.25, schedule=True)
            await self.dm.land([drone])
        self.current_stage = UAMStages.Start

    async def single_search(self):
        # Do the search pattern (Stage single search during, then return to start)
        assert self.ready()
        assert self.current_stage is UAMStages.Start
        self.current_stage = UAMStages.SearchSingle
        try:
            flying_drone = self.drones[0]
            # Do the thing
            # Arm, takeoff and fly half a meter forward
            await self.dm.arm([flying_drone])
            await self.dm.takeoff([flying_drone])

            # Do the pattern
            for repeat in range(1, math.floor(self.search_space[1] - self.search_space[0] / self.single_search_forward_leg)):
                x_pos_new = self.search_space[0] + repeat*self.single_search_forward_leg
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
            await asyncio.sleep(5)
            # RTB
            await self.dm.fly_to(flying_drone, 0, 0, -2, 0, tol=0.25, schedule=True)
            await self.dm.land([flying_drone])

            self.current_stage = UAMStages.Start
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def group_search(self):
        # TODO: All of it
        # Do the search pattern (Stage group stage during, then go to POIFound)
        assert self.ready()
        assert self.current_stage is UAMStages.Start
        self.current_stage = UAMStages.SearchGroup
        try:
            # Do the thing
            self.current_stage = UAMStages.POIFound
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    async def observation(self):
        # TODO: All of it
        # Do the observation stage, with battery swap and everything (Stage observation throughout). If there are more
        # drones at the observation point then necessary, this should send them back
        # This function should check if we are still in the observation state and return when we change state
        assert self.ready()
        assert self.current_stage is UAMStages.POIFound
        self.current_stage = UAMStages.Observation
        try:
            # Do the thing
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

    async def autonomous_search(self):
        # Do group search and automatically go into observation, but rtb still manually
        assert self.ready()
        assert self.current_stage is UAMStages.Start
        try:
            await self.group_search()
            await self.observation()
        except Exception as e:
            self.logger.error("Encountered an exception!")
            self.logger.debug(repr(e), exc_info=True)
            self.current_stage = UAMStages.Uninitialized

    def status(self):
        self.logger.info(f"Mission {self.PREFIX} of type {self.__class__.__name__}: In stage {self.current_stage.name} "
                         f"with drones {self.drones}. Ready: {self.ready()}")

    def ready(self):
        return len(self.drones) == self.n_drones_required and self.current_stage is not UAMStages.Uninitialized

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
