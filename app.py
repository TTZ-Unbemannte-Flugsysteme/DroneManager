import asyncio
import datetime
import os
import argparse
import shlex
import math
import numpy as np
from typing import Dict, List, Tuple
from collections import OrderedDict

from dronecontrol import DroneManager
from drones import Drone

import textual.css.query
from textual import on, events
from textual.app import App, Screen, Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import Footer, Header, Log, Static, RadioSet, RadioButton, ProgressBar
from textual.widget import Widget

from widgets import InputWithHistory, TextualLogHandler, DroneOverview
from drones import DroneMAVSDK, DummyMAVDrone
from betterparser import ArgParser, ArgumentParserError

import logging


common_formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")
pane_formatter = logging.Formatter('%(asctime)s %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")

DRONE_DICT = {
    "luke":   "udp://192.168.1.31:14561",
    "wedge":  "udp://192.168.1.32:14562",
    "derek":  "udp://192.168.1.33:14563",
    "tycho":  "udp://192.168.1.34:14564",
    "gavin":  "udp://192.168.1.35:14565",
    "corran": "udp://192.168.1.36:14566",
    "jaina":  "udp://192.168.1.37:14567"
}

UPDATE_RATE = 20  # How often the various screens update in Hz


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
        self.launch_pos = None  # Set above the actual flight altitude


class RedCross:
    # TODO: Checks to prevent flying in the wrong stage (only allow current stage +1 and reset to stage 0(landed) or
    #  stage 1(at start positions)

    STAGE2_WP = [
        WayPoint(3, 0, -2, 0, False, True),
        WayPoint(3, 0, -2, 90, False, False)
    ]

    STAGE3_WP = [
        WayPoint(3, 37, -2, 90, False, False),
        WayPoint(3, 37, -2, -90, False, True),
        WayPoint(27, 37, -2, -90, False, True),
        WayPoint(27, 37, -2, -90, False, False),
        WayPoint(27, 20, -2, -90, False, False),
    ]

    STAGE4_WP = [
        WayPoint(27, 20, -2, -90, False, True),
        WayPoint(31, 21, -2, -90, True, True),
    ]

    STAGE5_WP = [
        WayPoint(27, 20, -2, -90, False, True),
        WayPoint(3, 0, -2, -90, False, True),
        WayPoint(3, 0, -2, 90, False, True),
    ]

    def __init__(self, logger, dm: DroneManager):
        self.dm = dm
        self.drones: Dict[str, DemoDrone] = OrderedDict()
        self.logger = logger
        self.cur_stage = 0

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
        forward = 3
        right = 0
        altitude = -1 if waypoint.offset_altitudes else 0
        position_yaw_local_drones = [(waypoint.x + forward*i,
                                      waypoint.y + right*i,
                                      waypoint.z + altitude*i,
                                      waypoint.heading) for i in range(len(self.drones))]
        return position_yaw_local_drones

    def formation_circle(self, waypoint: WayPoint) -> List[Tuple[float, float, float, float]]:
        circle_radius = 6  # Adjust radius size as needed
        angle_offset = 2*math.pi/len(self.drones)
        altitude = -1 if waypoint.offset_altitudes else 0
        position_yaw_local_drones = [(waypoint.x + circle_radius * math.sin(i*angle_offset),
                                      waypoint.y + circle_radius * math.cos(i*angle_offset),
                                      waypoint.z + altitude*i,
                                      waypoint.heading) for i in range(len(self.drones))]
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
            at_pos = drone.is_at_pos(np.asarray(self.drones[name].waypoint[:3]))
            at_heading = drone.is_at_heading(self.drones[name].waypoint[3])
            if at_pos and at_heading:
                drones_at_coordinates.append(True)
            else:
                drones_at_coordinates.append(False)
        return drones_at_coordinates

    async def fly_stage(self, waypoints):
        for waypoint in waypoints:
            coordinates = self.formation(waypoint)

            # Spin first
            coros = []
            for i, name in enumerate(self.drones):
                drone = self.dm.drones[name]
                x, y, z = drone.position_ned
                new_heading = coordinates[i][3]
                coros.append(drone.yaw_to(x, y, z, new_heading, 20, 2))
            results = await asyncio.wait_for(asyncio.gather(*coros, return_exceptions=True), timeout=15)
            for i, name in enumerate(self.drones):
                result = results[i]
                if isinstance(result, Exception):
                    self.logger.error(f"Drone {name} couldn't complete this heading change!")
                    self.logger.debug(repr(result), exc_info=True)
                    #self.remove(name)

            # Make x,y,z moves
            for i, name in enumerate(self.drones):
                drone = self.dm.drones[name]
                self.drones[name].waypoint = coordinates[i]
                await drone.set_waypoint_ned(coordinates[i])
            # Check that all drones have reached the waypoint before proceeding
            while not any(self._are_at_coordinates()):
                await asyncio.sleep(0.1)
        return True

    async def stage_1(self):
        self.logger.info(f"Starting stage 1 with {self.current_drone_list()}")
        base_altitude = -2.0
        takeoff_offsets = [-i for i in range(len(self.drones))]
        try:
            await self.dm.arm(self.drones.keys(), schedule=True)
            takeoff_altitudes = [base_altitude + takeoff_offsets[i] for i, name in enumerate(self.drones)]
            self.logger.info(f"Takeoff altitudes: {takeoff_altitudes}")
            await asyncio.gather(*[self.dm.drones[name].takeoff(altitude=base_altitude + takeoff_offsets[i]) for i, name in enumerate(self.drones)])
            for name in self.drones:
                x, y, z = self.dm.drones[name].position_ned
                yaw = self.dm.drones[name].attitude[2]
                self.drones[name].waypoint = (x, y, z, yaw)
                self.drones[name].launch_pos = (x, y, z, yaw)
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
            while not any(self._are_at_coordinates()):
                await asyncio.sleep(0.1)
            await self.dm.land(self.drones.keys())
            self.logger.info("All drones landed!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)


class StatusScreen(Screen):

    CSS = """
ProgressBar {
    width: 25;
    height: 1;
    layout: horizontal;
}

Bar {
    width: 20;
    height: 1;
}
"""

    def __init__(self, drone_manager, logger, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dm: DroneManager = drone_manager
        self.cur_drone: Drone | None = None
        self.logger = logger
        asyncio.create_task(self._update_values())
        self.dm.add_connect_func(self._add_drone)
        self.dm.add_remove_func(self._remove_drone)

    async def _update_values(self):
        while True:
            # Update fields
            try:
                if self.cur_drone is not None:
                    self.query_one("#name", expect_type=Static).update(f"{self.cur_drone.name}")
                    self.query_one("#address", expect_type=Static).update(f"{self.cur_drone.drone_addr}")
                    self.query_one("#attitude", expect_type=Static).update(f"{self.cur_drone.attitude}")
                    self.query_one("#battery", expect_type=ProgressBar).update(progress=self.cur_drone.batteries[0].remaining)
                else:
                    self.query_one("#name", expect_type=Static).update("NAME: NO DRONE SELECTED")
                    self.query_one("#address", expect_type=Static).update("ADDRESS: NO DRONE SELECTED")
                    self.query_one("#attitude", expect_type=Static).update("ATTITUDE: NO DRONE SELECTED")
                    self.query_one("#battery", expect_type=ProgressBar).update(progress=0)
            except textual.app.NoMatches:
                pass
            except Exception as e:
                self.logger.error(f"Error updating values: {repr(e)}", exc_info=True)
            await asyncio.sleep(1/UPDATE_RATE)

    def compose(self):
        with Horizontal():
            with RadioSet(id="droneselector"):
                yield RadioButton("None", id="button_no_drone")
            with Vertical():
                yield Static(id="name", renderable="NAME: NO DRONE SELECTED")
                yield Static(id="address", renderable="ADDRESS: NO DRONE SELECTED")
                yield ProgressBar(id="battery", total=100, show_eta=False)
                yield Static(id="attitude", renderable="ATTITUDE: NO DRONE SELECTED")
        yield Footer()

    def on_radio_set_changed(self, event: RadioSet.Changed) -> None:
        if event.pressed.label.plain == "None":
            self.cur_drone = None
        else:
            try:
                self.cur_drone = self.dm.drones[event.pressed.label.plain]
            except KeyError:
                pass

    async def _add_drone(self, name, drone):
        try:
            self.logger.debug(f"Adding radio button for {name}")
            radio_selector = RadioButton(f"{name}", id=f"button_{name}")
            radio_field = self.query_one("#droneselector", expect_type=RadioSet)
            await radio_field.mount(radio_selector)
        except Exception as e:
            self.logger.error(f"{repr(e)}", exc_info=True)

    async def _remove_drone(self, name):
        try:
            if self.cur_drone is not None and self.cur_drone.name == name:
                # Have to change current drone to prevent stuff breaking
                self.cur_drone = None
            try:
                self.logger.debug(f"Removing radio button for {name}")
                await self.query_one(f"#button_{name}", expect_type=RadioButton).remove()
                # Move currently selected button after removal to prevent index errors
                selector = self.query_one(f"#droneselector", expect_type=RadioSet)
                selector.action_next_button()
            except textual.css.query.NoMatches:
                pass
        except Exception as e:
            self.logger.error(f"{repr(e)}", exc_info=True)
            raise


class CommandScreen(Screen):
    # TODO: Make the CSS better, change widths and whatever dynamically
    # TODO: Print a pretty usage/command overview thing somewhere.

    # How often the drone overview screen is updated.
    STATUS_REFRESH_RATE = 20

    CSS = """
.text {
    text-style: bold;
}

#status {
    height: 5fr;
}

#usage {
    height: 1fr;
}

#sidebar {
    width: 77;
}
"""

    def __init__(self, drone_manager, logger, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dm: DroneManager = drone_manager
        self.drone_widgets: Dict[str, Widget] = {}
        self.running_tasks = set()
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self._kill_counter = 0  # Require kill all to be entered twice
        self.logger = logger
        self.log_pane_handlers = {}
        self.dm.add_connect_func(self._add_drone_object)
        self.dm.add_remove_func(self._remove_drone_object)
        self.redcross = RedCross(self.logger, self.dm)

        self.parser = ArgParser(
            description="Interactive command line interface to connect and control multiple drones")
        subparsers = self.parser.add_subparsers(title="command",
                                                description="Command to execute.", dest="command")
        connect_parser = subparsers.add_parser("connect", help="Connect a drone")
        connect_parser.add_argument("drone", type=str, help="Name for the drone.")
        connect_parser.add_argument("drone_address", type=str, nargs='?',
                                    help="Connection string. Something like udp://:14540")
        connect_parser.add_argument("-sa", "--server_address", type=str, default=None,
                                    help="Address for the mavsdk server. If omitted, a server is started "
                                         "automatically. Use this only if you already have a server for this drone "
                                         "running (for example on another machine). Default None")
        connect_parser.add_argument("-sp", "--server_port", type=int, default=50051,
                                    help="Port for the mavsdk server. Default 50051.")
        connect_parser.add_argument("-t", "--timeout", type=float, default=5, required=False,
                                    help="Timeout in seconds for connection attempts.")
        arm_parser = subparsers.add_parser("arm", help="Arm the named drone(s).")
        arm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to arm")
        arm_parser.add_argument("-s", "--schedule", action="store_true", help="Queue this action instead of "
                                                                              "executing immediately.")

        disarm_parser = subparsers.add_parser("disarm", help="Disarm the named drone(s).")
        disarm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to disarm")
        disarm_parser.add_argument("-s", "--schedule", action="store_true", help="Queue this action instead of "
                                                                                 "executing immediately.")

        takeoff_parser = subparsers.add_parser("takeoff", help="Puts the drone(s) into takeoff mode.")
        takeoff_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to take off with.")
        takeoff_parser.add_argument("-s", "--schedule", action="store_true", help="Queue this action instead of "
                                                                                  "executing immediately.")

        offboard_parser = subparsers.add_parser("mode", help="Change the drone(s) flight mode")
        offboard_parser.add_argument("mode", type=str, help="Target flight mode. Must be one of {}.".format(
            self.dm.drone_class.VALID_FLIGHTMODES))
        offboard_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to change flight mode on.")
        #offboard_parser.add_argument("-s", "--schedule", action="store_true", help="Queue this action instead of "
        #                                                                           "executing immediately.")

        fly_to_parser = subparsers.add_parser("flyto", help="Send the drone to a local coordinate.")
        fly_to_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_parser.add_argument("x", type=float, help="Target x coordinate")
        fly_to_parser.add_argument("y", type=float, help="Target y coordinate")
        fly_to_parser.add_argument("z", type=float, help="Target z coordinate")
        fly_to_parser.add_argument("yaw", type=float, nargs="?", default=0.0, help="Target yaw in degrees. Default 0.")
        fly_to_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.5,
                                   help="Position tolerance")

        fly_to_gps_parser = subparsers.add_parser("flytogps", help="Send the drone to a GPS coordinate")
        fly_to_gps_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_gps_parser.add_argument("lat", type=float, help="Target latitude")
        fly_to_gps_parser.add_argument("long", type=float, help="Target longitude")
        fly_to_gps_parser.add_argument("alt", type=float, help="Target altitude (relative to takeoff)")
        fly_to_gps_parser.add_argument("yaw", type=float, nargs="?", default=0.0, help="Target yaw in degrees. "
                                                                                       "Default 0.")
        fly_to_gps_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.5,
                                       help="Position tolerance")

        fly_circle_parser = subparsers.add_parser("orbit", help="Fly in a circle, facing the center point")
        fly_circle_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_circle_parser.add_argument("radius", type=float, help="Radius of the circle")
        fly_circle_parser.add_argument("vel", type=float, help="Target velocity (negative for opposite direction)")
        fly_circle_parser.add_argument("center_lat", type=float, help="Latitude of the center of the circle")
        fly_circle_parser.add_argument("center_long", type=float, help="Longitude of the center of the circle")
        fly_circle_parser.add_argument("amsl", type=float, help="Altitude in terms of AMSL.")

        land_parser = subparsers.add_parser("land", help="Land the drone(s)")
        land_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to land")
        land_parser.add_argument("-s", "--schedule", action="store_true", help="Queue this action instead of "
                                                                               "executing immediately.")

        pause_parser = subparsers.add_parser("pause", help="Pause the drone(s) task execution")
        pause_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to pause")

        resume_parser = subparsers.add_parser("resume", help="Resume the drone(s) task execution")
        resume_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to resume")

        stop_parser = subparsers.add_parser("stop", help="Stops (i.e. lands) drones. If no drones are listed, "
                                                         "stops all of them and then exits the application")
        stop_parser.add_argument("drones", type=str, nargs="*", help="Drone(s) to stop.")

        kill_parser = subparsers.add_parser("kill", help="Kills (i.e. disarms and stops everything) drones. If no "
                                                         "drones are listed, kills all of them.")
        kill_parser.add_argument("drones", type=str, nargs="*", help="Drone(s) to kill.")

        ql_parser = subparsers.add_parser("qualify", help="Executes the 'qualify' function for the specified drones")
        ql_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to qualify.")
        ql_parser.add_argument("-a", "--altitude", type=float, required=False, default=2.0,
                               help="Altitude in meters above takeoff at which the course will be completed. "
                                    "Positive for up. Default 2m")

        rc_add_parser = subparsers.add_parser("rc-add", help="Add drone(s) to the redcross demo")
        rc_add_parser.add_argument("drones", type=str, nargs="+", help="Drones to add.")

        rc_remove_parser = subparsers.add_parser("rc-rm", help="Remove drone(s) from the redcross demo")
        rc_remove_parser.add_argument("drones", type=str, nargs="+", help="Drones to remove.")

        rc_stage_parser = subparsers.add_parser("rc-stage", help="Perform stage 1 with the current drones")
        rc_stage_parser.add_argument("stage", type=int, help="Which stage to execute. Must be consecutive to the previous stage")

    async def _add_drone_object(self, name, drone):
        output = self.query_one("#output", expect_type=Log)
        status_field = self.query_one("#status", expect_type=VerticalScroll)
        self.logger.debug(f"Adding log pane handlers to {name}")
        drone_handler = TextualLogHandler(output)
        drone_handler.setLevel(logging.INFO)
        drone_handler.setFormatter(pane_formatter)
        drone.add_handler(drone_handler)
        self.logger.debug(f"Adding overview widget for {name}")
        drone_status_widget = DroneOverview(drone, UPDATE_RATE)
        self.drone_widgets[name] = drone_status_widget
        await status_field.mount(drone_status_widget)

    async def _remove_drone_object(self, name):
        await self.drone_widgets[name].remove()

    @on(InputWithHistory.Submitted, "#cli")
    async def cli(self, message):
        value = message.value
        message.control.clear()
        tmp = None
        try:
            args = self.parser.parse_args(shlex.split(value))
        except ValueError as e:
            self.logger.warning(str(e))
            return
        except ArgumentParserError as e:
            self.logger.error(f"Exception parsing the argument: {repr(e)}", exc_info=True)
            return
        try:
            if args.command != "kill" or args.drones:
                self._kill_counter = 0

            if args.command == "connect":
                address = args.drone_address
                if args.drone in DRONE_DICT and not address:
                    address = DRONE_DICT[args.drone]
                elif not address:
                    address = "udp://:14540"
                tmp = asyncio.create_task(self.dm.connect_to_drone(args.drone, args.server_address, args.server_port,
                                                                   address, args.timeout))
            elif args.command == "arm":
                tmp = asyncio.create_task(self.dm.arm(args.drones, schedule=args.schedule))
            elif args.command == "disarm":
                tmp = asyncio.create_task(self.dm.disarm(args.drones, schedule=args.schedule))
            elif args.command == "takeoff":
                tmp = asyncio.create_task(self.dm.takeoff(args.drones, schedule=args.schedule))
            elif args.command == "mode":
                tmp = asyncio.create_task(self.dm.change_flightmode(args.drones, args.mode))
            elif args.command == "flyto":
                tmp = asyncio.create_task(self.dm.fly_to(args.drone, args.x, args.y, args.z, args.yaw, tol=args.tolerance))
            elif args.command == "flytogps":
                tmp = asyncio.create_task(self.dm.fly_to_gps(args.drone, args.lat, args.long, args.alt, args.yaw,
                                                          tol=args.tolerance))
            elif args.command == "orbit":
                tmp = asyncio.create_task(self.dm.orbit(args.drone, args.radius, args.vel, args.center_lat,
                                                        args.center_long, args.amsl))
            elif args.command == "land":
                tmp = asyncio.create_task(self.dm.land(args.drones, schedule=args.schedule))
            elif args.command == "pause":
                tmp = asyncio.create_task(self.dm.pause(args.drones))
            elif args.command == "resume":
                tmp = asyncio.create_task(self.dm.resume(args.drones))
            elif args.command == "stop":
                tmp = asyncio.create_task(self.action_stop(args.drones))
            elif args.command == "kill":
                if not args.drones:
                    if self._kill_counter:
                        tmp = asyncio.create_task(self.dm.kill(args.drones))
                    else:
                        self.logger.warning("Are you sure? Enter kill again")
                        self._kill_counter += 1
                else:
                    tmp = asyncio.create_task(self.dm.kill(args.drones))
            elif args.command == "qualify":
                self.qualify(args.drones, args.altitude)

            elif args.command == "rc-add":
                self.rc_add_drones(args.drones)
            elif args.command == "rc-rm":
                self.rc_remove_drones(args.drones)
            elif args.command == "rc-stage":
                tmp = asyncio.create_task(self.stages(args.stage))
            self.running_tasks.add(tmp)
        except Exception as e:
            self.logger.error(repr(e))

    def qualify(self, names, altitude=2.0):
        good_names = []
        for name in names:
            if name in self.dm.drones:
                good_names.append(name)
            else:
                self.logger.warning(f"No drone named {name}")
        for name in good_names:
            tmp = asyncio.create_task(self._qualify(name, altitude))

    async def _qualify(self, name, altitude):
        cur_pos = self.dm.drones[name].position_ned
        x, y, z = cur_pos
        try:
            drone = self.dm.drones[name]
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
            return
        try:
            await self.dm.arm([name])
            await asyncio.sleep(2)
            await self.dm.takeoff([name])
            await asyncio.sleep(2)
        ############
            self.logger.info("Big move forward 5m")
            await drone.set_waypoint_ned(np.asarray([5+x, y, -altitude, 0], dtype=float))
            await asyncio.sleep(7)
            await drone.set_waypoint_ned(np.asarray([0+x, y, -altitude, 0], dtype=float))
            await asyncio.sleep(9)
        #############
            await drone.set_waypoint_ned(np.asarray([5+x, y, -altitude, 0], dtype=float))
            await asyncio.sleep(7)
            self.logger.info("Turning, rate 10deg/2, 10Hz")
            await drone.spin_at_rate(10, 36, "cw")
            await asyncio.sleep(2)
            self.logger.info("Turning, rate 30deg/2, 10Hz")
            await drone.spin_at_rate(30, 12, "cw")
            await asyncio.sleep(2)
        #############
            await drone.set_waypoint_ned(np.asarray([0+x, y, -altitude, 0], dtype=float))
            await asyncio.sleep(7)
            await self.dm.land([name])
        except Exception as e:
            self.logger.error(f"{repr(e)}", exc_info=True)

    def rc_add_drones(self, names):
        for name in names:
            if name not in self.dm.drones:
                self.logger.warning(f"Can't add {name} to demo, no drone with that name!")
            else:
                self.redcross.add(name)
        self.logger.info(f"{self.redcross.current_drone_list()} currently taking part!")

    def rc_remove_drones(self, names):
        for name in names:
            if name not in self.dm.drones or name not in self.redcross.drones:
                self.logger.warning(f"Can't remove {name} to demo, no drone with that name!")
            else:
                self.redcross.remove(name)
        self.logger.info(f"{self.redcross.current_drone_list()} currently taking part!")

    async def stages(self, stage):
        if stage == 1:
            await self.redcross.stage_1()
        elif stage == 2:
            await self.redcross.stage_2()
        elif stage == 3:
            await self.redcross.stage_3()
        elif stage == 4:
            await self.redcross.stage_4()
        elif stage == 5:
            await self.redcross.stage_5()

    async def action_stop(self, names):
        stop_app = False
        if not names:
            stop_app = True
        results = await self.dm.action_stop(names)
        for i, result in enumerate(results):
            # If one of the drones encounters an excepton
            if isinstance(result, Exception):
                stop_app = False
        if stop_app:
            self.logger.info("All drones stopped, exiting...")
            await asyncio.sleep(2)  # Beauty pause
            self.app.exit()

    def _schedule_initialization_tasks(self):
        asyncio.create_task(self._logging_setup())

    async def _logging_setup(self):
        output = None
        while output is None:
            try:
                output = self.query_one("#output", expect_type=Log)
            except textual.css.query.NoMatches:
                await asyncio.sleep(0.1)
        handler = TextualLogHandler(output)
        handler.setLevel(logging.INFO)
        handler.setFormatter(pane_formatter)
        self.logger.addHandler(handler)
        self.dm.logger.addHandler(handler)

    def _on_mount(self, event: events.Mount) -> None:
        super()._on_mount(event)
        self.query_one("#output", expect_type=Log).can_focus = False

    def compose(self):
        status_string = ""
        status_string += "Drone Status\n" + DroneOverview.header_string()

        yield Header()
        yield Vertical(
            Horizontal(
                Log(id="output", classes="text"),
                Vertical(
                    VerticalScroll(
                        Static(id="status_header", renderable=status_string),
                        id="status", classes="text evenvert"),
                    Static(id="usage", classes="text evenvert", renderable=self.parser.format_help()),
                    id="sidebar",
                )
            ),
            InputWithHistory(placeholder="Command line", id="cli")
        )
        yield Footer()
        self._schedule_initialization_tasks()


class DroneApp(App):

    BINDINGS = {
        Binding("s", "cycle_control", "Swap Status/Control"),
    }

    def __init__(self, drone_manager: DroneManager, logger=None):
        self.drone_manager = drone_manager
        if logger is None:
            self.logger = logging.getLogger("App")
            self.logger.setLevel(logging.DEBUG)
            filename = f"app_{datetime.datetime.now()}"
            filename = filename.replace(":", "_").replace(".", "_") + ".log"
            logdir = os.path.abspath("./logs")
            os.makedirs(logdir, exist_ok=True)
            file_handler = logging.FileHandler(os.path.join(logdir, filename))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(common_formatter)
            self.logger.addHandler(file_handler)
        else:
            self.logger = logger
        self.command_screen: CommandScreen | None = None
        self.status_screen: StatusScreen | None = None
        super().__init__()

    def on_mount(self):
        screen = CommandScreen(drone_manager=self.drone_manager,
                               logger=self.logger,
                               name="control-screen")
        self.install_screen(screen, name=screen.name)
        self.add_mode("control", base_screen=screen)
        self.command_screen = screen
        status_screen = StatusScreen(drone_manager=self.drone_manager,
                                     logger=self.logger,
                                     name="status-screen")
        self.install_screen(status_screen, name=status_screen.name)
        self.add_mode("status", base_screen=status_screen)
        self.status_screen = status_screen
        self.switch_mode("status")
        self.switch_mode("control")

    def action_cycle_control(self):
        self.logger.debug("Switching between control and status screens")
        if self.current_mode == "control":
            self.logger.debug(f"Switching from control to status.")
            self.switch_mode("status")
        elif self.current_mode != "control":
            self.logger.debug("Switching to control")
            self.switch_mode("control")
        else:
            self.logger.debug("No valid target for switching")


if __name__ == "__main__":
    start_parser = argparse.ArgumentParser()
    start_parser.add_argument("-d", "--dummy", action="store_true", help="If set, use a dummy drone class")

    start_args = start_parser.parse_args()

    drone_type = DummyMAVDrone if start_args.dummy else DroneMAVSDK
    dm = DroneManager(drone_type)
    app = DroneApp(drone_manager=dm, logger=dm.logger)
    app.run()

    logging.shutdown()
