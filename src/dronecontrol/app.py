import asyncio
import datetime
import os
import shlex

from dronecontrol.dronemanager import DroneManager
from dronecontrol.drone import Drone, DroneMAVSDK

import textual.css.query
from textual import on, events
from textual.app import App, Screen, Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import Footer, Header, Log, Static, RadioSet, RadioButton, ProgressBar
from textual.widget import Widget

from dronecontrol.widgets import InputWithHistory, TextualLogHandler, DroneOverview, ArgParser, ArgumentParserError


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
    "jaina":  "udp://192.168.1.37:14567",
    "kira":   "serial://COM5:56700",
}

UPDATE_RATE = 20  # How often the various screens update in Hz


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

    def __init__(self, dm, logger, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dm: DroneManager = dm
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
                    self.query_one("#battery", expect_type=ProgressBar).update(
                        progress=self.cur_drone.batteries[0].remaining)
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
        self.logger.debug(f"Adding radio button for {name}")
        radio_selector = RadioButton(f"{name}", id=f"button_{name}")
        radio_field = self.query_one("#droneselector", expect_type=RadioSet)
        await radio_field.mount(radio_selector)

    async def _remove_drone(self, name):
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


class CommandScreen(Screen):
    # TODO: Make the CSS better, change widths and whatever dynamically
    # TODO: Print a pretty usage/command overview thing somewhere.
    # TODO: Currently you can enter commands after going "exit", need to prevent that

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
    width: 97;
}
"""

    def __init__(self, dm, logger, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dm: DroneManager = dm
        self.drone_widgets: dict[str, Widget] = {}
        self.running_tasks = set()
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self._kill_counter = 0  # Require kill all to be entered twice
        self.logger = logger
        self.log_pane_handlers = {}
        self.dm.add_connect_func(self._add_drone_object)
        self.dm.add_remove_func(self._remove_drone_object)

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

        disconnect_parser = subparsers.add_parser("disconnect", help="Disconnect one or more drones.")
        disconnect_parser.add_argument("drones", type=str, nargs="+", help="Which drones to disconnect.")
        disconnect_parser.add_argument("-f", "--force", action="store_true",
                                       help="If this flag is set, ignore any potential checks and force the disconnect.")

        arm_parser = subparsers.add_parser("arm", help="Arm the named drone(s).")
        arm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to arm")
        arm_parser.add_argument("-s", "--schedule", action="store_true",
                                help="Queue this action instead of executing immediately.")

        disarm_parser = subparsers.add_parser("disarm", help="Disarm the named drone(s).")
        disarm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to disarm")
        disarm_parser.add_argument("-s", "--schedule", action="store_true",
                                   help="Queue this action instead of executing immediately.")

        takeoff_parser = subparsers.add_parser("takeoff", help="Puts the drone(s) into takeoff mode.")
        takeoff_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to take off with.")
        takeoff_parser.add_argument("-s", "--schedule", action="store_true",
                                    help="Queue this action instead of executing immediately.")

        offboard_parser = subparsers.add_parser("mode", help="Change the drone(s) flight mode")
        offboard_parser.add_argument("mode", type=str,
                                     help="Target flight mode. Must be one of {}.".format(self.dm.drone_class.VALID_FLIGHTMODES))
        offboard_parser.add_argument("drones", type=str, nargs="+",
                                     help="Drone(s) to change flight mode on.")

        fly_to_parser = subparsers.add_parser("flyto", help="Send the drone to a local coordinate.")
        fly_to_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_parser.add_argument("x", type=float, help="Target x coordinate")
        fly_to_parser.add_argument("y", type=float, help="Target y coordinate")
        fly_to_parser.add_argument("z", type=float, help="Target z coordinate")
        fly_to_parser.add_argument("yaw", type=float, nargs="?", default=0.0,
                                   help="Target yaw in degrees. Default 0.")
        fly_to_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.25,
                                   help="Position tolerance")
        fly_to_parser.add_argument("-s", "--schedule", action="store_true",
                                   help="Queue this action instead of executing immediately.")

        fly_to_gps_parser = subparsers.add_parser("flytogps", help="Send the drone to a GPS coordinate")
        fly_to_gps_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_gps_parser.add_argument("lat", type=float, help="Target latitude")
        fly_to_gps_parser.add_argument("long", type=float, help="Target longitude")
        fly_to_gps_parser.add_argument("alt", type=float, help="Target altitude (relative to takeoff)")
        fly_to_gps_parser.add_argument("yaw", type=float, nargs="?", default=0.0,
                                       help="Target yaw in degrees. Default 0.")
        fly_to_gps_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.25,
                                       help="Position tolerance")
        fly_to_gps_parser.add_argument("-s", "--schedule", action="store_true",
                                       help="Queue this action instead of executing immediately.")

        move_parser = subparsers.add_parser("move", help="Send the drones x, y, z meters north, east or down.")
        move_parser.add_argument("drone", type=str, help="Name of the drone")
        move_parser.add_argument("x", type=float, help="How many meters to move north (negative for south).")
        move_parser.add_argument("y", type=float, help="How many meters to move east (negative for west).")
        move_parser.add_argument("z", type=float, help="How many meters to move down (negative for up).")
        move_parser.add_argument("yaw", type=float, nargs="?", default=0.0,
                                 help="How many degrees to move to the right (negative for left). Note that this does "
                                      "not wrap around, i.e. 350 degrees to right will move the drone 10 degrees to "
                                      "the left. Default 0 degrees.")
        move_parser.add_argument("-nogps", action="store_true",
                                 help="If this flag is set we move using the drones local coordinate system.")
        move_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.25,
                                 help="Position tolerance")
        move_parser.add_argument("-s", "--schedule", action="store_true",
                                 help="Queue this action instead of executing immediately.")

        #fly_circle_parser = subparsers.add_parser("orbit", help="Fly in a circle, facing the center point")
        #fly_circle_parser.add_argument("drone", type=str, help="Name of the drone")
        #fly_circle_parser.add_argument("radius", type=float, help="Radius of the circle")
        #fly_circle_parser.add_argument("vel", type=float, help="Target velocity (negative for opposite direction)")
        #fly_circle_parser.add_argument("center_lat", type=float, help="Latitude of the center of the circle")
        #fly_circle_parser.add_argument("center_long", type=float, help="Longitude of the center of the circle")
        #fly_circle_parser.add_argument("amsl", type=float, help="Altitude in terms of AMSL.")

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

        exit_parser = subparsers.add_parser("exit", help="Exits the application")

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
        try:
            await self.drone_widgets[name].remove()
        except KeyError:
            pass

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

            match args.command:
                case "connect":
                    address = args.drone_address
                    if args.drone in DRONE_DICT and not address:
                        address = DRONE_DICT[args.drone]
                    elif not address:
                        address = "udp://:14540"
                    tmp = asyncio.create_task(self.dm.connect_to_drone(args.drone, args.server_address,
                                                                       args.server_port, address, args.timeout))
                case "disconnect":
                    tmp = asyncio.create_task(self.dm.disconnect(args.drones, force=args.force))
                case "arm":
                    tmp = asyncio.create_task(self.dm.arm(args.drones, schedule=args.schedule))
                case "disarm":
                    tmp = asyncio.create_task(self.dm.disarm(args.drones, schedule=args.schedule))
                case "takeoff":
                    tmp = asyncio.create_task(self.dm.takeoff(args.drones, schedule=args.schedule))
                case "mode":
                    tmp = asyncio.create_task(self.dm.change_flightmode(args.drones, args.mode))
                case "flyto":
                    tmp = asyncio.create_task(self.dm.fly_to(args.drone, args.x, args.y, args.z, args.yaw,
                                                             tol=args.tolerance, schedule=args.schedule))
                case "flytogps":
                    tmp = asyncio.create_task(self.dm.fly_to_gps(args.drone, args.lat, args.long, args.alt, args.yaw,
                                                                 tol=args.tolerance, schedule=args.schedule))
                case "move":
                    tmp = asyncio.create_task(self.dm.move(args.drone, args.x, args.y, args.z, args.yaw,
                                                           no_gps=args.nogps, tol=args.tolerance,
                                                           schedule=args.schedule))
                #case "orbit":
                #    tmp = asyncio.create_task(self.dm.orbit(args.drone, args.radius, args.vel, args.center_lat,
                #                                            args.center_long, args.amsl))
                case "land":
                    tmp = asyncio.create_task(self.dm.land(args.drones, schedule=args.schedule))
                case "pause":
                    tmp = asyncio.create_task(self.dm.pause(args.drones))
                case "resume":
                    tmp = asyncio.create_task(self.dm.resume(args.drones))
                case "stop":
                    tmp = asyncio.create_task(self.dm.action_stop(args.drones))
                case "kill":
                    if not args.drones:
                        if self._kill_counter:
                            tmp = asyncio.create_task(self.dm.kill(args.drones))
                        else:
                            self.logger.warning("Are you sure? Enter kill again")
                            self._kill_counter += 1
                    else:
                        tmp = asyncio.create_task(self.dm.kill(args.drones))
                case "exit":
                    tmp = asyncio.create_task(self.exit())
            self.running_tasks.add(tmp)
        except Exception as e:
            self.logger.error(repr(e))

    async def exit(self):
        """ Checks if any drones are armed and exits the app if not."""
        stop_app = True
        try:
            for name in self.dm.drones:
                if self.dm.drones[name].is_armed:
                    stop_app = False
            if stop_app:
                self.logger.info("Exiting...")
                await asyncio.sleep(2)  # Beauty pause
                self.app.exit()
        except Exception as e:
            self.logger.error(f"{repr(e)}", exc_info=True)

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

    def __init__(self, dm: DroneManager, logger=None):
        self.drone_manager = dm
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
        screen = CommandScreen(self.drone_manager, self.logger, name="control-screen")
        self.install_screen(screen, name=screen.name)
        self.add_mode("control", base_screen=screen)
        self.command_screen = screen
        status_screen = StatusScreen(self.drone_manager, self.logger, name="status-screen")
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


def main():
    drone_type = DroneMAVSDK
    drone_manager = DroneManager(drone_type)
    app = DroneApp(drone_manager, logger=drone_manager.logger)
    app.run()

    logging.shutdown()


if __name__ == "__main__":
    main()
