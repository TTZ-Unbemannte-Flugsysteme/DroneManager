import asyncio
from asyncio.exceptions import TimeoutError
import sys
import argparse
import shlex
from typing import Dict
import numpy as np

import textual.css.query
from textual import on
from textual.app import App
from textual.containers import Horizontal, Vertical
from textual.widgets import Footer, Header, Log, Static

from widgets import InputWithHistory
from drones import Drone, DroneMAVSDK, DummyMAVDrone, parse_address
from betterparser import ArgParser

# Must start mavsdk_server first, with arguments: mavsdk_server_bin.exe -p <serverport> udp://:<droneport>
# Each mavsdk server can handle exactly one drone and there is no possibility of disconnecting or connecting to another
# drone (unless both drones use the same connection and whichever connected first dies)


class DroneManager(App):
    # TODO: Figure out multiple real drones
    # TODO: Status pane for each drone with much info: positions, velocity, attitude, gps info, battery, "health" checks, check what else
    # TODO: Put a bunch of the code into their associated widgets and put those into their own files (i.e. cli into the
    #  tweaked input)
    # TODO: Print a pretty usage/command overview thing somewhere.

    # TODO: Safety checks, such as vortex ring state avoidance.

    # TODO: Look again at offboard setpoint setting.

    # TODO: Figure out what stop should do and if we even want it
    # TODO: Should changes in drone status other than due to a command also print a message to the log?
    # TODO: Proper logging: use logging library, with logging levels, add a lot of logging calls in general,
    #  save a logfile, with drone status info including position/attitude and velocity every x ms
    # TODO: Test with actual drones.
    # TODO: Handle mavsdk errors (such as offboard error) better. Currently prints either a DENIED message with little
    #   information or an even more useless "<ErrorObject at 0x...>".

    # NOTES: Look into sysid (not compid) for mavsdk server. This argument might be necessary to untangle communication.
    #   It seems that PX4 gazebo instances MAVlink stuff is not independent, they do some kind of dumb stuff

    # How often the status screen is updated.
    STATUS_REFRESH_RATE = 20

#    BINDINGS = [
#        ("k", "stop", "STOP ALL")
#    ]

    CSS = """
.text {
    text-style: bold;
}

.evenvert {
    height: 1fr;
}

#sidebar {
    width: 98;
}
"""

    def __init__(self, drone_class):
        super().__init__()
        self._drone_class = drone_class
        self.drones: Dict[str, Drone] = {}
        self.running_tasks = set()
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self.drone_lock = asyncio.Lock()
        self._kill_counter = 0  # Require kill all to be entered twice

        self.compid = 160

        self.parser = ArgParser(
            description="Interactive command line interface to connect and control multiple drones")
        subparsers = self.parser.add_subparsers(title="command",
                                                description="Command to execute.", dest="command")
        connect_parser = subparsers.add_parser("connect")
        connect_parser.add_argument("drone", type=str, help="Name for the drone.")
        connect_parser.add_argument("drone_address", type=str, nargs='?', default="udp://:14540",
                                    help="Connection string. Something like udp://:14540")
        connect_parser.add_argument("-sa", "--server_address", type=str, default=None,
                                    help="Address for the mavsdk server. If omitted, a server is started "
                                         "automatically. Use this only if you already have a server for this drone "
                                         "running (for example on another machine). Default None")
        connect_parser.add_argument("-sp", "--server_port", type=int, default=50051,
                                    help="Port for the mavsdk server. Default 50051.")
        connect_parser.add_argument("-t", "--timeout", type=float, default=5, required=False,
                                    help="Timeout in seconds for connection attempts.")
        arm_parser = subparsers.add_parser("arm")
        arm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to arm")

        disarm_parser = subparsers.add_parser("disarm")
        disarm_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to disarm")

        takeoff_parser = subparsers.add_parser("takeoff")
        takeoff_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to take off with.")

        offboard_parser = subparsers.add_parser("mode", help="Change the drones flight mode")
        offboard_parser.add_argument("mode", type=str, help="Target flight mode. Must be one of {}.".format(
            self._drone_class.VALID_FLIGHTMODES))
        offboard_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to change flight mode on.")

        fly_to_parser = subparsers.add_parser("flyto")
        fly_to_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_parser.add_argument("x", type=float, help="Target x coordinate")
        fly_to_parser.add_argument("y", type=float, help="Target y coordinate")
        fly_to_parser.add_argument("z", type=float, help="Target z coordinate")
        fly_to_parser.add_argument("yaw", type=float, nargs="?", default=0.0, help="Target yaw in degrees. Default 0.")
        fly_to_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.5,
                                   help="Position tolerance")

        fly_to_gps_parser = subparsers.add_parser("flytogps")
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

        land_parser = subparsers.add_parser("land")
        land_parser.add_argument("drones", type=str, nargs="+", help="Drone(s) to land")

        stop_parser = subparsers.add_parser("stop", help="Stops (i.e. lands) drones. If no drones are listed, "
                                                         "stops all of them and then exits the application")
        stop_parser.add_argument("drones", type=str, nargs="*", help="Drone(s) to stop.")

        kill_parser = subparsers.add_parser("kill", help="Kills (i.e. disarms and stops everything) drones. If no "
                                                         "drones are listed, kills all of them.")
        kill_parser.add_argument("drones", type=str, nargs="*", help="Drone(s) to kill.")

    def run(self, *args, **kwargs):
        super().run(*args, **kwargs)

    @on(InputWithHistory.Submitted, "#cli")
    async def cli(self, message):
        output = self.query_one("#output", expect_type=Log)
        value = message.value
        message.control.clear()
        try:
            args = self.parser.parse_args(shlex.split(value))
        except ValueError as e:
            output.write_line(repr(e))
            return
        try:
            if args.command != "kill" or args.drones:
                self._kill_counter = 0

            if args.command == "connect":
                asyncio.create_task(self.connect_to_drone(args.drone, args.server_address, args.server_port,
                                                          args.drone_address, args.timeout), name=args.drone)
            elif args.command == "arm":
                asyncio.create_task(self.arm(args.drones))
            elif args.command == "disarm":
                asyncio.create_task(self.disarm(args.drones))
            elif args.command == "takeoff":
                asyncio.create_task(self.takeoff(args.drones))
            elif args.command == "mode":
                asyncio.create_task(self.change_flightmode(args.drones, args.mode))
            elif args.command == "flyto":
                asyncio.create_task(self.fly_to(args.drone, args.x, args.y, args.z, args.yaw, tol=args.tolerance))
            elif args.command == "flytogps":
                asyncio.create_task(self.fly_to_gps(args.drone, args.lat, args.long, args.alt, args.yaw,
                                                    tol=args.tolerance))
            elif args.command == "orbit":
                asyncio.create_task(self.orbit(args.drone, args.radius, args.vel, args.center_lat, args.center_long,
                                               args.amsl))
            elif args.command == "land":
                asyncio.create_task(self.land(args.drones))
            elif args.command == "stop":
                await self.action_stop(args.drones)
            elif args.command == "kill":
                if not args.drones:
                    if self._kill_counter:
                        await self.kill(args.drones)
                    else:
                        output.write_line("Are you sure? Enter kill again")
                        self._kill_counter += 1
                else:
                    asyncio.create_task(self.kill(args.drones))
        except Exception as e:
            output.write_line(repr(e))

    @property
    def used_drone_addrs(self):
        return [drone.drone_addr for drone in self.drones.values()]

    async def connect_to_drone(self,
                               name: str,
                               mavsdk_server_address: str | None,
                               mavsdk_server_port: int,
                               drone_address: str,
                               timeout: float):
        output = self.query_one("#output", expect_type=Log)
        _, parsed_addr, parsed_port = parse_address(string=drone_address)
        parsed_connection_string = parse_address(string=drone_address, return_string=True)
        output.write_line(f"Trying to connect to drone {name} @{parsed_connection_string}")
        async with self.drone_lock:
            try:
                # Ensure that for each drone there is a one-to-one-to-one relation between name, mavsdk port and drone
                if name in self.drones:
                    output.write_line(f"A drone called {name} already exists. Each drone must have a unique name.")
                    return False
                if not mavsdk_server_address:
                    used_ports = [drone.server_port for drone in self.drones.values()]
                    while mavsdk_server_port in used_ports:
                        mavsdk_server_port += 17
                        self.compid += 1
                # Check that we don't already have this drone connected.
                for other_name in self.drones:
                    other_drone = self.drones[other_name]
                    _, other_addr, other_port = parse_address(string=other_drone.drone_addr)
                    if parsed_addr == other_addr and parsed_port == other_port:
                        output.write_line(f"{other_name} is already connected to drone with address {drone_address}.")
                        return False
                drone = self._drone_class(name, mavsdk_server_address, mavsdk_server_port, compid=self.compid)
                connected = await asyncio.wait_for(drone.connect(drone_address), timeout)
                if connected:
                    output.write_line(f"Connected to drone {name}!")
                    self.drones[name] = drone
                    return True
                else:
                    output.write_line(f"Failed to connect to drone {name}!")
                    del drone
                    return False
            except TimeoutError:
                output.write_line(f"Connection attempts to {name} timed out!")
                del drone
                return False

    async def _multiple_drone_action(self, action, names, start_string, success_string, fail_string, *args, **kwargs):
        """

        :param action:
        :param names:
        :param start_string:
        :param success_string: Output string for the success of a single drone. Will have the name of the drone passed
                               to it.
        :param fail_string: Output string for the failure of a single drone. Will have the name of the drone passed
                            to it.
        :return:
        """
        output = self.query_one("#output", expect_type=Log)
        output.write_line(start_string.format(names))
        try:
            results = await asyncio.gather(*[action(self.drones[name], *args, **kwargs) for name in names],
                                           return_exceptions=True)
            for i, result in enumerate(results):
                if not result:
                    output.write_line(fail_string.format(names[i]))
                elif result and not isinstance(result, Exception):
                    output.write_line(success_string.format(names[i]))
                else:
                    output.write_line(f"Drone {names[i]} failed due to {repr(result)}")
        except KeyError:
            output.write_line("No drones named {}!".format([name for name in names if name not in self.drones]))
        except Exception as e:
            output.write_line(repr(e))

    async def arm(self, names):
        await self._multiple_drone_action(self._drone_class.arm,
                                          names,
                                          "Arming drone(s) {}.",
                                          "{} armed!",
                                          "{} couldn't be armed!")

    async def disarm(self, names):
        await self._multiple_drone_action(self._drone_class.disarm,
                                          names,
                                          "Disarming drone(s) {}.",
                                          "{} disarmed!",
                                          "{} couldn't be disarmed!")

    async def takeoff(self, names):
        await self._multiple_drone_action(self._drone_class.takeoff,
                                          names,
                                          "Drone(s) {} taking off.",
                                          "{} taking off!",
                                          "{} couldn't take off!")

    async def change_flightmode(self, names, flightmode):
        await self._multiple_drone_action(self._drone_class.change_flight_mode,
                                          names,
                                          "Changing flightmode for drone(s) {} to " + flightmode + ".",
                                          "{} in " + flightmode + " mode!",
                                          flightmode + " failed for drone {}!",
                                          flightmode)

    async def land(self, names):
        await self._multiple_drone_action(self._drone_class.land,
                                          names,
                                          "Landing drone(s) {}.",
                                          "{} landing.",
                                          "{} couldn't start landing!")

    async def fly_to(self, name, x, y, z, yaw, tol=0.5):
        point = np.array([x, y, z, yaw])
        output = self.query_one("#output", expect_type=Log)
        output.write_line(f"Sending {name} to {point}")
        try:
            await self.drones[name].fly_to_point(point, tolerance=tol)
            output.write_line(f"{name} arrived at {point}!")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def fly_to_gps(self, name, lat, long, alt, yaw, tol=0.5):
        output = self.query_one("#output", expect_type=Log)
        output.write_line(f"Sending {name} to {(lat, long, alt)}")
        try:
            await self.drones[name].fly_to_gps(lat, long, alt, yaw, tolerance=tol)
            output.write_line(f"{name} arrived at {(lat, long, alt)}")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def orbit(self, name, radius, velocity, center_lat, center_long, amsl):
        output = self.query_one("#output", expect_type=Log)
        try:
            await self.drones[name].orbit(radius, velocity, center_lat, center_long, amsl)
            output.write_line(f"{name} flying in a circle.")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def _stop_drone(self, name):
        drone = self.drones[name]
        result = await drone.stop()
        self.drones.pop(name)
        del drone
        return result

    async def action_stop(self, names):
        stop_app = False
        if not names:
            stop_app = True
        output = self.query_one("#output", expect_type=Log)
        async with self.drone_lock:
            if not names:
                output.write_line("Stopping all drones!")
            else:
                output.write_line(f"Stopping {names}")
            drones_to_stop = names if names else list(self.drones.keys())
            results = await asyncio.gather(*[self._stop_drone(name) for name in drones_to_stop], return_exceptions=True)
            for i, result in enumerate(results):
                # If one of the drones encounters an excepton
                if isinstance(result, Exception):
                    output.write_line(f"During stopping, drone {drones_to_stop[i]} encountered an exception "
                                      f"{repr(result)}!")
                    stop_app = False
            if stop_app:
                output.write_line("All drones stopped, exiting...")
                await asyncio.sleep(2)  # Beauty pause
                self.exit()

    async def kill(self, names):
        output = self.query_one("#output", expect_type=Log)
        async with self.drone_lock:
            if not names:
                output.write_line("Killing all drones!")
            else:
                output.write_line(f"Killing {names}")
            drones_to_stop = names if names else list(self.drones.keys())
            for name in drones_to_stop:
                asyncio.create_task(self.drones[name].kill())

    async def update_status(self):
        output = None
        log = None
        while not output and not log:
            try:
                output = self.query_one("#status", expect_type=Static)
                log = self.query_one("#output", expect_type=Log)
            except textual.css.query.NoMatches:
                await asyncio.sleep(0.1)
        try:
            while True:
                status_string = ""
                status_string += "Drone Status\n"
                format_string_drones = "{:<10}   {:>9}   {:>5}   {:>6}   {:>15}   {:>10.7f}   {:>6.3f}   {:>6.3f}   {:>6.3f}"
                format_string_header = "{:<10}   {:>9}   {:>5}   {:>6}   {:>15}   {:>10}   {:>6}   {:>6}   {:>6}"
                header_string = format_string_header.format("Name", "Connected", "Armed", "In-Air", "FlightMode",
                                                            "GPS", "NED", "Vel", "Alt")
                status_string += header_string + "\n"
                status_string += "="*len(header_string) + "\n"
                for name in list(self.drones.keys()):
                    drone = self.drones[name]
                    if len(name) > 10:
                        name = name[:7] + "..."
                    status_string += format_string_drones.format("", "", "", "", "",
                                                                 drone.position_global[0],
                                                                 drone.position_ned[0], drone.velocity[0], 0) + "\n"
                    status_string += format_string_drones.format(str(name), str(drone.is_connected),
                                                                 str(drone.is_armed), str(drone.in_air),
                                                                 str(drone.flightmode), drone.position_global[1],
                                                                 drone.position_ned[1], drone.velocity[1],
                                                                 drone.position_global[3]) + "\n"
                    status_string += format_string_drones.format("", "", "", "", "",
                                                                 drone.position_global[2],
                                                                 drone.position_ned[2], drone.velocity[2], 0) + "\n\n"

                output.update(status_string)
                await asyncio.sleep(1/self.STATUS_REFRESH_RATE)
        except Exception as e:
            log.write_line(repr(e))

    def schedule_background_tasks(self):
        asyncio.create_task(self.update_status())

    def compose(self):
        yield Header()
        yield Vertical(
            Horizontal(
                Log(id="output", classes="text"),
                Vertical(
                    Static(id="status", classes="text evenvert"),
                    Static(id="usage", classes="text evenvert", renderable=self.parser.format_usage()),
                    id="sidebar",
                )
            ),
            InputWithHistory(placeholder="Command line", id="cli")
        )

        yield Footer()
        self.schedule_background_tasks()

        # Try to redirect stdout to our logger window to get prints from other modules
        # Redirect all the drone output to our text widget
        def decorator(func):
            def inner(inputstr):
                try:
                    output = self.query_one("#output", expect_type=Log)
                    output.write_line(inputstr)
                    return func(inputstr)
                except:
                    return func(inputstr)
            return inner
        sys.stdout.write = decorator(sys.stdout.write)
        sys.stderr.write = decorator(sys.stderr.write)


if __name__ == "__main__":
    start_parser = argparse.ArgumentParser()
    start_parser.add_argument("-d", "--dummy", action="store_true", help="If set, use a dummy drone class")

    start_args = start_parser.parse_args()

    drone_type = DummyMAVDrone if start_args.dummy else DroneMAVSDK
    app = DroneManager(drone_type)
    app.run()
