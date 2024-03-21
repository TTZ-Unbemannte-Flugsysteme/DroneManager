import asyncio
import numpy as np
import argparse
import shlex
import random
from typing import Dict

import textual.css.query
from textual import on
from textual.app import App
from textual.containers import Horizontal, Vertical
from textual.widgets import Footer, Header, Input, Log, Static


# TURNS OUT ARGPARSE IS ARSE, DOESN'T THROW EXCEPTIONS AND JUST QUITS INSTEAD, LMAO
class ArgumentParserError(Exception):
    pass


class ArgParser(argparse.ArgumentParser):
    def error(self, message):
        if "invalid choice" in message:
            raise ValueError(message)
        elif "arguments are required" in message:
            raise ValueError(message)
        elif "unrecognized argument" in message:
            raise ValueError(message)
        else:
            raise ArgumentParserError(message)


# DUMMY STUFF #

class Drone:
    def __init__(self, drone_id, connection):
        self.id = drone_id
        self.connection = connection
        self.drone_ref = None
        self.is_armed = False
        self.offboard_mode = False
        asyncio.create_task(self._fail_offboard())

    async def _fail_offboard(self):
        while True:
            until_next = random.uniform(10, 15)
            duration = random.uniform(3, 5)
            if self.offboard_mode:
                self.offboard_mode = False
                await asyncio.sleep(duration)
                await self.offboard()
            await asyncio.sleep(until_next)

    @classmethod
    async def connect(cls, name, address):
        #drone = Craft(name, address)
        con_delay = random.uniform(3, 10)
        con_success = random.uniform(0, 1)
        con_success_rate = 0.5
        await asyncio.sleep(con_delay)
        if con_success < con_success_rate:
            return cls(name, True)
        else:
            raise RuntimeError(f"Failed to connect to drone!")

    async def arm(self):
        #drone.arm()
        self.is_armed = True

    async def disarm(self):
        #drone.disarm()
        self.is_armed = False

    async def offboard(self):
        if not self.is_armed:
            raise RuntimeError("Can't offboard a disarmed drone!")
        self.offboard_mode = True

    async def _drone_change_mode(self):
        raise NotImplementedError

    async def fly_to_point(self, point: np.ndarray, tolerance=0.5):
        #self.drone_ref.add_action(actions.point.FlyToPoint(point, tolerance=tolerance))
        await asyncio.sleep(random.uniform(0.1, 3))
        if not self.is_armed:
            raise RuntimeError("Can't fly an unarmed drone!")
        return 1

    async def fly_circle(self, velocity, radius, angle, direction):
        #self.drone_ref.add_action(actions.circle.Circle(velocity=velocity, radius=radius, angle=angle,
        #                                                direction=direction))
        await asyncio.sleep(random.uniform(0.1, 3))
        if not self.is_armed:
            raise RuntimeError("Can't fly an unarmed drone!")
        return 1

    async def land(self):
        await asyncio.sleep(random.uniform(0.1, 3))
        if not self.is_armed:
            raise RuntimeError("Can't land an unarmed drone!")
        await asyncio.sleep(random.uniform(3, 10))
        await self.disarm()
        return 1


# END DUMMY STUFF #


class DroneManager(App):

    STATUS_REFRESH_RATE = 4

#    BINDINGS = [
#        ("k", "stop", "STOP ALL")
#    ]

    CSS = """
.text {
    text-style: bold;
}
    
#status {
    width: 40;
}
"""

    def __init__(self):
        super().__init__()
        self.drones: Dict[str, Drone] = {}
        self.running_tasks = set()
        self.stop_execution = False

        self.parser = ArgParser(
            description="Interactive command line interface to connect and control multiple drones")
        subparsers = self.parser.add_subparsers(title="command",
                                                description="Command to execute.", dest="command")
        connect_parser = subparsers.add_parser("connect")
        connect_parser.add_argument("drone", type=str, help="Name for the drone.")
        connect_parser.add_argument("address", type=str,
                                    help="Connection string. Something like udp://:14550")
        arm_parser = subparsers.add_parser("arm")
        arm_parser.add_argument("drone", type=str, help="Drone to arm")
        disarm_parser = subparsers.add_parser("disarm")
        disarm_parser.add_argument("drone", type=str, help="Drone to arm")
        offboard_parser = subparsers.add_parser("offboard")
        offboard_parser.add_argument("drone", type=str, help="Drone to put into offboard mode")
        fly_to_parser = subparsers.add_parser("flyto")
        fly_to_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_to_parser.add_argument("x", type=float, help="Target x coordinate")
        fly_to_parser.add_argument("y", type=float, help="Target y coordinate")
        fly_to_parser.add_argument("z", type=float, help="Target z coordinate")
        fly_to_parser.add_argument("-t", "--tolerance", type=float, required=False, default=0.5,
                                   help="Position tolerance")
        fly_circle_parser = subparsers.add_parser("flycircle")
        fly_circle_parser.add_argument("drone", type=str, help="Name of the drone")
        fly_circle_parser.add_argument("vel", type=float, help="Target velocity")
        fly_circle_parser.add_argument("radius", type=float, help="Radius of the circle")
        fly_circle_parser.add_argument("angle", type=float, help="Angle (?) of the circle")
        fly_circle_parser.add_argument("dir", type=str, choices=["cw", "ccw"],
                                       help="Direction of the circle")
        land_parser = subparsers.add_parser("land")
        land_parser.add_argument("drone", type=str, help="Drone to land")
        exit_parser = subparsers.add_parser("exit")

    def run(self, *args, **kwargs):
        super().run(*args, **kwargs)

    @on(Input.Submitted, "#cli")
    def cli(self, message):
        output = self.query_one("#output", expect_type=Log)
        value = message.value
        message.control.clear()
        try:
            args = self.parser.parse_args(shlex.split(value))
        except ValueError as e:
            output.write_line(repr(e))
            return
        try:
            if args.command == "connect":
                task = asyncio.create_task(self.connect_to_drone(args.drone, args.address), name=args.drone)
                self.running_tasks.add(task)
                task.add_done_callback(self.connect_callback)
            if args.command == "arm":
                asyncio.create_task(self.arm(args.drone))
            if args.command == "disarm":
                asyncio.create_task(self.disarm(args.drone))
            if args.command == "offboard":
                asyncio.create_task(self.offboard(args.drone))
            if args.command == "flyto":
                asyncio.create_task(self.fly_to(args.drone, args.x, args.y, args.z, args.tolerance))
            if args.command == "flycircle":
                asyncio.create_task(self.fly_circle(args.drone, args.vel, args.radius, args.angle, args.dir))
            if args.command == "land":
                asyncio.create_task(self.land(args.drone))
            if args.command == "exit":
                self.stop_execution = True
                self.action_stop()
        except Exception as e:
            output.write_line(repr(e))

    def connect_callback(self, task):
        drone = task.result()
        if drone is not None:
            self.drones[task.get_name()] = drone
        self.running_tasks.discard(task)

    async def connect_to_drone(self, name, address):
        output = self.query_one("#output", expect_type=Log)
        output.write_line(f"Connecting to drone {name}...")
        try:
            drone = await Drone.connect(name, address)
            output.write_line(f"Connected to drone {name}!")
            return drone
        except RuntimeError:
            output.write_line(f"Failed to connect to {name}!")

    async def arm(self, name):
        output = self.query_one("#output", expect_type=Log)
        output.write_line(f"Arming drone {name}.")
        try:
            await self.drones[name].arm()
            output.write_line(f"{name} armed!")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def disarm(self, name):
        output = self.query_one("#output", expect_type=Log)
        output.write_line(f"Disarming drone {name}.")
        try:
            await self.drones[name].disarm()
            output.write_line(f"{name} disarmed!")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def offboard(self, name):
        output = self.query_one("#output", expect_type=Log)
        output.write_line(f"Offboarding drone {name}.")
        try:
            await self.drones[name].offboard()
            output.write_line(f"{name} in offboard mode!")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def fly_to(self, name, x, y, z, tol=0.5):
        point = np.array([x, y, z])
        output = self.query_one("#output", expect_type=Log)
        try:
            await self.drones[name].fly_to_point(point, tolerance=tol)
            output.write_line(f"Sending {name} to point {point}.")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def fly_circle(self, name, velocity, radius, angle, direction):
        output = self.query_one("#output", expect_type=Log)
        try:
            await self.drones[name].fly_circle(velocity, radius, angle, direction)
            output.write_line(f"{name} flying in a circle.")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    async def land(self, name):
        output = self.query_one("#output", expect_type=Log)
        try:
            output.write_line(f"Landing drone {name}.")
            await self.drones[name].land()
            output.write_line(f"{name} landed.")
        except KeyError:
            output.write_line(f"No drone named {name}!")
        except Exception as e:
            output.write_line(repr(e))

    def action_stop(self):
        # TODO: Figure out what the best thing to do here would be

        for name in self.drones:
            drone = self.drones[name]
            #drone.override_action(actions.land)
            #drone.close_conn()
            #drone.join()
        self.exit()

    async def update_status(self):
        output = None
        while not output:
            try:
                output = self.query_one("#status", expect_type=Static)
            except textual.css.query.NoMatches:
                await asyncio.sleep(0.5)
        while True:
            status_string = ""
            status_string += "Drone Status\n"
            header_string = "{:<10}    {:>5}   {:>8}".format("Name", "Armed", "Offboard")
            status_string += header_string + "\n"
            status_string += "="*len(header_string) + "\n"
            for name in self.drones:
                drone = self.drones[name]
                if len(name) > 10:
                    name = name[:7] + "..."
                status_string += "{:<10}    {:>5}   {:>5}".format(str(name),
                                                                  str(drone.is_armed),
                                                                  str(drone.offboard_mode),
                                                                  ) + "\n"
            output.update(status_string)
            await asyncio.sleep(1/self.STATUS_REFRESH_RATE)

    def schedule_background_tasks(self):
        asyncio.create_task(self.update_status())

    def compose(self):
        yield Header()
        yield Vertical(
            Horizontal(
                Log(id="output", classes="text"),
                Static(id="status", classes="text")
            ),
            Input(placeholder="Command line", id="cli")
        )
        # TODO: Add history to input

        yield Footer()
        self.schedule_background_tasks()


if __name__ == "__main__":
    app = DroneManager()
    app.run()
