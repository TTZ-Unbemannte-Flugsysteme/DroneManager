import asyncio
import argparse
import shlex

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

import random


class DummyDrone:
    def __init__(self, drone_id, connection):
        self.id = drone_id
        self.connection = connection
        self.arm = False
        self.offboard = False
        asyncio.create_task(self._fail_offboard())

    async def _fail_offboard(self):
        while True:
            until_next = random.uniform(10, 15)
            duration = random.uniform(3, 5)
            if self.offboard:
                self.offboard = False
                await asyncio.sleep(duration)
                self.offboard = True
            await asyncio.sleep(until_next)


async def _drone_connect(name, address):
    #drone = Craft(name, address)
    con_delay = random.uniform(3, 10)
    con_success = random.uniform(0, 1)
    con_success_rate = 0.5
    if con_success < con_success_rate:
        drone = DummyDrone(name, True)
        return drone
    else:
        raise RuntimeError("Failed to connect to drone!")


async def _drone_arm(drone):
    #drone.arm()
    drone.arm = True


async def _drone_disarm(drone):
    #drone.disarm()
    drone.arm = False


async def _drone_offboard(drone):
    if not drone.arm:
        raise RuntimeError("Can't offboard a disarmed drone!")
    drone.offboard = True


async def _drone_change_mode():
    pass

# END DUMMY STUFF #


class DroneManager(App):

    STATUS_REFRESH_RATE = 4

    BINDINGS = [
        ("k", "stop", "STOP ALL")
    ]

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
        self.drones = {}
        self.running_tasks = set()
        #self.loop = asyncio.get_event_loop()
        self.stop_execution = False
        self.refresh_task = None

        self.parser = ArgParser(
            description="Interactive command line interface to connect and control multiple drones")
        subparsers = self.parser.add_subparsers(title="command", description="Command to execute.", dest="command")
        connect_parser = subparsers.add_parser("connect")
        connect_parser.add_argument("drone", help="Name for the drone.")
        connect_parser.add_argument("address", help="Connection string. Something like udp://:14550")
        arm_parser = subparsers.add_parser("arm")
        arm_parser.add_argument("drone", help="Drone to arm")
        disarm_parser = subparsers.add_parser("disarm")
        disarm_parser.add_argument("drone", help="Drone to arm")
        offboard_parser = subparsers.add_parser("offboard")
        offboard_parser.add_argument("drone", help="Drone to put into offboard mode")
        exit_parser = subparsers.add_parser("exit")

    def run(self, *args, **kwargs):
        super().run(*args, **kwargs)

    @on(Input.Submitted, "#cli")
    def cli(self, message):
        output = self.query_one("#output")
        value = message.value
        message.control.clear()
        try:
            args = self.parser.parse_args(shlex.split(value))
        except ValueError as e:
            output.write_line(repr(e))
            return
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
        if args.command == "exit":
            self.stop_execution = True
            self.action_stop()

    def connect_callback(self, task):
        drone = task.result()
        if drone is not None:
            self.drones[task.get_name()] = drone
        self.running_tasks.discard(task)

    async def connect_to_drone(self, name, address):
        output = self.query_one("#output")
        output.write_line(f"Connecting to drone {name}...")
        try:
            drone = await _drone_connect(name, address)
            output.write_line(f"Connected to drone {name}!")
            return drone
        except RuntimeError:
            output.write_line("Failed to connect to drone!")

    async def arm(self, name):
        output = self.query_one("#output")
        output.write_line(f"Arming drone {name}")
        try:
            drone = self.drones[name]
            await _drone_arm(drone)
            output.write_line(f"{name} armed!")
        except KeyError:
            output.write_line(f"No drone named {name}")
        except Exception as e:
            output.write_line(repr(e))

    async def disarm(self, name):
        output = self.query_one("#output")
        output.write_line(f"Disarming drone {name}")
        try:
            drone = self.drones[name]
            await _drone_disarm(drone)
            output.write_line(f"{name} disarmed!")
        except KeyError:
            output.write_line(f"No drone named {name}")
        except Exception as e:
            output.write_line(repr(e))

    async def offboard(self, name):
        output = self.query_one("#output")
        output.write_line(f"Offboarding drone {name}")
        try:
            drone = self.drones[name]
            await _drone_offboard(drone)
            output.write_line(f"{name} in offboard mode!")
        except KeyError:
            output.write_line(f"No drone named {name}")
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
                output = self.query_one("#status")
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
                status_string += "{:<10}    {:>5}   {:>5}".format(str(name), str(drone.arm), str(drone.offboard)) + "\n"
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
