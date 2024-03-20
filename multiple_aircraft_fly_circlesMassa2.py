import asyncio
import time
import numpy as np
import argparse
import shlex
from typing import Dict

from mavfleetcontrol.craft import Craft
import mavfleetcontrol.actions as actions

# TURNS OUT ARGPARSE IS ARSE, DOESN'T THROW EXCEPTIONS AND JUST QUITS INSTEAD, LMAO
class ArgumentParserError(Exception): pass

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
    def __init__(self, id, connection):
        self.id = id
        self.connection = connection
        self.arm = False
        self.offboard = False


async def _drone_connect(name, address):
    #drone = Craft(name, address)
    con_delay = random.uniform(3,10)
    con_success = random.uniform(0,1)
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
    drone.offboard = True
async def _drone_change_mode():
    pass

# END DUMMY STUFF #


async def connect_to_drone(name, address):
    print(f"Connecting to drone {name}...")
    try:
        drone = await _drone_connect(name, address)
        print(f"Connected to drone {name}!")
        return drone
    except RuntimeError:
        print("Failed to connect to drone!")
        return None


async def get_command(parser):
    loop = asyncio.get_running_loop()
    invalid_command = True
    while invalid_command:
        try:
            icl = await loop.run_in_executor(None, input, "> ")
            args = parser.parse_args(shlex.split(icl))
            invalid_command = False
        except ValueError as e:
            print(repr(e))
            invalid_command = True
    return args


class DroneManager:
    def __init__(self):
        self.drones = {}
        self.parsers = {}
        self.running_tasks = set()
        #self.loop = asyncio.get_event_loop()
        self.stop_execution = False

        self.parser = ArgParser(
            description="Interactive command line interface to connect and control multiple drones")
        subparsers = self.parser.add_subparsers(title="command", description="Command to execute.", dest="command")
        connect_parser = subparsers.add_parser("connect")
        connect_parser.add_argument("drone", help="Name for the drone.")
        connect_parser.add_argument("address", help="Connection string. Something like udp://:14550")
        arm_parser = subparsers.add_parser("arm")
        arm_parser.add_argument("drone", help="Drone to arm")
        status_parser = subparsers.add_parser("status")
        exit_parser = subparsers.add_parser("exit")
        self.parsers["connect"] = connect_parser
        self.parsers["arm"] = arm_parser
        self.parsers["status"] = status_parser
        self.parsers["exit"] = exit_parser

    def run(self):
        asyncio.run(self.icli())

    async def icli(self):
        while not self.stop_execution:
            # TODO: Catch Exceptions
            args = await get_command(self.parser)
            if args.command == "connect":
                task = asyncio.create_task(connect_to_drone(args.drone, args.address), name=args.drone)
                self.running_tasks.add(task)
                task.add_done_callback(self.connect_callback)
            if args.command == "arm":
                task = asyncio.create_task(self.arm_drone(args.drone))
                self.running_tasks.add(task)
            if args.command == "status":
                self.status()
            if args.command == "exit":
                self.stop_execution = True
                self.stop()

    def connect_callback(self, task):
        drone = task.result()
        if drone is not None:
            self.drones[task.get_name()] = drone
        self.running_tasks.discard(task)

    async def arm_drone(self, name):
        print(f"Arming drone {name}")
        drone = self.drones[name]
        try:
            await _drone_arm(drone)
        except KeyError:
            print(f"No drone named {name}")
        print(f"{name} armed!")

    def status(self):
        print("Drone Status")
        header_string = "{:<10}    {:>5}   {:>8}".format("Name", "Armed", "Offboard")
        print("{:<10}    {:>5}   {:>8}".format("Name", "Armed", "Offboard"))
        print("="*len(header_string))
        for name in self.drones:
            drone = self.drones[name]
            if len(name) > 10:
                name = name[:7] + "..."
            print("{:<10}    {:>5}   {:>5}".format(str(name), str(drone.arm), str(drone.offboard)))
        print("="*len(header_string))

    def stop(self):
        # TODO: Figure out what the best thing to do here would be
        for name in self.drones:
            drone = self.drones[name]
            #drone.override_action(actions.land)
            #drone.close_conn()
            #drone.join()
            self.drones.pop(name)


def full_loop():
    loop = asyncio.get_event_loop()

    drone2 = Craft('drone2', "udp://:14562")
    drone1 = Craft('drone1', "udp://:14563")
    drone4 = Craft('drone4', "udp://:14566")
    drone0 = Craft('drone0', "udp://:14561")
    drone3 = Craft('drone3', "udp://:14564")
    drone5 = Craft('drone5', "udp://:14565")
    # drone6 = Craft('drone6',"udp://:14566")
    # drone1 = Craft('drone1',"serial:/dev/ttyS8:14550")

    # loop.run_until_complete(drone.arm(coordinate=[0.0,0.0,0.0],attitude=[0.0,0.0,0.0]))
    print("Starting.....")
    drone0.start()
    time.sleep(1)
    drone1.start()
    time.sleep(1)
    drone2.start()
    time.sleep(1)
    drone3.start()
    time.sleep(1)
    drone4.start()
    time.sleep(1)
    drone5.start()
    # drone0.add_action(land)
    # drone0.add_action(Disarm())
    time.sleep(2)
    print("Arming drone 0....")
    drone0.add_action(actions.arm.Arm())
    time.sleep(1)
    print("Arming drone 1....")
    drone1.add_action(actions.arm.Arm())
    time.sleep(2)
    print("Arming drone 2....")
    drone2.add_action(actions.arm.Arm())
    time.sleep(2)
    print("Arming drone 3....")
    drone3.add_action(actions.arm.Arm())
    time.sleep(1)
    print("Arming drone 4....")
    drone4.add_action(actions.arm.Arm())
    time.sleep(1)
    print("Arming drone 5....")
    drone5.add_action(actions.arm.Arm())

    time.sleep(2)

    drone0.add_action(actions.point.FlyToPoint(np.array([3, 0, -5]), tolerance=1))
    time.sleep(2)
    drone1.add_action(actions.point.FlyToPoint(np.array([0, 0, -5]), tolerance=1))
    time.sleep(2)
    drone2.add_action(actions.point.FlyToPoint(np.array([2, 0, -5]), tolerance=1))
    time.sleep(2)
    drone3.add_action(actions.point.FlyToPoint(np.array([3, 0, -5]), tolerance=1))
    time.sleep(2)
    drone4.add_action(actions.point.FlyToPoint(np.array([0, 0, -5]), tolerance=1))
    time.sleep(2)
    drone5.add_action(actions.point.FlyToPoint(np.array([0, 0, -5]), tolerance=1))
    time.sleep(2)
    drone0.add_action(actions.circle.Circle(velocity=3.0, radius=5.0, angle=0.0, direction='cw'))
    time.sleep(2)
    drone1.add_action(actions.circle.Circle(velocity=3.0, radius=6.0, angle=0.0, direction='ccw'))
    time.sleep(2)
    drone2.add_action(actions.circle.Circle(velocity=3.0, radius=6.0, angle=0.0, direction='ccw'))
    time.sleep(2)
    drone3.add_action(actions.circle.Circle(velocity=3.0, radius=6.0, angle=0.0, direction='ccw'))
    time.sleep(2)
    drone4.add_action(actions.circle.Circle(velocity=3.0, radius=6.0, angle=0.0, direction='ccw'))
    time.sleep(2)
    drone5.add_action(actions.circle.Circle(velocity=3.0, radius=6.0, angle=0.0, direction='ccw'))
    time.sleep(2)

    drone0.add_action(actions.land)
    drone1.add_action(actions.land)
    drone2.add_action(actions.land)
    drone3.add_action(actions.land)
    drone4.add_action(actions.land)
    drone5.add_action(actions.land)
    # drone0.override_action(land)

    # drone0.close_conn()#will run after FLYTOPOINT IS DONE)
    # drone0.join()

    # drone1.override_action(land)
    drone0.close_conn()  # will run after FLYTOPOINT IS DONE)
    drone0.join()

    drone1.close_conn()  # will run after FLYTOPOINT IS DONE)
    drone1.join()

    drone2.close_conn()  # will run after FLYTOPOINT IS DONE)
    drone2.join()

    drone3.close_conn()  # will run after FLYTOPOINT IS DONE)
    drone3.join()

    drone4.close_conn()  # will run after FLYTOPOINT IS DONE)
    drone4.join()

    drone5.close_conn()  # will run after FLYTOPOINT IS DONE)
    drone5.join()


if __name__ == "__main__":
    # Fixed script
    #full_loop()

    # Interactive terminal
    dm = DroneManager()
    dm.run()


