import asyncio
import time
import numpy as np
import argparse
import shlex
from typing import Dict

from mavfleetcontrol.craft import Craft
import mavfleetcontrol.actions as actions


# DUMMY STUFF #

import random


class DummyDrone:
    def __init__(self, id, connection):
        self.id = id
        self.connection = connection
        self.arm = False


async def _drone_connect(name, address):
    #drone = Craft(name, address)
    con_delay = random.uniform(3,10)
    con_success = random.uniform(0,1)
    con_success_rate = 0.5
    # TODO: DUMMY WAITS
    print(f"Connecting drone {name}...")
    if con_success < con_success_rate:
        print(f"Connected to drone {name}!")
        drone = DummyDrone(name, True)
        return drone
    else:
        raise RuntimeError("Failed drone connection in some way.")


async def _drone_arm(drone):
    #drone.add_action(actions.arm.Arm())
    drone.arm = True

# END DUMMY STUFF #


async def connect_to_drone(name, address):
    print(f"Connecting drone {name}...")
    drone = await _drone_connect(name, address)
    print(f"Connected to drone {name}!")
    return drone


async def get_command(parser):
    loop = asyncio.get_running_loop()
    invalid_command = True
    while invalid_command:
        try:
            icl = await loop.run_in_executor(None, input, "> ")
            invalid_command = False
        except SystemExit:
            pass
    args = parser.parse_args(shlex.split(icl))
    return args


async def icli():

    def connect_callback(task_t):
        drone = task_t.result()
        connected_drones[task_t.get_name()] = drone
        running_tasks.discard(task_t)
        print(f"Connected to {len(connected_drones)} drones!")

    async def arm_drone(name):
        print(f"Arming drone {name}")
        drone = connected_drones[name]
        await _drone_arm(drone)
        print(f"Armed drone!")

    def stop():
        loop.stop()
        for name, drone in connected_drones.values():
            drone.override_action(actions.land)
            drone.close_conn()
            drone.join()
            connected_drones.pop(name)

    def status():
        print("Drone Status")
        print("Name\tArmed")
        print("=====")
        for name in connected_drones:
            drone = connected_drones[name]
            print("\t".join([name, str(drone.arm)]))
        print("\n=====")

    connected_drones: Dict[str, Craft] = {}
    running_tasks = set()
    loop = asyncio.get_running_loop()

    parser = argparse.ArgumentParser(description="Interactive command line interface to connect and control multiple drones")
    subparsers = parser.add_subparsers(title="command", description="Command to execute.", dest="command")
    connect_parser = subparsers.add_parser("connect")
    connect_parser.add_argument("drone", help="Name for the drone.")
    connect_parser.add_argument("address", help="Connection string. Something like udp://:14550")
    arm_parser = subparsers.add_parser("arm")
    arm_parser.add_argument("drone", help="Drone to arm")
    status_parser = subparsers.add_parser("status")
    exit_parser = subparsers.add_parser("exit")

    while True:
        # TODO: Catch Exceptions
        args = await get_command(parser)
        if args.command == "connect":
            task = loop.create_task(connect_to_drone(args.drone, args.address), name=args.drone)
            running_tasks.add(task)
            task.add_done_callback(connect_callback)
        if args.command == "arm":
            task = loop.create_task(arm_drone(args.drone))
            running_tasks.add(task)
        if args.command == "status":
            status()
        if args.command == "exit":
            stop()
            break


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
    #full_loop()
    asyncio.run(icli())






