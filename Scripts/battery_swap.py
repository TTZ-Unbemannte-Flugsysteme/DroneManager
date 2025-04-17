from dronecontrol.dronemanager import DroneManager
from dronecontrol.drone import DroneMAVSDK
import asyncio
import time
import math

	
async def main():
    dm = DroneManager(DroneMAVSDK, log_to_console=True)

    connected_d1 = await dm.connect_to_drone("luke", None, None, "udp://:14540", timeout=30)
    connected_d2 = await dm.connect_to_drone("tycho", None, None, "udp://:14541", timeout=30)
    connected_d3 = await dm.connect_to_drone("jaina", None, None, "udp://:14542", timeout=30)  

    forward_dist = 5
    total_length = 30
    total_width = 10
    hight = 2

    await go_to_start(dm)
    #await search_pattern_1(dm, total_length, total_width, forward_dist, hight)
    await search_pattern_2(dm, total_length, hight)



async def go_to_start(dm):
    # Move drones to starting position
    await dm.arm(["luke"])
    await dm.takeoff(["luke"])
    await dm.fly_to("luke", 0, 0, -2, 0, tol=0.25, schedule=True)
    await dm.land(["luke"])
    await dm.disarm(["luke"])
    await dm.arm(["tycho"])
    await dm.takeoff(["tycho"])
    await dm.fly_to("tycho", 0, 5, -2, 0, tol=0.25, schedule=True)
    await dm.land(["tycho"])
    await dm.disarm(["tycho"])
    await dm.arm(["jaina"])
    await dm.takeoff(["jaina"])
    await dm.fly_to("jaina", 0, 10, -2, 0, tol=0.25, schedule=True)
    await dm.land(["jaina"])
    await dm.disarm(["jaina"])

async def search_pattern_1(dm, total_length, total_width, forward_dist, hight):
    await dm.arm(["luke"])
    await dm.takeoff(["luke"])
    for repeat in range(math.floor(total_length/forward_dist)):
        current_pos = dm.drones["luke"].position_ned
        # fly right
        await dm.fly_to("luke", (current_pos[0] + forward_dist), current_pos[1], -hight, 0, tol=0.25, schedule=True)
        current_pos = dm.drones["luke"].position_ned
        if repeat % 2 == 0:
            # fly up 
            await dm.fly_to("luke", current_pos[0], (current_pos[1] + total_width), -hight, 0, tol=0.25, schedule=True)
        else:
            # fly down
            await dm.fly_to("luke", current_pos[0], (current_pos[1] - total_width), -hight, 0, tol=0.25, schedule=True)
    await dm.fly_to("luke", 0, 0, -2, 0, tol=0.25, schedule=True)
    await dm.land(["luke"])
    await dm.disarm(["luke"])

async def search_pattern_2(dm, total_length, hight):
    armed = asyncio.create_task(dm.arm(["luke", "tycho", "jaina"], schedule=True))
    await armed
    take_off = asyncio.create_task(dm.takeoff(["luke", "tycho", "jaina"], schedule=True))
    await take_off
    current_pos_luke = dm.drones["luke"].position_ned
    current_pos_tycho = dm.drones["tycho"].position_ned
    current_pos_jaina = dm.drones["jaina"].position_ned
    move = []
    move.append(asyncio.create_task(dm.move("luke", total_length, current_pos_luke[1], 0, 0, no_gps=False, tol=0.25, schedule=True)))
    move.append(asyncio.create_task(dm.move("tycho", total_length, current_pos_tycho[1], 0, 0, no_gps=False, tol=0.25, schedule=True)))
    move.append(asyncio.create_task(dm.move("jaina", total_length, current_pos_jaina[1], 0, 0, no_gps=False, tol=0.25, schedule=True)))
    await asyncio.gather(*move)
    move = []
    move.append(asyncio.create_task(dm.move("luke", -total_length, current_pos_luke[1], 0, 0, no_gps=False, tol=0.25, schedule=True)))
    move.append(asyncio.create_task(dm.move("jaina", -total_length, current_pos_jaina[1], 0, 0, no_gps=False, tol=0.25, schedule=True)))
    await asyncio.gather(*move)
    current_pos_tycho = dm.drones["tycho"].position_ned
    await dm.orbit("tycho", 2, 1, current_pos_tycho[1], current_pos_tycho[2], 2)

if __name__=="__main__":
	asyncio.run(main())
