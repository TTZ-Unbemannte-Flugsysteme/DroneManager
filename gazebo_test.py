import asyncio
from mavsdk import System

async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Arming drone...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(5)  # Hover for 5 seconds

    print("Flying to a new position...")
    await drone.action.goto_location(47.397606, 8.543060, 10, 0)

    await asyncio.sleep(10)

    print("Landing...")
    await drone.action.land()

    print("Mission complete!")

asyncio.run(run())
