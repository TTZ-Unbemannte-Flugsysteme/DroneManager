from dronecontrol.dronemanager import DroneManager
from dronecontrol.drone import DroneMAVSDK
import asyncio
import time

dm = DroneManager(DroneMAVSDK)
async def main():
	connected = await dm.connect_to_drone("luke", None, None, "udp://:14540", timeout=30)

	if connected:
		# Schedule a bunch of actions immediately to be executed on the drones schedule.
		armed = asyncio.create_task(dm.arm(["luke"], schedule=True))	# Note that many dm functions work for multiple drones and expect a list of drones
		take_off = asyncio.create_task(dm.takeoff(["luke"], schedule=True))
		landed = asyncio.create_task(dm.land(["luke"], schedule=True))
		disarmed = asyncio.create_task(dm.disarm(["luke"], schedule=True))
		print("Done scheduling")
		await disarmed # Wait for the last task to be executed.
	


		# Perform actions and wait before continuing to next code line (schedule parameter irrelevant)
		await dm.arm(["luke"])
		print("Armed")
		await dm.takeoff(["luke"])
		print("Took off")
		await dm.land(["luke"])
		print("Landed")



		# Perform an action and cancel it
		await dm.arm(["luke"])
		await dm.takeoff(["luke"])
		fly_to = asyncio.create_task(dm.fly_to("luke", 100, 0, -3, 0, schedule=True)) # We will interrupt this
		asyncio.sleep(1)
		await dm.fly_to("luke", 0, 0, -3, 0) # Interrupts the previous fly_to command
		await dm.land(["luke"])



		# Could also do this through the drone object. These are now functions of DroneMAVSDK, not DroneManager!
		luke = dm.drones["luke"]
		armed = luke.execute_task(luke.arm()) # Using task scheduler. Return value is a future, which can be awaited.
		took_off = luke.schedule_task(luke.takeoff())
		asyncio.sleep(1)	
		await luke.land()	# Can call functions directly, but this interferes with task_scheduler and can lead to issues with exceptions, not recommended!





async def battery_swap():
	# Two drones: luke and tycho. Luke currently observing, tycho landed and waiting

	observing_drone = "luke"
	stand_by_drone = "tycho"
	do_swapping = True	
	observe_altitude = -2
	exchange_altitude = -3

	while do_swapping:
		# Check observing drone battery
		battery_percentage = dm.drones[observing_drone].batteries[0].remaining
		if battery_percentage < 40:
			# Do the whole swap thing, this would probably be a separate function in the real deal
			# Determine a nearby position somehow, will just send to same position for now

			# current positions of both drones
			cur_pos_stand_by = dm.drones[stand_by_drone].position_ned
			cur_pos_observe = dm.drones[observing_drone].position_ned

			# Determine waypoints for standby drone
			home_pos_stand_by = cur_pos_stand_by.copy()	
			home_pos_stand_by[2] = exchange_altitude		# Position above landing spot for stand-by drone
			home_pos_observe = cur_pos_stand_by.copy()		
			home_pos_observe[2] = observe_altitude			# Position above landing spot for observing drone
			target_pos_stand_by = cur_pos_observe.copy()		
			target_pos_stand_by[2] = exchange_altitude		# Position above observation spot for stand-by drone

			# Start moving drones, takeoff stand-by drone
			await dm.arm([stand_by_drone])
			await dm.takeoff([stand_by_drone])

			# Fly stand-by drone to above observing drone
			await dm.fly_to(stand_by_drone, *home_pos_stand_by)
			await dm.fly_to(stand_by_drone, *target_pos_stand_by)
			# Fly observing drone back to home position
			await dm.fly_to(observing_drone, *home_pos_observe)

			# in parallel: Land old observing drone and lower current observing drone to proper height.
			tasks = []
			tasks.append(asyncio.create_task(dm.land([observing_drone], schedule=True)))
			tasks.append(asyncio.create_task(dm.disarm([observing_drone], schedule=True)))
			tasks.append(asyncio.create_task(dm.fly_to(stand_by_drone, *cur_pos_observe)))
			await asyncio.gather(*tasks)

			# Swap labels for observing and stand-by drone
			temp_drone = observing_drone
			observing_drone = stand_by_drone
			stand_by_drone = temp_drone
			


if __name__=="__main__":
	asyncio.run(main())
