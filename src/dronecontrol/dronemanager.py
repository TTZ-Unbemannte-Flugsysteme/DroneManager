import asyncio
import datetime
import os
import socket
from asyncio.exceptions import TimeoutError, CancelledError
import random

from dronecontrol.drone import Drone, parse_address

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


class DroneManager:
    # TODO: Figure out how to get voxl values from the drone
    # TODO: Better error handling for the multi_action tasks
    # TODO: Handle MAVSDK crashes

    def __init__(self, drone_class, logger=None):
        self.drone_class = drone_class
        self.drones: dict[str, Drone] = {}
        self.running_tasks = set()
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self.drone_lock = asyncio.Lock()

        self._on_drone_removal_coros = set()
        self._on_drone_connect_coros = set()

        if logger is None:
            self.logger = logging.getLogger("Manager")
            self.logger.setLevel(logging.DEBUG)
            filename = f"manager_{datetime.datetime.now()}"
            filename = filename.replace(":", "_").replace(".", "_") + ".log"
            logdir = os.path.abspath("./logs")
            os.makedirs(logdir, exist_ok=True)
            file_handler = logging.FileHandler(os.path.join(logdir, filename))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(common_formatter)
            self.logger.addHandler(file_handler)
        else:
            self.logger = logger

    @property
    def used_drone_addrs(self):
        return [drone.drone_addr for drone in self.drones.values()]

    async def connect_to_drone(self,
                               name: str,
                               mavsdk_server_address: str | None,
                               mavsdk_server_port: int,
                               drone_address: str,
                               timeout: float, compid=190):
        try:
            scheme, parsed_addr, parsed_port = parse_address(string=drone_address)
        except Exception as e:
            self.logger.info(repr(e))
            return False
        if scheme == "serial":
            self.logger.info(f"Trying to connect to drone {name} @{scheme}://{parsed_addr}")
        else:
            self.logger.info(f"Trying to connect to drone {name} @{scheme}://{parsed_addr}:{parsed_port}")
        drone = None
        async with self.drone_lock:
            try:
                # Ensure that for each drone there is a one-to-one-to-one relation between name, mavsdk port and drone
                if name in self.drones:
                    self.logger.warning(f"A drone called {name} already exists. Each drone must have a unique name.")
                    return False
                if not mavsdk_server_address:
                    used_ports = [drone.server_port for drone in self.drones.values()]
                    used_compids = [drone.compid for drone in self.drones.values()]
                    while mavsdk_server_port in used_ports:
                        mavsdk_server_port = random.randint(10000, 60000)
                    while compid in used_compids:
                        compid += 1
                # Check that we don't already have this drone connected.
                for other_name in self.drones:
                    other_drone = self.drones[other_name]
                    _, other_addr, other_port = parse_address(string=other_drone.drone_addr)
                    if parsed_addr == other_addr and parsed_port == other_port:
                        self.logger.warning(f"{other_name} is already connected to drone with address {drone_address}.")
                        return False
                drone = self.drone_class(name, mavsdk_server_address, mavsdk_server_port, compid=compid)
                try:
                    connected = await asyncio.wait_for(drone.connect(drone_address), timeout)
                except (TimeoutError, CancelledError):
                    self.logger.warning(f"Connection attempts to {name} timed out!")
                    await self._remove_drone_object(name, drone)
                    return False
                except (OSError, socket.gaierror) as e:
                    self.logger.info(f"Address error, probably due to invalid address")
                    self.logger.debug(f"{repr(e)}", exc_info=True)
                    await self._remove_drone_object(name, drone)
                    return False
                except AssertionError as e:
                    self.logger.info("Connection failed, we only support UDP connection protocol at the moment.")
                    self.logger.debug(f"{repr(e)}", exc_info=True)
                    await self._remove_drone_object(name, drone)
                    return False
                if connected:
                    self.logger.info(f"Connected to {name}!")
                    self.drones[name] = drone
                    for func in self._on_drone_connect_coros:
                        try:
                            await asyncio.create_task(func(name, drone))
                        except Exception as e:
                            self.logger.error(f"Failed post-connection process: {repr(e)}")
                            self.logger.debug(repr(e), exc_info=True)
                            await self._remove_drone_object(name, drone)
                            return False
                    return True
                else:
                    self.logger.warning(f"Failed to connect to drone {name}!")
                    await self._remove_drone_object(name, drone)
                    return False
            except (TimeoutError, CancelledError):
                self.logger.warning(f"Connection attempts to {name} timed out!")
                if drone is not None:
                    await self._remove_drone_object(name, drone)
                return False
            except Exception as e:
                self.logger.debug(repr(e), exc_info=True)
                if drone is not None:
                    await self._remove_drone_object(name, drone)
                return False

    async def _multiple_drone_action(self, action, names, start_string, *args, schedule=False, **kwargs):
        try:
            coros = [action(self.drones[name], *args, **kwargs) for name in names]
            if schedule:
                self.logger.info("Queuing action: " + start_string.format(names))
                results = [self.drones[name].schedule_task(coros[i]) for i, name in enumerate(names)]
            else:
                self.logger.info(start_string.format(names))
                results = [self.drones[name].execute_task(coros[i]) for i, name in enumerate(names)]
            results = await asyncio.gather(*results, return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    self.logger.error(f"Drone {names[i]} failed due to: {str(result)}")
            return results
        except KeyError:
            self.logger.warning("No drones named {}!".format([name for name in names if name not in self.drones]))
        except Exception as e:
            self.logger.error(repr(e))

    async def arm(self, names, schedule=False):
        return await self._multiple_drone_action(self.drone_class.arm, names,
                                                 "Arming drone(s) {}.", schedule=schedule)

    async def disarm(self, names, schedule=False):
        return await self._multiple_drone_action(self.drone_class.disarm, names,
                                                 "Disarming drone(s) {}.", schedule=schedule)

    async def takeoff(self, names, schedule=False):
        return await self._multiple_drone_action(self.drone_class.takeoff, names,
                                                 "Takeoff for Drone(s) {}.", schedule=schedule)

    async def change_flightmode(self, names, flightmode, schedule=False):
        await self._multiple_drone_action(self.drone_class.change_flight_mode,
                                          names,
                                          "Changing flightmode for drone(s) {} to " + flightmode + ".",
                                          flightmode, schedule=schedule)

    async def land(self, names, schedule=False):
        await self._multiple_drone_action(self.drone_class.land, names,
                                          "Landing drone(s) {}.", schedule=schedule)

    async def pause(self, names):
        self.logger.info(f"Pausing drone(s) {names}")
        for name in names:
            self.drones[name].pause()

    async def resume(self, names):
        self.logger.info(f"Resuming task execution for drone(s) {names}")
        for name in names:
            self.drones[name].resume()

    async def fly_to(self, name, x, y, z, yaw, tol=0.25, schedule=True):
        self.logger.info(f"Queueing move to {x, y, z, yaw} for {name}.")
        try:
            coro = self.drones[name].fly_to(x=x, y=y, z=z, yaw=yaw, tolerance=tol)
            if schedule:
                result = self.drones[name].schedule_task(coro)
            else:
                result = self.drones[name].execute_task(coro)
            await result
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))
            self.logger.debug(repr(e), exc_info=True)

    async def fly_to_gps(self, name, lat, long, alt, yaw, tol=0.25):
        self.logger.info(f"Queuing move to {(lat, long, alt)} for  {name}")
        try:
            coro = self.drones[name].fly_to(lat=lat, long=long, amsl=alt, yaw=yaw, tolerance=tol)
            result = self.drones[name].schedule_task(coro)
            await result
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))

    async def move(self, name, x, y, z, yaw, no_gps=False, tol=0.25):
        self.logger.info(f"Queuing move by {(x, y, z)} for  {name}")
        try:
            coro = self.drones[name].move(x, y, z, yaw, use_gps=not no_gps, tolerance=tol)
            result = self.drones[name].schedule_task(coro)
            await result
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))

    async def orbit(self, name, radius, velocity, center_lat, center_long, amsl):
        try:
            await self.drones[name].orbit(radius, velocity, center_lat, center_long, amsl)
            self.logger.info(f"{name} flying in a circle.")
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))

    async def action_stop(self, names):
        async with self.drone_lock:
            if not names:
                self.logger.info("Stopping all drones!")
            else:
                self.logger.info(f"Stopping {names}")
            drones_to_stop = names if names else list(self.drones.keys())
            results = await asyncio.gather(*[self._stop_drone(name) for name in drones_to_stop], return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    self.logger.critical(f"During stopping, drone {drones_to_stop[i]} encountered an exception "
                                         f"{repr(result)}!", exc_info=True)
            return results

    async def kill(self, names):
        async with self.drone_lock:
            if not names:
                self.logger.info("Killing all drones!")
            else:
                self.logger.info(f"Killing {names}")
            drones_to_stop = names if names else list(self.drones.keys())
            results = await asyncio.gather(*[self._kill_drone(name) for name in drones_to_stop], return_exceptions=True)
            return results

    async def _stop_drone(self, name):
        try:
            drone = self.drones[name]
            result = await drone.stop()
            await self._remove_drone_object(name, drone)
            return result
        except KeyError:
            pass

    async def _kill_drone(self, name):
        try:
            drone = self.drones[name]
            result = await drone.kill()
            await self._remove_drone_object(name, drone)
            return result
        except KeyError:
            pass

    async def _remove_drone_object(self, name, drone: Drone):
        try:
            self.drones.pop(name)
        except KeyError:
            pass
        for func in self._on_drone_removal_coros:
            try:
                await asyncio.create_task(func(name))
            except Exception as e:
                self.logger.debug(repr(e), exc_info=True)
        if drone is not None:
            try:
                drone.should_stop.set()
                drone.__del__()
                del drone
            except Exception as e:
                self.logger.error(repr(e), exc_info=True)

    def add_remove_func(self, func):
        self._on_drone_removal_coros.add(func)

    def add_connect_func(self, func):
        self._on_drone_connect_coros.add(func)
