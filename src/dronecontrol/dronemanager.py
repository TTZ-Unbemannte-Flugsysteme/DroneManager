import asyncio
import datetime
import os
import socket
import sys
from asyncio.exceptions import TimeoutError, CancelledError

from dronecontrol.drone import Drone, parse_address
from dronecontrol.utils import common_formatter, get_free_port

import logging

from dronecontrol.gimbal import GimbalPlugin
#from dronecontrol.formations import FormationsPlugin

# TODO: Plugin Discovery
PLUGINS = {
    "gimbal": GimbalPlugin,
    #"formations": FormationsPlugin,
}


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
    # TODO: Catch plugin command errors somehow: Maybe add a function that wraps all calls to plugin commands in a
    #  separate function that awaits them and does error handling? Alternatively, the CLI function should do some final
    #  error catching somehow, maybe the same way?

    def __init__(self, drone_class, logger=None, log_to_console=False, console_log_level=logging.DEBUG):
        self.drone_class = drone_class
        self.drones: dict[str, Drone] = {}
        self.running_tasks = set()
        # self.drones acts as the list/manager of connected drones, any function that writes or deletes items should
        # protect those writes/deletes with this lock. Read only functions can ignore it.
        self.drone_lock = asyncio.Lock()

        self._on_drone_removal_coros = set()
        self._on_drone_connect_coros = set()

        self._on_plugin_load_coros = set()
        self._on_plugin_unload_coros = set()
        self.plugins = set()

        self.system_id = 246
        self.component_id = 190

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

        if log_to_console:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(console_log_level)
            console_handler.setFormatter(common_formatter)
            self.logger.addHandler(console_handler)

    async def connect_to_drone(self,
                               name: str,
                               mavsdk_server_address: str | None,
                               mavsdk_server_port: int,
                               drone_address: str,
                               timeout: float):
        try:
            scheme, parsed_addr, parsed_port = parse_address(string=drone_address)
        except Exception as e:
            self.logger.warning("Couldn't connect due to an exception parsing the address")
            self.logger.debug(repr(e), exc_info=True)
            return False
        if scheme == "serial":
            self.logger.info(f"Trying to connect to drone {name} @{scheme}://{parsed_addr} with baud {parsed_port}")
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
                    mavsdk_server_port = get_free_port()
                # Check that we don't already have this drone connected.
                for other_name in self.drones:
                    other_drone = self.drones[other_name]
                    _, other_addr, other_port = parse_address(string=other_drone.drone_addr)
                    if parsed_addr == other_addr and parsed_port == other_port:
                        self.logger.warning(f"{other_name} is already connected to drone with address {drone_address}.")
                        return False
                drone = self.drone_class(name, mavsdk_server_address, mavsdk_server_port)
                try:
                    connected = await asyncio.wait_for(drone.connect(drone_address, system_id=self.system_id,
                                                                     component_id=self.component_id), timeout)
                except (CancelledError, TimeoutError, OSError, socket.gaierror, AssertionError) as e:
                    if isinstance(e, CancelledError):
                        self.logger.info(f"Aborting connection attempt to {name}")
                    elif isinstance(e, TimeoutError):
                        self.logger.warning(f"Connection attempts to {name} timed out!")
                    elif isinstance(e, OSError) or isinstance(e, socket.gaierror):
                        self.logger.error(f"Address error, probably due to invalid address")
                        self.logger.debug(f"{repr(e)}", exc_info=True)
                    else:
                        self.logger.error("Connection failed due to an exception")
                        self.logger.debug(f"{repr(e)}", exc_info=True)
                    if isinstance(connected, asyncio.Task):
                        connected.cancel()
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
            except TimeoutError:
                self.logger.warning(f"Connection attempts to {name} timed out!")
                await self._remove_drone_object(name, drone)
                return False
            except Exception as e:
                self.logger.error("Couldn't connect to the drone due to an exception: ", repr(e))
                self.logger.debug(repr(e), exc_info=True)
                if drone is not None:
                    await self._remove_drone_object(name, drone)
                return False

    async def disconnect(self, names, force=False):
        self.logger.info(f"Disconnecting {names} ...")
        async with self.drone_lock:
            for name in names:
                try:
                    drone = self.drones[name]
                except KeyError:
                    continue
                try:
                    disconnected = await drone.disconnect(force=force)
                except Exception as e:
                    disconnected = False
                    self.logger.error(f"An error occurred during disconnect for {name}")
                    self.logger.debug(repr(e), exc_info=True)
                if disconnected:
                    await self._remove_drone_object(name, drone)
                    self.logger.info(f"Disconnected {name}")

    async def _single_drone_action(self, action, name, start_string, *args, schedule=False, **kwargs):
        return await self._multiple_drone_action(action, [name], start_string, *args, schedule=schedule, **kwargs)

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
            self.logger.debug(repr(e), exc_info=True)

    async def _multiple_drone_multiple_params_action(self, action, names, start_string, *args, schedule=False, **kwargs):
        # TODO: Allow a list of drones and args, kwargs and scheduleing params and unpack these for each drone.
        pass

    async def arm(self, names, schedule=False):
        return await self._multiple_drone_action(self.drone_class.arm, names,
                                                 "Arming drone(s) {}.", schedule=schedule)

    async def disarm(self, names, schedule=False):
        return await self._multiple_drone_action(self.drone_class.disarm, names,
                                                 "Disarming drone(s) {}.", schedule=schedule)

    async def takeoff(self, names, altitude=2.0, schedule=False):
        return await self._multiple_drone_action(self.drone_class.takeoff, names,
                                                 "Takeoff for Drone(s) {}.", altitude, schedule=schedule)

    async def change_flightmode(self, names, flightmode, schedule=False):
        await self._multiple_drone_action(self.drone_class.change_flight_mode,
                                          names,
                                          "Changing flightmode for drone(s) {} to " + flightmode + ".",
                                          flightmode, schedule=schedule)

    async def land(self, names, schedule=False):
        await self._multiple_drone_action(self.drone_class.land, names,
                                          "Landing drone(s) {}.", schedule=schedule)

    def pause(self, names):
        self.logger.info(f"Pausing drone(s) {names}")
        for name in names:
            self.drones[name].pause()

    def resume(self, names):
        self.logger.info(f"Resuming task execution for drone(s) {names}")
        for name in names:
            self.drones[name].resume()

    async def fly_to(self, name, x, y, z, yaw, tol=0.25, schedule=True):
        """Fly the drone to an absolute position in the local coordinate system.

        :param name:
        :param x:
        :param y:
        :param z:
        :param yaw:
        :param tol:
        :param schedule:
        :return:
        """
        await self._single_drone_action(self.drone_class.fly_to, name,
                                        f"Flying to {x, y, z} with heading {yaw} and tolerance {tol}",
                                        schedule=schedule,
                                        x=x, y=y, z=z, yaw=yaw, tolerance=tol)

    async def fly_to_gps(self, name, lat, long, alt, yaw, tol=0.25, schedule=True):
        """ Fly the drone to an absolute GPS position.

        :param name:
        :param lat:
        :param long:
        :param alt:
        :param yaw:
        :param tol:
        :param schedule:
        :return:
        """
        await self._single_drone_action(self.drone_class.fly_to, name,
                                        f"Flying to {lat, long, alt} with heading {yaw} and tolerance {tol}",
                                        schedule=schedule,
                                        lat=lat, long=long, amsl=alt, yaw=yaw, tolerance=tol)

    async def fly_to_point(self, name, waypoint, tol=0.25, schedule=True):
        await self._single_drone_action(self.drone_class.fly_to, name, f"Flying to waypoint {waypoint}",
                                        schedule=schedule, waypoint=waypoint, tolerance=tol)

    async def move(self, name, x, y, z, yaw, no_gps=False, tol=0.25, schedule=True):
        """ Move the drone by x, y, z meters. Which coordinate system is used depends on no_gps.

        :param name:
        :param x: How many meters to move along the "x" axis. For local coordinates, this is usually either North or
                  Forward. For GPS, it is north.
        :param y: How many meters to move along the "y" axis. For local coordinates, this is usually either East or
                  Right. For GPS, it is east.
        :param z: How many meters to move along the "z" axis. For local coordinates, this is usually down.
                  For GPS, this is up.
        :param yaw:
        :param no_gps: If True, use the local coordinate system, otherwise use GPS.
        :param tol:
        :param schedule:
        :return:
        """
        await self._single_drone_action(self.drone_class.move, name,
                                        f"Moving by {x, y, z} and heading {yaw} and tolerance {tol}",
                                        x, y, z, yaw,
                                        schedule=schedule,
                                        use_gps=not no_gps, tolerance=tol)

    async def orbit(self, name, radius, velocity, center_lat, center_long, amsl):
        try:
            await self.drones[name].orbit(radius, velocity, center_lat, center_long, amsl)
            self.logger.info(f"{name} flying in a circle.")
        except KeyError:
            self.logger.warning(f"No drone named {name}!")
        except Exception as e:
            self.logger.error(repr(e))

    async def action_stop(self, names):
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
            return result
        except KeyError:
            pass

    async def _kill_drone(self, name):
        try:
            drone = self.drones[name]
            result = await drone.kill()
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
            await drone.stop_execution()
            del drone

    def add_remove_func(self, func):
        self._on_drone_removal_coros.add(func)

    def add_connect_func(self, func):
        self._on_drone_connect_coros.add(func)

# PLUGINS ##############################################################################################################

    def plugin_options(self):
        return PLUGINS.keys()

    def currently_loaded_plugins(self):
        return self.plugins

    def add_plugin_load_func(self, func):
        self._on_plugin_load_coros.add(func)

    def add_plugin_unload_func(self, func):
        self._on_drone_connect_coros.add(func)

    async def load_plugin(self, plugin_name):
        # TODO: Check to prevent collision between plugin name and existing attributes.
        if plugin_name in self.plugins:
            self.logger.warning(f"Plugin {plugin_name} already loaded!")
            return False
        if plugin_name not in PLUGINS:
            self.logger.warning(f"No plugin '{plugin_name}' found!")
            return False
        self.logger.info(f"Loading plugin {plugin_name}...")
        try:
            self.plugins.add(plugin_name)
            plugin = PLUGINS[plugin_name](self, self.logger)
            setattr(self, plugin_name, plugin)
            await plugin.start()
        except Exception as e:
            self.logger.error(f"Couldn't load plugin {plugin_name} due to an exception!")
            self.logger.debug(repr(e), exc_info=True)
            return False
        self.logger.debug(f"Performing callbacks for plugin loading...")
        for func in self._on_plugin_load_coros:
            res = await asyncio.create_task(func(plugin_name, plugin))
            if isinstance(res, Exception):
                self.logger.warning("Couldn't perform a callback for this plugin!")
        self.logger.info(f"Completed loading Plugin {plugin_name}!")
        return True

    async def unload_plugin(self, plugin_name):
        if plugin_name not in self.plugins:
            self.logger.warning(f"No plugin named {plugin_name} loaded!")
            return False
        self.logger.info(f"Unloading plugin {plugin_name}")
        self.plugins.remove(plugin_name)
        plugin = getattr(self, plugin_name)
        unload_tasks = set()
        for func in self._on_plugin_unload_coros:
            unload_tasks.add(func(plugin_name, plugin))
        await asyncio.gather(*unload_tasks, return_exceptions=True)
        await plugin.close()
        delattr(self, plugin_name)

# Camera Stuff #########################################################################################################

    async def prepare(self, name):
        await self.drones[name].prepare()

    async def get_settings(self, name):
        await self.drones[name].get_settings()

    async def take_picture(self, name):
        await self.drones[name].take_picture()

    async def start_video(self, name):
        await self.drones[name].start_video()

    async def stop_video(self, name):
        await self.drones[name].stop_video()

    async def set_zoom(self, name, zoom):
        await self.drones[name].set_zoom(zoom)
