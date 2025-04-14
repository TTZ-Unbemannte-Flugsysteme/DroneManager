import abc
import asyncio
from pathlib import Path
import importlib
import inspect

from dronecontrol.plugin import Plugin


class MissionPlugin(Plugin):
    PREFIX = "mission"

    def __init__(self, dm, logger):
        super().__init__(dm, logger)
        self.cli_commands = {
            "load": self.load,
            "status": self.status,
        }
        self.missions: dict[Mission] = {}

    async def start(self):
        pass

    async def close(self):
        """ Stop all missions.

        :return:
        """
        await asyncio.gather(*[mission.close() for mission in self.missions])

    def mission_options(self):
        # Go through every file in plugins folder
        _base_dir = Path(__file__).parent
        _plugin_dir = _base_dir.joinpath("missions")
        modules = [name.stem for name in _plugin_dir.iterdir()
                   if name.is_file() and name.suffix == ".py" and not name.stem.startswith("_")]
        return modules

    def _get_mission_class(self, module) -> None | type:
        try:
            plugin_mod = importlib.import_module("." + module, "dronecontrol.missions")
            plugin_classes = [member[1] for member in inspect.getmembers(plugin_mod, inspect.isclass)
                              if issubclass(member[1], Mission)
                              and not member[1] is Mission]  # Strict subclass check
            if len(plugin_classes) != 1:
                return None
            return plugin_classes[0]
        except ImportError as e:
            self.logger.error(f"Couldn't load mission {module} due to a python import error!")
            self.logger.debug(repr(e), exc_info=True)

    async def load(self, mission_module: str, name: str):
        """ Load a new mission, which work like plugins with the name taking the role of the prefix.

        :return:
        """
        try:
            if hasattr(self.dm, name):
                raise RuntimeError(
                    f"Can't load mission {mission_module} under {name} due to name collision with an existing "
                    f"attribute! Give the mission a different name.")
            if name in self.missions:
                self.logger.warning(f"A mission with name {name} is already loaded!")
                return False
            if mission_module not in self.mission_options():
                self.logger.warning(f"No mission '{mission_module}' found!")
                return False
            self.logger.info(f"Loading mission {mission_module} under name {name}...")
            mission = None
            try:
                plugin_class = self._get_mission_class(mission_module)
                if not plugin_class:
                    self.logger.error(
                        f"Module {mission_module} contains no or multiple missions, which is currently not supported!")
                    return False
                mission = plugin_class(name, self.dm, self.logger)
                setattr(self.dm, name, mission)
                self.missions[name] = mission
                await mission.start()
            except Exception as e:
                self.logger.error(f"Couldn't load mission {mission_module} due to an exception!")
                self.logger.debug(repr(e), exc_info=True)
                if mission is not None:
                    await mission.close()
                if hasattr(self.dm, name):
                    delattr(self.dm, name)
                if name in self.missions:
                    self.missions.pop(name)
                return False
            self.logger.debug(f"Performing callbacks for plugin loading...")
            for func in self.dm._on_plugin_load_coros:
                res = await asyncio.create_task(func(name, mission))
                if isinstance(res, Exception):
                    self.logger.warning("Couldn't perform a callback for this plugin!")
            self.logger.info(f"Completed loading mission {mission} under name {name}!")
        except Exception as e:
            self.logger.error(repr(e), exc_info=True)
        return True

    async def status(self):
        """ Status of running missions and missions that could be loaded."""
        self.logger.info("Status of running missions:")
        for mission in self.missions:
            mission.status()
        if len(self.missions) == 0:
            self.logger.info("No running missions!")
        self.logger.info(f"Available missions for loading: {self.mission_options()}")


class Mission(Plugin, abc.ABC):
    PREFIX = "YOUDIDSOMETHINGWRONG"

    def __init__(self, name, dm, logger):
        self.PREFIX = name
        super().__init__(dm, logger)
        self.cli_commands = {
            "reset": self.reset,
            "status": self.status,
            "add": self.add_drones,
            "remove": self.remove_drones,
        }

    async def start(self):
        """ This function is called when the mission is loaded to start all the necessary processes asynchronously.
        It is NOT a "start this mission" function."""
        pass

    async def close(self):
        for task in self._running_tasks:
            if isinstance(task, asyncio.Task):
                task.cancel()

    @abc.abstractmethod
    async def reset(self):
        """ Resets the mission back to the initial position.

        Keep safety in mind when this requires moving drones."""
        pass

    @abc.abstractmethod
    def status(self):
        """ Should write information about the current status of the mission to the logger under INFO."""
        pass

    @abc.abstractmethod
    async def add_drones(self, names: list[str]):
        """ Add drones to the mission. Implementations should check that the drones are capable and meet mission
        requirements."""
        pass

    @abc.abstractmethod
    async def remove_drones(self, names: list[str]):
        """ Remove drones from the mission. Implementations must take measures to prevent missions from running with
        too few drones"""
        pass

    @abc.abstractmethod
    async def mission_ready(self, drone: str):
        """ Check whether any given drone is ready to keep going, i.e. is still connected etc. """
        pass
