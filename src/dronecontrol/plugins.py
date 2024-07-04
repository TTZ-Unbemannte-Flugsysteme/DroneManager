""" Class for extra, loadable plugins.

Plugins extend the functionality of DroneManager or Drone Classes by providing extra functions. They can also register
their own commands to the CLI.
"""
import asyncio
from abc import ABC


class Plugin(ABC):
    """ Generic plugin class.

    The attribute cli_commands is called by the DroneManager CLI (and could be called by other UIs) to populate their
    interfaces. This is a dictionary with callable functions as values and human-readable names as keys. In DroneManager
    the names are used together with the class prefix to determine the command, while the signature of the function is
    used to populate the CLI parser.
    The attribute background_functions should list coroutines that will run indefinitely, for example those
    polling for status updates from a camera. They will be started during construction of the class object, usually
    when the module is loaded. Note that these must be coroutines.

    """

    PREFIX = "abs"

    def __init__(self, logger):
        self.logger = logger.getChild(self.__class__.__name__)
        self.cli_commands = {}
        self.background_functions = []
        self._running_tasks = []

    def start_background_functions(self):
        for coro in self.background_functions:
            self._running_tasks.append(asyncio.create_task(coro))


class ManagerPlugin(Plugin):
    """ Plugins for the drone manager class. Conceptually these handle functions for multiple drones, for example,
    multiple drones interacting with a single mapping system.

    """

    PREFIX = "mng"

    def __init__(self, logger, dm):
        super().__init__(logger)
        self.dm = dm


class DronePlugin(Plugin):
    """ Plugins for drone classes. These should address things for single drones, such as gimbals or cameras.

    """

    PREFIX = "drn"

    def __init__(self, logger, drone):
        super().__init__(logger)
        self.drone = drone

