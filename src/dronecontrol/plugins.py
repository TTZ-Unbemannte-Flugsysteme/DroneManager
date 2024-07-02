""" Class for extra, loadable plugins.

Plugins extend the functionality of DroneManager by providing extra functions. They can also register their own commands
for the CLI.
"""

from abc import ABC


class Plugin(ABC):

    PREFIX = "ABS"

    def __init__(self, dm, logger):
        self.dm = dm
        self.logger = logger.getChild(self.__class__.__name__)
        self.dm.plugins.append(self)
        self._cli_commands = {}

    @property
    def cli_commands(self):
        return self._cli_commands

    def add_cli_command(self, command, func, help="", overwrite=False):
        """ Add a command to the CLI.

        :param command: Name of the command. Will be prefixed by the plugin, i.e. for the abstract class the command in
            the CLI would be "ABS-<name>"
        :param func: The function called when the command is called
        :param help: Help string for the command
        :return:
        """
        if command in self._cli_commands and not overwrite:
            raise RuntimeError(f"{self.__class__.__name__} tried to create CLI command {command}, but it already "
                               f"exists!")
        self._cli_commands[command] = {
            "func": func,
            "help": help,
            "args": []
        }
        pass

    def add_cli_command_arg(self, command, arg, **kwargs):
        """ Add an argument to a CLI command. Only works on commands from this plugin

        :param command: Which command this argument will be added to.
        :param arg: Name of the argument
        :param kwargs: These will be passed to the argparse constructor and can be used to configure the argument,
            such as default values, how many values this argument accepts, etc.
        :return:
        """
        if command not in self._cli_commands:
            raise RuntimeError(f"Tried adding an argument to a command that doesn't exist!")
        self._cli_commands[command]["args"].append({"arg": arg, **kwargs})
