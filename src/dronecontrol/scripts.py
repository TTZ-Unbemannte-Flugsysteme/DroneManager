import asyncio
import math
from typing import Dict
import os
import subprocess

from dronecontrol.plugins import Plugin



class ScriptsPlugin(Plugin):
    
    PREFIX = "scpt"

    def __init__(self, dm, logger):
        super().__init__(dm, logger)
        self.cli_commands = {
            "execute": self.execute_script
        }

    async def start(self):
        self.logger.debug("Starting Script plugin...")

    async def execute_script(self, script_name: str):
        """ Run Script in ./Scripts with given Name"""
        self.logger.info(f"Executing Script {script_name}")
        script_path = os.path.join("Scripts", script_name)
        # Ensure script exists
        if not os.path.isfile(script_path):
            self.logger.warning(f"Script {script_name} not found in ./Scripts")
            return
        try:
            # Execute the script
            result = subprocess.run(
                ["python3", script_path], capture_output=True, text=True, check=True
            )
            self.logger.info(f"Script Output:\n{result.stdout}")
        except subprocess.CalledProcessError as e:
            self.logger.warning(f"Script execution failed: {e.stderr}")
        except Exception as e:
            self.logger.warning(f"Unexpected error: {repr(e)}")



