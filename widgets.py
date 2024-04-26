import asyncio
import math

from textual.widgets import Input, Log, Static
from mavsdk.telemetry import FlightMode
from textual.binding import Binding
from rich.text import Text

import logging


class InputWithHistory(Input):

    BINDINGS = Input.BINDINGS.copy()
    BINDINGS.append(Binding("up", "history_prev", "Previous item from history", show=False))
    BINDINGS.append(Binding("down", "history_rec", "Next item in history", show=False))

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.history = []
        self.history_cursor = 0     # Shows where in the history we are
        self.rolling_zero = 0       # Current "0" index. If we had more entries, these get overwritten.
        self.history_max_length = 50

    # Behaviour: Depends on if we are already using history.
    # A new submission is added to the history
    # Up then goes back into the history
    # Down does nothing
    # But: if we submit a history entry (i.e. go back and do not change)
    # Up displays the same entry (from the history)
    # Down shows the next entry in the history.
    # i.e. submit "a", up -> "a", down -> nothing
    # submit "a", submit "b", submit "c", up -> "c", up ->"b", submit "b", [up -> "b", down ->"c"]

    @property
    def _current_history_cursor(self):
        return (self.rolling_zero - self.history_cursor) % min(len(self.history), self.history_max_length)

    def _increase_history_cursor(self):
        if self.history_cursor < self.history_max_length and self.history_cursor < len(self.history):
            self.history_cursor += 1

    def _decrease_history_cursor(self):
        if self.history_cursor > 1:
            self.history_cursor -= 1

    def _increase_history_rolling_pos(self):
        self.rolling_zero = (self.rolling_zero + 1) % self.history_max_length

    def action_history_prev(self) -> None:
        if self.history:
            self._increase_history_cursor()
            self.value = self.history[self._current_history_cursor]

    def action_history_rec(self) -> None:
        if self.history:
            self._decrease_history_cursor()
            self.value = self.history[self._current_history_cursor]
        if self.history_cursor == 1:
            self.value = ""

    def add_to_history(self, item) -> None:
        # If we add extra entries, overwrite old ones
        if len(self.history) == self.history_max_length:
            # Entry to be overridden is at rolling_zero
            self.history[self.rolling_zero] = item
            self._increase_history_rolling_pos()
        else:
            self. history.append(item)

    async def action_submit(self) -> None:
        self.add_to_history(self.value)
        # TODO: Fancy history maintaining, i.e. do not reset cursor if we enter a historic prompt without changing it.
        self.history_cursor = 0
        await super().action_submit()


class DroneOverview(Static):

    COLUMN_NAMES = ["Name", "Status", "FlightMode", "Local", "Vel", "Yaw/Bat"]
    COLUMN_WIDTHS = [10, 11, 11, 9, 9, 8]
    COLUMN_ALIGN = ["<", ">", ">", ">", ">", ">"]
    COLUMN_SPACING = 3

    def __init__(self, drone, update_frequency, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.drone = drone
        self.update_frequency = update_frequency
        self.logger = self.app.logger
        self.format_string = (" "*self.COLUMN_SPACING).join([f"{{:{self.COLUMN_ALIGN[i]}{self.COLUMN_WIDTHS[i]}}}"
                                                             for i
                                                             in range(len(self.COLUMN_NAMES))])

    @classmethod
    def header_string(cls):
        return (" "*cls.COLUMN_SPACING).join([f"{cls.COLUMN_NAMES[i]:{cls.COLUMN_ALIGN[i]}{cls.COLUMN_WIDTHS[i]}}"
                                              for i
                                              in range(len(cls.COLUMN_NAMES))])

    @classmethod
    def gadget_width(cls):
        return (len(cls.COLUMN_NAMES)-1)*cls.COLUMN_SPACING + sum(cls.COLUMN_WIDTHS)

    def on_mount(self) -> None:
        asyncio.create_task(self.update_display())

    async def update_display(self):
        while True:
            try:
                if self.drone.flightmode == FlightMode.OFFBOARD:
                    color = "green"
                else:
                    color = "red"
                battery_remaining = math.nan
                battery_voltage = math.nan
                try:
                    battery_remaining = self.drone.batteries[0].remaining
                    battery_voltage = self.drone.batteries[0].voltage
                except KeyError:
                    pass
                output = ""
                output += self.format_string.format("",
                                                    f"Conn: {str(self.drone.is_connected):>{self.COLUMN_WIDTHS[1]-6}}",
                                                    "",
                                                    f"F: {self.drone.position_ned[0]:{self.COLUMN_WIDTHS[3]-3}.3f}",
                                                    f"F: {self.drone.velocity[0]:{self.COLUMN_WIDTHS[4]-3}.3f}",
                                                    f"Y: {self.drone.attitude[2]:{self.COLUMN_WIDTHS[5]-3}.1f}") + "\n"
                output += self.format_string.format(self.drone.name,
                                                    f"Arm: {str(self.drone.is_armed):>{self.COLUMN_WIDTHS[1]-5}}",
                                                    str(self.drone.flightmode),
                                                    f"R: {self.drone.position_ned[1]:{self.COLUMN_WIDTHS[3]-3}.3f}",
                                                    f"R: {self.drone.velocity[1]:{self.COLUMN_WIDTHS[4]-3}.3f}",
                                                    f"{battery_remaining:{self.COLUMN_WIDTHS[5]-1}.0f}%") + "\n"
                output += self.format_string.format("",
                                                    f"Air: {str(self.drone.in_air):>{self.COLUMN_WIDTHS[1]-5}}",
                                                    "",
                                                    f"D: {self.drone.position_ned[2]:{self.COLUMN_WIDTHS[3]-3}.3f}",
                                                    f"D: {self.drone.velocity[2]:{self.COLUMN_WIDTHS[4]-3}.3f}",
                                                    f"{battery_voltage:{self.COLUMN_WIDTHS[5]-1}.2f}V") + "\n"
                self.update(Text(output, style=f"bold {color}"))
            except Exception as e:
                self.logger.debug(f"Exception updating status pane for drone {self.drone.name}: {repr(e)}",
                                  exc_info=True)
            await asyncio.sleep(1 / self.update_frequency)


class TextualLogHandler(logging.Handler):
    def __init__(self, log_textual, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.log_textual: Log = log_textual

    def emit(self, record):
        self.log_textual.write_line(self.format(record))
