import asyncio

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
            # Entry to be overriden is at rolling_zero
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

    def __init__(self, drone, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.drone = drone
        self.logger = self.app.logger

    def on_mount(self) -> None:
        asyncio.create_task(self.update_display())

    async def update_display(self):
        while True:
            if self.drone.flightmode == FlightMode.OFFBOARD:
                color = "green"
            else:
                color = "red"
            format_string = "{:<10}   {:>9}   {:>5}   {:>6}   {:>11}   {:>10.7f}   {:>6.3f}   {:>6.3f}   {:>8.3f}"
            output = ""
            output += format_string.format("", "", "", "", "", self.drone.position_global[0],
                                           self.drone.position_ned[0], self.drone.velocity[0], self.drone.attitude[2]) + "\n"
            output += format_string.format(self.drone.name, str(self.drone.is_connected), str(self.drone.is_armed),
                                           str(self.drone.in_air), str(self.drone.flightmode),
                                           self.drone.position_global[1], self.drone.position_ned[1],
                                           self.drone.velocity[1], self.drone.position_global[3]) + "\n"
            output += format_string.format("", "", "", "", "", self.drone.position_global[2],
                                           self.drone.position_ned[2], self.drone.velocity[2], self.drone._max_position_discontinuity) + "\n"
            self.update(Text(output, style=f"bold {color}"))
            await asyncio.sleep(1/20)


class TextualLogHandler(logging.Handler):
    def __init__(self, log_textual, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.log_textual: Log = log_textual

    def emit(self, record):
        self.log_textual.write_line(self.format(record))
