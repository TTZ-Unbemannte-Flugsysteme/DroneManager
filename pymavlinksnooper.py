""" CURRENTLY DOES NOT WORK.

Look at mavlink_map in the dialect files (seem to keep a list of classes for each message type)
Might be able to cast classes using "get_msgId" and mavlink_map, but check that fields are set appropriately
"""
import logging
import datetime
import time
import os
import asyncio

from pymavlink import mavutil
from pymavlink.dialects.v10 import common


formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")


# PyMAVLink has an issue that received messages which contain strings
# cannot be resent, because they become Python strings (not bytestrings)
# This converts those messages so your code doesn't crash when
# you try to send the message again.
def _strencode_mav(msg):
    msg_type = msg.get_type()
    if msg_type in ('PARAM_VALUE', 'PARAM_REQUEST_READ', 'PARAM_SET'):
        if type(msg.param_id) == str:
            msg.param_id = msg.param_id.encode()
    elif msg_type == 'STATUSTEXT':
        if type(msg.text) == str:
            msg.text = msg.text.encode()
    return msg


class Snooper:
    def __init__(self):
        self.source_system = 245
        self.source_component = 201
        self.con_drone: mavutil.mavudp | None = None
        self.con_gcs: mavutil.mavudp | None = None

        self.logger = logging.getLogger("manager")
        self.logger.setLevel(logging.DEBUG)

        filename = f"snooper_{datetime.datetime.now()}"
        filename = filename.replace(":", "_").replace(".", "_") + ".log"
        logdir = os.path.abspath("./logs")
        os.makedirs(logdir, exist_ok=True)
        self.file_handler = logging.FileHandler(os.path.join(logdir, filename))
        self.file_handler.setLevel(logging.DEBUG)
        self.file_handler.setFormatter(formatter)
        self.logger.addHandler(self.file_handler)
        self.running_tasks = set()
        self._schedule_background_tasks()

    def _schedule_background_tasks(self):
        self.running_tasks.add(asyncio.create_task(self._listen_gcs()))
        self.running_tasks.add(asyncio.create_task(self._listen_drone()))

    def connect_gcs(self, address):
        self.con_gcs = mavutil.mavlink_connection("udpout:" + address,
                                                  source_system=self.source_system,
                                                  source_component=self.source_component, dialect="common")
        self.running_tasks.add(asyncio.create_task(self._send_heartbeats_gcs()))

    def connect_drone(self, address):
        self.con_drone = mavutil.mavlink_connection("udpin:" + address,
                                                    source_system=self.source_system,
                                                    source_component=self.source_component, dialect="common")
        self.running_tasks.add(asyncio.create_task(self._send_heartbeats_drone()))

    async def _listen_gcs(self):
        while True:
            if self.con_gcs is not None:
                while True:
                    # Receive and log all messages from the GCS
                    msg = self.con_gcs.recv_match(blocking=False)
                    if msg is None:
                        await asyncio.sleep(0.0001)
                    else:
                        self.logger.info(f"Message from GCS, {msg.to_dict()}")
                        if self.con_drone is not None and msg.get_type != "BAD_DATA":  # Send onward to the drone
                            self.con_drone.mav.srcSystem = msg.get_srcSystem()
                            self.con_drone.mav.srcComponent = msg.get_srcComponent()
                            self.con_drone.mav.send(msg)
            else:
                await asyncio.sleep(1)

    async def _listen_drone(self):
        while True:
            if self.con_drone is not None:
                while True:
                    # Receive and log all messages from the GCS
                    msg = self.con_drone.recv_match(blocking=False)
                    if msg is None:
                        await asyncio.sleep(0.0001)
                    else:
                        self.logger.info(f"Message from Drone, {msg.to_dict()}")
                        if self.con_gcs is not None and msg.get_type != "BAD_DATA":    # Send onward to GCS
                            self.con_gcs.mav.srcSystem = msg.get_srcSystem()
                            self.con_gcs.mav.srcComponent = msg.get_srcComponent()
                            self.con_gcs.mav.send(msg)
            else:
                await asyncio.sleep(1)

    async def _send_heartbeats_gcs(self):
        while True:
            self.con_gcs.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
            await asyncio.sleep(1)

    async def _send_heartbeats_drone(self):
        while True:
            self.con_drone.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
            await asyncio.sleep(1)


async def main():
    snoop = Snooper()
    snoop.connect_drone("127.0.0.1:14540")
    snoop.connect_gcs("127.0.0.1:15540")
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
