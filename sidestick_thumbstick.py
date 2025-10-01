#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Node providing MANUAL_CONTROL messages based on a LabJack T4 device,
measuring analog voltages. The linear mapping between voltages and axis values
can be configured at runtime via the Parameter microservice.

Based on:
https://github.com/labjack/labjack-ljm-python/blob/master/Examples/More/Stream/stream_basic.py

Adapted for the RPC simulator platform by Andrea Zanoni.

Update 01/10/2025:
- Reads AIN0 (collective), AIN1 (thumbstick), AIN2 (pedals).
- Pitch and Roll from Brunner sidestick 
- Yaw now mapped to pedals.
- Thumbstick scaled with THUMB_GAIN and added to collective input.

Adapted and extended by Paolo De Franceschi.
"""

from labjack import ljm
from pymavlink import mavutil
import socket
import struct
import sys
import threading
import time
from queue import Empty, Queue

import mavlink_all as mavlink
from utils.param_dict import ParamDict

PARAM_FILENAME = 'labjack.param'
LABJACK_SCAN_RATE = 128.0

# Updated by Brunner (pitch/roll values)
brunner_axes = {"PTCH": 0.0, "ROLL": 0.0}


def main():
    queue = Queue()
    # Read three channels: AIN0 (collective), AIN1 (thumbstick), AIN2 (pedals)
    ljmr = LJMReader(queue, 1 / LABJACK_SCAN_RATE,
                     aScanListNames=['AIN0', 'AIN1', 'AIN2'])
    node = ControlsNode(queue, "127.0.0.1", send_voltage=False)

    print("Starting threads...")
    ljmr.start()
    node.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping threads")
        ljmr.should_stop.set()
        node.should_stop.set()

    ljmr.join()
    node.join()


class LJMReader(threading.Thread):
    """
    Thread that reads analog signals from LabJack using stream mode
    and pushes them into a shared queue.
    """
    def __init__(self, queue: Queue, period: float, **kwargs):
        super().__init__()
        self.queue = queue

        # Default configuration for LabJack device
        config = {
            'LJType': 'T4',
            'LJConnection': 'ETHERNET',
            'aScanListNames': ['AIN0', 'AIN1', 'AIN2'],  # collective, thumbstick, pedals
            'ScanRate': LABJACK_SCAN_RATE
        }
        config.update(kwargs)

        self.aScanListNames = config['aScanListNames']
        self.ScanRate = config['ScanRate']
        self.should_stop = threading.Event()

        # Open connection to LabJack
        self.LJhandle = ljm.openS(config['LJType'], config['LJConnection'], 'ANY')
        if self.LJhandle:
            info = ljm.getHandleInfo(self.LJhandle)
            print(f'LJMReader: Found LabJack with Device type: {info[0]}, Connection type: {info[1]}')
            print(f'LJMReader: Serial: {info[2]}, IP: {ljm.numberToIP(info[3])}, Port: {info[4]}, MaxBytesPerMB: {info[5]}')
        else:
            print('LJMReader: Could not find any LabJack device. Aborting...', file=sys.stderr)
            sys.exit(1)

        try:
            # Configure stream parameters
            aNames = ['STREAM_SETTLING_US', 'STREAM_RESOLUTION_INDEX']
            aValues = [0, 0]

            ljm.eWriteNames(self.LJhandle, len(aNames), aNames, aValues)
            aScanList = ljm.namesToAddresses(len(self.aScanListNames), self.aScanListNames)[0]

            # Start streaming from LabJack
            scanRate = ljm.eStreamStart(self.LJhandle, 1, len(self.aScanListNames), aScanList, self.ScanRate)
            if scanRate == self.ScanRate:
                print(f'LJMReader: Stream started with {scanRate} Hz')
            else:
                print(f'LJMReader: WARNING stream started with {scanRate} Hz', file=sys.stderr)

            self.period = period
            self.i = 0
            self.t0 = time.time()

        except Exception as e:
            print(f"LJMReader: ERROR {e}", file=sys.stderr)
            ljm.eStreamStop(self.LJhandle)
            ljm.close(self.LJhandle)

    def sleep(self):
        """Synchronize the loop to achieve the configured period."""
        self.i += 1
        delta = self.t0 + self.period * self.i - time.time()
        if delta > 0:
            time.sleep(delta)

    def run(self):
        """Main loop: read from LabJack and push data into the queue."""
        while not self.should_stop.is_set():
            self.read_and_send()
            self.sleep()

        # Cleanup on exit
        ljm.eStreamStop(self.LJhandle)
        print("LJMReader: closing the handle")
        ljm.close(self.LJhandle)

    def read_and_send(self):
        """Read a sample from LabJack and push collective, thumbstick, and pedals to queue."""
        aData, _, _ = ljm.eStreamRead(self.LJhandle)

        curSkip = aData.count(-9999.0)
        if curSkip:
            print(f"LJMReader: WARNING {curSkip} skipped samples at read #{self.i}", file=sys.stderr)

        # Collective (AIN0), Thumbstick (AIN1), Pedals (AIN2)
        thr = aData[0] if aData[0] != -9999.9 else None
        thumb = aData[1] if aData[1] != -9999.9 else None
        yaw = aData[2] if aData[2] != -9999.9 else None

        self.queue.put((thr, thumb, yaw))


class ControlsNode(threading.Thread):
    """
    Thread that processes input values, scales them,
    and sends MANUAL_CONTROL messages to MARSH manager via MAVLink.
    """
    def __init__(self, queue: Queue, manager_addr: str, send_voltage: bool, **kwargs):
        super().__init__()
        self.queue = queue
        self.manager_addr = manager_addr
        self.send_voltage = send_voltage
        self.should_stop = threading.Event()

    def run(self):
        # Create MAVLink connection
        connection_string = f'udpout:{self.manager_addr}:24400'
        mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
        mav.srcSystem = 1
        mav.srcComponent = mavlink.MAV_COMP_ID_USER1 + (mavlink.MARSH_TYPE_CONTROLS - mavlink.MARSH_TYPE_MANAGER)
        print(f'Sending to {connection_string}')

        # Parameter dictionary (all stored as float)
        params = ParamDict()
        params['THR_V_MIN'] = 0.0
        params['THR_V_MAX'] = 5.0
        params['THUMB_V_MIN'] = 0.0
        params['THUMB_V_MAX'] = 5.0
        params['THUMB_GAIN'] = 0.05
        params['YAW_V_MIN'] = 0.0
        params['YAW_V_MAX'] = 5.0

        # Load custom parameters if available
        try:
            with open(PARAM_FILENAME, 'r') as param_file:
                params.load(param_file)
        except FileNotFoundError:
            pass

        # Timing
        heartbeat_interval = 1.0
        control_interval = 0.02
        timeout_interval = 5.0

        heartbeat_next = 0.0
        control_next = 0.0
        manager_timeout = 0.0
        manager_connected = False

        while not self.should_stop.is_set():
            now = time.time()

            # Send heartbeat
            if now >= heartbeat_next:
                mav.heartbeat_send(
                    mavlink.MARSH_TYPE_CONTROLS,
                    mavlink.MAV_AUTOPILOT_INVALID,
                    mavlink.MAV_MODE_FLAG_TEST_ENABLED,
                    0,
                    mavlink.MAV_STATE_ACTIVE
                )
                heartbeat_next = now + heartbeat_interval

            # Send control input
            if now >= control_next:
                thr, thumb, yaw = 0, 0, 0
                try:
                    while True:
                        thr, thumb, yaw = self.queue.get(block=False)
                except Empty:
                    pass

                ptch = brunner_axes["PTCH"]
                roll = brunner_axes["ROLL"]

                axes = [0x7FFF] * 4  # default invalid values

                def scale(v, vmin, vmax):
                    if vmax == vmin or v is None:
                        return 0
                    return round(1000 * max(-1, min(1, (v - vmin) / (vmax - vmin) * 2 - 1)))

                # Pitch and Roll from Brunner
                axes[0] = ptch
                axes[1] = roll

                # Collective + thumbstick scaled and combined
                thr_scaled = scale(thr, params['THR_V_MIN'], params['THR_V_MAX'])
                thumb_scaled = scale(thumb, params['THUMB_V_MIN'], params['THUMB_V_MAX'])
                axes[2] = thr_scaled + int(thumb_scaled * params['THUMB_GAIN'])

                # Pedals (Yaw)
                yaw_scaled = scale(yaw, params['YAW_V_MIN'], params['YAW_V_MAX'])
                axes[3] = yaw_scaled

                # No buttons used
                buttons = 0

                mav.manual_control_send(
                    mav.srcSystem,
                    axes[0], axes[1], axes[2], axes[3],
                    buttons,
                )

                control_next = now + control_interval

            # Handle incoming messages
            try:
                while (message := mav.file.recv_msg()) is not None:
                    if message.get_type() == 'HEARTBEAT':
                        if message.type == mavlink.MARSH_TYPE_MANAGER:
                            if not manager_connected:
                                print('Connected to simulation manager')
                            manager_connected = True
                            manager_timeout = time.time() + timeout_interval
                    elif params.should_handle_message(message):
                        params.handle_message(mav, message)
            except ConnectionResetError:
                pass

            if manager_connected and now > manager_timeout:
                manager_connected = False
                print('Lost connection to simulation manager')


if __name__ == '__main__':
    main()
