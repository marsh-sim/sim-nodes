#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Node providing MANUAL_CONTROL messages based on LabJack T4 device
measuring analog voltages. The linear mapping between voltages and axis values
can be configured in runtime with Parameter microservice.

Based on https://github.com/labjack/labjack-ljm-python/blob/master/Examples/More/Stream/stream_basic.py
Adapted for RPC platform by Andrea Zanoni

Modifica 30/09/25: ora legge solo AIN0 (collettivo). 
Pitch e Roll arrivano dal Brunner. Yaw non è usato.
"""

from argparse import ArgumentParser
from typing import Optional, Tuple
from labjack import ljm
from pymavlink import mavutil
import socket
import struct
import sys
import threading
import time
from queue import Empty, Queue

import mavlink_all as mavlink
from utils import NodeFormatter
from utils.param_dict import ParamDict

PARAM_FILENAME = 'labjack.param'
LABJACK_SCAN_RATE = 128.0

# valori aggiornati dal Brunner
brunner_axes = {"PTCH": 0.0, "ROLL": 0.0}


def main():
    queue = Queue()
    ljmr = LJMReader(queue, 1/LABJACK_SCAN_RATE,
                     aScanListNames=['AIN0'])  # TODO: Choose a channel for thumbstick
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
    def __init__(self, queue: Queue, period: float, extra_signal: tuple[Queue, str] | None = None, **kwargs):
        super().__init__()

        self.queue = queue
        self.extra_queue = None
        if extra_signal is not None:
            self.extra_queue = extra_signal[0]

        # initialize the config with default values
        config = {
            'LJType': 'T4',
            'LJConnection': 'ETHERNET',
            'aScanListNames': ['AIN0'],  # TODO: Choose a channel for thumbstick
            'ScanRate': LABJACK_SCAN_RATE
        }
        config.update(kwargs)  # override with any kwargs passed

        if extra_signal is not None:
            config['aScanListNames'].append(extra_signal[1])

        self.LJType = config['LJType']
        """LabJack Type, defaults to T4"""
        if self.LJType not in ('T4', 'T7', 'T8'):
            print('LJMReader: unrecognised LabJack type. Aborting...', file=sys.stderr)
            sys.exit(100)

        self.LJConnection = config['LJConnection']
        """LabJack Connection, defaults to ETHERNET"""
        if self.LJConnection not in ('USB', 'ETHERNET', 'ANY'):
            print('LJMReader: unrecognised LabJack connection type. Aborting...', file=sys.stderr)
            sys.exit(101)

        self.aScanListNames = config['aScanListNames']
        """LabJack signals to be output. For now, no check on channel name
        is performed, so it is up to the user to get it right"""

        self.ScanRate = config['ScanRate']

        self.should_stop = threading.Event()

        self.LJhandle = ljm.openS(self.LJType, self.LJConnection, 'ANY')
        if self.LJhandle:
            info = ljm.getHandleInfo(self.LJhandle)
            print('LJMReader: Found LabJack with Device type: {}, Connection type: {},'.format(
                info[0], info[1]))
            print('LJMReader: Serial number: {}, IP address: {}, Port: {}, Max bytes per MB: {}'.format(
                info[2], ljm.numberToIP(info[3]), info[4], info[5]))
        else:
            print(
                'LJMReader: Could not find any LabJack device. Aborting...', file=sys.stderr)
            sys.exit(1)

        try:
            aNames = ['STREAM_SETTLING_US', 'STREAM_RESOLUTION_INDEX']
            aValues = [0, 0]

            s_out_bufsize = struct.calcsize('d'*len(self.aScanListNames))
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            ljm.eWriteNames(self.LJhandle, len(aNames), aNames, aValues)

            aScanList = ljm.namesToAddresses(
                len(self.aScanListNames), self.aScanListNames)[0]

            # Configure and start the LabJack stream, with scansPerRead = 1
            scanRate = ljm.eStreamStart(
                self.LJhandle,
                # scansPerRead = 1
                1,
                len(self.aScanListNames),
                aScanList,
                self.ScanRate)
            if scanRate == self.ScanRate:
                print(
                    'LJMReader: Stream started with a scan rate of {} Hz.'.format(scanRate))
            else:
                print('LJMReader: WARNING stream started with a scan rate of {} Hz.'.format(
                    scanRate), file=sys.stderr)

            self.period = period
            self.i = 0
            self.t0 = time.time()

        except Exception as e:
            if isinstance(e, ljm.LJMError):
                print("LJMReader: LJM ERROR", e, file=sys.stderr)
            else:
                print("labjack_stream.py: ERROR {}".format(e))

            ljm.eStreamStop(self.LJhandle)
            ljm.close(self.LJhandle)

    def sleep(self):
        self.i += 1
        delta = self.t0 + self.period*self.i - time.time()
        if delta > 0:
            time.sleep(delta)

    def run(self):
        while not self.should_stop.is_set():
            self.read_and_send()
            self.sleep()

        ljm.eStreamStop(self.LJhandle)
        print("LJMReader: closing the handle")
        ljm.close(self.LJhandle)

    def read_and_send(self):
        aData, _, _ = ljm.eStreamRead(self.LJhandle)

        # Skipped samples are indicated with -9999.0 values
        curSkip = aData.count(-9999.0)
        if curSkip:
            print("LJMReader: WARNING {} skipped samples at read #{}".format(
                curSkip, self.i), file=sys.stderr)

        # TODO: Also measure and queue thumbstick voltage
        thr = aData[0] if aData[0] != -9999.9 else None
        self.queue.put(thr)


class ControlsNode(threading.Thread):
    def __init__(self, queue: Queue, manager_addr: str, send_voltage: bool, **kwargs):
        super().__init__()

        self.queue = queue
        self.manager_addr = manager_addr
        self.send_voltage = send_voltage
        self.should_stop = threading.Event()

    def run(self):
        # create MAVLink connection
        connection_string = f'udpout:{self.manager_addr}:24400'
        mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
        mav.srcSystem = 1  # default system
        mav.srcComponent = mavlink.MAV_COMP_ID_USER1 + (mavlink.MARSH_TYPE_CONTROLS - mavlink.MARSH_TYPE_MANAGER)
        print(f'Sending to {connection_string}')

        # create parameters database, all parameters are float to simplify code
        # default values for inceptors on RPC platform
        params = ParamDict()
        params['THR_V_MIN'] = 0.0
        params['THR_V_MAX'] = 5.0
        params['THUMB_V_MIN'] = 0.0
        params['THUMB_V_MAX'] = 5.0
        params['THUMB_GAIN'] = 0.05

        try:
            with open(PARAM_FILENAME, 'r') as param_file:
                params.load(param_file)
        except FileNotFoundError:
            pass

        start_time = time.time()

        # controlling when messages should be sent
        heartbeat_next = 0.0
        heartbeat_interval = 1.0
        control_next = 0.0
        control_interval = 0.02

        # monitoring connection to manager with heartbeat
        timeout_interval = 5.0
        manager_timeout = 0.0
        manager_connected = False

        # the loop goes as fast as it can, relying on the variables above for timing
        while not self.should_stop.is_set():
            if time.time() >= heartbeat_next:
                mav.heartbeat_send(
                    mavlink.MARSH_TYPE_CONTROLS,
                    mavlink.MAV_AUTOPILOT_INVALID,
                    mavlink.MAV_MODE_FLAG_TEST_ENABLED,
                    0,
                    mavlink.MAV_STATE_ACTIVE
                )
                heartbeat_next = time.time() + heartbeat_interval

            if time.time() >= control_next:
                thr = 0
                try:
                    while True:  # get everything from queue
                        thr = self.queue.get(block=False)
                except Empty:
                    pass

                # pitch/roll dal Brunner
                ptch = brunner_axes["PTCH"]
                roll = brunner_axes["ROLL"]

                # assign axis values based on voltages scaled with parameters
                axes = [0x7FFF] * 4  # set each axis to invalid (INT16_MAX)

                def scale(v, vmin, vmax):
                    if vmax == vmin:
                        return 0
                    return round(1000 * max(-1, min(1, (v - vmin) / (vmax - vmin) * 2 - 1)))

                # TODO: Scale thumb by params, add thumb * THUMB_GAIN to thr
                axes[0] = ptch
                axes[1] = roll
                axes[2] = scale(thr, params['THR_V_MIN'], params['THR_V_MAX'])     # THR
                axes[3] = 0  # TODO: Restore pedal functionality remember to do it

                # no buttons are used
                buttons = 0

                mav.manual_control_send(
                    mav.srcSystem,
                    axes[0], axes[1], axes[2], axes[3],
                    buttons,
                )

                control_next = time.time() + control_interval

            # handle incoming messages (come originale)
            try:
                while (message := mav.file.recv_msg()) is not None:
                    if message.get_type() == 'HEARTBEAT':
                        heartbeat = message
                        if heartbeat.type == mavlink.MARSH_TYPE_MANAGER:
                            if not manager_connected:
                                print('Connected to simulation manager')
                            manager_connected = True
                            manager_timeout = time.time() + timeout_interval
                    elif params.should_handle_message(message):
                        params.handle_message(mav, message)
            except ConnectionResetError:
                # thrown on Windows when there is no peer listening
                pass

            if manager_connected and time.time() > manager_timeout:
                manager_connected = False
                print('Lost connection to simulation manager')

if __name__ == '__main__':
    main()

