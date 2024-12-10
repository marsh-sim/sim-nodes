#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Node providing MANUAL_CONTROL messages based on USB HID LabJack T4 device
measuring analog voltages. The linear mapping between voltages and axis values
can be configured in runtime with Parameter microservice.

Based on https://github.com/labjack/labjack-ljm-python/blob/master/Examples/More/Stream/stream_basic.py
Adapted for RPC platform by Andrea Zanoni
"""

from argparse import ArgumentParser
from collections import OrderedDict
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


def main():
    parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
    parser.add_argument('-m', '--manager',
                        help='MARSH Manager IP addr', default='127.0.0.1')
    args = parser.parse_args()
    # assign to typed variables for convenience
    args_manager: str = args.manager

    queue = Queue()

    ljmr = LJMReader(
        queue, 1/256., aScanListNames=['AIN0', 'AIN4', 'AIN5', 'AIN6'])
    node = ControlsNode(queue, args_manager)

    print('Starting threads')
    ljmr.start()
    node.start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print('Stopping all threads')
            ljmr.should_stop.set()
            node.should_stop.set()
            break

    print('Waiting on threads to finish')
    ljmr.join()
    node.join()


class LJMReader(threading.Thread):
    def __init__(self, queue: Queue, period: float, **kwargs):
        super().__init__()

        self.queue = queue

        # initialize the config with default values
        config = {
            'LJType': 'T4',
            'LJConnection': 'ETHERNET',
            'aScanListNames': ['AIN0', 'AIN1', 'AIN2', 'AIN3'],
            'ScanRate': 256  # corresponding to 1000 Hz
        }
        config.update(kwargs)  # override with any kwargs passed

        self.LJType = config['LJType']
        """LabJack Type, defaults to T4"""
        if self.LJType not in ('T4', 'T7', 'T8'):
            print('LJMReader: unrecognised LabJack type. Aborting...', file=sys.stderr)
            sys.exit(100)

        self.LJConnection = config['LJConnection']
        """LabJack Connection, defaults to ETHERNET"""
        if self.LJConnection not in ('USB', 'ETHERNET', 'ANY'):
            print(
                'LJMReader: unrecognised LabJack connection type. Aborting...', file=sys.stderr)
            sys.exit(101)

        self.aScanListNames = config['aScanListNames']
        """LabJack signals to be output. For now, no check on channel name
        is perfomed, so it is up to the user to get it right"""

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

            # Configure and start the LabJackstream, with scansPerRead = 1
            scanRate = ljm.eStreamStart(
                self.LJhandle, \
                # scansPerRead = 1
                1, \
                len(self.aScanListNames), \
                aScanList, \
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

        except ljm.LJMError:
            ljme = sys.exc_info()[1]
            print("LJMReader: LJM ERROR " + ljme, file=sys.stderr)
            # Close LabJack handle
            ljm.close(self.LJhandle)
        except Exception:
            e = sys.exc_info()[1]
            print("labjack_stream.py: ERROR {}".format(e))
            # Close LabJack handle
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

        print("LJMReader: closing the handle")
        ljm.close(self.LJhandle)

    def read_and_send(self):
        ret = ljm.eStreamRead(self.LJhandle)
        aData = ret[0]

        # Skipped samples are indicated with -9999.0 values
        curSkip = aData.count(-9999.0)
        if curSkip:
            print("LJMReader: WARNING {} skipped samples at read #{}".format(
                curSkip, self.i), file=sys.stderr)

        axes = tuple([(v if v != -9999.9 else None) for v in aData[:4]])
        self.queue.put(axes)


class ControlsNode(threading.Thread):
    def __init__(self, queue: Queue, manager_addr: str, **kwargs):
        super().__init__()

        self.queue = queue
        self.manager_addr = manager_addr
        self.should_stop = threading.Event()

    def run(self):
        # create MAVLink connection
        connection_string = f'udpout:{self.manager_addr}:24400'
        mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
        mav.srcSystem = 1  # default system
        mav.srcComponent = mavlink.MARSH_COMP_ID_CONTROLS
        print(f'Sending to {connection_string}')

        # create parameters database, all parameters are float to simplify code
        # default values for inceptors on RPC platform
        params = ParamDict()
        params['PTCH_V_MIN'] = 1.4164
        params['PTCH_V_MAX'] = 0.7820
        params['ROLL_V_MIN'] = 1.4920
        params['ROLL_V_MAX'] = 0.7820
        params['THR_V_MIN'] = 0.0100
        params['THR_V_MAX'] = 2.6650
        params['YAW_V_MIN'] = 0.896
        params['YAW_V_MAX'] = 1.941

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
                    mavlink.MAV_TYPE_GENERIC,
                    mavlink.MAV_AUTOPILOT_INVALID,
                    mavlink.MAV_MODE_FLAG_TEST_ENABLED,
                    0,
                    mavlink.MAV_STATE_ACTIVE
                )
                heartbeat_next = time.time() + heartbeat_interval

            voltages: Optional[Tuple[float, float, float, float]] = None
            try:
                voltages = self.queue.get(block=False)
            except Empty:
                pass

            if voltages is not None:
                axes = [0x7FFF] * 4  # set each axis to invalid (INT16_MAX)
                v0, v1, v2, v3 = voltages

                # assign axis values based on voltages scaled with parameters
                for i, (prefix, voltage) in enumerate(zip(['PTCH', 'ROLL', 'THR', 'YAW'], [v2, v1, v0, v3])):
                    if voltage is None:
                        continue

                    v_min = params[f'{prefix}_V_MIN']
                    v_max = params[f'{prefix}_V_MAX']
                    value = (voltage - v_min) / (v_max - v_min)  # 0 to 1
                    value = (value - 0.5) * 2.0  # -1 to 1

                    # scale to the range expected in message
                    axes[i] = round(1000 * max(-1, min(1, value)))

                # no buttons are used
                buttons = 0

                mav.manual_control_send(
                    mav.srcSystem,
                    axes[0], axes[1], axes[2], axes[3],
                    buttons,
                )
                control_next = time.time() + control_interval

            # handle incoming messages
            try:
                while (message := mav.file.recv_msg()) is not None:
                    message: mavlink.MAVLink_message
                    if message.get_type() == 'HEARTBEAT':
                        if message.get_srcComponent() == mavlink.MARSH_COMP_ID_MANAGER:
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
