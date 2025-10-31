#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Node providing MANUAL_CONTROL messages based on LabJack T4 device
measuring analog voltages. The linear mapping between voltages and axis values
can be configured in runtime with Parameter microservice.

Based on https://github.com/labjack/labjack-ljm-python/blob/master/Examples/More/Stream/stream_basic.py
Adapted for RPC platform by Andrea Zanoni
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


PARAM_FILENAME = "labjack.param"
LABJACK_SCAN_RATE = 128.0


def main():
    # fmt: off
    parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
    parser.add_argument('-m', '--manager',
                        help='MARSH Manager IP addr', default='127.0.0.1')
    parser.add_argument('--extra-channel',
                        help='labjack channel to measure for MOTION_CUE_EXTRA acceleration Z', default=None)
    parser.add_argument('--send-voltage', action='store_true',
                        help='send voltage measured for collective in NAMED_VALUE_FLOAT')
    parser.add_argument("--cyclic-port", type=int,
                        help="override x and y fields with MANUAL_CONTROL received on this port")
    parser.add_argument("--thumbstick", action="store_true",
                        help="use pedals channel to add thumbstick to collective") # TODO: Make this configurable after channel mapping refactor
    # fmt: on
    args = parser.parse_args()
    # assign to typed variables for convenience
    args_manager: str = args.manager
    args_extra_channel: str | None = args.extra_channel
    args_send_voltage: bool = args.send_voltage
    args_cyclic_port: int | None = args.cyclic_port
    args_thumbstick: bool = args.thumbstick

    queue = Queue()
    extra_queue: Queue | None = None
    if args_extra_channel is not None:
        extra_queue = Queue()

    ljmr = LJMReader(
        queue,
        1 / LABJACK_SCAN_RATE,
        (extra_queue, args_extra_channel) if args_extra_channel is not None else None,
        aScanListNames=["AIN0", "AIN4", "AIN5", "AIN6"],
    )
    node = ControlsNode(
        queue, args_manager, args_send_voltage, args_thumbstick, args_cyclic_port
    )

    extra_node: ExtraNode | None = None
    if args_extra_channel is not None:
        extra_node = ExtraNode(extra_queue, args_manager)

    print("Starting threads")
    ljmr.start()
    node.start()
    if extra_node is not None:
        extra_node.start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping all threads")
            ljmr.should_stop.set()
            node.should_stop.set()
            break

    print("Waiting on threads to finish")
    ljmr.join()
    node.join()


class LJMReader(threading.Thread):
    def __init__(
        self,
        queue: Queue,
        period: float,
        extra_signal: tuple[Queue, str] | None = None,
        **kwargs,
    ):
        super().__init__()

        self.queue = queue
        self.extra_queue = None
        if extra_signal is not None:
            self.extra_queue = extra_signal[0]

        # initialize the config with default values
        config = {
            "LJType": "T4",
            "LJConnection": "ETHERNET",
            "aScanListNames": ["AIN0", "AIN1", "AIN2", "AIN3"],
            "ScanRate": LABJACK_SCAN_RATE,
        }
        config.update(kwargs)  # override with any kwargs passed

        if extra_signal is not None:
            config["aScanListNames"].append(extra_signal[1])

        self.LJType = config["LJType"]
        """LabJack Type, defaults to T4"""
        if self.LJType not in ("T4", "T7", "T8"):
            print("LJMReader: unrecognised LabJack type. Aborting...", file=sys.stderr)
            sys.exit(100)

        self.LJConnection = config["LJConnection"]
        """LabJack Connection, defaults to ETHERNET"""
        if self.LJConnection not in ("USB", "ETHERNET", "ANY"):
            print(
                "LJMReader: unrecognised LabJack connection type. Aborting...",
                file=sys.stderr,
            )
            sys.exit(101)

        self.aScanListNames = config["aScanListNames"]
        """LabJack signals to be output. For now, no check on channel name
        is perfomed, so it is up to the user to get it right"""

        self.ScanRate = config["ScanRate"]

        self.should_stop = threading.Event()

        self.LJhandle = ljm.openS(self.LJType, self.LJConnection, "ANY")
        if self.LJhandle:
            info = ljm.getHandleInfo(self.LJhandle)
            print(
                "LJMReader: Found LabJack with Device type: {}, Connection type: {},".format(
                    info[0], info[1]
                )
            )
            print(
                "LJMReader: Serial number: {}, IP address: {}, Port: {}, Max bytes per MB: {}".format(
                    info[2], ljm.numberToIP(info[3]), info[4], info[5]
                )
            )
        else:
            print(
                "LJMReader: Could not find any LabJack device. Aborting...",
                file=sys.stderr,
            )
            sys.exit(1)

        try:
            aNames = ["STREAM_SETTLING_US", "STREAM_RESOLUTION_INDEX"]
            aValues = [0, 0]

            s_out_bufsize = struct.calcsize("d" * len(self.aScanListNames))
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            ljm.eWriteNames(self.LJhandle, len(aNames), aNames, aValues)

            aScanList = ljm.namesToAddresses(
                len(self.aScanListNames), self.aScanListNames
            )[0]

            # Configure and start the LabJackstream, with scansPerRead = 1
            scanRate = ljm.eStreamStart(
                self.LJhandle,  # scansPerRead = 1
                1,
                len(self.aScanListNames),
                aScanList,
                self.ScanRate,
            )
            if scanRate == self.ScanRate:
                print(
                    "LJMReader: Stream started with a scan rate of {} Hz.".format(
                        scanRate
                    )
                )
            else:
                print(
                    "LJMReader: WARNING stream started with a scan rate of {} Hz.".format(
                        scanRate
                    ),
                    file=sys.stderr,
                )

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
        delta = self.t0 + self.period * self.i - time.time()
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
            print(
                "LJMReader: WARNING {} skipped samples at read #{}".format(
                    curSkip, self.i
                ),
                file=sys.stderr,
            )

        axes = tuple([(v if v != -9999.9 else None) for v in aData[:4]])
        self.queue.put(axes)
        if self.extra_queue is not None:
            value = aData[4]
            if value != -9999.9:
                self.extra_queue.put(value)


class ControlsNode(threading.Thread):
    def __init__(
        self,
        queue: Queue,
        manager_addr: str,
        send_voltage: bool,
        collective_thumbstick: bool,
        cyclic_port: int | None = None,
        **kwargs,
    ):
        super().__init__()

        self.queue = queue
        self.manager_addr = manager_addr
        self.send_voltage = send_voltage
        self.collective_thumbstick = collective_thumbstick
        self.cyclic_port = cyclic_port
        self.override_x: int | None = None
        self.override_y: int | None = None
        self.should_stop = threading.Event()

    def run(self):
        # create MAVLink connection
        connection_string = f"udpout:{self.manager_addr}:24400"
        mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
        mav.srcSystem = 1  # default system
        mav.srcComponent = mavlink.MAV_COMP_ID_USER1 + (
            mavlink.MARSH_TYPE_CONTROLS - mavlink.MARSH_TYPE_MANAGER
        )
        print(f"Sending to {connection_string}")

        cyclic_mav: mavlink.MAVLink | None = None
        if self.cyclic_port is not None:
            connection_string = f"udpin:0.0.0.0:{self.cyclic_port}"
            cyclic_mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
            print(f"Listening to MANUAL_CONTROL on {connection_string}")

        # create parameters database, all parameters are float to simplify code
        # default values for inceptors on RPC platform
        params = ParamDict()
        params["PTCH_V_MIN"] = 0.9500
        params["PTCH_V_MAX"] = 0.4502
        params["ROLL_V_MIN"] = 1.3800
        params["ROLL_V_MAX"] = 0.9000
        # Temporarily swap voltages for measuring control loading lever with external sensor
        params["THR_V_MIN"] = 3.500
        params["THR_V_MAX"] = 0.800
        params["YAW_V_MIN"] = 0.896
        params["YAW_V_MAX"] = 1.941
        params["THUMB_V_MIN"] = 0.615
        params["THUMB_V_MAX"] = 2.505
        params["THUMB_GAIN"] = 0.05

        try:
            with open(PARAM_FILENAME, "r") as param_file:
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
                    mavlink.MAV_STATE_ACTIVE,
                )
                heartbeat_next = time.time() + heartbeat_interval

            if time.time() >= control_next:
                voltages: Optional[Tuple[float, float, float, float]] = None
                try:
                    while True:  # get everything from queue
                        voltages = self.queue.get(block=False)
                except Empty:
                    pass

                if voltages is not None:
                    axes = [0x7FFF] * 4  # set each axis to invalid (INT16_MAX)
                    v0, v1, v2, v3 = voltages

                    # assign axis values based on voltages scaled with parameters
                    for i, (prefix, voltage) in enumerate(
                        zip(
                            [
                                "PTCH",
                                "ROLL",
                                "THR",
                                "YAW" if not self.collective_thumbstick else "THUMB",
                            ],
                            [v2, v1, v0, v3],
                        )
                    ):
                        if voltage is None:
                            continue

                        v_min = params[f"{prefix}_V_MIN"]
                        v_max = params[f"{prefix}_V_MAX"]
                        value = (voltage - v_min) / (v_max - v_min)  # 0 to 1
                        value = (value - 0.5) * 2.0  # -1 to 1

                        # scale to the range expected in message
                        axes[i] = round(1000 * max(-1, min(1, value)))

                        if prefix == "THR":
                            # send collective between 0 and 1000
                            axes[i] = round((axes[i] + 1000) / 2)

                    # we use thumbstick for collective on pedals channel
                    if self.collective_thumbstick:
                        axes[2] = axes[2] + round(params["THUMB_GAIN"] * axes[3])
                        axes[3] = 0x7FFF

                    # no buttons are used
                    buttons = 0

                    mav.manual_control_send(
                        mav.srcSystem,
                        self.override_x if self.override_x is not None else axes[0],
                        self.override_y if self.override_y is not None else axes[1],
                        axes[2],
                        axes[3],
                        buttons,
                    )

                    if self.send_voltage:
                        mav.named_value_float_send(
                            round((time.time() - start_time) * 1000),
                            "coll_v\0\0\0\0".encode(),
                            v0,  # cf. order in zip() above
                        )

                control_next = time.time() + control_interval

            # handle incoming messages
            try:
                while (message := mav.file.recv_msg()) is not None:
                    message: mavlink.MAVLink_message
                    if message.get_type() == "HEARTBEAT":
                        heartbeat: mavlink.MAVLink_heartbeat_message = message
                        if heartbeat.type == mavlink.MARSH_TYPE_MANAGER:
                            if not manager_connected:
                                print("Connected to simulation manager")
                            manager_connected = True
                            manager_timeout = time.time() + timeout_interval
                    elif params.should_handle_message(message):
                        params.handle_message(mav, message)
            except ConnectionResetError:
                # thrown on Windows when there is no peer listening
                pass

            if cyclic_mav is not None:
                # handle incoming messages
                try:
                    while (message := cyclic_mav.file.recv_msg()) is not None:
                        message: mavlink.MAVLink_message
                        if message.get_type() == "MANUAL_CONTROL":
                            control: mavlink.MAVLink_manual_control_message = message
                            self.override_x = control.x
                            self.override_y = control.y
                except ConnectionResetError:
                    # thrown on Windows when there is no peer listening
                    pass

            if manager_connected and time.time() > manager_timeout:
                manager_connected = False
                print("Lost connection to simulation manager")


class ExtraNode(threading.Thread):
    def __init__(self, queue: Queue, manager_addr: str, **kwargs):
        super().__init__()

        self.queue = queue
        self.manager_addr = manager_addr
        self.should_stop = threading.Event()

    def run(self):
        # create MAVLink connection
        connection_string = f"udpout:{self.manager_addr}:24400"
        mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
        mav.srcSystem = 1  # default system
        mav.srcComponent = mavlink.MARSH_COMP_ID_VIBRATION_SOURCE
        print(f"Sending to {connection_string}")

        params = ParamDict()
        params["V_MIN"] = -1.0
        params["V_MAX"] = 1.0
        params["ACC_Z_MIN"] = -1.0
        params["ACC_Z_MAX"] = 1.0

        start_time = time.time()

        # controlling when messages should be sent
        heartbeat_next = 0.0
        heartbeat_interval = 1.0

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
                    mavlink.MAV_STATE_ACTIVE,
                )
                heartbeat_next = time.time() + heartbeat_interval

            voltage: Optional[float] = None
            try:
                voltage = self.queue.get(block=False)
            except Empty:
                pass

            if voltage is not None:
                v_min = params["V_MIN"]
                v_max = params["V_MAX"]
                acc_z_min = params["ACC_Z_MIN"]
                acc_z_max = params["ACC_Z_MAX"]
                if v_min != v_max:
                    value = acc_z_min + (acc_z_max - acc_z_min) * (voltage - v_min) / (
                        v_max - v_min
                    )

                    mav.motion_cue_extra_send(
                        round((time.time() - start_time) * 1000), 0, 0, 0, 0, 0, value
                    )

            # handle incoming messages
            try:
                while (message := mav.file.recv_msg()) is not None:
                    message: mavlink.MAVLink_message
                    if message.get_type() == "HEARTBEAT":
                        if message.get_srcComponent() == mavlink.MARSH_COMP_ID_MANAGER:
                            if not manager_connected:
                                print("Connected to simulation manager")
                            manager_connected = True
                            manager_timeout = time.time() + timeout_interval
                    elif params.should_handle_message(message):
                        changes = params.handle_message(mav, message)
                        if changes is not None:
                            with open(PARAM_FILENAME, "w") as param_file:
                                params.dump(param_file)
            except ConnectionResetError:
                # thrown on Windows when there is no peer listening
                pass

            if manager_connected and time.time() > manager_timeout:
                manager_connected = False
                print("Lost connection to simulation manager")


if __name__ == "__main__":
    main()
