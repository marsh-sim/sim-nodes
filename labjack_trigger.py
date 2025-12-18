#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Node for controlling a LabJack T7 DAC output based on MAVLink COMMAND_LONG messages.
Listens for MAV_CMD_DO_EXPERIMENT_CONTROL commands and sets a DAC voltage accordingly.
When param1=1, sets voltage to the acquisition start level; when param1=0, sets to the other level.

Adapted for MARSH platform by Andrea Zanoni
"""

from argparse import ArgumentParser
from labjack import ljm
from pymavlink import mavutil
import sys
import threading
import time

import mavlink_all as mavlink
from utils import NodeFormatter


# MAVLink command for experiment control trigger
MAV_CMD_DO_EXPERIMENT_CONTROL = 52500


def main():
    epilog = """
examples:
  # Acquisition starts at HIGH voltage (5.0V), stops at LOW (0.0V)
  %(prog)s --trigger DAC0 0.0 5.0 HIGH

  # Acquisition starts at LOW voltage (0.0V), stops at HIGH (3.3V)
  %(prog)s --trigger DAC0 0.0 3.3 LOW

  # Both levels non-zero, start at lower voltage
  %(prog)s --trigger DAC1 1.5 4.5 LOW

  # With custom manager address
  %(prog)s --manager 192.168.1.100 --trigger DAC0 0.0 5.0 HIGH
"""
    parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__, epilog=epilog)
    parser.add_argument('-m', '--manager',
                        help='MARSH Manager IP addr', default='127.0.0.1')
    parser.add_argument("--trigger", nargs=4, required=True,
                        metavar=("<DAC#>", "<LOW_VOLTAGE>", "<HIGH_VOLTAGE>", "<START_LEVEL>"),
                        help="DAC channel, low voltage, high voltage, and start level (HIGH or LOW)")
    args = parser.parse_args()

    # assign to typed variables for convenience
    args_manager: str = args.manager
    dac_channel: str = args.trigger[0]
    low_voltage: float = float(args.trigger[1])
    high_voltage: float = float(args.trigger[2])
    start_level: str = args.trigger[3].upper()

    # Validate start_level
    if start_level not in ('HIGH', 'LOW'):
        parser.error("START_LEVEL must be either 'HIGH' or 'LOW'")
        sys.exit(1)

    trigger_node = TriggerNode(args_manager, dac_channel, low_voltage, high_voltage, start_level)

    print('Starting trigger node')
    trigger_node.start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print('Stopping trigger node')
            trigger_node.should_stop.set()
            break

    print('Waiting on trigger node to finish')
    trigger_node.join()


class TriggerNode(threading.Thread):
    def __init__(self, manager_addr: str, dac_channel: str, low_voltage: float, high_voltage: float, start_level: str, **kwargs):
        super().__init__()

        self.manager_addr = manager_addr
        self.dac_channel = dac_channel
        self.low_voltage = low_voltage
        self.high_voltage = high_voltage
        self.start_level = start_level
        self.should_stop = threading.Event()

        # Determine which voltage corresponds to acquisition start and stop
        if start_level == 'HIGH':
            self.acquisition_start_voltage = high_voltage
            self.acquisition_stop_voltage = low_voltage
        else:  # start_level == 'LOW'
            self.acquisition_start_voltage = low_voltage
            self.acquisition_stop_voltage = high_voltage

        self.target_voltage = self.acquisition_stop_voltage  # Initially at stop level

        # Open LabJack T7 connection
        self.LJhandle = ljm.openS('T7', 'ETHERNET', 'ANY')
        if self.LJhandle:
            info = ljm.getHandleInfo(self.LJhandle)
            print('TriggerNode: Found LabJack with Device type: {}, Connection type: {},'.format(
                info[0], info[1]))
            print('TriggerNode: Serial number: {}, IP address: {}, Port: {}, Max bytes per MB: {}'.format(
                info[2], ljm.numberToIP(info[3]), info[4], info[5]))
        else:
            print('TriggerNode: Could not find any LabJack device. Aborting...', file=sys.stderr)
            sys.exit(1)

        # Set initial DAC voltage to acquisition stop level
        try:
            ljm.eWriteName(self.LJhandle, self.dac_channel, self.target_voltage)
            print(f'TriggerNode: Initialized {self.dac_channel} to {self.target_voltage}V (acquisition stop level)')
        except ljm.LJMError as e:
            print(f"TriggerNode: LJM ERROR initializing {self.dac_channel}: {e}", file=sys.stderr)
            sys.exit(2)

    def run(self):
        # create MAVLink connection to receive commands
        connection_string = f'udpin:0.0.0.0:24400'
        mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
        mav.srcSystem = 1  # default system
        mav.srcComponent = mavlink.MAV_COMP_ID_USER1 + (mavlink.MARSH_TYPE_CONTROLS - mavlink.MARSH_TYPE_MANAGER)
        print(f'TriggerNode: Listening for COMMAND_LONG on {connection_string}')
        print(f'TriggerNode: Will control {self.dac_channel}:')
        print(f'  Acquisition START (param1=1): {self.acquisition_start_voltage}V ({self.start_level})')
        print(f'  Acquisition STOP  (param1=0): {self.acquisition_stop_voltage}V')

        # controlling when messages should be sent
        heartbeat_next = 0.0
        heartbeat_interval = 1.0
        dac_update_next = 0.0
        dac_update_interval = 0.1  # Update DAC at 10Hz

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

            # Periodically update DAC voltage to ensure it stays at target
            if time.time() >= dac_update_next:
                try:
                    ljm.eWriteName(self.LJhandle, self.dac_channel, self.target_voltage)
                except ljm.LJMError as e:
                    print(f"TriggerNode: LJM ERROR writing to {self.dac_channel}: {e}", file=sys.stderr)
                dac_update_next = time.time() + dac_update_interval

            # handle incoming messages
            try:
                while (message := mav.file.recv_msg()) is not None:
                    message: mavlink.MAVLink_message
                    if message.get_type() == 'HEARTBEAT':
                        heartbeat: mavlink.MAVLink_heartbeat_message = message
                        if heartbeat.type == mavlink.MARSH_TYPE_MANAGER:
                            if not manager_connected:
                                print('TriggerNode: Connected to simulation manager')
                            manager_connected = True
                            manager_timeout = time.time() + timeout_interval
                    elif message.get_type() == 'COMMAND_LONG':
                        cmd: mavlink.MAVLink_command_long_message = message
                        if cmd.command == MAV_CMD_DO_EXPERIMENT_CONTROL:
                            if cmd.param1 == 1.0:
                                self.target_voltage = self.acquisition_start_voltage
                                print(f'TriggerNode: Acquisition START - setting {self.dac_channel} to {self.acquisition_start_voltage}V')
                                # Immediately set the voltage
                                try:
                                    ljm.eWriteName(self.LJhandle, self.dac_channel, self.target_voltage)
                                except ljm.LJMError as e:
                                    print(f"TriggerNode: LJM ERROR writing to {self.dac_channel}: {e}", file=sys.stderr)
                            else:
                                self.target_voltage = self.acquisition_stop_voltage
                                print(f'TriggerNode: Acquisition STOP - setting {self.dac_channel} to {self.acquisition_stop_voltage}V')
                                # Immediately set the voltage
                                try:
                                    ljm.eWriteName(self.LJhandle, self.dac_channel, self.target_voltage)
                                except ljm.LJMError as e:
                                    print(f"TriggerNode: LJM ERROR writing to {self.dac_channel}: {e}", file=sys.stderr)
            except ConnectionResetError:
                # thrown on Windows when there is no peer listening
                pass

            if manager_connected and time.time() > manager_timeout:
                manager_connected = False
                print('TriggerNode: Lost connection to simulation manager')

            time.sleep(0.01)  # Small sleep to prevent busy-waiting

        # Clean up - set DAC to acquisition stop level before closing
        try:
            ljm.eWriteName(self.LJhandle, self.dac_channel, self.acquisition_stop_voltage)
            print(f"TriggerNode: Reset {self.dac_channel} to {self.acquisition_stop_voltage}V (acquisition stop level)")
        except ljm.LJMError as e:
            print(f"TriggerNode: LJM ERROR during cleanup: {e}", file=sys.stderr)

        print("TriggerNode: closing the handle")
        ljm.close(self.LJhandle)


if __name__ == '__main__':
    main()
