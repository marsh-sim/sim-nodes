#!/usr/bin/env python3

"""
Node providing MANUAL_CONTROL messages based on USB HID Joystick device.

Axis assignment and reversal can be configured in runtime with parameters.

Default parameters are for Thrustmaster T-Flight HOTAS X with yaw on throttle.
(see https://www.thrustmaster.com/products/t-flight-hotas-x/)
"""

from argparse import ArgumentParser
from collections import OrderedDict
from pymavlink import mavutil
from time import time

import mavlink_all as mavlink
from utils import NodeFormatter
from utils.param_dict import ParamDict

parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
parser.add_argument('-i', '--input-index', type=int,
                    help='index of input device to use', default=0)
parser.add_argument('-m', '--manager',
                    help='MARSH Manager IP addr', default='127.0.0.1')
parser.add_argument('-t', '--time-interval', type=float,
                    help='interval between sent messages, in seconds', default=0.02)
args = parser.parse_args()
# assign to typed variables for convenience
args_manager: str = args.manager
args_input_index: int = args.input_index
args_time_interval: float = args.time_interval

import pygame  # noqa - prints on import
from pygame import joystick  # noqa - prints on import
pygame.init()
joystick.init()

if joystick.get_count() == 0:
    print('No joystick connected')
    exit(1)


# validate arguments
if args_input_index < 0:
    parser.error('Input index must be non-negative')

if args_input_index >= joystick.get_count():
    parser.error(
        f'Input index {args_input_index} too large, only {joystick.get_count()} joysticks connected')

# create joystick device
device = joystick.Joystick(args_input_index)
device.init()

# create MAVLink connection
connection_string = f'udpout:{args_manager}:24400'
mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
mav.srcSystem = 1  # default system
mav.srcComponent = mavlink.MARSH_COMP_ID_CONTROLS
print(f'Sending to {connection_string}')

params = ParamDict()
params['PTCH_AXIS'] = 1.0
params['PTCH_REVERSED'] = 0.0
params['ROLL_AXIS'] = 0.0
params['ROLL_REVERSED'] = 0.0
params['THR_AXIS'] = 2.0
params['THR_REVERSED'] = 0.0
params['YAW_AXIS'] = 3.0
params['YAW_REVERSED'] = 0.0


if 'hotas x' in device.get_name().lower():
    print('Loading parameters for Thrustmaster T-Flight HOTAS X with yaw on throttle')
    params['PTCH_AXIS'] = 1.0
    params['PTCH_REVERSED'] = 1.0
    params['ROLL_AXIS'] = 0.0
    params['ROLL_REVERSED'] = 0.0
    params['THR_AXIS'] = 2.0
    params['THR_REVERSED'] = 1.0
    params['YAW_AXIS'] = 4.0
    params['YAW_REVERSED'] = 0.0
elif device.get_name() == 'Arduino Leonardo':
    print('Loading parameters for Arduino Leonardo in ATTILA simulator')
    params['PTCH_AXIS'] = 2.0
    params['PTCH_REVERSED'] = 1.0
    params['ROLL_AXIS'] = 0.0
    params['ROLL_REVERSED'] = 0.0
    params['THR_AXIS'] = 5.0
    params['THR_REVERSED'] = 0.0
    params['YAW_AXIS'] = 4.0
    params['YAW_REVERSED'] = 0.0


# controlling when messages should be sent
heartbeat_next = 0.0
heartbeat_interval = 1.0
control_next = 0.0
control_interval = args_time_interval

# monitoring connection to manager with heartbeat
timeout_interval = 5.0
manager_timeout = 0.0
manager_connected = False

# the loop goes as fast as it can, relying on the variables above for timing
while True:
    # process pygame loop
    for event in pygame.event.get():
        pass

    if time() >= heartbeat_next:
        mav.heartbeat_send(
            mavlink.MAV_TYPE_GENERIC,
            mavlink.MAV_AUTOPILOT_INVALID,
            mavlink.MAV_MODE_FLAG_TEST_ENABLED,
            0,
            mavlink.MAV_STATE_ACTIVE
        )
        heartbeat_next = time() + heartbeat_interval

    if time() >= control_next:
        axes = [0x7FFF] * 4  # set each axis to invalid (INT16_MAX)

        # get joystick values based on parameters
        for i, prefix in enumerate(['PTCH', 'ROLL', 'THR', 'YAW']):
            axis_number = int(params[f'{prefix}_AXIS'])
            if axis_number >= 0 and axis_number < device.get_numaxes():
                value = device.get_axis(axis_number)
                if params[f'{prefix}_REVERSED'] != 0.0:
                    value *= -1.0
                # scale to the normalized range
                axes[i] = round(1000 * max(-1, min(1, value)))

                if prefix == 'THR':
                    # send collective between 0 and 1000
                    axes[i] = round((axes[i] + 1000) / 2)

        # set each bit in buttons corresponding to pressed state
        buttons = 0
        for button in range(min(16, device.get_numbuttons())):
            if device.get_button(button):
                buttons += 1 << button

        mav.manual_control_send(
            mav.srcSystem,
            axes[0], axes[1], axes[2], axes[3],
            buttons,
        )
        control_next = time() + control_interval

    # handle incoming messages
    try:
        while (message := mav.file.recv_msg()) is not None:
            message: mavlink.MAVLink_message
            if message.get_type() == 'HEARTBEAT':
                if message.get_srcComponent() == mavlink.MARSH_COMP_ID_MANAGER:
                    if not manager_connected:
                        print('Connected to simulation manager')
                    manager_connected = True
                    manager_timeout = time() + timeout_interval
            elif params.should_handle_message(message):
                params.handle_message(mav, message)
    except ConnectionResetError:
        # thrown on Windows when there is no peer listening
        pass

    if manager_connected and time() > manager_timeout:
        manager_connected = False
        print('Lost connection to simulation manager')
