#!/usr/bin/env python3

"""
Node for controlling a simple G-Seat connected on UART.
Developed for the https://gitlab.polimi.it/DAER/frame-sim/haptic-seat device

The control is sent as a decimal number from 0.0000 to 1.0000, followed by
a newline (LF) character. The value is based on vertical acceleration
in SIM_STATE message. The linear mapping between acceleration and control
value can be configured in runtime with Parameter microservice.
"""

from argparse import ArgumentParser
from collections import OrderedDict
from pymavlink import mavutil
import serial
from time import time

import mavlink_all as mavlink
from utils import NodeFormatter
from utils.param_dict import ParamDict
from utils.model_utils import STD_G

parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
parser.add_argument('device',
                    help='serial device to open (e.g. COM5 on Windows)')
parser.add_argument('-m', '--manager',
                    help='MARSH Manager IP addr', default='127.0.0.1')
parser.add_argument('--interval', type=float,
                    help='time between messages, in seconds', default=0.05)
args = parser.parse_args()
# assign to typed variables for convenience
args_device: str = args.device
args_manager: str = args.manager
args_interval: float = args.interval


# create MAVLink connection
connection_string = f'udpout:{args_manager}:24400'
mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
mav.srcSystem = 1  # default system
mav.srcComponent = mavlink.MAV_COMP_ID_USER1 + (mavlink.MARSH_TYPE_GSEAT - mavlink.MARSH_TYPE_MANAGER)
print(f'Sending to {connection_string}')

# create parameters database, all parameters are float to simplify code
params = ParamDict()
params['ACCEL_Z_MIN'] = 0.8 * STD_G
params['ACCEL_Z_MAX'] = 1.2 * STD_G
params['OUT_MIN'] = 0.4
params['OUT_MAX'] = 0.9


def vertical_output(zacc: float) -> float:
    zmin = params['ACCEL_Z_MIN']
    zmax = params['ACCEL_Z_MAX']
    out_min = params['OUT_MIN']
    out_max = params['OUT_MAX']
    if zmin == zmax:
        print('ERROR: Parameters ACCEL_Z_MIN and ACCEL_Z_MAX have same value')
        return 0.5
    if out_min == out_max:
        print('ERROR: Parameters OUT_MIN and OUT_MAX have same value')
        return 0.5

    # zacc is negative for stationary vehicle, flip to do all math on positive values
    raw_value = max(0, min(1, (-zacc - zmin)/(zmax - zmin)))
    return out_min + raw_value * (out_max - out_min)


last_state = mavlink.MAVLink_sim_state_message(
    1, 0, 0, 0, 0, 0, 0, 0, 0, -STD_G, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

# controlling when messages should be sent
heartbeat_next = 0.0
heartbeat_interval = 1.0
control_next = 0.0
control_interval = args_interval

# monitoring connection to manager with heartbeat
timeout_interval = 5.0
manager_timeout = 0.0
manager_connected = False

with serial.Serial(args_device, 57600, timeout=0.01) as ser:
    print('Opened {} with baud {}'.format(ser.name, ser.baudrate))

    # the loop goes as fast as it can, relying on the variables above for timing
    while True:
        line = ser.readline().decode('ascii')
        if len(line) > 0:
            print('RX:\t', end='')
            print(line, end='')

        if time() >= heartbeat_next:
            mav.heartbeat_send(
                mavlink.MARSH_TYPE_GSEAT,
                mavlink.MAV_AUTOPILOT_INVALID,
                mavlink.MAV_MODE_FLAG_TEST_ENABLED,
                0,
                mavlink.MAV_STATE_ACTIVE
            )
            heartbeat_next = time() + heartbeat_interval

        if time() >= control_next:
            text = '{:.4f}'.format(vertical_output(last_state.zacc))
            ser.write((text + '\n').encode('ascii'))
            control_next = time() + control_interval

        # handle incoming messages
        try:
            while (message := mav.file.recv_msg()) is not None:
                # pyright couldn't handle this annotation without quoting
                message: 'mavlink.MAVLink_message'
                if message.get_type() == 'HEARTBEAT':
                    heartbeat: mavlink.MAVLink_heartbeat_message = message
                    if heartbeat.type == mavlink.MARSH_TYPE_MANAGER:
                        if not manager_connected:
                            print('Connected to simulation manager')
                        manager_connected = True
                        manager_timeout = time() + timeout_interval
                elif message.get_type() == 'SIM_STATE':
                    last_state = message
                elif params.should_handle_message(message):
                    params.handle_message(mav, message)
        except ConnectionResetError:
            # thrown on Windows when there is no peer listening
            pass

        if manager_connected and time() > manager_timeout:
            manager_connected = False
            print('Lost connection to simulation manager')
