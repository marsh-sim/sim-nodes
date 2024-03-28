#!/usr/bin/env python3

"""
Node for controlling a simple G-Seat connected on UART.
Developed for the https://gitlab.polimi.it/DAER/frame-sim/haptic-seat device

The control is sent as a decimal number from 0.0000 to 1.0000, followed by a newline (LF) character.
The value is based on vertical acceleration in SIM_STATE message.
The linear mapping between acceleration and control value can be configured in runtime with Parameter microservice.
"""

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from collections import OrderedDict
from pymavlink import mavutil
import serial
from time import time

import mavlink_all as mavlink
from utils.model_utils import STD_G

parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
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
mav.srcComponent = mavlink.MARSH_COMP_ID_GSEAT
print(f'Sending to {connection_string}')

# create parameters database, all parameters are float to simplify code
params: OrderedDict[str, float] = OrderedDict()
params['ACCEL_Z_MIN'] = 0.8 * STD_G
params['ACCEL_Z_MAX'] = 1.2 * STD_G

for k in params.keys():
    assert len(k) <= 16, 'parameter names must fit into param_id field'


def send_param(index: int, name=''):
    """
    convenience function to send PARAM_VALUE
    pass index -1 to use name instead

    silently returns on invalid index or name
    """
    param_id = bytearray(16)

    if index >= 0:
        if index >= len(params):
            return

        # HACK: is there a nicer way to get items from OrderedDict by order?
        name = list(params.keys())[index]
    else:
        if name not in params:
            return

        index = list(params.keys()).index(name)
    name_bytes = name.encode('utf8')
    param_id[:len(name_bytes)] = name_bytes

    mav.param_value_send(param_id, params[name], mavlink.MAV_PARAM_TYPE_REAL32,
                         len(params), index)


def vertical_output(zacc: float) -> float:
    zmin = params['ACCEL_Z_MIN']
    zmax = params['ACCEL_Z_MAX']
    if zmin == zmax:
        print('ERROR: Parameters ACCEL_Z_MIN and ACCEL_Z_MAX have same value')
        return 0.5
    return max(0, min(1, (zacc - zmin)/(zmax - zmin)))


last_state = mavlink.MAVLink_sim_state_message(
    1, 0, 0, 0, 0, 0, 0, 0, 0, STD_G, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

# controlling when messages should be sent
heartbeat_next = 0.0
heartbeat_interval = 1.0
control_next = 0.0
control_interval = args_interval

# monitoring connection to manager with heartbeat
timeout_interval = 5.0
manager_timeout = 0.0
manager_connected = False

with serial.Serial(args_device, 230400, timeout=0.01) as ser:
    print('Opened {} with baud {}'.format(ser.name, ser.baudrate))

    # the loop goes as fast as it can, relying on the variables above for timing
    while True:
        line = ser.readline().decode('ascii')
        if len(line) > 0:
            print('RX:\t', end='')
            print(line, end='')

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
            text = '{:.4f}'.format(vertical_output(last_state.zacc))
            ser.write((text + '\n').encode('ascii'))
            control_next = time() + control_interval

        # handle incoming messages
        while (message := mav.file.recv_msg()) is not None:
            message: mavlink.MAVLink_message
            if message.get_type() == 'HEARTBEAT':
                if message.get_srcComponent() == mavlink.MARSH_COMP_ID_MANAGER:
                    if not manager_connected:
                        print('Connected to simulation manager')
                    manager_connected = True
                    manager_timeout = time() + timeout_interval
            elif message.get_type() == 'SIM_STATE':
                last_state = message
            elif message.get_type() in ['PARAM_REQUEST_READ', 'PARAM_REQUEST_LIST', 'PARAM_SET']:
                # check that this is relevant to us
                if message.target_system == mav.srcSystem and message.target_component == mav.srcComponent:
                    if message.get_type() == 'PARAM_REQUEST_READ':
                        m: mavlink.MAVLink_param_request_read_message = message
                        send_param(m.param_index, m.param_id)
                    elif message.get_type() == 'PARAM_REQUEST_LIST':
                        for i in range(len(params)):
                            send_param(i)
                    elif message.get_type() == 'PARAM_SET':
                        m: mavlink.MAVLink_param_set_message = message
                        # check that parameter is defined and sent as float
                        if m.param_id in params and m.param_type == mavlink.MAV_PARAM_TYPE_REAL32:
                            params[m.param_id] = m.param_value
                        send_param(-1, m.param_id)

        if manager_connected and time() > manager_timeout:
            manager_connected = False
            print('Lost connection to simulation manager')
