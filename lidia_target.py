#!/usr/bin/env python3

"""
Node setting target values for lidia instrument display (https://pypi.org/project/lidia/)

The data corresponds to the AircraftState class which is defined here:
https://gitlab.com/Maarrk/lidia/-/blob/main/src/lidia/aircraft.py

The message V2_EXTENSION used here can be used for tunneling other binary data through MARSH.
The receiving node must subscribe to this message id and recognize the payload
based on the message_type field, which is a uint16 identifier.
"""

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from math import radians
from msgpack import packb
from pymavlink import mavutil
from typing import Optional

import mavlink_all as mavlink

parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument('-m', '--manager',
                    help='MARSH Manager IP addr', default='127.0.0.1')
parser.add_argument('-a', '--alt', type=float,
                    help='target altitude, in feet')
parser.add_argument('-i', '--ias', type=float,
                    help='target indicated airspeed (IAS), in knots')
parser.add_argument('-y', '--yaw', type=float,
                    help='target heading, in degrees')
args = parser.parse_args()
# assign to typed variables for convenience
args_manager: str = args.manager
args_alt: Optional[float] = args.alt
args_ias: Optional[float] = args.ias
args_yaw: Optional[float] = args.yaw

connection_string = f'udpout:{args_manager}:24400'
mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
mav.srcSystem = 1  # default system
mav.srcComponent = mavlink.MAV_COMP_ID_USER50  # some component unused by MARSH
print(f'Sending to {connection_string}')

# register in manager to be able to affect other nodes
mav.heartbeat_send(
    mavlink.MAV_TYPE_GENERIC,
    mavlink.MAV_AUTOPILOT_INVALID,
    mavlink.MAV_MODE_FLAG_TEST_ENABLED,
    0,
    mavlink.MAV_STATE_ACTIVE
)

state = {'trgt': {'instr': {}, }}
if args_yaw is not None:
    state['trgt']['att'] = [0, 0, radians(args_yaw)]
if args_alt is not None:
    state['trgt']['instr']['alt'] = args_alt
if args_ias is not None:
    state['trgt']['instr']['ias'] = args_ias

# pack the data with msgpack as expected by lidia
payload = bytearray(packb(state))
# pad payload with zeros to maximum length
payload.extend(bytearray(249 - len(payload)))

# send the payload
mav.v2_extension_send(0, 0, 0, 44400, payload)
print('Sent state:', state)
