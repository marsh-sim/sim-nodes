#!/usr/bin/env python3

"""
Node setting target values for lidia instrument display
(https://pypi.org/project/lidia/)

The data corresponds to the AircraftState class which is defined here:
https://gitlab.com/Maarrk/lidia/-/blob/main/src/lidia/aircraft.py

The message V2_EXTENSION used here can be used for tunneling other binary data
through MARSH. The receiving node must subscribe to this message id
and recognize the payload based on the message_type field, which is a uint16
identifier.
"""

from argparse import ArgumentParser
from math import nan, radians
from msgpack import packb
from pymavlink import mavutil
from typing import Optional

import mavlink_all as mavlink
from utils import NodeFormatter

parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
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
mav.srcComponent = mavlink.MARSH_COMP_ID_PILOT_TARGET
print(f'Sending to {connection_string}')

# register in manager to be able to affect other nodes
mav.heartbeat_send(
    mavlink.MAV_TYPE_GENERIC,
    mavlink.MAV_AUTOPILOT_INVALID,
    mavlink.MAV_MODE_FLAG_TEST_ENABLED,
    0,
    mavlink.MAV_STATE_ACTIVE
)

type_mask = 0b0000_1101_1111_1111  # ignore all, don't use force instead of acceleration

yaw = nan
if args_yaw is not None:
    yaw = radians(args_yaw)
    type_mask ^= mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE  # flip the bit to not ignored
z = nan
if args_alt is not None:
    z = -args_alt * 0.3048  # convert from feet to meters
    type_mask ^= mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE  # flip the bit to not ignored
vx = nan
if args_ias is not None:
    vx = args_ias * 1852.0 / 3600.0  # convert from knots to meters per second
    type_mask ^= mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE  # flip the bit to not ignored

mav.set_position_target_local_ned_send(
    1, 1, mavlink.MARSH_COMP_ID_INSTRUMENTS, mavlink.MAV_FRAME_LOCAL_NED, type_mask,
    nan, nan, z, vx, nan, nan, nan, nan, nan, yaw, nan,
)
print(f'Sent target yaw: {yaw} rad, z: {z} m, vx: {vx} m/s')