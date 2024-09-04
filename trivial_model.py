#!/usr/bin/env python3

"""
Trivial flight model:
- aircraft not moving, at 0 position
- roll and pitch in a limited range, directly following controls
- acceleration pointing down (considering roll and pitch),
  with magnitude dependent on throttle input
"""

from argparse import ArgumentParser
from math import cos, radians, sin
from pymavlink import mavutil
from time import time

import mavlink_all as mavlink
from utils import NodeFormatter
from utils.model_utils import *

MAX_ANGLE = radians(5.0)
"""Maximum tilt from horizontal corresponding to controls deflection, in radians"""
MAX_ACCEL = 0.2 * STD_G
"""Maximum acceleration change from thrust control, in meters per second squared"""

def main():
    parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
    parser.add_argument('-m', '--manager',
                        help='MARSH Manager IP addr', default='127.0.0.1')
    parser.add_argument('--no-heartbeat', action='store_false', dest='heartbeat',
                        help='toggle sending HEARTBEAT messages for testing Manager with unregistered clients')
    args = parser.parse_args()
    # assign to typed variables for convenience
    args_manager: str = args.manager
    args_heartbeat: bool = args.heartbeat

    connection_string = f'udpout:{args_manager}:24400'
    mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
    mav.srcSystem = 1  # default system
    mav.srcComponent = mavlink.MARSH_COMP_ID_FLIGHT_MODEL
    print(f'Sending to {connection_string}')
    loop(mav, args_heartbeat)


def simulate(_previous_state: mavlink.MAVLink_sim_state_message, controls: Controls, _delta_time: float) -> mavlink.MAVLink_sim_state_message:
    """
    Trivial example, rotate the attitude with directly with joystick
    """

    pitch = MAX_ANGLE * -controls.pitch
    roll = MAX_ANGLE * controls.roll
    yaw = 0.0
    q1, q2, q3, q4 = euler_to_quaternion(roll, pitch, yaw)

    # for acceleration start with gravity vector and rotate by attitude
    # gravity causes the same effect as accelerating upward, so it's negative Z in NED system
    acc = [0.0, 0.0, -(MAX_ACCEL * controls.thrust + STD_G)]
    acc[1] = acc[1] * cos(roll) + acc[2] * sin(roll)
    acc[2] = acc[1] * -sin(roll) + acc[2] * cos(roll)
    acc[0] = acc[0] * cos(pitch) + acc[2] * -sin(pitch)
    acc[2] = acc[0] * sin(pitch) + acc[2] * cos(pitch)

    state = mavlink.MAVLink_sim_state_message(
        q1, q2, q3, q4,  # attitude quaternion
        roll, pitch, yaw,  # attitude Euler angles, roll, pitch, yaw
        acc[0], acc[1], acc[2],  # local acceleration X, Y, Z
        0, 0, 0,  # local angular speed X, Y, Z
        0, 0, 0,  # latitude, longitude, altitude
        0, 0,  # position standard deviation horizontal, vertical
        0, 0, 0,  # velocity N, E, D
        0, 0,  # lat, lon as degrees * 10^7
    )
    return state


def loop(mav: mavlink.MAVLink, args_heartbeat: bool):
    last_controls = Controls()
    last_controls_time = time()
    last_state = mavlink.MAVLink_sim_state_message(
        1, 0, 0, 0, 0, 0, 0, 0, 0, -STD_G, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    last_state_time = time()

    # controlling when messages should be sent
    heartbeat_next = 0.0
    heartbeat_interval = 1.0
    state_next = 0.0
    state_interval = 0.25

    # monitoring connection to manager with heartbeat
    timeout_interval = 5.0
    manager_timeout = 0.0
    manager_connected = False

    # the loop goes as fast as it can, relying on the variables above for timing
    while True:
        if args_heartbeat and time() >= heartbeat_next:
            mav.heartbeat_send(
                mavlink.MAV_TYPE_GENERIC,
                mavlink.MAV_AUTOPILOT_INVALID,
                mavlink.MAV_MODE_FLAG_TEST_ENABLED,
                0,
                mavlink.MAV_STATE_ACTIVE
            )
            heartbeat_next = time() + heartbeat_interval

        if time() >= state_next or last_controls_time > last_state_time:
            # simulate on interval or new controls
            state = simulate(last_state, last_controls, time() - last_state_time)
            mav.send(state)

            last_state = state
            last_state_time = time()
            state_next = time() + state_interval

        # handle incoming messages
        try:
            while (message := mav.file.recv_msg()) is not None:
                message: mavlink.MAVLink_message
                if message.get_type() == 'HEARTBEAT':
                    # the following line only helps with type hints
                    heartbeat: mavlink.MAVLink_heartbeat_message = message

                    if heartbeat.get_srcComponent() == mavlink.MARSH_COMP_ID_MANAGER:
                        if not manager_connected:
                            # example of showing text for enum
                            state = mavlink.enums['MAV_STATE'][heartbeat.system_status]
                            print('Connected to simulation manager in state', state.name)
                        manager_connected = True
                        manager_timeout = time() + timeout_interval
                elif message.get_type() == 'MANUAL_CONTROL':
                    last_controls = Controls.from_message(message)
                    last_controls_time = time()
        except ConnectionResetError:
            # thrown on Windows when there is no peer listening
            pass

        if manager_connected and time() > manager_timeout:
            manager_connected = False
            print('Lost connection to simulation manager')

if __name__ == '__main__':
    main()
