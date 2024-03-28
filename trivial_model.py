#!/usr/bin/env python3

from argparse import ArgumentParser
from dataclasses import dataclass
from math import cos, radians, sin
from time import time
from typing import Tuple

from pymavlink import mavutil
import mavlink_all as mavlink

parser = ArgumentParser()
parser.add_argument('-m', '--manager',
                    help='MARSH Manager IP addr', default='127.0.0.1')
args = parser.parse_args()

connection_string = f'udpout:{args.manager}:24400'
mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
mav.srcSystem = 1  # default system
mav.srcComponent = mavlink.MARSH_COMP_ID_FLIGHT_MODEL
print(f'Sending to {connection_string}')

STD_G = 9.80665


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    adapted from mavlink_conversions.h
    """
    cosPhi_2 = cos(roll / 2)
    sinPhi_2 = sin(roll / 2)
    cosTheta_2 = cos(pitch / 2)
    sinTheta_2 = sin(pitch / 2)
    cosPsi_2 = cos(yaw / 2)
    sinPsi_2 = sin(yaw / 2)
    q1 = (cosPhi_2 * cosTheta_2 * cosPsi_2 +
          sinPhi_2 * sinTheta_2 * sinPsi_2)
    q2 = (sinPhi_2 * cosTheta_2 * cosPsi_2 -
          cosPhi_2 * sinTheta_2 * sinPsi_2)
    q3 = (cosPhi_2 * sinTheta_2 * cosPsi_2 +
          sinPhi_2 * cosTheta_2 * sinPsi_2)
    q4 = (cosPhi_2 * cosTheta_2 * sinPsi_2 -
          sinPhi_2 * sinTheta_2 * cosPsi_2)
    return q1, q2, q3, q4


@dataclass
class Controls:
    pitch = 0.0  # positive nose down
    roll = 0.0  # positive roll right
    thrust = 0.0  # positive go up
    yaw = 0.0  # positive turn right


def simulate(_previous_state: mavlink.MAVLink_sim_state_message, controls: Controls, _delta_time: float) -> mavlink.MAVLink_sim_state_message:
    """
    Trivial example, rotate the attitude with directly with joystick
    """
    MAX_ANGLE = radians(5.0)
    MAX_ACCEL = 0.2 * STD_G

    pitch = MAX_ANGLE * -controls.pitch
    roll = MAX_ANGLE * controls.roll
    yaw = 0.0
    q1, q2, q3, q4 = euler_to_quaternion(roll, pitch, yaw)

    # for acceleration start with gravity vector and rotate by attitude
    acc = [0.0, 0.0, MAX_ACCEL * controls.thrust + STD_G]
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


last_controls = Controls()
last_controls_time = time()
last_state = mavlink.MAVLink_sim_state_message(
    1, 0, 0, 0, 0, 0, 0, 0, 0, STD_G, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
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
    if time() >= heartbeat_next:
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
            manual_control: mavlink.MAVLink_manual_control_message = message
            controls = Controls()
            # assign only valid axes
            if -1000 <= manual_control.x <= 1000:
                controls.pitch = manual_control.x / 1000.0
            if -1000 <= manual_control.y <= 1000:
                controls.roll = manual_control.y / 1000.0
            if -1000 <= manual_control.z <= 1000:
                controls.thrust = manual_control.z / 1000.0
            if -1000 <= manual_control.r <= 1000:
                controls.yaw = manual_control.r / 1000.0
            last_controls = controls
            last_controls_time = time()

    if manager_connected and time() > manager_timeout:
        manager_connected = False
        print('Lost connection to simulation manager')
