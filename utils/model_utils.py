"""
Constants, functions and types useful for implementing flight models (MARSH_COMP_ID_FLIGHT_MODEL)
"""

from dataclasses import dataclass
from math import cos, sin
from typing import Tuple

import mavlink_all as mavlink

STD_G = 9.80665
"""Standard gravity on Earth, in meters per second squared"""


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
    """
    Convenience wrapper for main control axes
    """
    pitch = 0.0
    """positive nose down"""
    roll = 0.0
    """positive roll right"""
    thrust = 0.0
    """positive go up"""
    yaw = 0.0
    """positive turn right"""

    @classmethod
    def from_message(cls, message: mavlink.MAVLink_message) -> 'Controls':
        assert message.get_type() == 'MANUAL_CONTROL'

        # this line only helps with type hints
        manual_control: mavlink.MAVLink_manual_control_message = message

        controls = cls()
        # assign only valid axes
        if -1000 <= manual_control.x <= 1000:
            controls.pitch = manual_control.x / 1000.0
        if -1000 <= manual_control.y <= 1000:
            controls.roll = manual_control.y / 1000.0
        if -1000 <= manual_control.z <= 1000:
            controls.thrust = manual_control.z / 1000.0
        if -1000 <= manual_control.r <= 1000:
            controls.yaw = manual_control.r / 1000.0
        return controls
