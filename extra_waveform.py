#!/usr/bin/env python3

"""
Node for sending MOTION_CUE_EXTRA message with a generated waveform
"""

from argparse import ArgumentParser
from collections.abc import Sequence
import numpy as np
from pymavlink import mavutil
from random import randint
from time import time
from typing import TypeVar

import mavlink_all as mavlink
from utils import check_number, NodeFormatter
from utils.param_dict import ParamDict
from utils.pcg import PCG


def main():
    parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__, epilog=f'''
For arguments with multiple default values, they are selected based on the --output argument.
When sending each output:
- MOTION_CUE_EXTRA: XYZ correspond to accelerations in meters per second squared.
- MANUAL_SETPOINT: XYZ correspond to roll, pitch in rad/s and normalized thrust.
''')

    output_choices = ['MOTION_CUE_EXTRA', 'MANUAL_SETPOINT']
    type_choices = [mavlink.MARSH_TYPE_VIBRATION_SOURCE, mavlink.MARSH_TYPE_PILOT_TARGET]
    component_choices = [mavlink.MAV_COMP_ID_USER1 + (t - mavlink.MARSH_TYPE_MANAGER) for t in type_choices]

    parser.add_argument('-m', '--manager',
                        help='MARSH Manager IP addr', default='127.0.0.1')
    parser.add_argument('-o', '--output', choices=output_choices,
                        help='which MAVLink message to send the signal in', default=output_choices[0])
    parser.add_argument('-c', '--component', type=lambda s: check_number(int, s, True),
                        help='component ID to use', default=component_choices)
    parser.add_argument('-t', '--time-interval', type=lambda s: check_number(float, s),
                        help='how much time between sending messages, in seconds', default=0.01)
    parser.add_argument('-N', '--waveform-number', type=lambda s: check_number(int, s, True),
                        help='how many sine waves to use in the signal generation', default=[71, 5])
    parser.add_argument('-f', '--min-frequency', type=lambda s: check_number(float, s, True),
                        help='minimal frequency of the generated signal, in Hertz', default=[0.5, 0.05])
    parser.add_argument('-F', '--max-frequency', type=lambda s: check_number(float, s, True),
                        help='maximal frequency of the generated signal, in Hertz', default=[7.5, 0.2])
    parser.add_argument('--seed', type=lambda s: check_number(int, s),
                        help='random number generator initial seed, between 0 and 2^24-1')

    parser.add_argument('-x', type=lambda s: check_number(float, s),
                        help='initial amplitude in X axis, RMS in signal units', default=[0.0, 0.05])
    parser.add_argument('-y', type=lambda s: check_number(float, s),
                        help='initial amplitude in Y axis, RMS in signal units', default=[0.0, 0.05])
    parser.add_argument('-z', type=lambda s: check_number(float, s),
                        help='initial amplitude in Z axis, RMS in signal units', default=[0.0, 0.025])
    parser.add_argument('--offset-x', type=lambda s: check_number(float, s, True, True),
                        help='initial offset in X axis, in signal units', default=0.0)
    parser.add_argument('--offset-y', type=lambda s: check_number(float, s, True, True),
                        help='initial offset in Y axis, in signal units', default=0.0)
    parser.add_argument('--offset-z', type=lambda s: check_number(float, s, True, True),
                        help='initial offset in Z axis, in signal units', default=[0.0, 0.5])
    parser.add_argument('--ramp-time', type=lambda s: check_number(float, s, True),
                        help='how long to interpolate the amplitude towards new value, in seconds', default=5.0)

    args = parser.parse_args()
    # assign all arguments to typed variables for convenience
    args_manager: str = args.manager
    args_output: str = args.output

    T = TypeVar('T')

    def select_default(arg_value: Sequence[T] | T) -> T:
        if isinstance(arg_value, Sequence):
            return arg_value[output_choices.index(args_output)]  # pyright:ignore[reportUnknownVariableType]
        else:
            return arg_value

    args_component: int = select_default(args.component)
    args_time_interval: float = args.time_interval
    args_ramp_time: float = args.ramp_time
    mav_type: int = type_choices[output_choices.index(args_output)]

    N: int = select_default(args.waveform_number)
    F_MIN: float = select_default(args.min_frequency)
    F_MAX: float = select_default(args.max_frequency)
    # limit the range of automatic random seed to make them easier to remember
    SEED: int = args.seed if (args.seed is not None) else randint(0, 10000)

    print(N, F_MIN, F_MAX)

    args_x: float = select_default(args.x)
    args_y: float = select_default(args.y)
    args_z: float = select_default(args.z)
    args_offset_x: float = select_default(args.offset_x)
    args_offset_y: float = select_default(args.offset_y)
    args_offset_z: float = select_default(args.offset_z)

    # validate arguments
    if F_MIN >= F_MAX:
        parser.error(
            "Minimum frequency must be smaller than maximum frequency")
    if SEED > 2**24 - 1:
        parser.error(
            "Random generator seed must be exactly representable in 32-bit float")

    connection_string = f'udpout:{args_manager}:24400'
    mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
    mav.srcSystem = 1  # default system
    mav.srcComponent = args_component
    print(f'Sending to {connection_string}')

    pcg = PCG(SEED)
    signal = Multisine(pcg, N, F_MIN, F_MAX)

    # create parameters database, all parameters are float to simplify code
    params = ParamDict()
    params['RMS_X'] = args_x
    params['RMS_Y'] = args_y
    params['RMS_Z'] = args_z
    params['OFFSET_X'] = args_offset_x
    params['OFFSET_Y'] = args_offset_y
    params['OFFSET_Z'] = args_offset_z
    params['RAMP_TIME'] = args_ramp_time
    params['FREQ_NUM_CONST'] = N
    params['FREQ_MIN_CONST'] = F_MIN
    params['FREQ_MAX_CONST'] = F_MAX
    params['PCG_SEED_CONST'] = SEED

    for k in params.keys():
        if k.endswith('CONST'):
            params.readonly_names.add(k)

    # controlling when messages should be sent
    heartbeat_next = 0.0
    heartbeat_interval = 1.0
    signal_next = 0.0
    signal_interval = args_time_interval

    # monitoring connection to manager with heartbeat
    timeout_interval = 5.0
    manager_timeout = 0.0
    manager_connected = False

    start_time = time()

    # linear soft start
    amplitude_x = RampValue(0.0)
    amplitude_y = RampValue(0.0)
    amplitude_z = RampValue(0.0)
    offset_x = RampValue(0.0)
    offset_y = RampValue(0.0)
    offset_z = RampValue(0.0)
    amplitude_x.set(0.0, params['RMS_X'])
    amplitude_y.set(0.0, params['RMS_Y'])
    amplitude_z.set(0.0, params['RMS_Z'])
    offset_x.set(0.0, params['OFFSET_X'])
    offset_y.set(0.0, params['OFFSET_Y'])
    offset_z.set(0.0, params['OFFSET_Z'])

    # the loop goes as fast as it can, relying on the variables above for timing
    while True:
        if time() >= heartbeat_next:
            mav.heartbeat_send(
                mav_type,
                mavlink.MAV_AUTOPILOT_INVALID,
                mavlink.MAV_MODE_FLAG_TEST_ENABLED,
                0,
                mavlink.MAV_STATE_ACTIVE
            )
            heartbeat_next = time() + heartbeat_interval

        if time() >= signal_next:
            sample_time = time() - start_time
            sig_x, sig_y, sig_z = signal.sample(sample_time)

            ramp_time = params['RAMP_TIME']
            out_x = sig_x * amplitude_x.get(ramp_time, sample_time) \
                + offset_x.get(ramp_time, sample_time)
            out_y = sig_y * amplitude_y.get(ramp_time, sample_time) \
                + offset_y.get(ramp_time, sample_time)
            out_z = sig_z * amplitude_z.get(ramp_time, sample_time) \
                + offset_z.get(ramp_time, sample_time)

            if args_output == "MOTION_CUE_EXTRA":
                mav.motion_cue_extra_send(
                    round(sample_time * 1000),
                    0, 0, 0,  # pitch, roll, yaw velocity
                    out_x, out_y, out_z
                )
            elif args_output == "MANUAL_SETPOINT":
                mav.manual_setpoint_send(
                    round(sample_time * 1000),
                    out_x, out_y, 0.0,  # roll, pitch, yaw
                    (out_z / 2 + 0.5),  # thrust scaled from -1..1 to 0..1
                    0, 0
                )
            signal_next = time() + signal_interval

        # handle incoming messages
        try:
            while (message := mav.file.recv_msg()) is not None:
                message: mavlink.MAVLink_message
                if message.get_type() == 'HEARTBEAT':
                    heartbeat: mavlink.MAVLink_heartbeat_message = message
                    if heartbeat.type == mavlink.MARSH_TYPE_MANAGER:
                        if not manager_connected:
                            print('Connected to simulation manager')
                        manager_connected = True
                        manager_timeout = time() + timeout_interval
                elif params.should_handle_message(message):
                    changed = params.handle_message(mav, message)
                    if changed is not None:
                        name, value = changed

                        # update the interpolated amplitudes
                        sample_time = time() - start_time
                        if name == 'RMS_X':
                            amplitude_x.set(sample_time, value)
                        elif name == 'RMS_Y':
                            amplitude_y.set(sample_time, value)
                        elif name == 'RMS_Z':
                            amplitude_z.set(sample_time, value)
                        elif name == 'OFFSET_X':
                            offset_x.set(sample_time, value)
                        elif name == 'OFFSET_Y':
                            offset_y.set(sample_time, value)
                        elif name == 'OFFSET_Z':
                            offset_z.set(sample_time, value)
        except ConnectionResetError:
            # thrown on Windows when there is no peer listening
            pass

        if manager_connected and time() > manager_timeout:
            manager_connected = False
            print('Lost connection to simulation manager')


class Multisine:
    def __init__(self, rng: PCG, n: int, f_min: float, f_max: float):
        # Generate the signal
        AMPLITUDE_SCALE = 1.0 / np.sqrt((n - 1) / 2)
        """Scale factor for all amplitudes to create signal with RMS of 1"""
        frequencies = np.linspace(f_min, f_max, n)

        self.angular_frequencies = frequencies * 2 * np.pi
        self.amplitudes = AMPLITUDE_SCALE

        # Randomly initialised between 0 and 1, for three directions
        phase_factors = np.zeros([3, frequencies.shape[0]])
        for i in range(frequencies.shape[0]):
            for dir in range(3):
                phase_factors[dir, i] = rng.uniform(0, 1)

        self.phases = 2 * np.pi * phase_factors

    def sample(self, time: float) -> tuple[float, float, float]:
        arguments = time * self.angular_frequencies + self.phases
        signals = np.sum(self.amplitudes * np.sin(arguments), axis=1)
        return signals[0], signals[1], signals[2]


class RampValue:
    """
    Linearly change the output to new value over ramp_time
    """

    def __init__(self, value: float):
        self.start_value = value
        self.target_value = value
        self.ramp_fraction = 1.0
        self.last_update_time: float | None = None

    def _lerp(self) -> float:
        return self.start_value + (self.target_value - self.start_value) * self.ramp_fraction

    def set(self, time: float, value: float):
        self.start_value = self._lerp()  # set to last output
        # now interpolate to new value
        self.target_value = value
        self.ramp_fraction = 0.0
        self.last_update_time = time

    def get(self, ramp_time: float, time: float) -> float:
        if self.ramp_fraction < 1.0 and self.last_update_time is not None:
            # update ramp fraction
            ramp_change = (time - self.last_update_time) / ramp_time
            self.ramp_fraction = min(1.0, self.ramp_fraction + ramp_change)
        self.last_update_time = time
        return self._lerp()


if __name__ == '__main__':
    main()
