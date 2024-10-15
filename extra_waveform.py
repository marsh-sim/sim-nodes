#!/usr/bin/env python3

"""
Node for sending MOTION_CUE_EXTRA message with a generated waveform
"""

from argparse import ArgumentParser
from collections import OrderedDict
import numpy as np
from pymavlink import mavutil
from random import randint
from time import time

import mavlink_all as mavlink
from utils import check_number, NodeFormatter
from utils.pcg import PCG

def main():
    parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)

    parser.add_argument('-m', '--manager',
                        help='MARSH Manager IP addr', default='127.0.0.1')
    # random default, not used for any specific component so far
    parser.add_argument('-c', '--component', type=lambda s: check_number(int, s, True),
                        help='component ID to use', default=mavlink.MAV_COMP_ID_USER47)
    parser.add_argument('-t', '--time-interval', type=lambda s: check_number(float, s),
                        help='how much time between sending messages, in seconds', default=0.01)

    parser.add_argument('-N', '--waveform-number', type=lambda s: check_number(int, s, True),
                        help='how many sine waves to use in the signal generation', default=71)
    parser.add_argument('-f', '--min-frequency', type=lambda s: check_number(float, s, True),
                        help='minimal frequency of the generated signal, in Hertz', default=0.5)
    parser.add_argument('-F', '--max-frequency', type=lambda s: check_number(float, s, True),
                        help='maximal frequency of the generated signal, in Hertz', default=7.5)
    parser.add_argument('--seed', type=lambda s: check_number(int, s),
                        help='random number generator initial seed, between 0 and 2^24-1')

    parser.add_argument('-x', type=lambda s: check_number(float, s),
                        help='initial amplitude in X axis, in meters per second squared RMS', default=0.0)
    parser.add_argument('-y', type=lambda s: check_number(float, s),
                        help='initial amplitude in Y axis, in meters per second squared RMS', default=0.0)
    parser.add_argument('-z', type=lambda s: check_number(float, s),
                        help='initial amplitude in Z axis, in meters per second squared RMS', default=0.0)
    parser.add_argument('--ramp-time', type=lambda s: check_number(float, s, True),
                        help='how long to interpolate the amplitude towards new value, in seconds', default=5.0)

    args = parser.parse_args()
    # assign to typed variables for convenience
    args_manager: str = args.manager
    args_component: int = args.component
    args_time_interval: float = args.time_interval
    args_ramp_time: float = args.ramp_time

    N: int = args.waveform_number
    F_MIN: float = args.min_frequency
    F_MAX: float = args.max_frequency
    # limit the range of automatic random seed to make them easier to remember
    SEED: int = args.seed if (args.seed is not None) else randint(0, 10000)

    args_x: float = args.x
    args_y: float = args.y
    args_z: float = args.z

    # validate arguments
    if F_MIN >= F_MAX:
        parser.error("Minimum frequency must be smaller than maximum frequency")
    if SEED > 2**24 - 1:
        parser.error("Random generator seed must be exactly representable in 32-bit float")

    connection_string = f'udpout:{args_manager}:24400'
    mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
    mav.srcSystem = 1  # default system
    mav.srcComponent = args_component
    print(f'Sending to {connection_string}')

    pcg = PCG(SEED)
    signal = Multisine(pcg, N, F_MIN, F_MAX)

    # create parameters database, all parameters are float to simplify code
    params: OrderedDict[str, float] = OrderedDict()
    params['ACC_RMS_X'] = args_x
    params['ACC_RMS_Y'] = args_y
    params['ACC_RMS_Z'] = args_z
    params['RAMP_TIME'] = args_ramp_time
    params['FREQ_NUM_CONST'] = N
    params['FREQ_MIN_CONST'] = F_MIN
    params['FREQ_MAX_CONST'] = F_MAX
    params['PCG_SEED_CONST'] = SEED

    for k in params.keys():
        assert len(k) <= 16, 'parameter names must fit into param_id field'

    readonly_params: set[str] = set()
    for k in params.keys():
        if k.endswith('CONST'):
            readonly_params.add(k)

    
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

        if time() >= signal_next:
            sample_time = time() - start_time
            sig_x, sig_y, sig_z = signal.sample(sample_time)

            # TODO: Amplitude interpolation with ramp time
            acc_x = sig_x * params['ACC_RMS_X']
            acc_y = sig_y * params['ACC_RMS_Y']
            acc_z = sig_z * params['ACC_RMS_Z']

            mav.motion_cue_extra_send(
                round(sample_time * 1000),
                0, 0, 0,  # pitch, roll, yaw velocity
                acc_x, acc_y, acc_z
            )
            signal_next = time() + signal_interval

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
                elif message.get_type() in ['PARAM_REQUEST_READ', 'PARAM_REQUEST_LIST', 'PARAM_SET']:
                    # check that this is relevant to us
                    if message.target_system == mav.srcSystem and message.target_component == mav.srcComponent:
                        if message.get_type() == 'PARAM_REQUEST_READ':
                            m: mavlink.MAVLink_param_request_read_message = message
                            send_param(mav, params, m.param_index, m.param_id)
                        elif message.get_type() == 'PARAM_REQUEST_LIST':
                            for i in range(len(params)):
                                send_param(mav, params, i)
                        elif message.get_type() == 'PARAM_SET':
                            m: mavlink.MAVLink_param_set_message = message
                            # check that parameter is defined and sent as float
                            if m.param_id in params and m.param_type == mavlink.MAV_PARAM_TYPE_REAL32:
                                # don't write to read-only parameters but report the value
                                if m.param_id not in readonly_params:
                                    params[m.param_id] = m.param_value
                            send_param(mav, params, -1, m.param_id)
        except ConnectionResetError:
            # thrown on Windows when there is no peer listening
            pass

        if manager_connected and time() > manager_timeout:
            manager_connected = False
            print('Lost connection to simulation manager')


def send_param(mav: mavlink.MAVLink, params: OrderedDict[str, float], index: int, name: str=''):
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

    mav.param_value_send(bytes(param_id), params[name], mavlink.MAV_PARAM_TYPE_REAL32,
                         len(params), index)

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
   
if __name__ == '__main__':
    main()
