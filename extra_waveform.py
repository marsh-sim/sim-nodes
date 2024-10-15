#!/usr/bin/env python3

"""
Node for sending MOTION_CUE_EXTRA message with a generated waveform
"""

from argparse import ArgumentParser
import numpy as np
from pymavlink import mavutil
from random import randint

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

    args = parser.parse_args()
    # assign to typed variables for convenience
    args_manager: str = args.manager
    args_component: int = args.component
    args_time_interval: float = args.time_interval

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

class Multisine:
    def __init__(self, rng: PCG, n: int, f_min: float, f_max: float):
        # Generate the signal
        AMPLITUDE_SCALE = 1.0 / np.sqrt((n - 1) / 2)
        """Scale factor for all amplitudes to create signal with RMS of 1"""
        frequencies = np.linspace(f_min, f_max, n)

        self.angular_frequencies = frequencies * 2 * np.pi
        self.amplitudes = AMPLITUDE_SCALE / np.square(self.angular_frequencies)

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
