from argparse import ArgumentParser
import matplotlib.pyplot as plt
import numpy as np

from extra_waveform import Multisine
from utils import check_number, NodeFormatter
from utils.pcg import PCG

parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
parser.add_argument('-t', '--time-interval', type=lambda s: check_number(float, s, True),
                    help='how much time between sending messages, in seconds', default=0.01)
parser.add_argument('-T', '--plotted-time', type=lambda s: check_number(float, s, True),
                    help='how much time to plot, in seconds', default=30)
parser.add_argument('-N', '--waveform-number', type=lambda s: check_number(int, s, True),
                    help='how many sine waves to use in the signal generation', default=71)
parser.add_argument('-f', '--min-frequency', type=lambda s: check_number(float, s, True),
                    help='minimal frequency of the generated signal, in Hertz', default=0.5)
parser.add_argument('-F', '--max-frequency', type=lambda s: check_number(float, s, True),
                    help='maximal frequency of the generated signal, in Hertz', default=7.5)
parser.add_argument('--seed', type=lambda s: check_number(int, s),
                    help='random number generator initial seed, between 0 and 2^24-1', default=0)
axis = parser.add_mutually_exclusive_group()
axis.add_argument('--x-only', action='store_true', help='only plot X axis')
axis.add_argument('--y-only', action='store_true', help='only plot Y axis')
axis.add_argument('--z-only', action='store_true', help='only plot Z axis')
args = parser.parse_args()

pcg = PCG(args.seed)
signal = Multisine(pcg, args.waveform_number, args.min_frequency, args.max_frequency)
time = np.arange(0, args.plotted_time, args.time_interval)

x = np.zeros_like(time)
y = np.zeros_like(time)
z = np.zeros_like(time)

for i, t in enumerate(time):
    x[i], y[i], z[i] = signal.sample(t)

plt.figure('Accelerations')
if not (args.y_only or args.z_only):
    plt.plot(time, x, 'r', label=f'X (RMS {np.sqrt(np.mean(x**2)):.3f})')
if not (args.x_only or args.z_only):
    plt.plot(time, y, 'g', label=f'Y (RMS {np.sqrt(np.mean(y**2)):.3f})')
if not (args.x_only or args.y_only):
    plt.plot(time, z, 'b', label=f'Z (RMS {np.sqrt(np.mean(z**2)):.3f})')
plt.legend()
plt.xlabel('Time, s')
plt.ylabel('Acceleration, m/s^2')
plt.show()

# Numeric integration to show velocities
vx = np.cumsum(x) * args.time_interval
vy = np.cumsum(y) * args.time_interval
vz = np.cumsum(z) * args.time_interval

plt.figure('Velocities')
if not (args.y_only or args.z_only):
    plt.plot(time, vx, 'r', label='X')
if not (args.x_only or args.z_only):
    plt.plot(time, vy, 'g', label='Y')
if not (args.x_only or args.y_only):
    plt.plot(time, vz, 'b', label='Z')
plt.legend()
plt.xlabel('Time, s')
plt.ylabel('Velocity, m/s')
plt.show()
