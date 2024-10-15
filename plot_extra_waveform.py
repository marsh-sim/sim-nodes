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
                    help='how much time to plot, in seconds', default=300)
parser.add_argument('-N', '--waveform-number', type=lambda s: check_number(int, s, True),
                    help='how many sine waves to use in the signal generation', default=71)
parser.add_argument('-f', '--min-frequency', type=lambda s: check_number(float, s, True),
                    help='minimal frequency of the generated signal, in Hertz', default=0.5)
parser.add_argument('-F', '--max-frequency', type=lambda s: check_number(float, s, True),
                    help='maximal frequency of the generated signal, in Hertz', default=7.5)
parser.add_argument('--seed', type=lambda s: check_number(int, s),
                    help='random number generator initial seed, between 0 and 2^24-1', default=0)
args = parser.parse_args()

pcg = PCG(args.seed)
signal = Multisine(pcg, args.waveform_number, args.min_frequency, args.max_frequency)
time = np.arange(0, args.plotted_time, args.time_interval)

x = np.zeros_like(time)
y = np.zeros_like(time)
z = np.zeros_like(time)

for i, t in enumerate(time):
    x[i], y[i], z[i] = signal.sample(t)

plt.plot(time, x, 'r', label='X')
plt.plot(time, y, 'g', label='Y')
plt.plot(time, z, 'b', label='Z')
plt.legend()
plt.show()
