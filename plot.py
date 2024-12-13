import argparse
import matplotlib.pyplot as plt
from pymavlink import mavutil

import mavlink_all as mavlink

parser = argparse.ArgumentParser()
parser.add_argument('input')
parser.add_argument('plot')
args = parser.parse_args()

args_input: str = args.input
args_plot: str = args.plot

data_x, data_y = tuple(args_plot.split(','))
msg_x, field_x = tuple(data_x.split('.'))
msg_y, field_y = tuple(data_y.split('.'))

x = []
y = []
mav = mavlink.MAVLink(None)
with open(args_input, 'rb') as file:
    while (byte := file.read(1)):
        try:
            message = mav.parse_char(byte)
            if message is not None:
                # print(message)
                message: mavlink.MAVLink_message
                if message.get_type() == msg_x:
                    x.append(message.to_dict()[field_x])
                if message.get_type() == msg_y:
                    y.append(message.to_dict()[field_y])
        except:
            pass

plt.plot(x, y)
plt.show()
