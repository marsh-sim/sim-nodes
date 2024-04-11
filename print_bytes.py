from pymavlink import mavutil
from sys import stdout

import mavlink_all as mavlink

message = mavlink.MAVLink_manual_control_message(1, 2, 3, 4, 5, 0)

mav = mavlink.MAVLink(None, srcSystem=100, srcComponent=101)
data = message.pack(mav)

mavutil.dump_message_verbose(stdout, message)
print()
print('len:', len(data))
print('dec:', '  '.join('{:>3}'.format(d) for d in data))
print('hex:', '  '.join(' {:02X}'.format(d) for d in data))
print('idx:', '  '.join('{:>3}'.format(i % 100) for i in range(len(data))))
