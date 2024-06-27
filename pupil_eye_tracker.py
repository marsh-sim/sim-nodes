#!/usr/bin/env python3

"""
Node providing EYE_TRACKING messages based on a Pupil Labs eye tracker.

For details about data received from the device, see:
https://docs.pupil-labs.com/core/developer/#gaze-datum-format
"""

from argparse import ArgumentParser
from math import nan, sqrt
import msgpack
from pymavlink import mavutil
from time import time
from typing import Any, Dict
import zmq

import mavlink_all as mavlink
from utils import NodeFormatter

parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
parser.add_argument('-m', '--manager',
                    help='MARSH Manager IP addr', default='127.0.0.1')
parser.add_argument('-r', '--pupil-remote',
                    help='Pupil Remote IP addr', default='127.0.0.1')
parser.add_argument('-p', '--pupil-port', type=int,
                    help='Pupil Remote port', default=50020)
args = parser.parse_args()
# assign to typed variables for convenience
args_manager: str = args.manager
args_pupil_remote: str = args.pupil_remote
args_pupil_port: int = args.pupil_port

ctx = zmq.Context()
pupil_remote = zmq.Socket(ctx, zmq.REQ)
pupil_remote.connect(f'tcp://{args_pupil_remote}:{args_pupil_port}')

# Request 'SUB_PORT' for reading data
pupil_remote.send_string('SUB_PORT')
subscribe_port = pupil_remote.recv_string()

subscriber = ctx.socket(zmq.SUB)
subscriber.connect(f'tcp://{args_pupil_remote}:{subscribe_port}')
subscriber.subscribe('gaze.')
subscriber.subscribe('surfaces.')

# create MAVLink connection
connection_string = f'udpout:{args_manager}:24400'
mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
mav.srcSystem = 1  # default system
mav.srcComponent = mavlink.MARSH_COMP_ID_EYE_TRACKER
print(f'Sending to {connection_string}')

start_time = time()
pupil_time_offset = None

# controlling when messages should be sent
heartbeat_next = 0.0
heartbeat_interval = 1.0

# monitoring connection to manager with heartbeat
timeout_interval = 5.0
manager_timeout = 0.0
manager_connected = False

max_surf_len = 0

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

    # Handle eye tracking data
    topic_bytes, payload = subscriber.recv_multipart()
    topic = topic_bytes.decode()
    pupil_message: Dict[str, Any] = msgpack.loads(payload)

    if topic == 'gaze.3d.01.':
        if pupil_time_offset is None:
            pupil_time_offset = (time() - start_time) - \
                pupil_message['timestamp']

        # point magnitude for normalization
        point_mag = sqrt(sum([p**2 for p in pupil_message['gaze_point_3d']]))

        mav.eye_tracking_data_send(
            round((pupil_message['timestamp'] + pupil_time_offset) * 1e6),
            nan, nan, nan,  # no info about head position
            # direction vector normalized to unit length
            pupil_message['gaze_point_3d'][0] / point_mag,
            pupil_message['gaze_point_3d'][1] / point_mag,
            pupil_message['gaze_point_3d'][2] / point_mag,
            # gaze position in video coordinates
            pupil_message['norm_pos'][0],
            pupil_message['norm_pos'][1],
            0, nan, nan,  # no surface tracking for now
        )
        print(
            f'gaze {len(pupil_message["base_data"])} {pupil_message["timestamp"]:.6f}')

    # TODO: Decide how to put in the corresponding gaze messages or decide that it's missing
    # The surface tracker publishes gaze on surface data in batches of no more than 10, also tested with more surfaces visible
    # The used timestamps are consistently all the gaze data in-between
    #
    # Based on these, assume there will be no surface data for a given gaze:
    #   - if it's older than 10 samples
    #   - if there was surface data for a newer gaze
    elif topic.startswith("surface"):
        max_surf_len = max(max_surf_len, len(
            pupil_message['gaze_on_surfaces']))
        print(
            f'surf {len(pupil_message["gaze_on_surfaces"])} {pupil_message["timestamp"]} max len: {max_surf_len}')
        for gaze_message in pupil_message['gaze_on_surfaces']:
            print(
                f'    gaze {len(gaze_message["base_data"])} {gaze_message["base_data"][1]:.6f} {gaze_message["base_data"][0]}')

    # handle incoming MAVLink messages
    try:
        while (message := mav.file.recv_msg()) is not None:
            message: mavlink.MAVLink_message
            if message.get_type() == 'HEARTBEAT':
                if message.get_srcComponent() == mavlink.MARSH_COMP_ID_MANAGER:
                    if not manager_connected:
                        print('Connected to simulation manager')
                    manager_connected = True
                    manager_timeout = time() + timeout_interval
    except ConnectionResetError:
        # thrown on Windows when there is no peer listening
        pass

    if manager_connected and time() > manager_timeout:
        manager_connected = False
        print('Lost connection to simulation manager')
