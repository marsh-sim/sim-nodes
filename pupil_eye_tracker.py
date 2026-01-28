#!/usr/bin/env python3

"""
Node providing EYE_TRACKING messages based on a Pupil Labs eye tracker.

For details about data received from the device, see:
https://docs.pupil-labs.com/core/developer/#gaze-datum-format
"""

from argparse import ArgumentParser
from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import nan, sqrt
import msgpack
from pymavlink import mavutil
from time import time
from typing import Any, Deque, Dict, Literal, Optional, Tuple
import zmq

import mavlink_all as mavlink
from utils import NodeFormatter

parser = ArgumentParser(formatter_class=NodeFormatter, description=__doc__)
# fmt: off
parser.add_argument('-m', '--manager',
                    help='MARSH Manager IP addr', default='127.0.0.1')
parser.add_argument('-r', '--pupil-remote',
                    help='Pupil Remote IP addr', default='127.0.0.1')
parser.add_argument('-p', '--pupil-port', type=int,
                    help='Pupil Remote port', default=50020)
parser.add_argument('-v', '--verbose', action='store_true',
                    help='print sent data')
# fmt: on
args = parser.parse_args()
# assign to typed variables for convenience
args_manager: str = args.manager
args_pupil_remote: str = args.pupil_remote
args_pupil_port: int = args.pupil_port
args_verbose: bool = args.verbose

ctx = zmq.Context()
pupil_remote = zmq.Socket(ctx, zmq.REQ)
pupil_remote.connect(f"tcp://{args_pupil_remote}:{args_pupil_port}")

# Request 'SUB_PORT' for reading data
pupil_remote.send_string("SUB_PORT")
subscribe_port = pupil_remote.recv_string()

subscriber = ctx.socket(zmq.SUB)
subscriber.connect(f"tcp://{args_pupil_remote}:{subscribe_port}")
subscriber.subscribe("gaze.")
subscriber.subscribe("surfaces.")
subscriber.subscribe("pupil.")

# per fixation and blink
subscriber.subscribe("fixation")
subscriber.subscribe("blink")

poller = zmq.Poller()
poller.register(subscriber, zmq.POLLIN)

# create MAVLink connection
connection_string = f"udpout:{args_manager}:24400"
mav = mavlink.MAVLink(mavutil.mavlink_connection(connection_string))
mav.srcSystem = 1  # default system
mav.srcComponent = mavlink.MAV_COMP_ID_USER1 + (
    mavlink.MARSH_TYPE_EYE_TRACKER - mavlink.MARSH_TYPE_MANAGER
)
print(f"Sending to {connection_string}")


# see https://marsh-sim.github.io/nodes/eye-tracking/#defined-screens
class DefinedSurface(Enum):
    UNRECOGNIZED = 0
    INSTRUMENTS = 1
    OUTSIDE_VIEW = 2
    INCEPTORS = 3
    DRONE_CONTROL = 4


surface_names = {
    "Instruments": DefinedSurface.INSTRUMENTS,
    "Outside view": DefinedSurface.OUTSIDE_VIEW,
    "Inceptors": DefinedSurface.INCEPTORS,
    "Drone control": DefinedSurface.DRONE_CONTROL,
}


@dataclass
class GazeData:
    timestamp: float
    direction: Tuple[float, float, float]
    video: Tuple[float, float]
    surface: Optional[Tuple[DefinedSurface, float, float]] = None
    diameter_left: Optional[float] = None
    diameter_right: Optional[float] = None


start_time = time()
pupil_time_offset: Optional[float] = None

sending_queue: Deque[GazeData] = deque([], maxlen=10)
"""The surface tracker processes gaze data in order, in batches up to 10.
This buffer keeps the last 10 gaze data messages to add surface data to them before sending if available"""


def send_gaze_data(data: GazeData):
    offset = pupil_time_offset if pupil_time_offset is not None else 0.0
    time_usec = max(0, round((data.timestamp + offset) * 1e6))

    # Pupil Core and MARSH both use mm
    diameter = nan
    if data.diameter_left is not None and data.diameter_right is not None:
        diameter = (data.diameter_left + data.diameter_right) / 2.0
    if data.diameter_left is not None:
        diameter = data.diameter_left
    if data.diameter_right is not None:
        diameter = data.diameter_right

    if args_verbose:
        print(data)

    mav.eye_tracking_data_send(
        time_usec,
        0,  # default instance
        nan,
        nan,
        nan,  # no info about head position
        # direction vector normalized to unit length
        data.direction[0],
        data.direction[1],
        data.direction[2],
        # gaze position in video coordinates
        data.video[0],
        data.video[1],
        # gaze position on surface
        data.surface[0].value if data.surface is not None else 0,
        data.surface[1] if data.surface is not None else nan,
        data.surface[2] if data.surface is not None else nan,
        diameter,
    )
    
last_blink_onset: Optional[int] = None

# controlling when messages should be sent
heartbeat_next = 0.0
heartbeat_interval = 1.0

# monitoring connection to manager with heartbeat
timeout_interval = 5.0
manager_timeout = 0.0
manager_connected = False

while True:
    if time() >= heartbeat_next:
        mav.heartbeat_send(
            mavlink.MARSH_TYPE_EYE_TRACKER,
            mavlink.MAV_AUTOPILOT_INVALID,
            mavlink.MAV_MODE_FLAG_TEST_ENABLED,
            0,
            mavlink.MAV_STATE_ACTIVE,
        )
        heartbeat_next = time() + heartbeat_interval

    # Receive one eye tracking message in non-blocking mode
    topic = ""
    pupil_message: Optional[Dict[str, Any]] = None
    evts = dict(poller.poll())
    if subscriber in evts:
        topic_bytes, payload = subscriber.recv_multipart(flags=zmq.NOBLOCK)
        topic = topic_bytes.decode()
        pupil_message = msgpack.loads(payload)

        if pupil_time_offset is None:
            pupil_time_offset = (time() - start_time) - pupil_message["timestamp"]

    if topic == "gaze.3d.01.":
        # point magnitude for normalization
        point_mag = sqrt(sum([p**2 for p in pupil_message["gaze_point_3d"]]))

        data = GazeData(
            timestamp=pupil_message["timestamp"],
            direction=(
                pupil_message["gaze_point_3d"][0] / point_mag,
                pupil_message["gaze_point_3d"][1] / point_mag,
                pupil_message["gaze_point_3d"][2] / point_mag,
            ),
            video=(
                pupil_message["norm_pos"][0],
                pupil_message["norm_pos"][1],
            ),
        )

        if len(sending_queue) == sending_queue.maxlen:
            # send the oldest message
            send_gaze_data(sending_queue.popleft())
        sending_queue.append(data)

    elif topic == "pupil.0.3d":
        # if args_verbose:
        #     print("right diameter:", pupil_message["diameter_3d"])
        try:
            sending_queue[-1].diameter_right = pupil_message["diameter_3d"]
        except IndexError:
            pass  # empty queue
    elif topic == "pupil.1.3d":
        # if args_verbose:
        #     print("left diameter:", pupil_message["diameter_3d"])
        try:
            sending_queue[-1].diameter_left = pupil_message["diameter_3d"]
        except IndexError:
            pass  # empty queue

    elif topic.startswith("surface"):
        for gaze_message in pupil_message["gaze_on_surfaces"]:
            # assign to corresponding data waiting to be sent
            for data in sending_queue:
                # make sure the timestamp matches and gaze is in the surface
                if (
                    data.timestamp == gaze_message["base_data"][1]
                    and 0.0 <= gaze_message["norm_pos"][0] <= 1.0
                    and 0.0 <= gaze_message["norm_pos"][1] <= 1.0
                ):
                    data.surface = (
                        surface_names.get(
                            pupil_message["name"], DefinedSurface.UNRECOGNIZED
                        ),
                        gaze_message["norm_pos"][0],
                        gaze_message["norm_pos"][1],
                    )
                    #test for blink and fixation
    elif topic.startswith("fixation"):
        offset = pupil_time_offset if pupil_time_offset is not None else 0.0
        time_usec = max(0, round((pupil_message["timestamp"] + offset) * 1e6))

        if args_verbose:
            print("FIXATION:", time_usec, pupil_message["duration"] * 1000.0, pupil_message["confidence"], pupil_message["dispersion"])

        mav.eye_tracking_event_send(
            time_usec,
            0,
            mavlink.EYE_TRACKING_EVENT_TYPE_FIXATION,
            round(pupil_message["duration"] * 1000.0),
            pupil_message["confidence"],
            pupil_message["dispersion"],
        )

    elif topic.startswith("blink"):
        offset = pupil_time_offset if pupil_time_offset is not None else 0.0
        time_usec = max(0, round((pupil_message["timestamp"] + offset) * 1e6))

        if pupil_message["type"] == "onset":
            last_blink_onset = time_usec
        elif pupil_message["type"] == "offset" and last_blink_onset is not None:

            if args_verbose:
                print("BLINK:", time_usec, time_usec - last_blink_onset, pupil_message["confidence"])

            mav.eye_tracking_event_send(
                time_usec,
                0,
                mavlink.EYE_TRACKING_EVENT_TYPE_BLINK,
                time_usec - last_blink_onset,
                pupil_message["confidence"],
                nan,
            )
            last_blink_onset = None

    # else:
    #     if args_verbose:
    #         print("other topic:", topic)

    # handle incoming MAVLink messages
    try:
        while (message := mav.file.recv_msg()) is not None:
            message: mavlink.MAVLink_message
            if message.get_type() == "HEARTBEAT":
                heartbeat: mavlink.MAVLink_heartbeat_message = message
                if heartbeat.type == mavlink.MARSH_TYPE_MANAGER:
                    if not manager_connected:
                        print("Connected to simulation manager")
                    manager_connected = True
                    manager_timeout = time() + timeout_interval
    except ConnectionResetError:
        # thrown on Windows when there is no peer listening
        pass

    if manager_connected and time() > manager_timeout:
        manager_connected = False
        print("Lost connection to simulation manager")
