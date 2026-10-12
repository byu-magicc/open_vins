"""Read HoloOcean truth for initialization and trajectory evaluation without ROSflight."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


NED_TO_ENU = Rotation.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])


def read_bag_truth(bag_path, offset_interval, imu_topic):
    try:
        from rosbags.highlevel import AnyReader, AnyReaderError
    except ImportError as error:
        raise ValueError('Install rosbags, numpy, and scipy in PLOTTER_PYTHON; see ReadMe.md') from error

    rows = []
    clock = []
    try:
        with AnyReader([Path(bag_path)]) as reader:
            truth_connections = [c for c in reader.connections if c.topic == '/sim/truth_state']
            imu_connections = [c for c in reader.connections if c.topic == imu_topic]
            if not truth_connections or not imu_connections:
                raise ValueError(f'{bag_path} requires /sim/truth_state and {imu_topic}')
            if any(c.msgtype != 'rosflight_msgs/msg/SimState' for c in truth_connections):
                raise ValueError('Expected SimState on /sim/truth_state')
            if any(c.msgtype != 'sensor_msgs/msg/Imu' for c in imu_connections):
                raise ValueError(f'Expected sensor_msgs/Imu on {imu_topic}')
            first = next(reader.messages(connections=imu_connections), None)
            if first is None:
                raise ValueError(f'{bag_path} has no IMU samples')
            message = reader.deserialize(first[2], first[0].msgtype)
            first_imu_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            # These simulator bags record truth on the simulation clock. Firmware IMU
            # headers lag this clock. Keep both clocks instead of guessing a fixed lag.
            start = None if offset_interval is None else int((first_imu_time + offset_interval[0] - 1.0) * 1e9)
            stop = None if offset_interval is None else int((first_imu_time + offset_interval[1] + 1.0) * 1e9)
            for connection, recorded, raw in reader.messages(
                    connections=truth_connections + imu_connections, start=start, stop=stop):
                message = reader.deserialize(raw, connection.msgtype)
                timestamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                if connection.topic == imu_topic:
                    clock.append([timestamp, recorded * 1e-9])
                    continue
                position = message.pose.position
                orientation = message.pose.orientation
                velocity = message.twist.linear
                rows.append([
                    timestamp, position.x, position.y, position.z,
                    orientation.x, orientation.y, orientation.z, orientation.w,
                    velocity.x, velocity.y, velocity.z,
                ])
    except AnyReaderError as error:
        raise ValueError(f'Unable to read truth from {bag_path}: {error}') from error
    if len(rows) < 2 or len(clock) < 2:
        raise ValueError(f'{bag_path} has insufficient truth/IMU coverage for the requested interval')
    rows = np.asarray(rows)
    rows = rows[np.argsort(rows[:, 0])]
    clock = np.asarray(clock)
    for data in (rows, clock):
        if not np.all(np.isfinite(data)) or np.any(np.diff(data[:, 0]) <= 0):
            raise ValueError(f'{bag_path} has non-finite data or non-increasing header timestamps')
    if np.any(np.diff(clock[:, 1]) <= 0):
        raise ValueError(f'{bag_path} has non-increasing IMU record timestamps')
    return rows, clock, first_imu_time


def enu_states(rows, timestamps):
    if np.any(timestamps < rows[0, 0]) or np.any(timestamps > rows[-1, 0]):
        raise ValueError('Requested timestamp is outside the truth coverage')
    # SimState uses NED positions and FRD body-to-world Hamilton quaternions.
    body_to_ned = Rotation.from_quat(rows[:, 4:8])
    world_velocities = body_to_ned.apply(rows[:, 8:11])
    positions = np.column_stack([np.interp(timestamps, rows[:, 0], rows[:, axis]) for axis in range(1, 4)])
    velocities = np.column_stack([np.interp(timestamps, rows[:, 0], world_velocities[:, axis]) for axis in range(3)])
    orientations = Slerp(rows[:, 0], body_to_ned)(timestamps)
    # Hamilton body-to-ENU coefficients also represent JPL ENU-to-IMU; do not conjugate.
    return np.column_stack((timestamps, (NED_TO_ENU * orientations).as_quat(),
                            NED_TO_ENU.apply(positions), NED_TO_ENU.apply(velocities), np.zeros((len(timestamps), 6))))


def extract_initial_state(bag_path, start_time):
    if not np.isfinite(start_time) or start_time < 0:
        raise ValueError('start_time must be finite and nonnegative')
    rows, clock, first_imu_time = read_bag_truth(bag_path, (start_time, start_time), '/imu/data')
    imu_time = first_imu_time + start_time
    if imu_time < clock[0, 0] or imu_time > clock[-1, 0]:
        raise ValueError('start_time is outside the IMU coverage')
    truth_time = np.interp(imu_time, clock[:, 0], clock[:, 1])
    state = enu_states(rows, np.array([truth_time]))[0]
    state[0] = imu_time
    print(f'HoloOcean truth start: offset={start_time:.9f}, IMU={imu_time:.9f}, '
          f'simulation={truth_time:.9f}, global ENU p={state[5:8]}, v={state[8:11]}', file=sys.stderr)
    return state.tolist()


def export_bag_truth(bag_path, output_directory, imu_topic):
    rows, clock, _ = read_bag_truth(bag_path, None, imu_topic)
    states = enu_states(rows, rows[:, 0])
    output_directory.mkdir(parents=True, exist_ok=True)
    np.savetxt(output_directory / 'truth.csv', states, delimiter=',', fmt='%.17g', comments='',
               header='timestamp,q_x,q_y,q_z,q_w,p_x,p_y,p_z,v_x,v_y,v_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z')
    np.savetxt(output_directory / 'clock.csv', clock, delimiter=',', fmt='%.17g', comments='',
               header='imu_timestamp,simulation_timestamp')



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract a global ENU HoloOcean truth state from a bag.')
    parser.add_argument('bag_path', type=Path)
    parser.add_argument('start_time', type=float, nargs='?', help='Seconds from the first /imu/data header timestamp')
    parser.add_argument('--export-dir', type=Path, help='Export ENU truth and clock CSVs for multi-agent bag playback')
    parser.add_argument('--topic-imu', default='/imu/data', help='IMU topic used for the exported clock mapping')
    arguments = parser.parse_args()
    try:
        if arguments.export_dir is not None:
            export_bag_truth(arguments.bag_path, arguments.export_dir, arguments.topic_imu)
        elif arguments.start_time is not None:
            print(json.dumps(extract_initial_state(arguments.bag_path, arguments.start_time), allow_nan=False))
        else:
            parser.error('start_time is required unless --export-dir is supplied')
    except (ValueError, OSError) as error:
        parser.exit(1, f'Error: {error}\n')
