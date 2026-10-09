"""Read HoloOcean truth for initialization and trajectory evaluation without ROSflight."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


NED_TO_ENU = Rotation.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])


def read_bag_truth(bag_path, offset_interval):
    try:
        from rosbags.highlevel import AnyReader, AnyReaderError
    except ImportError as error:
        raise ValueError('Install rosbags, numpy, and scipy in PLOTTER_PYTHON; see ReadMe.md') from error

    rows = []
    clock = []
    try:
        with AnyReader([Path(bag_path)]) as reader:
            truth_connections = [c for c in reader.connections if c.topic == '/sim/truth_state']
            imu_connections = [c for c in reader.connections if c.topic == '/imu/data']
            if not truth_connections or not imu_connections:
                raise ValueError(f'{bag_path} requires /sim/truth_state and /imu/data')
            if any(c.msgtype != 'rosflight_msgs/msg/SimState' for c in truth_connections):
                raise ValueError('Expected SimState on /sim/truth_state')
            if any(c.msgtype != 'sensor_msgs/msg/Imu' for c in imu_connections):
                raise ValueError('Expected sensor_msgs/Imu on /imu/data')
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
                if connection.topic == '/imu/data':
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


def interpolate_truth(rows, timestamps):
    if np.any(timestamps < rows[0, 0]) or np.any(timestamps > rows[-1, 0]):
        raise ValueError('Requested timestamp is outside the truth coverage')
    # SimState uses NED positions and FRD body-to-world Hamilton quaternions.
    body_to_ned = Rotation.from_quat(rows[:, 4:8])
    world_velocities = body_to_ned.apply(rows[:, 8:11])
    positions = np.column_stack([np.interp(timestamps, rows[:, 0], rows[:, axis]) for axis in range(1, 4)])
    velocities = np.column_stack([np.interp(timestamps, rows[:, 0], world_velocities[:, axis]) for axis in range(3)])
    orientations = Slerp(rows[:, 0], body_to_ned)(timestamps)
    return positions, orientations, velocities


def extract_initial_state(bag_path, start_time):
    if not np.isfinite(start_time) or start_time < 0:
        raise ValueError('start_time must be finite and nonnegative')
    rows, clock, first_imu_time = read_bag_truth(bag_path, (start_time, start_time))
    imu_time = first_imu_time + start_time
    if imu_time < clock[0, 0] or imu_time > clock[-1, 0]:
        raise ValueError('start_time is outside the IMU coverage')
    truth_time = np.interp(imu_time, clock[:, 0], clock[:, 1])
    positions, orientations, velocities = interpolate_truth(rows, np.array([truth_time]))
    # The coefficients of this Hamilton body-to-ENU quaternion also represent
    # OpenVINS's JPL ENU-to-IMU quaternion. Do not conjugate the coefficients.
    state = np.concatenate((
        [imu_time], (NED_TO_ENU * orientations).as_quat()[0],
        NED_TO_ENU.apply(positions)[0], NED_TO_ENU.apply(velocities)[0], np.zeros(6),
    ))
    print(f'HoloOcean truth start: offset={start_time:.9f}, IMU={imu_time:.9f}, '
          f'simulation={truth_time:.9f}, global ENU p={state[5:8]}, v={state[8:11]}', file=sys.stderr)
    return state.tolist()


def load_bag_reference(bag_path, estimate, camera_imu_offset):
    rows, clock, _ = read_bag_truth(bag_path, None)
    estimate = np.sort(estimate, order='timestamp').copy()
    imu_times = estimate['timestamp'] + camera_imu_offset
    overlap = (imu_times >= clock[0, 0]) & (imu_times <= clock[-1, 0])
    estimate = estimate[overlap]
    reference_times = np.interp(imu_times[overlap], clock[:, 0], clock[:, 1])
    overlap = (reference_times >= rows[0, 0]) & (reference_times <= rows[-1, 0])
    estimate = estimate[overlap]
    reference_times = reference_times[overlap]
    if estimate.size < 2:
        raise ValueError(f'Fewer than two estimate timestamps overlap truth in {bag_path}')
    timestamps = estimate['timestamp']
    positions, interpolated_rotations, velocities = interpolate_truth(rows, reference_times)
    truth = np.empty(estimate.size, dtype=[(name, float) for name in (
        'timestamp', 'p_x', 'p_y', 'p_z', 'q_x', 'q_y', 'q_z', 'q_w', 'v_x', 'v_y', 'v_z',
    )])
    truth['timestamp'] = timestamps
    for axis, name in enumerate(('x', 'y', 'z')):
        truth[f'p_{name}'] = positions[:, axis]
        truth[f'v_{name}'] = velocities[:, axis]
    truth_quaternions = interpolated_rotations.as_quat()
    for axis, name in enumerate(('x', 'y', 'z', 'w')):
        truth[f'q_{name}'] = truth_quaternions[:, axis]
    enu_to_ned = NED_TO_ENU
    # OpenVINS stores JPL global-to-IMU quaternions. Interpreted as Hamilton
    # quaternions by SciPy, the same coefficients represent IMU-to-global.
    estimate_rotations = enu_to_ned * Rotation.from_quat(np.column_stack([
        estimate[f'q_{name}'] for name in ('x', 'y', 'z', 'w')
    ]))
    positions = enu_to_ned.apply(np.column_stack([estimate[f'p_{name}'] for name in ('x', 'y', 'z')]))
    velocities = enu_to_ned.apply(np.column_stack([estimate[f'v_{name}'] for name in ('x', 'y', 'z')]))
    quaternions = estimate_rotations.as_quat()
    for axis, name in enumerate(('x', 'y', 'z')):
        estimate[f'p_{name}'] = positions[:, axis]
        estimate[f'v_{name}'] = velocities[:, axis]
    for axis, name in enumerate(('x', 'y', 'z', 'w')):
        estimate[f'q_{name}'] = quaternions[:, axis]
    # Position and velocity uncertainties are world-frame quantities. Attitude
    # covariance is a body-frame perturbation and is unchanged by this world-frame conversion.
    rotation = enu_to_ned.as_matrix()
    for start in (3, 6):
        covariance = np.array([
            [estimate[f'cov_{row}_{column}'] for column in range(start, start + 3)]
            for row in range(start, start + 3)
        ]).transpose(2, 0, 1)
        covariance = rotation @ covariance @ rotation.T
        for row in range(3):
            for column in range(3):
                estimate[f'cov_{start + row}_{start + column}'] = covariance[:, row, column]
    print(f'Bag reference: {bag_path}/sim/truth_state; NED, fixed global frame, no alignment fit')
    print(f'Using {estimate.size} estimates over {timestamps[-1] - timestamps[0]:.3f} seconds')
    return truth, estimate


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract a global ENU HoloOcean truth state from a bag.')
    parser.add_argument('bag_path', type=Path)
    parser.add_argument('start_time', type=float, help='Seconds from the first /imu/data header timestamp')
    arguments = parser.parse_args()
    try:
        print(json.dumps(extract_initial_state(arguments.bag_path, arguments.start_time), allow_nan=False))
    except (ValueError, OSError) as error:
        parser.exit(1, f'Error: {error}\n')
