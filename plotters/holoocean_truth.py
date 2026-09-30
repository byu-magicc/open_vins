"""Load and align HoloOcean simulator truth for the shared result plotter."""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load_bag_reference(bag_path, estimate):
    try:
        from rosbags.highlevel import AnyReader, AnyReaderError
    except ImportError as error:
        raise ValueError('Bag plotting requires rosbags; see the HoloOcean setup in ReadMe.md') from error

    # SimState uses NED world coordinates and FRD body-to-world Hamilton quaternions.
    enu_to_ned = Rotation.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    rows = []
    try:
        with AnyReader([bag_path]) as reader:
            connections = [connection for connection in reader.connections if connection.topic == '/sim/truth_state']
            if not connections:
                raise ValueError(f'{bag_path} has no /sim/truth_state topic')
            for connection, _, raw in reader.messages(connections=connections):
                message = reader.deserialize(raw, connection.msgtype)
                timestamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
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
    if len(rows) < 2:
        raise ValueError(f'{bag_path} contains fewer than two truth samples')
    rows = np.asarray(rows)
    rows = rows[np.argsort(rows[:, 0])]
    if not np.all(np.isfinite(rows)) or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError(f'{bag_path} has non-finite truth data or duplicate header timestamps')

    body_to_ned = Rotation.from_quat(rows[:, 4:8])
    truth_positions = rows[:, 1:4]
    truth_velocities = body_to_ned.apply(rows[:, 8:11])
    overlap = (estimate['timestamp'] >= rows[0, 0]) & (estimate['timestamp'] <= rows[-1, 0])
    estimate = np.sort(estimate[overlap], order='timestamp').copy()
    if estimate.size < 2:
        raise ValueError(f'Fewer than two estimate timestamps overlap truth in {bag_path}')
    timestamps = estimate['timestamp']
    truth = np.empty(estimate.size, dtype=[(name, float) for name in (
        'timestamp', 'p_x', 'p_y', 'p_z', 'q_x', 'q_y', 'q_z', 'q_w', 'v_x', 'v_y', 'v_z',
    )])
    truth['timestamp'] = timestamps
    for axis, name in enumerate(('x', 'y', 'z')):
        truth[f'p_{name}'] = np.interp(timestamps, rows[:, 0], truth_positions[:, axis])
        truth[f'v_{name}'] = np.interp(timestamps, rows[:, 0], truth_velocities[:, axis])
    interpolated_rotations = Slerp(rows[:, 0], body_to_ned)(timestamps)
    truth_quaternions = interpolated_rotations.as_quat()
    for axis, name in enumerate(('x', 'y', 'z', 'w')):
        truth[f'q_{name}'] = truth_quaternions[:, axis]

    # OpenVINS stores JPL global-to-IMU quaternions. Interpreted as Hamilton
    # quaternions by SciPy, the same coefficients represent IMU-to-global.
    estimate_rotations = enu_to_ned * Rotation.from_quat(np.column_stack([
        estimate[f'q_{name}'] for name in ('x', 'y', 'z', 'w')
    ]))
    yaw = interpolated_rotations[0].as_euler('xyz')[2] - estimate_rotations[0].as_euler('xyz')[2]
    alignment = Rotation.from_euler('z', yaw)
    world_rotation = alignment * enu_to_ned
    positions = world_rotation.apply(np.column_stack([estimate[f'p_{name}'] for name in ('x', 'y', 'z')]))
    translation = np.array([truth[f'p_{name}'][0] for name in ('x', 'y', 'z')]) - positions[0]
    positions += translation
    velocities = world_rotation.apply(np.column_stack([estimate[f'v_{name}'] for name in ('x', 'y', 'z')]))
    quaternions = (alignment * estimate_rotations).as_quat()
    for axis, name in enumerate(('x', 'y', 'z')):
        estimate[f'p_{name}'] = positions[:, axis]
        estimate[f'v_{name}'] = velocities[:, axis]
    for axis, name in enumerate(('x', 'y', 'z', 'w')):
        estimate[f'q_{name}'] = quaternions[:, axis]
    # Position and velocity uncertainties are world-frame quantities. Attitude
    # covariance is a body-frame perturbation and is unchanged by this alignment.
    rotation = world_rotation.as_matrix()
    for start in (3, 6):
        covariance = np.array([
            [estimate[f'cov_{row}_{column}'] for column in range(start, start + 3)]
            for row in range(start, start + 3)
        ]).transpose(2, 0, 1)
        covariance = rotation @ covariance @ rotation.T
        for row in range(3):
            for column in range(3):
                estimate[f'cov_{start + row}_{start + column}'] = covariance[:, row, column]
    print(f'Bag reference: {bag_path}/sim/truth_state; NED, initial position/yaw alignment, no scale fit')
    print(f'Using {estimate.size} estimates over {timestamps[-1] - timestamps[0]:.3f} seconds')
    return truth, estimate
