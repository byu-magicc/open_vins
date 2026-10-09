#!/usr/bin/env python3
"""Plot recorded OpenVINS results for one or more agents.

Usage:
    python3 plotters/plot_results.py results plots

The results directory may either be one agent directory, or a directory whose
immediate children are agent directories. Each agent directory must contain
``estimate.csv`` and ``groundtruth.csv`` as written with ``save_results:=true``.
With ``--truth-bag BAG_DIRECTORY``, one agent is compared against the bagged
HoloOcean ``/sim/truth_state`` instead, comparing global coordinates without
fitting position or yaw.
If present, the fleet-level ``ranges.csv`` supplies range-event annotations.
The script writes the same three SVG plots and NPZ data archive as the former
ROS plotter into the output directory.
"""

import argparse
import csv
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from holoocean_truth import load_bag_reference


def load_csv(path, required_columns):
    if not path.is_file():
        raise ValueError(f"Missing input file: {path}")
    data = np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True, dtype=float))
    missing = sorted(set(required_columns) - set(data.dtype.names or ()))
    if missing:
        raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
    if 'valid' in (data.dtype.names or ()):
        data = data[data['valid'] == 1]
    if data.size == 0:
        raise ValueError(f"{path} contains no valid result rows")
    if not np.all(np.isfinite(np.column_stack([data[column] for column in required_columns]))):
        raise ValueError(f'{path} contains non-finite state or covariance values')
    timestamps = np.asarray(data['timestamp'], dtype=float)
    if not np.all(np.isfinite(timestamps)) or np.unique(timestamps).size != timestamps.size:
        raise ValueError(f"{path} has invalid or duplicate timestamps")
    return data


class DataPlotter:
    def __init__(self, results_directory, output_directory, truth_bag, camera_imu_offset):
        self.output_directory = output_directory
        if truth_bag is None:
            enu_to_ned = R.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
            frd_to_flu = R.from_euler('x', np.pi)
        if (results_directory / 'estimate.csv').is_file() and (truth_bag is not None or (results_directory / 'groundtruth.csv').is_file()):
            agent_directories = [results_directory]
        else:
            agent_directories = sorted(
                directory for directory in results_directory.iterdir()
                if directory.is_dir()
                and (directory / 'estimate.csv').is_file()
                and (truth_bag is not None or (directory / 'groundtruth.csv').is_file())
            )
        if not agent_directories:
            raise ValueError(f"No agent results found in {results_directory}")

        if truth_bag is not None and len(agent_directories) != 1:
            raise ValueError('Bag plotting requires exactly one agent result directory')

        state_columns = (
            'timestamp', 'q_x', 'q_y', 'q_z', 'q_w', 'p_x', 'p_y', 'p_z', 'v_x', 'v_y', 'v_z'
        )
        estimator_columns = state_columns + tuple(f'cov_{index}_{index}' for index in range(9))

        self.time_data = {}
        self.global_truth_data = {}
        self.global_estimate_data = {}
        self.global_position_std = {}
        if truth_bag is not None:
            estimator_columns += tuple(
                f'cov_{row}_{column}' for start in (3, 6)
                for row in range(start, start + 3) for column in range(start, start + 3) if row != column
            )

        self.global_orientation_std = {}
        self.global_truth_velocity = {}
        self.global_estimate_velocity = {}
        self.global_velocity_std = {}
        for directory in agent_directories:
            estimate = load_csv(directory / 'estimate.csv', estimator_columns)
            if truth_bag is not None:
                truth, estimate = load_bag_reference(truth_bag, estimate, camera_imu_offset)
                timestamps = estimate['timestamp']
            else:
                truth = load_csv(directory / 'groundtruth.csv', state_columns)
                timestamps, truth_indices, estimate_indices = np.intersect1d(
                    truth['timestamp'], estimate['timestamp'], return_indices=True,
                )
                if timestamps.size < 2:
                    raise ValueError(f"Fewer than two timestamps match in {directory}")
                dropped = truth.size + estimate.size - 2 * timestamps.size
                truth = truth[truth_indices]
                estimate = estimate[estimate_indices]
                if dropped:
                    print(f"Dropped {dropped} unmatched rows in {directory}")

            key = directory.name
            self.time_data[key] = timestamps
            self.global_truth_data[key] = np.column_stack([
                truth['p_x'], truth['p_y'], truth['p_z'],
                truth['q_x'], truth['q_y'], truth['q_z'], truth['q_w'],
            ])
            self.global_estimate_data[key] = np.column_stack([
                estimate['p_x'], estimate['p_y'], estimate['p_z'],
                estimate['q_x'], estimate['q_y'], estimate['q_z'], estimate['q_w'],
            ])
            self.global_truth_velocity[key] = np.column_stack([truth[f'v_{axis}'] for axis in ('x', 'y', 'z')])
            self.global_estimate_velocity[key] = np.column_stack([estimate[f'v_{axis}'] for axis in ('x', 'y', 'z')])
            std = np.sqrt(np.maximum(0, np.column_stack([estimate[f'cov_{index}_{index}'] for index in range(9)])))
            if truth_bag is None:
                # Simulation CSVs use a Z-up world and IMU body; plot in NED/FRD.
                for state in (self.global_truth_data[key], self.global_estimate_data[key]):
                    state[:, :3] = enu_to_ned.apply(state[:, :3])
                    state[:, 3:] = (enu_to_ned * R.from_quat(state[:, 3:]) * frd_to_flu).as_quat()
                self.global_truth_velocity[key] = enu_to_ned.apply(self.global_truth_velocity[key])
                self.global_estimate_velocity[key] = enu_to_ned.apply(self.global_estimate_velocity[key])
                std = std[:, [0, 1, 2, 4, 3, 5, 7, 6, 8]]
            self.global_orientation_std[key] = std[:, :3]
            self.global_position_std[key] = std[:, 3:6]
            self.global_velocity_std[key] = std[:, 6:9]

        self.range_measurements = []
        ranges_path = results_directory / 'ranges.csv'
        if not ranges_path.is_file() and len(agent_directories) == 1:
            ranges_path = results_directory.parent / 'ranges.csv'
        if ranges_path.is_file():
            with ranges_path.open(newline='') as ranges_file:
                ranges = csv.DictReader(ranges_file)
                required_columns = {'owner', 'neighbor', 'owner_timestamp', 'neighbor_timestamp'}
                missing = sorted(required_columns - set(ranges.fieldnames or ()))
                if missing:
                    raise ValueError(f"{ranges_path} is missing columns: {', '.join(missing)}")
                for measurement in ranges:
                    owner = measurement['owner']
                    neighbor = measurement['neighbor']
                    if owner not in self.time_data and neighbor not in self.time_data:
                        continue
                    owner_timestamp = float(measurement['owner_timestamp'])
                    neighbor_timestamp = float(measurement['neighbor_timestamp'])
                    if not np.isfinite(owner_timestamp) or not np.isfinite(neighbor_timestamp):
                        raise ValueError(f'{ranges_path} contains a non-finite timestamp')
                    self.range_measurements.append((owner, neighbor, owner_timestamp, neighbor_timestamp))


    def plot_data(self):
        # Extract data into usable format
        time = {}
        global_truth_position = {}
        global_truth_orientation = {}
        global_estimate_position = {}
        global_position_std = {}
        global_estimate_orientation = {}
        global_orientation_std = {}
        for key in self.global_truth_data.keys():
            time[key] = self.time_data[key] - self.time_data[key][0]
            global_truth_position[key] = self.global_truth_data[key][:, :3]
            global_truth_orientation[key] = self.global_truth_data[key][:, 3:]
            global_estimate_position[key] = self.global_estimate_data[key][:, :3]
            global_position_std[key] = self.global_position_std[key]
            global_estimate_orientation[key] = self.global_estimate_data[key][:, 3:]
            global_orientation_std[key] = self.global_orientation_std[key]

        range_times = {key: [] for key in time}
        fleet_range_times = []
        range_segments = []
        for owner, neighbor, owner_timestamp, neighbor_timestamp in self.range_measurements:
            if owner in time:
                owner_time = owner_timestamp - self.time_data[owner][0]
                if time[owner][0] <= owner_time <= time[owner][-1]:
                    range_times[owner].append(owner_time)
                    fleet_range_times.append(owner_time)
            if neighbor in time:
                neighbor_time = neighbor_timestamp - self.time_data[neighbor][0]
                if time[neighbor][0] <= neighbor_time <= time[neighbor][-1]:
                    range_times[neighbor].append(neighbor_time)
                    if owner not in time:
                        fleet_range_times.append(neighbor_time)
            if owner in time and neighbor in time:
                owner_timestamps = self.time_data[owner]
                neighbor_timestamps = self.time_data[neighbor]
                if (owner_timestamps[0] <= owner_timestamp <= owner_timestamps[-1]
                        and neighbor_timestamps[0] <= neighbor_timestamp <= neighbor_timestamps[-1]):
                    owner_position = np.array([
                        np.interp(owner_timestamp, owner_timestamps, global_estimate_position[owner][:, axis])
                        for axis in range(2)
                    ])
                    neighbor_position = np.array([
                        np.interp(neighbor_timestamp, neighbor_timestamps, global_estimate_position[neighbor][:, axis])
                        for axis in range(2)
                    ])
                    range_segments.append((owner_position, neighbor_position))
        for key in range_times:
            range_times[key].sort()
        fleet_range_times.sort()

        # Get filenames for saving plots and data
        plots_directory = self.output_directory
        os.makedirs(plots_directory, exist_ok=True)
        data_filename = os.path.join(plots_directory, 'data.npz')
        global_xy_position_and_error_filename = os.path.join(
            plots_directory, 'global_xy_position_and_error.svg')
        global_position_filename = os.path.join(
            plots_directory, 'global_position.svg')
        global_error_filename = os.path.join(
            plots_directory, 'global_error.svg')

        # Save all data to a .npz file
        data = {}
        for key in self.global_truth_data.keys():
            data[f'{key}_time'] = time[key]
            data[f'{key}_global_truth_position'] = global_truth_position[key]
            data[f'{key}_global_truth_orientation'] = global_truth_orientation[key]
            data[f'{key}_global_estimate_position'] = global_estimate_position[key]
            data[f'{key}_global_position_std'] = global_position_std[key]
            data[f'{key}_global_estimate_orientation'] = global_estimate_orientation[key]
            data[f'{key}_global_orientation_std'] = global_orientation_std[key]
            data[f'{key}_global_truth_velocity'] = self.global_truth_velocity[key]
            data[f'{key}_global_estimate_velocity'] = self.global_estimate_velocity[key]
            data[f'{key}_global_velocity_std'] = self.global_velocity_std[key]
        np.savez(data_filename, **data)


        ### Process data prior to plotting ###

        global_position_error = {}
        global_orientation_error = {}
        for key in self.global_truth_data.keys():
            # Convert quaternions to euler angles
            global_truth_orientation[key] = R.from_quat(global_truth_orientation[key]).as_euler('xyz')
            global_estimate_orientation[key] = R.from_quat(global_estimate_orientation[key]).as_euler('xyz')

            # Calculate errors between truth and estimates
            global_position_error[key] = global_truth_position[key] - global_estimate_position[key]
            global_orientation_error[key] = global_truth_orientation[key] - global_estimate_orientation[key]
            global_orientation_error[key] = (global_orientation_error[key] + np.pi) % (2 * np.pi) - np.pi


        ### XY Global Position and Error ###

        # Create 2x1 plot
        fig, axs = plt.subplots(2, figsize=(16, 12))

        # Global xy position data
        for key in self.global_truth_data.keys():
            if key == next(iter(self.global_truth_data)):
                axs[0].plot(global_truth_position[key][:, 1], global_truth_position[key][:, 0], color='blue', label='Truth')
                axs[0].plot(global_estimate_position[key][:, 1], global_estimate_position[key][:, 0], color='red', label='Estimate')
            else:
                axs[0].plot(global_truth_position[key][:, 1], global_truth_position[key][:, 0], color='blue')
                axs[0].plot(global_estimate_position[key][:, 1], global_estimate_position[key][:, 0], color='red')

        for segment_index, (owner_position, neighbor_position) in enumerate(range_segments):
            label = 'Range measurement' if segment_index == 0 else None
            axs[0].plot(
                [owner_position[1], neighbor_position[1]],
                [owner_position[0], neighbor_position[0]],
                color='grey', linestyle='--', linewidth=0.8, alpha=0.7, label=label,
            )
        axs[0].set_xlabel('East Position (m)')
        axs[0].set_ylabel('North Position (m)')
        axs[0].set_title('XY Position of Agents (Global Estimate)')
        axs[0].legend()
        axs[0].axis('equal')

        # Global norm position error data
        for key in self.global_truth_data.keys():
            axs[1].plot(time[key], np.linalg.norm(global_position_error[key], axis=1), label=key)
        for event_index, range_time in enumerate(fleet_range_times):
            label = 'Range measurement' if event_index == 0 else None
            axs[1].axvline(range_time, color='grey', linestyle='--', linewidth=0.8, alpha=0.7, label=label)
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Normed Position Error (m)')
        axs[1].set_title('Position Error of Agents (Global Estimate)')
        axs[1].legend()
        axs[1].set_ylim(bottom=0)

        plt.tight_layout()
        plt.savefig(global_xy_position_and_error_filename)
        plt.close(fig)


        ### Individual State and Error Plots ###

        axes = ('North', 'East', 'Down')
        labels = tuple(f'{axis} (m)' for axis in axes) + ('Roll (rad)', 'Pitch (rad)', 'Yaw (rad)') + tuple(
            f'{axis} Velocity (m/s)' for axis in axes
        )
        error_labels = tuple(f'{axis} Error (m)' for axis in axes) + (
            'Roll Error (rad)', 'Pitch Error (rad)', 'Yaw Error (rad)'
        ) + tuple(f'{axis} Velocity Error (m/s)' for axis in axes)
        for errors, filename in ((False, global_position_filename), (True, global_error_filename)):
            fig, axs = plt.subplots(9, len(time), figsize=(16, 18), squeeze=False)
            for column, key in enumerate(time):
                truth_states = np.column_stack([
                    global_truth_position[key], global_truth_orientation[key], self.global_truth_velocity[key],
                ])
                estimate_states = np.column_stack([
                    global_estimate_position[key], global_estimate_orientation[key], self.global_estimate_velocity[key],
                ])
                state_errors = np.column_stack([
                    global_position_error[key], global_orientation_error[key],
                    self.global_truth_velocity[key] - self.global_estimate_velocity[key],
                ])
                std = np.column_stack([global_position_std[key], global_orientation_std[key], self.global_velocity_std[key]])
                for row in range(9):
                    ax = axs[row, column]
                    if errors:
                        ax.plot(time[key], state_errors[:, row], color='red', label='Error')
                        ax.plot(time[key], 2 * std[:, row], color='blue', label='2 Sigma')
                        ax.plot(time[key], -2 * std[:, row], color='blue')
                    else:
                        ax.plot(time[key], truth_states[:, row], color='blue', label='Truth')
                        ax.plot(time[key], estimate_states[:, row], color='red', label='Estimate')
                    for event_index, range_time in enumerate(range_times[key]):
                        label = 'Range measurement' if row == 0 and event_index == 0 else None
                        ax.axvline(range_time, color='grey', linestyle='--', linewidth=0.8, alpha=0.7, label=label)
                    if column == 0:
                        ax.set_ylabel((error_labels if errors else labels)[row])
                    if row == 0:
                        ax.set_title(key)
                        if column == 0:
                            ax.legend()
                    if row == 8:
                        ax.set_xlabel('Time (s)')
            fig.tight_layout()
            fig.savefig(filename)
            plt.close(fig)

        print(f'Plots generated and data saved in {plots_directory}')


def main():
    parser = argparse.ArgumentParser(description='Plot recorded OpenVINS results for one or more agents.')
    parser.add_argument('results_directory', type=Path)
    parser.add_argument('output_directory', type=Path)
    parser.add_argument('--truth-bag', type=Path, help='HoloOcean bag directory containing /sim/truth_state')
    parser.add_argument('--camera-imu-offset', type=float, default=0.0, help='IMU timestamp minus camera timestamp (seconds)')
    args = parser.parse_args()
    try:
        DataPlotter(args.results_directory, args.output_directory, args.truth_bag, args.camera_imu_offset).plot_data()
    except (ValueError, OSError) as error:
        parser.exit(1, f'Error: {error}\n')


if __name__ == '__main__':
    main()
