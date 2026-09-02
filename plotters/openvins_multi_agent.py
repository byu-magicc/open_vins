#!/usr/bin/env python3
"""Plot recorded OpenVINS results for one or more agents.

Usage:
    python3 plotters/openvins_multi_agent.py results plots

The results directory may either be one agent directory, or a directory whose
immediate children are agent directories. Each agent directory must contain
``openvins.csv`` and ``groundtruth.csv`` as written with ``save_results:=true``.
The script writes the same three SVG plots and NPZ data archive as the former
ROS plotter into the output directory.
"""

import argparse
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


def load_csv(path, required_columns):
    if not path.is_file():
        raise ValueError(f"Missing input file: {path}")
    data = np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True, dtype=float))
    missing = sorted(set(required_columns) - set(data.dtype.names or ()))
    if missing:
        raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
    if data.size == 0:
        raise ValueError(f"{path} contains no result rows")
    return data


class DataPlotter:
    def __init__(self, results_directory, output_directory):
        self.output_directory = output_directory
        if (results_directory / 'openvins.csv').is_file() and (results_directory / 'groundtruth.csv').is_file():
            agent_directories = [results_directory]
        else:
            agent_directories = sorted(
                directory for directory in results_directory.iterdir()
                if directory.is_dir()
                and (directory / 'openvins.csv').is_file()
                and (directory / 'groundtruth.csv').is_file()
            )
        if not agent_directories:
            raise ValueError(f"No agent results found in {results_directory}")

        state_columns = (
            'timestamp', 'q_x', 'q_y', 'q_z', 'q_w', 'p_x', 'p_y', 'p_z'
        )
        estimator_columns = state_columns + tuple(f'cov_{index}_{index}' for index in range(6))

        self.time_data = {}
        self.global_truth_data = {}
        self.global_estimate_data = {}
        self.global_position_std = {}
        self.global_orientation_std = {}
        for directory in agent_directories:
            truth = load_csv(directory / 'groundtruth.csv', state_columns)
            estimate = load_csv(directory / 'openvins.csv', estimator_columns)
            if truth.size != estimate.size or not np.array_equal(truth['timestamp'], estimate['timestamp']):
                raise ValueError(f"Timestamps do not match in {directory}")

            key = directory.name
            self.time_data[key] = np.asarray(truth['timestamp'])
            self.global_truth_data[key] = np.column_stack([
                truth['p_x'], truth['p_y'], truth['p_z'],
                truth['q_x'], truth['q_y'], truth['q_z'], truth['q_w'],
            ])
            self.global_estimate_data[key] = np.column_stack([
                estimate['p_x'], estimate['p_y'], estimate['p_z'],
                estimate['q_x'], estimate['q_y'], estimate['q_z'], estimate['q_w'],
            ])
            self.global_position_std[key] = np.sqrt(np.column_stack([
                estimate['cov_3_3'], estimate['cov_4_4'], estimate['cov_5_5'],
            ]))
            self.global_orientation_std[key] = np.sqrt(np.column_stack([
                estimate['cov_0_0'], estimate['cov_1_1'], estimate['cov_2_2'],
            ]))


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

        # Get filenames for saving plots and data
        plots_directory = self.output_directory
        os.makedirs(plots_directory, exist_ok=True)
        counter = 0
        while os.path.exists(os.path.join(plots_directory, f'data_{counter}.npz')):
            counter += 1
        data_filename = os.path.join(plots_directory, f'data_{counter}.npz')
        global_xy_position_and_error_filename = os.path.join(
            plots_directory, f'global_xy_position_and_error_{counter}.svg')
        global_position_filename = os.path.join(
            plots_directory, f'global_position_{counter}.svg')
        global_error_filename = os.path.join(
            plots_directory, f'global_error_{counter}.svg')

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
        np.savez(data_filename, **data)


        ### Process data prior to plotting ###

        global_position_error = {}
        global_orientation_error = {}
        for key in self.global_truth_data.keys():
            # Convert quaternions to euler angles
            global_truth_orientation[key] = np.array([R.from_quat(q).as_euler('xyz', degrees=False) for q in global_truth_orientation[key]])
            global_estimate_orientation[key] = np.array([R.from_quat(q).as_euler('xyz', degrees=False) for q in global_estimate_orientation[key]])

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
                axs[0].plot(global_truth_position[key][:, 0], global_truth_position[key][:, 1], color='blue', label='Truth')
                axs[0].plot(global_estimate_position[key][:, 0], global_estimate_position[key][:, 1], color='red', label='Estimate')
            else:
                axs[0].plot(global_truth_position[key][:, 0], global_truth_position[key][:, 1], color='blue')
                axs[0].plot(global_estimate_position[key][:, 0], global_estimate_position[key][:, 1], color='red')

        blue_patch = mpatches.Patch(color='blue', label='Truth')
        red_patch = mpatches.Patch(color='red', label='Global Estimate')
        axs[0].set_xlabel('X Position (m)')
        axs[0].set_ylabel('Y Position (m)')
        axs[0].set_title('XY Position of Agents (Global Estimate)')
        axs[0].legend()
        axs[0].axis('equal')

        # Global norm position error data
        for key in self.global_truth_data.keys():
            axs[1].plot(time[key], np.linalg.norm(global_position_error[key], axis=1), label=key)
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Normed Position Error (m)')
        axs[1].set_title('Position Error of Agents (Global Estimate)')
        axs[1].legend()
        axs[1].grid()
        axs[1].set_ylim(bottom=0)

        plt.tight_layout()
        plt.savefig(global_xy_position_and_error_filename)


        ### Global Position Plots ###

        fig, axs = plt.subplots(6, len(global_truth_position.keys()), figsize=(16, 12))

        if len(global_truth_position.keys()) == 1:
            axs = np.expand_dims(axs, axis=1)

        column_idx = 0
        for key in self.global_truth_data.keys():

            # X Position
            axs[0, column_idx].plot(time[key], global_truth_position[key][:, 0], color='blue', label='Truth')
            axs[0, column_idx].plot(time[key], global_estimate_position[key][:, 0], color='red', label='Estimate')
            axs[0, column_idx].set_title(key)
            axs[0, column_idx].grid()

            # Y Position
            axs[1, column_idx].plot(time[key], global_truth_position[key][:, 1], color='blue')
            axs[1, column_idx].plot(time[key], global_estimate_position[key][:, 1], color='red')
            axs[1, column_idx].grid()

            # Z Position
            axs[2, column_idx].plot(time[key], global_truth_position[key][:, 2], color='blue')
            axs[2, column_idx].plot(time[key], global_estimate_position[key][:, 2], color='red')
            axs[2, column_idx].grid()

            # Roll
            axs[3, column_idx].plot(time[key], global_truth_orientation[key][:, 0], color='blue')
            axs[3, column_idx].plot(time[key], global_estimate_orientation[key][:, 0], color='red')
            axs[3, column_idx].grid()

            # Pitch
            axs[4, column_idx].plot(time[key], global_truth_orientation[key][:, 1], color='blue')
            axs[4, column_idx].plot(time[key], global_estimate_orientation[key][:, 1], color='red')
            axs[4, column_idx].grid()

            # Yaw
            axs[5, column_idx].plot(time[key], global_truth_orientation[key][:, 2], color='blue')
            axs[5, column_idx].plot(time[key], global_estimate_orientation[key][:, 2], color='red')
            axs[5, column_idx].set_xlabel('Time (s)')
            axs[5, column_idx].grid()

            # Add ylabels and legend
            if column_idx == 0:
                axs[0, column_idx].set_ylabel('East (m)')
                axs[1, column_idx].set_ylabel('North (m)')
                axs[2, column_idx].set_ylabel('Up (m)')
                axs[3, column_idx].set_ylabel('Roll (rad)')
                axs[4, column_idx].set_ylabel('Pitch (rad)')
                axs[5, column_idx].set_ylabel('Yaw (rad)')
                axs[0, column_idx].legend()

            column_idx += 1

        plt.tight_layout()
        plt.savefig(global_position_filename)


        ### Global Error Plots ###

        fig, axs = plt.subplots(6, len(global_truth_position.keys()), figsize=(16, 12))

        if len(global_truth_position.keys()) == 1:
            axs = np.expand_dims(axs, axis=1)

        column_idx = 0
        for key in self.global_truth_data.keys():

            # X Position
            axs[0, column_idx].plot(time[key], global_position_error[key][:, 0], color='red', label='Error')
            axs[0, column_idx].plot(time[key], global_position_std[key][:, 0]*2, color='blue', label='2 Sigma')
            axs[0, column_idx].plot(time[key], -global_position_std[key][:, 0]*2, color='blue')
            axs[0, column_idx].set_title(key)
            axs[0, column_idx].grid()

            # Y Position
            axs[1, column_idx].plot(time[key], global_position_error[key][:, 1], color='red')
            axs[1, column_idx].plot(time[key], global_position_std[key][:, 1]*2, color='blue')
            axs[1, column_idx].plot(time[key], -global_position_std[key][:, 1]*2, color='blue')
            axs[1, column_idx].grid()

            # Z Position
            axs[2, column_idx].plot(time[key], global_position_error[key][:, 2], color='red')
            axs[2, column_idx].plot(time[key], global_position_std[key][:, 2]*2, color='blue')
            axs[2, column_idx].plot(time[key], -global_position_std[key][:, 2]*2, color='blue')
            axs[2, column_idx].grid()

            # Pitch
            axs[3, column_idx].plot(time[key], global_orientation_error[key][:, 0], color='red')
            axs[3, column_idx].plot(time[key], global_orientation_std[key][:, 0]*2, color='blue')
            axs[3, column_idx].plot(time[key], -global_orientation_std[key][:, 0]*2, color='blue')
            axs[3, column_idx].grid()

            # Roll
            axs[4, column_idx].plot(time[key], global_orientation_error[key][:, 1], color='red')
            axs[4, column_idx].plot(time[key], global_orientation_std[key][:, 1]*2, color='blue')
            axs[4, column_idx].plot(time[key], -global_orientation_std[key][:, 1]*2, color='blue')
            axs[4, column_idx].grid()

            # Yaw
            axs[5, column_idx].plot(time[key], global_orientation_error[key][:, 2], color='red')
            axs[5, column_idx].plot(time[key], global_orientation_std[key][:, 2]*2, color='blue')
            axs[5, column_idx].plot(time[key], -global_orientation_std[key][:, 2]*2, color='blue')
            axs[5, column_idx].set_xlabel('Time (s)')
            axs[5, column_idx].grid()

            # Add ylabels to leftmost plots
            if column_idx == 0:
                axs[0, column_idx].set_ylabel('East Error (m)')
                axs[1, column_idx].set_ylabel('North Error (m)')
                axs[2, column_idx].set_ylabel('Up Error (m)')
                axs[3, column_idx].set_ylabel('Roll Error (rad)')
                axs[4, column_idx].set_ylabel('Pitch Error (rad)')
                axs[5, column_idx].set_ylabel('Yaw Error (rad)')
                axs[0, column_idx].legend()

            column_idx += 1

        plt.tight_layout()
        plt.savefig(global_error_filename)

        print(f'Plots generated and data saved in {plots_directory}')


def main():
    parser = argparse.ArgumentParser(description='Plot recorded OpenVINS results for one or more agents.')
    parser.add_argument('results_directory', type=Path)
    parser.add_argument('output_directory', type=Path)
    args = parser.parse_args()
    DataPlotter(args.results_directory, args.output_directory).plot_data()


if __name__ == '__main__':
    main()
