#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


class DataPlotterNode(Node):
    def __init__(self):
        super().__init__('data_plotter_node')

        # Declare parameters
        self.declare_parameter('agent_namespaces', ['ov_msckf'])

        # Get parameters
        agent_namespaces = self.get_parameter('agent_namespaces').get_parameter_value().string_array_value

        # Initialize data storage
        self.time_data = {}
        for namespace in agent_namespaces:
            self.time_data[namespace] = []
        self.truth_data = {}
        for namespace in agent_namespaces:
            self.truth_data[namespace] = []
        self.estimate_data = {}
        for namespace in agent_namespaces:
            self.estimate_data[namespace] = []

        # Create subscribers
        self.time_subs = {}
        self.truth_subs = {}
        self.estimate_subs = {}
        for namespace in agent_namespaces:
            truth_topic_name = '/' + namespace + '/posegt'
            self.truth_subs[namespace] = self.create_subscription(
                PoseStamped,
                truth_topic_name,
                lambda msg, n=namespace: self.truth_data[n].append(msg.pose),
                1000
            )
            estimate_topic_name = '/' + namespace + '/poseimu'
            self.time_subs[namespace] = self.create_subscription(
                PoseWithCovarianceStamped,
                estimate_topic_name,
                lambda msg, n=namespace: self.time_data[n].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
                1000
            )
            self.estimate_subs[namespace] = self.create_subscription(
                PoseWithCovarianceStamped,
                estimate_topic_name,
                lambda msg, n=namespace: self.estimate_data[n].append(msg.pose),
                1000
            )


    def plot_data(self):
        # Extract data into usable format
        time = {}
        truth_position = {}
        truth_orientation = {}
        estimate_position = {}
        estimate_position_std = {}
        estimate_orientation = {}
        estimate_orientation_std = {}
        for key in self.truth_data.keys():
            time[key] = []
            truth_position[key] = []
            truth_orientation[key] = []
            estimate_position[key] = []
            estimate_position_std[key] = []
            estimate_orientation[key] = []
            estimate_orientation_std[key] = []

            time_data = self.time_data[key]
            truth_data = self.truth_data[key]
            estimate_data = self.estimate_data[key]
            for i in range(len(truth_data)):
                time[key].append(time_data[i])
                truth_position[key].append([
                    truth_data[i].position.x,
                    truth_data[i].position.y,
                    truth_data[i].position.z
                ])
                truth_orientation[key].append([
                    truth_data[i].orientation.x,
                    truth_data[i].orientation.y,
                    truth_data[i].orientation.z,
                    truth_data[i].orientation.w
                ])
                estimate_position[key].append([
                    estimate_data[i].pose.position.x,
                    estimate_data[i].pose.position.y,
                    estimate_data[i].pose.position.z
                ])
                estimate_position_std[key].append([
                    np.sqrt(estimate_data[i].covariance[0]),
                    np.sqrt(estimate_data[i].covariance[7]),
                    np.sqrt(estimate_data[i].covariance[14])
                ])
                estimate_orientation[key].append([
                    estimate_data[i].pose.orientation.x,
                    estimate_data[i].pose.orientation.y,
                    estimate_data[i].pose.orientation.z,
                    estimate_data[i].pose.orientation.w
                ])
                estimate_orientation_std[key].append([
                    np.sqrt(estimate_data[i].covariance[21]),
                    np.sqrt(estimate_data[i].covariance[28]),
                    np.sqrt(estimate_data[i].covariance[35])
                ])
            time[key] = np.array(time[key]) - time[key][0]
            truth_position[key] = np.array(truth_position[key])
            truth_orientation[key] = np.array(truth_orientation[key])
            estimate_position[key] = np.array(estimate_position[key])
            estimate_position_std[key] = np.array(estimate_position_std[key])
            estimate_orientation[key] = np.array(estimate_orientation[key])
            estimate_orientation_std[key] = np.array(estimate_orientation_std[key])

        # Get filenames for saving plots and data
        counter = 0
        while os.path.exists(f'data_{counter}.npz'):
            counter += 1
        data_filename = f'data_{counter}.npz'
        xy_position_and_error_filename = f'xy_position_and_error_{counter}.svg'
        position_filename = f'position_{counter}.svg'
        error_filename = f'error_{counter}.svg'

        # Save all data to a .npz file
        data = {}
        for key in self.truth_data.keys():
            data[f'{key}_time'] = time[key]
            data[f'{key}_truth_position'] = truth_position[key]
            data[f'{key}_truth_orientation'] = truth_orientation[key]
            data[f'{key}_estimate_position'] = estimate_position[key]
            data[f'{key}_estimate_std'] = estimate_position_std[key]
            data[f'{key}_estimate_orientation'] = estimate_orientation[key]
            data[f'{key}_orientation_std'] = estimate_orientation_std[key]
        np.savez(data_filename, **data)


        ### Process data prior to plotting ###

        position_error = {}
        orientation_error = {}
        for key in self.truth_data.keys():
            # Convert quaternions to euler angles
            truth_orientation[key] = np.array([R.from_quat(q).as_euler('xyz', degrees=False) for q in truth_orientation[key]])
            estimate_orientation[key] = np.array([R.from_quat(q).as_euler('xyz', degrees=False) for q in estimate_orientation[key]])

            # Calculate errors between truth and estimates
            position_error[key] = truth_position[key] - estimate_position[key]
            orientation_error[key] = truth_orientation[key] - estimate_orientation[key]
            orientation_error[key] = (orientation_error[key] + np.pi) % (2 * np.pi) - np.pi


        ### XY Position and Error ###

        # Create 2x1 plot
        fig, axs = plt.subplots(2, figsize=(16, 12))

        # xy position data
        for key in self.truth_data.keys():
            if key == next(iter(self.truth_data)):
                axs[0].plot(truth_position[key][:, 0], truth_position[key][:, 1], color='blue', label='Truth')
                axs[0].plot(estimate_position[key][:, 0], estimate_position[key][:, 1], color='red', label='Estimate')
            else:
                axs[0].plot(truth_position[key][:, 0], truth_position[key][:, 1], color='blue')
                axs[0].plot(estimate_position[key][:, 0], estimate_position[key][:, 1], color='red')

        blue_patch = mpatches.Patch(color='blue', label='Truth')
        red_patch = mpatches.Patch(color='red', label='Estimate')
        axs[0].set_xlabel('X Position (m)')
        axs[0].set_ylabel('Y Position (m)')
        axs[0].set_title('XY Position of Agents (Estimate)')
        axs[0].legend()
        axs[0].axis('equal')

        # norm position error data
        for key in self.truth_data.keys():
            axs[1].plot(time[key], np.linalg.norm(position_error[key], axis=1), label=key)
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Normed Position Error (m)')
        axs[1].set_title('Position Error of Agents (Estimate)')
        axs[1].legend()
        axs[1].grid()
        axs[1].set_ylim(bottom=0)

        plt.tight_layout()
        plt.savefig(xy_position_and_error_filename)


        ### Position Plots ###

        fig, axs = plt.subplots(6, len(truth_position.keys()), figsize=(16, 12))

        if len(truth_position.keys()) == 1:
            axs = np.expand_dims(axs, axis=1)

        column_idx = 0
        for key in self.truth_data.keys():

            # X Position
            axs[0, column_idx].plot(time[key], truth_position[key][:, 0], color='blue', label='Truth')
            axs[0, column_idx].plot(time[key], estimate_position[key][:, 0], color='red', label='Estimate')
            axs[0, column_idx].set_title(key)
            axs[0, column_idx].grid()

            # Y Position
            axs[1, column_idx].plot(time[key], truth_position[key][:, 1], color='blue')
            axs[1, column_idx].plot(time[key], estimate_position[key][:, 1], color='red')
            axs[1, column_idx].grid()

            # Z Position
            axs[2, column_idx].plot(time[key], truth_position[key][:, 2], color='blue')
            axs[2, column_idx].plot(time[key], estimate_position[key][:, 2], color='red')
            axs[2, column_idx].grid()

            # Roll
            axs[3, column_idx].plot(time[key], truth_orientation[key][:, 0], color='blue')
            axs[3, column_idx].plot(time[key], estimate_orientation[key][:, 0], color='red')
            axs[3, column_idx].grid()

            # Pitch
            axs[4, column_idx].plot(time[key], truth_orientation[key][:, 1], color='blue')
            axs[4, column_idx].plot(time[key], estimate_orientation[key][:, 1], color='red')
            axs[4, column_idx].grid()

            # Yaw
            axs[5, column_idx].plot(time[key], truth_orientation[key][:, 2], color='blue')
            axs[5, column_idx].plot(time[key], estimate_orientation[key][:, 2], color='red')
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
        plt.savefig(position_filename)


        ### Error Plots ###

        fig, axs = plt.subplots(6, len(truth_position.keys()), figsize=(16, 12))

        if len(truth_position.keys()) == 1:
            axs = np.expand_dims(axs, axis=1)

        column_idx = 0
        for key in self.truth_data.keys():

            # X Position
            axs[0, column_idx].plot(time[key], position_error[key][:, 0], color='red', label='Error')
            axs[0, column_idx].plot(time[key], estimate_position_std[key][:, 0]*2, color='blue', label='2 Sigma')
            axs[0, column_idx].plot(time[key], -estimate_position_std[key][:, 0]*2, color='blue')
            axs[0, column_idx].set_title(key)
            axs[0, column_idx].grid()

            # Y Position
            axs[1, column_idx].plot(time[key], position_error[key][:, 1], color='red')
            axs[1, column_idx].plot(time[key], estimate_position_std[key][:, 1]*2, color='blue')
            axs[1, column_idx].plot(time[key], -estimate_position_std[key][:, 1]*2, color='blue')
            axs[1, column_idx].grid()

            # Z Position
            axs[2, column_idx].plot(time[key], position_error[key][:, 2], color='red')
            axs[2, column_idx].plot(time[key], estimate_position_std[key][:, 2]*2, color='blue')
            axs[2, column_idx].plot(time[key], -estimate_position_std[key][:, 2]*2, color='blue')
            axs[2, column_idx].grid()

            # Pitch
            axs[3, column_idx].plot(time[key], orientation_error[key][:, 0], color='red')
            axs[3, column_idx].plot(time[key], estimate_orientation_std[key][:, 0]*2, color='blue')
            axs[3, column_idx].plot(time[key], -estimate_orientation_std[key][:, 0]*2, color='blue')
            axs[3, column_idx].grid()

            # Roll
            axs[4, column_idx].plot(time[key], orientation_error[key][:, 1], color='red')
            axs[4, column_idx].plot(time[key], estimate_orientation_std[key][:, 1]*2, color='blue')
            axs[4, column_idx].plot(time[key], -estimate_orientation_std[key][:, 1]*2, color='blue')
            axs[4, column_idx].grid()

            # Yaw
            axs[5, column_idx].plot(time[key], orientation_error[key][:, 2], color='red')
            axs[5, column_idx].plot(time[key], estimate_orientation_std[key][:, 2]*2, color='blue')
            axs[5, column_idx].plot(time[key], -estimate_orientation_std[key][:, 2]*2, color='blue')
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
        plt.savefig(error_filename)

        self.get_logger().info('Plots generated and data saved successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = DataPlotterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Generating plots and saving data...')
    node.plot_data()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
