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
        self.global_truth_data = {}
        for namespace in agent_namespaces:
            self.global_truth_data[namespace] = []
        self.global_estimate_data = {}
        for namespace in agent_namespaces:
            self.global_estimate_data[namespace] = []

        # Create subscribers
        self.time_subs = {}
        self.global_truth_subs = {}
        self.global_estimate_subs = {}
        for namespace in agent_namespaces:
            truth_topic_name = '/' + namespace + '/posegt'
            self.time_subs[namespace] = self.create_subscription(
                PoseStamped,
                truth_topic_name,
                lambda msg, n=namespace: self.time_data[n].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
                1000
            )
            self.global_truth_subs[namespace] = self.create_subscription(
                PoseStamped,
                truth_topic_name,
                lambda msg, n=namespace: self.global_truth_data[n].append(msg.pose),
                1000
            )
            global_estimate_topic_name = '/' + namespace + '/poseimu'
            self.global_estimate_subs[namespace] = self.create_subscription(
                PoseWithCovarianceStamped,
                global_estimate_topic_name,
                lambda msg, n=namespace: self.global_estimate_data[n].append(msg.pose),
                1000
            )


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
            time[key] = []
            global_truth_position[key] = []
            global_truth_orientation[key] = []
            global_estimate_position[key] = []
            global_position_std[key] = []
            global_estimate_orientation[key] = []
            global_orientation_std[key] = []

            time_data = self.time_data[key]
            global_truth_data = self.global_truth_data[key]
            global_estimate_data = self.global_estimate_data[key]
            for i in range(len(global_truth_data)):
                time[key].append(time_data[i])
                global_truth_position[key].append([
                    global_truth_data[i].position.x,
                    global_truth_data[i].position.y,
                    global_truth_data[i].position.z
                ])
                global_truth_orientation[key].append([
                    global_truth_data[i].orientation.x,
                    global_truth_data[i].orientation.y,
                    global_truth_data[i].orientation.z,
                    global_truth_data[i].orientation.w
                ])
                global_estimate_position[key].append([
                    global_estimate_data[i].pose.position.x,
                    global_estimate_data[i].pose.position.y,
                    global_estimate_data[i].pose.position.z
                ])
                global_position_std[key].append([
                    np.sqrt(global_estimate_data[i].covariance[0]),
                    np.sqrt(global_estimate_data[i].covariance[7]),
                    np.sqrt(global_estimate_data[i].covariance[14])
                ])
                global_estimate_orientation[key].append([
                    global_estimate_data[i].pose.orientation.x,
                    global_estimate_data[i].pose.orientation.y,
                    global_estimate_data[i].pose.orientation.z,
                    global_estimate_data[i].pose.orientation.w
                ])
                global_orientation_std[key].append([
                    np.sqrt(global_estimate_data[i].covariance[21]),
                    np.sqrt(global_estimate_data[i].covariance[28]),
                    np.sqrt(global_estimate_data[i].covariance[35])
                ])
            time[key] = np.array(time[key]) - time[key][0]
            global_truth_position[key] = np.array(global_truth_position[key])
            global_truth_orientation[key] = np.array(global_truth_orientation[key])
            global_estimate_position[key] = np.array(global_estimate_position[key])
            global_position_std[key] = np.array(global_position_std[key])
            global_estimate_orientation[key] = np.array(global_estimate_orientation[key])
            global_orientation_std[key] = np.array(global_orientation_std[key])

        # Get filenames for saving plots and data
        counter = 0
        while os.path.exists(f'data_{counter}.npz'):
            counter += 1
        data_filename = f'data_{counter}.npz'
        global_xy_position_and_error_filename = f'global_xy_position_and_error_{counter}.svg'
        global_position_filename = f'global_position_{counter}.svg'
        global_error_filename = f'global_error_{counter}.svg'

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
