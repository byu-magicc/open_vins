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
        self.global_estimate_data = {}
        for namespace in agent_namespaces:
            self.global_estimate_data[namespace] = []
        self.keyframe_estimate_data = {}
        for namespace in agent_namespaces:
            self.keyframe_estimate_data[namespace] = []
        self.keyframe_data = {}
        for namespace in agent_namespaces:
            self.keyframe_data[namespace] = []

        # Create subscribers
        self.time_subs = {}
        self.truth_subs = {}
        self.global_estimate_subs = {}
        self.keyframe_estimate_subs = {}
        self.keyframe_subs = {}
        for namespace in agent_namespaces:
            truth_topic_name = '/' + namespace + '/posegt'
            self.time_subs[namespace] = self.create_subscription(
                PoseStamped,
                truth_topic_name,
                lambda msg, n=namespace: self.time_data[n].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
                1000
            )
            self.truth_subs[namespace] = self.create_subscription(
                PoseStamped,
                truth_topic_name,
                lambda msg, n=namespace: self.truth_data[n].append(msg.pose),
                1000
            )
            global_estimate_topic_name = '/' + namespace + '/poseimu'
            self.global_estimate_subs[namespace] = self.create_subscription(
                PoseWithCovarianceStamped,
                global_estimate_topic_name,
                lambda msg, n=namespace: self.global_estimate_data[n].append(msg.pose),
                1000
            )
            keyframe_estimate_topic_name = '/' + namespace + '/poseimu_keyframe'
            self.keyframe_estimate_subs[namespace] = self.create_subscription(
                PoseWithCovarianceStamped,
                keyframe_estimate_topic_name,
                lambda msg, n=namespace: self.keyframe_estimate_data[n].append(msg.pose),
                1000
            )
            keyframe_topic_name = '/' + namespace + '/keyframe'
            self.keyframe_subs[namespace] = self.create_subscription(
                PoseStamped,
                keyframe_topic_name,
                lambda msg, n=namespace: self.keyframe_data[n].append(msg.pose),
                1000
            )


    def plot_data(self):
        # Extract data into usable format
        time = {}
        truth_position = {}
        truth_orientation = {}
        global_estimate_position = {}
        global_estimate_position_std = {}
        global_estimate_orientation = {}
        global_estimate_orientation_std = {}
        keyframe_estimate_position = {}
        keyframe_estimate_position_std = {}
        keyframe_estimate_orientation = {}
        keyframe_estimate_orientation_std = {}
        keyframe_position = {}
        keyframe_orientation = {}
        for key in self.truth_data.keys():
            time[key] = []
            truth_position[key] = []
            truth_orientation[key] = []
            global_estimate_position[key] = []
            global_estimate_position_std[key] = []
            global_estimate_orientation[key] = []
            global_estimate_orientation_std[key] = []
            keyframe_estimate_position[key] = []
            keyframe_estimate_position_std[key] = []
            keyframe_estimate_orientation[key] = []
            keyframe_estimate_orientation_std[key] = []
            keyframe_position[key] = []
            keyframe_orientation[key] = []

            time_data = self.time_data[key]
            truth_data = self.truth_data[key]
            global_estimate_data = self.global_estimate_data[key]
            keyframe_estimate_data = self.keyframe_estimate_data[key]
            keyframe_data = self.keyframe_data[key]
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
                global_estimate_position[key].append([
                    global_estimate_data[i].pose.position.x,
                    global_estimate_data[i].pose.position.y,
                    global_estimate_data[i].pose.position.z
                ])
                global_estimate_position_std[key].append([
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
                global_estimate_orientation_std[key].append([
                    np.sqrt(global_estimate_data[i].covariance[21]),
                    np.sqrt(global_estimate_data[i].covariance[28]),
                    np.sqrt(global_estimate_data[i].covariance[35])
                ])
                keyframe_estimate_position[key].append([
                    keyframe_estimate_data[i].pose.position.x,
                    keyframe_estimate_data[i].pose.position.y,
                    keyframe_estimate_data[i].pose.position.z
                ])
                keyframe_estimate_position_std[key].append([
                    np.sqrt(keyframe_estimate_data[i].covariance[0]),
                    np.sqrt(keyframe_estimate_data[i].covariance[7]),
                    np.sqrt(keyframe_estimate_data[i].covariance[14])
                ])
                keyframe_estimate_orientation[key].append([
                    keyframe_estimate_data[i].pose.orientation.x,
                    keyframe_estimate_data[i].pose.orientation.y,
                    keyframe_estimate_data[i].pose.orientation.z,
                    keyframe_estimate_data[i].pose.orientation.w
                ])
                keyframe_estimate_orientation_std[key].append([
                    np.sqrt(keyframe_estimate_data[i].covariance[21]),
                    np.sqrt(keyframe_estimate_data[i].covariance[28]),
                    np.sqrt(keyframe_estimate_data[i].covariance[35])
                ])
                keyframe_position[key].append([
                    keyframe_data[i].position.x,
                    keyframe_data[i].position.y,
                    keyframe_data[i].position.z
                ])
                keyframe_orientation[key].append([
                    keyframe_data[i].orientation.x,
                    keyframe_data[i].orientation.y,
                    keyframe_data[i].orientation.z,
                    keyframe_data[i].orientation.w
                ])
            time[key] = np.array(time[key]) - time[key][0]
            truth_position[key] = np.array(truth_position[key])
            truth_orientation[key] = np.array(truth_orientation[key])
            global_estimate_position[key] = np.array(global_estimate_position[key])
            global_estimate_position_std[key] = np.array(global_estimate_position_std[key])
            global_estimate_orientation[key] = np.array(global_estimate_orientation[key])
            global_estimate_orientation_std[key] = np.array(global_estimate_orientation_std[key])
            keyframe_estimate_position[key] = np.array(keyframe_estimate_position[key])
            keyframe_estimate_position_std[key] = np.array(keyframe_estimate_position_std[key])
            keyframe_estimate_orientation[key] = np.array(keyframe_estimate_orientation[key])
            keyframe_estimate_orientation_std[key] = np.array(keyframe_estimate_orientation_std[key])
            keyframe_position[key] = np.array(keyframe_position[key])
            keyframe_orientation[key] = np.array(keyframe_orientation[key])

        # Get filenames for saving plots and data
        counter = 0
        while os.path.exists(f'estimates_{counter}.png') \
                or os.path.exists(f'data_{counter}.npz'):
            counter += 1
        data_filename = f'data_{counter}.npz'
        combined_position_and_error_filename = f'combined_position_and_error_{counter}.png'
        global_error_filename = f'global_error_{counter}.png'
        keyframe_error_filename = f'keyframe_error_{counter}.png'

        # Save all data to a .npz file
        data = {}
        for key in self.truth_data.keys():
            data[f'{key}_time'] = time[key]
            data[f'{key}_truth_position'] = truth_position[key]
            data[f'{key}_truth_orientation'] = truth_orientation[key]
            data[f'{key}_global_estimate_position'] = global_estimate_position[key]
            data[f'{key}_global_estimate_position_std'] = global_estimate_position_std[key]
            data[f'{key}_global_estimate_orientation'] = global_estimate_orientation[key]
            data[f'{key}_global_estimate_orientation_std'] = global_estimate_orientation_std[key]
            data[f'{key}_keyframe_estimate_position'] = keyframe_estimate_position[key]
            data[f'{key}_keyframe_estimate_position_std'] = keyframe_estimate_position_std[key]
            data[f'{key}_keyframe_estimate_orientation'] = keyframe_estimate_orientation[key]
            data[f'{key}_keyframe_estimate_orientation_std'] = keyframe_estimate_orientation_std[key]
            data[f'{key}_keyframe_position'] = keyframe_position[key]
            data[f'{key}_keyframe_orientation'] = keyframe_orientation[key]
        np.savez(data_filename, **data)

        ### Combined Position and Position Error Plots ###

        # Calculate errors between truth and estimates
        global_position_error = {}
        keyframe_position_error = {}
        for key in self.truth_data.keys():
            global_position_error[key] = np.linalg.norm(truth_position[key] - global_estimate_position[key], axis=1)
            keyframe_position_error[key] = np.linalg.norm(truth_position[key] - keyframe_estimate_position[key] - keyframe_position[key], axis=1)

        # Create 2x2 plot for global and keyframe estimates
        fig, axs = plt.subplots(2, 2, figsize=(16, 12))

        # Global position data
        for key in self.truth_data.keys():
            axs[0, 0].plot(truth_position[key][:, 0], truth_position[key][:, 1], color='blue')
            axs[0, 0].plot(global_estimate_position[key][:, 0], global_estimate_position[key][:, 1], color='red')
        blue_patch = mpatches.Patch(color='blue', label='Truth')
        red_patch = mpatches.Patch(color='red', label='Global Estimate')
        axs[0, 0].set_xlabel('X Position (m)')
        axs[0, 0].set_ylabel('Y Position (m)')
        axs[0, 0].set_title('XY Position of Agents (Global Estimate)')
        axs[0, 0].legend(handles=[blue_patch, red_patch], handlelength=1.5, handleheight=0.1)
        axs[0, 0].axis('equal')

        # Global position error data
        for key in self.truth_data.keys():
            axs[1, 0].plot(time[key], global_position_error[key], label=key)
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Position Error (m)')
        axs[1, 0].set_title('Position Error of Agents (Global Estimate)')
        axs[1, 0].legend()
        axs[1, 0].grid()
        axs[1, 0].set_ylim(bottom=0)

        # Keyframe position data
        for key in self.truth_data.keys():
            keyframe_chained_positions = keyframe_estimate_position[key] + keyframe_position[key]
            axs[0, 1].plot(truth_position[key][:, 0], truth_position[key][:, 1], color='blue')
            axs[0, 1].plot(keyframe_chained_positions[:, 0], keyframe_chained_positions[:, 1], color='green')
        blue_patch = mpatches.Patch(color='blue', label='Truth')
        green_patch = mpatches.Patch(color='green', label='Keyframe Estimate')
        axs[0, 1].set_xlabel('X Position (m)')
        axs[0, 1].set_ylabel('Y Position (m)')
        axs[0, 1].set_title('XY Position of Agents (Keyframe Estimate)')
        axs[0, 1].legend(handles=[blue_patch, green_patch], handlelength=1.5, handleheight=0.1)
        axs[0, 1].axis('equal')

        # Keyframe position error data
        for key in self.truth_data.keys():
            axs[1, 1].plot(time[key], keyframe_position_error[key], label=key)
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Position Error (m)')
        axs[1, 1].set_title('Position Error of Agents (Keyframe Estimate)')
        axs[1, 1].legend()
        axs[1, 1].grid()
        axs[1, 1].set_ylim(bottom=0)

        plt.tight_layout()
        plt.savefig(combined_position_and_error_filename)

        ### Global Error Plots ###

        fig, axs = plt.subplots(6, len(truth_position.keys()), figsize=(16, 12))

        if len(truth_position.keys()) == 1:
            axs = np.expand_dims(axs, axis=1)

        column_idx = 0
        for key in self.truth_data.keys():

            # Calculate errors between truth and estimates
            global_estimate_position_error = global_estimate_position[key] - truth_position[key]
            truth_orientation[key] = np.array([R.from_quat(q).as_euler('xyz', degrees=False) for q in truth_orientation[key]])
            global_estimate_orientation[key] = np.array([R.from_quat(q).as_euler('xyz', degrees=False) for q in global_estimate_orientation[key]])
            global_estimate_orientation_error = global_estimate_orientation[key] - truth_orientation[key]

            # Wrap angles to [-pi, pi]
            global_estimate_orientation_error[:, 0] = (global_estimate_orientation_error[:, 0] + np.pi) % (2 * np.pi) - np.pi
            global_estimate_orientation_error[:, 1] = (global_estimate_orientation_error[:, 1] + np.pi) % (2 * np.pi) - np.pi
            global_estimate_orientation_error[:, 2] = (global_estimate_orientation_error[:, 2] + np.pi) % (2 * np.pi) - np.pi

            # X Position
            axs[0, column_idx].plot(time[key], global_estimate_position_error[:, 0], color='red')
            axs[0, column_idx].plot(time[key], global_estimate_position_std[key][:, 0]*3, color='blue')
            axs[0, column_idx].plot(time[key], -global_estimate_position_std[key][:, 0]*3, color='blue')
            axs[0, column_idx].set_title(key)
            axs[0, column_idx].grid()

            # Y Position
            axs[1, column_idx].plot(time[key], global_estimate_position_error[:, 1], color='red')
            axs[1, column_idx].plot(time[key], global_estimate_position_std[key][:, 1]*3, color='blue')
            axs[1, column_idx].plot(time[key], -global_estimate_position_std[key][:, 1]*3, color='blue')
            axs[1, column_idx].grid()

            # Z Position
            axs[2, column_idx].plot(time[key], global_estimate_position_error[:, 2], color='red')
            axs[2, column_idx].plot(time[key], global_estimate_position_std[key][:, 2]*3, color='blue')
            axs[2, column_idx].plot(time[key], -global_estimate_position_std[key][:, 2]*3, color='blue')
            axs[2, column_idx].grid()

            # Pitch
            axs[3, column_idx].plot(time[key], global_estimate_orientation_error[:, 0], color='red')
            axs[3, column_idx].plot(time[key], global_estimate_orientation_std[key][:, 0]*3, color='blue')
            axs[3, column_idx].plot(time[key], -global_estimate_orientation_std[key][:, 0]*3, color='blue')
            axs[3, column_idx].grid()

            # Roll
            axs[4, column_idx].plot(time[key], global_estimate_orientation_error[:, 1], color='red')
            axs[4, column_idx].plot(time[key], global_estimate_orientation_std[key][:, 1]*3, color='blue')
            axs[4, column_idx].plot(time[key], -global_estimate_orientation_std[key][:, 1]*3, color='blue')
            axs[4, column_idx].grid()

            # Yaw
            axs[5, column_idx].plot(time[key], global_estimate_orientation_error[:, 2], color='red')
            axs[5, column_idx].plot(time[key], global_estimate_orientation_std[key][:, 2]*3, color='blue')
            axs[5, column_idx].plot(time[key], -global_estimate_orientation_std[key][:, 2]*3, color='blue')
            axs[5, column_idx].set_xlabel('Time (s)')
            axs[5, column_idx].grid()

            # Add ylabels to leftmost plots
            if column_idx == 0:
                axs[0, column_idx].set_ylabel('North Error (m)')
                axs[1, column_idx].set_ylabel('East Error (m)')
                axs[2, column_idx].set_ylabel('Down Error (m)')
                axs[3, column_idx].set_ylabel('Roll Error (rad)')
                axs[4, column_idx].set_ylabel('Pitch Error (rad)')
                axs[5, column_idx].set_ylabel('Yaw Error (rad)')

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
