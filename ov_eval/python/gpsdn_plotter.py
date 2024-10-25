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


class DataPlotterNode(Node):
    def __init__(self):
        super().__init__('data_plotter_node')

        # Declare parameters
        self.declare_parameter('agent_namespaces', ['ov_msckf'])

        # Get parameters
        agent_namespaces = self.get_parameter('agent_namespaces').get_parameter_value().string_array_value

        # Initialize data storage
        self.truth_data = {}
        for namespace in agent_namespaces:
            self.truth_data[namespace] = []
        self.global_estimate_data = {}
        for namespace in agent_namespaces:
            self.global_estimate_data[namespace] = []
        self.keyframe_estimate_data = {}
        for namespace in agent_namespaces:
            self.keyframe_estimate_data[namespace] = []

        # Create subscribers
        self.truth_subs = {}
        self.global_estimate_subs = {}
        self.keyframe_estimate_subs = {}
        for namespace in agent_namespaces:
            truth_topic_name = '/' + namespace + '/posegt'
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
                lambda msg, n=namespace: self.global_estimate_data[n].append(msg.pose.pose),
                1000
            )
            keyframe_estimate_topic_name = '/' + namespace + '/poseimukeyframe'
            self.keyframe_estimate_subs[namespace] = self.create_subscription(
                PoseWithCovarianceStamped,
                keyframe_estimate_topic_name,
                lambda msg, n=namespace: self.keyframe_estimate_data[n].append(msg.pose.pose),
                1000
            )


    def plot_data(self):
        # Extract data into usable format
        truth_position = {}
        truth_orientation = {}
        global_estimate_position = {}
        global_estimate_orientation = {}
        keyframe_estimate_position = {}
        keyframe_estimate_orientation = {}
        for key in self.truth_data.keys():
            truth_position[key] = []
            truth_orientation[key] = []
            global_estimate_position[key] = []
            global_estimate_orientation[key] = []
            keyframe_estimate_position[key] = []
            keyframe_estimate_orientation[key] = []

            truth_data = self.truth_data[key]
            global_estimate_data = self.global_estimate_data[key]
            keyframe_estimate_data = self.keyframe_estimate_data[key]
            for i in range(len(truth_data)):
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
                    global_estimate_data[i].position.x,
                    global_estimate_data[i].position.y,
                    global_estimate_data[i].position.z
                ])
                global_estimate_orientation[key].append([
                    global_estimate_data[i].orientation.x,
                    global_estimate_data[i].orientation.y,
                    global_estimate_data[i].orientation.z,
                    global_estimate_data[i].orientation.w
                ])
                keyframe_estimate_position[key].append([
                    keyframe_estimate_data[i].position.x,
                    keyframe_estimate_data[i].position.y,
                    keyframe_estimate_data[i].position.z
                ])
                keyframe_estimate_orientation[key].append([
                    keyframe_estimate_data[i].orientation.x,
                    keyframe_estimate_data[i].orientation.y,
                    keyframe_estimate_data[i].orientation.z,
                    keyframe_estimate_data[i].orientation.w
                ])
            truth_position[key] = np.array(truth_position[key])
            truth_orientation[key] = np.array(truth_orientation[key])
            global_estimate_position[key] = np.array(global_estimate_position[key])
            global_estimate_orientation[key] = np.array(global_estimate_orientation[key])
            keyframe_estimate_position[key] = np.array(keyframe_estimate_position[key])
            keyframe_estimate_orientation[key] = np.array(keyframe_estimate_orientation[key])

        # Get filenames for saving plots and data
        counter = 0
        while os.path.exists(f'estimates_{counter}.png') \
                or os.path.exists(f'data_{counter}.npz'):
            counter += 1
        estimates_plot_filename = f'estimates_{counter}.png'
        data_filename = f'data_{counter}.npz'

        # Calculate errors between truth and estimates
        # MAGICC TODO: Stack keyframes once reset is implemented
        global_position_error = {}
        global_orientation_error = {}
        keyframe_position_error = {}
        keyframe_orientation_error = {}
        for key in self.truth_data.keys():
            global_position_error[key] = np.linalg.norm(truth_position[key] - global_estimate_position[key], axis=1)
            global_orientation_error[key] = np.linalg.norm(truth_orientation[key] - global_estimate_orientation[key], axis=1)
            keyframe_position_error[key] = np.linalg.norm(truth_position[key] - keyframe_estimate_position[key], axis=1)
            keyframe_orientation_error[key] = np.linalg.norm(truth_orientation[key] - keyframe_estimate_orientation[key], axis=1)

        # Save all data to a .npz file
        data = {}
        for key in self.truth_data.keys():
            data[f'{key}_truth_position'] = truth_position[key]
            data[f'{key}_truth_orientation'] = truth_orientation[key]
            data[f'{key}_global_estimate_position'] = global_estimate_position[key]
            data[f'{key}_global_estimate_orientation'] = global_estimate_orientation[key]
            data[f'{key}_global_estimate_position_error'] = global_position_error[key]
            data[f'{key}_global_estimate_orientation_error'] = global_orientation_error[key]
            data[f'{key}_keyframe_estimate_position'] = keyframe_estimate_position[key]
            data[f'{key}_keyframe_estimate_orientation'] = keyframe_estimate_orientation[key]
            data[f'{key}_keyframe_estimate_position_error'] = keyframe_position_error[key]
            data[f'{key}_keyframe_estimate_orientation_error'] = keyframe_orientation_error[key]
        np.savez(data_filename, **data)

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
            axs[1, 0].plot(global_position_error[key], label=key)
        axs[1, 0].set_ylabel('Position Error (m)')
        axs[1, 0].set_title('Position Error of Agents (Global Estimate)')
        axs[1, 0].legend()
        axs[1, 0].grid()
        axs[1, 0].set_ylim(bottom=0)

        # Keyframe position data
        for key in self.truth_data.keys():
            axs[0, 1].plot(truth_position[key][:, 0], truth_position[key][:, 1], color='blue')
            axs[0, 1].plot(keyframe_estimate_position[key][:, 0], keyframe_estimate_position[key][:, 1], color='green')
        blue_patch = mpatches.Patch(color='blue', label='Truth')
        green_patch = mpatches.Patch(color='green', label='Keyframe Estimate')
        axs[0, 1].set_xlabel('X Position (m)')
        axs[0, 1].set_ylabel('Y Position (m)')
        axs[0, 1].set_title('XY Position of Agents (Keyframe Estimate)')
        axs[0, 1].legend(handles=[blue_patch, green_patch], handlelength=1.5, handleheight=0.1)
        axs[0, 1].axis('equal')

        # Keyframe position error data
        for key in self.truth_data.keys():
            axs[1, 1].plot(keyframe_position_error[key], label=key)
        axs[1, 1].set_ylabel('Position Error (m)')
        axs[1, 1].set_title('Position Error of Agents (Keyframe Estimate)')
        axs[1, 1].legend()
        axs[1, 1].grid()
        axs[1, 1].set_ylim(bottom=0)

        plt.tight_layout()
        plt.savefig(estimates_plot_filename)

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
