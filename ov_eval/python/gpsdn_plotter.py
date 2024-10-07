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
        self.estimated_data = {}
        for namespace in agent_namespaces:
            self.estimated_data[namespace] = []

        # Create subscribers
        self.truth_subs = {}
        self.estimated_subs = {}
        for namespace in agent_namespaces:
            truth_topic_name = '/' + namespace + '/posegt'
            self.truth_subs[namespace] = self.create_subscription(
                PoseStamped,
                truth_topic_name,
                lambda msg, n=namespace: self.truth_data[n].append(msg.pose),
                1000
            )
            estimated_topic_name = '/' + namespace + '/poseimu'
            self.estimated_subs[namespace] = self.create_subscription(
                PoseWithCovarianceStamped,
                estimated_topic_name,
                lambda msg, n=namespace: self.estimated_data[n].append(msg.pose.pose),
                1000
            )

    def plot_data(self):
        # Extract data into usable format
        truth_position = {}
        truth_orientation = {}
        estimated_position = {}
        estimated_orientation = {}
        for key in self.truth_data.keys():
            truth_position[key] = []
            truth_orientation[key] = []
            estimated_position[key] = []
            estimated_orientation[key] = []

            truth_data = self.truth_data[key]
            estimated_data = self.estimated_data[key]
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
                estimated_position[key].append([
                    estimated_data[i].position.x,
                    estimated_data[i].position.y,
                    estimated_data[i].position.z
                ])
                estimated_orientation[key].append([
                    estimated_data[i].orientation.x,
                    estimated_data[i].orientation.y,
                    estimated_data[i].orientation.z,
                    estimated_data[i].orientation.w
                ])
            truth_position[key] = np.array(truth_position[key])
            truth_orientation[key] = np.array(truth_orientation[key])
            estimated_position[key] = np.array(estimated_position[key])
            estimated_orientation[key] = np.array(estimated_orientation[key])

        # Get filenames for saving plots and data
        counter = 0
        while os.path.exists(f'xy_position_{counter}.png') \
                or os.path.exists(f'position_error_{counter}.png') \
                or os.path.exists(f'data_{counter}.npz'):
            counter += 1
        position_filename = f'xy_position_{counter}.png'
        error_filename = f'position_error_{counter}.png'
        data_filename = f'data_{counter}.npz'

        # Calculate errors between truth and estimate
        position_error = {}
        orientation_error = {}
        for key in self.truth_data.keys():
            position_error[key] = np.linalg.norm(truth_position[key] - estimated_position[key], axis=1)
            orientation_error[key] = np.linalg.norm(truth_orientation[key] - estimated_orientation[key], axis=1)

        # Plot position data
        plt.figure()
        for key in self.truth_data.keys():
            plt.plot(truth_position[key][:, 0], truth_position[key][:, 1], color='blue')
            plt.plot(estimated_position[key][:, 0], estimated_position[key][:, 1], color='red')
        blue_patch = mpatches.Patch(color='blue', label='Truth')
        red_patch = mpatches.Patch(color='red', label='Estimated')
        plt.legend(handles=[blue_patch, red_patch], handlelength=1.5, handleheight=0.1)
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('XY Position of Agents')
        plt.axis('equal')
        plt.savefig(position_filename)

        # Plot position error data
        plt.figure()
        for key in self.truth_data.keys():
            plt.plot(position_error[key], label=key)
        plt.ylabel('Position Error (m)')
        plt.title('Position Error of Agents')
        plt.legend()
        plt.grid()
        plt.gca().set_ylim(bottom=0)
        plt.savefig(error_filename)

        # Save all data to a .npz file
        data = {}
        for key in self.truth_data.keys():
            data[f'{key}_truth_position'] = truth_position[key]
            data[f'{key}_truth_orientation'] = truth_orientation[key]
            data[f'{key}_estimated_position'] = estimated_position[key]
            data[f'{key}_estimated_orientation'] = estimated_orientation[key]
            data[f'{key}_position_error'] = position_error[key]
            data[f'{key}_orientation_error'] = orientation_error[key]
        counter = 0
        np.savez(data_filename, **data)

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
