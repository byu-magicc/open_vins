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

        # Plot data
        plt.figure()
        for key in self.truth_data.keys():
            plt.plot(truth_position[key][:, 0], truth_position[key][:, 1], color='blue')
            plt.plot(estimated_position[key][:, 0], estimated_position[key][:, 1], color='red')

        # Add legend
        blue_patch = mpatches.Patch(color='blue', label='Truth')
        red_patch = mpatches.Patch(color='red', label='Estimated')
        plt.legend(handles=[blue_patch, red_patch], handlelength=1.5, handleheight=0.1)

        # Set plot labels
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('XY Position of Agents')

        # Lock aspect ratio
        plt.axis('equal')

        # Save figure, using iteration number as filename
        counter = 0
        while os.path.exists(f'xy_position_{counter}.png'):
            counter += 1
        plt.savefig(f'xy_position_{counter}.png')

        self.get_logger().info('Plots generated successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = DataPlotterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Generating plots...')
    node.plot_data()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
