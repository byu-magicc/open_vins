import os
import sys

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

sys.path.insert(0, os.path.dirname(__file__))
from openvins_launch import initialization_arguments

def generate_launch_description():
    return LaunchDescription([
        *initialization_arguments("true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'initial_state_covariance': LaunchConfiguration('initial_state_covariance'),
                "initial_velocity_uncertainty_in_local_frame": LaunchConfiguration("initial_velocity_uncertainty_in_local_frame"),
                'add_initial_error': LaunchConfiguration('add_initial_error'),
                'namespace': 'center',
                'dataset': 'gazebo_sinusoid/center_trajectory.txt',
                'config': 'magicc_fixedwing_sim',
                'use_ground_plane_features': 'true',
            }.items(),
        )
    ])
