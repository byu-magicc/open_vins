from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'namespace': 'center',
                'dataset': 'gazebo_sinusoid/center_trajectory.txt',
                'config': 'magicc_fixedwing_sim',
                'use_ground_plane_features': 'true',
            }.items(),
        )
    ])
