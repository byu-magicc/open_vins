from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz_enable', default_value='false'),
        DeclareLaunchArgument('plotting_enable', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'rviz_enable': LaunchConfiguration('rviz_enable'),
                'namespace': 'ov_msckf_0',
                'dataset': 'gazebo_sinusoid/center_trajectory.txt',
                'config': 'magicc_fixedwing_sim',
                'max_cameras': '1',
                'use_stereo': 'false',
                'use_ground_plane_features': 'true',
                'ground_plane_features_range': '2.0',
            }.items(),
        ),

        Node(
            package='ov_eval',
            executable='gpsdn_plotter.py',
            name='gpsdn_plotter',
            output='screen',
            condition=IfCondition(LaunchConfiguration("plotting_enable")),
            parameters=[{
                'agent_namespaces': ['ov_msckf_0']
            }]
        )
    ])
