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
        DeclareLaunchArgument('plotting_enable', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/subscribe.launch.py']),
            launch_arguments={
                'namespace': 'ov_msckf',
                'config': 'magicc_fixedwing_gary',
                'max_cameras': '1',
                'use_stereo': 'false',
            }.items(),
        ),

        Node(
            package='ov_eval',
            executable='gpsdn_plotter.py',
            name='gpsdn_plotter',
            output='screen',
            condition=IfCondition(LaunchConfiguration("plotting_enable")),
            parameters=[{
                'agent_namespaces': ['ov_msckf']
            }]
        )
    ])
