from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    workspace_root = Path(__file__).resolve().parents[4]

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=str(workspace_root / 'data' / 'gary_data' / 'flight22'),
            description='Path to the rosbag to play',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/subscribe.launch.py']),
            launch_arguments={
                'namespace': 'ov_msckf',
                'config': 'magicc_fixedwing_gary',
                'max_cameras': '1',
                'use_stereo': 'false',
            }.items(),
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
            output='screen',
        ),
    ])
