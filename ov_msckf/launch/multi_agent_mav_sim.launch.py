from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Instance 0
        DeclareLaunchArgument('dataset_0', default_value='gazebo_sinusoid/center_trajectory.txt'),
        DeclareLaunchArgument('namespace_0', default_value='ov_msckf_0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'dataset': LaunchConfiguration('dataset_0'),
                'namespace': LaunchConfiguration('namespace_0'),
            }.items(),
        ),

        # Instance 1
        DeclareLaunchArgument('dataset_1', default_value='gazebo_sinusoid/left_trajectory.txt'),
        DeclareLaunchArgument('namespace_1', default_value='ov_msckf_1'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'dataset': LaunchConfiguration('dataset_1'),
                'namespace': LaunchConfiguration('namespace_1'),
            }.items(),
        ),

        # Instance 2
        DeclareLaunchArgument('dataset_2', default_value='gazebo_sinusoid/right_trajectory.txt'),
        DeclareLaunchArgument('namespace_2', default_value='ov_msckf_2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'dataset': LaunchConfiguration('dataset_2'),
                'namespace': LaunchConfiguration('namespace_2'),
            }.items(),
        ),
    ])
