from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dataset', default_value='gazebo_sinusoid/center_trajectory.txt'),
        DeclareLaunchArgument('config', default_value='magicc_fixedwing_sim'),
        DeclareLaunchArgument('rviz_enable', default_value='false'),
        DeclareLaunchArgument('max_cameras', default_value='1'),
        DeclareLaunchArgument('use_stereo', default_value='false'),
        DeclareLaunchArgument('feat_dist_min', default_value='45.0'),
        DeclareLaunchArgument('feat_dist_max', default_value='55.0'),
        DeclareLaunchArgument('freq_cam', default_value='10.0'),
        DeclareLaunchArgument('freq_imu', default_value='400.0'),
        DeclareLaunchArgument('namespace', default_value='ov_msckf_single_agent'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'dataset': LaunchConfiguration('dataset'),
                'config': LaunchConfiguration('config'),
                'rviz_enable': LaunchConfiguration('rviz_enable'),
                'max_cameras': LaunchConfiguration('max_cameras'),
                'use_stereo': LaunchConfiguration('use_stereo'),
                'feat_dist_min': LaunchConfiguration('feat_dist_min'),
                'feat_dist_max': LaunchConfiguration('feat_dist_max'),
                'freq_cam': LaunchConfiguration('freq_cam'),
                'freq_imu': LaunchConfiguration('freq_imu'),
                'namespace': LaunchConfiguration('namespace'),
            }.items(),
        ),
    ])
