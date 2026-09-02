from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('save_results', default_value='false'),
        DeclareLaunchArgument('results_path', default_value='results'),
        DeclareLaunchArgument('use_factor_graph', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'namespace': 'ov_msckf',
                'dataset': 'gazebo_sinusoid/center_trajectory.txt',
                'config': 'magicc_fixedwing_sim',
                'max_cameras': '1',
                'use_stereo': 'false',
                'use_factor_graph': LaunchConfiguration('use_factor_graph'),
                'save_results': LaunchConfiguration('save_results'),
                'results_path': LaunchConfiguration('results_path'),
                'use_ground_plane_features': 'true',
                'ground_plane_features_range': '2.0',
            }.items(),
        )
    ])
