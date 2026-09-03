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
        DeclareLaunchArgument('relinearize_skip', default_value='10'),
        DeclareLaunchArgument('relinearize_threshold', default_value='0.1'),
        DeclareLaunchArgument('use_qr', default_value='false'),

        # Instance 0
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'namespace': 'ov_msckf',
                'dataset': 'gazebo_sinusoid/center_trajectory.txt',
                'config': 'magicc_fixedwing_sim',
                'max_cameras': '1',
                'use_stereo': 'false',
                'use_factor_graph': 'true',
                'relinearize_skip': LaunchConfiguration('relinearize_skip'),
                'relinearize_threshold': LaunchConfiguration('relinearize_threshold'),
                'use_qr': LaunchConfiguration('use_qr'),
                'save_results': LaunchConfiguration('save_results'),
                'results_path': LaunchConfiguration('results_path'),
                'use_ground_plane_features': 'true',
                'ground_plane_features_range': '2.0',
            }.items(),
        ),

        # Instance 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'rviz_enable': 'false',
                'namespace': 'ov_msckf_left',
                'dataset': 'gazebo_sinusoid/left_trajectory.txt',
                'config': 'magicc_fixedwing_sim',
                'max_cameras': '1',
                'use_stereo': 'false',
                'use_factor_graph': 'true',
                'relinearize_skip': LaunchConfiguration('relinearize_skip'),
                'relinearize_threshold': LaunchConfiguration('relinearize_threshold'),
                'use_qr': LaunchConfiguration('use_qr'),
                'save_results': LaunchConfiguration('save_results'),
                'results_path': LaunchConfiguration('results_path'),
                'use_ground_plane_features': 'true',
                'ground_plane_features_range': '2.0',
            }.items(),
        ),

        # Instance 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'rviz_enable': 'false',
                'namespace': 'ov_msckf_right',
                'dataset': 'gazebo_sinusoid/right_trajectory.txt',
                'config': 'magicc_fixedwing_sim',
                'max_cameras': '1',
                'use_stereo': 'false',
                'use_factor_graph': 'true',
                'relinearize_skip': LaunchConfiguration('relinearize_skip'),
                'relinearize_threshold': LaunchConfiguration('relinearize_threshold'),
                'use_qr': LaunchConfiguration('use_qr'),
                'save_results': LaunchConfiguration('save_results'),
                'results_path': LaunchConfiguration('results_path'),
                'use_ground_plane_features': 'true',
                'ground_plane_features_range': '2.0',
            }.items(),
        )
    ])
