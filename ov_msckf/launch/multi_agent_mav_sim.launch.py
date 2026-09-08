from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agent_names', default_value='center,left,right'),
        DeclareLaunchArgument('save_results', default_value='false'),
        DeclareLaunchArgument('results_path', default_value='results'),
        DeclareLaunchArgument('relinearize_skip', default_value='10'),
        DeclareLaunchArgument('relinearize_threshold', default_value='0.1'),
        DeclareLaunchArgument('use_qr', default_value='false'),
        DeclareLaunchArgument(
            'datasets',
            default_value='gazebo_sinusoid/center_trajectory.txt,'
                          'gazebo_sinusoid/left_trajectory.txt,'
                          'gazebo_sinusoid/right_trajectory.txt',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/simulation.launch.py']),
            launch_arguments={
                'simulation_executable': 'run_multi_agent_simulation',
                'namespace': 'multi_agent',
                'rviz_enable': 'false',
                'agent_names': LaunchConfiguration('agent_names'),
                'datasets': LaunchConfiguration('datasets'),
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
    ])
