from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag_path", description="HoloOcean ROS 2 bag directory"),
        DeclareLaunchArgument("rviz_enable", default_value="false"),
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument("save_results", default_value="false"),
        DeclareLaunchArgument("results_path", default_value="results"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/bag.launch.py"]),
            launch_arguments={
                "bag_path": LaunchConfiguration("bag_path"),
                "config": "holoocean_fixedwing",
                "topic_imu": "/imu/data",
                "camera_topics": "/fixedwing/camera/image_raw",
                "rviz_enable": LaunchConfiguration("rviz_enable"),
                "verbosity": LaunchConfiguration("verbosity"),
                "save_results": LaunchConfiguration("save_results"),
                "results_path": LaunchConfiguration("results_path"),
            }.items(),
        ),
    ])
