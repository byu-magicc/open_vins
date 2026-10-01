from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag_path", description="HoloOcean ROS 2 bag directory"),
        DeclareLaunchArgument("rviz_enable", default_value="false"),
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument(
            "filter_type", default_value="openvins", description="estimator to expose: openvins, factor_graph, or hybrid",
        ),
        DeclareLaunchArgument(
            "relinearize_skip", default_value="10", description="number of iSAM2 updates between relinearization checks",
        ),
        DeclareLaunchArgument(
            "relinearize_threshold", default_value="0.1", description="change in a variable required for iSAM2 to relinearize it",
        ),
        DeclareLaunchArgument(
            "use_qr", default_value="false", description="use QR factorization in iSAM2 instead of Cholesky",
        ),
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
                "filter_type": LaunchConfiguration("filter_type"),
                "relinearize_skip": LaunchConfiguration("relinearize_skip"),
                "relinearize_threshold": LaunchConfiguration("relinearize_threshold"),
                "use_qr": LaunchConfiguration("use_qr"),
                "save_results": LaunchConfiguration("save_results"),
                "results_path": LaunchConfiguration("results_path"),
            }.items(),
        ),
    ])
