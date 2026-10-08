from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag_path", description="HoloOcean ROS 2 bag directory"),
        DeclareLaunchArgument("bag_start", default_value="0.0"),
        DeclareLaunchArgument("bag_duration", default_value="-1.0"),
        DeclareLaunchArgument("max_gps_init_time", default_value=""),
        DeclareLaunchArgument("initial_global_yaw", default_value=""),
        DeclareLaunchArgument("topic_gps_fix", default_value="/gps/fix"),
        DeclareLaunchArgument("topic_gps_velocity", default_value="/gps/velocity"),
        DeclareLaunchArgument("rviz_enable", default_value="false"),
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument("track_frequency", default_value=""),
        DeclareLaunchArgument("save_total_state", default_value=""),
        DeclareLaunchArgument("filepath_est", default_value=""),
        DeclareLaunchArgument("filepath_std", default_value=""),
        DeclareLaunchArgument("path_gt", default_value=""),
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
                "bag_start": LaunchConfiguration("bag_start"),
                "bag_duration": LaunchConfiguration("bag_duration"),
                "config": "holoocean_fixedwing",
                "max_gps_init_time": LaunchConfiguration("max_gps_init_time"),
                "initial_global_yaw": LaunchConfiguration("initial_global_yaw"),
                "topic_gps_fix": LaunchConfiguration("topic_gps_fix"),
                "topic_gps_velocity": LaunchConfiguration("topic_gps_velocity"),
                "topic_imu": "/imu/data",
                "camera_topics": "/fixedwing/camera/image_raw",
                "rviz_enable": LaunchConfiguration("rviz_enable"),
                "verbosity": LaunchConfiguration("verbosity"),
                "track_frequency": LaunchConfiguration("track_frequency"),
                "save_total_state": LaunchConfiguration("save_total_state"),
                "filepath_est": LaunchConfiguration("filepath_est"),
                "filepath_std": LaunchConfiguration("filepath_std"),
                "path_gt": LaunchConfiguration("path_gt"),
                "filter_type": LaunchConfiguration("filter_type"),
                "relinearize_skip": LaunchConfiguration("relinearize_skip"),
                "relinearize_threshold": LaunchConfiguration("relinearize_threshold"),
                "use_qr": LaunchConfiguration("use_qr"),
                "save_results": LaunchConfiguration("save_results"),
                "results_path": LaunchConfiguration("results_path"),
            }.items(),
        ),
    ])
