import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    if not bag_path or not os.path.exists(bag_path):
        raise RuntimeError(f"Bag path does not exist: {bag_path}")

    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        config_path = os.path.join(
            get_package_share_directory("ov_msckf"),
            "config",
            LaunchConfiguration("config").perform(context),
            "estimator_config.yaml",
        )
    if not os.path.isfile(config_path):
        raise RuntimeError(f"Estimator config does not exist: {config_path}")

    parameters = {
        "bag_path": bag_path,
        "config_path": config_path,
        "verbosity": LaunchConfiguration("verbosity"),
        "filter_type": LaunchConfiguration("filter_type"),
        "relinearize_skip": ParameterValue(LaunchConfiguration("relinearize_skip"), value_type=int),
        "relinearize_threshold": ParameterValue(LaunchConfiguration("relinearize_threshold"), value_type=float),
        "use_qr": ParameterValue(LaunchConfiguration("use_qr"), value_type=bool),
        "save_results": ParameterValue(LaunchConfiguration("save_results"), value_type=bool),
        "results_path": ParameterValue(LaunchConfiguration("results_path"), value_type=str),
        "visualize": ParameterValue(LaunchConfiguration("rviz_enable"), value_type=bool),
    }
    for name, value_type in (
        ("max_cameras", int), ("use_stereo", bool), ("topic_imu", str),
        ("max_gps_init_time", float), ("initial_global_yaw", float),
        ("topic_gps_fix", str), ("topic_gps_velocity", str),
    ):
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = float(value) if value_type is float else ParameterValue(value, value_type=value_type)
    camera_topics = LaunchConfiguration("camera_topics").perform(context)
    if camera_topics:
        parameters["camera_topics"] = [topic.strip() for topic in camera_topics.split(",")]

    def finish_bag(event, context):
        if event.returncode != 0:
            raise RuntimeError(f"run_bag failed with exit code {event.returncode}")
        return [Shutdown(reason="Bag processing finished")]

    return [
        Node(
            package="ov_msckf",
            executable="run_bag",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[parameters],
            on_exit=finish_bag,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(LaunchConfiguration("rviz_enable")),
            arguments=[
                "-d", os.path.join(get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"),
                "--ros-args", "--log-level", "warn",
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag_path", description="ROS 2 bag directory or file"),
        DeclareLaunchArgument("namespace", default_value="ov_msckf"),
        DeclareLaunchArgument("config", default_value="euroc_mav"),
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("topic_imu", default_value="", description="Override the IMU topic from the config"),
        DeclareLaunchArgument("camera_topics", default_value="", description="Comma-separated camera topics in camera ID order"),
        DeclareLaunchArgument("max_gps_init_time", default_value="", description="GPS assistance duration from VIO initialization (seconds)"),
        DeclareLaunchArgument("initial_global_yaw", default_value="", description="IMU heading at VIO initialization in ENU radians, counterclockwise from east"),
        DeclareLaunchArgument("topic_gps_fix", default_value="/gps/fix"),
        DeclareLaunchArgument("topic_gps_velocity", default_value="/gps/velocity"),
        DeclareLaunchArgument("max_cameras", default_value=""),
        DeclareLaunchArgument("use_stereo", default_value=""),
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
        DeclareLaunchArgument("rviz_enable", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
