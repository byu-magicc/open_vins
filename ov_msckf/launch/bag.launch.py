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
        "bag_start": ParameterValue(LaunchConfiguration("bag_start"), value_type=float),
        "bag_duration": ParameterValue(LaunchConfiguration("bag_duration"), value_type=float),
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
        ("track_frequency", float), ("num_opencv_threads", int), ("multi_threading_pubs", bool),
        ("save_total_state", bool), ("filepath_est", str), ("filepath_std", str), ("path_gt", str),
        ("publish_global_to_imu_tf", bool), ("publish_calibration_tf", bool),
    ):
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    camera_topics = LaunchConfiguration("camera_topics").perform(context)
    if camera_topics:
        parameters["camera_topics"] = [topic.strip() for topic in camera_topics.split(",")]
    for index in range(2):
        topic = LaunchConfiguration("topic_camera" + str(index)).perform(context)
        if topic:
            parameters["topic_camera" + str(index)] = topic

    def finish_bag(event, context):
        if event.returncode != 0:
            raise RuntimeError(f"run_bag failed with exit code {event.returncode}")
        return [Shutdown(reason="Bag processing finished")]

    rviz_config_path = LaunchConfiguration("rviz_config_path").perform(context)
    if not rviz_config_path:
        rviz_config_path = os.path.join(get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz")

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
            remappings=[
                ("/ov_msckf/" + topic,
                 os.path.join("/", LaunchConfiguration("namespace").perform(context).strip("/"), topic))
                for topic in [
                    "trackhist", "loop_depth_colored", "pathimu", "pathgt",
                    "points_msckf", "points_slam", "points_aruco", "loop_feats", "points_sim",
                ]
            ],
            arguments=[
                "-d", rviz_config_path,
                "--ros-args", "--log-level", "warn",
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag_path", description="ROS 2 bag directory or file"),
        DeclareLaunchArgument("bag_start", default_value="0.0", description="Start offset from the first IMU header timestamp (seconds)"),
        DeclareLaunchArgument("bag_duration", default_value="-1.0", description="Replay duration in seconds; -1 processes the remaining bag"),
        DeclareLaunchArgument("namespace", default_value="ov_msckf"),
        DeclareLaunchArgument("config", default_value="euroc_mav"),
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("topic_imu", default_value="", description="Override the IMU topic from the config"),
        DeclareLaunchArgument("camera_topics", default_value="", description="Comma-separated camera topics in camera ID order"),
        DeclareLaunchArgument("topic_camera0", default_value="", description="Override camera 0 topic; camera_topics takes precedence"),
        DeclareLaunchArgument("topic_camera1", default_value="", description="Override camera 1 topic; camera_topics takes precedence"),
        DeclareLaunchArgument("max_gps_init_time", default_value="", description="GPS assistance duration from VIO initialization (seconds)"),
        DeclareLaunchArgument("initial_global_yaw", default_value="", description="IMU heading at VIO initialization in ENU radians, counterclockwise from east"),
        DeclareLaunchArgument("topic_gps_fix", default_value="/gps/fix"),
        DeclareLaunchArgument("topic_gps_velocity", default_value="/gps/velocity"),
        DeclareLaunchArgument("max_cameras", default_value=""),
        DeclareLaunchArgument("use_stereo", default_value=""),
        DeclareLaunchArgument("track_frequency", default_value="", description="Maximum camera tracking rate per stream (Hz)"),
        DeclareLaunchArgument("num_opencv_threads", default_value="", description="Override OpenCV worker count from the config"),
        DeclareLaunchArgument("multi_threading_pubs", default_value="", description="Publish tracking images in a background thread"),
        DeclareLaunchArgument("save_total_state", default_value="", description="Save legacy state and deviation files, including without RViz"),
        DeclareLaunchArgument("filepath_est", default_value=""),
        DeclareLaunchArgument("filepath_std", default_value=""),
        DeclareLaunchArgument("path_gt", default_value="", description="ASL-format ground-truth CSV for evaluation"),
        DeclareLaunchArgument("publish_global_to_imu_tf", default_value=""),
        DeclareLaunchArgument("publish_calibration_tf", default_value=""),
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
        DeclareLaunchArgument("rviz_config_path", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
