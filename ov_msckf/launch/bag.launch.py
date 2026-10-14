import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from openvins_launch import graph_arguments, graph_parameters, camera_parameters

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
        **graph_parameters(),
        "bag_path": bag_path,
        "bag_start": ParameterValue(LaunchConfiguration("bag_start"), value_type=float),
        "bag_duration": ParameterValue(LaunchConfiguration("bag_duration"), value_type=float),
        "config_path": config_path,
        "num_opencv_threads": 0,
        "verbosity": LaunchConfiguration("verbosity"),
        "visualize": ParameterValue(LaunchConfiguration("rviz_enable"), value_type=bool),
    }
    for name, value_type in (
        ("topic_imu", str),
        ("save_total_state", bool), ("filepath_est", str), ("filepath_std", str), ("path_gt", str),
        ("publish_global_to_imu_tf", bool), ("publish_calibration_tf", bool),
    ):
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    initial_state = LaunchConfiguration("initial_state_imu").perform(context)
    if initial_state:
        state = json.loads(initial_state)
        if (not isinstance(state, list) or len(state) != 17 or
                any(type(value) not in (int, float) or not math.isfinite(value) for value in state)):
            raise RuntimeError("initial_state_imu must be a JSON array of 17 finite numbers")
        parameters["initial_state_imu"] = [float(value) for value in state]

    parameters.update(camera_parameters(context))

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
        *graph_arguments(10),
        DeclareLaunchArgument("bag_path", description="ROS 2 bag directory or file"),
        DeclareLaunchArgument("bag_start", default_value="0.0", description="Start offset from the first IMU header timestamp (seconds)"),
        DeclareLaunchArgument("initial_state_imu", default_value="", description="Optional JSON truth state [IMU timestamp, q_xyzw, p, v, bg, ba]"),
        DeclareLaunchArgument("bag_duration", default_value="-1.0", description="Replay duration in seconds; -1 processes the remaining bag"),
        DeclareLaunchArgument("namespace", default_value="ov_msckf"),
        DeclareLaunchArgument("config", default_value="euroc_mav"),
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("topic_imu", default_value="", description="Override the IMU topic from the config"),
        DeclareLaunchArgument("camera_topics", default_value="", description="Comma-separated camera topics in camera ID order"),
        DeclareLaunchArgument("topic_camera0", default_value="", description="Override camera 0 topic; camera_topics takes precedence"),
        DeclareLaunchArgument("topic_camera1", default_value="", description="Override camera 1 topic; camera_topics takes precedence"),
        DeclareLaunchArgument("save_total_state", default_value="", description="Save legacy state and deviation files, including without RViz"),
        DeclareLaunchArgument("filepath_est", default_value=""),
        DeclareLaunchArgument("filepath_std", default_value=""),
        DeclareLaunchArgument("path_gt", default_value="", description="ASL-format ground-truth CSV for evaluation"),
        DeclareLaunchArgument("publish_global_to_imu_tf", default_value=""),
        DeclareLaunchArgument("publish_calibration_tf", default_value=""),
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument("rviz_enable", default_value="false"),
        DeclareLaunchArgument("rviz_config_path", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
