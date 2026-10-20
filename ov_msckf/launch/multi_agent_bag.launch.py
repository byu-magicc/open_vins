import math
import os
import re
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(__file__))
from openvins_launch import graph_arguments, graph_parameters, camera_parameters, rviz_configuration
from openvins_launch import initialization_arguments, initialization_parameters

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    names = [name.strip() for name in LaunchConfiguration("agent_names").perform(context).split(",")]
    bags = [path.strip() for path in LaunchConfiguration("bag_paths").perform(context).split(",")]
    if (len(names) != len(bags) or len(set(names)) != len(names) or
            any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name) for name in names)):
        raise RuntimeError("agent_names must be unique ROS namespace tokens, with one name per bag")
    for bag in bags:
        if not bag or not os.path.exists(bag):
            raise RuntimeError(f"Bag path does not exist: {bag}")
    truth_paths = [path.strip() for path in LaunchConfiguration("truth_paths").perform(context).split(",")]
    if len(truth_paths) != len(names) or any(not os.path.isdir(path) for path in truth_paths):
        raise RuntimeError("truth_paths must contain one existing truth directory per bag")
    starts_argument = LaunchConfiguration("bag_starts").perform(context)
    starts = [float(value) for value in starts_argument.split(",")] if starts_argument else [0.0] * len(names)
    if len(starts) != len(names) or any(not math.isfinite(value) or value < 0 for value in starts):
        raise RuntimeError("bag_starts must contain one finite nonnegative offset per agent")
    duration = float(LaunchConfiguration("bag_duration").perform(context))
    if not math.isfinite(duration) or (duration != -1.0 and duration <= 0):
        raise RuntimeError("bag_duration must be positive or -1")

    package_directory = get_package_share_directory("ov_msckf")
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        config_path = os.path.join(package_directory, "config", LaunchConfiguration("config").perform(context), "estimator_config.yaml")
    if not os.path.isfile(config_path):
        raise RuntimeError(f"Estimator config does not exist: {config_path}")
    imu_topic = LaunchConfiguration("topic_imu").perform(context)
    if not imu_topic:
        with open(config_path, encoding="utf-8") as config_file:
            configuration = yaml.safe_load(config_file.read().removeprefix("%YAML:1.0"))
        imu_path = os.path.join(os.path.dirname(config_path), configuration["relative_config_imu"])
        with open(imu_path, encoding="utf-8") as imu_file:
            imu_configuration = yaml.safe_load(imu_file.read().removeprefix("%YAML:1.0"))
        imu_topic = imu_configuration["imu0"]["rostopic"]

    parameters = {
        **graph_parameters(),
        "agent_names": names,
        **initialization_parameters(context),
        "bag_paths": bags,
        "truth_paths": truth_paths,
        "bag_starts": starts,
        "bag_duration": duration,
        "config_path": config_path,
        "num_opencv_threads": 0,
        "topic_imu": imu_topic,
        "verbosity": LaunchConfiguration("verbosity"),
        "range_stddev": ParameterValue(LaunchConfiguration("range_stddev"), value_type=float),
        "range_interval": ParameterValue(LaunchConfiguration("range_interval"), value_type=float),
        "range_jitter_fraction": ParameterValue(LaunchConfiguration("range_jitter_fraction"), value_type=float),
        "sim_seed_measurements": ParameterValue(LaunchConfiguration("seed"), value_type=int),
        "visualize": ParameterValue(LaunchConfiguration("rviz_enable"), value_type=bool),
    }
    for name, value_type in (
        ("publish_global_to_imu_tf", bool), ("publish_calibration_tf", bool),
    ):
        if LaunchConfiguration(name).perform(context):
            parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)

    parameters.update(camera_parameters(context))

    temporary_directory = tempfile.mkdtemp(prefix="openvins_multi_agent_bag_")
    try:
        rviz_config_path = LaunchConfiguration("rviz_config_path").perform(context)
        if IfCondition(LaunchConfiguration("rviz_enable")).evaluate(context) and not rviz_config_path:
            namespace = LaunchConfiguration("namespace").perform(context)
            configuration = rviz_configuration(names, namespace)
            rviz_config_path = os.path.join(temporary_directory, "multi_agent.rviz")
            with open(rviz_config_path, "w", encoding="utf-8") as rviz_file:
                rviz_file.write(configuration)

        def remove_temporary_files(event, launch_context):
            del event, launch_context
            shutil.rmtree(temporary_directory, ignore_errors=True)

        def finish_bags(event, launch_context):
            del launch_context
            if event.returncode != 0:
                raise RuntimeError(f"run_multi_agent_bag failed with exit code {event.returncode}")
            return [Shutdown(reason="Multi-agent bag processing finished")]

        return [
            RegisterEventHandler(OnShutdown(on_shutdown=remove_temporary_files)),
            Node(
                package="ov_msckf", executable="run_multi_agent_bag",
                namespace=LaunchConfiguration("namespace"), output="screen",
                parameters=[parameters], on_exit=finish_bags,
            ),
            Node(
                package="rviz2", executable="rviz2", output="screen",
                condition=IfCondition(LaunchConfiguration("rviz_enable")),
                arguments=["-d", rviz_config_path, "--ros-args", "--log-level", "warn"],
            ),
        ]
    except BaseException:
        shutil.rmtree(temporary_directory, ignore_errors=True)
        raise


def generate_launch_description():
    return LaunchDescription([
        *graph_arguments(10),
        *initialization_arguments("false"),
        DeclareLaunchArgument("bag_paths", description="Comma-separated ROS 2 bag directories or files"),
        DeclareLaunchArgument(
            "truth_paths", description="Comma-separated directories containing truth.csv and clock.csv, one per bag",
        ),
        DeclareLaunchArgument("agent_names", default_value="center,left,right"),
        DeclareLaunchArgument("bag_starts", default_value="", description="One start offset per bag, relative to its first IMU header; empty uses zeros"),
        DeclareLaunchArgument("bag_duration", default_value="-1.0", description="Replay duration per bag; -1 processes the remaining bags"),
        DeclareLaunchArgument("namespace", default_value="multi_agent"),
        DeclareLaunchArgument("config", default_value="euroc_mav"),
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("topic_imu", default_value=""),
        DeclareLaunchArgument("camera_topics", default_value="", description="Comma-separated camera topics in camera ID order"),
        DeclareLaunchArgument("topic_camera0", default_value=""),
        DeclareLaunchArgument("topic_camera1", default_value=""),
        DeclareLaunchArgument("publish_global_to_imu_tf", default_value=""),
        DeclareLaunchArgument("publish_calibration_tf", default_value=""),
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument("range_stddev", default_value="1.0", description="Range noise standard deviation in meters"),
        DeclareLaunchArgument("range_interval", default_value="90.0", description="Nominal range interval per pair in seconds"),
        DeclareLaunchArgument("range_jitter_fraction", default_value="0.5"),
        DeclareLaunchArgument("seed", default_value="5"),
        DeclareLaunchArgument("rviz_enable", default_value="false"),
        DeclareLaunchArgument("rviz_config_path", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
