"""Shared estimator parameters, truth extraction, and RViz generation for ROS 2 launches."""
import json
import math
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def initialization_arguments(add_initial_error):
    return [
        DeclareLaunchArgument(
            "initial_state_stddev",
            default_value="[0.017, 0.017, 0.017, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02]",
            description="IMU initialization standard deviations: JSON array [theta_xyz (rad), p_xyz (m), v_xyz (m/s), "
                        "bg_xyz (rad/s), ba_xyz (m/s^2)], shared by all agents",
        ),
        DeclareLaunchArgument(
            "add_initial_error", default_value=add_initial_error, choices=["true", "false"],
            description="Add a zero-mean Gaussian error matching the configured initial IMU covariance",
        ),
    ]


def initialization_parameters(context):
    values = json.loads(LaunchConfiguration("initial_state_stddev").perform(context))
    if (not isinstance(values, list) or len(values) != 15 or
            any(type(value) not in (int, float) or not math.isfinite(value) or value < 0 for value in values)):
        raise RuntimeError("initial_state_stddev must be a JSON array of 15 finite nonnegative numbers")
    return {
        "initial_state_stddev": [float(value) for value in values],
        "add_initial_error": ParameterValue(LaunchConfiguration("add_initial_error"), value_type=bool),
    }


def graph_arguments(relinearize_skip):
    return [
        DeclareLaunchArgument("filter_type", default_value="openvins", choices=["openvins", "factor_graph", "hybrid"]),
        DeclareLaunchArgument("relinearize_skip", default_value=str(relinearize_skip)),
        DeclareLaunchArgument("relinearize_threshold", default_value="0.1"),
        DeclareLaunchArgument("use_qr", default_value="false"),
        DeclareLaunchArgument("save_results", default_value="false"),
        DeclareLaunchArgument("results_path", default_value="results"),
    ]


def graph_parameters():
    return {name: ParameterValue(LaunchConfiguration(name), value_type=kind) for name, kind in (
        ("filter_type", str), ("relinearize_skip", int), ("relinearize_threshold", float),
        ("use_qr", bool), ("save_results", bool), ("results_path", str),
    )}


def camera_parameters(context):
    parameters = {}
    topics = LaunchConfiguration("camera_topics").perform(context)
    if topics:
        parameters["camera_topics"] = [topic.strip() for topic in topics.split(",")]
    for index in range(2):
        name = "topic_camera" + str(index)
        topic = LaunchConfiguration(name).perform(context)
        if topic:
            parameters[name] = topic
    return parameters


def rviz_configuration(names, namespace):
    path = os.path.join(get_package_share_directory("ov_msckf"), "launch", "display_multi_agent_ros2.rviz")
    with open(path, encoding="utf-8") as stream:
        before, start, template = stream.read().partition("# BEGIN AGENT TEMPLATE\n")
    template, end, after = template.partition("# END AGENT TEMPLATE\n")
    if not start or not end:
        raise RuntimeError("Multi-agent RViz template is missing its markers")
    displays = "".join(template.replace("{namespace}", namespace.strip("/")).replace("{agent}", name) for name in names)
    return before + displays + after


def extract_truth(bag_path, arguments):
    utility = os.path.join(get_package_share_directory("ov_msckf"), "scripts", "holoocean_truth.py")
    result = subprocess.run([os.environ.get("PLOTTER_PYTHON", sys.executable), utility, bag_path, *arguments],
                            capture_output=True, text=True, check=False)
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise RuntimeError(f"Unable to extract HoloOcean truth from {bag_path}; see the extraction error above")
    return result.stdout.strip()
