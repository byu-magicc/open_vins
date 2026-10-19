import math
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


sys.path.insert(0, os.path.dirname(__file__))
from openvins_launch import extract_truth, initialization_arguments

def launch_setup(context):
    arguments = {
        "bag_path": LaunchConfiguration("bag_path"),
        "initial_state_covariance": LaunchConfiguration("initial_state_covariance"),
        "initial_velocity_uncertainty_in_local_frame": LaunchConfiguration("initial_velocity_uncertainty_in_local_frame"),
        "add_initial_error": LaunchConfiguration("add_initial_error"),
        "relinearize_skip": LaunchConfiguration("relinearize_skip"),
        "config": "holoocean_fixedwing",
        "topic_imu": "/imu/data",
        "camera_topics": "/fixedwing/camera/image_raw",
    }
    start = LaunchConfiguration("start_time").perform(context)
    if start:
        offset = float(start)
        if not math.isfinite(offset) or offset < 0:
            raise RuntimeError("start_time must be finite and nonnegative")
        initial_state = extract_truth(LaunchConfiguration("bag_path").perform(context), [start])
        arguments.update({"bag_start": start, "initial_state_imu": initial_state})
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), "bag.launch.py")),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        *initialization_arguments("true"),
        DeclareLaunchArgument("bag_path", description="ROS 2 HoloOcean bag directory or file"),
        DeclareLaunchArgument(
            "start_time", default_value="",
            description="Optional truth start in seconds from the first IMU header; empty uses normal initialization",
        ),
        DeclareLaunchArgument(
            "relinearize_skip", default_value="1", description="number of iSAM2 updates between relinearization checks",
        ),
        OpaqueFunction(function=launch_setup),
    ])
