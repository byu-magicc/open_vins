import json
import os
import shutil
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from openvins_launch import extract_truth


def launch_setup(context):
    bags = [path.strip() for path in LaunchConfiguration("bag_paths").perform(context).split(",")]
    for bag in bags:
        if not bag or not os.path.exists(bag):
            raise RuntimeError(f"Bag path does not exist: {bag}")

    temporary_directory = tempfile.mkdtemp(prefix="openvins_holoocean_truth_")
    try:
        truth_paths = []
        for index, bag in enumerate(bags):
            truth_path = os.path.join(temporary_directory, str(index))
            extract_truth(bag, ["--export-dir", truth_path, "--topic-imu", "/imu/data"])
            truth_paths.append(truth_path)

        def remove_temporary_files(event, launch_context):
            del event, launch_context
            shutil.rmtree(temporary_directory, ignore_errors=True)

        return [
            RegisterEventHandler(OnShutdown(on_shutdown=remove_temporary_files)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), "multi_agent_bag.launch.py")),
                launch_arguments={
                    "bag_paths": LaunchConfiguration("bag_paths"),
                    "relinearize_skip": LaunchConfiguration("relinearize_skip"),
                    "truth_paths": ",".join(truth_paths),
                    "config": "holoocean_fixedwing",
                    "topic_imu": "/imu/data",
                    "camera_topics": "/fixedwing/camera/image_raw",
                    "add_initial_error": LaunchConfiguration("add_initial_error"),
                    "initial_state_covariance": LaunchConfiguration("initial_state_covariance"),
                    "initial_velocity_uncertainty_in_local_frame": LaunchConfiguration("initial_velocity_uncertainty_in_local_frame"),
                }.items(),
            ),
        ]
    except BaseException:
        shutil.rmtree(temporary_directory, ignore_errors=True)
        raise


def generate_launch_description():
    initial_state_stddev = [0.035, 0.035, 0.17, 0.5, 0.5, 0.5, 0.05, 0.05, 0.05, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02]
    initial_state_covariance = [
        [stddev ** 2 if i == j else 0.0 for j in range(15)]
        for i, stddev in enumerate(initial_state_stddev)
    ]
    return LaunchDescription([
        DeclareLaunchArgument("bag_paths", description="Comma-separated HoloOcean ROS 2 bag directories or files"),
        DeclareLaunchArgument("relinearize_skip", default_value="1", description="Number of iSAM2 updates between relinearization checks"),
        DeclareLaunchArgument(
            "initial_velocity_uncertainty_in_local_frame", default_value="true", choices=["true", "false"],
            description="Interpret velocity uncertainty in the local IMU frame; velocity state values remain global",
        ),
        DeclareLaunchArgument("add_initial_error", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "initial_state_covariance",
            default_value=json.dumps(initial_state_covariance),
            description="HoloOcean IMU initialization covariance: JSON 15x15 matrix ordered as "
                        "[theta_xyz, p_xyz, v_xyz, bg_xyz, ba_xyz]",
        ),
        OpaqueFunction(function=launch_setup),
    ])
