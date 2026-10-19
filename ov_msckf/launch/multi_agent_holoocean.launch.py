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
                    "truth_paths": ",".join(truth_paths),
                    "config": "holoocean_fixedwing",
                    "topic_imu": "/imu/data",
                    "camera_topics": "/fixedwing/camera/image_raw",
                    "add_initial_error": LaunchConfiguration("add_initial_error"),
                    "initial_state_stddev": LaunchConfiguration("initial_state_stddev"),
                }.items(),
            ),
        ]
    except BaseException:
        shutil.rmtree(temporary_directory, ignore_errors=True)
        raise


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag_paths", description="Comma-separated HoloOcean ROS 2 bag directories or files"),
        DeclareLaunchArgument("add_initial_error", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "initial_state_stddev",
            default_value="[0.017, 0.017, 0.017, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02]",
            description="HoloOcean IMU initialization standard deviations: JSON array [theta_xyz, p_xyz, v_xyz, bg_xyz, ba_xyz]",
        ),
        OpaqueFunction(function=launch_setup),
    ])
