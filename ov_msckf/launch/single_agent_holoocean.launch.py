import json
import math
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    arguments = {
        "bag_path": LaunchConfiguration("bag_path"),
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
        utility = os.path.join(get_package_share_directory("ov_msckf"), "scripts", "holoocean_truth.py")
        result = subprocess.run(
            [os.environ.get("PLOTTER_PYTHON", sys.executable), utility,
             LaunchConfiguration("bag_path").perform(context), start],
            capture_output=True, text=True, check=False,
        )
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        if result.returncode:
            raise RuntimeError("Unable to extract HoloOcean truth; see the extraction error above")
        # Ensure extraction output is JSON before passing it through the include.
        json.loads(result.stdout)
        arguments.update({"bag_start": start, "initial_state_imu": result.stdout.strip()})
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), "bag.launch.py")),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
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
