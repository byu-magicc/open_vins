import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Shared arguments are declared by bag.launch.py and inherit CLI overrides.
    return LaunchDescription([
        DeclareLaunchArgument(
            "relinearize_skip", default_value="1", description="number of iSAM2 updates between relinearization checks",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), "bag.launch.py")),
            launch_arguments={
                # Includes require explicitly forwarding arguments without defaults.
                "bag_path": LaunchConfiguration("bag_path"),
                "relinearize_skip": LaunchConfiguration("relinearize_skip"),
                "config": "holoocean_fixedwing",
                "topic_imu": "/imu/data",
                "camera_topics": "/fixedwing/camera/image_raw",
            }.items(),
        ),
    ])
