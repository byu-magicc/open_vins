from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_enable",
            default_value="false",
            description="Start RViz with the single-agent OpenVINS display",
        ),
        DeclareLaunchArgument(
            "verbosity",
            default_value="INFO",
            description="OpenVINS logging level",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ThisLaunchFileDir(),
                "/subscribe.launch.py",
            ]),
            launch_arguments={
                "namespace": "ov_msckf",
                "config": "holoocean_fixedwing",
                "max_cameras": "1",
                "use_stereo": "false",
                "rviz_enable": LaunchConfiguration("rviz_enable"),
                "verbosity": LaunchConfiguration("verbosity"),
            }.items(),
        ),
    ])
