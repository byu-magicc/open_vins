from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def finish_playback(event, context):
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        raise RuntimeError(f"Bag playback failed with exit code {event.returncode}")
    # Allow queued camera updates to finish before shutting down the estimator.
    return [TimerAction(period=5.0, actions=[Shutdown(reason="Bag playback finished")])]


def bag_playback(context):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    if not bag_path:
        return []
    return [ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", bag_path,
            "--rate", LaunchConfiguration("bag_rate"),
            "--clock", "--delay", "3", "--disable-keyboard-controls",
            "--topics", "/imu/data", "/fixedwing/camera/image_raw",
        ],
        output="screen",
        on_exit=finish_playback,
    )]


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
        DeclareLaunchArgument("save_results", default_value="false"),
        DeclareLaunchArgument("results_path", default_value="results"),
        DeclareLaunchArgument("filter_type", default_value="openvins", choices=["openvins", "factor_graph", "hybrid"]),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("bag_path", default_value="", description="optional HoloOcean bag to play and then stop"),
        DeclareLaunchArgument("bag_rate", default_value="1.0", description="bag playback speed"),
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
                "save_results": LaunchConfiguration("save_results"),
                "results_path": LaunchConfiguration("results_path"),
                "filter_type": LaunchConfiguration("filter_type"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        OpaqueFunction(function=bag_playback),
    ])
