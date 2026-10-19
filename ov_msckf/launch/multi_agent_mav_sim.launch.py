import os
import re
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


sys.path.insert(0, os.path.dirname(__file__))
from openvins_launch import initialization_arguments, rviz_configuration

def launch_setup(context):
    namespace = "multi_agent"
    agent_names = [
        name.strip()
        for name in LaunchConfiguration("agent_names").perform(context).split(",")
    ]
    if not agent_names or any(not name for name in agent_names):
        raise RuntimeError("agent_names must contain nonempty comma-separated names")

    rviz_config_path = ""
    actions = []
    if IfCondition(LaunchConfiguration("rviz_enable")).evaluate(context):
        invalid_names = [
            name for name in agent_names if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)
        ]
        if invalid_names:
            raise RuntimeError(
                "RViz requires agent_names to be valid ROS namespace tokens: "
                + ", ".join(invalid_names)
            )
        rviz_config = rviz_configuration(agent_names, namespace)

        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix="openvins_multi_agent_",
            suffix=".rviz",
            delete=False,
        ) as rviz_config_file:
            rviz_config_file.write(rviz_config)
            rviz_config_path = rviz_config_file.name

        def remove_rviz_config(event, launch_context):
            del event, launch_context
            try:
                os.unlink(rviz_config_path)
            except FileNotFoundError:
                pass

        actions.append(
            RegisterEventHandler(OnShutdown(on_shutdown=remove_rviz_config))
        )

    actions.append(SetLaunchConfiguration("rviz_config_path", rviz_config_path))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            *initialization_arguments("true"),
            # These are needed by launch_setup before the included declarations run.
            DeclareLaunchArgument("agent_names", default_value="center,left,right"),
            DeclareLaunchArgument("rviz_enable", default_value="false"),
            DeclareLaunchArgument("range_interval", default_value="45.0"),
            DeclareLaunchArgument("range_jitter_fraction", default_value="0.4"),
            DeclareLaunchArgument(
                "datasets",
                default_value="gazebo_sinusoid/center_trajectory.txt,"
                "gazebo_sinusoid/left_trajectory.txt,"
                "gazebo_sinusoid/right_trajectory.txt",
            ),
            OpaqueFunction(function=launch_setup),
            # Shared arguments are declared by simulation.launch.py and inherit CLI overrides.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), "simulation.launch.py")),
                launch_arguments={
                    "initial_state_stddev": LaunchConfiguration("initial_state_stddev"),
                    "add_initial_error": LaunchConfiguration("add_initial_error"),
                    "simulation_executable": "run_multi_agent_simulation",
                    "namespace": "multi_agent",
                    "config": "magicc_fixedwing_sim",
                    "use_ground_plane_features": "true",
                    "ground_plane_features_range": "2.0",
                    "dosave_state": "false",
                }.items(),
            ),
        ]
    )
