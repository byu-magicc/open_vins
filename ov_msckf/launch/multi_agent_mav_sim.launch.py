import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
        template_path = os.path.join(
            get_package_share_directory("ov_msckf"),
            "launch",
            "display_multi_agent_ros2.rviz",
        )
        with open(template_path, encoding="utf-8") as template_file:
            rviz_template = template_file.read()
        before_agents, separator, agent_template = rviz_template.partition(
            "# BEGIN AGENT TEMPLATE\n"
        )
        if not separator:
            raise RuntimeError("Multi-agent RViz template is missing its start marker")
        agent_template, separator, after_agents = agent_template.partition(
            "# END AGENT TEMPLATE\n"
        )
        if not separator:
            raise RuntimeError("Multi-agent RViz template is missing its end marker")

        agent_displays = "".join(
            agent_template.replace("{namespace}", namespace).replace(
                "{agent}", agent_name
            )
            for agent_name in agent_names
        )
        rviz_config = before_agents + agent_displays + after_agents

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

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ov_msckf"),
                    "launch",
                    "simulation.launch.py",
                )
            ),
            launch_arguments={
                "simulation_executable": "run_multi_agent_simulation",
                "namespace": namespace,
                "rviz_enable": LaunchConfiguration("rviz_enable"),
                "rviz_config_path": rviz_config_path,
                "agent_names": LaunchConfiguration("agent_names"),
                "datasets": LaunchConfiguration("datasets"),
                "config": "magicc_fixedwing_sim",
                "max_cameras": "1",
                "use_stereo": "false",
                "use_factor_graph": "true",
                "range_stddev": LaunchConfiguration("range_stddev"),
                "range_probability": LaunchConfiguration("range_probability"),
                "range_seed": LaunchConfiguration("range_seed"),
                "relinearize_skip": LaunchConfiguration("relinearize_skip"),
                "relinearize_threshold": LaunchConfiguration(
                    "relinearize_threshold"
                ),
                "use_qr": LaunchConfiguration("use_qr"),
                "save_results": LaunchConfiguration("save_results"),
                "results_path": LaunchConfiguration("results_path"),
                "use_ground_plane_features": "true",
                "ground_plane_features_range": "2.0",
                "dosave_state": "false",
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_names", default_value="center,left,right"),
            DeclareLaunchArgument("range_stddev", default_value="1.0"),
            DeclareLaunchArgument("range_probability", default_value="0.006"),
            DeclareLaunchArgument("range_seed", default_value="5"),
            DeclareLaunchArgument("rviz_enable", default_value="false"),
            DeclareLaunchArgument("save_results", default_value="false"),
            DeclareLaunchArgument("results_path", default_value="results"),
            DeclareLaunchArgument("relinearize_skip", default_value="10"),
            DeclareLaunchArgument("relinearize_threshold", default_value="0.1"),
            DeclareLaunchArgument("use_qr", default_value="false"),
            DeclareLaunchArgument(
                "datasets",
                default_value="gazebo_sinusoid/center_trajectory.txt,"
                "gazebo_sinusoid/left_trajectory.txt,"
                "gazebo_sinusoid/right_trajectory.txt",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
