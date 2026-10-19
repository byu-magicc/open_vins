from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from openvins_launch import graph_arguments, graph_parameters, initialization_arguments, initialization_parameters

launch_args = [
    *graph_arguments(10),
    *initialization_arguments("true"),
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="false", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="rviz_config_path",
        default_value="",
        description="RViz configuration file; defaults to the single-agent configuration",
    ),
    DeclareLaunchArgument(name="simulation_executable", default_value="run_simulation"),
    DeclareLaunchArgument(name="agent_names", default_value="center,left,right"),
    DeclareLaunchArgument(name="datasets", default_value=""),
    DeclareLaunchArgument(name="range_stddev", default_value="1.0", description="Range noise standard deviation in meters"),
    DeclareLaunchArgument(name="range_interval", default_value="15.0", description="Nominal range interval per pair in seconds"),
    DeclareLaunchArgument(name="range_jitter_fraction", default_value="0.6", description="Uniform fractional jitter on range intervals"),

    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="rpng_sim",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),

    DeclareLaunchArgument(
        name="seed",
        default_value="5"
    ),
    DeclareLaunchArgument(
        name="num_pts",
        default_value="100"
    ),
    DeclareLaunchArgument(
        name="feat_dist_min",
        default_value="5.0"
    ),
    DeclareLaunchArgument(
        name="feat_dist_max",
        default_value="7.0"
    ),
    DeclareLaunchArgument(
        name="use_ground_plane_features",
        default_value="false"
    ),
    DeclareLaunchArgument(
        name="ground_plane_features_range",
        default_value="2.0"
    ),

    DeclareLaunchArgument(
        name="freq_cam",
        default_value="10.0"
    ),
    DeclareLaunchArgument(
        name="freq_imu",
        default_value="400.0"
    ),
    DeclareLaunchArgument(
        name="dataset",
        default_value="tum_corridor1_512_16_okvis.txt"
    ),

    DeclareLaunchArgument(
        name="sim_do_perturbation",
        default_value="true"
    ),

    DeclareLaunchArgument(
        name="dosave_state",
        default_value="true"
    ),
    DeclareLaunchArgument(
        name="path_state_est",
        default_value=""
    ),
    DeclareLaunchArgument(
        name="path_state_std",
        default_value=""
    ),
    DeclareLaunchArgument(
        name="path_state_gt",
        default_value=""
    ),
]

def launch_setup(context):
    # Get config filepath
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        available_configs = os.listdir(configs_dir)
        config = LaunchConfiguration("config").perform(context)
        if config in available_configs:
            config_path = os.path.join(
                    get_package_share_directory("ov_msckf"),
                    "config",
                    config,
                    "estimator_config.yaml"
                )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                        config_path)
                    )
            ]

    # Get path_state_est filepath
    path_state_est = LaunchConfiguration("path_state_est").perform(context)
    if not path_state_est:
        path_state_est = os.path.join(
            get_package_share_directory("ov_eval"),
            "data",
            "sim",
            "state_estimate.txt"
        )

    # Get path_state_std filepath
    path_state_std = LaunchConfiguration("path_state_std").perform(context)
    if not path_state_std:
        path_state_std = os.path.join(
            get_package_share_directory("ov_eval"),
            "data",
            "sim",
            "state_deviation.txt"
        )

    # Get path_state_gt filepath
    path_state_gt = LaunchConfiguration("path_state_gt").perform(context)
    if not path_state_gt:
        path_state_gt = os.path.join(
            get_package_share_directory("ov_eval"),
            "data",
            "sim",
            "state_groundtruth.txt"
        )

    master_node = Node(
        name="ov_msckf",
        package="ov_msckf",
        executable=LaunchConfiguration("simulation_executable"),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        on_exit=Shutdown(),
        parameters=[
            graph_parameters(),
            initialization_parameters(context),
            {"sim_traj_path":
                os.path.join(
                    get_package_share_directory("ov_data"),
                    "sim",
                    LaunchConfiguration("dataset").perform(context)
                )
            },
            {"sim_seed_state_init": 0},
            {"range_stddev": ParameterValue(LaunchConfiguration("range_stddev"), value_type=float)},
            {"range_interval": ParameterValue(LaunchConfiguration("range_interval"), value_type=float)},
            {"range_jitter_fraction": ParameterValue(LaunchConfiguration("range_jitter_fraction"), value_type=float)},
            {"sim_seed_measurements": LaunchConfiguration("seed")},
            {"sim_seed_preturb": LaunchConfiguration("seed")},
            {"sim_freq_cam": LaunchConfiguration("freq_cam")},
            {"sim_freq_imu": LaunchConfiguration("freq_imu")},
            {"sim_do_perturbation": LaunchConfiguration("sim_do_perturbation")},
            {"agent_names": [
                name.strip()
                for name in LaunchConfiguration("agent_names").perform(context).split(",")
            ]},
            {"trajectory_paths": [
                os.path.join(get_package_share_directory("ov_data"), "sim", dataset.strip())
                for dataset in (LaunchConfiguration("datasets").perform(context)
                                or LaunchConfiguration("dataset").perform(context)).split(",")
            ]},

            {"sim_min_feature_gen_dist": LaunchConfiguration("feat_dist_min")},
            {"sim_max_feature_gen_dist": LaunchConfiguration("feat_dist_max")},
            {"sim_use_ground_plane_features": LaunchConfiguration("use_ground_plane_features")},
            {"sim_ground_plane_features_range": LaunchConfiguration("ground_plane_features_range")},

            {"save_total_state": LaunchConfiguration("dosave_state")},
            {"filepath_est": path_state_est},
            {"filepath_std": path_state_std},
            {"filepath_gt": path_state_gt},

            {"verbosity": LaunchConfiguration("verbosity")},
            {"config_path": config_path},
            {"num_opencv_threads": 0},
            {"num_pts": LaunchConfiguration("num_pts")},
            {"visualize": LaunchConfiguration("rviz_enable")},
        ],
    )

    rviz_config_path = LaunchConfiguration("rviz_config_path").perform(context)
    if not rviz_config_path:
        rviz_config_path = os.path.join(
            get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
        )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        # The saved RViz displays use /ov_msckf; follow the simulation namespace.
        remappings=[
            ("/ov_msckf/" + topic,
             os.path.join("/", LaunchConfiguration("namespace").perform(context).strip("/"), topic))
            for topic in [
                "trackhist", "loop_depth_colored", "pathimu", "pathgt",
                "points_msckf", "points_slam", "points_aruco", "loop_feats", "points_sim",
            ]
        ],
        arguments=[
            "-d",
            rviz_config_path,
            "--ros-args",
            "--log-level",
            "warn",
            ],
    )

    return [master_node, rviz_node]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
