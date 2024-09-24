from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="false", description="enable rviz node"
    ),

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
        name="fej",
        default_value="true"
    ),
    DeclareLaunchArgument(
        name="feat_rep",
        default_value="GLOBAL_3D"
    ),
    DeclareLaunchArgument(
        name="num_clones",
        default_value="11"
    ),
    DeclareLaunchArgument(
        name="num_slam",
        default_value="50"
    ),
    DeclareLaunchArgument(
        name="num_pts",
        default_value="100"
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
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
        name="sim_do_calibration",
        default_value="true"
    ),
    DeclareLaunchArgument(
        name="sim_do_calib_imu_intrinsics",
        default_value="true"
    ),
    DeclareLaunchArgument(
        name="sim_do_calib_g_sensitivity",
        default_value="true"
    ),

    DeclareLaunchArgument(
        name="dosave_pose",
        default_value="true"
    ),
    DeclareLaunchArgument(
        name="path_est",
        default_value=""
    ),
    DeclareLaunchArgument(
        name="path_gt",
        default_value=""
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

    # Get path_est filepath
    path_est = LaunchConfiguration("path_est").perform(context)
    if not path_est:
        path_est = os.path.join(
            get_package_share_directory("ov_eval"),
            "data",
            "sim",
            "traj_estimate.txt"
        )
    else:
        if not os.path.isfile(path_est):
            return [
                LogInfo(
                    msg="ERROR: path_est file: '{}' - does not exist. - not starting OpenVINS simulation".format(
                        path_est)
                    )
            ]

    # Get path_gt filepath
    path_gt = LaunchConfiguration("path_gt").perform(context)
    if not path_gt:
        path_gt = os.path.join(
            get_package_share_directory("ov_eval"),
            "data",
            "sim",
            "traj_groundtruth.txt"
        )
    else:
        if not os.path.isfile(path_gt):
            return [
                LogInfo(
                    msg="ERROR: path_gt file: '{}' - does not exist. - not starting OpenVINS simulation".format(
                        path_gt)
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
    else:
        if not os.path.isfile(path_state_est):
            return [
                LogInfo(
                    msg="ERROR: path_state_est file: '{}' - does not exist. - not starting OpenVINS simulation".format(
                        path_state_est)
                    )
            ]

    # Get path_state_std filepath
    path_state_std = LaunchConfiguration("path_state_std").perform(context)
    if not path_state_std:
        path_state_std = os.path.join(
            get_package_share_directory("ov_eval"),
            "data",
            "sim",
            "state_deviation.txt"
        )
    else:
        if not os.path.isfile(path_state_std):
            return [
                LogInfo(
                    msg="ERROR: path_state_std file: '{}' - does not exist. - not starting OpenVINS simulation".format(
                        path_state_std)
                    )
            ]

    # Get path_state_gt filepath
    path_state_gt = LaunchConfiguration("path_state_gt").perform(context)
    if not path_state_gt:
        path_state_gt = os.path.join(
            get_package_share_directory("ov_eval"),
            "data",
            "sim",
            "state_groundtruth.txt"
        )
    else:
        if not os.path.isfile(path_state_gt):
            return [
                LogInfo(
                    msg="ERROR: path_state_gt file: '{}' - does not exist. - not starting OpenVINS simulation".format(
                        path_state_gt)
                    )
            ]

    master_node = Node(
        name="ov_msckf",
        package="ov_msckf",
        executable="run_simulation",
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        on_exit=Shutdown(),
        parameters=[
            {"sim_traj_path":
                os.path.join(
                    get_package_share_directory("ov_data"),
                    "sim",
                    LaunchConfiguration("dataset").perform(context)
                )
            },
            {"sim_seed_state_init": 0},
            {"sim_seed_measurements": LaunchConfiguration("seed")},
            {"sim_seed_preturb": LaunchConfiguration("seed")},
            {"sim_freq_cam": LaunchConfiguration("freq_cam")},
            {"sim_freq_imu": LaunchConfiguration("freq_imu")},
            {"sim_do_perturbation": LaunchConfiguration("sim_do_perturbation")},

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

            {"use_fej": LaunchConfiguration("fej")},
            {"calib_cam_extrinsics": LaunchConfiguration("sim_do_calibration")},
            {"calib_cam_intrinsics": LaunchConfiguration("sim_do_calibration")},
            {"calib_cam_timeoffset": LaunchConfiguration("sim_do_calibration")},
            {"calib_imu_intrinsics": LaunchConfiguration("sim_do_calib_imu_intrinsics")},
            {"calib_imu_g_sensitivity": LaunchConfiguration("sim_do_calib_g_sensitivity")},
            {"max_clones": LaunchConfiguration("num_clones")},
            {"max_slam": LaunchConfiguration("num_slam")},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"feat_rep_msckf": LaunchConfiguration("feat_rep")},
            {"feat_rep_slam": LaunchConfiguration("feat_rep")},
            {"feat_rep_aruco": LaunchConfiguration("feat_rep")},

            {"num_pts": LaunchConfiguration("num_pts")},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
            ),
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
