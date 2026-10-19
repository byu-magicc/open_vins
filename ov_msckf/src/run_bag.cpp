/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "core/VioManager.h"
#include "ros/BagPlayer.h"
#include "ros/ROS2Visualizer.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

using namespace ov_msckf;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("run_bag", options);

    std::string config_path;
    std::string bag_path;
    if (!node->get_parameter("config_path", config_path) || !node->get_parameter("bag_path", bag_path) || bag_path.empty())
      throw std::runtime_error("config_path and bag_path are required");

    auto parser = std::make_shared<ov_core::YamlParser>(config_path);
    parser->set_node(node);
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);

    VioManagerOptions params;
    params.print_and_load(parser);
    node->get_parameter("add_initial_error", params.add_initial_error);
    std::vector<double> initial_state_imu;
    const bool truth_initialized = node->get_parameter("initial_state_imu", initial_state_imu);
    if (truth_initialized) {
      if (initial_state_imu.size() != 17 ||
          !std::all_of(initial_state_imu.begin(), initial_state_imu.end(), [](double value) { return std::isfinite(value); }))
        throw std::runtime_error("initial_state_imu must contain 17 finite values");
      const Eigen::Map<const Eigen::Vector4d> quaternion(initial_state_imu.data() + 1);
      if (std::abs(quaternion.norm() - 1.0) > 1e-6)
        throw std::runtime_error("initial_state_imu quaternion must have unit norm");
    }
    params.set_results_namespace(node->get_namespace());
    // Bag measurements are dispatched directly, without subscription workers.
    params.use_multi_threading_subs = false;
    double bag_start = 0.0;
    double bag_duration = -1.0;
    node->get_parameter("bag_start", bag_start);
    node->get_parameter("bag_duration", bag_duration);

    BagPlayer bag(bag_path, params, *parser, *node, bag_start, bag_duration, truth_initialized);
    if (truth_initialized && std::abs(initial_state_imu[0] - bag.start_time()) > 1e-6)
      throw std::runtime_error("initial_state_imu timestamp must match the first IMU timestamp plus bag_start");

    auto sys = std::make_shared<VioManager>(params);
    if (truth_initialized) {
      Eigen::Matrix<double, 17, 1> initial = Eigen::Map<const Eigen::Matrix<double, 17, 1>>(initial_state_imu.data());
      initial(0) -= params.calib_camimu_dt;
      sys->initialize_with_gt(initial);
      PRINT_INFO("[BAG]: Truth initialization offset %.9f, IMU %.9f, camera %.9f\n", bag_start, initial_state_imu[0], initial(0));
      PRINT_INFO("[BAG]: Global initial position %.9f %.9f %.9f, velocity %.9f %.9f %.9f\n", initial(5), initial(6), initial(7), initial(8),
                 initial(9), initial(10));
    }
    bool visualize = false;
    node->get_parameter("visualize", visualize);
    bool save_total_state = false;
    node->get_parameter("save_total_state", save_total_state);
    std::string path_gt;
    node->get_parameter("path_gt", path_gt);
    std::shared_ptr<ROS2Visualizer> viz;
    if (visualize || save_total_state || !path_gt.empty())
      viz = std::make_shared<ROS2Visualizer>(node, sys);

    while (rclcpp::ok() && bag.process_next_camera(*sys, viz.get())) {
      if (viz)
        viz->visualize();
    }
    bag.finish(*sys, viz.get());
    if (viz)
      viz->visualize_final();
    viz.reset();
    sys.reset();
    parser.reset();
    node.reset();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("run_bag"), "%s", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
}
