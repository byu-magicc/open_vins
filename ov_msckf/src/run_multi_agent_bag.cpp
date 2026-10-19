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
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "core/VioManager.h"
#include "ros/BagPlayer.h"
#include "ros/BagTruth.h"
#include "ros/FleetRunner.h"
#include "ros/ROS2Visualizer.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

using namespace ov_msckf;

struct BagAgent : FleetAgent {
  std::unique_ptr<BagPlayer> bag;
  std::unique_ptr<BagTruth> truth;

  bool step() override {
    if (!bag->process_next_camera(*sys, viz.get()))
      return false;
    processed_time = sys->get_state()->_timestamp;
    if (processed_time == bag->camera_time()) {
      groundtruth = truth->get_state(processed_time + camera_offset);
      processed = true;
    }
    return true;
  }
  void finish() override { bag->finish(*sys, viz.get()); }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("run_multi_agent_bag", options);
  ov_core::Printer::setThreadLabel("multi_agent");
  std::string active_agent = "multi_agent";
  try {
    const auto names = node->get_parameter("agent_names").as_string_array();
    const auto paths = node->get_parameter("bag_paths").as_string_array();
    const auto truth_paths = node->get_parameter("truth_paths").as_string_array();
    const auto config_path = node->get_parameter("config_path").as_string();
    if (names.empty() || names.size() != paths.size() || names.size() != truth_paths.size())
      throw std::invalid_argument("agent_names, bag_paths, and truth_paths must have the same nonzero length");
    std::set<std::string> unique_names;
    const std::regex namespace_token("[A-Za-z_][A-Za-z0-9_]*");
    for (const auto &name : names) {
      if (!std::regex_match(name, namespace_token) || !unique_names.insert(name).second)
        throw std::invalid_argument("agent_names must be unique ROS namespace tokens");
    }
    std::vector<double> starts(names.size(), 0.0);
    node->get_parameter("bag_starts", starts);
    if (starts.size() != names.size())
      throw std::invalid_argument("bag_starts must contain one offset per agent");
    double duration = -1.0;
    node->get_parameter("bag_duration", duration);
    if (!std::isfinite(duration) || (duration != -1.0 && duration <= 0.0) ||
        !std::all_of(starts.begin(), starts.end(), [](double start) { return std::isfinite(start) && start >= 0.0; }))
      throw std::invalid_argument("bag_starts must be finite and nonnegative; bag_duration must be positive or -1");
    bool visualize = false;
    node->get_parameter("visualize", visualize);
    std::vector<std::unique_ptr<FleetAgent>> agents;
    for (size_t index = 0; index < names.size(); ++index) {
      active_agent = names[index];
      ov_core::Printer::setThreadLabel(active_agent);
      auto parser = std::make_shared<ov_core::YamlParser>(config_path);
      parser->set_node(node);
      std::string verbosity = "INFO";
      parser->parse_config("verbosity", verbosity);
      ov_core::Printer::setPrintLevel(verbosity);
      VioManagerOptions params;
      params.print_and_load(parser);
      node->get_parameter("initial_state_stddev", params.initial_state_stddev);
      node->get_parameter("add_initial_error", params.add_initial_error);
      params.set_results_namespace(names[index]);
      params.factor_graph_agent_id = index;
      params.defer_results = true;
      params.num_opencv_threads = 0;
      params.use_multi_threading_pubs = false;
      params.use_multi_threading_subs = false;

      auto agent = std::make_unique<BagAgent>();
      agent->name = names[index];
      agent->bag = std::make_unique<BagPlayer>(paths[index], params, *parser, *node, starts[index], duration, true);
      agent->truth = std::make_unique<BagTruth>(truth_paths[index]);
      agent->start_time = agent->bag->start_time();
      agent->camera_offset = params.calib_camimu_dt;
      Eigen::Matrix<double, 17, 1> initial_state = agent->truth->get_state(agent->start_time);
      initial_state(0) -= agent->camera_offset;
      agent->sys = std::make_shared<VioManager>(params);
      agent->sys->initialize_with_gt(initial_state);
      PRINT_INFO("[BAG]: Truth initialization offset %.9f, IMU %.9f, camera %.9f\n", starts[index], agent->start_time, initial_state(0));
      PRINT_INFO("[BAG]: Global initial position %.9f %.9f %.9f, velocity %.9f %.9f %.9f\n", initial_state(5), initial_state(6),
                 initial_state(7), initial_state(8), initial_state(9), initial_state(10));
      if (visualize)
        agent->viz = std::make_shared<ROS2Visualizer>(node->create_sub_node(agent->name), agent->sys);
      agents.push_back(std::move(agent));
    }
    ov_core::Printer::setThreadLabel("multi_agent");
    active_agent = "multi_agent";

    run_fleet(node, agents);
    agents.clear();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "[%s] %s", active_agent.c_str(), error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
