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

#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "core/VioManager.h"
#include "ros/FleetRunner.h"
#include "ros/ROS2Visualizer.h"
#include "sim/Simulator.h"
#include "state/State.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

struct SimulatedAgent : FleetAgent {
  std::shared_ptr<Simulator> sim;
  double camera_time = -1;
  std::vector<int> camera_ids;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> camera_features;

  void process_camera() {
    if (camera_time == -1)
      return;
    sys->feed_measurement_simulation(camera_time, camera_ids, camera_features);
    processed_time = sys->get_state()->_timestamp;
    if (processed_time == camera_time) {
      if (!sim->get_state(camera_time + camera_offset, groundtruth))
        throw std::runtime_error("Camera epoch has no simulation truth");
      processed = true;
    }
    camera_time = -1;
  }
  bool step() override {
    if (!sim->ok())
      return false;
    while (sim->ok() && rclcpp::ok()) {
      ov_core::ImuData imu;
      if (sim->get_next_imu(imu.timestamp, imu.wm, imu.am))
        sys->feed_measurement_imu(imu);
      double timestamp;
      std::vector<int> ids;
      std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> features;
      if (sim->get_next_cam(timestamp, ids, features)) {
        process_camera();
        camera_time = timestamp;
        camera_ids = std::move(ids);
        camera_features = std::move(features);
        return true;
      }
    }
    process_camera();
    return processed;
  }
  void finish() override { PRINT_INFO("Completed at elapsed %.6f s\n", sim->current_timestamp() - start_time); }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("run_multi_agent_simulation", options);
  ov_core::Printer::setThreadLabel("multi_agent");
  std::string active_agent = "multi_agent";
  try {
    const auto names = node->get_parameter("agent_names").as_string_array();
    const auto paths = node->get_parameter("trajectory_paths").as_string_array();
    const auto config_path = node->get_parameter("config_path").as_string();
    bool visualize = false;
    node->get_parameter("visualize", visualize);
    if (names.empty() || names.size() != paths.size())
      throw std::invalid_argument("agent_names and datasets must have the same nonzero length");
    std::set<std::string> unique_names;
    for (const auto &name : names) {
      if (name.empty() || name == "." || name == ".." || name.find('/') != std::string::npos || !unique_names.insert(name).second)
        throw std::invalid_argument("agent_names must be unique nonempty directory names without slashes");
    }

    std::vector<std::unique_ptr<FleetAgent>> agents;
    for (size_t index = 0; index < names.size(); index++) {
      active_agent = names.at(index);
      ov_core::Printer::setThreadLabel(names.at(index));
      // Reload for each agent: Simulator perturbs camera calibration in params.
      auto parser = std::make_shared<ov_core::YamlParser>(config_path);
      parser->set_node(node);
      std::string verbosity = "INFO";
      parser->parse_config("verbosity", verbosity);
      ov_core::Printer::setPrintLevel(verbosity);
      VioManagerOptions params;
      params.print_and_load(parser);
      node->get_parameter("initial_state_stddev", params.initial_state_stddev);
      node->get_parameter("add_initial_error", params.add_initial_error);
      params.print_and_load_simulation(parser);
      if (!parser->successful())
        throw std::runtime_error("Unable to parse simulation parameters");
      if (!std::isfinite(params.sim_freq_imu) || params.sim_freq_imu <= 0 || !std::isfinite(params.sim_freq_cam) ||
          params.sim_freq_cam <= 0)
        throw std::invalid_argument("Sensor frequencies must be finite and positive");
      params.sim_traj_path = paths.at(index);
      params.set_results_namespace(names.at(index));
      params.factor_graph_agent_id = index;
      params.defer_results = true;
      params.num_opencv_threads = 0;
      params.use_multi_threading_pubs = false;
      params.use_multi_threading_subs = false;

      auto agent = std::make_unique<SimulatedAgent>();
      agent->name = names.at(index);
      agent->sim = std::make_shared<Simulator>(params);
      agent->sys = std::make_shared<VioManager>(params);
      agent->start_time = agent->sim->current_timestamp();
      agent->camera_offset = agent->sim->get_true_parameters().calib_camimu_dt;
      Eigen::Matrix<double, 17, 1> initial_state;
      if (!agent->sim->get_state(agent->start_time + 1.0 / params.sim_freq_imu, initial_state))
        throw std::runtime_error("Could not initialize " + agent->name);
      initial_state(0) -= agent->camera_offset;
      agent->sys->initialize_with_gt(initial_state);
      if (visualize)
        agent->viz = std::make_shared<ROS2Visualizer>(node->create_sub_node(agent->name), agent->sys, agent->sim);
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
