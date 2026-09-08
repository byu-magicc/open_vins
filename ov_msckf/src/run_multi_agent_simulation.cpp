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

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "core/VioManager.h"
#include "sim/Simulator.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

// All estimators live in one process and synchronize at camera epochs through an in-memory barrier.
struct SimulatedAgent {
  std::string name;
  std::shared_ptr<Simulator> sim;
  std::unique_ptr<VioManager> sys;
  double start_time;
  double camera_offset;
  bool save_results;
  double camera_time = -1;
  std::vector<int> camera_ids;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> camera_features;

  void process_camera() {
    if (camera_time == -1)
      return;
    sys->feed_measurement_simulation(camera_time, camera_ids, camera_features);
    if (save_results) {
      Eigen::Matrix<double, 17, 1> groundtruth;
      if (sim->get_state(camera_time + camera_offset, groundtruth))
        sys->record_groundtruth(camera_time, groundtruth.segment<16>(1));
    }
    camera_time = -1;
  }
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
    if (names.empty() || names.size() != paths.size())
      throw std::invalid_argument("agent_names and datasets must have the same nonzero length");
    std::set<std::string> unique_names;
    for (const auto &name : names) {
      if (name.empty() || name == "." || name == ".." || name.find('/') != std::string::npos || !unique_names.insert(name).second)
        throw std::invalid_argument("agent_names must be unique nonempty directory names without slashes");
    }

    std::vector<std::unique_ptr<SimulatedAgent>> agents;
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
      params.print_and_load_simulation(parser);
      if (!parser->successful())
        throw std::runtime_error("Unable to parse simulation parameters");
      if (!std::isfinite(params.sim_freq_imu) || params.sim_freq_imu <= 0 || !std::isfinite(params.sim_freq_cam) ||
          params.sim_freq_cam <= 0)
        throw std::invalid_argument("Sensor frequencies must be finite and positive");
      params.sim_traj_path = paths.at(index);
      params.set_results_namespace(names.at(index));
      params.num_opencv_threads = 0;
      params.use_multi_threading_pubs = false;
      params.use_multi_threading_subs = false;

      auto agent = std::make_unique<SimulatedAgent>();
      agent->name = names.at(index);
      agent->sim = std::make_shared<Simulator>(params);
      agent->sys = std::make_unique<VioManager>(params);
      agent->start_time = agent->sim->current_timestamp();
      agent->camera_offset = agent->sim->get_true_parameters().calib_camimu_dt;
      agent->save_results = params.save_results;
      Eigen::Matrix<double, 17, 1> initial_state;
      if (!agent->sim->get_state(agent->start_time + 1.0 / params.sim_freq_imu, initial_state))
        throw std::runtime_error("Could not initialize " + agent->name);
      initial_state(0) -= agent->camera_offset;
      agent->sys->initialize_with_gt(initial_state);
      agents.push_back(std::move(agent));
    }
    ov_core::Printer::setThreadLabel("multi_agent");
    active_agent = "multi_agent";

    std::mutex barrier_mutex;
    std::condition_variable barrier_condition;
    size_t barrier_arrivals = 0;
    size_t barrier_generation = 0;
    bool stop = false;
    std::atomic<bool> failed(false);
    std::exception_ptr worker_error;
    std::vector<std::thread> workers;
    workers.reserve(agents.size());
    for (auto &agent_ptr : agents) {
      workers.emplace_back([&, agent = agent_ptr.get()]() {
        ov_core::Printer::setThreadLabel(agent->name);
        while (true) {
          if (agent->sim->ok() && rclcpp::ok() && !failed.load()) {
            try {
              bool reached_camera = false;
              while (agent->sim->ok() && rclcpp::ok() && !reached_camera) {
                ov_core::ImuData imu;
                if (agent->sim->get_next_imu(imu.timestamp, imu.wm, imu.am))
                  agent->sys->feed_measurement_imu(imu);

                double camera_time;
                std::vector<int> camera_ids;
                std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> features;
                if (agent->sim->get_next_cam(camera_time, camera_ids, features)) {
                  // Delay camera processing for IMU lookahead, as in run_simulation.
                  agent->process_camera();
                  agent->camera_time = camera_time;
                  agent->camera_ids = std::move(camera_ids);
                  agent->camera_features = std::move(features);
                  reached_camera = true;
                }
              }
            } catch (...) {
              PRINT_ERROR("Simulation worker failed\n");
              std::lock_guard<std::mutex> lock(barrier_mutex);
              if (!worker_error) {
                worker_error = std::current_exception();
                active_agent = agent->name;
              }
              failed.store(true);
            }
          }

          std::unique_lock<std::mutex> lock(barrier_mutex);
          const size_t generation = barrier_generation;
          barrier_arrivals++;
          if (barrier_arrivals == agents.size()) {
            stop = failed.load() || !rclcpp::ok();
            if (!stop) {
              stop = true;
              for (const auto &candidate : agents) {
                if (candidate->sim->ok()) {
                  stop = false;
                  break;
                }
              }
            }
            barrier_arrivals = 0;
            barrier_generation++;
            lock.unlock();
            barrier_condition.notify_all();
          } else {
            barrier_condition.wait(lock, [&]() { return barrier_generation != generation; });
          }
          if (stop)
            break;
        }

        if (!failed.load())
          agent->process_camera();
        agent->sys.reset();
        RCLCPP_INFO(node->get_logger(), "[%s] completed at elapsed %.6f s", agent->name.c_str(),
                    agent->sim->current_timestamp() - agent->start_time);
      });
    }
    for (auto &worker : workers)
      worker.join();
    if (worker_error)
      std::rethrow_exception(worker_error);
    agents.clear();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "[%s] %s", active_agent.c_str(), error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
