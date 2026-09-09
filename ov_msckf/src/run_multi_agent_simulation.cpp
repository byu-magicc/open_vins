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
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>

#include "core/VioManager.h"
#include "ros/ROS2Visualizer.h"
#include "sim/Simulator.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

// All estimators live in one process and synchronize at camera epochs through an in-memory barrier.
struct SimulatedAgent {
  std::string name;
  std::shared_ptr<Simulator> sim;
  std::shared_ptr<VioManager> sys;
  std::shared_ptr<ROS2Visualizer> viz;
  double start_time;
  double camera_offset;
  bool save_results;
  double camera_time = -1;
  double processed_time = -1;
  std::vector<int> camera_ids;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> camera_features;

  void process_camera() {
    if (camera_time == -1)
      return;
    sys->feed_measurement_simulation(camera_time, camera_ids, camera_features);
    processed_time = camera_time;
    if (save_results) {
      Eigen::Matrix<double, 17, 1> groundtruth;
      if (sim->get_state(camera_time + camera_offset, groundtruth))
        sys->record_groundtruth(camera_time, groundtruth.segment<16>(1));
    }
    if (viz) {
      viz->visualize();
      viz->visualize_odometry(camera_time + camera_offset);
    }
    camera_time = -1;
  }
};

struct RangeSchedule {
  size_t owner_index;
  size_t neighbor_index;
  double next_elapsed;
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
    const double range_stddev = node->get_parameter("range_stddev").as_double();
    const double range_variance = range_stddev * range_stddev;
    const double range_interval = node->get_parameter("range_interval").as_double();
    const double range_jitter_fraction = node->get_parameter("range_jitter_fraction").as_double();
    const int64_t simulation_seed = node->get_parameter("sim_seed_measurements").as_int();
    if (!std::isfinite(range_stddev) || range_stddev <= 0 || !std::isfinite(range_variance) || range_variance <= 0 ||
        !std::isfinite(range_interval) || range_interval <= 0 || !std::isfinite(range_jitter_fraction) || range_jitter_fraction < 0 ||
        range_jitter_fraction >= 1 || simulation_seed < 0 || simulation_seed > std::numeric_limits<uint32_t>::max())
      throw std::invalid_argument("Invalid range standard deviation, interval, jitter, or simulation seed");
    std::mt19937 random_generator(static_cast<uint32_t>(simulation_seed));
    std::uniform_real_distribution<double> initial_range_phase(0.0, range_interval);
    std::uniform_real_distribution<double> next_range_interval(range_interval * (1.0 - range_jitter_fraction),
                                                               range_interval * (1.0 + range_jitter_fraction));
    std::normal_distribution<double> range_noise(0.0, range_stddev);
    size_t range_count = 0;
    std::ofstream range_results;
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
      if (index == 0 && params.save_results) {
        // VioManager creates per-agent directories below this fleet result directory.
        boost::filesystem::create_directories(params.results_path);
        range_results.open(params.results_path + "/ranges.csv");
        if (!range_results)
          throw std::runtime_error("Unable to open fleet range results");
        range_results << "owner,neighbor,elapsed,owner_timestamp,neighbor_timestamp,true_range,measured_range,variance\n";
        range_results << std::setprecision(17);
      }
      params.set_results_namespace(names.at(index));
      params.factor_graph_agent_id = index;
      params.defer_factor_graph_results = true;
      params.num_opencv_threads = 0;
      params.use_multi_threading_pubs = false;
      params.use_multi_threading_subs = false;

      auto agent = std::make_unique<SimulatedAgent>();
      agent->name = names.at(index);
      agent->sim = std::make_shared<Simulator>(params);
      agent->sys = std::make_shared<VioManager>(params);
      agent->start_time = agent->sim->current_timestamp();
      agent->camera_offset = agent->sim->get_true_parameters().calib_camimu_dt;
      agent->save_results = params.save_results;
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

    std::vector<RangeSchedule> range_schedules;
    range_schedules.reserve(agents.size() * (agents.size() - 1) / 2);
    for (size_t i = 0; i < agents.size(); ++i) {
      for (size_t j = i + 1; j < agents.size(); ++j)
        range_schedules.push_back({i, j, initial_range_phase(random_generator)});
    }

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
          agent->processed_time = -1;
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
              if (!agent->sim->ok())
                agent->process_camera();
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
            if (!failed.load() && rclcpp::ok()) {
              try {
                RangeSchedule *selected_schedule = nullptr;
                double selected_elapsed = 0.0;
                double selected_lateness = -std::numeric_limits<double>::infinity();
                for (auto &schedule : range_schedules) {
                  const auto &owner = *agents[schedule.owner_index];
                  const auto &neighbor = *agents[schedule.neighbor_index];
                  if (owner.processed_time < 0 || neighbor.processed_time < 0)
                    continue;
                  const double owner_elapsed = owner.processed_time + owner.camera_offset - owner.start_time;
                  const double neighbor_elapsed = neighbor.processed_time + neighbor.camera_offset - neighbor.start_time;
                  const double pair_elapsed = std::min(owner_elapsed, neighbor_elapsed);
                  const double lateness = pair_elapsed - schedule.next_elapsed;
                  if (lateness >= 0 && lateness > selected_lateness) {
                    selected_schedule = &schedule;
                    selected_elapsed = pair_elapsed;
                    selected_lateness = lateness;
                  }
                }
                if (selected_schedule != nullptr) {
                  auto &owner = *agents[selected_schedule->owner_index];
                  auto &neighbor = *agents[selected_schedule->neighbor_index];
                  Eigen::Matrix<double, 17, 1> owner_truth, neighbor_truth;
                  if (!owner.sim->get_state(owner.processed_time + owner.camera_offset, owner_truth) ||
                      !neighbor.sim->get_state(neighbor.processed_time + neighbor.camera_offset, neighbor_truth))
                    throw std::runtime_error("Range epoch has no simulation truth");
                  const double truth = (owner_truth.segment<3>(5) - neighbor_truth.segment<3>(5)).norm();
                  const double measurement = std::max(0.0, truth + range_noise(random_generator));
                  owner.sys->communicate_range(*neighbor.sys, owner.processed_time, neighbor.processed_time, measurement, range_variance);
                  selected_schedule->next_elapsed = selected_elapsed + next_range_interval(random_generator);
                  ++range_count;
                  const double elapsed = owner.processed_time + owner.camera_offset - owner.start_time;
                  RCLCPP_INFO(node->get_logger(), "Range %zu %s <- %s at %.3f s (%.6f, %.6f): truth %.3f m, measured %.3f m", range_count,
                              owner.name.c_str(), neighbor.name.c_str(), elapsed, owner.processed_time, neighbor.processed_time, truth,
                              measurement);
                  if (range_results.is_open()) {
                    range_results << owner.name << ',' << neighbor.name << ',' << elapsed << ',' << owner.processed_time << ','
                                  << neighbor.processed_time << ',' << truth << ',' << measurement << ',' << range_variance << '\n';
                    range_results.flush();
                  }
                }
                for (const auto &candidate : agents) {
                  if (candidate->processed_time >= 0)
                    candidate->sys->record_estimator_results();
                }
              } catch (...) {
                worker_error = std::current_exception();
                active_agent = "camera barrier";
                failed.store(true);
              }
            }
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

        if (agent->viz) {
          agent->viz->visualize_final();
          agent->viz.reset();
        }
        agent->sys.reset();
        RCLCPP_INFO(node->get_logger(), "[%s] completed at elapsed %.6f s", agent->name.c_str(),
                    agent->sim->current_timestamp() - agent->start_time);
      });
    }
    for (auto &worker : workers)
      worker.join();
    if (worker_error)
      std::rethrow_exception(worker_error);
    RCLCPP_INFO(node->get_logger(), "Completed %zu range measurements", range_count);
    agents.clear();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "[%s] %s", active_agent.c_str(), error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
