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
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>

#include "core/VioManager.h"
#include "ros/BagPlayer.h"
#include "ros/BagTruth.h"
#include "ros/ROS2Visualizer.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

using namespace ov_msckf;

// Each worker owns its bag and estimator; range exchanges run while all workers are at the camera barrier.
struct BagAgent {
  std::string name;
  std::unique_ptr<BagPlayer> bag;
  std::unique_ptr<BagTruth> truth;
  std::shared_ptr<VioManager> sys;
  std::shared_ptr<ROS2Visualizer> viz;
  double start_time;
  double camera_offset;
  double processed_time;
  Eigen::Matrix<double, 17, 1> groundtruth;
  bool processed = false;
  bool finished = false;
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
    double range_stddev = 1.0;
    double range_interval = 15.0;
    double range_jitter_fraction = 0.6;
    int64_t simulation_seed = 5;
    node->get_parameter("range_stddev", range_stddev);
    node->get_parameter("range_interval", range_interval);
    node->get_parameter("range_jitter_fraction", range_jitter_fraction);
    node->get_parameter("sim_seed_measurements", simulation_seed);
    const double range_variance = range_stddev * range_stddev;
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

    std::vector<std::unique_ptr<BagAgent>> agents;
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
      if (index == 0 && params.save_results) {
        boost::filesystem::create_directories(params.results_path);
        range_results.open(params.results_path + "/ranges.csv");
        if (!range_results)
          throw std::runtime_error("Unable to open fleet range results");
        range_results << "owner,neighbor,elapsed,owner_timestamp,neighbor_timestamp,true_range,measured_range,variance\n";
        range_results << std::setprecision(17);
      }
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

    std::vector<RangeSchedule> range_schedules;
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
    try {
      for (auto &agent_ptr : agents) {
        workers.emplace_back([&, agent = agent_ptr.get()]() {
          ov_core::Printer::setThreadLabel(agent->name);
          while (true) {
            agent->processed = false;
            if (!agent->finished && rclcpp::ok() && !failed.load()) {
              try {
                if (agent->bag->process_next_camera(*agent->sys, agent->viz.get())) {
                  // A rejected camera must not create a range at an unprocessed timestamp.
                  agent->processed_time = agent->sys->get_state()->_timestamp;
                  if (agent->processed_time == agent->bag->camera_time()) {
                    agent->groundtruth = agent->truth->get_state(agent->processed_time + agent->camera_offset);
                    agent->processed = true;
                  }
                } else {
                  agent->bag->finish(*agent->sys, agent->viz.get());
                  agent->finished = true;
                }
              } catch (...) {
                std::lock_guard<std::mutex> lock(barrier_mutex);
                if (!worker_error) {
                  worker_error = std::current_exception();
                  active_agent = agent->name;
                }
                failed.store(true);
                barrier_condition.notify_all();
              }
            }

            std::unique_lock<std::mutex> lock(barrier_mutex);
            if (failed.load())
              break;
            const size_t generation = barrier_generation;
            ++barrier_arrivals;
            if (barrier_arrivals == agents.size()) {
              if (rclcpp::ok()) {
                try {
                  RangeSchedule *selected_schedule = nullptr;
                  double selected_elapsed = 0.0;
                  double selected_lateness = -std::numeric_limits<double>::infinity();
                  for (auto &schedule : range_schedules) {
                    const auto &owner = *agents[schedule.owner_index];
                    const auto &neighbor = *agents[schedule.neighbor_index];
                    if (!owner.processed || !neighbor.processed)
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
                    const double truth = (owner.groundtruth.segment<3>(5) - neighbor.groundtruth.segment<3>(5)).norm();
                    const double measurement = std::max(0.0, truth + range_noise(random_generator));
                    if (owner.sys->get_params().runs_factor_graph())
                      owner.sys->communicate_range(*neighbor.sys, owner.processed_time, neighbor.processed_time, measurement,
                                                   range_variance);
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
                    if (candidate->processed) {
                      candidate->sys->record_groundtruth(candidate->processed_time, candidate->groundtruth.segment<16>(1));
                      candidate->sys->record_estimator_results();
                      if (candidate->viz) {
                        candidate->viz->visualize();
                        candidate->viz->visualize_odometry(candidate->processed_time + candidate->camera_offset);
                      }
                    }
                  }
                } catch (...) {
                  worker_error = std::current_exception();
                  active_agent = "camera barrier";
                  failed.store(true);
                }
              }
              stop =
                  failed.load() || !rclcpp::ok() ||
                  std::all_of(agents.begin(), agents.end(), [](const std::unique_ptr<BagAgent> &candidate) { return candidate->finished; });
              barrier_arrivals = 0;
              ++barrier_generation;
              lock.unlock();
              barrier_condition.notify_all();
            } else {
              barrier_condition.wait(lock, [&]() { return barrier_generation != generation || failed.load(); });
            }
            if (stop || failed.load())
              break;
          }
        });
      }
    } catch (...) {
      // Thread creation can fail before all expected workers have joined the barrier.
      std::lock_guard<std::mutex> lock(barrier_mutex);
      if (!worker_error)
        worker_error = std::current_exception();
      failed.store(true);
      barrier_condition.notify_all();
    }
    for (auto &worker : workers)
      worker.join();
    if (worker_error)
      std::rethrow_exception(worker_error);
    size_t successful_resets = 0;
    size_t skipped_resets = 0;
    for (const auto &agent : agents) {
      ov_core::Printer::setThreadLabel(agent->name);
      agent->bag->finish(*agent->sys, agent->viz.get());
      if (agent->viz)
        agent->viz->visualize_final();
      successful_resets += agent->sys->successful_resets();
      skipped_resets += agent->sys->skipped_resets();
    }
    ov_core::Printer::setThreadLabel("multi_agent");
    RCLCPP_INFO(node->get_logger(), "Completed %zu range measurements; hybrid vehicle resets: %zu successful, %zu skipped", range_count,
                successful_resets, skipped_resets);
    agents.clear();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "[%s] %s", active_agent.c_str(), error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
