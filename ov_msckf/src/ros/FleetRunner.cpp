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

#include "FleetRunner.h"
#include "core/VioManager.h"
#include "ros/ROS2Visualizer.h"
#include "utils/print.h"

#include <atomic>
#include <boost/filesystem.hpp>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>

using namespace ov_msckf;

struct RangeSchedule {
  size_t owner_index;
  size_t neighbor_index;
  double next_elapsed;
};

void ov_msckf::run_fleet(const std::shared_ptr<rclcpp::Node> &node, std::vector<std::unique_ptr<FleetAgent>> &agents) {
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

  if (agents.front()->sys->get_params().save_results) {
    const auto directory = boost::filesystem::path(agents.front()->sys->get_params().results_path).parent_path();
    range_results.open((directory / "ranges.csv").string());
    if (!range_results)
      throw std::runtime_error("Unable to open fleet range results");
    range_results << "owner,neighbor,elapsed,owner_timestamp,neighbor_timestamp,true_range,measured_range,variance\n";
    range_results << std::setprecision(17);
  }
  std::string active_agent;
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
    for (size_t index = 0; index < agents.size(); ++index) {
      workers.emplace_back([&, agent = agents[index].get()]() {
        ov_core::Printer::setThreadLabel(agent->name);
        while (true) {
          agent->processed = false;
          if (!agent->finished && rclcpp::ok() && !failed.load()) {
            try {
              agent->finished = !agent->step();
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
                std::all_of(agents.begin(), agents.end(), [](const std::unique_ptr<FleetAgent> &candidate) { return candidate->finished; });
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
  if (worker_error) {
    PRINT_ERROR("[%s] Fleet processing failed\n", active_agent.c_str());
    std::rethrow_exception(worker_error);
  }
  size_t successful_resets = 0;
  size_t skipped_resets = 0;
  for (const auto &agent : agents) {
    ov_core::Printer::setThreadLabel(agent->name);
    agent->finish();
    if (agent->viz)
      agent->viz->visualize_final();
    successful_resets += agent->sys->successful_resets();
    skipped_resets += agent->sys->skipped_resets();
  }
  ov_core::Printer::setThreadLabel("multi_agent");
  RCLCPP_INFO(node->get_logger(), "Completed %zu range measurements; hybrid vehicle resets: %zu successful, %zu skipped", range_count,
              successful_resets, skipped_resets);
}
