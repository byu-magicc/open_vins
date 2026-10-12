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

#ifndef OV_MSCKF_FLEETRUNNER_H
#define OV_MSCKF_FLEETRUNNER_H

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ov_msckf {
class VioManager;
class ROS2Visualizer;

/** One estimator and its camera-step source. All range exchanges happen at the shared barrier. */
struct FleetAgent {
  virtual ~FleetAgent() = default;
  virtual bool step() = 0;
  virtual void finish() = 0;
  std::string name;
  std::shared_ptr<VioManager> sys;
  std::shared_ptr<ROS2Visualizer> viz;
  double start_time;
  double camera_offset;
  double processed_time;
  Eigen::Matrix<double, 17, 1> groundtruth;
  bool processed = false;
  bool finished = false;
};

void run_fleet(const std::shared_ptr<rclcpp::Node> &node, std::vector<std::unique_ptr<FleetAgent>> &agents);
} // namespace ov_msckf
#endif // OV_MSCKF_FLEETRUNNER_H
