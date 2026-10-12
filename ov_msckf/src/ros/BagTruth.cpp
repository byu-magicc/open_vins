/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "BagTruth.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Geometry>

using namespace ov_msckf;

namespace {

template <size_t Columns> std::vector<std::array<double, Columns>> read_csv(const std::string &path, const std::string &header) {
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line) || line != header)
    throw std::runtime_error("Unable to read truth CSV header: " + path);
  std::vector<std::array<double, Columns>> rows;
  while (std::getline(input, line)) {
    std::istringstream stream(line);
    std::array<double, Columns> row;
    std::string cell;
    for (size_t column = 0; column < Columns; ++column) {
      if (!std::getline(stream, cell, ','))
        throw std::runtime_error("Missing truth CSV value: " + path);
      size_t consumed;
      row[column] = std::stod(cell, &consumed);
      if (consumed != cell.size() || !std::isfinite(row[column]))
        throw std::runtime_error("Invalid truth CSV value: " + path);
    }
    if (stream.peek() != std::char_traits<char>::eof() || (!line.empty() && line.back() == ','))
      throw std::runtime_error("Extra truth CSV values: " + path);
    if (!rows.empty() && row[0] <= rows.back()[0])
      throw std::runtime_error("Truth CSV timestamps must increase: " + path);
    rows.push_back(row);
  }
  if (input.bad() || rows.size() < 2)
    throw std::runtime_error("Insufficient truth CSV samples: " + path);
  return rows;
}

} // namespace

BagTruth::BagTruth(const std::string &directory)
    : states(read_csv<17>(directory + "/truth.csv", "timestamp,q_x,q_y,q_z,q_w,p_x,p_y,p_z,v_x,v_y,v_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z")),
      clock(read_csv<2>(directory + "/clock.csv", "imu_timestamp,simulation_timestamp")) {
  for (size_t index = 1; index < clock.size(); ++index) {
    if (clock[index][1] <= clock[index - 1][1])
      throw std::runtime_error("Simulation clock timestamps must increase: " + directory);
  }
  for (const auto &state : states) {
    if (std::abs(Eigen::Map<const Eigen::Vector4d>(state.data() + 1).norm() - 1.0) > 1e-6)
      throw std::runtime_error("Truth quaternion must have unit norm: " + directory);
  }
}

Eigen::Matrix<double, 17, 1> BagTruth::get_state(double imu_timestamp) const {
  if (!std::isfinite(imu_timestamp) || imu_timestamp < clock.front()[0] || imu_timestamp > clock.back()[0])
    throw std::runtime_error("Requested IMU timestamp is outside truth clock coverage: " + std::to_string(imu_timestamp));
  auto clock_right = std::lower_bound(clock.begin() + 1, clock.end(), imu_timestamp,
                                      [](const std::array<double, 2> &row, double timestamp) { return row[0] < timestamp; });
  const auto &clock_left = *(clock_right - 1);
  const double clock_weight = (imu_timestamp - clock_left[0]) / ((*clock_right)[0] - clock_left[0]);
  const double simulation_time = clock_left[1] + clock_weight * ((*clock_right)[1] - clock_left[1]);
  if (simulation_time < states.front()[0] || simulation_time > states.back()[0])
    throw std::runtime_error("Requested simulation timestamp is outside truth coverage: " + std::to_string(simulation_time));
  auto state_right = std::lower_bound(states.begin() + 1, states.end(), simulation_time,
                                      [](const std::array<double, 17> &row, double timestamp) { return row[0] < timestamp; });
  const auto &state_left = *(state_right - 1);
  const double weight = (simulation_time - state_left[0]) / ((*state_right)[0] - state_left[0]);
  const Eigen::Map<const Eigen::Matrix<double, 17, 1>> left(state_left.data()), right(state_right->data());
  Eigen::Matrix<double, 17, 1> state = (1.0 - weight) * left + weight * right;
  const Eigen::Quaterniond left_quaternion(left(4), left(1), left(2), left(3));
  const Eigen::Quaterniond right_quaternion(right(4), right(1), right(2), right(3));
  state.segment<4>(1) = left_quaternion.slerp(weight, right_quaternion).normalized().coeffs();
  state(0) = imu_timestamp;
  return state;
}
