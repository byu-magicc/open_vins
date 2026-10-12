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

#ifndef OV_MSCKF_BAG_TRUTH_H
#define OV_MSCKF_BAG_TRUTH_H

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace ov_msckf {

/// HoloOcean ENU states and the IMU-to-simulation clock exported by holoocean_truth.py.
class BagTruth {
public:
  explicit BagTruth(const std::string &directory);
  Eigen::Matrix<double, 17, 1> get_state(double imu_timestamp) const;

private:
  std::vector<std::array<double, 17>> states;
  std::vector<std::array<double, 2>> clock;
};

} // namespace ov_msckf

#endif // OV_MSCKF_BAG_TRUTH_H
