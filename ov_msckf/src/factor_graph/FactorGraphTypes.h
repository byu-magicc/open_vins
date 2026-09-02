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

#ifndef OV_MSCKF_FACTORGRAPHTYPES_H
#define OV_MSCKF_FACTORGRAPHTYPES_H

#include <Eigen/Eigen>
#include <cstddef>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace ov_msckf {

/**
 * @brief Initial value and uncertainty supplied by the OpenVINS initializer.
 *
 * The IMU state uses the OpenVINS value ordering [q_GtoI, p_IinG, v_IinG, bg, ba].
 * The factor graph converts this value and its joint error covariance into GTSAM values
 * and one correlated prior factor.
 */
struct FactorGraphInitialization {
  double timestamp = -1;
  Eigen::Matrix<double, 16, 1> imu_state = Eigen::Matrix<double, 16, 1>::Zero();
  Eigen::MatrixXd covariance;
  std::vector<std::string> variable_names;
  std::vector<int> variable_dimensions;
  std::map<std::string, Eigen::VectorXd> calibration_values;
};

/**
 * @brief Immutable copy of the measurements belonging to one accepted feature track.
 *
 * The maps are keyed by camera ID. For each camera, timestamps and raw distorted pixels
 * have matching indices. The accepted OpenVINS triangulation is copied only as a
 * numerical seed; it is not an extra measurement.
 */
struct FactorGraphFeatureTrack {
  size_t feature_id = 0;
  bool is_aruco = false;
  int anchor_camera_id = -1;
  double anchor_timestamp = -1;
  Eigen::Vector3d position_global = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_anchor = Eigen::Vector3d::Zero();
  std::unordered_map<size_t, std::vector<double>> timestamps;
  std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs;
};

/** @brief How OpenVINS used an accepted visual track at this camera timestamp. */
enum class FactorGraphVisualUpdateType {
  STRUCTURELESS,
  PERSISTENT_UPDATE,
  PERSISTENT_INITIALIZATION,
};

/** @brief Accepted visual measurements delivered together at one camera timestamp. */
struct FactorGraphVisualUpdate {
  FactorGraphVisualUpdateType type = FactorGraphVisualUpdateType::STRUCTURELESS;
  std::vector<FactorGraphFeatureTrack> tracks;
};

/** @brief Factor-graph navigation result aligned with one camera transaction. */
struct FactorGraphResult {
  double timestamp = -1;
  bool valid = false;
  Eigen::Matrix<double, 16, 1> imu_state = Eigen::Matrix<double, 16, 1>::Zero();
  Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Zero();
  size_t factor_count = 0;
  size_t value_count = 0;
  double update_seconds = 0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_FACTORGRAPHTYPES_H
