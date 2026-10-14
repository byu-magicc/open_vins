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
#include <string>
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
  // Navigation followed by camera intrinsics, in camera ID order, when supplied.
  Eigen::MatrixXd covariance;
  std::vector<Eigen::Matrix<double, 8, 1>> camera_intrinsics;
};

/** Selected navigation posterior and optional online camera intrinsics at one camera-clock timestamp. */
struct EstimatorResult {
  double timestamp = -1;
  bool valid = false;
  Eigen::Matrix<double, 16, 1> state = Eigen::Matrix<double, 16, 1>::Zero();
  Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Zero();
  std::vector<Eigen::Matrix<double, 8, 1>> camera_intrinsics;
};

enum class FactorGraphResetVariableType { IMU, CLONE, CAMERA_INTRINSICS };

/** A requested OpenVINS variable in active-state covariance order. */
struct FactorGraphResetVariable {
  FactorGraphResetVariableType type;
  double timestamp;
  size_t camera_id = 0;
};

/** Graph posterior converted into OpenVINS values and error coordinates. */
struct FactorGraphResetSnapshot {
  bool valid = false;
  std::string error;
  std::vector<Eigen::VectorXd> values;
  Eigen::MatrixXd covariance;
};

} // namespace ov_msckf

#endif // OV_MSCKF_FACTORGRAPHTYPES_H
