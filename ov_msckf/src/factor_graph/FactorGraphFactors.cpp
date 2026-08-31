/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "FactorGraphFactors.h"

#include <gtsam/inference/Key.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <cmath>
#include <set>

using namespace ov_msckf;

namespace {

gtsam::KeyVector projection_keys(gtsam::Key pose_key, gtsam::Key landmark_key, const FactorGraphCameraCalibration &calibration) {
  gtsam::KeyVector keys{pose_key, landmark_key};
  if (calibration.estimate_extrinsics)
    keys.push_back(calibration.extrinsic_key);
  if (calibration.estimate_intrinsics)
    keys.push_back(calibration.intrinsic_key);
  return keys;
}

gtsam::KeyVector structureless_keys(const std::vector<FactorGraphObservation> &observations,
                                    const std::map<size_t, FactorGraphCameraCalibration> &calibrations) {
  gtsam::KeyVector keys;
  std::set<gtsam::Key> unique_keys;
  for (const auto &observation : observations) {
    if (unique_keys.insert(observation.pose_key).second)
      keys.push_back(observation.pose_key);
    const auto &calibration = calibrations.at(observation.camera_id);
    if (calibration.estimate_extrinsics && unique_keys.insert(calibration.extrinsic_key).second)
      keys.push_back(calibration.extrinsic_key);
    if (calibration.estimate_intrinsics && unique_keys.insert(calibration.intrinsic_key).second)
      keys.push_back(calibration.intrinsic_key);
  }
  return keys;
}

gtsam::Pose3 camera_extrinsic(const gtsam::Values &values, const FactorGraphCameraCalibration &calibration) {
  return calibration.estimate_extrinsics ? values.at<gtsam::Pose3>(calibration.extrinsic_key) : calibration.imu_T_camera;
}

gtsam::Vector camera_intrinsics(const gtsam::Values &values, const FactorGraphCameraCalibration &calibration) {
  return calibration.estimate_intrinsics ? values.at<gtsam::Vector>(calibration.intrinsic_key) : calibration.intrinsics;
}

Eigen::Vector2d distort(const Eigen::Vector2d &normalized, const gtsam::Vector &intrinsics, bool fisheye,
                        Eigen::Matrix2d *pixel_H_normalized = nullptr, Eigen::Matrix<double, 2, 8> *pixel_H_intrinsics = nullptr) {
  const double x = normalized.x();
  const double y = normalized.y();
  double distorted_x = x;
  double distorted_y = y;
  Eigen::Matrix2d distorted_H_normalized = Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 2, 4> distorted_H_coefficients = Eigen::Matrix<double, 2, 4>::Zero();
  if (fisheye) {
    const double radius = std::sqrt(x * x + y * y);
    if (radius > 1e-10) {
      const double theta = std::atan(radius);
      const double theta2 = theta * theta;
      const double theta4 = theta2 * theta2;
      const double theta6 = theta4 * theta2;
      const double theta8 = theta4 * theta4;
      const double theta_distorted =
          theta * (1.0 + intrinsics(4) * theta2 + intrinsics(5) * theta4 + intrinsics(6) * theta6 + intrinsics(7) * theta8);
      const double scale = theta_distorted / radius;
      distorted_x = x * scale;
      distorted_y = y * scale;
      const double distorted_derivative =
          1.0 + 3.0 * intrinsics(4) * theta2 + 5.0 * intrinsics(5) * theta4 + 7.0 * intrinsics(6) * theta6 + 9.0 * intrinsics(7) * theta8;
      const double scale_derivative = (distorted_derivative * radius / (1.0 + radius * radius) - theta_distorted) / (radius * radius);
      const Eigen::Vector2d direction(x / radius, y / radius);
      distorted_H_normalized = scale * Eigen::Matrix2d::Identity() + Eigen::Vector2d(x, y) * (scale_derivative * direction).transpose();
      const Eigen::Vector4d coefficient_derivatives(theta * theta2, theta * theta4, theta * theta6, theta * theta8);
      distorted_H_coefficients = direction * coefficient_derivatives.transpose();
    }
  } else {
    const double radius2 = x * x + y * y;
    const double radius4 = radius2 * radius2;
    const double radial = 1.0 + intrinsics(4) * radius2 + intrinsics(5) * radius4;
    distorted_x = x * radial + 2.0 * intrinsics(6) * x * y + intrinsics(7) * (radius2 + 2.0 * x * x);
    distorted_y = y * radial + intrinsics(6) * (radius2 + 2.0 * y * y) + 2.0 * intrinsics(7) * x * y;
    const double radial_x = 2.0 * intrinsics(4) * x + 4.0 * intrinsics(5) * radius2 * x;
    const double radial_y = 2.0 * intrinsics(4) * y + 4.0 * intrinsics(5) * radius2 * y;
    distorted_H_normalized << radial + x * radial_x + 2.0 * intrinsics(6) * y + 6.0 * intrinsics(7) * x,
        x * radial_y + 2.0 * intrinsics(6) * x + 2.0 * intrinsics(7) * y, y * radial_x + 2.0 * intrinsics(6) * x + 2.0 * intrinsics(7) * y,
        radial + y * radial_y + 6.0 * intrinsics(6) * y + 2.0 * intrinsics(7) * x;
    distorted_H_coefficients << x * radius2, x * radius4, 2.0 * x * y, radius2 + 2.0 * x * x, y * radius2, y * radius4,
        radius2 + 2.0 * y * y, 2.0 * x * y;
  }
  if (pixel_H_normalized)
    *pixel_H_normalized = Eigen::Vector2d(intrinsics(0), intrinsics(1)).asDiagonal() * distorted_H_normalized;
  if (pixel_H_intrinsics) {
    pixel_H_intrinsics->setZero();
    (*pixel_H_intrinsics)(0, 0) = distorted_x;
    (*pixel_H_intrinsics)(1, 1) = distorted_y;
    (*pixel_H_intrinsics)(0, 2) = 1.0;
    (*pixel_H_intrinsics)(1, 3) = 1.0;
    pixel_H_intrinsics->block<1, 4>(0, 4) = intrinsics(0) * distorted_H_coefficients.row(0);
    pixel_H_intrinsics->block<1, 4>(1, 4) = intrinsics(1) * distorted_H_coefficients.row(1);
  }
  return {intrinsics(0) * distorted_x + intrinsics(2), intrinsics(1) * distorted_y + intrinsics(3)};
}

Eigen::Vector2d project(const gtsam::Values &values, gtsam::Key pose_key, const Eigen::Vector3d &point,
                        const FactorGraphCameraCalibration &calibration, Eigen::Matrix<double, 2, 6> *pixel_H_pose = nullptr,
                        Eigen::Matrix<double, 2, 3> *pixel_H_point = nullptr, Eigen::Matrix<double, 2, 6> *pixel_H_extrinsic = nullptr,
                        Eigen::Matrix<double, 2, 8> *pixel_H_intrinsic = nullptr) {
  const gtsam::Pose3 world_T_imu = values.at<gtsam::Pose3>(pose_key);
  gtsam::Matrix66 camera_H_imu;
  gtsam::Matrix66 camera_H_extrinsic;
  const gtsam::Pose3 world_T_camera = world_T_imu.compose(camera_extrinsic(values, calibration), camera_H_imu, camera_H_extrinsic);
  gtsam::Matrix36 point_camera_H_camera;
  gtsam::Matrix33 point_camera_H_point;
  const gtsam::Point3 point_camera = world_T_camera.transformTo(gtsam::Point3(point), point_camera_H_camera, point_camera_H_point);
  double depth = point_camera.z();
  if (std::abs(depth) < 1e-10)
    depth = std::copysign(1e-10, depth == 0 ? 1.0 : depth);
  Eigen::Matrix<double, 2, 3> normalized_H_point_camera;
  normalized_H_point_camera << 1.0 / depth, 0.0, -point_camera.x() / (depth * depth), 0.0, 1.0 / depth, -point_camera.y() / (depth * depth);
  Eigen::Matrix2d pixel_H_normalized;
  const Eigen::Vector2d pixel = distort({point_camera.x() / depth, point_camera.y() / depth}, camera_intrinsics(values, calibration),
                                        calibration.fisheye, &pixel_H_normalized, pixel_H_intrinsic);
  const Eigen::Matrix<double, 2, 3> pixel_H_point_camera = pixel_H_normalized * normalized_H_point_camera;
  if (pixel_H_pose)
    *pixel_H_pose = pixel_H_point_camera * point_camera_H_camera * camera_H_imu;
  if (pixel_H_point)
    *pixel_H_point = pixel_H_point_camera * point_camera_H_point;
  if (pixel_H_extrinsic)
    *pixel_H_extrinsic = pixel_H_point_camera * point_camera_H_camera * camera_H_extrinsic;
  return pixel;
}

gtsam::Vector raw_reprojection_error(const gtsam::Values &values, const std::vector<FactorGraphObservation> &observations,
                                     const std::map<size_t, FactorGraphCameraCalibration> &calibrations, const Eigen::Vector3d &point,
                                     std::map<gtsam::Key, gtsam::Matrix> *jacobians = nullptr, gtsam::Matrix *point_jacobian = nullptr) {
  gtsam::Vector error(2 * observations.size());
  if (jacobians) {
    jacobians->clear();
    for (gtsam::Key key : structureless_keys(observations, calibrations))
      jacobians->insert({key, gtsam::Matrix::Zero(error.rows(), values.at(key).dim())});
  }
  if (point_jacobian)
    *point_jacobian = gtsam::Matrix::Zero(error.rows(), 3);
  for (size_t index = 0; index < observations.size(); index++) {
    const auto &observation = observations.at(index);
    const auto &calibration = calibrations.at(observation.camera_id);
    Eigen::Matrix<double, 2, 6> pixel_H_pose;
    Eigen::Matrix<double, 2, 3> pixel_H_point;
    Eigen::Matrix<double, 2, 6> pixel_H_extrinsic;
    Eigen::Matrix<double, 2, 8> pixel_H_intrinsic;
    error.segment<2>(2 * index) =
        project(values, observation.pose_key, point, calibration, &pixel_H_pose, &pixel_H_point, &pixel_H_extrinsic, &pixel_H_intrinsic) -
        observation.uv;
    if (jacobians) {
      jacobians->at(observation.pose_key).block(2 * index, 0, 2, 6) = pixel_H_pose;
      if (calibration.estimate_extrinsics)
        jacobians->at(calibration.extrinsic_key).block(2 * index, 0, 2, 6) = pixel_H_extrinsic;
      if (calibration.estimate_intrinsics)
        jacobians->at(calibration.intrinsic_key).block(2 * index, 0, 2, 8) = pixel_H_intrinsic;
    }
    if (point_jacobian)
      point_jacobian->block(2 * index, 0, 2, 3) = pixel_H_point;
  }
  return error;
}

Eigen::Matrix3d calibration_matrix(bool kalibr_model, const gtsam::Vector &parameters) {
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
  if (kalibr_model) {
    matrix << parameters(0), 0, 0, parameters(1), parameters(3), 0, parameters(2), parameters(4), parameters(5);
  } else {
    matrix << parameters(0), parameters(1), parameters(3), 0, parameters(2), parameters(4), 0, 0, parameters(5);
  }
  return matrix;
}

Eigen::Matrix3d gravity_sensitivity_matrix(const gtsam::Vector &parameters) {
  Eigen::Matrix3d matrix;
  matrix << parameters(0), parameters(3), parameters(6), parameters(1), parameters(4), parameters(7), parameters(2), parameters(5),
      parameters(8);
  return matrix;
}

struct DifferentiableImuSample {
  ov_core::ImuData data;
  Eigen::Vector3d wm_time_derivative = Eigen::Vector3d::Zero();
  Eigen::Vector3d am_time_derivative = Eigen::Vector3d::Zero();
  double timestamp_time_derivative = 0.0;
};

DifferentiableImuSample interpolate_imu(const ov_core::ImuData &first, const ov_core::ImuData &second, double timestamp) {
  const double alpha = (timestamp - first.timestamp) / (second.timestamp - first.timestamp);
  DifferentiableImuSample result;
  result.data.timestamp = timestamp;
  result.data.wm = (1.0 - alpha) * first.wm + alpha * second.wm;
  result.data.am = (1.0 - alpha) * first.am + alpha * second.am;
  result.wm_time_derivative = (second.wm - first.wm) / (second.timestamp - first.timestamp);
  result.am_time_derivative = (second.am - first.am) / (second.timestamp - first.timestamp);
  result.timestamp_time_derivative = 1.0;
  return result;
}

std::vector<DifferentiableImuSample> select_imu(const std::vector<ov_core::ImuData> &data, double start, double end) {
  std::vector<DifferentiableImuSample> selected;
  if (data.size() < 2 || end <= start)
    return selected;

  auto sample = [&data](double timestamp) {
    const auto upper = std::upper_bound(data.begin(), data.end(), timestamp,
                                        [](double value, const ov_core::ImuData &measurement) { return value < measurement.timestamp; });
    if (upper == data.begin())
      return interpolate_imu(data.at(0), data.at(1), timestamp);
    if (upper == data.end())
      return interpolate_imu(data.at(data.size() - 2), data.back(), timestamp);
    return interpolate_imu(*std::prev(upper), *upper, timestamp);
  };

  selected.push_back(sample(start));
  for (const auto &measurement : data)
    if (measurement.timestamp > start && measurement.timestamp < end)
      selected.push_back({measurement});
  selected.push_back(sample(end));
  return selected;
}

Eigen::Matrix<double, 3, 6> calibration_product_jacobian(bool kalibr_model, const Eigen::Matrix3d &left, const Eigen::Vector3d &right) {
  Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
  if (kalibr_model) {
    jacobian.col(0) = left.col(0) * right(0);
    jacobian.col(1) = left.col(1) * right(0);
    jacobian.col(2) = left.col(2) * right(0);
    jacobian.col(3) = left.col(1) * right(1);
    jacobian.col(4) = left.col(2) * right(1);
    jacobian.col(5) = left.col(2) * right(2);
  } else {
    jacobian.col(0) = left.col(0) * right(0);
    jacobian.col(1) = left.col(0) * right(1);
    jacobian.col(2) = left.col(1) * right(1);
    jacobian.col(3) = left.col(0) * right(2);
    jacobian.col(4) = left.col(1) * right(2);
    jacobian.col(5) = left.col(2) * right(2);
  }
  return jacobian;
}

Eigen::Matrix<double, 3, 9> gravity_product_jacobian(const Eigen::Vector3d &right) {
  Eigen::Matrix<double, 3, 9> jacobian = Eigen::Matrix<double, 3, 9>::Zero();
  for (int column = 0; column < 3; column++)
    for (int row = 0; row < 3; row++)
      jacobian(row, row + 3 * column) = right(column);
  return jacobian;
}

struct PreintegratedErrorJacobians {
  gtsam::Matrix9 state_i;
  gtsam::Matrix9 state_j;
  gtsam::Matrix9 delta;
};

PreintegratedErrorJacobians preintegrated_error_jacobians(const gtsam::PreintegratedImuMeasurements &preintegrated,
                                                          const gtsam::NavState &state_i, const gtsam::NavState &state_j) {
  gtsam::Matrix9 corrected_H_state_i;
  gtsam::Matrix9 corrected_H_delta;
  const gtsam::Vector9 corrected =
      state_i.correctPIM(preintegrated.preintegrated(), preintegrated.deltaTij(), preintegrated.p().n_gravity,
                         preintegrated.p().omegaCoriolis, preintegrated.p().use2ndOrderCoriolis, corrected_H_state_i, corrected_H_delta);
  gtsam::Matrix9 predicted_H_state_i;
  gtsam::Matrix9 predicted_H_corrected;
  const gtsam::NavState predicted = state_i.retract(corrected, predicted_H_state_i, predicted_H_corrected);
  gtsam::Matrix9 error_H_state_j;
  gtsam::Matrix9 error_H_predicted;
  state_j.localCoordinates(predicted, error_H_state_j, error_H_predicted);
  PreintegratedErrorJacobians result;
  result.state_i = error_H_predicted * (predicted_H_state_i + predicted_H_corrected * corrected_H_state_i);
  result.state_j = error_H_state_j;
  result.delta = error_H_predicted * predicted_H_corrected * corrected_H_delta;
  return result;
}

gtsam::KeyVector imu_keys(gtsam::Key pose_i, gtsam::Key velocity_i, gtsam::Key bias_i, gtsam::Key pose_j, gtsam::Key velocity_j,
                          gtsam::Key bias_j, const FactorGraphImuCalibration &calibration) {
  gtsam::KeyVector keys{pose_i, velocity_i, bias_i, pose_j, velocity_j, bias_j};
  if (calibration.estimate_intrinsics) {
    keys.push_back(calibration.dw_key);
    keys.push_back(calibration.da_key);
    keys.push_back(calibration.imu_rotation_key);
  }
  if (calibration.estimate_gravity_sensitivity)
    keys.push_back(calibration.tg_key);
  if (calibration.estimate_time_offset)
    keys.push_back(calibration.time_offset_key);
  return keys;
}

gtsam::SharedNoiseModel imu_noise(double elapsed, double sigma_gyro, double sigma_accel, double sigma_gyro_bias, double sigma_accel_bias) {
  elapsed = std::max(elapsed, 1e-6);
  gtsam::Vector sigmas(15);
  sigmas.segment<3>(0).setConstant(std::max(sigma_gyro * std::sqrt(elapsed), 1e-9));
  sigmas.segment<3>(3).setConstant(std::max(sigma_accel * elapsed * std::sqrt(elapsed) / std::sqrt(3.0), 1e-9));
  sigmas.segment<3>(6).setConstant(std::max(sigma_accel * std::sqrt(elapsed), 1e-9));
  sigmas.segment<3>(9).setConstant(std::max(sigma_accel_bias * std::sqrt(elapsed), 1e-9));
  sigmas.segment<3>(12).setConstant(std::max(sigma_gyro_bias * std::sqrt(elapsed), 1e-9));
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

} // namespace

FactorGraphProjectionFactor::FactorGraphProjectionFactor(gtsam::Key landmark_key_, const FactorGraphObservation &observation_,
                                                         const FactorGraphCameraCalibration &calibration_, double sigma_pixels)
    : gtsam::NoiseModelFactor(gtsam::noiseModel::Isotropic::Sigma(2, sigma_pixels),
                              projection_keys(observation_.pose_key, landmark_key_, calibration_)),
      landmark_key(landmark_key_), observation(observation_), calibration(calibration_) {}

gtsam::Vector FactorGraphProjectionFactor::unwhitenedError(const gtsam::Values &values,
                                                           boost::optional<std::vector<gtsam::Matrix> &> jacobians) const {
  Eigen::Matrix<double, 2, 6> pixel_H_pose;
  Eigen::Matrix<double, 2, 3> pixel_H_point;
  Eigen::Matrix<double, 2, 6> pixel_H_extrinsic;
  Eigen::Matrix<double, 2, 8> pixel_H_intrinsic;
  const gtsam::Vector error = project(values, observation.pose_key, values.at<gtsam::Point3>(landmark_key), calibration, &pixel_H_pose,
                                      &pixel_H_point, &pixel_H_extrinsic, &pixel_H_intrinsic) -
                              observation.uv;
  if (jacobians) {
    jacobians->clear();
    jacobians->push_back(pixel_H_pose);
    jacobians->push_back(pixel_H_point);
    if (calibration.estimate_extrinsics)
      jacobians->push_back(pixel_H_extrinsic);
    if (calibration.estimate_intrinsics)
      jacobians->push_back(pixel_H_intrinsic);
  }
  return error;
}

gtsam::NonlinearFactor::shared_ptr FactorGraphProjectionFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new FactorGraphProjectionFactor(*this));
}

FactorGraphStructurelessFactor::FactorGraphStructurelessFactor(const std::vector<FactorGraphObservation> &observations_,
                                                               const std::map<size_t, FactorGraphCameraCalibration> &calibrations_,
                                                               const Eigen::Vector3d &initial_point_, double sigma_pixels)
    : gtsam::NoiseModelFactor(gtsam::noiseModel::Isotropic::Sigma(2 * observations_.size(), sigma_pixels),
                              structureless_keys(observations_, calibrations_)),
      observations(observations_), calibrations(calibrations_), initial_point(initial_point_), sigma_pixels(sigma_pixels) {}

Eigen::Vector3d FactorGraphStructurelessFactor::optimize_point(const gtsam::Values &values, const Eigen::Vector3d &seed) const {
  Eigen::Vector3d point = seed;
  double cost = raw_reprojection_error(values, observations, calibrations, point).squaredNorm();
  for (int iteration = 0; iteration < 8; iteration++) {
    Eigen::MatrixXd jacobian;
    const gtsam::Vector residual = raw_reprojection_error(values, observations, calibrations, point, nullptr, &jacobian);
    const Eigen::Matrix3d hessian = jacobian.transpose() * jacobian + 1e-6 * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d increment = -hessian.ldlt().solve(jacobian.transpose() * residual);
    if (!increment.allFinite())
      break;
    double scale = 1.0;
    bool accepted = false;
    while (scale >= 1.0 / 128.0) {
      const Eigen::Vector3d candidate = point + scale * increment;
      const double candidate_cost = raw_reprojection_error(values, observations, calibrations, candidate).squaredNorm();
      if (std::isfinite(candidate_cost) && candidate_cost < cost) {
        point = candidate;
        cost = candidate_cost;
        accepted = true;
        break;
      }
      scale *= 0.5;
    }
    if (!accepted || scale * increment.norm() < 1e-7)
      break;
  }
  return point;
}

gtsam::Vector FactorGraphStructurelessFactor::unwhitenedError(const gtsam::Values &values,
                                                              boost::optional<std::vector<gtsam::Matrix> &> jacobians) const {
  const Eigen::Vector3d point = optimize_point(values, initial_point);
  std::map<gtsam::Key, gtsam::Matrix> raw_jacobians;
  gtsam::Matrix point_jacobian;
  const gtsam::Vector residual = raw_reprojection_error(values, observations, calibrations, point, jacobians ? &raw_jacobians : nullptr,
                                                        jacobians ? &point_jacobian : nullptr);
  if (jacobians) {
    const Eigen::Matrix3d hessian = point_jacobian.transpose() * point_jacobian + 1e-6 * Eigen::Matrix3d::Identity();
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(hessian);
    jacobians->clear();
    for (gtsam::Key key : keys_) {
      const gtsam::Matrix &raw = raw_jacobians.at(key);
      jacobians->push_back(raw - point_jacobian * decomposition.solve(point_jacobian.transpose() * raw));
    }
  }
  return residual;
}

boost::shared_ptr<gtsam::GaussianFactor> FactorGraphStructurelessFactor::linearize(const gtsam::Values &values) const {
  const Eigen::Vector3d point = optimize_point(values, initial_point);
  std::map<gtsam::Key, gtsam::Matrix> raw_jacobians;
  gtsam::Matrix point_jacobian;
  const gtsam::Vector residual = raw_reprojection_error(values, observations, calibrations, point, &raw_jacobians, &point_jacobian);

  const Eigen::JacobiSVD<gtsam::Matrix> decomposition(point_jacobian, Eigen::ComputeFullU);
  const gtsam::Matrix nullspace = decomposition.matrixU().rightCols(point_jacobian.rows() - 3);
  const gtsam::Vector projected_residual = nullspace.transpose() * residual / sigma_pixels;
  std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
  for (gtsam::Key key : keys_)
    terms.emplace_back(key, nullspace.transpose() * raw_jacobians.at(key) / sigma_pixels);
  return boost::make_shared<gtsam::JacobianFactor>(terms, -projected_residual, gtsam::noiseModel::Unit::Create(projected_residual.rows()));
}

gtsam::NonlinearFactor::shared_ptr FactorGraphStructurelessFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new FactorGraphStructurelessFactor(*this));
}

FactorGraphImuFactor::FactorGraphImuFactor(gtsam::Key pose_i_, gtsam::Key velocity_i_, gtsam::Key bias_i_, gtsam::Key pose_j_,
                                           gtsam::Key velocity_j_, gtsam::Key bias_j_, double camera_time_i_, double camera_time_j_,
                                           const std::vector<ov_core::ImuData> &imu_data_, const FactorGraphImuCalibration &calibration_,
                                           double gravity_, double sigma_gyro_, double sigma_accel_, double sigma_gyro_bias_,
                                           double sigma_accel_bias_)
    : gtsam::NoiseModelFactor(imu_noise(camera_time_j_ - camera_time_i_, sigma_gyro_, sigma_accel_, sigma_gyro_bias_, sigma_accel_bias_),
                              imu_keys(pose_i_, velocity_i_, bias_i_, pose_j_, velocity_j_, bias_j_, calibration_)),
      pose_i(pose_i_), velocity_i(velocity_i_), bias_i(bias_i_), pose_j(pose_j_), velocity_j(velocity_j_), bias_j(bias_j_),
      camera_time_i(camera_time_i_), camera_time_j(camera_time_j_), imu_data(&imu_data_), calibration(calibration_), gravity(gravity_),
      sigma_gyro(sigma_gyro_), sigma_accel(sigma_accel_) {}

gtsam::PreintegratedImuMeasurements
FactorGraphImuFactor::preintegrate(const gtsam::Values &values, const gtsam::imuBias::ConstantBias &bias,
                                   std::map<gtsam::Key, gtsam::Matrix> *measurement_sensitivities) const {
  const gtsam::Vector dw = calibration.estimate_intrinsics ? values.at<gtsam::Vector>(calibration.dw_key) : calibration.dw;
  const gtsam::Vector da = calibration.estimate_intrinsics ? values.at<gtsam::Vector>(calibration.da_key) : calibration.da;
  const gtsam::Vector tg = calibration.estimate_gravity_sensitivity ? values.at<gtsam::Vector>(calibration.tg_key) : calibration.tg;
  const double time_offset = calibration.estimate_time_offset ? values.at<double>(calibration.time_offset_key) : calibration.time_offset;
  Eigen::Matrix3d gyro_to_imu = calibration.gyro_to_imu.matrix();
  Eigen::Matrix3d accel_to_imu = calibration.accel_to_imu.matrix();
  if (calibration.estimate_intrinsics) {
    const Eigen::Matrix3d inverse_rotation = values.at<gtsam::Rot3>(calibration.imu_rotation_key).matrix();
    if (calibration.kalibr_model)
      gyro_to_imu = inverse_rotation.transpose();
    else
      accel_to_imu = inverse_rotation.transpose();
  }
  const Eigen::Matrix3d dw_matrix = calibration_matrix(calibration.kalibr_model, dw);
  const Eigen::Matrix3d da_matrix = calibration_matrix(calibration.kalibr_model, da);
  const Eigen::Matrix3d tg_matrix = gravity_sensitivity_matrix(tg);

  auto parameters = gtsam::PreintegrationParams::MakeSharedU(gravity);
  parameters->gyroscopeCovariance = sigma_gyro * sigma_gyro * Eigen::Matrix3d::Identity();
  parameters->accelerometerCovariance = sigma_accel * sigma_accel * Eigen::Matrix3d::Identity();
  parameters->integrationCovariance = 1e-10 * Eigen::Matrix3d::Identity();
  gtsam::PreintegratedImuMeasurements preintegrated(parameters, gtsam::imuBias::ConstantBias());
  const auto selected = select_imu(*imu_data, camera_time_i + time_offset, camera_time_j + time_offset);
  if (measurement_sensitivities) {
    measurement_sensitivities->clear();
    measurement_sensitivities->insert({bias_i, gtsam::Matrix::Zero(9, 6)});
    if (calibration.estimate_intrinsics) {
      measurement_sensitivities->insert({calibration.dw_key, gtsam::Matrix::Zero(9, 6)});
      measurement_sensitivities->insert({calibration.da_key, gtsam::Matrix::Zero(9, 6)});
      measurement_sensitivities->insert({calibration.imu_rotation_key, gtsam::Matrix::Zero(9, 3)});
    }
    if (calibration.estimate_gravity_sensitivity)
      measurement_sensitivities->insert({calibration.tg_key, gtsam::Matrix::Zero(9, 9)});
    if (calibration.estimate_time_offset)
      measurement_sensitivities->insert({calibration.time_offset_key, gtsam::Matrix::Zero(9, 1)});
  }

  const Eigen::Matrix3d accel_calibration = accel_to_imu * da_matrix;
  const Eigen::Matrix3d gyro_calibration = gyro_to_imu * dw_matrix;
  for (size_t index = 0; index + 1 < selected.size(); index++) {
    const auto &first = selected.at(index);
    const auto &second = selected.at(index + 1);
    const double dt = second.data.timestamp - first.data.timestamp;
    const double dt_time_derivative = second.timestamp_time_derivative - first.timestamp_time_derivative;
    const Eigen::Vector3d raw_accel = 0.5 * (first.data.am + second.data.am) - bias.accelerometer();
    const Eigen::Vector3d corrected_accel = accel_calibration * raw_accel;
    const Eigen::Vector3d raw_gyro = 0.5 * (first.data.wm + second.data.wm) - bias.gyroscope() - tg_matrix * corrected_accel;
    const Eigen::Vector3d corrected_gyro = gyro_calibration * raw_gyro;

    std::map<gtsam::Key, gtsam::Matrix> accel_jacobians;
    std::map<gtsam::Key, gtsam::Matrix> gyro_jacobians;
    if (measurement_sensitivities) {
      gtsam::Matrix accel_H_bias = gtsam::Matrix::Zero(3, 6);
      accel_H_bias.leftCols<3>() = -accel_calibration;
      gtsam::Matrix gyro_H_bias = -gyro_calibration * tg_matrix * accel_H_bias;
      gyro_H_bias.rightCols<3>() -= gyro_calibration;
      accel_jacobians.insert({bias_i, accel_H_bias});
      gyro_jacobians.insert({bias_i, gyro_H_bias});

      if (calibration.estimate_intrinsics) {
        const gtsam::Matrix accel_H_da = calibration_product_jacobian(calibration.kalibr_model, accel_to_imu, raw_accel);
        const gtsam::Matrix gyro_H_da = -gyro_calibration * tg_matrix * accel_H_da;
        const gtsam::Matrix gyro_H_dw = calibration_product_jacobian(calibration.kalibr_model, gyro_to_imu, raw_gyro);
        gtsam::Matrix accel_H_rotation = gtsam::Matrix::Zero(3, 3);
        gtsam::Matrix gyro_H_rotation = gtsam::Matrix::Zero(3, 3);
        if (calibration.kalibr_model) {
          gyro_H_rotation = gtsam::skewSymmetric(corrected_gyro);
        } else {
          accel_H_rotation = gtsam::skewSymmetric(corrected_accel);
          gyro_H_rotation = -gyro_calibration * tg_matrix * accel_H_rotation;
        }
        accel_jacobians.insert({calibration.dw_key, gtsam::Matrix::Zero(3, 6)});
        gyro_jacobians.insert({calibration.dw_key, gyro_H_dw});
        accel_jacobians.insert({calibration.da_key, accel_H_da});
        gyro_jacobians.insert({calibration.da_key, gyro_H_da});
        accel_jacobians.insert({calibration.imu_rotation_key, accel_H_rotation});
        gyro_jacobians.insert({calibration.imu_rotation_key, gyro_H_rotation});
      }
      if (calibration.estimate_gravity_sensitivity) {
        accel_jacobians.insert({calibration.tg_key, gtsam::Matrix::Zero(3, 9)});
        gyro_jacobians.insert({calibration.tg_key, -gyro_calibration * gravity_product_jacobian(corrected_accel)});
      }
      if (calibration.estimate_time_offset) {
        const Eigen::Vector3d raw_accel_H_time = 0.5 * (first.am_time_derivative + second.am_time_derivative);
        const Eigen::Vector3d accel_H_time = accel_calibration * raw_accel_H_time;
        const Eigen::Vector3d raw_gyro_H_time = 0.5 * (first.wm_time_derivative + second.wm_time_derivative) - tg_matrix * accel_H_time;
        accel_jacobians.insert({calibration.time_offset_key, accel_H_time});
        gyro_jacobians.insert({calibration.time_offset_key, gyro_calibration * raw_gyro_H_time});
      }
    }

    const gtsam::Vector9 old_delta = preintegrated.preintegrated();
    gtsam::Matrix9 old_H_new;
    gtsam::Matrix93 new_H_accel;
    gtsam::Matrix93 new_H_gyro;
    preintegrated.update(corrected_accel, corrected_gyro, dt, &old_H_new, &new_H_accel, &new_H_gyro);
    if (measurement_sensitivities) {
      gtsam::Vector9 new_H_dt;
      new_H_dt.head<3>() = (preintegrated.preintegrated().head<3>() - old_delta.head<3>()) / dt;
      new_H_dt.segment<3>(3) = old_delta.tail<3>() + gtsam::Rot3::Expmap(old_delta.head<3>()) * corrected_accel * dt;
      new_H_dt.tail<3>() = gtsam::Rot3::Expmap(old_delta.head<3>()) * corrected_accel;
      for (auto &entry : *measurement_sensitivities) {
        entry.second =
            old_H_new * entry.second + new_H_accel * accel_jacobians.at(entry.first) + new_H_gyro * gyro_jacobians.at(entry.first);
        if (entry.first == calibration.time_offset_key && calibration.estimate_time_offset)
          entry.second += new_H_dt * dt_time_derivative;
      }
    }
  }
  return preintegrated;
}

gtsam::Vector FactorGraphImuFactor::unwhitenedError(const gtsam::Values &values,
                                                    boost::optional<std::vector<gtsam::Matrix> &> jacobians) const {
  const auto bias_start = values.at<gtsam::imuBias::ConstantBias>(bias_i);
  const auto bias_end = values.at<gtsam::imuBias::ConstantBias>(bias_j);
  std::map<gtsam::Key, gtsam::Matrix> measurement_sensitivities;
  const auto preintegrated = preintegrate(values, bias_start, jacobians ? &measurement_sensitivities : nullptr);
  gtsam::Vector error(15);
  error.head<9>() = preintegrated.computeErrorAndJacobians(values.at<gtsam::Pose3>(pose_i), values.at<gtsam::Vector3>(velocity_i),
                                                           values.at<gtsam::Pose3>(pose_j), values.at<gtsam::Vector3>(velocity_j),
                                                           gtsam::imuBias::ConstantBias());
  error.tail<6>() = bias_end.vector() - bias_start.vector();
  if (jacobians) {
    const gtsam::NavState state_i(values.at<gtsam::Pose3>(pose_i), values.at<gtsam::Vector3>(velocity_i));
    const gtsam::NavState state_j(values.at<gtsam::Pose3>(pose_j), values.at<gtsam::Vector3>(velocity_j));
    const PreintegratedErrorJacobians derivatives = preintegrated_error_jacobians(preintegrated, state_i, state_j);
    jacobians->clear();
    gtsam::Matrix pose_i_jacobian = gtsam::Matrix::Zero(15, 6);
    pose_i_jacobian.topRows<9>() = derivatives.state_i.leftCols<6>();
    jacobians->push_back(pose_i_jacobian);
    gtsam::Matrix velocity_i_jacobian = gtsam::Matrix::Zero(15, 3);
    velocity_i_jacobian.topRows<9>() = derivatives.state_i.rightCols<3>() * state_i.R().transpose();
    jacobians->push_back(velocity_i_jacobian);
    gtsam::Matrix bias_i_jacobian = gtsam::Matrix::Zero(15, 6);
    bias_i_jacobian.topRows<9>() = derivatives.delta * measurement_sensitivities.at(bias_i);
    bias_i_jacobian.bottomRows<6>() = -gtsam::Matrix6::Identity();
    jacobians->push_back(bias_i_jacobian);
    gtsam::Matrix pose_j_jacobian = gtsam::Matrix::Zero(15, 6);
    pose_j_jacobian.topRows<9>() = derivatives.state_j.leftCols<6>();
    jacobians->push_back(pose_j_jacobian);
    gtsam::Matrix velocity_j_jacobian = gtsam::Matrix::Zero(15, 3);
    velocity_j_jacobian.topRows<9>() = derivatives.state_j.rightCols<3>() * state_j.R().transpose();
    jacobians->push_back(velocity_j_jacobian);
    gtsam::Matrix bias_j_jacobian = gtsam::Matrix::Zero(15, 6);
    bias_j_jacobian.bottomRows<6>() = gtsam::Matrix6::Identity();
    jacobians->push_back(bias_j_jacobian);
    for (size_t key_index = 6; key_index < keys_.size(); key_index++) {
      const gtsam::Key key = keys_.at(key_index);
      gtsam::Matrix calibration_jacobian = gtsam::Matrix::Zero(15, measurement_sensitivities.at(key).cols());
      calibration_jacobian.topRows<9>() = derivatives.delta * measurement_sensitivities.at(key);
      jacobians->push_back(calibration_jacobian);
    }
  }
  return error;
}

gtsam::NavState FactorGraphImuFactor::predict(const gtsam::Values &values, const gtsam::NavState &state_i,
                                              const gtsam::imuBias::ConstantBias &bias_start) const {
  return preintegrate(values, bias_start).predict(state_i, gtsam::imuBias::ConstantBias());
}

gtsam::NonlinearFactor::shared_ptr FactorGraphImuFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new FactorGraphImuFactor(*this));
}

FactorGraphZeroVelocityFactor::FactorGraphZeroVelocityFactor(gtsam::Key pose_key_, gtsam::Key bias_key_, double camera_time_i_,
                                                             double camera_time_j_, const std::vector<ov_core::ImuData> &imu_data_,
                                                             const FactorGraphImuCalibration &fixed_calibration, double gravity_,
                                                             double sigma_gyro_, double sigma_accel_, double noise_multiplier_)
    : gtsam::NoiseModelFactor(
          gtsam::noiseModel::Unit::Create(6 * std::max<size_t>(select_imu(imu_data_, camera_time_i_ + fixed_calibration.time_offset,
                                                                          camera_time_j_ + fixed_calibration.time_offset)
                                                                   .size(),
                                                               2) -
                                          6),
          gtsam::KeyVector{pose_key_, bias_key_}),
      pose_key(pose_key_), bias_key(bias_key_), camera_time_i(camera_time_i_), camera_time_j(camera_time_j_), imu_data(&imu_data_),
      calibration(fixed_calibration), gravity(gravity_), sigma_gyro(sigma_gyro_), sigma_accel(sigma_accel_),
      noise_multiplier(noise_multiplier_),
      interval_count(
          std::max<size_t>(
              select_imu(imu_data_, camera_time_i_ + fixed_calibration.time_offset, camera_time_j_ + fixed_calibration.time_offset).size(),
              2) -
          1) {}

gtsam::Vector FactorGraphZeroVelocityFactor::unwhitenedError(const gtsam::Values &values,
                                                             boost::optional<std::vector<gtsam::Matrix> &> jacobians) const {
  const auto selected = select_imu(*imu_data, camera_time_i + calibration.time_offset, camera_time_j + calibration.time_offset);
  gtsam::Vector error = gtsam::Vector::Zero(6 * interval_count);
  gtsam::Matrix pose_jacobian = gtsam::Matrix::Zero(error.rows(), 6);
  gtsam::Matrix bias_jacobian = gtsam::Matrix::Zero(error.rows(), 6);
  const auto bias = values.at<gtsam::imuBias::ConstantBias>(bias_key);
  const Eigen::Matrix3d da_matrix = calibration_matrix(calibration.kalibr_model, calibration.da);
  const Eigen::Matrix3d dw_matrix = calibration_matrix(calibration.kalibr_model, calibration.dw);
  const Eigen::Matrix3d tg_matrix = gravity_sensitivity_matrix(calibration.tg);
  const Eigen::Matrix3d accel_calibration = calibration.accel_to_imu.matrix() * da_matrix;
  const Eigen::Matrix3d gyro_calibration = calibration.gyro_to_imu.matrix() * dw_matrix;
  const double multiplier = std::sqrt(std::max(noise_multiplier, 1e-12));
  const Eigen::Vector3d gravity_global(0.0, 0.0, gravity);
  gtsam::Matrix33 gravity_imu_H_rotation;
  const Eigen::Vector3d gravity_imu =
      values.at<gtsam::Pose3>(pose_key).rotation().unrotate(gravity_global, gravity_imu_H_rotation, boost::none);
  for (size_t index = 0; index < interval_count && index + 1 < selected.size(); index++) {
    const double dt = selected.at(index + 1).data.timestamp - selected.at(index).data.timestamp;
    const Eigen::Vector3d corrected_accel = accel_calibration * (selected.at(index).data.am - bias.accelerometer());
    const Eigen::Vector3d corrected_gyro = gyro_calibration * (selected.at(index).data.wm - bias.gyroscope() - tg_matrix * corrected_accel);
    const double gyro_weight = std::sqrt(dt) / (sigma_gyro * multiplier);
    const double accel_weight = std::sqrt(dt) / (sigma_accel * multiplier);
    error.segment<3>(6 * index) = gyro_weight * corrected_gyro;
    error.segment<3>(6 * index + 3) = accel_weight * (corrected_accel - gravity_imu);
    pose_jacobian.block<3, 3>(6 * index + 3, 0) = -accel_weight * gravity_imu_H_rotation;
    bias_jacobian.block<3, 3>(6 * index, 0) = gyro_weight * gyro_calibration * tg_matrix * accel_calibration;
    bias_jacobian.block<3, 3>(6 * index, 3) = -gyro_weight * gyro_calibration;
    bias_jacobian.block<3, 3>(6 * index + 3, 0) = -accel_weight * accel_calibration;
  }
  if (jacobians) {
    jacobians->clear();
    jacobians->push_back(pose_jacobian);
    jacobians->push_back(bias_jacobian);
  }
  return error;
}

gtsam::NonlinearFactor::shared_ptr FactorGraphZeroVelocityFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new FactorGraphZeroVelocityFactor(*this));
}
