/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_FACTORGRAPHFACTORS_H
#define OV_MSCKF_FACTORGRAPHFACTORS_H

#include "utils/sensor_data.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <vector>

namespace ov_msckf {

/** Camera calibration used by factor-graph projection factors. */
struct FactorGraphCameraCalibration {
  bool fisheye = false;
  bool estimate_extrinsics = false;
  bool estimate_intrinsics = false;
  gtsam::Key extrinsic_key = 0;
  gtsam::Key intrinsic_key = 0;
  gtsam::Pose3 imu_T_camera;
  gtsam::Vector intrinsics;
};

/** One raw-pixel observation of a graph pose. */
struct FactorGraphObservation {
  gtsam::Key pose_key = 0;
  size_t camera_id = 0;
  Eigen::Vector2d uv = Eigen::Vector2d::Zero();
};

/** Optional online IMU calibration variables and their fixed fallbacks. */
struct FactorGraphImuCalibration {
  bool kalibr_model = true;
  bool estimate_intrinsics = false;
  bool estimate_gravity_sensitivity = false;
  bool estimate_time_offset = false;
  gtsam::Key dw_key = 0;
  gtsam::Key da_key = 0;
  gtsam::Key tg_key = 0;
  gtsam::Key imu_rotation_key = 0;
  gtsam::Key time_offset_key = 0;
  gtsam::Vector dw;
  gtsam::Vector da;
  gtsam::Vector tg;
  gtsam::Rot3 gyro_to_imu;
  gtsam::Rot3 accel_to_imu;
  double time_offset = 0;
};

/** Projection factor for a persistent global landmark. */
class FactorGraphProjectionFactor : public gtsam::NoiseModelFactor {
public:
  FactorGraphProjectionFactor(gtsam::Key landmark_key, const FactorGraphObservation &observation,
                              const FactorGraphCameraCalibration &calibration, double sigma_pixels);

  gtsam::Vector unwhitenedError(const gtsam::Values &values,
                                boost::optional<std::vector<gtsam::Matrix> &> jacobians = boost::none) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  gtsam::Key landmark_key;
  FactorGraphObservation observation;
  FactorGraphCameraCalibration calibration;
};

/** Variable-projection visual factor used for an accepted MSCKF track. */
class FactorGraphStructurelessFactor : public gtsam::NoiseModelFactor {
public:
  FactorGraphStructurelessFactor(const std::vector<FactorGraphObservation> &observations,
                                 const std::map<size_t, FactorGraphCameraCalibration> &calibrations, const Eigen::Vector3d &initial_point,
                                 double sigma_pixels);

  gtsam::Vector unwhitenedError(const gtsam::Values &values,
                                boost::optional<std::vector<gtsam::Matrix> &> jacobians = boost::none) const override;
  boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values &values) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  Eigen::Vector3d optimize_point(const gtsam::Values &values, const Eigen::Vector3d &seed) const;

  std::vector<FactorGraphObservation> observations;
  std::map<size_t, FactorGraphCameraCalibration> calibrations;
  Eigen::Vector3d initial_point;
  double sigma_pixels;
};

/** Calibration-aware preintegrated IMU factor with a bias random walk. */
class FactorGraphImuFactor : public gtsam::NoiseModelFactor {
public:
  FactorGraphImuFactor(gtsam::Key pose_i, gtsam::Key velocity_i, gtsam::Key bias_i, gtsam::Key pose_j, gtsam::Key velocity_j,
                       gtsam::Key bias_j, double camera_time_i, double camera_time_j, const std::vector<ov_core::ImuData> &imu_data,
                       const FactorGraphImuCalibration &calibration, double gravity, double sigma_gyro, double sigma_accel,
                       double sigma_gyro_bias, double sigma_accel_bias);

  gtsam::Vector unwhitenedError(const gtsam::Values &values,
                                boost::optional<std::vector<gtsam::Matrix> &> jacobians = boost::none) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;

  gtsam::NavState predict(const gtsam::Values &values, const gtsam::NavState &state_i, const gtsam::imuBias::ConstantBias &bias_i) const;

private:
  gtsam::PreintegratedImuMeasurements preintegrate(const gtsam::Values &values, const gtsam::imuBias::ConstantBias &bias,
                                                   std::map<gtsam::Key, gtsam::Matrix> *measurement_sensitivities = nullptr) const;

  gtsam::Key pose_i;
  gtsam::Key velocity_i;
  gtsam::Key bias_i;
  gtsam::Key pose_j;
  gtsam::Key velocity_j;
  gtsam::Key bias_j;
  double camera_time_i;
  double camera_time_j;
  // The state owns this history and holds its mutex throughout every graph update.
  // Referencing it lets old factors re-evaluate an estimated time offset using IMU
  // samples that arrived after the factor was first constructed.
  const std::vector<ov_core::ImuData> *imu_data;
  FactorGraphImuCalibration calibration;
  double gravity;
  double sigma_gyro;
  double sigma_accel;
};

/** Raw stationary-IMU factor matching the default OpenVINS ZUPT update. */
class FactorGraphZeroVelocityFactor : public gtsam::NoiseModelFactor {
public:
  FactorGraphZeroVelocityFactor(gtsam::Key pose_key, gtsam::Key bias_key, double camera_time_i, double camera_time_j,
                                const std::vector<ov_core::ImuData> &imu_data, const FactorGraphImuCalibration &fixed_calibration,
                                double gravity, double sigma_gyro, double sigma_accel, double noise_multiplier);

  gtsam::Vector unwhitenedError(const gtsam::Values &values,
                                boost::optional<std::vector<gtsam::Matrix> &> jacobians = boost::none) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  gtsam::Key pose_key;
  gtsam::Key bias_key;
  double camera_time_i;
  double camera_time_j;
  const std::vector<ov_core::ImuData> *imu_data;
  FactorGraphImuCalibration calibration;
  double gravity;
  double sigma_gyro;
  double sigma_accel;
  double noise_multiplier;
  size_t interval_count;
};

} // namespace ov_msckf

#endif // OV_MSCKF_FACTORGRAPHFACTORS_H
