/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_FACTORGRAPHSTATE_H
#define OV_MSCKF_FACTORGRAPHSTATE_H

#include "FactorGraphFactors.h"
#include "FactorGraphTypes.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <memory>
#include <mutex>

namespace ov_core {
struct GPSData;
struct ImuData;
} // namespace ov_core

namespace ov_msckf {

struct VioManagerOptions;

/** Parallel GTSAM estimator that consumes decisions made by VioManager. */
class FactorGraphState {
public:
  explicit FactorGraphState(const VioManagerOptions &options);

  void feed_imu(const ov_core::ImuData &message);
  void feed_gps(const ov_core::GPSData &message);
  void initialize(const FactorGraphInitialization &initialization);
  void materialize_clone(double timestamp);
  void add_zero_velocity_factor(double timestamp);
  void add_visual_factors(const FactorGraphVisualUpdate &update);
  void marginalize_landmarks(const std::vector<size_t> &feature_ids);
  void apply_pending_global_factors(double timestamp);

  /** Commit one camera transaction without extracting an estimate or covariance. */
  void finish_update();

  /** Return the estimate propagated to the requested timestamp. */
  FactorGraphResult get_estimate(double timestamp);

private:
  struct Frame {
    double timestamp = -1;
    gtsam::Key pose_key = 0;
    gtsam::Key velocity_key = 0;
    gtsam::Key bias_key = 0;
  };

  gtsam::Values current_values() const;
  Frame &ensure_frame(double timestamp);
  void commit();

  std::unique_ptr<gtsam::ISAM2> optimizer;
  gtsam::NonlinearFactorGraph pending_factors;
  gtsam::Values pending_values;
  std::map<double, Frame> frames;
  std::map<size_t, gtsam::Key> landmark_keys;
  std::vector<ov_core::ImuData> imu_buffer;
  std::vector<ov_core::GPSData> gps_buffer;
  std::map<size_t, FactorGraphCameraCalibration> camera_calibrations;
  FactorGraphImuCalibration imu_calibration;

  double gravity = 9.81;
  double sigma_gyro = 0;
  double sigma_accel = 0;
  double sigma_gyro_bias = 0;
  double sigma_accel_bias = 0;
  double sigma_msckf_pixels = 1;
  double sigma_slam_pixels = 1;
  double sigma_aruco_pixels = 1;
  double zupt_noise_multiplier = 1;
  bool initialized = false;
  bool failed = false;
  size_t next_frame_index = 0;
  size_t next_landmark_index = 0;
  double last_update_seconds = 0;

  std::mutex mutex;
};

} // namespace ov_msckf

#endif // OV_MSCKF_FACTORGRAPHSTATE_H
