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

#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>

namespace ov_core {
struct ImuData;
class Feature;
} // namespace ov_core

namespace ov_msckf {

struct VioManagerOptions;
class State;

/** Parallel GTSAM estimator that consumes decisions made by VioManager. */
class FactorGraphState {
public:
  explicit FactorGraphState(const VioManagerOptions &options);

  void feed_imu(const ov_core::ImuData &message);
  void initialize(const FactorGraphInitialization &initialization);
  void initialize(const std::shared_ptr<State> &openvins_state);
  bool reset_openvins(const std::shared_ptr<State> &openvins_state, std::string &error);
  void materialize_clone(double timestamp);
  void add_visual_factors(const std::vector<std::shared_ptr<ov_core::Feature>> &features);

  /** Commit one camera transaction without extracting an estimate or covariance. */
  void finish_update();

  /** Return the estimate propagated to the requested timestamp. */
  EstimatorResult get_estimate(double timestamp);
  FactorGraphResetSnapshot get_reset_snapshot(const std::vector<FactorGraphResetVariable> &variables);

  void communicate(FactorGraphState &neighbor, double timestamp, double neighbor_timestamp, double range, double variance);

private:
  struct Summary {
    uint64_t version;
    gtsam::JacobianFactor::shared_ptr factor;
    gtsam::Values values;
  };

  struct CachedSummary {
    Summary summary;
    gtsam::NonlinearFactor::shared_ptr graph_factor;
  };

  gtsam::Key declare_shared(double timestamp);
  Summary get_summary();
  void update_summary(const Summary &summary, size_t origin);
  struct Frame {
    double timestamp = -1;
    gtsam::Key pose_key = 0;
    gtsam::Key velocity_key = 0;
    gtsam::Key bias_key = 0;
  };

  gtsam::Values current_values() const;
  Frame &ensure_frame(double timestamp);
  void commit(bool full_update = false);
  gtsam::PreintegratedCombinedMeasurements preintegrate(double start, double end, const gtsam::imuBias::ConstantBias &bias) const;

  size_t agent_id;
  uint64_t summary_version = 0;
  gtsam::KeySet shared_keys;
  gtsam::KeySet local_shared_keys;
  std::map<size_t, CachedSummary> cached_summaries;
  gtsam::FactorIndices pending_remove_factor_indices;

  std::unique_ptr<gtsam::ISAM2> optimizer;
  gtsam::NonlinearFactorGraph pending_factors;
  gtsam::Values pending_values;
  std::map<double, Frame> frames;
  std::vector<ov_core::ImuData> imu_buffer;
  boost::shared_ptr<GraphCameraRig> camera_rig;
  gtsam::KeyVector calibration_keys;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_parameters;
  Eigen::Matrix3d accel_calibration;
  Eigen::Matrix3d gyro_calibration;
  Eigen::Matrix3d gravity_sensitivity;
  // Graph biases are in calibrated IMU coordinates; OpenVINS boundaries use raw sensor biases.
  gtsam::Matrix6 raw_to_corrected;
  gtsam::Matrix6 corrected_to_raw;
  double time_offset;
  double sigma_msckf_pixels;
  bool initialized = false;
  bool failed = false;
  size_t next_frame_index = 0;

  std::mutex mutex;
};

} // namespace ov_msckf

#endif // OV_MSCKF_FACTORGRAPHSTATE_H
