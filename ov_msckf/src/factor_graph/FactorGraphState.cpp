/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "FactorGraphState.h"

#include "core/VioManagerOptions.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>

using namespace ov_msckf;

namespace {

gtsam::Key pose_key(size_t index) { return gtsam::Symbol('x', index); }
gtsam::Key velocity_key(size_t index) { return gtsam::Symbol('v', index); }
gtsam::Key bias_key(size_t index) { return gtsam::Symbol('b', index); }
gtsam::Key landmark_key(size_t feature_id) { return gtsam::Symbol('l', feature_id); }
gtsam::Key extrinsic_key(size_t camera_id) { return gtsam::Symbol('e', camera_id); }
gtsam::Key intrinsic_key(size_t camera_id) { return gtsam::Symbol('k', camera_id); }
gtsam::Key dw_key() { return gtsam::Symbol('d', 0); }
gtsam::Key da_key() { return gtsam::Symbol('d', 1); }
gtsam::Key tg_key() { return gtsam::Symbol('d', 2); }
gtsam::Key imu_rotation_key() { return gtsam::Symbol('d', 3); }
gtsam::Key time_offset_key() { return gtsam::Symbol('d', 4); }

gtsam::Pose3 imu_pose(const Eigen::Matrix<double, 16, 1> &state) {
  const Eigen::Matrix3d global_to_imu = ov_core::quat_2_Rot(state.segment<4>(0));
  return gtsam::Pose3(gtsam::Rot3(global_to_imu.transpose()), gtsam::Point3(state.segment<3>(4)));
}

gtsam::imuBias::ConstantBias imu_bias(const Eigen::Matrix<double, 16, 1> &state) {
  return gtsam::imuBias::ConstantBias(state.segment<3>(13), state.segment<3>(10));
}

gtsam::Pose3 camera_pose_from_openvins(const Eigen::VectorXd &value) {
  const Eigen::Matrix3d imu_to_camera = ov_core::quat_2_Rot(value.head<4>());
  const Eigen::Matrix3d camera_to_imu = imu_to_camera.transpose();
  return gtsam::Pose3(gtsam::Rot3(camera_to_imu), gtsam::Point3(-camera_to_imu * value.segment<3>(4)));
}

gtsam::Rot3 inverse_rotation_from_openvins(const Eigen::VectorXd &value) { return gtsam::Rot3(ov_core::quat_2_Rot(value).transpose()); }

gtsam::Key calibration_key(const std::string &name) {
  if (name == "imu_dw")
    return dw_key();
  if (name == "imu_da")
    return da_key();
  if (name == "imu_tg")
    return tg_key();
  if (name == "imu_rotation")
    return imu_rotation_key();
  if (name == "camera_time_offset")
    return time_offset_key();
  const std::string extrinsic_prefix = "camera_extrinsics_";
  const std::string intrinsic_prefix = "camera_intrinsics_";
  if (name.compare(0, extrinsic_prefix.size(), extrinsic_prefix) == 0)
    return extrinsic_key(std::stoul(name.substr(extrinsic_prefix.size())));
  if (name.compare(0, intrinsic_prefix.size(), intrinsic_prefix) == 0)
    return intrinsic_key(std::stoul(name.substr(intrinsic_prefix.size())));
  throw std::runtime_error("Unknown factor-graph calibration variable " + name);
}

Eigen::MatrixXd covariance_square_root_information(const Eigen::MatrixXd &covariance) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(0.5 * (covariance + covariance.transpose()));
  Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
  for (int index = 0; index < eigenvalues.rows(); index++)
    eigenvalues(index) = std::max(eigenvalues(index), 1e-14);
  return eigenvalues.cwiseSqrt().cwiseInverse().asDiagonal() * eigen_solver.eigenvectors().transpose();
}

Eigen::MatrixXd pair_marginal_covariance(const gtsam::ISAM2 &optimizer, gtsam::Key first, gtsam::Key second, int first_dimension,
                                         int second_dimension) {
  const auto joint = optimizer.joint(first, second, gtsam::EliminateQR);
  const gtsam::Ordering ordering{first, second};
  const Eigen::MatrixXd information = joint->hessian(ordering).first;
  const Eigen::LDLT<Eigen::MatrixXd> decomposition(0.5 * (information + information.transpose()));
  if (decomposition.info() != Eigen::Success || (decomposition.vectorD().array() <= 0).any())
    throw std::runtime_error("non-positive pair marginal information");
  const int dimension = first_dimension + second_dimension;
  return decomposition.solve(Eigen::MatrixXd::Identity(dimension, dimension));
}

Eigen::MatrixXd joint_marginal_covariance(const gtsam::ISAM2 &optimizer, const gtsam::KeyVector &keys,
                                          const std::vector<int> &dimensions) {
  const int total_dimension = std::accumulate(dimensions.begin(), dimensions.end(), 0);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(total_dimension, total_dimension);
  int row = 0;
  for (size_t first = 0; first < keys.size(); first++) {
    covariance.block(row, row, dimensions.at(first), dimensions.at(first)) = optimizer.marginalCovariance(keys.at(first));
    int column = row + dimensions.at(first);
    for (size_t second = first + 1; second < keys.size(); second++) {
      const Eigen::MatrixXd pair = pair_marginal_covariance(optimizer, keys.at(first), keys.at(second), dimensions.at(first),
                                                            dimensions.at(second));
      covariance.block(row, column, dimensions.at(first), dimensions.at(second)) =
          pair.block(0, dimensions.at(first), dimensions.at(first), dimensions.at(second));
      covariance.block(column, row, dimensions.at(second), dimensions.at(first)) =
          covariance.block(row, column, dimensions.at(first), dimensions.at(second)).transpose();
      column += dimensions.at(second);
    }
    row += dimensions.at(first);
  }
  return covariance;
}

} // namespace

FactorGraphState::FactorGraphState(const VioManagerOptions &options) {
  gtsam::ISAM2Params optimizer_parameters;
  optimizer_parameters.relinearizeThreshold = 0.05;
  optimizer_parameters.relinearizeSkip = 1;
  optimizer_parameters.cacheLinearizedFactors = true;
  optimizer_parameters.findUnusedFactorSlots = true;
  optimizer_parameters.factorization = gtsam::ISAM2Params::QR;
  optimizer = std::make_unique<gtsam::ISAM2>(optimizer_parameters);

  gravity = options.gravity_mag;
  sigma_gyro = options.imu_noises.sigma_w;
  sigma_accel = options.imu_noises.sigma_a;
  sigma_gyro_bias = options.imu_noises.sigma_wb;
  sigma_accel_bias = options.imu_noises.sigma_ab;
  sigma_msckf_pixels = options.msckf_options.sigma_pix;
  sigma_slam_pixels = options.slam_options.sigma_pix;
  sigma_aruco_pixels = options.aruco_options.sigma_pix;
  zupt_noise_multiplier = options.zupt_noise_multiplier;
  imu_calibration.kalibr_model = options.state_options.imu_model == StateOptions::ImuModel::KALIBR;
  imu_calibration.estimate_intrinsics = options.state_options.do_calib_imu_intrinsics;
  imu_calibration.estimate_gravity_sensitivity =
      options.state_options.do_calib_imu_intrinsics && options.state_options.do_calib_imu_g_sensitivity;
  imu_calibration.estimate_time_offset = options.state_options.do_calib_camera_timeoffset;
  imu_calibration.dw_key = dw_key();
  imu_calibration.da_key = da_key();
  imu_calibration.tg_key = tg_key();
  imu_calibration.imu_rotation_key = imu_rotation_key();
  imu_calibration.time_offset_key = time_offset_key();
  imu_calibration.dw = options.vec_dw;
  imu_calibration.da = options.vec_da;
  imu_calibration.tg = options.vec_tg;
  imu_calibration.gyro_to_imu = gtsam::Rot3(ov_core::quat_2_Rot(options.q_GYROtoIMU));
  imu_calibration.accel_to_imu = gtsam::Rot3(ov_core::quat_2_Rot(options.q_ACCtoIMU));
  imu_calibration.time_offset = options.calib_camimu_dt;

  for (int camera_id = 0; camera_id < options.state_options.num_cameras; camera_id++) {
    FactorGraphCameraCalibration calibration;
    calibration.fisheye = std::dynamic_pointer_cast<ov_core::CamEqui>(options.camera_intrinsics.at(camera_id)) != nullptr;
    calibration.estimate_extrinsics = options.state_options.do_calib_camera_pose;
    calibration.estimate_intrinsics = options.state_options.do_calib_camera_intrinsics;
    calibration.extrinsic_key = extrinsic_key(camera_id);
    calibration.intrinsic_key = intrinsic_key(camera_id);
    calibration.imu_T_camera = camera_pose_from_openvins(options.camera_extrinsics.at(camera_id));
    calibration.intrinsics = options.camera_intrinsics.at(camera_id)->get_value();
    camera_calibrations.insert({camera_id, calibration});
  }

}

void FactorGraphState::feed_imu(const ov_core::ImuData &message) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!imu_buffer.empty() && message.timestamp <= imu_buffer.back().timestamp)
    return;
  imu_buffer.push_back(message);
}

void FactorGraphState::feed_gps(const ov_core::GPSData &message) {
  std::lock_guard<std::mutex> lock(mutex);
  gps_buffer.push_back(message);
}

void FactorGraphState::initialize(const FactorGraphInitialization &initialization) {
  std::lock_guard<std::mutex> lock(mutex);
  pending_factors.resize(0);
  pending_values.clear();
  frames.clear();
  landmark_keys.clear();
  next_frame_index = 0;
  next_landmark_index = 0;
  failed = false;

  Frame initial_frame;
  const size_t initial_frame_index = next_frame_index++;
  initial_frame.timestamp = initialization.timestamp;
  initial_frame.pose_key = pose_key(initial_frame_index);
  initial_frame.velocity_key = velocity_key(initial_frame_index);
  initial_frame.bias_key = bias_key(initial_frame_index);
  frames.insert({initial_frame.timestamp, initial_frame});
  gtsam::Values initial_values;
  initial_values.insert(initial_frame.pose_key, imu_pose(initialization.imu_state));
  initial_values.insert(initial_frame.velocity_key, gtsam::Vector3(initialization.imu_state.segment<3>(7)));
  initial_values.insert(initial_frame.bias_key, imu_bias(initialization.imu_state));

  for (const auto &entry : initialization.calibration_values) {
    const gtsam::Key key = calibration_key(entry.first);
    if (entry.first == "imu_rotation")
      initial_values.insert(key, inverse_rotation_from_openvins(entry.second));
    else if (entry.first.find("camera_extrinsics_") == 0)
      initial_values.insert(key, camera_pose_from_openvins(entry.second));
    else if (entry.first == "camera_time_offset")
      initial_values.insert(key, entry.second(0));
    else
      initial_values.insert(key, gtsam::Vector(entry.second));
  }

  const int covariance_size = initialization.covariance.rows();
  Eigen::MatrixXd tangent_transform = Eigen::MatrixXd::Identity(covariance_size, covariance_size);
  gtsam::KeyVector prior_keys;
  std::vector<int> prior_dimensions;
  int input_offset = 0;
  int graph_offset = 0;
  for (size_t index = 0; index < initialization.variable_names.size(); index++) {
    const std::string &name = initialization.variable_names.at(index);
    const int dimension = initialization.variable_dimensions.at(index);
    if (name == "imu") {
      prior_keys.push_back(initial_frame.pose_key);
      prior_dimensions.push_back(6);
      prior_keys.push_back(initial_frame.velocity_key);
      prior_dimensions.push_back(3);
      prior_keys.push_back(initial_frame.bias_key);
      prior_dimensions.push_back(6);
      Eigen::Matrix<double, 15, 15> transform = Eigen::Matrix<double, 15, 15>::Zero();
      transform.block<3, 3>(0, 0).setIdentity();
      transform.block<3, 3>(3, 3) = ov_core::quat_2_Rot(initialization.imu_state.segment<4>(0));
      transform.block<3, 3>(6, 6).setIdentity();
      transform.block<3, 3>(9, 12).setIdentity();
      transform.block<3, 3>(12, 9).setIdentity();
      tangent_transform.block(graph_offset, input_offset, 15, 15) = transform;
      graph_offset += 15;
    } else {
      const gtsam::Key key = calibration_key(name);
      prior_keys.push_back(key);
      prior_dimensions.push_back(dimension);
      if (name.find("camera_extrinsics_") == 0) {
        const Eigen::Vector3d position_in_camera = initialization.calibration_values.at(name).segment<3>(4);
        Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
        transform.block<3, 3>(0, 0).setIdentity();
        transform.block<3, 3>(3, 0) = ov_core::skew_x(position_in_camera);
        transform.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity();
        tangent_transform.block(graph_offset, input_offset, 6, 6) = transform;
      } else if (name == "imu_rotation") {
        tangent_transform.block<3, 3>(graph_offset, input_offset).setIdentity();
      }
      graph_offset += dimension;
    }
    input_offset += dimension;
  }

  Eigen::MatrixXd graph_covariance = tangent_transform * initialization.covariance * tangent_transform.transpose();
  const Eigen::MatrixXd square_root_information = covariance_square_root_information(graph_covariance);
  std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
  int column = 0;
  for (size_t index = 0; index < prior_keys.size(); index++) {
    terms.emplace_back(prior_keys.at(index), square_root_information.block(0, column, covariance_size, prior_dimensions.at(index)));
    column += prior_dimensions.at(index);
  }
  const gtsam::JacobianFactor joint_prior(terms, gtsam::Vector::Zero(covariance_size), gtsam::noiseModel::Unit::Create(covariance_size));
  pending_factors.emplace_shared<gtsam::LinearContainerFactor>(joint_prior, initial_values);
  pending_values.insert(initial_values);
  commit();
  initialized = true;
}

gtsam::Values FactorGraphState::current_values() const {
  gtsam::Values values = optimizer->calculateEstimate();
  for (const auto &entry : pending_values)
    values.insert(entry.key, entry.value);
  return values;
}

FactorGraphState::Frame &FactorGraphState::ensure_frame(double timestamp) {
  auto existing = frames.find(timestamp);
  if (existing != frames.end())
    return existing->second;
  if (frames.empty() || timestamp <= frames.rbegin()->first)
    throw std::runtime_error("Cannot insert a factor-graph pose behind the latest materialized pose");

  const Frame previous = frames.rbegin()->second;
  const gtsam::Values values = current_values();
  const auto previous_pose = values.at<gtsam::Pose3>(previous.pose_key);
  const auto previous_velocity = values.at<gtsam::Vector3>(previous.velocity_key);
  const auto previous_bias = values.at<gtsam::imuBias::ConstantBias>(previous.bias_key);
  auto factor = boost::make_shared<FactorGraphImuFactor>(
      previous.pose_key, previous.velocity_key, previous.bias_key, pose_key(next_frame_index), velocity_key(next_frame_index),
      bias_key(next_frame_index), previous.timestamp, timestamp, imu_buffer, imu_calibration, gravity, sigma_gyro, sigma_accel,
      sigma_gyro_bias, sigma_accel_bias);
  const gtsam::NavState prediction = factor->predict(values, gtsam::NavState(previous_pose, previous_velocity), previous_bias);

  Frame frame;
  const size_t frame_index = next_frame_index++;
  frame.timestamp = timestamp;
  frame.pose_key = pose_key(frame_index);
  frame.velocity_key = velocity_key(frame_index);
  frame.bias_key = bias_key(frame_index);
  pending_values.insert(frame.pose_key, prediction.pose());
  pending_values.insert(frame.velocity_key, prediction.velocity());
  pending_values.insert(frame.bias_key, previous_bias);
  // Keep the IMU factor nonlinear so iSAM2 can update its preintegration and
  // Jacobians when the connected navigation or calibration variables move.
  pending_factors.push_back(factor);
  frames.insert({timestamp, frame});
  return frames.at(timestamp);
}

void FactorGraphState::add_zero_velocity_factor(double timestamp) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!initialized || failed)
    return;
  if (frames.empty() || timestamp <= frames.rbegin()->first)
    return;

  const double previous_timestamp = frames.rbegin()->first;
  Frame frame = frames.rbegin()->second;
  const gtsam::Values values = current_values();
  // The stationary interval reuses the same navigation variables. Preserve the
  // earlier timestamp as an alias because retained feature tracks can still
  // contain observations from before the ZUPT interval.
  frame.timestamp = timestamp;
  frames.insert({timestamp, frame});

  // The OpenVINS ZUPT currently treats calibration as fixed during this update.
  // Freeze the graph's current calibration values for the same behavior.
  FactorGraphImuCalibration fixed_calibration = imu_calibration;
  if (imu_calibration.estimate_intrinsics) {
    fixed_calibration.dw = values.at<gtsam::Vector>(imu_calibration.dw_key);
    fixed_calibration.da = values.at<gtsam::Vector>(imu_calibration.da_key);
    const gtsam::Rot3 inverse_rotation = values.at<gtsam::Rot3>(imu_calibration.imu_rotation_key);
    if (imu_calibration.kalibr_model)
      fixed_calibration.gyro_to_imu = inverse_rotation.inverse();
    else
      fixed_calibration.accel_to_imu = inverse_rotation.inverse();
  }
  if (imu_calibration.estimate_gravity_sensitivity)
    fixed_calibration.tg = values.at<gtsam::Vector>(imu_calibration.tg_key);
  if (imu_calibration.estimate_time_offset)
    fixed_calibration.time_offset = values.at<double>(imu_calibration.time_offset_key);
  fixed_calibration.estimate_intrinsics = false;
  fixed_calibration.estimate_gravity_sensitivity = false;
  fixed_calibration.estimate_time_offset = false;
  pending_factors.push_back(boost::make_shared<FactorGraphZeroVelocityFactor>(frame.pose_key, frame.bias_key, previous_timestamp, timestamp,
                                                                              imu_buffer, fixed_calibration, gravity, sigma_gyro,
                                                                              sigma_accel, zupt_noise_multiplier));
}

void FactorGraphState::add_visual_factors(const FactorGraphVisualUpdate &update) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!initialized || failed)
    return;

  std::set<double> required_times;
  for (const auto &track : update.tracks)
    for (const auto &camera : track.timestamps)
      required_times.insert(camera.second.begin(), camera.second.end());
  for (double timestamp : required_times) {
    if (frames.find(timestamp) == frames.end() && timestamp > frames.rbegin()->first)
      ensure_frame(timestamp);
  }

  for (const auto &track : update.tracks) {
    std::vector<FactorGraphObservation> observations;
    for (const auto &camera : track.timestamps) {
      const auto pixels = track.uvs.find(camera.first);
      if (pixels == track.uvs.end())
        continue;
      for (size_t index = 0; index < camera.second.size() && index < pixels->second.size(); index++) {
        const auto frame = frames.find(camera.second.at(index));
        if (frame == frames.end())
          continue;
        FactorGraphObservation observation;
        observation.pose_key = frame->second.pose_key;
        observation.camera_id = camera.first;
        observation.uv = pixels->second.at(index).head<2>().cast<double>();
        observations.push_back(observation);
      }
    }

    if (update.type == FactorGraphVisualUpdateType::STRUCTURELESS) {
      if (observations.size() >= 2) {
        auto factor = boost::make_shared<FactorGraphStructurelessFactor>(observations, camera_calibrations, track.position_global,
                                                                         sigma_msckf_pixels);
        const gtsam::Vector residual = factor->unwhitenedError(current_values());
        const double rms = residual.norm() / std::sqrt(static_cast<double>(residual.rows()));
        if (residual.allFinite()) {
          pending_factors.push_back(factor);
        } else {
          PRINT_WARNING("[FACTOR-GRAPH]: rejected non-finite structureless factor %zu (RMS %.3f px)\n", track.feature_id, rms);
        }
      }
      continue;
    }

    Eigen::Vector3d initial_point = track.position_global;
    const auto anchor_frame = frames.find(track.anchor_timestamp);
    const auto anchor_calibration = camera_calibrations.find(track.anchor_camera_id);
    if (track.anchor_camera_id >= 0 && anchor_frame != frames.end() && anchor_calibration != camera_calibrations.end()) {
      const gtsam::Values values = current_values();
      const gtsam::Pose3 world_T_imu = values.at<gtsam::Pose3>(anchor_frame->second.pose_key);
      const FactorGraphCameraCalibration &calibration = anchor_calibration->second;
      const gtsam::Pose3 imu_T_camera =
          calibration.estimate_extrinsics ? values.at<gtsam::Pose3>(calibration.extrinsic_key) : calibration.imu_T_camera;
      initial_point = world_T_imu.compose(imu_T_camera).transformFrom(gtsam::Point3(track.position_anchor));
    }

    auto landmark = landmark_keys.find(track.feature_id);
    if (landmark == landmark_keys.end()) {
      // A delayed SLAM track can outlive all but one of its graph poses across
      // a long ZUPT interval. One pixel observation cannot constrain a new
      // global Point3, so wait for another retained observation.
      if (observations.size() < 2)
        continue;
      const gtsam::Key candidate_key = landmark_key(next_landmark_index);
      gtsam::Values candidate_values = current_values();
      candidate_values.insert(candidate_key, gtsam::Point3(initial_point));
      Eigen::MatrixXd point_jacobian = Eigen::MatrixXd::Zero(2 * observations.size(), 3);
      bool finite_jacobian = true;
      for (size_t index = 0; index < observations.size(); index++) {
        FactorGraphProjectionFactor factor(candidate_key, observations.at(index),
                                           camera_calibrations.at(observations.at(index).camera_id), sigma_slam_pixels);
        std::vector<gtsam::Matrix> jacobians;
        const gtsam::Vector residual = factor.unwhitenedError(candidate_values, jacobians);
        if (!residual.allFinite() || jacobians.size() < 2 || !jacobians.at(1).allFinite()) {
          finite_jacobian = false;
          break;
        }
        point_jacobian.block<2, 3>(2 * index, 0) = jacobians.at(1);
      }
      const Eigen::Vector3d singular_values = Eigen::JacobiSVD<Eigen::MatrixXd>(point_jacobian).singularValues().head<3>();
      if (!finite_jacobian || singular_values(2) <= std::max(1e-9, 1e-6 * singular_values(0)))
        continue;
      // A frontend feature ID can be retired and later initialized again, so
      // each graph lifetime receives a fresh landmark key.
      next_landmark_index++;
      landmark_keys.insert({track.feature_id, candidate_key});
      pending_values.insert(candidate_key, gtsam::Point3(initial_point));
      landmark = landmark_keys.find(track.feature_id);
    }
    const double sigma = track.is_aruco ? sigma_aruco_pixels : sigma_slam_pixels;
    for (const auto &observation : observations) {
      auto factor = boost::make_shared<FactorGraphProjectionFactor>(landmark->second, observation,
                                                                    camera_calibrations.at(observation.camera_id), sigma);
      pending_factors.push_back(factor);
    }
    // StateHelper initializes delayed SLAM features sequentially, updating the
    // navigation state and covariance between features. Preserve that ordering
    // instead of turning the entire delayed-initialization batch into one solve.
    if (update.type == FactorGraphVisualUpdateType::PERSISTENT_INITIALIZATION)
      commit();
  }
}

void FactorGraphState::commit() {
  if (pending_factors.empty() && pending_values.empty())
    return;
  const auto start = std::chrono::steady_clock::now();
  try {
    optimizer->update(pending_factors, pending_values);
  } catch (const std::exception &exception) {
    PRINT_ERROR("[FACTOR-GRAPH]: disabling parallel estimator after iSAM2 failure: %s\n", exception.what());
    failed = true;
    pending_factors.resize(0);
    pending_values.clear();
    return;
  }
  last_update_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  pending_factors.resize(0);
  pending_values.clear();
}

void FactorGraphState::marginalize_landmarks(const std::vector<size_t> &feature_ids) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!initialized || failed)
    return;
  for (size_t feature_id : feature_ids) {
    const auto landmark = landmark_keys.find(feature_id);
    if (landmark == landmark_keys.end())
      continue;
    // Stop associating future frontend tracks with this graph variable.
    landmark_keys.erase(landmark);
  }
}

void FactorGraphState::apply_pending_global_factors(double timestamp) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!initialized || failed || gps_buffer.empty())
    return;
  Frame &frame = ensure_frame(timestamp);
  for (const auto &gps : gps_buffer) {
    pending_factors.emplace_shared<gtsam::GPSFactor>(frame.pose_key, gtsam::Point3(gps.z_global),
                                                     gtsam::noiseModel::Gaussian::Covariance(gps.cov_z_global));
  }
  gps_buffer.clear();
  commit();
}

FactorGraphResult FactorGraphState::current_estimate(double timestamp) {
  FactorGraphResult estimate;
  estimate.timestamp = timestamp;
  if (!initialized || frames.empty())
    return estimate;
  const gtsam::Values values = optimizer->calculateEstimate();
  const Frame &frame = frames.rbegin()->second;
  const gtsam::Pose3 frame_pose = values.at<gtsam::Pose3>(frame.pose_key);
  gtsam::Pose3 pose = frame_pose;
  gtsam::Vector3 velocity = values.at<gtsam::Vector3>(frame.velocity_key);
  const auto bias = values.at<gtsam::imuBias::ConstantBias>(frame.bias_key);
  gtsam::KeyVector propagation_input_keys{frame.pose_key, frame.velocity_key, frame.bias_key};
  std::vector<int> propagation_input_dimensions{6, 3, 6};
  Eigen::MatrixXd propagation_mapping = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> propagation_covariance = Eigen::Matrix<double, 15, 15>::Zero();

  if (timestamp > frame.timestamp) {
    const gtsam::Key propagated_pose_key = pose_key(next_frame_index);
    const gtsam::Key propagated_velocity_key = velocity_key(next_frame_index);
    const gtsam::Key propagated_bias_key = bias_key(next_frame_index);
    FactorGraphImuFactor propagation(frame.pose_key, frame.velocity_key, frame.bias_key, propagated_pose_key, propagated_velocity_key,
                                     propagated_bias_key, frame.timestamp, timestamp, imu_buffer, imu_calibration, gravity, sigma_gyro,
                                     sigma_accel, sigma_gyro_bias, sigma_accel_bias);
    const gtsam::NavState propagated = propagation.predict(values, gtsam::NavState(pose, velocity), bias);
    pose = propagated.pose();
    velocity = propagated.velocity();

    gtsam::Values propagation_values = values;
    propagation_values.insert(propagated_pose_key, pose);
    propagation_values.insert(propagated_velocity_key, velocity);
    propagation_values.insert(propagated_bias_key, bias);
    std::vector<gtsam::Matrix> jacobians;
    propagation.unwhitenedError(propagation_values, jacobians);
    Eigen::Matrix<double, 15, 15> start_jacobian;
    start_jacobian << jacobians.at(0), jacobians.at(1), jacobians.at(2);
    Eigen::Matrix<double, 15, 15> end_jacobian;
    end_jacobian << jacobians.at(3), jacobians.at(4), jacobians.at(5);
    const Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 15, 15>> end_solver(end_jacobian);
    if (end_solver.rank() != 15)
      throw std::runtime_error("rank-deficient propagated IMU state Jacobian");
    int input_dimension = 15;
    for (size_t key_index = 6; key_index < propagation.keys().size(); key_index++) {
      propagation_input_keys.push_back(propagation.keys().at(key_index));
      propagation_input_dimensions.push_back(jacobians.at(key_index).cols());
      input_dimension += jacobians.at(key_index).cols();
    }
    Eigen::MatrixXd input_jacobian = Eigen::MatrixXd::Zero(15, input_dimension);
    input_jacobian.leftCols<15>() = start_jacobian;
    int input_column = 15;
    for (size_t key_index = 6; key_index < propagation.keys().size(); key_index++) {
      input_jacobian.middleCols(input_column, jacobians.at(key_index).cols()) = jacobians.at(key_index);
      input_column += jacobians.at(key_index).cols();
    }
    propagation_mapping = end_solver.solve(-input_jacobian);
    const Eigen::Matrix<double, 15, 15> noise_mapping =
        end_solver.solve(Eigen::Matrix<double, 15, 15>::Identity());
    const auto gaussian_noise = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(propagation.noiseModel());
    if (gaussian_noise == nullptr)
      throw std::runtime_error("propagated IMU factor does not have Gaussian noise");
    propagation_covariance = noise_mapping * gaussian_noise->covariance() * noise_mapping.transpose();
  }

  estimate.valid = true;
  const Eigen::Matrix3d global_to_imu = pose.rotation().matrix().transpose();
  estimate.imu_state.segment<4>(0) = ov_core::rot_2_quat(global_to_imu);
  estimate.imu_state.segment<3>(4) = pose.translation();
  estimate.imu_state.segment<3>(7) = velocity;
  estimate.imu_state.segment<3>(10) = bias.gyroscope();
  estimate.imu_state.segment<3>(13) = bias.accelerometer();

  Eigen::Matrix<double, 15, 15> transform = Eigen::Matrix<double, 15, 15>::Zero();
  transform.block<3, 3>(0, 0).setIdentity();
  transform.block<3, 3>(3, 3) = pose.rotation().matrix();
  transform.block<3, 3>(6, 6).setIdentity();
  transform.block<3, 3>(9, 12).setIdentity();
  transform.block<3, 3>(12, 9).setIdentity();
  const auto block_marginals = [&]() {
    Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Zero();
    covariance.block<6, 6>(0, 0) = optimizer->marginalCovariance(frame.pose_key);
    covariance.block<3, 3>(6, 6) = optimizer->marginalCovariance(frame.velocity_key);
    covariance.block<6, 6>(9, 9) = optimizer->marginalCovariance(frame.bias_key);
    return covariance;
  };

  try {
    const Eigen::MatrixXd input_covariance =
        joint_marginal_covariance(*optimizer, propagation_input_keys, propagation_input_dimensions);
    Eigen::Matrix<double, 15, 15> graph_covariance =
        propagation_mapping * input_covariance * propagation_mapping.transpose() + propagation_covariance;
    graph_covariance = 0.5 * (graph_covariance + graph_covariance.transpose());
    estimate.covariance = transform * graph_covariance * transform.transpose();
  } catch (const std::exception &exception) {
    PRINT_WARNING("[FACTOR-GRAPH]: joint covariance extraction failed; using block marginals: %s\n", exception.what());
    try {
      estimate.covariance = transform * block_marginals() * transform.transpose();
    } catch (const std::exception &fallback_exception) {
      PRINT_WARNING("[FACTOR-GRAPH]: block covariance extraction failed: %s\n", fallback_exception.what());
      estimate.covariance.setConstant(std::numeric_limits<double>::quiet_NaN());
    }
  }
  estimate.factor_count = static_cast<size_t>(std::count_if(optimizer->getFactorsUnsafe().begin(), optimizer->getFactorsUnsafe().end(),
                                                            [](const auto &factor) { return factor != nullptr; }));
  estimate.value_count = values.size();
  estimate.update_seconds = last_update_seconds;
  return estimate;
}

FactorGraphResult FactorGraphState::finish_update(double timestamp) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!initialized)
    return FactorGraphResult();
  commit();
  if (failed)
    return FactorGraphResult();
  return current_estimate(timestamp);
}
