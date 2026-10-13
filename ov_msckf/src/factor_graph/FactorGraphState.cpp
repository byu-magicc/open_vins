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
#include "feat/Feature.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/sam/RangeFactor.h>

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>

using namespace ov_msckf;

namespace {

gtsam::Key pose_key(size_t index) { return gtsam::Symbol('x', index); }
gtsam::Key velocity_key(size_t index) { return gtsam::Symbol('v', index); }
gtsam::Key bias_key(size_t index) { return gtsam::Symbol('b', index); }

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

Eigen::MatrixXd joint_marginal_covariance(const gtsam::ISAM2 &optimizer, const gtsam::KeyVector &keys, const std::vector<int> &dimensions) {
  const int total_dimension = std::accumulate(dimensions.begin(), dimensions.end(), 0);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(total_dimension, total_dimension);
  int row = 0;
  for (size_t first = 0; first < keys.size(); first++) {
    covariance.block(row, row, dimensions.at(first), dimensions.at(first)) = optimizer.marginalCovariance(keys.at(first));
    int column = row + dimensions.at(first);
    for (size_t second = first + 1; second < keys.size(); second++) {
      const Eigen::MatrixXd pair =
          pair_marginal_covariance(optimizer, keys.at(first), keys.at(second), dimensions.at(first), dimensions.at(second));
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

FactorGraphState::FactorGraphState(const VioManagerOptions &options) : agent_id(options.factor_graph_agent_id) {
  options.validate_graph_features();
  if (agent_id >= (uint64_t{1} << 24))
    throw std::invalid_argument("Factor graph fleet ID exceeds the shared-key namespace");
  gtsam::ISAM2Params optimizer_parameters;
  optimizer_parameters.relinearizeThreshold = options.relinearize_threshold;
  optimizer_parameters.relinearizeSkip = options.relinearize_skip;
  optimizer_parameters.cacheLinearizedFactors = true;
  optimizer_parameters.findUnusedFactorSlots = true;
  optimizer_parameters.factorization = options.use_qr ? gtsam::ISAM2Params::QR : gtsam::ISAM2Params::CHOLESKY;
  optimizer = std::make_unique<gtsam::ISAM2>(optimizer_parameters);

  time_offset = options.calib_camimu_dt;
  sigma_msckf_pixels = options.msckf_options.sigma_pix;
  accel_calibration = ov_core::quat_2_Rot(options.q_ACCtoIMU) * State::Dm(options.state_options.imu_model, options.vec_da);
  gyro_calibration = ov_core::quat_2_Rot(options.q_GYROtoIMU) * State::Dm(options.state_options.imu_model, options.vec_dw);
  gravity_sensitivity = State::Tg(options.vec_tg);
  raw_to_corrected.setZero();
  raw_to_corrected.topLeftCorner<3, 3>() = accel_calibration;
  raw_to_corrected.bottomLeftCorner<3, 3>() = -gyro_calibration * gravity_sensitivity * accel_calibration;
  raw_to_corrected.bottomRightCorner<3, 3>() = gyro_calibration;
  noise_to_raw.setZero();
  noise_to_raw.topLeftCorner<3, 3>() = accel_calibration.inverse();
  noise_to_raw.bottomRightCorner<3, 3>() = gyro_calibration.inverse();

  preintegration_parameters = gtsam::PreintegrationCombinedParams::MakeSharedU(options.gravity_mag);
  preintegration_parameters->accelerometerCovariance =
      std::pow(options.imu_noises.sigma_a, 2) * accel_calibration * accel_calibration.transpose();
  preintegration_parameters->gyroscopeCovariance =
      std::pow(options.imu_noises.sigma_w, 2) * gyro_calibration * gyro_calibration.transpose() +
      gyro_calibration * gravity_sensitivity * preintegration_parameters->accelerometerCovariance * gravity_sensitivity.transpose() *
          gyro_calibration.transpose();
  preintegration_parameters->integrationCovariance = 1e-10 * Eigen::Matrix3d::Identity();
  preintegration_parameters->biasAccCovariance =
      std::pow(options.imu_noises.sigma_ab, 2) * accel_calibration * accel_calibration.transpose();
  preintegration_parameters->biasOmegaCovariance =
      std::pow(options.imu_noises.sigma_wb, 2) * gyro_calibration * gyro_calibration.transpose();
  preintegration_parameters->biasAccOmegaInt.setZero();

  camera_rig = boost::make_shared<GraphCameraRig>();
  for (int camera_id = 0; camera_id < options.state_options.num_cameras; ++camera_id) {
    const auto camera = options.camera_intrinsics.at(camera_id);
    const Eigen::VectorXd parameters = camera->get_value();
    gtsam::Vector9 calibration;
    calibration << parameters.head<2>(), 0.0, parameters.tail<6>();
    const bool fisheye = std::dynamic_pointer_cast<ov_core::CamEqui>(camera) != nullptr;
    camera_rig->emplace_back(camera_pose_from_openvins(options.camera_extrinsics.at(camera_id)),
                             boost::make_shared<FixedCameraCalibration>(fisheye, calibration));
  }
}

void FactorGraphState::feed_imu(const ov_core::ImuData &message) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!imu_buffer.empty() && message.timestamp <= imu_buffer.back().timestamp)
    return;
  imu_buffer.push_back(message);
}

void FactorGraphState::initialize(const FactorGraphInitialization &initialization) {
  std::lock_guard<std::mutex> lock(mutex);
  pending_factors.resize(0);
  pending_values.clear();
  frames.clear();
  shared_keys.clear();
  local_shared_keys.clear();
  cached_summaries.clear();
  pending_remove_factor_indices.clear();
  summary_version = 0;
  next_frame_index = 0;
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

  Eigen::Matrix<double, 15, 15> tangent_transform = Eigen::Matrix<double, 15, 15>::Zero();
  tangent_transform.block<3, 3>(0, 0).setIdentity();
  tangent_transform.block<3, 3>(3, 3) = ov_core::quat_2_Rot(initialization.imu_state.head<4>());
  tangent_transform.block<3, 3>(6, 6).setIdentity();
  tangent_transform.block<3, 3>(9, 12).setIdentity();
  tangent_transform.block<3, 3>(12, 9).setIdentity();
  const Eigen::MatrixXd information =
      covariance_square_root_information(tangent_transform * initialization.covariance * tangent_transform.transpose());
  const std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms = {{initial_frame.pose_key, information.leftCols<6>()},
                                                                   {initial_frame.velocity_key, information.middleCols<3>(6)},
                                                                   {initial_frame.bias_key, information.rightCols<6>()}};
  const gtsam::JacobianFactor joint_prior(terms, gtsam::Vector::Zero(15), gtsam::noiseModel::Unit::Create(15));
  pending_factors.emplace_shared<gtsam::LinearContainerFactor>(joint_prior, initial_values);
  pending_values.insert(initial_values);
  commit();
  initialized = true;
}

void FactorGraphState::initialize(const std::shared_ptr<State> &openvins_state) {
  FactorGraphInitialization initial;
  initial.timestamp = openvins_state->_timestamp;
  initial.imu_state = openvins_state->_imu->value();
  initial.covariance = StateHelper::get_marginal_covariance(openvins_state, {openvins_state->_imu});
  initialize(initial);
}

bool FactorGraphState::reset_openvins(const std::shared_ptr<State> &openvins_state, std::string &error) {
  std::vector<FactorGraphResetVariable> requests{{FactorGraphResetVariableType::IMU, openvins_state->_timestamp}};
  std::vector<std::shared_ptr<ov_type::Type>> variables{openvins_state->_imu};
  for (const auto &clone : openvins_state->_clones_IMU) {
    requests.push_back({FactorGraphResetVariableType::CLONE, clone.first});
    variables.push_back(clone.second);
  }
  const FactorGraphResetSnapshot snapshot = get_reset_snapshot(requests);
  if (!snapshot.valid) {
    error = snapshot.error;
    return false;
  }
  return StateHelper::reset(openvins_state, variables, snapshot.values, snapshot.covariance, error);
}

gtsam::PreintegratedCombinedMeasurements FactorGraphState::preintegrate(double start, double end,
                                                                        const gtsam::imuBias::ConstantBias &bias) const {
  const auto samples = Propagator::select_imu_readings(imu_buffer, start + time_offset, end + time_offset);
  if (samples.size() < 2)
    throw std::runtime_error("Insufficient IMU samples for factor-graph propagation");
  gtsam::PreintegratedCombinedMeasurements measurements(preintegration_parameters,
                                                        gtsam::imuBias::ConstantBias(raw_to_corrected * bias.vector()));
  for (size_t index = 0; index + 1 < samples.size(); ++index) {
    const auto &first = samples[index];
    const auto &second = samples[index + 1];
    const Eigen::Vector3d acceleration = accel_calibration * (0.5 * (first.am + second.am));
    const Eigen::Vector3d angular_velocity = gyro_calibration * (0.5 * (first.wm + second.wm) - gravity_sensitivity * acceleration);
    measurements.integrateMeasurement(acceleration, angular_velocity, second.timestamp - first.timestamp);
  }
  return measurements;
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
  const auto measurements = preintegrate(previous.timestamp, timestamp, previous_bias);
  auto factor = boost::make_shared<FactorGraphImuFactor>(previous.pose_key, previous.velocity_key, pose_key(next_frame_index),
                                                         velocity_key(next_frame_index), previous.bias_key, bias_key(next_frame_index),
                                                         measurements, raw_to_corrected, noise_to_raw);
  const gtsam::NavState prediction = measurements.predict(gtsam::NavState(previous_pose, previous_velocity),
                                                          gtsam::imuBias::ConstantBias(raw_to_corrected * previous_bias.vector()));

  Frame frame;
  const size_t frame_index = next_frame_index++;
  frame.timestamp = timestamp;
  frame.pose_key = pose_key(frame_index);
  frame.velocity_key = velocity_key(frame_index);
  frame.bias_key = bias_key(frame_index);
  pending_values.insert(frame.pose_key, prediction.pose());
  pending_values.insert(frame.velocity_key, prediction.velocity());
  pending_values.insert(frame.bias_key, previous_bias);
  pending_factors.push_back(factor);
  frames.insert({timestamp, frame});
  return frames.at(timestamp);
}

void FactorGraphState::materialize_clone(double timestamp) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!initialized || failed || frames.find(timestamp) != frames.end())
    return;
  ensure_frame(timestamp);
}

void FactorGraphState::add_visual_factors(const std::vector<std::shared_ptr<ov_core::Feature>> &features) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!initialized || failed)
    return;
  for (const auto &feature : features) {
    auto factor = boost::make_shared<FactorGraphVisualFactor>(camera_rig, sigma_msckf_pixels,
                                                           optimizer->params().factorization == gtsam::ISAM2Params::QR);
    for (const auto &camera : feature->timestamps) {
      const auto &pixels = feature->uvs.at(camera.first);
      for (size_t index = 0; index < camera.second.size(); ++index) {
        const auto frame = frames.find(camera.second[index]);
        if (frame != frames.end())
          factor->add(pixels.at(index).head<2>().cast<double>(), frame->second.pose_key, camera.first);
      }
    }
    if (factor->measured().size() >= 2)
      pending_factors.push_back(factor);
  }
}

void FactorGraphState::commit(bool full_update) {
  if (failed)
    throw std::runtime_error("Factor graph is unavailable after an earlier solver failure");
  const bool has_pending_update = !pending_factors.empty() || !pending_values.empty() || !pending_remove_factor_indices.empty();
  if (!full_update && !has_pending_update)
    return;
  try {
    if (has_pending_update) {
      gtsam::ISAM2UpdateParams update_params;
      update_params.removeFactorIndices = pending_remove_factor_indices;
      optimizer->update(pending_factors, pending_values, update_params);
    }
    if (full_update) {
      gtsam::NonlinearFactorGraph graph;
      for (const auto &factor : optimizer->getFactorsUnsafe())
        if (factor)
          graph.push_back(factor);

      gtsam::LevenbergMarquardtParams lm_parameters = gtsam::LevenbergMarquardtParams::CeresDefaults();
      const gtsam::ISAM2Params isam_parameters = optimizer->params();
      lm_parameters.linearSolverType = isam_parameters.factorization == gtsam::ISAM2Params::QR
                                           ? gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR
                                           : gtsam::NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;
      const gtsam::Values optimized = gtsam::LevenbergMarquardtOptimizer(graph, optimizer->calculateEstimate(), lm_parameters).optimize();

      auto rebuilt_optimizer = std::make_unique<gtsam::ISAM2>(isam_parameters);
      rebuilt_optimizer->update(graph, optimized);
      optimizer = std::move(rebuilt_optimizer);
    }
  } catch (const std::exception &exception) {
    PRINT_ERROR("[FACTOR-GRAPH]: disabling parallel estimator after graph optimization failure: %s\n", exception.what());
    failed = true;
    pending_factors.resize(0);
    pending_values.clear();
    pending_remove_factor_indices.clear();
    return;
  }
  pending_factors.resize(0);
  pending_values.clear();
  pending_remove_factor_indices.clear();
}

EstimatorResult FactorGraphState::get_estimate(double timestamp) {
  std::lock_guard<std::mutex> lock(mutex);
  EstimatorResult estimate;
  estimate.timestamp = timestamp;
  if (!initialized || failed || frames.empty())
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

  if (timestamp > frames.rbegin()->first) {
    const gtsam::Key propagated_pose_key = pose_key(next_frame_index);
    const gtsam::Key propagated_velocity_key = velocity_key(next_frame_index);
    const gtsam::Key propagated_bias_key = bias_key(next_frame_index);
    const auto measurements = preintegrate(frame.timestamp, timestamp, bias);
    FactorGraphImuFactor propagation(frame.pose_key, frame.velocity_key, propagated_pose_key, propagated_velocity_key, frame.bias_key,
                                     propagated_bias_key, measurements, raw_to_corrected, noise_to_raw);
    const gtsam::NavState propagated =
        measurements.predict(gtsam::NavState(pose, velocity), gtsam::imuBias::ConstantBias(raw_to_corrected * bias.vector()));
    pose = propagated.pose();
    velocity = propagated.velocity();

    gtsam::Values propagation_values = values;
    propagation_values.insert(propagated_pose_key, pose);
    propagation_values.insert(propagated_velocity_key, velocity);
    propagation_values.insert(propagated_bias_key, bias);
    std::vector<gtsam::Matrix> jacobians(propagation.size());
    propagation.unwhitenedError(propagation_values, jacobians);
    Eigen::Matrix<double, 15, 15> start_jacobian;
    start_jacobian << jacobians.at(0), jacobians.at(1), jacobians.at(4);
    Eigen::Matrix<double, 15, 15> end_jacobian;
    end_jacobian << jacobians.at(2), jacobians.at(3), jacobians.at(5);
    const Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 15, 15>> end_solver(end_jacobian);
    if (end_solver.rank() != 15)
      throw std::runtime_error("rank-deficient propagated IMU state Jacobian");
    propagation_mapping = end_solver.solve(-start_jacobian);
    const Eigen::Matrix<double, 15, 15> noise_mapping = end_solver.solve(Eigen::Matrix<double, 15, 15>::Identity());
    const auto gaussian_noise = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(propagation.noiseModel());
    if (gaussian_noise == nullptr)
      throw std::runtime_error("propagated IMU factor does not have Gaussian noise");
    propagation_covariance = noise_mapping * gaussian_noise->covariance() * noise_mapping.transpose();
  }

  estimate.valid = true;
  const Eigen::Matrix3d global_to_imu = pose.rotation().matrix().transpose();
  estimate.state.segment<4>(0) = ov_core::rot_2_quat(global_to_imu);
  estimate.state.segment<3>(4) = pose.translation();
  estimate.state.segment<3>(7) = velocity;
  estimate.state.segment<3>(10) = bias.gyroscope();
  estimate.state.segment<3>(13) = bias.accelerometer();

  Eigen::Matrix<double, 15, 15> transform = Eigen::Matrix<double, 15, 15>::Zero();
  transform.block<3, 3>(0, 0).setIdentity();
  transform.block<3, 3>(3, 3) = pose.rotation().matrix();
  transform.block<3, 3>(6, 6).setIdentity();
  transform.block<3, 3>(9, 12).setIdentity();
  transform.block<3, 3>(12, 9).setIdentity();
  try {
    const Eigen::MatrixXd input_covariance = joint_marginal_covariance(*optimizer, propagation_input_keys, propagation_input_dimensions);
    Eigen::Matrix<double, 15, 15> graph_covariance =
        propagation_mapping * input_covariance * propagation_mapping.transpose() + propagation_covariance;
    graph_covariance = 0.5 * (graph_covariance + graph_covariance.transpose());
    estimate.covariance = transform * graph_covariance * transform.transpose();
  } catch (const std::exception &exception) {
    PRINT_WARNING("[FACTOR-GRAPH]: joint covariance extraction failed: %s\n", exception.what());
    estimate.covariance.setConstant(std::numeric_limits<double>::quiet_NaN());
  }

  return estimate;
}

FactorGraphResetSnapshot FactorGraphState::get_reset_snapshot(const std::vector<FactorGraphResetVariable> &variables) {
  std::lock_guard<std::mutex> lock(mutex);
  FactorGraphResetSnapshot snapshot;
  try {
    if (!initialized || failed)
      throw std::runtime_error("factor graph is not initialized and healthy");
    commit();
    const gtsam::Values values = optimizer->calculateEstimate();
    gtsam::KeyVector keys;
    std::vector<int> dimensions;
    auto add_key = [&](gtsam::Key key, int dimension) {
      if (!values.exists(key))
        throw std::runtime_error("requested reset variable is missing from factor graph");
      if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
        keys.push_back(key);
        dimensions.push_back(dimension);
      }
    };
    auto frame_at = [&](double timestamp) -> const Frame & {
      const auto frame = frames.find(timestamp);
      if (frame == frames.end())
        throw std::runtime_error("requested reset timestamp is missing from factor graph");
      return frame->second;
    };
    for (const auto &variable : variables) {
      switch (variable.type) {
      case FactorGraphResetVariableType::IMU: {
        const Frame &frame = frame_at(variable.timestamp);
        add_key(frame.pose_key, 6);
        add_key(frame.velocity_key, 3);
        add_key(frame.bias_key, 6);
        break;
      }
      case FactorGraphResetVariableType::CLONE:
        add_key(frame_at(variable.timestamp).pose_key, 6);
        break;
      }
    }

    std::map<gtsam::Key, int> key_columns;
    int graph_dimension = 0;
    for (size_t index = 0; index < keys.size(); ++index) {
      key_columns.emplace(keys.at(index), graph_dimension);
      graph_dimension += dimensions.at(index);
    }
    int output_dimension = 0;
    for (const auto &variable : variables)
      output_dimension += variable.type == FactorGraphResetVariableType::IMU ? 15 : 6;
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Zero(output_dimension, graph_dimension);
    int row = 0;
    for (const auto &variable : variables) {
      Eigen::VectorXd output;
      switch (variable.type) {
      case FactorGraphResetVariableType::IMU: {
        const Frame &frame = frame_at(variable.timestamp);
        const auto pose = values.at<gtsam::Pose3>(frame.pose_key);
        const auto velocity = values.at<gtsam::Vector3>(frame.velocity_key);
        const auto bias = values.at<gtsam::imuBias::ConstantBias>(frame.bias_key);
        output = Eigen::VectorXd::Zero(16);
        output.segment<4>(0) = ov_core::rot_2_quat(pose.rotation().matrix().transpose());
        output.segment<3>(4) = pose.translation();
        output.segment<3>(7) = velocity;
        output.segment<3>(10) = bias.gyroscope();
        output.segment<3>(13) = bias.accelerometer();
        mapping.block<3, 3>(row, key_columns.at(frame.pose_key)).setIdentity();
        mapping.block<3, 3>(row + 3, key_columns.at(frame.pose_key) + 3) = pose.rotation().matrix();
        mapping.block<3, 3>(row + 6, key_columns.at(frame.velocity_key)).setIdentity();
        mapping.block<3, 3>(row + 9, key_columns.at(frame.bias_key) + 3).setIdentity();
        mapping.block<3, 3>(row + 12, key_columns.at(frame.bias_key)).setIdentity();
        row += 15;
        break;
      }
      case FactorGraphResetVariableType::CLONE: {
        const gtsam::Key key = frame_at(variable.timestamp).pose_key;
        const auto pose = values.at<gtsam::Pose3>(key);
        output.resize(7);
        output.head<4>() = ov_core::rot_2_quat(pose.rotation().matrix().transpose());
        output.tail<3>() = pose.translation();
        mapping.block<3, 3>(row, key_columns.at(key)).setIdentity();
        mapping.block<3, 3>(row + 3, key_columns.at(key) + 3) = pose.rotation().matrix();
        row += 6;
        break;
      }
      }
      snapshot.values.push_back(output);
    }

    gtsam::GaussianFactorGraph gaussian;
    optimizer->addFactorsToGraph(&gaussian);
    const auto marginal = gaussian.marginal(keys);
    const gtsam::HessianFactor dense(*marginal);
    const Eigen::MatrixXd information = dense.information();
    const Eigen::LDLT<Eigen::MatrixXd> decomposition(0.5 * (information + information.transpose()));
    if (decomposition.info() != Eigen::Success || (decomposition.vectorD().array() < -1e-10).any())
      throw std::runtime_error("reset marginal is not positive semidefinite");
    const Eigen::MatrixXd dense_covariance = decomposition.solve(Eigen::MatrixXd::Identity(graph_dimension, graph_dimension));
    std::map<gtsam::Key, int> dense_columns;
    int dense_column = 0;
    for (gtsam::Key key : dense.keys()) {
      const auto requested = std::find(keys.begin(), keys.end(), key);
      if (requested == keys.end())
        throw std::runtime_error("reset marginal returned an unexpected key");
      dense_columns.emplace(key, dense_column);
      dense_column += dimensions.at(std::distance(keys.begin(), requested));
    }
    Eigen::MatrixXd graph_covariance(graph_dimension, graph_dimension);
    for (size_t first = 0; first < keys.size(); ++first)
      for (size_t second = 0; second < keys.size(); ++second)
        graph_covariance.block(key_columns.at(keys.at(first)), key_columns.at(keys.at(second)), dimensions.at(first),
                               dimensions.at(second)) =
            dense_covariance.block(dense_columns.at(keys.at(first)), dense_columns.at(keys.at(second)), dimensions.at(first),
                                   dimensions.at(second));
    snapshot.covariance = mapping * graph_covariance * mapping.transpose();
    snapshot.covariance = 0.5 * (snapshot.covariance + snapshot.covariance.transpose());
    if (!snapshot.covariance.allFinite())
      throw std::runtime_error("reset covariance is non-finite");
    snapshot.valid = true;
  } catch (const std::exception &exception) {
    snapshot.error = exception.what();
    snapshot.values.clear();
    snapshot.covariance.resize(0, 0);
  }
  return snapshot;
}

void FactorGraphState::finish_update() {
  std::lock_guard<std::mutex> lock(mutex);
  if (!initialized || failed)
    return;
  commit();
}

void FactorGraphState::communicate(FactorGraphState &neighbor, double timestamp, double neighbor_timestamp, double range, double variance) {
  if (&neighbor == this || neighbor.agent_id == agent_id)
    throw std::invalid_argument("Ranging requires distinct fleet identities");
  if (!std::isfinite(timestamp) || !std::isfinite(neighbor_timestamp) || !std::isfinite(range) || range < 0 || !std::isfinite(variance) ||
      variance <= 0)
    throw std::invalid_argument("Invalid range measurement or timestamp");
  std::unique_lock<std::mutex> local_lock(mutex, std::defer_lock);
  std::unique_lock<std::mutex> neighbor_lock(neighbor.mutex, std::defer_lock);
  std::lock(local_lock, neighbor_lock);
  if (!initialized || !neighbor.initialized || failed || neighbor.failed)
    throw std::runtime_error("Ranging requires two initialized, healthy factor graphs");

  // This graph owns the range; the neighbor supplies its position and information first.
  const gtsam::Key position = neighbor.declare_shared(neighbor_timestamp);
  update_summary(neighbor.get_summary(), neighbor.agent_id);
  for (const auto &entry : neighbor.cached_summaries)
    update_summary(entry.second.summary, entry.first);
  local_shared_keys.insert(position);
  const Frame &frame = ensure_frame(timestamp);
  // GTSAM's native range Jacobian assumes distinct pose and point linearization positions.
  pending_factors.emplace_shared<gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>>(frame.pose_key, position, range,
                                                                                  gtsam::noiseModel::Isotropic::Variance(1, variance));

  // get_summary commits the range before summarizing it.
  neighbor.update_summary(get_summary(), agent_id);
  for (const auto &entry : cached_summaries)
    neighbor.update_summary(entry.second.summary, entry.first);
  neighbor.commit(true);
}

gtsam::Key FactorGraphState::declare_shared(double timestamp) {
  const Frame &frame = ensure_frame(timestamp);
  const uint64_t frame_index = gtsam::Symbol(frame.pose_key).index();
  if (frame_index > std::numeric_limits<uint32_t>::max())
    throw std::overflow_error("Shared position frame index overflow");
  const gtsam::Key key = gtsam::Symbol('s', (uint64_t{agent_id} << 32) | frame_index);
  if (!optimizer->valueExists(key) && !pending_values.exists(key)) {
    const auto pose = pending_values.exists(frame.pose_key) ? pending_values.at<gtsam::Pose3>(frame.pose_key)
                                                            : optimizer->calculateEstimate<gtsam::Pose3>(frame.pose_key);
    pending_values.insert(key, gtsam::Point3(pose.translation()));
    pending_factors.emplace_shared<FactorGraphPosePositionFactor>(frame.pose_key, key);
  }
  shared_keys.insert(key);
  local_shared_keys.insert(key);
  return key;
}

void FactorGraphState::update_summary(const Summary &summary, size_t origin) {
  if (origin == agent_id)
    return;
  const auto cached = cached_summaries.find(origin);
  if (cached != cached_summaries.end() && summary.version <= cached->second.summary.version)
    return;

  for (gtsam::Key key : summary.factor->keys()) {
    shared_keys.insert(key);
    if (!optimizer->valueExists(key) && !pending_values.exists(key))
      pending_values.insert(key, summary.values.at<gtsam::Point3>(key));
  }
  if (cached != cached_summaries.end()) {
    const auto &old_factor = cached->second.graph_factor;
    const auto pending = std::find(pending_factors.begin(), pending_factors.end(), old_factor);
    if (pending != pending_factors.end()) {
      pending_factors.erase(pending);
    } else {
      const auto &committed = optimizer->getFactorsUnsafe();
      const auto old = std::find(committed.begin(), committed.end(), old_factor);
      if (old == committed.end())
        throw std::logic_error("Cached distributed summary is missing from the graph");
      pending_remove_factor_indices.push_back(std::distance(committed.begin(), old));
    }
  }
  auto factor = boost::make_shared<gtsam::LinearContainerFactor>(summary.factor, summary.values);
  pending_factors.push_back(factor);
  cached_summaries.erase(origin);
  cached_summaries.emplace(origin, CachedSummary{summary, factor});
}

FactorGraphState::Summary FactorGraphState::get_summary() {
  commit(true);
  if (shared_keys.empty() || local_shared_keys.empty())
    throw std::logic_error("Cannot summarize a graph without locally relevant shared positions");

  // Marginalize the entire Bayes-tree Gaussian system at its linearization point.
  gtsam::GaussianFactorGraph gaussian;
  optimizer->addFactorsToGraph(&gaussian);
  const gtsam::KeyVector marginal_keys(shared_keys.begin(), shared_keys.end());
  const auto marginal = gaussian.marginal(marginal_keys);
  const auto dense = boost::make_shared<gtsam::HessianFactor>(*marginal);
  const auto &values = optimizer->getLinearizationPoint();

  // Remove information already received, in the same linearization coordinates.
  gtsam::GaussianFactorGraph corrected;
  corrected.push_back(dense);
  for (const auto &entry : cached_summaries)
    corrected.push_back(entry.second.graph_factor->linearize(values)->negate());
  const gtsam::HessianFactor corrected_dense(corrected);

  gtsam::KeyVector selected_keys;
  std::vector<gtsam::DenseIndex> selected_indices;
  const auto &keys = corrected_dense.keys();
  for (size_t index = 0; index < keys.size(); ++index) {
    if (local_shared_keys.count(keys[index])) {
      selected_keys.push_back(keys[index]);
      selected_indices.push_back(static_cast<gtsam::DenseIndex>(index));
    }
  }
  std::vector<gtsam::Matrix> blocks;
  std::vector<gtsam::Vector> linear_blocks;
  for (size_t i = 0; i < selected_indices.size(); ++i) {
    for (size_t j = i; j < selected_indices.size(); ++j)
      blocks.push_back(corrected_dense.info().block(selected_indices[i], selected_indices[j]));
    linear_blocks.push_back(corrected_dense.linearTerm(corrected_dense.begin() + selected_indices[i]));
  }
  const gtsam::HessianFactor reduced(selected_keys, blocks, linear_blocks, corrected_dense.constantTerm());
  const gtsam::Matrix information = reduced.information();
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen_solver(0.5 * (information + information.transpose()));
  if (eigen_solver.info() != Eigen::Success)
    throw std::runtime_error("Failed to factor distributed summary information");
  const auto &eigenvalues = eigen_solver.eigenvalues();
  const double tolerance = 10.0 * information.rows() * std::numeric_limits<double>::epsilon() * dense->information().norm();
  const Eigen::Index rank = (eigenvalues.array() > tolerance).count();
  gtsam::Matrix jacobian(rank, information.cols());
  gtsam::Vector rhs(rank);
  gtsam::Vector linear_term(information.rows());
  Eigen::Index offset = 0;
  for (const auto &block : linear_blocks) {
    linear_term.segment(offset, block.size()) = block;
    offset += block.size();
  }
  Eigen::Index row = 0;
  for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
    if (eigenvalues(i) <= tolerance)
      continue;
    const double root = std::sqrt(eigenvalues(i));
    jacobian.row(row) = root * eigen_solver.eigenvectors().col(i).transpose();
    rhs(row++) = eigen_solver.eigenvectors().col(i).dot(linear_term) / root;
  }
  std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
  gtsam::Values summary_values;
  offset = 0;
  for (size_t i = 0; i < selected_keys.size(); ++i) {
    const Eigen::Index dimension = linear_blocks[i].size();
    terms.emplace_back(selected_keys[i], jacobian.middleCols(offset, dimension));
    summary_values.insert(selected_keys[i], values.at<gtsam::Point3>(selected_keys[i]));
    offset += dimension;
  }
  return {++summary_version, boost::make_shared<gtsam::JacobianFactor>(terms, rhs), summary_values};
}
