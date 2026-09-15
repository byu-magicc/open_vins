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

#include "FactorGraphManager.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "types/Type.h"

using namespace ov_msckf;

FactorGraphManager::FactorGraphManager(const VioManagerOptions &options) : state(std::make_unique<FactorGraphState>(options)) {}

void FactorGraphManager::feed_imu(const ov_core::ImuData &message) { state->feed_imu(message); }

void FactorGraphManager::feed_gps(const ov_core::GPSData &message) { state->feed_gps(message); }

void FactorGraphManager::initialize(const std::shared_ptr<State> &openvins_state) {
  FactorGraphInitialization initialization;
  initialization.timestamp = openvins_state->_timestamp;
  initialization.imu_state = openvins_state->_imu->value();

  std::vector<std::shared_ptr<ov_type::Type>> covariance_order;
  covariance_order.push_back(openvins_state->_imu);
  initialization.variable_names.push_back("imu");
  initialization.variable_dimensions.push_back(15);

  auto append_vector = [&](const std::string &name, const std::shared_ptr<ov_type::Type> &variable) {
    covariance_order.push_back(variable);
    initialization.variable_names.push_back(name);
    initialization.variable_dimensions.push_back(variable->size());
    initialization.calibration_values.insert({name, variable->value()});
  };

  if (openvins_state->_options.do_calib_imu_intrinsics) {
    append_vector("imu_dw", openvins_state->_calib_imu_dw);
    append_vector("imu_da", openvins_state->_calib_imu_da);
    if (openvins_state->_options.do_calib_imu_g_sensitivity)
      append_vector("imu_tg", openvins_state->_calib_imu_tg);
    append_vector("imu_rotation", openvins_state->_options.imu_model == StateOptions::ImuModel::KALIBR
                                      ? std::static_pointer_cast<ov_type::Type>(openvins_state->_calib_imu_GYROtoIMU)
                                      : std::static_pointer_cast<ov_type::Type>(openvins_state->_calib_imu_ACCtoIMU));
  }
  if (openvins_state->_options.do_calib_camera_timeoffset)
    append_vector("camera_time_offset", openvins_state->_calib_dt_CAMtoIMU);
  for (int camera_id = 0; camera_id < openvins_state->_options.num_cameras; camera_id++) {
    if (openvins_state->_options.do_calib_camera_pose)
      append_vector("camera_extrinsics_" + std::to_string(camera_id), openvins_state->_calib_IMUtoCAM.at(camera_id));
    if (openvins_state->_options.do_calib_camera_intrinsics)
      append_vector("camera_intrinsics_" + std::to_string(camera_id), openvins_state->_cam_intrinsics.at(camera_id));
  }
  initialization.covariance = StateHelper::get_marginal_covariance(openvins_state, covariance_order);
  state->initialize(initialization);
}

void FactorGraphManager::materialize_clone(double timestamp) { state->materialize_clone(timestamp); }

void FactorGraphManager::add_zero_velocity_factor(double timestamp) { state->add_zero_velocity_factor(timestamp); }

void FactorGraphManager::add_visual_factors(const FactorGraphVisualUpdate &update) { state->add_visual_factors(update); }

void FactorGraphManager::marginalize_landmarks(const std::vector<size_t> &feature_ids) { state->marginalize_landmarks(feature_ids); }

void FactorGraphManager::apply_pending_global_factors(double timestamp) { state->apply_pending_global_factors(timestamp); }

void FactorGraphManager::finish_camera_update() { state->finish_update(); }

FactorGraphResult FactorGraphManager::get_estimate(double timestamp) { return state->get_estimate(timestamp); }

bool FactorGraphManager::reset_openvins(const std::shared_ptr<State> &openvins_state, std::string &error) {
  std::vector<FactorGraphResetVariable> requests;
  std::vector<std::shared_ptr<ov_type::Type>> variables;
  auto append = [&](FactorGraphResetVariable request, const std::shared_ptr<ov_type::Type> &variable) {
    requests.push_back(request);
    variables.push_back(variable);
  };
  FactorGraphResetVariable request;
  request.type = FactorGraphResetVariableType::IMU;
  request.timestamp = openvins_state->_timestamp;
  append(request, openvins_state->_imu);
  if (openvins_state->_options.do_calib_imu_intrinsics) {
    request = {};
    request.type = FactorGraphResetVariableType::IMU_DW;
    append(request, openvins_state->_calib_imu_dw);
    request.type = FactorGraphResetVariableType::IMU_DA;
    append(request, openvins_state->_calib_imu_da);
    if (openvins_state->_options.do_calib_imu_g_sensitivity) {
      request.type = FactorGraphResetVariableType::IMU_TG;
      append(request, openvins_state->_calib_imu_tg);
    }
    request.type = FactorGraphResetVariableType::IMU_ROTATION;
    append(request, openvins_state->_options.imu_model == StateOptions::ImuModel::KALIBR
                        ? std::static_pointer_cast<ov_type::Type>(openvins_state->_calib_imu_GYROtoIMU)
                        : std::static_pointer_cast<ov_type::Type>(openvins_state->_calib_imu_ACCtoIMU));
  }
  if (openvins_state->_options.do_calib_camera_timeoffset) {
    request = {};
    request.type = FactorGraphResetVariableType::CAMERA_TIME_OFFSET;
    append(request, openvins_state->_calib_dt_CAMtoIMU);
  }
  for (int camera_id = 0; camera_id < openvins_state->_options.num_cameras; ++camera_id) {
    request = {};
    request.id = camera_id;
    if (openvins_state->_options.do_calib_camera_pose) {
      request.type = FactorGraphResetVariableType::CAMERA_EXTRINSICS;
      append(request, openvins_state->_calib_IMUtoCAM.at(camera_id));
    }
    if (openvins_state->_options.do_calib_camera_intrinsics) {
      request.type = FactorGraphResetVariableType::CAMERA_INTRINSICS;
      append(request, openvins_state->_cam_intrinsics.at(camera_id));
    }
  }
  for (const auto &clone : openvins_state->_clones_IMU) {
    request = {};
    request.type = FactorGraphResetVariableType::CLONE;
    request.timestamp = clone.first;
    append(request, clone.second);
  }
  for (const auto &feature : openvins_state->_features_SLAM) {
    request = {};
    request.id = feature.first;
    if (feature.second->_feat_representation == ov_type::LandmarkRepresentation::GLOBAL_3D) {
      request.type = FactorGraphResetVariableType::LANDMARK_GLOBAL;
    } else if (feature.second->_feat_representation == ov_type::LandmarkRepresentation::ANCHORED_3D) {
      request.type = FactorGraphResetVariableType::LANDMARK_ANCHORED;
      request.anchor_timestamp = feature.second->_anchor_clone_timestamp;
      request.anchor_camera_id = feature.second->_anchor_cam_id;
    } else {
      error = "hybrid reset requires GLOBAL_3D or ANCHORED_3D persistent landmarks";
      return false;
    }
    append(request, feature.second);
  }

  const FactorGraphResetSnapshot snapshot = state->get_reset_snapshot(requests);
  if (!snapshot.valid) {
    error = snapshot.error;
    return false;
  }
  if (!StateHelper::reset(openvins_state, variables, snapshot.values, snapshot.covariance, error))
    return false;
  for (const auto &camera : openvins_state->_cam_intrinsics)
    openvins_state->_cam_intrinsics_cameras.at(camera.first)->set_value(camera.second->value());
  return true;
}

void FactorGraphManager::communicate(FactorGraphManager &neighbor, double timestamp, double neighbor_timestamp, double range,
                                     double variance) {
  state->communicate(*neighbor.state, timestamp, neighbor_timestamp, range, variance);
}
