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
