/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "core/VioManager.h"
#include "factor_graph/FactorGraphState.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterGlobal.h"
#include "utils/geodesy.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace ov_msckf;

void check(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

void check_global_update(bool use_qr) {
  VioManagerOptions options;
  options.state_options.num_cameras = 0;
  options.use_qr = use_qr;
  options.vec_dw << 1, 0, 0, 1, 0, 1;
  options.vec_da = options.vec_dw;
  options.vec_tg.setZero();
  options.q_ACCtoIMU << 0, 0, 0, 1;
  options.q_GYROtoIMU = options.q_ACCtoIMU;
  auto state = std::make_shared<State>(options.state_options);
  state->_timestamp = 2.3;
  Eigen::Matrix<double, 16, 1> initial_state = Eigen::Matrix<double, 16, 1>::Zero();
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(-0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  initial_state.head<4>() = ov_core::rot_2_quat(rotation);
  initial_state.segment<3>(4) << 3, -2, 1;
  state->_imu->set_value(initial_state);
  state->_imu->set_fej(initial_state);
  Eigen::Matrix<double, 15, 15> initial_covariance = Eigen::Matrix<double, 15, 15>::Identity();
  initial_covariance.block<3, 3>(3, 6) = 0.2 * Eigen::Matrix3d::Identity();
  initial_covariance.block<3, 3>(6, 3) = initial_covariance.block<3, 3>(3, 6).transpose();
  initial_covariance.block<3, 3>(6, 12) = 0.1 * Eigen::Matrix3d::Identity();
  initial_covariance.block<3, 3>(12, 6) = initial_covariance.block<3, 3>(6, 12).transpose();
  StateHelper::set_initial_covariance(state, initial_covariance, {state->_imu});

  FactorGraphState graph(options);
  FactorGraphInitialization initialization;
  initialization.timestamp = state->_timestamp;
  initialization.imu_state = initial_state;
  initialization.variable_names = {"imu"};
  initialization.variable_dimensions = {15};
  initialization.covariance = initial_covariance;
  graph.initialize(initialization);

  ov_core::GPSData gps;
  gps.timestamp = 2.225;
  gps.lla << 40.2463724, -111.6474138, 1387;
  gps.velocity << 2, -1, 0.5;
  gps.cov_position << 2, 0.3, 0.1, 0.3, 3, -0.2, 0.1, -0.2, 4;
  gps.cov_velocity << 0.4, 0.05, 0.01, 0.05, 0.6, -0.03, 0.01, -0.03, 0.8;
  UpdaterGlobal updater(0.2);
  updater.set_initial_attitude(rotation);
  const auto observation = updater.update(state, gps);
  const Eigen::Matrix3d enu_rotation = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  check(observation.timestamp == state->_timestamp, "GPS observation did not use the camera clock");
  check((observation.position - initial_state.segment<3>(4)).norm() < 1e-12, "First GPS fix did not anchor at the propagated position");
  check((observation.velocity - enu_rotation * gps.velocity).norm() < 1e-12, "GPS velocity yaw alignment is incorrect");
  check((observation.cov_position - enu_rotation * gps.cov_position * enu_rotation.transpose()).norm() < 1e-12 &&
            (observation.cov_velocity - enu_rotation * gps.cov_velocity * enu_rotation.transpose()).norm() < 1e-12,
        "GPS covariance rotation is incorrect");
  graph.add_gps_factors(observation);
  const auto posterior = graph.get_estimate(state->_timestamp);
  check(posterior.valid && (posterior.imu_state - state->_imu->value()).norm() < 1e-8, "Graph GPS posterior disagrees with the EKF update");
  check((posterior.covariance - StateHelper::get_marginal_covariance(state, {state->_imu})).norm() < 1e-8,
        "Graph GPS covariance disagrees with the correlated EKF posterior");
  check(posterior.factor_count == 3 && posterior.value_count == 3, "GPS at an existing frame duplicated navigation variables");

  gps.lla(0) += 0.00001;
  gps.lla(1) -= 0.00002;
  gps.lla(2) += 0.3;
  const Eigen::Vector3d expected_position =
      initial_state.segment<3>(4) +
      enu_rotation * ov_core::ecef_to_enu(Eigen::Vector3d(40.2463724, -111.6474138, 1387)) *
          (ov_core::lla_to_ecef(gps.lla) - ov_core::lla_to_ecef(Eigen::Vector3d(40.2463724, -111.6474138, 1387)));
  check((updater.update(state, gps).position - expected_position).norm() < 1e-9, "GPS origin changed after the first accepted fix");

  ov_core::ImuData imu;
  imu.wm.setZero();
  imu.am << 0, 0, options.gravity_mag;
  for (int i = 0; i <= 30; ++i) {
    imu.timestamp = 2.2 + 0.01 * i;
    graph.feed_imu(imu);
  }
  auto next = observation;
  next.timestamp = 2.35;
  graph.add_gps_factors(next);
  next.timestamp = 2.4;
  graph.add_gps_factors(next);
  const auto before_camera = graph.get_estimate(next.timestamp);
  check(before_camera.valid && before_camera.value_count == 9 && before_camera.factor_count == 9,
        "GPS navigation states or immediate commits are incorrect");
  graph.materialize_clone(next.timestamp);
  graph.finish_update();
  check(graph.get_estimate(next.timestamp).value_count == before_camera.value_count, "Coincident camera created a duplicate GPS frame");
  graph.materialize_clone(2.45);
  graph.finish_update();
  check(graph.get_estimate(2.45).valid, "Camera propagation failed after GPS updates");
}

void check_manager(const std::string &config_path, VioManagerOptions::FilterType mode, bool use_qr) {
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  VioManagerOptions options;
  options.print_and_load(parser);
  check(parser->successful(), "Could not parse the GPS test configuration");
  options.filter_type = mode;
  options.use_qr = use_qr;
  options.save_results = false;
  options.record_timing_information = false;
  options.use_multi_threading_subs = false;
  options.use_multi_threading_pubs = false;
  options.num_opencv_threads = 0;
  options.max_gps_init_time = 0.2;
  ov_core::GPSData gps;
  gps.timestamp = 1 + options.calib_camimu_dt;
  gps.lla << 40, -111, 1000;
  gps.velocity << 1, 2, 0;
  gps.cov_position = Eigen::Matrix3d::Identity();
  gps.cov_velocity = Eigen::Matrix3d::Identity();
  VioManager manager(options);
  check(!manager.feed_measurement_gps(gps), "GPS applied before VIO initialization");
  Eigen::Matrix<double, 17, 1> initial = Eigen::Matrix<double, 17, 1>::Zero();
  initial(0) = 1;
  initial(4) = 1;
  manager.initialize_with_gt(initial);
  ov_core::ImuData imu;
  imu.wm.setZero();
  imu.am << 0, 0, options.gravity_mag;
  for (int i = 0; i <= 200; ++i) {
    imu.timestamp = 0.5 + 0.01 * i;
    manager.feed_measurement_imu(imu);
  }
  auto invalid = gps;
  invalid.timestamp -= 0.01;
  check(!manager.feed_measurement_gps(invalid), "GPS applied before the initialization window");
  invalid = gps;
  invalid.lla(0) = 91;
  check(!manager.feed_measurement_gps(invalid), "Invalid latitude was accepted");
  invalid = gps;
  invalid.velocity(0) = std::numeric_limits<double>::quiet_NaN();
  check(!manager.feed_measurement_gps(invalid), "Nonfinite velocity was accepted");
  invalid = gps;
  invalid.timestamp = std::numeric_limits<double>::quiet_NaN();
  check(!manager.feed_measurement_gps(invalid), "Nonfinite GPS timestamp was accepted");
  invalid = gps;
  invalid.cov_position(0, 0) = -1;
  check(!manager.feed_measurement_gps(invalid), "Nonpositive GPS covariance was accepted");
  invalid = gps;
  invalid.cov_velocity(0, 1) = 0.2;
  check(!manager.feed_measurement_gps(invalid), "Asymmetric GPS covariance was accepted");
  check(manager.feed_measurement_gps(gps), "GPS rejected at the initialization window start");
  check(!manager.feed_measurement_gps(gps), "Duplicate GPS timestamp was accepted");
  gps.timestamp = 1.05 + options.calib_camimu_dt;
  check(manager.feed_measurement_gps(gps), "First GPS sample between cameras was rejected");
  gps.timestamp = 1.1 + options.calib_camimu_dt;
  check(manager.feed_measurement_gps(gps), "Second GPS sample between cameras was rejected");
  check(manager.get_state()->_clones_IMU.empty(), "GPS created OpenVINS camera clones");
  manager.feed_measurement_simulation(1.1, {0}, {{}});
  check(manager.get_state()->_clones_IMU.count(1.1) == 1, "Camera coincident with GPS did not create its clone");
  check(manager.get_estimator_result().valid, "Selected estimator was invalid after coincident GPS and camera");
  gps.timestamp = 1.09 + options.calib_camimu_dt;
  check(!manager.feed_measurement_gps(gps), "Out-of-order GPS sample was accepted");
  gps.timestamp = 1.2 + options.calib_camimu_dt;
  check(!manager.feed_measurement_gps(gps), "GPS applied at the exclusive cutoff");
  gps.timestamp += 0.01;
  check(!manager.feed_measurement_gps(gps), "GPS applied after the cutoff");
  manager.feed_measurement_simulation(1.25, {0}, {{}});
  check(manager.get_estimator_result().valid, "Camera updates failed after the GPS cutoff");
  check(manager.successful_resets() == 0 && manager.skipped_resets() == 0, "GPS triggered a hybrid reset");

  options.max_gps_init_time = 0;
  VioManager disabled(options);
  disabled.initialize_with_gt(initial);
  gps.timestamp = 1 + options.calib_camimu_dt;
  check(!disabled.feed_measurement_gps(gps), "Disabled GPS assistance applied a measurement");
}

int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::invalid_argument("Usage: test_gps_init <HoloOcean estimator_config.yaml>");
    ov_core::Printer::setPrintLevel("WARNING");
    for (bool use_qr : {false, true}) {
      check_global_update(use_qr);
      for (auto mode :
           {VioManagerOptions::FilterType::OPENVINS, VioManagerOptions::FilterType::FACTOR_GRAPH, VioManagerOptions::FilterType::HYBRID})
        check_manager(argv[1], mode, use_qr);
    }
    std::cout << "GPS initialization checks passed (all estimators, Cholesky and QR)\n";
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
