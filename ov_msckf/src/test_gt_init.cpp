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
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

using namespace ov_msckf;

int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::invalid_argument("Usage: test_gt_init <HoloOcean estimator_config.yaml>");
    ov_core::Printer::setPrintLevel("WARNING");
    auto check = [](bool condition, const char *message) {
      if (!condition)
        throw std::runtime_error(message);
    };
    for (auto mode :
         {VioManagerOptions::FilterType::OPENVINS, VioManagerOptions::FilterType::FACTOR_GRAPH, VioManagerOptions::FilterType::HYBRID}) {
      auto parser = std::make_shared<ov_core::YamlParser>(argv[1]);
      VioManagerOptions options;
      options.print_and_load(parser);
      check(parser->successful(), "Could not parse initialization test configuration");
      options.filter_type = mode;
      options.save_results = false;
      options.record_timing_information = false;
      options.use_multi_threading_subs = false;
      options.use_multi_threading_pubs = false;
      options.num_opencv_threads = 0;
      VioManager manager(options);
      Eigen::Matrix<double, 17, 1> initial = Eigen::Matrix<double, 17, 1>::Zero();
      initial(0) = 90.0;
      const Eigen::Matrix3d R_GtoI = (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(-1.1, Eigen::Vector3d::UnitZ()))
                                         .toRotationMatrix();
      initial.segment<4>(1) = ov_core::rot_2_quat(R_GtoI);
      initial.segment<3>(5) << -650, 800, 44;
      initial.segment<3>(8) << -9, 10, 0.5;
      initial.segment<3>(11) << 0.001, -0.002, 0.003;
      initial.segment<3>(14) << 0.01, 0.02, -0.01;
      manager.initialize_with_gt(initial);
      auto state = manager.get_state();
      check(state->_timestamp == initial(0) && manager.initialized_time() == initial(0), "Initialization timestamp changed");
      check((state->_imu->value() - initial.tail<16>()).norm() < 1e-12, "Global initial state changed");
      check((state->_imu->fej() - initial.tail<16>()).norm() < 1e-12, "Initial FEJ state changed");
      check((state->_imu->Rot() - R_GtoI).norm() < 1e-12, "Initial attitude convention is incorrect");
      const auto covariance = StateHelper::get_marginal_covariance(state, {state->_imu});
      check((covariance.block<3, 3>(0, 0) - std::pow(0.017, 2) * Eigen::Matrix3d::Identity()).norm() < 1e-12,
            "Orientation covariance changed");
      check((covariance.block<3, 3>(3, 3) - std::pow(0.05, 2) * Eigen::Matrix3d::Identity()).norm() < 1e-12, "Position covariance changed");
      check((covariance.block<3, 3>(6, 6) - std::pow(0.01, 2) * Eigen::Matrix3d::Identity()).norm() < 1e-12, "Velocity covariance changed");
      const auto result = manager.get_estimator_result();
      check(result.valid && (result.state - initial.tail<16>()).norm() < 1e-8, "Selected estimator recentered the global initial state");
      check((result.covariance - covariance).norm() < 1e-8, "Graph initialization covariance disagrees with OpenVINS");
      for (int i = 0; i <= 30; ++i) {
        ov_core::ImuData imu;
        imu.timestamp = initial(0) + options.calib_camimu_dt - 0.01 + 0.005 * i;
        imu.wm = initial.segment<3>(11);
        imu.am = R_GtoI * Eigen::Vector3d(0, 0, options.gravity_mag) + initial.segment<3>(14);
        manager.feed_measurement_imu(imu);
      }
      manager.feed_measurement_simulation(initial(0) + 0.05, {0}, {{}});
      check(state->_clones_IMU.count(initial(0) + 0.05) == 1, "Camera processing did not start from the supplied state");
      const auto propagated = manager.get_estimator_result();
      check(propagated.valid && propagated.state.allFinite() && propagated.covariance.allFinite(),
            "Propagation after truth startup failed");
      check((propagated.state.segment<3>(4) - initial.segment<3>(5) - 0.05 * initial.segment<3>(8)).norm() < 1e-6,
            "Global position was recentered or propagated incorrectly");
    }
    std::cout << "Ground-truth initialization checks passed (all estimators)\n";
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
