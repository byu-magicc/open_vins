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

#include "EstimatorResults.h"
#include "factor_graph/FactorGraphTypes.h"

#include <boost/filesystem.hpp>
#include <iomanip>
#include <stdexcept>

using namespace ov_msckf;

namespace {
const char *state_header = ",q_x,q_y,q_z,q_w,p_x,p_y,p_z,v_x,v_y,v_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z";

void write_state(std::ofstream &stream, const Eigen::Matrix<double, 16, 1> &state) {
  for (int index = 0; index < state.rows(); ++index)
    stream << ',' << state(index);
}
} // namespace

EstimatorResults::EstimatorResults(const std::string &directory) : directory(directory) {
  boost::filesystem::create_directories(directory);
  boost::filesystem::remove(boost::filesystem::path(directory) / "groundtruth.csv");
  boost::filesystem::remove(boost::filesystem::path(directory) / "camera_intrinsics.csv");
  estimates.open((boost::filesystem::path(directory) / "estimate.csv").string());
  if (!estimates)
    throw std::runtime_error("Unable to open estimator results in " + directory);
  estimates << std::setprecision(17) << "timestamp,valid" << state_header;
  for (int row = 0; row < 15; ++row)
    for (int column = 0; column < 15; ++column)
      estimates << ",cov_" << row << '_' << column;
  estimates << '\n';
}

void EstimatorResults::record(const EstimatorResult &result) {
  estimates << result.timestamp << ',' << result.valid;
  write_state(estimates, result.state);
  for (int row = 0; row < 15; ++row)
    for (int column = 0; column < 15; ++column)
      estimates << ',' << result.covariance(row, column);
  estimates << '\n';
  estimates.flush();
  if (result.valid && !result.camera_intrinsics.empty()) {
    if (!camera_intrinsics.is_open()) {
      camera_intrinsics.open((boost::filesystem::path(directory) / "camera_intrinsics.csv").string());
      if (!camera_intrinsics)
        throw std::runtime_error("Unable to open camera intrinsics results in " + directory);
      camera_intrinsics << std::setprecision(17) << "timestamp,camera_id,fx,fy,cx,cy,d0,d1,d2,d3\n";
    }
    for (size_t camera_id = 0; camera_id < result.camera_intrinsics.size(); ++camera_id) {
      camera_intrinsics << result.timestamp << ',' << camera_id;
      for (int index = 0; index < 8; ++index)
        camera_intrinsics << ',' << result.camera_intrinsics.at(camera_id)(index);
      camera_intrinsics << '\n';
    }
    camera_intrinsics.flush();
  }
}

void EstimatorResults::record_groundtruth(double timestamp, const Eigen::Matrix<double, 16, 1> &state) {
  if (!truth.is_open()) {
    truth.open((boost::filesystem::path(directory) / "groundtruth.csv").string());
    if (!truth)
      throw std::runtime_error("Unable to open ground truth results in " + directory);
    truth << std::setprecision(17) << "timestamp" << state_header << '\n';
  }
  truth << timestamp;
  write_state(truth, state);
  truth << '\n';
  truth.flush();
}
