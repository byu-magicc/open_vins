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

#ifndef OV_MSCKF_ESTIMATORRESULTS_H
#define OV_MSCKF_ESTIMATORRESULTS_H

#include <Eigen/Core>
#include <fstream>
#include <string>

namespace ov_msckf {
struct EstimatorResult;

/** Selected-estimator CSV output shared by all runners. */
class EstimatorResults {
public:
  explicit EstimatorResults(const std::string &directory);
  void record(const EstimatorResult &result);
  void record_groundtruth(double timestamp, const Eigen::Matrix<double, 16, 1> &state);

private:
  std::string directory;
  std::ofstream estimates;
  std::ofstream truth;
};
} // namespace ov_msckf

#endif // OV_MSCKF_ESTIMATORRESULTS_H
