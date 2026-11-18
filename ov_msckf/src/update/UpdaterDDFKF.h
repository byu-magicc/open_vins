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

#ifndef OV_MSCKF_UPDATERDDFKF_H
#define OV_MSCKF_UPDATERDDFKF_H

#include <Eigen/Core>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "types/Type.h"

namespace ov_msckf {
class State;

/// Range-only DDF-KF, ported from 2D_cooperative_localization_sim.
class UpdaterDDFKF {
public:
  UpdaterDDFKF(std::shared_ptr<State> state, size_t agent_id, int max_range_pair_count);
  void communicate(UpdaterDDFKF &neighbor, double event_time, double range, double variance);

private:
  struct Endpoint {
    size_t agent;
    double time;
    bool operator<(const Endpoint &other) const { return agent < other.agent || (agent == other.agent && time < other.time); }
  };
  struct Range {
    Endpoint a, b;
    double time, z, variance;
  };
  struct Summary {
    double creation;
    std::vector<double> times;
    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance;
    Summary retain(const std::set<double> &keep) const;
  };
  struct Measurement {
    std::vector<double> times;
    Eigen::VectorXd z, prediction, linearization;
    Eigen::MatrixXd H, Q;
  };
  Summary summarize() const;
  Measurement construct() const;
  void apply(const Measurement &measurement, double sign);
  void replace(Measurement measurement);
  void trim();

  std::shared_ptr<State> state;
  size_t id;
  int limit;
  std::map<double, std::shared_ptr<ov_type::Type>> shared;
  std::map<size_t, Summary> summaries;
  std::vector<Range> ranges;
  Measurement previous;
};
} // namespace ov_msckf
#endif
