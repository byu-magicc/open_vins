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

#include "UpdaterDDFKF.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/print.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

using namespace ov_msckf;

namespace {
// The same Joseph update is used for summaries, fleet estimates, and live-state updates.
// Negative measurement noise removes a stored, frozen linear measurement.
Eigen::VectorXd update(Eigen::MatrixXd &P, const Eigen::MatrixXd &H, const Eigen::VectorXd &residual, const Eigen::MatrixXd &Q) {
  if (H.rows() == 0)
    return Eigen::VectorXd::Zero(P.rows());
  const Eigen::MatrixXd M = P * H.transpose();
  const Eigen::MatrixXd S = H * M + Q;
  Eigen::LDLT<Eigen::MatrixXd> solve(S);
  if (solve.info() != Eigen::Success || (solve.vectorD().array() == 0).any())
    throw std::runtime_error("DDF-KF singular innovation covariance");
  const Eigen::MatrixXd K = solve.solve(M.transpose()).transpose();
  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * H;
  P = (A * P * A.transpose() + K * Q * K.transpose()).eval();
  P = (0.5 * (P + P.transpose())).eval();
  Eigen::VectorXd dx = K * residual;
  if (!P.allFinite() || !dx.allFinite())
    throw std::runtime_error("DDF-KF nonfinite update");
  return dx;
}

Eigen::RowVector3d direction(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  const double distance = (a - b).norm();
  if (!std::isfinite(distance) || distance <= 1e-9)
    throw std::runtime_error("DDF-KF undefined historical range geometry");
  return (a - b).transpose() / distance;
}
} // namespace

UpdaterDDFKF::UpdaterDDFKF(std::shared_ptr<State> state, size_t agent_id, int max_range_pair_count)
    : state(std::move(state)), id(agent_id), limit(max_range_pair_count) {
  if (limit < 0)
    throw std::invalid_argument("max_range_pair_count must be nonnegative");
}

UpdaterDDFKF::Summary UpdaterDDFKF::Summary::retain(const std::set<double> &keep) const {
  Summary result;
  result.creation = creation;
  std::vector<int> rows;
  for (size_t i = 0; i < times.size(); ++i) {
    if (keep.count(times[i])) {
      result.times.push_back(times[i]);
      for (int j = 0; j < 3; ++j)
        rows.push_back(3 * i + j);
    }
  }
  result.mean.resize(rows.size());
  result.covariance.resize(rows.size(), rows.size());
  for (size_t i = 0; i < rows.size(); ++i) {
    result.mean(i) = mean(rows[i]);
    for (size_t j = 0; j < rows.size(); ++j)
      result.covariance(i, j) = covariance(rows[i], rows[j]);
  }
  return result;
}

UpdaterDDFKF::Summary UpdaterDDFKF::summarize() const {
  Summary result;
  result.creation = state->_timestamp;
  std::vector<std::shared_ptr<ov_type::Type>> order;
  result.mean.resize(3 * shared.size());
  int row = 0;
  for (const auto &entry : shared) {
    result.times.push_back(entry.first);
    order.push_back(entry.second);
    result.mean.segment<3>(row) = entry.second->value();
    row += 3;
  }
  result.covariance = StateHelper::get_marginal_covariance(state, order);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(previous.times.size(), result.mean.size());
  Eigen::VectorXd predicted = previous.prediction;
  for (size_t i = 0; i < previous.times.size(); ++i) {
    const int col = 3 * (std::find(result.times.begin(), result.times.end(), previous.times[i]) - result.times.begin());
    H.block<1, 3>(i, col) += previous.H.block<1, 3>(i, 3 * i);
    predicted(i) += (previous.H.block<1, 3>(i, 3 * i) * (result.mean.segment<3>(col) - previous.linearization.segment<3>(3 * i)))(0);
  }
  result.mean += update(result.covariance, H, previous.z - predicted, -previous.Q);
  return result;
}

void UpdaterDDFKF::trim() {
  if (limit == 0)
    return;
  std::map<size_t, std::vector<size_t>> by_partner;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const auto &m = ranges[i];
    if (m.a.agent == id || m.b.agent == id)
      by_partner[m.a.agent == id ? m.b.agent : m.a.agent].push_back(i);
  }
  std::map<size_t, std::set<double>> dropped;
  for (const auto &entry : by_partner) {
    for (int i = 0; i < static_cast<int>(entry.second.size()) - limit; ++i) {
      const auto &m = ranges[entry.second[i]];
      dropped[m.a.agent].insert(m.a.time);
      dropped[m.b.agent].insert(m.b.time);
    }
  }
  for (const auto &entry : dropped) {
    auto found = summaries.find(entry.first);
    if (found == summaries.end())
      continue;
    std::set<double> keep(found->second.times.begin(), found->second.times.end());
    for (double time : entry.second)
      keep.erase(time);
    found->second = found->second.retain(keep);
  }
  ranges.erase(std::remove_if(ranges.begin(), ranges.end(),
                              [&](const Range &m) {
                                return (m.a.agent == id || m.b.agent == id) && (dropped[id].count(m.a.agent == id ? m.a.time : m.b.time));
                              }),
               ranges.end());
}

UpdaterDDFKF::Measurement UpdaterDDFKF::construct() const {
  int size = 0;
  for (const auto &entry : summaries)
    if (entry.first != id)
      size += entry.second.mean.size();
  Eigen::VectorXd mean(size);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(size, size);
  std::map<Endpoint, int> coordinates;
  int offset = 0;
  for (const auto &entry : summaries) {
    const auto &s = entry.second;
    if (entry.first != id) {
      mean.segment(offset, s.mean.size()) = s.mean;
      covariance.block(offset, offset, s.mean.size(), s.mean.size()) = s.covariance;
    }
    for (size_t i = 0; i < s.times.size(); ++i)
      coordinates[{entry.first, s.times[i]}] = (entry.first == id ? 0 : offset) + 3 * i;
    if (entry.first != id)
      offset += s.mean.size();
  }
  std::vector<Range> local;
  for (const auto &m : ranges) {
    const int a = coordinates.at(m.a), b = coordinates.at(m.b);
    if (m.a.agent == id || m.b.agent == id) {
      local.push_back(m);
      continue;
    }
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, size);
    H.block<1, 3>(0, a) = direction(mean.segment<3>(a), mean.segment<3>(b));
    H.block<1, 3>(0, b) = -H.block<1, 3>(0, a);
    Eigen::VectorXd residual(1);
    residual(0) = m.z - (mean.segment<3>(a) - mean.segment<3>(b)).norm();
    mean += update(covariance, H, residual, Eigen::MatrixXd::Constant(1, 1, m.variance));
  }
  Measurement result;
  const int n = local.size();
  result.z.resize(n);
  result.prediction.resize(n);
  result.linearization.resize(3 * n);
  result.H = Eigen::MatrixXd::Zero(n, 3 * n);
  Eigen::MatrixXd Hb = Eigen::MatrixXd::Zero(n, size);
  result.Q = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    const auto &m = local[i];
    const Endpoint own = m.a.agent == id ? m.a : m.b;
    const Endpoint peer = m.a.agent == id ? m.b : m.a;
    const Eigen::Vector3d a = summaries.at(id).mean.segment<3>(coordinates.at(own));
    const Eigen::Vector3d b = mean.segment<3>(coordinates.at(peer));
    result.times.push_back(own.time);
    result.z(i) = m.z;
    result.prediction(i) = (a - b).norm();
    result.linearization.segment<3>(3 * i) = a;
    result.H.block<1, 3>(i, 3 * i) = direction(a, b);
    Hb.block<1, 3>(i, coordinates.at(peer)) = -result.H.block<1, 3>(i, 3 * i);
    result.Q(i, i) = m.variance;
  }
  result.Q.noalias() += Hb * covariance * Hb.transpose();
  return result;
}

void UpdaterDDFKF::apply(const Measurement &m, double sign) {
  if (m.times.empty())
    return;
  Eigen::MatrixXd P = StateHelper::get_full_covariance(state);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m.times.size(), P.rows());
  Eigen::VectorXd predicted = m.prediction;
  for (size_t i = 0; i < m.times.size(); ++i) {
    const auto &variable = shared.at(m.times[i]);
    H.block<1, 3>(i, variable->id()) += m.H.block<1, 3>(i, 3 * i);
    predicted(i) += (m.H.block<1, 3>(i, 3 * i) * (variable->value() - m.linearization.segment<3>(3 * i)))(0);
  }
  const Eigen::VectorXd dx = update(P, H, m.z - predicted, sign * m.Q);
  StateHelper::apply_update(state, dx, P);
}

void UpdaterDDFKF::replace(Measurement measurement) {
  apply(previous, -1);
  const std::set<double> keep(measurement.times.begin(), measurement.times.end());
  for (auto it = shared.begin(); it != shared.end();) {
    if (!keep.count(it->first)) {
      StateHelper::marginalize(state, it->second);
      it = shared.erase(it);
    } else {
      ++it;
    }
  }
  apply(measurement, 1);
  previous = std::move(measurement);
}

void UpdaterDDFKF::communicate(UpdaterDDFKF &neighbor, double event_time, double range, double variance) {
  if (&neighbor == this || neighbor.id == id || !std::isfinite(event_time) || !std::isfinite(range) || range < 0 ||
      !std::isfinite(variance) || variance <= 0)
    throw std::invalid_argument("Invalid DDF-KF range exchange");
  if ((state->_imu->pos() - neighbor.state->_imu->pos()).norm() <= 1e-9) {
    PRINT_WARNING("[DDF-KF]: skipping coincident estimated positions\n");
    return;
  }
  Range measurement{{id, state->_timestamp}, {neighbor.id, neighbor.state->_timestamp}, event_time, range, variance};
  if (measurement.b < measurement.a)
    std::swap(measurement.a, measurement.b);
  auto key = [](const Range &m) { return std::make_tuple(m.time, m.a.agent, m.a.time, m.b.agent, m.b.time); };
  for (auto *owner : {this, &neighbor}) {
    if (std::any_of(owner->ranges.begin(), owner->ranges.end(), [&](const Range &m) { return key(m) == key(measurement); }))
      throw std::invalid_argument("Duplicate DDF-KF range event");
    if (!owner->shared.count(owner->state->_timestamp))
      owner->shared[owner->state->_timestamp] = StateHelper::clone(owner->state, owner->state->_imu->p());
    owner->summaries[owner->id] = owner->summarize();
    owner->ranges.push_back(measurement);
    std::sort(owner->ranges.begin(), owner->ranges.end(), [&](const Range &a, const Range &b) { return key(a) < key(b); });
    owner->trim();
  }
  auto merged = summaries;
  for (const auto &entry : neighbor.summaries) {
    auto found = merged.find(entry.first);
    if (found == merged.end() || entry.second.creation > found->second.creation) {
      merged[entry.first] = entry.second;
    } else if (entry.second.creation == found->second.creation) {
      std::set<double> common;
      for (double t : entry.second.times)
        if (std::find(found->second.times.begin(), found->second.times.end(), t) != found->second.times.end())
          common.insert(t);
      const Summary incoming = entry.second.retain(common), existing = found->second.retain(common);
      if (!(incoming.mean.array() == existing.mean.array()).all() || !(incoming.covariance.array() == existing.covariance.array()).all())
        throw std::runtime_error("Conflicting DDF-KF summaries");
      found->second = existing;
    }
  }
  auto history = ranges;
  history.insert(history.end(), neighbor.ranges.begin(), neighbor.ranges.end());
  std::sort(history.begin(), history.end(), [&](const Range &a, const Range &b) { return key(a) < key(b); });
  std::vector<Range> retained;
  std::map<size_t, std::set<double>> used;
  for (const auto &m : history) {
    if (!retained.empty() && key(retained.back()) == key(m)) {
      if (retained.back().z != m.z || retained.back().variance != m.variance)
        throw std::runtime_error("Conflicting DDF-KF ranges");
      continue;
    }
    bool available = true;
    for (const auto &endpoint : {m.a, m.b}) {
      const auto &times = merged.at(endpoint.agent).times;
      available &= std::find(times.begin(), times.end(), endpoint.time) != times.end();
    }
    if (available) {
      retained.push_back(m);
      used[m.a.agent].insert(m.a.time);
      used[m.b.agent].insert(m.b.time);
    }
  }
  for (auto &entry : merged)
    entry.second = entry.second.retain(used[entry.first]);
  summaries = neighbor.summaries = std::move(merged);
  ranges = neighbor.ranges = std::move(retained);
  Measurement own = construct(), peer = neighbor.construct();
  replace(std::move(own));
  neighbor.replace(std::move(peer));
}
