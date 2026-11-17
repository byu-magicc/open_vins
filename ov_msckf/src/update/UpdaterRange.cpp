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

#include "UpdaterRange.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/print.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ov_msckf {
namespace {

// Exact pose cloning makes the full covariance singular. Count its support after scaling out units.
// Used for both prior/posterior validation; never add artificial noise or alter the covariance.
int covariance_rank(const Eigen::MatrixXd &P) {
  if (!P.allFinite())
    throw std::runtime_error("Range covariance is not finite");
  Eigen::VectorXd scale = P.diagonal();
  for (int i = 0; i < scale.size(); ++i) {
    if (scale(i) < 0)
      throw std::runtime_error("Range covariance has a negative variance");
    scale(i) = scale(i) > 0 ? 1.0 / std::sqrt(scale(i)) : 1.0;
  }
  const Eigen::MatrixXd normalized = scale.asDiagonal() * P * scale.asDiagonal();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(normalized, Eigen::EigenvaluesOnly);
  if (eig.info() != Eigen::Success)
    throw std::runtime_error("Range covariance eigendecomposition failed");
  const double tolerance = 64 * P.rows() * std::numeric_limits<double>::epsilon() * std::max(1.0, eig.eigenvalues().maxCoeff());
  if (eig.eigenvalues().minCoeff() < -tolerance)
    throw std::runtime_error("Range covariance is not positive semidefinite");
  return (eig.eigenvalues().array() > tolerance).count();
}

// Compute one agent's posterior. False means the optimum leaves its belief unchanged.
bool pecmv_update(const Eigen::MatrixXd &P, const Eigen::VectorXd &u, double a, double b, double residual, double variance,
                  Eigen::VectorXd &dx, Eigen::MatrixXd &posterior) {
  // Eq. (31) in whitened coordinates: ||C|| <= kappa. The determinant lemma
  // reduces the objective to minimizing (A+t)^2 / (R+A^2+B^2+2At).
  const double A = std::sqrt(a), B = std::sqrt(b), bound = (1 - 1e-8) * B;
  const double t = A <= bound ? -A : std::max(-bound, std::min(bound, -(variance + b) / A));
  const double S = variance + (A + t) * (A + t) + (B - t) * (B + t);
  const double c = 1 + t / A;
  if (c == 0)
    return false;
  const double gain_scale = c / S, covariance_scale = c * c / S;
  dx = (gain_scale * residual) * u;
  posterior = P;
  posterior.noalias() -= covariance_scale * u * u.transpose();
  return true;
}

bool dmv_update(const Eigen::MatrixXd &P, const Eigen::VectorXd &u, double a, double b, double residual, double variance, int rank,
                Eigen::VectorXd &dx, Eigen::MatrixXd &posterior) {
  double gain_scale, covariance_scale;
  posterior = P;
  if (b == 0) {
    // A known peer coordinate gives the ordinary scalar EKF update (omega = 1).
    gain_scale = covariance_scale = 1 / (a + variance);
  } else {
    // DMV Eq. (15), gamma=1, on rank(P) independent coordinates.
    // Its convex objective has derivative -(r-1)/w - R/D - N'/N,
    // D=b+R(1-w), N=a(1-w)+bw+Rw(1-w). Bisect its zero.
    if (a <= rank * b)
      return false; // Exact omega=1 limit: no update.
    if (rank == 1 && a * b >= (b + variance) * (b + variance)) {
      dx = (residual / a) * u;
      posterior = ((b + variance) / a) * P; // Exact omega=0 limit.
      return true;
    }
    double lower = 0, upper = 1;
    for (int iteration = 0; iteration < 80; ++iteration) {
      const double w = 0.5 * (lower + upper);
      if (w == lower || w == upper)
        break;
      const double D = b + variance * (1 - w);
      const double N = a * (1 - w) + b * w + variance * w * (1 - w);
      const double derivative = -(rank - 1) / w - variance / D - (b - a + variance * (1 - 2 * w)) / N;
      if (derivative < 0)
        lower = w;
      else
        upper = w;
    }
    const double w = 0.5 * (lower + upper), T = b / (1 - w) + variance;
    gain_scale = 1 / (a + w * T);
    covariance_scale = gain_scale / w;
    posterior /= w;
  }
  dx = (gain_scale * residual) * u;
  posterior.noalias() -= covariance_scale * u * u.transpose();
  return true;
}

// Lift one endpoint's permanent-state gain numerator to its complete MSCKF state.
// The conditional-independence approximation supplies correlations to clones and landmarks.
void isokf_local_update(const Eigen::MatrixXd &prior, const Eigen::VectorXd &permanent_gain_numerator, double residual,
                        double innovation_variance, Eigen::VectorXd &correction, Eigen::MatrixXd &posterior) {
  const int permanent_size = permanent_gain_numerator.size();
  const Eigen::MatrixXd lift =
      StateHelper::isokf_transform(prior.topLeftCorner(permanent_size, permanent_size), prior.leftCols(permanent_size));
  const Eigen::VectorXd gain_numerator = lift * permanent_gain_numerator;

  correction = gain_numerator * (residual / innovation_variance);
  posterior = prior - gain_numerator * gain_numerator.transpose() / innovation_variance;
  posterior = (0.5 * (posterior + posterior.transpose())).eval();
  covariance_rank(posterior);
  if (!correction.allFinite())
    throw std::runtime_error("IsoKF correction is not finite");
}

} // namespace

void UpdaterRange::isokf_update(const std::shared_ptr<State> states[2], const Eigen::MatrixXd prior[2], const Eigen::Vector3d &direction,
                                double residual, double variance) {
  const auto &first = states[0];
  const auto &second = states[1];
  const int first_size = first->_permanent_size;
  const int second_size = second->_permanent_size;

  // Reconstruct the permanent cross-covariance from the two locally maintained factors.
  // A pair that has never communicated starts uncorrelated.
  const auto first_factor = first->_isokf_correlations.find(second);
  const auto second_factor = second->_isokf_correlations.find(first);
  if ((first_factor == first->_isokf_correlations.end()) != (second_factor == second->_isokf_correlations.end()))
    throw std::logic_error("IsoKF has a one-sided correlation record");
  Eigen::MatrixXd cross_covariance = Eigen::MatrixXd::Zero(first_size, second_size);
  if (first_factor != first->_isokf_correlations.end())
    cross_covariance.noalias() = first_factor->second * second_factor->second.transpose();

  // Algorithm 3: form the joint range update on the two permanent state blocks.
  Eigen::MatrixXd joint_prior(first_size + second_size, first_size + second_size);
  joint_prior << prior[0].topLeftCorner(first_size, first_size), cross_covariance, cross_covariance.transpose(),
      prior[1].topLeftCorner(second_size, second_size);
  covariance_rank(joint_prior);

  Eigen::VectorXd range_jacobian = Eigen::VectorXd::Zero(first_size + second_size);
  range_jacobian.segment<3>(first->_imu->p()->id()) = direction;
  range_jacobian.segment<3>(first_size + second->_imu->p()->id()) = -direction;
  const Eigen::VectorXd joint_gain_numerator = joint_prior * range_jacobian;
  const double innovation_variance = range_jacobian.dot(joint_gain_numerator) + variance;
  if (!std::isfinite(innovation_variance) || innovation_variance <= 0)
    throw std::runtime_error("IsoKF innovation variance is invalid");

  // Stage both full-state posteriors and factor maps before modifying either endpoint.
  Eigen::VectorXd correction[2];
  Eigen::MatrixXd posterior[2];
  decltype(first->_isokf_correlations) factors[] = {first->_isokf_correlations, second->_isokf_correlations};
  for (int i = 0; i < 2; ++i) {
    const int size = states[i]->_permanent_size;
    const int offset = i == 0 ? 0 : first_size;
    isokf_local_update(prior[i], joint_gain_numerator.segment(offset, size), residual, innovation_variance, correction[i], posterior[i]);

    // Eq. (26): update correlations to third parties with P_ss^+ P_ss^-1.
    // The communicating peer gets its exact pair factor below instead.
    const Eigen::MatrixXd transform =
        StateHelper::isokf_transform(prior[i].topLeftCorner(size, size), posterior[i].topLeftCorner(size, size));
    for (auto it = factors[i].begin(); it != factors[i].end();) {
      if (it->first.expired()) {
        it = factors[i].erase(it);
        continue;
      }
      if (it->first.lock() != states[1 - i])
        it->second = (transform * it->second).eval();
      if (!it->second.allFinite())
        throw std::runtime_error("IsoKF factor is not finite");
      ++it;
    }
  }

  // Store the updated permanent cross-block using the paper's asymmetric C / I decomposition.
  // Their product reconstructs the pair covariance at the next exchange.
  factors[0][second] =
      cross_covariance - joint_gain_numerator.head(first_size) * joint_gain_numerator.tail(second_size).transpose() / innovation_variance;
  factors[1][first] = Eigen::MatrixXd::Identity(second_size, second_size);

  StateHelper::apply_update(first, correction[0], posterior[0]);
  StateHelper::apply_update(second, correction[1], posterior[1]);
  first->_isokf_correlations = std::move(factors[0]);
  second->_isokf_correlations = std::move(factors[1]);
}

bool UpdaterRange::update(std::shared_ptr<State> first, std::shared_ptr<State> second, RangeMethod method, double range, double variance) {
  if (first == second || !std::isfinite(range) || range < 0 || !std::isfinite(variance) || variance <= 0)
    throw std::invalid_argument("Invalid range exchange");
  if (method != RangeMethod::DMV && method != RangeMethod::PECMV && method != RangeMethod::ISOKF)
    throw std::invalid_argument("Unsupported EKF range method");

  const Eigen::Vector3d difference = first->_imu->pos() - second->_imu->pos();
  const double distance = difference.norm();
  if (!std::isfinite(distance))
    throw std::runtime_error("Range positions are not finite");
  if (distance <= 1e-9) {
    PRINT_WARNING("[RANGE]: skipping coincident estimated positions\n");
    return false;
  }
  const Eigen::Vector3d direction = difference / distance;
  const double residual = range - distance;
  const std::shared_ptr<State> states[] = {first, second};
  Eigen::MatrixXd prior[2], posterior[2];
  Eigen::VectorXd u[2], dx[2];
  double projected[2];
  int rank[2];
  bool changed[2] = {false, false};
  for (int i = 0; i < 2; ++i) {
    prior[i] = StateHelper::get_full_covariance(states[i]);
    prior[i] = (0.5 * (prior[i] + prior[i].transpose())).eval();
    rank[i] = covariance_rank(prior[i]);
    const int position_id = states[i]->_imu->p()->id();
    const Eigen::Vector3d H = i == 0 ? direction : -direction;
    u[i] = prior[i].middleCols(position_id, 3) * H;
    projected[i] = std::max(0.0, H.dot(u[i].segment<3>(position_id)));
  }

  if (method == RangeMethod::ISOKF) {
    isokf_update(states, prior, direction, residual, variance);
    return true;
  }

  for (int i = 0; i < 2; ++i) {
    const double a = projected[i], b = projected[1 - i];
    if (a == 0)
      continue;
    if (method == RangeMethod::PECMV)
      changed[i] = pecmv_update(prior[i], u[i], a, b, residual, variance, dx[i], posterior[i]);
    else
      changed[i] = dmv_update(prior[i], u[i], a, b, residual, variance, rank[i], dx[i], posterior[i]);
    if (!changed[i])
      continue;
    posterior[i] = (0.5 * (posterior[i] + posterior[i].transpose())).eval();
    covariance_rank(posterior[i]);
    if (!dx[i].allFinite())
      throw std::runtime_error("Range correction is not finite");
  }

  // Both candidates have passed validation. Never use one agent's new belief to update the other.
  for (int i = 0; i < 2; ++i) {
    if (changed[i])
      StateHelper::apply_update(states[i], dx[i], posterior[i]);
  }
  return changed[0] || changed[1];
}
} // namespace ov_msckf
