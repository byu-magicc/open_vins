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

} // namespace

bool UpdaterRange::update(std::shared_ptr<State> first, std::shared_ptr<State> second, RangeMethod method, double range, double variance) {
  if (first == second || !std::isfinite(range) || range < 0 || !std::isfinite(variance) || variance <= 0)
    throw std::invalid_argument("Invalid range exchange");
  if (method != RangeMethod::DMV && method != RangeMethod::PECMV)
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
