/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "core/VioManagerOptions.h"
#include "factor_graph/FactorGraphState.h"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/RangeFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <stdexcept>

namespace ov_msckf {

class SquaredErrorFactor : public gtsam::NoiseModelFactor1<double> {
public:
  SquaredErrorFactor(gtsam::Key key, double measurement)
      : gtsam::NoiseModelFactor1<double>(gtsam::noiseModel::Unit::Create(1), key), measurement(measurement) {}

  gtsam::Vector evaluateError(const double &value, boost::optional<gtsam::Matrix &> jacobian = boost::none) const override {
    if (jacobian)
      *jacobian = gtsam::Matrix::Constant(1, 1, 2 * value);
    return gtsam::Vector1(value * value - measurement);
  }

private:
  double measurement;
};

struct FactorGraphDistributedTest {
  static void check(bool condition, const char *message) {
    if (!condition)
      throw std::runtime_error(message);
  }

  static void run(bool use_qr) {
    VioManagerOptions options;
    options.state_options.num_cameras = 0;
    options.state_options.do_calib_imu_intrinsics = false;
    options.state_options.do_calib_imu_g_sensitivity = false;
    options.state_options.do_calib_camera_timeoffset = false;
    options.use_qr = use_qr;
    options.relinearize_skip = 100;
    std::vector<std::unique_ptr<FactorGraphState>> graphs;
    for (size_t i = 0; i < 3; ++i) {
      options.factor_graph_agent_id = i;
      graphs.emplace_back(new FactorGraphState(options));
      FactorGraphInitialization initial;
      initial.timestamp = 0;
      initial.imu_state(3) = 1;
      initial.imu_state(4) = 10 * i;
      initial.variable_names = {"imu"};
      initial.variable_dimensions = {15};
      initial.covariance = Eigen::Matrix<double, 15, 15>::Identity();
      graphs.back()->initialize(initial);
    }
    auto &a = *graphs[0];
    auto &b = *graphs[1];
    auto &c = *graphs[2];
    const auto b_key = b.declare_shared(0);
    const auto c_key = c.declare_shared(0);
    check(b_key != c_key, "Shared keys collided between vehicles");
    const auto b_summary = b.get_summary(0);
    check((b_summary.factor->information() - gtsam::Matrix3::Identity() / 1.0001).norm() < 1e-8,
          "Single-position summary disagrees with analytic marginal");

    // Exercise replacement before commitment, duplicate delivery, and stale delivery.
    a.update_summary(b_summary, 1);
    const auto newer_b = b.get_summary(0);
    const auto old_pending = a.cached_summaries.at(1).graph_factor;
    a.update_summary(newer_b, 1);
    check(std::find(a.pending_factors.begin(), a.pending_factors.end(), old_pending) == a.pending_factors.end(),
          "Old pending summary survived replacement");
    const auto new_pending = a.cached_summaries.at(1).graph_factor;
    a.update_summary(b_summary, 1);
    a.update_summary(newer_b, 1);
    check(a.cached_summaries.at(1).graph_factor == new_pending, "Stale or duplicate summary was accepted");
    a.commit();
    a.update_summary(b.get_summary(0), 1);
    check(a.pending_remove_factor_indices.size() == 1, "Committed summary was not queued for removal");
    a.commit();
    check(std::count(a.optimizer->getFactorsUnsafe().begin(), a.optimizer->getFactorsUnsafe().end(), new_pending) == 0,
          "Old committed summary survived replacement");

    a.communicate(b, 0, 0, 8, 1);
    const gtsam::Key pose = gtsam::Symbol('x', 0);
    check(a.optimizer->calculateEstimate<gtsam::Pose3>(pose).x() > 0.1, "Range did not correct owner");
    check(b.optimizer->calculateEstimate<gtsam::Pose3>(pose).x() < 9.9, "Returned summary did not correct non-owner");
    size_t ranges = 0;
    for (const auto &graph : graphs)
      for (const auto &factor : graph->optimizer->getFactorsUnsafe())
        ranges += dynamic_cast<gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> *>(factor.get()) != nullptr;
    check(ranges == 1, "Range measurement was duplicated");

    // The owner's only local information on b is a single range direction.
    const auto range_summary = a.get_summary(0);
    check(range_summary.factor->rows() == 1, "Range-only summary did not preserve rank deficiency");
    gtsam::GaussianFactorGraph gaussian;
    a.optimizer->addFactorsToGraph(&gaussian);
    const auto marginal = gaussian.marginal(gtsam::KeyVector{b_key});
    const gtsam::HessianFactor total(*marginal);
    const auto incoming = a.cached_summaries.at(1).graph_factor->linearize(a.optimizer->getLinearizationPoint());
    gtsam::GaussianFactorGraph difference;
    difference.push_back(boost::make_shared<gtsam::HessianFactor>(total));
    difference.push_back(incoming->negate());
    const gtsam::HessianFactor expected(difference);
    check((range_summary.factor->information() - expected.information()).norm() < 1e-8,
          "Summary subtraction disagrees with dense calculation");
    const gtsam::HessianFactor actual(*range_summary.factor);
    check((actual.linearTerm(actual.begin()) - expected.linearTerm(expected.begin())).norm() < 1e-8,
          "Summary linear term disagrees with dense calculation");

    // c learns b's newer summary via a, and a learns c directly.
    const auto old_version = a.cached_summaries.at(1).summary.version;
    a.communicate(b, 0, 0, 8.5, 1);
    check(a.cached_summaries.at(1).summary.version > old_version, "Same-epoch summary was not refreshed");
    a.communicate(c, 0, 0, 19, 1);
    check(c.cached_summaries.count(1) == 1, "Third-party summary was not forwarded");
    check(c.cached_summaries.at(1).summary.version == a.cached_summaries.at(1).summary.version,
          "Third-party summary version changed in transit");
    check(a.shared_keys.count(c_key) && a.local_shared_keys.count(c_key), "Range endpoint was not locally relevant");
    check(!c.local_shared_keys.count(b_key) && c.shared_keys.count(b_key), "Transit key became locally relevant");
    const auto stale_b = a.cached_summaries.at(1).summary;
    c.communicate(b, 0, 0, 10, 1);
    check(c.cached_summaries.at(1).summary.version > stale_b.version, "Third party did not refresh its own summary");
    a.communicate(c, 0, 0, 19, 1);
    check(a.cached_summaries.at(1).summary.version == c.cached_summaries.at(1).summary.version,
          "Newer third-party summary was not forwarded to owner");
    a.update_summary(stale_b, 1);
    check(a.cached_summaries.at(1).summary.version > stale_b.version, "Relayed stale summary replaced newer information");
    for (const auto &graph : graphs) {
      const auto summary = graph->get_summary(0);
      check(summary.values.size() == summary.factor->keys().size(), "Summary values do not match factor keys");
      for (auto key : summary.factor->keys())
        check(graph->local_shared_keys.count(key), "Summary retained a transit-only key");
      check(summary.factor->information().allFinite(), "Nonfinite summary information");
      check(graph->get_estimate(0).valid, "Invalid distributed estimate");
    }
    FactorGraphResetVariable imu_request;
    imu_request.type = FactorGraphResetVariableType::IMU;
    imu_request.timestamp = 0;
    FactorGraphResetVariable clone_request;
    clone_request.type = FactorGraphResetVariableType::CLONE;
    clone_request.timestamp = 0;
    const auto reset = a.get_reset_snapshot({imu_request, clone_request});
    check(reset.valid && reset.values.size() == 2 && reset.covariance.rows() == 21, "Could not export reset posterior");
    check((reset.values.at(0).head<7>() - reset.values.at(1)).norm() < 1e-12, "Current pose and matching clone disagree");
    check((reset.covariance.block<6, 6>(0, 0) - reset.covariance.block<6, 6>(15, 15)).norm() < 1e-10,
          "Duplicated pose covariance disagrees");
    a.communicate(b, 0, 0, 0, 1);
    bool rejected = false;
    try {
      a.communicate(b, 0, 0, -1, 1);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "Negative range was accepted");

    // A light update takes one incremental step, while a full update converges
    // the complete nonlinear graph and leaves iSAM2 ready for future updates.
    options.factor_graph_agent_id = 3;
    FactorGraphState full_update_graph(options);
    FactorGraphInitialization initial;
    initial.timestamp = 0;
    initial.imu_state(3) = 1;
    initial.variable_names = {"imu"};
    initial.variable_dimensions = {15};
    initial.covariance = Eigen::Matrix<double, 15, 15>::Identity();
    full_update_graph.initialize(initial);
    const gtsam::Key nonlinear_key = gtsam::Symbol('n', 0);
    full_update_graph.pending_values.insert(nonlinear_key, 10.0);
    full_update_graph.pending_factors.emplace_shared<SquaredErrorFactor>(nonlinear_key, 1.0);
    full_update_graph.commit();
    const gtsam::Values light_values = full_update_graph.optimizer->calculateEstimate();
    const double light_value = light_values.at<double>(nonlinear_key);
    const double light_error = full_update_graph.optimizer->getFactorsUnsafe().error(light_values);
    check(light_value > 2, "Light iSAM2 update unexpectedly performed a batch solve");

    full_update_graph.commit(true);
    const gtsam::Values full_values = full_update_graph.optimizer->calculateEstimate();
    const double full_value = full_values.at<double>(nonlinear_key);
    const double full_error = full_update_graph.optimizer->getFactorsUnsafe().error(full_values);
    check(std::abs(full_value - 1) < 1e-5, "Full update did not converge to the LM solution");
    check(full_error < 1e-6 * light_error, "Full update did not substantially reduce nonlinear error");

    full_update_graph.pending_factors.emplace_shared<gtsam::PriorFactor<double>>(nonlinear_key, 1.1,
                                                                                 gtsam::noiseModel::Isotropic::Sigma(1, 0.1));
    full_update_graph.commit();
    check(full_update_graph.optimizer->calculateEstimate<double>(nonlinear_key) > full_value,
          "Rebuilt iSAM2 did not accept a subsequent incremental update");
  }
};

} // namespace ov_msckf

int main() {
  using namespace ov_msckf;
  try {
    const gtsam::Point3 position(2, 3, 4);
    const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.3, -0.2, 0.5), gtsam::Point3(-1, 2, 3));
    gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> range(2, 1, 5, gtsam::noiseModel::Isotropic::Variance(1, 1));
    gtsam::Matrix pose_jacobian, position_jacobian;
    range.evaluateError(pose, position, pose_jacobian, position_jacobian);
    std::function<gtsam::Vector(const gtsam::Pose3 &, const gtsam::Point3 &)> range_error =
        [&](const gtsam::Pose3 &x, const gtsam::Point3 &p) { return range.evaluateError(x, p); };
    FactorGraphDistributedTest::check((pose_jacobian - gtsam::numericalDerivative21(range_error, pose, position)).norm() < 1e-7 &&
                                          (position_jacobian - gtsam::numericalDerivative22(range_error, pose, position)).norm() < 1e-7,
                                      "Range Jacobians disagree with numerical derivatives");
    FactorGraphPosePositionFactor coupling(2, 1);
    coupling.evaluateError(pose, position, pose_jacobian, position_jacobian);
    std::function<gtsam::Vector(const gtsam::Pose3 &, const gtsam::Point3 &)> coupling_error =
        [&](const gtsam::Pose3 &x, const gtsam::Point3 &p) { return coupling.evaluateError(x, p); };
    FactorGraphDistributedTest::check((pose_jacobian - gtsam::numericalDerivative21(coupling_error, pose, position)).norm() < 1e-7 &&
                                          (position_jacobian - gtsam::numericalDerivative22(coupling_error, pose, position)).norm() < 1e-7,
                                      "Coupling Jacobians disagree with numerical derivatives");
    FactorGraphDistributedTest::run(false);
    FactorGraphDistributedTest::run(true);
    std::cout << "Distributed factor-graph checks passed (Cholesky and QR)\n";
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
