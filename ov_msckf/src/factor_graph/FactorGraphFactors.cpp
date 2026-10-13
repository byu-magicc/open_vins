/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "FactorGraphFactors.h"

#include <gtsam/linear/JacobianFactor.h>
#include <opencv2/calib3d.hpp>

using namespace ov_msckf;

gtsam::Point2 FixedCameraCalibration::calibrate(const gtsam::Point2 &point, gtsam::OptionalJacobian<2, 9> calibration,
                                              gtsam::OptionalJacobian<2, 2> pixel) const {
  // GTSAM 4.2's radtan inverse can fail at image edges. Use the frontend's OpenCV inverse instead.
  const auto parameters = vector();
  const cv::Matx33d intrinsic(parameters(0), 0, parameters(3), 0, parameters(1), parameters(4), 0, 0, 1);
  const cv::Vec4d distortion(parameters(5), parameters(6), parameters(7), parameters(8));
  std::vector<cv::Point2d> distorted{{point.x(), point.y()}}, normalized;
  const cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 50, 1e-12);
  if (fisheye_model)
    cv::fisheye::undistortPoints(distorted, normalized, intrinsic, distortion, cv::noArray(), cv::noArray(), criteria);
  else
    cv::undistortPoints(distorted, normalized, intrinsic, distortion, cv::noArray(), cv::noArray(), criteria);
  const gtsam::Point2 result(normalized.front().x, normalized.front().y);
  if (calibration || pixel) {
    gtsam::Matrix29 Dcal;
    gtsam::Matrix2 Dpoint;
    uncalibrate(result, Dcal, Dpoint);
    const gtsam::Matrix2 inverse = Dpoint.inverse();
    if (calibration)
      *calibration = -inverse * Dcal;
    if (pixel)
      *pixel = inverse;
  }
  return result;
}

FactorGraphVisualFactor::FactorGraphVisualFactor(const boost::shared_ptr<GraphCameraRig> &rig, double sigma_pixels, bool use_qr)
    : gtsam::SmartProjectionRigFactor<GraphCamera>(gtsam::noiseModel::Isotropic::Sigma(2, sigma_pixels), rig,
                                                   gtsam::SmartProjectionParams(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY)),
      use_qr(use_qr) {
  params_.triangulation.enableEPI = true;
  params_.triangulation.rankTolerance = 1e-9;
}

boost::shared_ptr<gtsam::GaussianFactor> FactorGraphVisualFactor::linearize(const gtsam::Values &values) const {
  if (!use_qr)
    return gtsam::SmartProjectionRigFactor<GraphCamera>::linearize(values);

  const auto cameras = this->cameras(values);
  this->triangulateSafe(cameras);
  std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
  if (!result_) {
    for (gtsam::Key key : keys_)
      terms.emplace_back(key, gtsam::Matrix::Zero(0, 6));
    return boost::make_shared<gtsam::JacobianFactor>(terms, gtsam::Vector::Zero(0));
  }
  FBlocks pose_jacobians;
  gtsam::Matrix point_jacobian;
  gtsam::Vector rhs;
  this->computeJacobiansWithTriangulatedPoint(pose_jacobians, point_jacobian, rhs, cameras);
  noiseModel_->WhitenSystem(point_jacobian, rhs);
  const gtsam::Matrix3 point_covariance = GraphCameraRig::PointCov(point_jacobian);
  // As in GTSAM's JacobianFactorQ, Q is the Schur projector: Q^T Q = Q.
  const gtsam::Matrix projector = gtsam::Matrix::Identity(point_jacobian.rows(), point_jacobian.rows()) -
                                  point_jacobian * point_covariance * point_jacobian.transpose();
  for (gtsam::Key key : keys_) {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(projector.rows(), 6);
    for (size_t index = 0; index < nonUniqueKeys_.size(); ++index)
      if (nonUniqueKeys_[index] == key)
        jacobian += projector.middleCols(2 * index, 2) * noiseModel_->Whiten(pose_jacobians[index]);
    terms.emplace_back(key, jacobian);
  }
  return boost::make_shared<gtsam::JacobianFactor>(terms, projector * rhs);
}

gtsam::NonlinearFactor::shared_ptr FactorGraphVisualFactor::clone() const {
  return boost::make_shared<FactorGraphVisualFactor>(*this);
}

FactorGraphPosePositionFactor::FactorGraphPosePositionFactor(gtsam::Key pose, gtsam::Key position)
    : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(gtsam::noiseModel::Isotropic::Sigma(3, 1e-2), pose, position) {}

gtsam::Vector FactorGraphPosePositionFactor::evaluateError(const gtsam::Pose3 &pose, const gtsam::Point3 &position,
                                                           boost::optional<gtsam::Matrix &> pose_jacobian,
                                                           boost::optional<gtsam::Matrix &> position_jacobian) const {
  if (position_jacobian)
    *position_jacobian = -gtsam::Matrix3::Identity();
  return pose.translation(pose_jacobian) - position;
}

gtsam::NonlinearFactor::shared_ptr FactorGraphPosePositionFactor::clone() const {
  return boost::make_shared<FactorGraphPosePositionFactor>(*this);
}
