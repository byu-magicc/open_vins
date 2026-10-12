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

FactorGraphVisualFactor::FactorGraphVisualFactor(const boost::shared_ptr<GraphCameraRig> &rig, double sigma_pixels)
    : gtsam::SmartProjectionRigFactor<GraphCamera>(gtsam::noiseModel::Isotropic::Sigma(2, sigma_pixels), rig,
                                                   gtsam::SmartProjectionParams(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY)) {
  params_.triangulation.enableEPI = true;
  params_.triangulation.rankTolerance = 1e-9;
}

boost::shared_ptr<gtsam::GaussianFactor> FactorGraphVisualFactor::linearize(const gtsam::Values &values) const {
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
  const Eigen::JacobiSVD<gtsam::Matrix> svd(point_jacobian, Eigen::ComputeFullU);
  const gtsam::Matrix nullspace = svd.matrixU().rightCols(point_jacobian.rows() - 3);
  for (gtsam::Key key : keys_) {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(nullspace.cols(), 6);
    for (size_t index = 0; index < nonUniqueKeys_.size(); ++index)
      if (nonUniqueKeys_[index] == key)
        jacobian += nullspace.middleRows(2 * index, 2).transpose() * noiseModel_->Whiten(pose_jacobians[index]);
    terms.emplace_back(key, jacobian);
  }
  return boost::make_shared<gtsam::JacobianFactor>(terms, nullspace.transpose() * rhs);
}

gtsam::NonlinearFactor::shared_ptr FactorGraphVisualFactor::clone() const {
  return boost::make_shared<FactorGraphVisualFactor>(*this);
}

FactorGraphImuFactor::FactorGraphImuFactor(gtsam::Key pose_i, gtsam::Key velocity_i, gtsam::Key pose_j, gtsam::Key velocity_j,
                                           gtsam::Key bias_i, gtsam::Key bias_j,
                                           const gtsam::PreintegratedCombinedMeasurements &measurements,
                                           const gtsam::Matrix6 &raw_to_corrected, const gtsam::Matrix6 &noise_to_raw)
    : gtsam::CombinedImuFactor(pose_i, velocity_i, pose_j, velocity_j, bias_i, bias_j, measurements), raw_to_corrected(raw_to_corrected) {
  Eigen::Matrix<double, 15, 15> transform = Eigen::Matrix<double, 15, 15>::Identity();
  transform.bottomRightCorner<6, 6>() = noise_to_raw;
  Eigen::Matrix<double, 15, 15> covariance = transform * measurements.preintMeasCov() * transform.transpose();
  covariance = 0.5 * (covariance + covariance.transpose());
  covariance.diagonal().array() += 1e-14;
  noiseModel_ = gtsam::noiseModel::Gaussian::Covariance(covariance);
}

gtsam::Vector FactorGraphImuFactor::evaluateError(const gtsam::Pose3 &pose_i, const gtsam::Vector3 &velocity_i, const gtsam::Pose3 &pose_j,
                                                  const gtsam::Vector3 &velocity_j, const gtsam::imuBias::ConstantBias &bias_i,
                                                  const gtsam::imuBias::ConstantBias &bias_j, boost::optional<gtsam::Matrix &> H1,
                                                  boost::optional<gtsam::Matrix &> H2, boost::optional<gtsam::Matrix &> H3,
                                                  boost::optional<gtsam::Matrix &> H4, boost::optional<gtsam::Matrix &> H5,
                                                  boost::optional<gtsam::Matrix &> H6) const {
  const gtsam::imuBias::ConstantBias corrected_i(raw_to_corrected * bias_i.vector());
  const gtsam::imuBias::ConstantBias corrected_j(raw_to_corrected * bias_j.vector());
  gtsam::Vector error =
      gtsam::CombinedImuFactor::evaluateError(pose_i, velocity_i, pose_j, velocity_j, corrected_i, corrected_j, H1, H2, H3, H4, H5, H6);
  error.tail<6>() = bias_i.vector() - bias_j.vector();
  if (H5) {
    *H5 = *H5 * raw_to_corrected;
    H5->bottomRows<6>().setIdentity();
  }
  if (H6) {
    *H6 = *H6 * raw_to_corrected;
    H6->bottomRows<6>() = -gtsam::Matrix6::Identity();
  }
  return error;
}

gtsam::NonlinearFactor::shared_ptr FactorGraphImuFactor::clone() const { return boost::make_shared<FactorGraphImuFactor>(*this); }

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
