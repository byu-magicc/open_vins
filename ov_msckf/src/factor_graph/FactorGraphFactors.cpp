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

#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <opencv2/calib3d.hpp>

#include <algorithm>

using namespace ov_msckf;

gtsam::Point2 CameraCalibration::calibrate(const gtsam::Point2 &point, gtsam::OptionalJacobian<2, 8> calibration,
                                           gtsam::OptionalJacobian<2, 2> pixel) const {
  // GTSAM 4.2's radtan inverse can fail at image edges. Use the frontend's OpenCV inverse instead.
  const auto parameters = vector();
  const cv::Matx33d intrinsic(parameters(0), 0, parameters(2), 0, parameters(1), parameters(3), 0, 0, 1);
  const cv::Vec4d distortion(parameters(4), parameters(5), parameters(6), parameters(7));
  std::vector<cv::Point2d> distorted{{point.x(), point.y()}}, normalized;
  const cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 50, 1e-12);
  if (fisheye_model)
    cv::fisheye::undistortPoints(distorted, normalized, intrinsic, distortion, cv::noArray(), cv::noArray(), criteria);
  else
    cv::undistortPoints(distorted, normalized, intrinsic, distortion, cv::noArray(), cv::noArray(), criteria);
  const gtsam::Point2 result(normalized.front().x, normalized.front().y);
  if (calibration || pixel) {
    Eigen::Matrix<double, 2, 8> Dcal;
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

FactorGraphVisualFactor::FactorGraphVisualFactor(const boost::shared_ptr<GraphCameraRig> &rig, double sigma_pixels, bool use_qr,
                                               const gtsam::KeyVector &calibration_keys)
    : gtsam::SmartProjectionRigFactor<GraphCamera>(gtsam::noiseModel::Isotropic::Sigma(2, sigma_pixels), rig,
                                                   gtsam::SmartProjectionParams(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY)),
      use_qr(use_qr), calibration_keys(calibration_keys) {
  params_.triangulation.enableEPI = true;
  params_.triangulation.rankTolerance = 1e-9;
}

void FactorGraphVisualFactor::add(const gtsam::Point2 &measured, gtsam::Key pose_key, size_t camera_id) {
  gtsam::SmartProjectionRigFactor<GraphCamera>::add(measured, pose_key, camera_id);
  if (!calibration_keys.empty()) {
    const auto key = calibration_keys.at(camera_id);
    if (std::find(keys_.begin(), keys_.end(), key) == keys_.end())
      keys_.push_back(key);
  }
}

GraphCameraRig FactorGraphVisualFactor::cameras(const gtsam::Values &values) const {
  if (calibration_keys.empty())
    return gtsam::SmartProjectionRigFactor<GraphCamera>::cameras(values);
  GraphCameraRig cameras;
  for (size_t index = 0; index < nonUniqueKeys_.size(); ++index) {
    const auto camera_id = cameraIds_[index];
    const auto &camera = cameraRig_->at(camera_id);
    const auto calibration = camera.calibration().retract(values.at<gtsam::Vector8>(calibration_keys.at(camera_id)) -
                                                         camera.calibration().vector());
    cameras.emplace_back(values.at<gtsam::Pose3>(nonUniqueKeys_[index]).compose(camera.pose()),
                         boost::make_shared<CameraCalibration>(calibration));
  }
  // GTSAM's triangulation cache tracks only poses, so changes to intrinsics must invalidate it.
  cameraPosesTriangulation_.clear();
  return cameras;
}

boost::shared_ptr<gtsam::GaussianFactor> FactorGraphVisualFactor::linearize(const gtsam::Values &values) const {
  if (!use_qr && calibration_keys.empty())
    return gtsam::SmartProjectionRigFactor<GraphCamera>::linearize(values);

  const auto cameras = this->cameras(values);
  this->triangulateSafe(cameras);
  std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
  if (!result_) {
    for (gtsam::Key key : keys_) {
      const bool intrinsic = std::find(calibration_keys.begin(), calibration_keys.end(), key) != calibration_keys.end();
      terms.emplace_back(key, gtsam::Matrix::Zero(0, intrinsic ? 8 : 6));
    }
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
    const bool intrinsic = std::find(calibration_keys.begin(), calibration_keys.end(), key) != calibration_keys.end();
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(projector.rows(), intrinsic ? 8 : 6);
    for (size_t index = 0; index < nonUniqueKeys_.size(); ++index) {
      if (!intrinsic && nonUniqueKeys_[index] == key)
        jacobian += projector.middleCols(2 * index, 2) * noiseModel_->Whiten(pose_jacobians[index]);
      if (intrinsic && calibration_keys.at(cameraIds_[index]) == key) {
        Eigen::Matrix<double, 2, 8> calibration_jacobian;
        cameras[index].project(*result_, boost::none, boost::none, calibration_jacobian);
        jacobian += projector.middleCols(2 * index, 2) * noiseModel_->Whiten(calibration_jacobian);
      }
    }
    terms.emplace_back(key, jacobian);
  }
  const auto factor = boost::make_shared<gtsam::JacobianFactor>(terms, projector * rhs);
  if (!use_qr)
    return boost::make_shared<gtsam::HessianFactor>(*factor);
  return factor;
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
