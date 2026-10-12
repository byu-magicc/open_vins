/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_FACTORGRAPHFACTORS_H
#define OV_MSCKF_FACTORGRAPHFACTORS_H

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/SmartProjectionRigFactor.h>

namespace ov_msckf {

/** Fixed lens selection lets native rig factors support mixed radtan/fisheye cameras. */
class FixedCameraCalibration {
public:
  enum { dimension = 9 };
  FixedCameraCalibration(bool fisheye, const gtsam::Vector9 &parameters)
      : fisheye_model(fisheye), radtan(parameters), equidistant(parameters) {}
  gtsam::Point2 uncalibrate(const gtsam::Point2 &point, gtsam::OptionalJacobian<2, 9> calibration = boost::none,
                            gtsam::OptionalJacobian<2, 2> pixel = boost::none) const {
    return fisheye_model ? equidistant.uncalibrate(point, calibration, pixel) : radtan.uncalibrate(point, calibration, pixel);
  }
  gtsam::Point2 calibrate(const gtsam::Point2 &point, gtsam::OptionalJacobian<2, 9> calibration = boost::none,
                         gtsam::OptionalJacobian<2, 2> pixel = boost::none) const;
  double fx() const { return radtan.fx(); }
  gtsam::Matrix3 K() const { return radtan.K(); }
  FixedCameraCalibration retract(const gtsam::Vector9 &delta) const { return {fisheye_model, vector() + delta}; }
  gtsam::Vector9 localCoordinates(const FixedCameraCalibration &other) const { return other.vector() - vector(); }
  bool equals(const FixedCameraCalibration &other, double tolerance) const {
    return fisheye_model == other.fisheye_model && radtan.equals(other.radtan, tolerance);
  }
  void print(const std::string &label) const {
    if (fisheye_model)
      equidistant.print(label);
    else
      radtan.print(label);
  }

private:
  gtsam::Vector9 vector() const { return radtan.vector(); }
  bool fisheye_model;
  gtsam::Cal3DS2 radtan;
  gtsam::Cal3Fisheye equidistant;
};

} // namespace ov_msckf

namespace gtsam {
template <> struct traits<ov_msckf::FixedCameraCalibration> : internal::Manifold<ov_msckf::FixedCameraCalibration> {};
} // namespace gtsam

namespace ov_msckf {

using GraphCamera = gtsam::PinholePose<FixedCameraCalibration>;
using GraphCameraRig = gtsam::CameraSet<GraphCamera>;

/** Native rig geometry/triangulation with Jacobian output for QR and repeated body-pose keys. */
class FactorGraphVisualFactor : public gtsam::SmartProjectionRigFactor<GraphCamera> {
public:
  FactorGraphVisualFactor(const boost::shared_ptr<GraphCameraRig> &rig, double sigma_pixels);
  boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values &values) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

/** Native combined IMU factor, preserving OpenVINS biases in raw sensor coordinates. */
class FactorGraphImuFactor : public gtsam::CombinedImuFactor {
public:
  FactorGraphImuFactor(gtsam::Key pose_i, gtsam::Key velocity_i, gtsam::Key pose_j, gtsam::Key velocity_j, gtsam::Key bias_i,
                       gtsam::Key bias_j, const gtsam::PreintegratedCombinedMeasurements &measurements,
                       const gtsam::Matrix6 &raw_to_corrected, const gtsam::Matrix6 &noise_to_raw);
  gtsam::Vector evaluateError(const gtsam::Pose3 &pose_i, const gtsam::Vector3 &velocity_i, const gtsam::Pose3 &pose_j,
                              const gtsam::Vector3 &velocity_j, const gtsam::imuBias::ConstantBias &bias_i,
                              const gtsam::imuBias::ConstantBias &bias_j, boost::optional<gtsam::Matrix &> H1 = boost::none,
                              boost::optional<gtsam::Matrix &> H2 = boost::none, boost::optional<gtsam::Matrix &> H3 = boost::none,
                              boost::optional<gtsam::Matrix &> H4 = boost::none, boost::optional<gtsam::Matrix &> H5 = boost::none,
                              boost::optional<gtsam::Matrix &> H6 = boost::none) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  gtsam::Matrix6 raw_to_corrected;
};

/** Tie an exchanged position to the translation of a local navigation pose. */
class FactorGraphPosePositionFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
public:
  FactorGraphPosePositionFactor(gtsam::Key pose, gtsam::Key position);
  gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Point3 &position,
                              boost::optional<gtsam::Matrix &> pose_jacobian = boost::none,
                              boost::optional<gtsam::Matrix &> position_jacobian = boost::none) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

} // namespace ov_msckf

#endif // OV_MSCKF_FACTORGRAPHFACTORS_H
