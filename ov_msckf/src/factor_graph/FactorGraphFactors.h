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
#include <gtsam/slam/SmartProjectionRigFactor.h>

namespace ov_msckf {

/** Eight OpenVINS intrinsics with fixed lens selection and zero skew. */
class CameraCalibration {
public:
  enum { dimension = 8 };
  CameraCalibration(bool fisheye, const gtsam::Vector8 &parameters) : fisheye_model(fisheye) {
    gtsam::Vector9 native;
    native << parameters.head<2>(), 0.0, parameters.tail<6>();
    radtan = gtsam::Cal3DS2(native);
    equidistant = gtsam::Cal3Fisheye(native);
  }
  gtsam::Point2 uncalibrate(const gtsam::Point2 &point, gtsam::OptionalJacobian<2, 8> calibration = boost::none,
                            gtsam::OptionalJacobian<2, 2> pixel = boost::none) const {
    gtsam::Matrix29 native;
    const auto result = fisheye_model ? equidistant.uncalibrate(point, calibration ? &native : nullptr, pixel)
                                     : radtan.uncalibrate(point, calibration ? &native : nullptr, pixel);
    if (calibration) {
      calibration->leftCols<2>() = native.leftCols<2>();
      calibration->rightCols<6>() = native.rightCols<6>();
    }
    return result;
  }
  gtsam::Point2 calibrate(const gtsam::Point2 &point, gtsam::OptionalJacobian<2, 8> calibration = boost::none,
                         gtsam::OptionalJacobian<2, 2> pixel = boost::none) const;
  double fx() const { return radtan.fx(); }
  gtsam::Matrix3 K() const { return radtan.K(); }
  CameraCalibration retract(const gtsam::Vector8 &delta) const { return {fisheye_model, vector() + delta}; }
  gtsam::Vector8 localCoordinates(const CameraCalibration &other) const { return other.vector() - vector(); }
  bool equals(const CameraCalibration &other, double tolerance) const {
    return fisheye_model == other.fisheye_model && radtan.equals(other.radtan, tolerance);
  }
  void print(const std::string &label) const {
    if (fisheye_model)
      equidistant.print(label);
    else
      radtan.print(label);
  }

  gtsam::Vector8 vector() const {
    gtsam::Vector8 parameters;
    parameters << radtan.vector().head<2>(), radtan.vector().tail<6>();
    return parameters;
  }

private:
  bool fisheye_model;
  gtsam::Cal3DS2 radtan;
  gtsam::Cal3Fisheye equidistant;
};

} // namespace ov_msckf

namespace gtsam {
template <> struct traits<ov_msckf::CameraCalibration> : internal::Manifold<ov_msckf::CameraCalibration> {};
} // namespace gtsam

namespace ov_msckf {

using GraphCamera = gtsam::PinholePose<CameraCalibration>;
using GraphCameraRig = gtsam::CameraSet<GraphCamera>;

/** Structureless rig factor with optional shared camera intrinsics. */
class FactorGraphVisualFactor : public gtsam::SmartProjectionRigFactor<GraphCamera> {
public:
  FactorGraphVisualFactor(const boost::shared_ptr<GraphCameraRig> &rig, double sigma_pixels, bool use_qr,
                         const gtsam::KeyVector &calibration_keys);
  void add(const gtsam::Point2 &measured, gtsam::Key pose_key, size_t camera_id);
  GraphCameraRig cameras(const gtsam::Values &values) const override;
  boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values &values) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  bool use_qr;
  gtsam::KeyVector calibration_keys;
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
