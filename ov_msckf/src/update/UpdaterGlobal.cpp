#include "UpdaterGlobal.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/geodesy.h"
#include "utils/print.h"

using namespace ov_msckf;

ov_core::GPSGlobalData UpdaterGlobal::update(std::shared_ptr<State> state, const ov_core::GPSData &message) {
  const Eigen::Vector3d position_ecef = ov_core::lla_to_ecef(message.lla);
  const Eigen::Matrix3d R_ecef_to_enu = ov_core::ecef_to_enu(message.lla);
  if (!aligned) {
    R_ecef_to_global = R_ecef_to_enu;
    origin_ecef = position_ecef;
    origin_global = state->_imu->pos();
    aligned = true;
    PRINT_INFO("[GPS]: Anchored at %.9f, position %.6f %.6f %.6f\n", message.timestamp, origin_global(0), origin_global(1),
               origin_global(2));
  }
  const Eigen::Matrix3d R_enu_to_global = R_ecef_to_global * R_ecef_to_enu.transpose();
  const Eigen::Vector3d position = origin_global + R_ecef_to_global * (position_ecef - origin_ecef);
  const Eigen::Vector3d velocity = R_enu_to_global * message.velocity;
  Eigen::Matrix<double, 6, 1> residual;
  residual << position - state->_imu->pos(), velocity - state->_imu->vel();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  covariance.topLeftCorner<3, 3>() = R_enu_to_global * message.cov_position * R_enu_to_global.transpose();
  covariance.bottomRightCorner<3, 3>() = R_enu_to_global * message.cov_velocity * R_enu_to_global.transpose();
  PRINT_DEBUG("[GPS]: t=%.9f residual p=%.6f %.6f %.6f v=%.6f %.6f %.6f\n", message.timestamp, residual(0), residual(1), residual(2),
              residual(3), residual(4), residual(5));
  StateHelper::EKFUpdate(state, {state->_imu->p(), state->_imu->v()}, Eigen::Matrix<double, 6, 6>::Identity(), residual, covariance);
  return {state->_timestamp, position, velocity, covariance.topLeftCorner<3, 3>(), covariance.bottomRightCorner<3, 3>()};
}
