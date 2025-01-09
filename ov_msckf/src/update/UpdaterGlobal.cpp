#include "UpdaterGlobal.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/PoseJPL.h"

using namespace ov_msckf;

UpdaterGlobal::UpdaterGlobal() {}

void UpdaterGlobal::feed_gps(const ov_core::GPSData &message) {
  std::lock_guard<std::mutex> lck(gps_data_mtx);
  gps_data.push_back(message);
}

void UpdaterGlobal::update(std::shared_ptr<State> state) {
  for (auto const &gps : gps_data) {
    std::shared_ptr<ov_type::PoseJPL> pose_imu = state->_imu->pose();

    // Calculate the measurement residual
    Eigen::Matrix<double, 3, 1> res = gps.z_global - (pose_imu->pos() - gps.T_V1toV2);

    // Calculate the measurement Jacobians
    Eigen::Matrix<double, 3, 3> H = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 6> F = Eigen::Matrix<double, 3, 6>::Zero();
    F.block(0, 0, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
    F.block(0, 3, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();

    // Calculate the measurement covariance
    Eigen::Matrix<double, 6, 6> Sigma_z = Eigen::Matrix<double, 6, 6>::Zero();
    Sigma_z.block(0, 0, 3, 3) = gps.cov_z_global;
    Sigma_z.block(3, 3, 3, 3) = gps.cov_T_V1toV2;
    Eigen::Matrix<double, 3, 3> R = F * Sigma_z * F.transpose();

    // Perform the EKF update
    StateHelper::EKFUpdate(state, {pose_imu->p()}, H, res, R);
  }

  // Clear out old/invalid data
  if (gps_data.size() > 0) {
    gps_data.clear();

    // Remove all clones
    for (auto& clone : state->_clones_IMU) {
      StateHelper::marginalize(state, clone.second);
    }
    state->_clones_IMU.clear();

    state->reset_keyframe();
  }
}
