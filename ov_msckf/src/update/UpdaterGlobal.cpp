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

    Eigen::Matrix<double, 3, 1> res = gps.z_global - pose_imu->pos();

    StateHelper::EKFUpdate(state, {pose_imu->p()}, Eigen::Matrix<double, 3, 3>::Identity(), res, gps.cov_z_global);
  }

  // Clear out old/invalid data
  if (gps_data.size() > 0) {
    gps_data.clear();

    // Remove all clones
    for (auto &clone : state->_clones_IMU) {
      StateHelper::marginalize(state, clone.second);
    }
    state->_clones_IMU.clear();
  }
}
