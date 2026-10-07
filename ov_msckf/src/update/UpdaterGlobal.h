#ifndef OV_MSCKF_UPDATER_GLOBAL_H
#define OV_MSCKF_UPDATER_GLOBAL_H

#include <Eigen/Eigen>
#include <memory>

#include "utils/sensor_data.h"

namespace ov_msckf {

class State;

/** @brief Apply GPS position/velocity updates without discarding correlated camera clones. */
class UpdaterGlobal {
public:
  explicit UpdaterGlobal(double initial_global_yaw);

  /// Resolve the global yaw gauge using the accepted VIO initialization attitude.
  void set_initial_attitude(const Eigen::Matrix3d &R_GtoI);

  /// Apply a measurement to a state propagated to the measurement's timestamp.
  void update(std::shared_ptr<State> state, const ov_core::GPSData &message);

private:
  double initial_global_yaw;
  double yaw_enu_to_global = 0.0;
  bool aligned = false;
  Eigen::Matrix3d R_ecef_to_global;
  Eigen::Vector3d origin_ecef;
  Eigen::Vector3d origin_global;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_GLOBAL_H
