#ifndef OV_MSCKF_UPDATER_GLOBAL_H
#define OV_MSCKF_UPDATER_GLOBAL_H

#include <Eigen/Eigen>
#include <memory>

#include "utils/sensor_data.h"

namespace ov_msckf {

class State;

/** @brief Apply GPS position/velocity updates in an ENU world without discarding correlated camera clones. */
class UpdaterGlobal {
public:
  /// Update the propagated state and return the same global-frame observation for the graph.
  ov_core::GPSGlobalData update(std::shared_ptr<State> state, const ov_core::GPSData &message);

private:
  bool aligned = false;
  Eigen::Matrix3d R_ecef_to_global;
  Eigen::Vector3d origin_ecef;
  Eigen::Vector3d origin_global;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_GLOBAL_H
