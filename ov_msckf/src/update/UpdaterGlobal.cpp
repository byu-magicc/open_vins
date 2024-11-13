#include "UpdaterGlobal.h"

#include "state/State.h"
#include <memory>

using namespace ov_msckf;

UpdaterGlobal::UpdaterGlobal() {}

void UpdaterGlobal::feed_gps(const ov_core::GPSData &message) {
  gps_data.push_back(message);
}

void UpdaterGlobal::update(std::shared_ptr<State> state) {
  for (auto const &gps : gps_data) {
    // Clear all IMU clones
    state->_clones_IMU.clear();
  }

  // Clear the GPS data
  gps_data.clear();
}
