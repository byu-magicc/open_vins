#ifndef OV_MSCKF_UPDATER_GLOBAL_H
#define OV_MSCKF_UPDATER_GLOBAL_H

#include <Eigen/Eigen>
#include <memory>

#include "utils/sensor_data.h"

namespace ov_msckf {

class State;

/**
 * @ brief Applied global measurements to the state
 *
 * This class is responsible for applying all forms of global measurements to the state. For GPS data from
 * the multi-agent backend, we use the associated vehicle transformation to calculate the transformed measurement
 * and uncertainty for the local vehicle. The IMU clones are removed from the state after a global measurement is applied
 * to avoid camera measurements attempting to triangluate features across a jump in estimated position.
 */
class UpdaterGlobal {
public:
  /**
   * @brief Default constructor for the global updater
   */
  UpdaterGlobal();

  /**
   * @brief Feed in GPS data from the multi-agent backend
   *
   * @param message Contains our timestamp, gps, uncertainty, and vehicle transformation
   */
  void feed_gps(const ov_core::GPSData &message);

  /**
   * @brief Update the state with the global measurements and remove IMU clones
   *
   * @param state State of the filter
   */
  void update(std::shared_ptr<State> state);

private:
  /// GPS data from the multi-agent backend
  std::vector<ov_core::GPSData> gps_data;
  std::mutex gps_data_mtx;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_GLOBAL_H
