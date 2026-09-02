/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_FACTORGRAPHMANAGER_H
#define OV_MSCKF_FACTORGRAPHMANAGER_H

#include "FactorGraphState.h"

#include <memory>

namespace ov_core {
struct GPSData;
struct ImuData;
} // namespace ov_core

namespace ov_msckf {

class State;
struct VioManagerOptions;

/**
 * @brief Passive interface between VioManager decisions and the parallel factor-graph estimator.
 *
 * VioManager owns this object and determines what measurements are supplied and when. This
 * manager must never modify the OpenVINS state or return information that affects its decisions.
 * The implementation synchronizes access because initialization and sensor callbacks may arrive
 * from different threads.
 */
class FactorGraphManager {
public:
  explicit FactorGraphManager(const VioManagerOptions &options);

  /** @brief Forward a raw IMU sample to the graph state's preintegration buffer. */
  void feed_imu(const ov_core::ImuData &message);

  /** @brief Forward a raw GPS sample to the graph state's pending global measurements. */
  void feed_gps(const ov_core::GPSData &message);

  /** @brief Seed the graph from the complete successful OpenVINS state and covariance. */
  void initialize(const std::shared_ptr<State> &openvins_state);

  /** @brief Incorporate a zero-velocity constraint accepted by OpenVINS. */
  void add_zero_velocity_factor(double timestamp);

  /** @brief Incorporate a value-owned batch of visual tracks accepted by OpenVINS. */
  void add_visual_factors(const FactorGraphVisualUpdate &update);

  /** @brief Match OpenVINS removal of persistent visual landmarks. */
  void marginalize_landmarks(const std::vector<size_t> &feature_ids);

  /** @brief Apply queued global measurements at the same update boundary as OpenVINS. */
  void apply_pending_global_factors(double timestamp);

  /** @brief Commit a camera transaction and return the aligned factor-graph result. */
  FactorGraphResult finish_camera_update(double timestamp);

private:
  std::unique_ptr<FactorGraphState> state;
};

} // namespace ov_msckf

#endif // OV_MSCKF_FACTORGRAPHMANAGER_H
