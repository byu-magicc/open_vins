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

#ifndef OV_CORE_GEODESY_H
#define OV_CORE_GEODESY_H

#include <Eigen/Core>
#include <cmath>

namespace ov_core {

/// WGS84 LLA (degrees, degrees, meters above ellipsoid) to ECEF (meters).
inline Eigen::Vector3d lla_to_ecef(const Eigen::Vector3d &lla) {
  const double lat = lla(0) * M_PI / 180.0;
  const double lon = lla(1) * M_PI / 180.0;
  const double f = 1.0 / 298.257223563;
  const double e2 = f * (2.0 - f);
  const double n = 6378137.0 / std::sqrt(1.0 - e2 * std::pow(std::sin(lat), 2));
  return {(n + lla(2)) * std::cos(lat) * std::cos(lon), (n + lla(2)) * std::cos(lat) * std::sin(lon),
          (n * (1.0 - e2) + lla(2)) * std::sin(lat)};
}

/// Rotate ECEF vectors into the local east/north/up basis at a WGS84 fix.
inline Eigen::Matrix3d ecef_to_enu(const Eigen::Vector3d &lla) {
  const double lat = lla(0) * M_PI / 180.0;
  const double lon = lla(1) * M_PI / 180.0;
  Eigen::Matrix3d rotation;
  rotation << -std::sin(lon), std::cos(lon), 0.0, -std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat),
      std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat);
  return rotation;
}

} // namespace ov_core

#endif // OV_CORE_GEODESY_H
