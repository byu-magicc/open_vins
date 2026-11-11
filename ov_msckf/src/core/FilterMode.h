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

#ifndef OV_MSCKF_FILTERMODE_H
#define OV_MSCKF_FILTERMODE_H

#include <array>
#include <stdexcept>
#include <string>

namespace ov_msckf {

enum class FilterType { OPENVINS, FACTOR_GRAPH, HYBRID };

/// Disabled, independent iSAM2 estimation, or a graph seeded by and resetting the EKF.
enum class FactorGraphMode { DISABLED, FACTOR_GRAPH, HYBRID };
enum class EstimateSource { MSCKF, FACTOR_GRAPH };
enum class RangeMethod { NONE, FACTOR_GRAPH };

/** Named estimator modes. All modes use the shared MSCKF pipeline in VioManager. */
struct FilterMode {
  FilterType type;
  const char *name;
  FactorGraphMode graph_mode;
  EstimateSource estimate_source;
  RangeMethod range_method;
};

/// Add an EKF mode here with its own range method and an explicitly disabled graph.
inline const std::array<FilterMode, 3> &filter_modes() {
  static constexpr std::array<FilterMode, 3> modes = {{
      {FilterType::OPENVINS, "openvins", FactorGraphMode::DISABLED, EstimateSource::MSCKF, RangeMethod::NONE},
      {FilterType::FACTOR_GRAPH, "factor_graph", FactorGraphMode::FACTOR_GRAPH, EstimateSource::FACTOR_GRAPH, RangeMethod::FACTOR_GRAPH},
      {FilterType::HYBRID, "hybrid", FactorGraphMode::HYBRID, EstimateSource::MSCKF, RangeMethod::FACTOR_GRAPH},
  }};
  return modes;
}

inline const FilterMode &get_filter_mode(FilterType type) {
  for (const auto &mode : filter_modes()) {
    if (mode.type == type)
      return mode;
  }
  throw std::invalid_argument("Unknown filter type");
}

inline FilterType parse_filter_type(const std::string &name) {
  std::string choices;
  for (const auto &mode : filter_modes()) {
    if (name == mode.name)
      return mode.type;
    if (!choices.empty())
      choices += ", ";
    choices += mode.name;
  }
  throw std::invalid_argument("Unknown filter_type '" + name + "'; expected one of: " + choices);
}

} // namespace ov_msckf

#endif // OV_MSCKF_FILTERMODE_H
