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

#ifndef OV_MSCKF_UPDATER_RANGE_H
#define OV_MSCKF_UPDATER_RANGE_H

#include "core/FilterMode.h"
#include <memory>

namespace ov_msckf {
class State;

/// Stateless, two-sided DMV/PECMV range update. Both posteriors use the same pre-exchange priors.
class UpdaterRange {
public:
  /// Returns false for undefined (coincident) range geometry; numerical errors throw before either state changes.
  static bool update(std::shared_ptr<State> first, std::shared_ptr<State> second, RangeMethod method, double range, double variance);
};
} // namespace ov_msckf
#endif
