// QuickMCL - a computationally efficient MCL implementation for ROS
// Copyright (C) 2019  Arvid Norlander
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#pragma once

//! @file
//! @brief Low pass filter class

namespace quickmcl {

//! @brief Simple discrete low pass filter
//!
//! Note that it is assumed that the value will always be above 0 after
//! initialisation.
//!
//! @tparam T Type of value
template<typename T = double> class LowPassFilter
{
public:
  //! Set the filter constant
  inline void set_alpha(T alpha) { this->alpha = alpha; }

  //! Update the filter with a new value
  inline void update(T new_value)
  {
    // Handle initial assignment
    if (value == 0) {
      value = new_value;
    } else {
      // Traditional low pass filter formulated to use one less arithmetic
      // operation. (Really smart idea taken from AMCL, but ultimately a
      // pointless micro-optimisation.)
      value = value + alpha * (new_value - value);
    }
  }

  //! Hard reset the filter to a specific value.
  inline void reset(T new_value) { value = new_value; }

  //! Get the current value
  inline T get() const { return value; }

private:
  //! Current value
  T value = 0;
  //! Current filter constant
  T alpha = 0.5;
};

} // namespace quickmcl
