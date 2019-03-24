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

#include <Eigen/Core>

//! @file
//! @brief Common data type definition for map data.

namespace quickmcl {

//! Data type for map. Use row major to match the ROS occupancy grid.
using MapContainer =
    Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

//! Valued used in MapStateContainer
enum MapState : int8_t
{
  MAP_STATE_FREE = 0,
  MAP_STATE_OCCUPIED = 1,
  MAP_STATE_UNKNOWN = 2,
};

//! Data type for map meta-data. Use row major to match the ROS occupancy grid.
using MapStateContainer =
    Eigen::Array<MapState, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

} // namespace quickmcl
