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

#include <memory>

//! @file
//! @brief Factory for likelihood map to reduce template bloat and cut down
//!        compile times.

namespace quickmcl {

class Map;

//! Factory for likelihood map to help Raspberry Pi compiler deal with main.cpp
std::shared_ptr<Map> create_likelihood_map();

} // namespace quickmcl
