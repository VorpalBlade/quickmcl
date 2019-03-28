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
#define POSE2D_CPP

#include "quickmcl/pose_2d.h"

#include <ostream>

//! @file
//! @brief Pose 2D and related operators

namespace quickmcl {

std::ostream &operator<<(std::ostream &os, const Pose2D<float> &pose)
{
  os << "[" << pose.x << "," << pose.y << "," << pose.theta << "]";
  return os;
}

std::ostream &operator<<(std::ostream &os, const Pose2D<double> &pose)
{
  os << "[" << pose.x << "," << pose.y << "," << pose.theta << "]";
  return os;
}

} // namespace quickmcl
