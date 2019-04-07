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
#include "quickmcl/laser.h"

#include <pcl_conversions/pcl_conversions.h>

//! @file
//! @brief Various code related to dealing with laser

namespace quickmcl {

void from_ros_msg(const sensor_msgs::PointCloud2 &msg, LaserPointCloud *output)
{
  // PointCloud2 is a bit annoying in that it is a binary blob of data + a
  // header with field definitions, and we really only care about getting x,y.
  // Our sensor doesn't report anything useful for intensity (either 10 for
  // valid beam or 0 otherwise), and everything is on a single plane anyway.
  //
  // As a result of all of this we will use PCL to help us convert this.
  pcl::fromROSMsg(msg, *output);
}

} // namespace quickmcl
