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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

//! @file
//! @brief Various code related to dealing with laser

namespace quickmcl {

//! Type of laser point cloud as used by this program
using LaserPointCloud = pcl::PointCloud<pcl::PointXY>;

//! Convert to a laser point cloud from a ROS point cloud 2 message.
void from_ros_msg(const sensor_msgs::PointCloud2::ConstPtr &msg,
                  LaserPointCloud &output);

} // namespace quickmcl
