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

#include "quickmcl/odometry.h"

#include <Eigen/Geometry>
#include <memory>

//! @file
//! @brief Header for QuickMCL node TF2 reader code

// Forward declarations
namespace ros {
class Time;
}
namespace tf2_ros {
class Buffer;
}

namespace quickmcl {
struct Parameters;
} // namespace quickmcl

// Main part of file
namespace quickmcl_node {

//! Class handling reading TF transforms
class TFReader
{
public:
  //! Constructor
  explicit TFReader(const std::shared_ptr<quickmcl::Parameters> &parameters);

  ~TFReader();

  //! @brief Get the odometry from TF at the specified time as an Eigen
  //! transform.
  //!
  //! @param t          Time to get transform at.
  //! @param transform  Output variable for transform.
  //! @return  True on success otherwise false.
  bool get_odometry_transform(const ros::Time &t, Eigen::Affine3d *transform);

  //! @brief Get the odometry from TF at the specified time as an odometry pose.
  //!
  //! @param t     Time to get transform at.
  //! @param odom  Output variable for pose.
  //! @return  True on success otherwise false.
  bool get_odometry_pose(const ros::Time &t, quickmcl::Odometry *odom);

  //! Get the buffer, for use with message filters.
  //!
  //! Ownership of pointer retained by this class.
  tf2_ros::Buffer *get_buffer();

private:
  //! Pimpl idiom
  class Impl;
  //! Pimpl idiom
  std::unique_ptr<Impl> impl;
};

} // namespace quickmcl_node
