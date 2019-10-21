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

#include <ros/node_handle.h>

//! @file
//! @brief Functionality for storing and restoring the pose to the parameter
//!        server

namespace quickmcl {

class IParticleFilter;

//! @brief Class to handle saving/restoring the pose to/from the parameter
//!        server.
class PoseRestorer
{
public:
  //! @brief Constructor
  //! @param nh_priv  Private node handle
  //! @param filter   Pointer to particle filter
  PoseRestorer(const ros::NodeHandle &nh_priv,
               const std::shared_ptr<IParticleFilter> &filter);

  //! Store the pose from the particle filter to the parameter server.
  void store_pose();

  //! Restore the pose from the parameter server to the particle filter.
  void restore_pose();

private:
  //! Private node handle
  ros::NodeHandle nh_priv;
  //! Pointer to particle filter.
  std::shared_ptr<IParticleFilter> filter;

  //! Get parameter if set and not NaN.
  void get_param_nan_safe(const std::vector<double> &v, size_t index,
                          double *output,
                          double default_value);
};

} // namespace quickmcl
