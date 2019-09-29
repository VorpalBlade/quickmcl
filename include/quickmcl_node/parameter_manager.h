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
#include <ros/node_handle.h>

//! @file
//! @brief Header for QuickMCL parameter loading & dynamic reconfiguration

namespace quickmcl {
struct Parameters;
class IParticleFilter;
} // namespace quickmcl

// Main part of file
namespace quickmcl_node {

//! Class handling publishing messages
class ParameterManager
{
public:
  //! Constructor
  ParameterManager(const std::shared_ptr<quickmcl::Parameters> &parameters,
                   const ros::NodeHandle &nh_priv);

  ~ParameterManager();

  //! Set particle filter pointer
  void
  set_particle_filter(const std::shared_ptr<quickmcl::IParticleFilter> &pf);

private:
  //! Pimpl idiom
  class Impl;
  //! Pimpl idiom
  std::unique_ptr<Impl> impl;
};

} // namespace quickmcl_node
