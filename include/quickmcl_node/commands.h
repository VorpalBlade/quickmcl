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
//! @brief Header for command handling

// Forward declarations
namespace quickmcl {
struct Parameters;
class IParticleFilter;
} // namespace quickmcl

namespace quickmcl_node {
class LaserHandler;
} // namespace quickmcl_node

// Main part of file
namespace quickmcl_node {

//! Class handling commands (initial pose, global localisation)
class Commands
{
public:
  //! Constructor
  Commands(
      const std::shared_ptr<quickmcl::Parameters> &parameters,
      const std::shared_ptr<quickmcl::IParticleFilter> &filter,
      const std::shared_ptr<quickmcl_node::LaserHandler> &laser_handler);

  ~Commands();

  //! Set up subscriptions, publishers etc.
  void setup();

private:
  //! Pimpl idiom
  class Impl;
  //! Pimpl idiom
  std::unique_ptr<Impl> impl;
};

} // namespace quickmcl_node
