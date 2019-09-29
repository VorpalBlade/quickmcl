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
//! @brief Header for handling the laser messages

// Forward declarations
namespace quickmcl {
struct Parameters;
class IParticleFilter;
class PoseRestorer;
} // namespace quickmcl

namespace quickmcl_node {
class TFReader;
class Publishing;
} // namespace quickmcl_node

// Main part of file
namespace quickmcl_node {

//! Class handling laser messages
class LaserHandler
{
public:
  //! Constructor
  LaserHandler(const std::shared_ptr<quickmcl::Parameters> &parameters,
               const std::shared_ptr<quickmcl::IParticleFilter> &filter,
               const std::shared_ptr<TFReader> &tf_reader,
               const std::shared_ptr<Publishing> &publishing,
               const std::shared_ptr<quickmcl::PoseRestorer> &pose_restorer);

  ~LaserHandler();

  //! Set up subscriptions, publishers etc.
  void setup();

  //! @brief Used to force calculating particle weights even when not moving
  //!        after a pose reset from rviz.
  void force_pose_reset();

private:
  //! Pimpl idiom
  class Impl;
  //! Pimpl idiom
  std::unique_ptr<Impl> impl;
};

} // namespace quickmcl_node
