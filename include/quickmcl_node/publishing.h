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
//! @brief Header for QuickMCL node publishing code

// Forward declarations
namespace ros {
class Time;
} // namespace ros

namespace quickmcl {
struct Parameters;
class Map;
class IParticleFilter;
} // namespace quickmcl

namespace quickmcl_node {
class TFReader;
} // namespace quickmcl_node

// Main part of file
namespace quickmcl_node {

//! Class handling publishing messages
class Publishing
{
public:
  //! Constructor
  Publishing(const std::shared_ptr<quickmcl::Parameters> &parameters,
             const std::shared_ptr<quickmcl::Map> &map,
             const std::shared_ptr<quickmcl::IParticleFilter> &filter,
             const std::shared_ptr<TFReader> &tf_reader);

  ~Publishing();

  //! Set up subscriptions, publishers etc.
  void setup();

  //! Publish likelihood map.
  void publish_map();

  //! Publish the particle cloud.
  void publish_cloud();

  //! Publishes the estimated pose.
  void publish_estimated_pose(const ros::Time &t, bool resampled);

private:
  //! Pimpl idiom
  class Impl;
  //! Pimpl idiom
  std::unique_ptr<Impl> impl;
};

} // namespace quickmcl_node
