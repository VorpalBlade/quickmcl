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
#include <nav_msgs/OccupancyGrid.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

//! @file
//! @brief Header for QuickMCL node main entry point

// Forward declarations
namespace quickmcl {
class IParticleFilter;
class Map;
struct Parameters;
} // namespace quickmcl

namespace quickmcl_node {
class Commands;
class LaserHandler;
class Publishing;
class TFReader;
} // namespace quickmcl_node

// Main part of file
namespace quickmcl_node {

//! Central class tying everything together
class Node
{
public:
  //! Constructor
  Node();

  //! Set up subscriptions, publishers etc.
  void setup();

private:
  //! Global node handle
  ros::NodeHandle nh;

  //! Global parameters from launch file
  std::shared_ptr<quickmcl::Parameters> parameters;

  //! Map object used for importance computations.
  std::shared_ptr<quickmcl::Map> map;
  //! The particle filter itself.
  std::shared_ptr<quickmcl::IParticleFilter> filter;

  //! Class for reading data from TF
  std::shared_ptr<quickmcl_node::TFReader> tf_reader;
  //! Class for publishing.
  std::shared_ptr<quickmcl_node::Publishing> publishing;
  //! Class for laser handling.
  std::shared_ptr<quickmcl_node::LaserHandler> laser_handler;
  //! Class for command.
  std::shared_ptr<quickmcl_node::Commands> commands;

  //! @name Subscribers
  //! @{
  ros::Subscriber map_sub;
  //! @}

  //! @name ROS callbacks
  //! @{
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  //! @}
};

} // namespace quickmcl_node
