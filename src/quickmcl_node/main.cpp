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
#include "quickmcl_node/main.h"

#include "quickmcl/i_particle_filter.h"
#include "quickmcl/map.h"
#include "quickmcl/map_factory.h"
#include "quickmcl/parameters.h"
#include "quickmcl/particle_filter_factory.h"
#include "quickmcl/timer.h"
#include "quickmcl_node/commands.h"
#include "quickmcl_node/laser_handler.h"
#include "quickmcl_node/publishing.h"
#include "quickmcl_node/tf_reader.h"

#include <boost/smart_ptr/make_shared_object.hpp>
#include <ros/init.h>

//! @file
//! @brief Main entry point and ROS-glue.

namespace quickmcl_node {

Node::Node()
  : parameters(new quickmcl::Parameters)
  , map(quickmcl::create_likelihood_map())
  , filter(quickmcl::create_particle_filter(map))
  , tf_reader(new TFReader(parameters))
  , publishing(new Publishing(parameters, map, filter, tf_reader))
  , laser_handler(new LaserHandler(parameters, filter, tf_reader, publishing))
  , commands(new Commands(parameters, filter, laser_handler))
{
  // Load parameters
  parameters->load();
  map->set_parameters(parameters->likelihood_map);
  filter->set_parameters(parameters->particle_filter);

  // Set up particle filter
  quickmcl::WeightedParticle::ParticleT::EigenMatrix cov;
  cov << 0.5f * 0.5f, 0,    // ...
      0, 0, 0.5f * 0.5f, 0, // ...
      0, 0, 0.01f * 0.01f;
  filter->initialise(quickmcl::WeightedParticle::ParticleT{0, 0, 0}, cov);
}

void Node::setup()
{
  publishing->setup();
  laser_handler->setup();
  commands->setup();

  // Subscribers
  map_sub = nh.subscribe("map", 10, &Node::map_callback, this);
}

void Node::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  quickmcl::CodeTimer timer("Map update");
  // Store new map
  map->new_map(msg);
  publishing->publish_map();
}

} // namespace quickmcl_node

//! Main entry point, duh
int main(int argc, char **argv)
{
  ros::init(argc, argv, "quickmcl");

  // Instantiate main class
  quickmcl_node::Node node;
  node.setup();
  ros::spin();

  return 0;
}
