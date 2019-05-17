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

#include "quickmcl/laser.h"
#include "quickmcl/map_state.h"
#include "quickmcl/parameters.h"
#include "quickmcl/particle.h"

#include <nav_msgs/OccupancyGrid.h>

//! @file
//! @brief Defines a common interface for all map implementations.

namespace quickmcl {

//! Base class for maps
class Map
{
public:
  virtual ~Map() = default;

  //! Set parameters, will only take effect on the next new_map() call
  //!
  //! @param parameters Parameters for this class.
  virtual void set_parameters(const Parameters::LikelihoodMap &parameters) = 0;

  //! Load a new map
  virtual void new_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) = 0;

  //! For each particle in the collection, compute the importance.
  //!
  //! @return Total weight
  virtual double update_importance(const LaserPointCloud &cloud,
                                   ParticleCollection *particles) const = 0;

  //! Generate a random pose from the map, preferably one that is in free space.
  virtual WeightedParticle::ParticleT generate_random_pose() const = 0;

  //! Get the likelihood map as an occupancy grid, for visualisation and
  //! debugging with rviz.
  virtual nav_msgs::OccupancyGrid get_likelihood_as_gridmap() const = 0;

  //! Get the map state at the specified world pose. This takes care of offset
  //! and resolution.i
  virtual MapState
  get_map_state_at(const Eigen::Vector2f &world_coord) const = 0;
};

} // namespace quickmcl
