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
#include "quickmcl/map_base.h"

#include "quickmcl/distance_calculator.h"

#include <Eigen/Core>

//! @file
//! @brief Defines a base map.

namespace quickmcl {

MapBase::MapBase()
{}

void MapBase::new_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  // Store a copy of the metadata, we will need stuff from it.
  metadata = msg->info;

  // Note: We assume that there is no rotation here, otherwise it will be
  // annoying.
  reconfigure_from_map_size(
      MapCoordinate{metadata.width, metadata.height},
      WorldCoordinate{metadata.origin.position.x, metadata.origin.position.y},
      metadata.resolution * WorldCoordinate::Ones());

  // Compute free space map.
  compute_state_map(*msg, &map_state);

  // Pre-compute a vector of free spaces to allow uniform sampling from it.
  free_cells.clear();
  for (Eigen::Index y = 0; y < map_state.rows(); y++) {
    for (Eigen::Index x = 0; x < map_state.cols(); x++) {
      if (map_state(y, x) == MapState::Free) {
        free_cells.push_back(MapCoordinate(x, y));
      }
    }
  }
  free_cells_dist =
      std::uniform_int_distribution<size_t>(0, free_cells.size() - 1);
}

WeightedParticle::ParticleT MapBase::generate_random_pose() const
{
  WorldCoordinate free_cell = map_to_world(free_cells.at(free_cells_dist(rng)));

  return WeightedParticle::ParticleT(
      free_cell(0), free_cell(1), rotation_dist(rng));
}

MapState MapBase::get_map_state_at(const WorldCoordinate &world_coord) const
{
  // Convert to grid coordinates
  MapCoordinate actual_pos = world_to_map(world_coord);

  // Penalise being outside map.
  MapState p = MapState::Unknown;
  if (is_in_map(actual_pos)) {
    p = map_state(actual_pos(1), actual_pos(0));
  }
  return p;
}

} // namespace quickmcl
