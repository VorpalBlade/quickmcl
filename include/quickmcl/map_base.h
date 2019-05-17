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

#include "quickmcl/map.h"
#include "quickmcl/map_types.h"
#include "quickmcl/scaled_map.h"

#include <vector>

//! @file
//! @brief Defines a base map.

namespace quickmcl {

//! Base Map implementation.
class MapBase
  : public Map
  , protected ScaledMapLogic<2>
{
public:
  MapBase();

  // See Map for documentation.
  void new_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) override;

  WeightedParticle::ParticleT generate_random_pose() const override;

  //! Get the map state at the specified world pose. This takes care of offset
  //! and resolution.i
  MapState get_map_state_at(const WorldCoordinate &world_coord) const override;

  // Fix potential assert
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  //! Metadata of the most recent map.
  nav_msgs::MapMetaData metadata;

  //! Map state for each cell.
  MapStateContainer map_state;

private:
  //! Indices of free cells in map, used for generating random poses
  std::vector<MapCoordinate> free_cells;
  //! Distribution for free cells.
  mutable std::uniform_int_distribution<size_t> free_cells_dist;
  //! Distribution for rotation
  mutable std::uniform_real_distribution<float> rotation_dist =
      std::uniform_real_distribution<float>(static_cast<float>(-M_PI),
                                            static_cast<float>(M_PI));

  //! Random number generator used to generate random valid poses.
  mutable std::mt19937 rng = std::mt19937{std::random_device()()};
};

} // namespace quickmcl
