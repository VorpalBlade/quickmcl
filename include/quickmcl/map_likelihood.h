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

#include "quickmcl/config.h"
#include "quickmcl/laser.h"
#include "quickmcl/map_base.h"
#include "quickmcl/map_types.h"
#include "quickmcl/parameters.h"
#include "quickmcl/scaled_map.h"

#if QUICKMCL_MAP_LIKELIHOOD_DEBUG_PUB == 1
#include <ros/ros.h>
#endif

//! @file
//! @brief Defines a likelihood field based map.

namespace quickmcl {

//! Map implementation based on likelihood model.
class MapLikelihood final : public MapBase
{
public:
  //! Constructor
  MapLikelihood();

  //! @brief Set parameters, will only take effect on the next new_map() call
  //!
  //! @param parameters Parameters for this class.
  void set_parameters(const Parameters::LikelihoodMap &parameters) override;

  // See Map for documentation.
  void new_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) override;

  double update_importance(const LaserPointCloud &cloud,
                           ParticleCollection *particles) const override;

  //! @brief Get the likelihood map as an occupancy grid, for visualisation and
  //!        debugging with rviz.
  nav_msgs::OccupancyGrid get_likelihood_as_gridmap() const override;

  // Fix potential assert
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  //! Parameters from command line / launch file
  Parameters::LikelihoodMap parameters;

  //! @brief The actual likelihood map, of the same dimension as the occupancy
  //!        grid.
  //!
  //! If USE_DISTANCE_MAP is enabled, this is distances to occupied cells.
  //! If USE_DISTANCE_MAP is not enabled, this is a sort of ill-defined Gaussian
  //! blur.
  MapContainer likelihood_data;

  //! z_hit pre-calculation
  float z_hit_pre_calculated = 1;

  //! Distribution for free cells.
  mutable std::uniform_int_distribution<size_t> free_cells_dist;
  //! Distribution for rotation
  mutable std::uniform_real_distribution<float> rotation_dist =
      std::uniform_real_distribution<float>(static_cast<float>(-M_PI),
                                            static_cast<float>(M_PI));

  //! Random number generator used to generate random valid poses.
  mutable std::mt19937 rng = std::mt19937{std::random_device()()};

  //! Probability at max distance
  double max_distance_probability = 0.01;

  //! @brief Get the probability at the specified world pose. This takes care of
  //!        offset and resolution.
  double get_probability_at(const WorldCoordinate &world_coord) const;

#if QUICKMCL_MAP_LIKELIHOOD_DEBUG_PUB == 1
  ros::Publisher debug_pub;
#endif
};

} // namespace quickmcl
