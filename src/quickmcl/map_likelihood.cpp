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
#include "quickmcl/map_likelihood.h"

#include "quickmcl/distance_calculator.h"

#include <Eigen/Core>
#include <cassert>

#if QUICKMCL_MAP_LIKELIHOOD_DEBUG_PUB == 1
#include <geometry_msgs/PoseArray.h>
#endif

//! @file
//! @brief Defines a likelihood field based map.

namespace quickmcl {

MapLikelihood::MapLikelihood()
{
#if QUICKMCL_MAP_LIKELIHOOD_DEBUG_PUB == 1
  ros::NodeHandle nh;
  debug_pub = nh.advertise<geometry_msgs::PoseArray>("/debug", 10, false);
#endif
}

void MapLikelihood::set_parameters(const Parameters::LikelihoodMap &parameters)
{
  this->parameters = parameters;

  // Pre-calculate values
  z_hit_pre_calculated = 2 * parameters.sigma_hit * parameters.sigma_hit;
  max_distance_probability = std::exp(
      -(parameters.max_obstacle_distance * parameters.max_obstacle_distance) /
      z_hit_pre_calculated);
}

void MapLikelihood::new_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  // Store a copy of the metadata, we will need stuff from it.
  metadata = msg->info;

  // Note: We assume that there is no rotation here, otherwise it will be
  // annoying.
  reconfigure_from_map_size(
      MapCoordinate{metadata.width, metadata.height},
      WorldCoordinate{metadata.origin.position.x, metadata.origin.position.y},
      metadata.resolution * WorldCoordinate::Ones());

  MapContainer tmp;
  compute_distance_map(parameters.max_obstacle_distance, *msg, &tmp, &map_state);
  // Now pre-compute the probability at each point. AMCL does this for every
  // scan instead, probably because it supports dynamic reconfiguration of
  // parameters.
  likelihood_data = (-(tmp * tmp) / z_hit_pre_calculated).exp();

  // Pre-compute a vector of free spaces to allow uniform sampling from it.
  free_cells.clear();
  for (Eigen::Index y = 0; y < map_state.rows(); y++) {
    for (Eigen::Index x = 0; x < map_state.cols(); x++) {
      if (map_state(y, x) == MAP_STATE_FREE) {
        free_cells.push_back(MapCoordinate(x, y));
      }
    }
  }
  free_cells_dist =
      std::uniform_int_distribution<size_t>(0, free_cells.size() - 1);
}

double MapLikelihood::update_importance(const LaserPointCloud &cloud,
                                        ParticleCollection *particles) const
{
  // Calculate step size to use.
  const size_t cloud_size = cloud.size();
  size_t step = 1;
  if (parameters.num_beams) {
    step = cloud.size() / static_cast<size_t>(parameters.num_beams);
  }
  if (step < 1) {
    step = 1;
  }
  // Pre-compute some values.
  const auto z_rand_coefficient =
      parameters.z_rand / parameters.max_laser_distance;

  double total_weight = 0;
  for (auto &particle : *particles) {
#if QUICKMCL_MAP_LIKELIHOOD_DEBUG_PUB == 1
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.poses.reserve(cloud.size());
#endif
    // Create an affine transform to offset based on the particle.
    Eigen::Isometry2f transform(particle.data);
    decltype(particle.weight) weight = 0;
    // Process the points from the point cloud, sample evenly from the point
    // cloud.
    for (size_t i = 0; i < cloud_size; i += step) {
      const auto &point = cloud.at(i);
      WorldCoordinate transformed_point =
          transform * WorldCoordinate(point.x, point.y);
      auto p_z = get_probability_at(transformed_point);
      auto point_weight = parameters.z_hit * p_z + z_rand_coefficient;
      assert(0 <= point_weight && point_weight <= 1);
      // Based on AMCL we take the cube of the point weight, it is noted in the
      // AMCL docs as "a hack", but "works well". Also notice at this point it
      // stops being proper probability, since we add them together.
      weight += point_weight * point_weight * point_weight;

#if QUICKMCL_MAP_LIKELIHOOD_DEBUG_PUB == 1
      // For debugging publish a pose at the relevant point, use angle to
      // indicate liklihood.
      auto debug_pose = Pose2D<double>(
          transformed_point(0), transformed_point(1), M_PI * point_weight);
      msg.poses.push_back(geometry_msgs::Pose(debug_pose));
#endif
    }
    total_weight += weight;

    // Now apply some penalties to particles that are impossible: inside
    // obstacles or outside the known part of the map
    auto map_state =
        get_map_state_at(WorldCoordinate(particle.data.x, particle.data.y));
    if (map_state == MAP_STATE_UNKNOWN) {
      weight = 0;
    } else if (map_state == MAP_STATE_OCCUPIED) {
      weight *= 0.01;
    }

    particle.weight = weight;
#if QUICKMCL_MAP_LIKELIHOOD_DEBUG_PUB == 1
    debug_pub.publish(msg);
#endif
  }
  return total_weight;
}

WeightedParticle::ParticleT MapLikelihood::generate_random_pose() const
{
  WorldCoordinate free_cell = map_to_world(free_cells.at(free_cells_dist(rng)));

  return WeightedParticle::ParticleT(
      free_cell(0), free_cell(1), rotation_dist(rng));
}

nav_msgs::OccupancyGrid MapLikelihood::get_likelihood_as_gridmap() const
{
  nav_msgs::OccupancyGrid result;
  result.info = metadata;
  result.data.resize(static_cast<size_t>(map_size(0) * map_size(1)));

  // Copy data
  for (Eigen::Index y = 0; y < map_size(1); y++) {
    for (Eigen::Index x = 0; x < map_size(0); x++) {
      result.data[static_cast<size_t>(y * map_size(0) + x)] =
          int8_t(100 * likelihood_data(y, x) + 0.5f);
    }
  }
  return result;
}

double
MapLikelihood::get_probability_at(const WorldCoordinate &world_coord) const
{
  // Convert to grid coordinates
  MapCoordinate actual_pos = world_to_map(world_coord);

  // Penalise being outside map.
  double p = max_distance_probability;
  if (is_in_map(actual_pos)) {
    p = likelihood_data(actual_pos(1), actual_pos(0));
  }
  return p;
}

MapState
MapLikelihood::get_map_state_at(const WorldCoordinate &world_coord) const
{
  // Convert to grid coordinates
  MapCoordinate actual_pos = world_to_map(world_coord);

  // Penalise being outside map.
  MapState p = MAP_STATE_UNKNOWN;
  if (is_in_map(actual_pos)) {
    p = map_state(actual_pos(1), actual_pos(0));
  }
  return p;
}

} // namespace quickmcl
