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
#include "quickmcl/space_partitioning.h"

//! @file
//! @brief KLD & clustering code

namespace quickmcl {

void SpacePartitioning::set_parameters(
    const Parameters::ParticleFilter &parameters)
{
  this->parameters = parameters;
  set_resolutions({parameters.space_partitioning_resolution_xy,
                   parameters.space_partitioning_resolution_xy,
                   parameters.space_partitioning_resolution_theta});
  precomputed_epsilon_factor = 2 * parameters.kld_epsilon;

  // Compute the valid range for theta dimension, compute the cell indices just
  // inside the min/max to get the inclusive range of valid IDs.
  theta_range = {world_to_map({0, 0, static_cast<float>(-M_PI + 0.001)})(2),
                 world_to_map({0, 0, static_cast<float>(M_PI - 0.001)})(2)};
  reset();
}

void SpacePartitioning::reset()
{
  cells.clear();
  cluster_count = 0;
}

void SpacePartitioning::add_point(const WeightedParticle::ParticleT &particle)
{
  MapCoordinate idx =
      world_to_map(WorldCoordinate(particle.x, particle.y, particle.theta));
  cells.insert({idx, BucketData{}});
}

size_t SpacePartitioning::limit() const
{
  const auto cells_with_data = cells.size();
  // Avoid divisions by zero
  if (cells_with_data < 2) {
    return static_cast<size_t>(parameters.particle_count_max);
  }

  // Compute formula from book, line 16 in table 8.4.
  double common = 2.0 / (9 * (cells_with_data - 1));
  double to_be_cubed = 1 - common - std::sqrt(common) * parameters.kld_z;
  double result = ((cells_with_data - 1) / precomputed_epsilon_factor) *
                  to_be_cubed * to_be_cubed * to_be_cubed;
  auto final_result = int32_t(std::ceil(result));

  // Check the result is within [min,max]
  if (final_result < parameters.particle_count_min) {
    return static_cast<size_t>(parameters.particle_count_min);
  } else if (final_result > parameters.particle_count_max) {
    return static_cast<size_t>(parameters.particle_count_max);
  }
  return static_cast<size_t>(final_result);
}

SpacePartitioning::ClusterId SpacePartitioning::get_cluster_id(
    const WeightedParticle::ParticleT &particle) const
{
  MapCoordinate idx =
      world_to_map(WorldCoordinate(particle.x, particle.y, particle.theta));
  auto it = cells.find(idx);
  if (it != cells.end()) {
    return it->second.cluster;
  } else {
    return INVALID_CLUSTER_ID;
  }
}

void SpacePartitioning::assign_clusters()
{
  cluster_count = 0;
  for (auto &item : cells) {
    // Skip if already assigned.
    if (item.second.cluster != INVALID_CLUSTER_ID) {
      continue;
    }
    item.second.cluster = cluster_count++;
    do_cluster(item);
  }
}

void SpacePartitioning::do_cluster(const Collection::value_type &item)
{
  // Generate neighbours
  for (Key::Scalar theta = -1; theta <= 1; theta++) {
    // Compute partial key with theta and handle wrapping
    auto partial_key = item.first;
    partial_key(2) += theta;
    // Wrap around circle
    if (partial_key(2) < theta_range(0)) {
      partial_key(2) = theta_range(1);
    } else if (partial_key(2) > theta_range(1)) {
      partial_key(2) = theta_range(0);
    }

    // Handle linear components
    for (Key::Scalar x = -1; x <= 1; x++) {
      for (Key::Scalar y = -1; y <= 1; y++) {
        auto it = cells.find(partial_key + Key{x, y, 0});
        if (it != cells.end() && it->second.cluster == INVALID_CLUSTER_ID) {
          it->second.cluster = item.second.cluster;
          // Recurse!
          do_cluster(*it);
        }
      }
    }
  }
}

} // namespace quickmcl
