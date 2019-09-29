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

#include "quickmcl/parameters.h"
#include "quickmcl/particle.h"
#include "quickmcl/scaled_map.h"

#include <Eigen/Core>
#include <boost/functional/hash.hpp>
#include <limits>
#include <unordered_map>

//! @file
//! @brief KLD & clustering code

namespace std {
//! Specialisation for key type in @a quickmcl::SpacePartitioning
template<> struct hash<Eigen::Vector3i>
{
  //! Functor operator
  inline size_t operator()(const Eigen::Vector3i &v) const
  {
    size_t h = 0;
    boost::hash_combine(h, v(0));
    boost::hash_combine(h, v(1));
    boost::hash_combine(h, v(2));
    return h;
  }
};
} // namespace std

namespace quickmcl {

//! @brief KLD bin / cluster data tracking class
class SpacePartitioning final : public ScaledInfiniteMapLogic<3>
{
  //! Type of base class
  using BaseType = ScaledInfiniteMapLogic<3>;

  //! Type of key used in collection
  using Key = MapCoordinate;

public:
  //! Type of cluster ID
  using ClusterId = uint16_t;

  //! Value used for invalid cluster ID
  static const constexpr ClusterId INVALID_CLUSTER_ID =
      std::numeric_limits<ClusterId>::max();

  SpacePartitioning() = default;

  //! Set parameters.
  void set_parameters(const Parameters::ParticleFilter &parameters);

  //! Reset state
  void reset();

  //! Add a particle (or increment it's counter)
  void add_point(const WeightedParticle::ParticleT &particle);

  //! Compute the particle limit (\f$M_\chi\f$ in the book)
  size_t limit() const;

  //! Get the cluster ID for the particle
  ClusterId get_cluster_id(const WeightedParticle::ParticleT &particle) const;

  //! Assign cluster IDs to all cells.
  void assign_clusters();

  //! @brief Get highest valid cluster ID.
  //!
  //! @return Number of clusters.
  //!         Valid range for clusters is [0, return value - 1].
  ClusterId get_cluster_count() const { return cluster_count; }

  // Fix potential assert
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  //! Parameters
  Parameters::ParticleFilter parameters;

  //! Data for each bucket
  struct BucketData
  {
    //! Cluster to which this belongs (or zero for unassigned).
    ClusterId cluster = INVALID_CLUSTER_ID;
  };

  //! Type of KLD data collection
  using Collection = std::unordered_map<Key, BucketData>;

  //! Particle collection
  Collection cells;

  //! Precomputed factor based on epsilon parameter
  double precomputed_epsilon_factor = 0;

  //! Valid range for theta map coordinates (inclusive range).
  Eigen::Vector2i theta_range;

  //! Number of clusters
  ClusterId cluster_count = 0;

  //! Recursive helper for clustering
  void do_cluster(const Collection::value_type &item);
};

} // namespace quickmcl
