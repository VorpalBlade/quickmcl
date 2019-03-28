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

#include "quickmcl/pose_2d.h"

#include <iosfwd>
#include <map>
#include <vector>

//! @file
//! @brief Definition of particle types and functions working on particle
//!        collections.

namespace quickmcl {

//! Structure adding weight information to particles
struct WeightedParticle
{
  //! @brief Type of particle payload data.
  //!
  //! To use something else, change here (and update the odometry & sensor
  //! models in the particle filter).
  using ParticleT = Pose2D<float>;

  //! Type of weight
  using WeightType = double;

  //! "Payload"
  ParticleT data;
  //! Weight of this particle.
  WeightType weight = 1.0;
};

//! Debug output operator
std::ostream &operator<<(std::ostream &os, const WeightedParticle &particle);

//! Type of the internal collection of particles
using ParticleCollection = std::vector<WeightedParticle>;

//! @brief Get number of effective particles in collection
//!
//! Particle weights must be normalised for correct results!
double effective_particles(const ParticleCollection &particles);

//! Normalise the weights of all particles when the total isn't known
void normalise_particle_weights(ParticleCollection *particles);

//! Normalise the weights of all particles when the total is known
//!
//! @param particles     Particle cloud to operate on.
//! @param total_weight  Known total weight
void normalise_particle_weights(ParticleCollection *particles,
                                double total_weight);

//! Set all weights to be equal.
void equalise_particle_weights(ParticleCollection *particles);

//! @brief Running sum of weights in a particle collection
//!
//! Consider all particles in a collection as a long strip, with each particle
//! being as wide as it's weight:
//!
//! <pre>
//! +-----+---------+----+----
//! | p1  | p2      | p3 | ...
//! +-----+---------+----+----
//! </pre>
//!
//! This class implements indexing into that collection, based on the the total
//! "weight so far". It is used as a helper to resampling.
//!
//! Note that the intervals of the weights are semi-open intervals: [a,b).
//! Example: p1 has weight 0.2, it will be returned for lookups in the range
//! [0,0.2).
class ParticlesRunningSum
{
public:
  //! Constructor
  //!
  //! @param particles Particle cloud to compute running sum for
  explicit ParticlesRunningSum(const ParticleCollection &particles);

  //! @brief Lookup particle index into the collection.
  inline size_t lookup_index(double weight) const
  {
    auto iter = interval_map.lower_bound(weight);
    // Deal with end of map and that lower_bound is not /quite/ what we need.
    if (iter == interval_map.end() || iter->first > weight) {
      --iter;
    }
    return iter->second;
  }

  //! Get the total weight of all the particles.
  inline double get_total_weight() const { return total_weight; }

private:
  //! Type of internal data structure
  using ParticleRunningSumMap = std::map<double, size_t>;
  //! Running sum map
  ParticleRunningSumMap interval_map;
  //! Total weight of all the particles.
  double total_weight;
};

} // namespace quickmcl
