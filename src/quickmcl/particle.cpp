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
#include "quickmcl/particle.h"

#include <ostream>

//! @file
//! @brief Definition of particle types and functions working on particle
//!        collections.

namespace quickmcl {

std::ostream &operator<<(std::ostream &os, const WeightedParticle &particle)
{
  os << "[" << particle.data << ", " << particle.weight << "]";
  return os;
}

double effective_particles(const ParticleCollection &particles)
{
  double sum = 0;
  for (const WeightedParticle &particle : particles) {
    sum += particle.weight * particle.weight;
  }
  return 1.0 / sum;
}

void normalise_particle_weights(ParticleCollection *particles)
{
  double total_weight = 0;
  for (const auto &p : *particles) {
    total_weight += p.weight;
  }
  normalise_particle_weights(particles, total_weight);
}

void normalise_particle_weights(ParticleCollection *particles,
                                double total_weight)
{
  for (auto &p : *particles) {
    p.weight /= total_weight;
  }
}

void equalise_particle_weights(ParticleCollection *particles)
{
  double avg_weight = 1.0 / particles->size();
  for (auto &p : *particles) {
    p.weight = avg_weight;
  }
}

ParticlesRunningSum::ParticlesRunningSum(const ParticleCollection &particles)
{
  const auto M = particles.size();

  double running_sum = 0;
  for (size_t i = 0; i < M; i++) {
    auto pw = particles.at(i).weight;
    // Skip 0 weight particles
    if (pw <= 0) {
      continue;
    }
    interval_map.insert(std::make_pair(running_sum, i));
    running_sum += pw;
  }

  total_weight = running_sum;
}

} // namespace quickmcl
