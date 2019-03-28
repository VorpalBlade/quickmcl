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
#include "quickmcl/statistics.h"

//! @file
//! @brief Functions for statistics as needed by MCL localisation.

namespace quickmcl {

void quickmcl::ParticleCloudStatistics::add(
    const quickmcl::WeightedParticle &particle)
{
  sample_count++;
  total_weight += particle.weight;

  auto decomposed_pose = particle.data.to_decomposed();
  mean.noalias() += particle.weight * decomposed_pose;

  // Update covariance partial result, based on decomposition done in AMCL code,
  // the "normal" approach would otherwise be to first compute the mean, then do
  // the covariance in a second pass.

  // Compute outer product of the linear components.
  covariance.block(0, 0, 2, 2).noalias() +=
      particle.weight * decomposed_pose.segment(0, 2) *
      decomposed_pose.segment(0, 2).transpose();
}

void ParticleCloudStatistics::add(const ParticleCollection &particles)
{
  for (const WeightedParticle &particle : particles) {
    add(particle);
  }
}

void ParticleCloudStatistics::add(const ParticleCloudStatistics &statistics)
{
  // Luckily these are trivially additive until finalise() is called.
  total_weight += statistics.total_weight;
  sample_count += statistics.sample_count;
  mean += statistics.mean;
  covariance += statistics.covariance;
}

void ParticleCloudStatistics::finalise()
{
  // Mean for linear components
  mean.segment(0, 2) /= total_weight;

  // Covariance for linear components
  Eigen::Matrix3d tmp = Eigen::Matrix3d::Zero();
  for (Eigen::Index i = 0; i < 2; i++) {
    for (Eigen::Index j = 0; j < 2; j++) {
      tmp(i, j) = covariance(i, j) / total_weight - mean(i) * mean(j);
    }
  }
  // Covariance for theta, see (Mardia and Jupp, 2000) section 2.3
  auto R = std::sqrt(mean(2) * mean(2) + mean(3) * mean(3));
  tmp(2, 2) = 1 - R;

  // Mean for theta
  mean(2) = std::atan2(mean(2), mean(3));
  mean(3) = 0;

  covariance = tmp;
}

std::ostream &operator<<(std::ostream &os, const ParticleCloudStatistics &v)
{
  os << "Statistics{" << std::endl;
  os << "  weight = " << v.total_weight << std::endl;
  os << "  count = " << v.sample_count << std::endl;
  os << "  mean = " << v.mean << std::endl;
  os << "  covariance = " << v.covariance << std::endl;
  os << "}";

  return os;
}

} // namespace quickmcl
