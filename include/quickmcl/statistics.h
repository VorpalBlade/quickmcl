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

#include "quickmcl/particle.h"

#include <iosfwd>

//! @file
//! @brief Functions for statistics as needed by MCL localisation.

namespace quickmcl {

//! Implements statistics on a set of particle clouds
//!
//! An interesting property is that we can combine multiple instances with
//! simple additions of the members as long as finalise() hasn't been called.
//! This allows easily calculating per cluster statistics, then combining all
//! the clusters to compute per-filter statistics. It would also allow a
//! map-reduce kind of implementation if you want to multi-thread it (shouldn't
//! really be needed).
struct ParticleCloudStatistics
{
  //! Total weight added to this cluster
  double total_weight = 0.0;
  //! Number of samples added to this set of statistics
  size_t sample_count = 0;

  //! Mean & temporary storage for total during computation.
  //!
  //! In the end, only the first three components are valid
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();

  //! Covariance & temporary storage for totals during computation
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

  //! Add a particle to this statistics object
  void add(const WeightedParticle &particle);

  //! Add a set of particles
  void add(const ParticleCollection &particles);

  //! Add another statistics object
  void add(const ParticleCloudStatistics &statistics);

  //! Finalise the statistics making mean and covariance valid.
  void finalise();

  //! Return mean as a pose, only valid after finalise().
  Pose2D<double> as_pose() const
  {
    return Pose2D<double>(mean(0), mean(1), mean(2));
  }
};

//! Stream operator for debugging.
std::ostream &operator<<(std::ostream &os, const ParticleCloudStatistics &v);

} // namespace quickmcl
