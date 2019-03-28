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

#include "quickmcl/laser.h"
#include "quickmcl/odometry.h"
#include "quickmcl/parameters.h"
#include "quickmcl/particle.h"
#include <Eigen/Core>

//! @file
//! @brief Interface for particle filter to cut down on compile time.

namespace quickmcl {

//! Particle filter interface.
class IParticleFilter
{
public:
  virtual ~IParticleFilter() = default;

  //! Initialise to "known" position (as a normal distribution).
  //!
  //! @param starting_point Central point of particle cloud
  //! @param covariance Covariance matrix (x, y, theta)
  virtual void initialise(const WeightedParticle::ParticleT &starting_point,
                          const Eigen::Matrix3f &covariance) = 0;

  //! Trigger a global localization. Discards all particles and places them
  //! uniformly in the free space in map.
  virtual void global_localization() = 0;

  //! Sample using motion model. Updates all particles.
  //!
  //! @param odom_new  Estimated x_t
  //! @param odom_old  Estimated x_(t-1)
  //! @param alpha     Weights for variance
  virtual void handle_odometry(const Odometry &odom_new,
                               const Odometry &odom_old,
                               float alpha[4]) = 0;

  //! Update importance from observation data.
  virtual void
  update_importance_from_observations(const LaserPointCloud &cloud) = 0;

  //! Resample (using low variance sampling) and cluster particles.
  virtual void resample_and_cluster() = 0;

  //! Just cluster particles.
  virtual void cluster() = 0;

  //! @brief Set parameters
  //!
  //! Note that this might only take effect at the next call to @a initialise or
  //! @a resample (depending on which parameters are changed).
  virtual void set_parameters(const Parameters::ParticleFilter &parameters) = 0;

  //! Returns a single "average" pose, for localisation as well as covariance
  //! matrix.
  //!
  //! @return True if we could estimate a pose, otherwise false
  virtual bool get_estimated_pose(Pose2D<double> *pose,
                                  Eigen::Matrix3d *covariance) const = 0;

  //! Access collection (debugging purposes only)
  virtual const ParticleCollection &get_particles() const = 0;

  //! Get number of particles currently in use
  virtual size_t num_particles() const = 0;
};

} // namespace quickmcl
