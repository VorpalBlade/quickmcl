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

#include "quickmcl/i_particle_filter.h"
#include "quickmcl/low_pass_filter.h"
#include "quickmcl/particle.h"
#include "quickmcl/space_partitioning.h"

#include <cstdint>
#include <random>

//! @file
//! @brief Actual particle filter.

namespace quickmcl {

class Map;
struct ParticleCloudStatistics;

//! Main particle filter class.
class ParticleFilter final : public IParticleFilter
{
public:
  //! Constructor
  explicit ParticleFilter(const std::shared_ptr<Map> &map);

  //! Initialise to "known" position (as a normal distribution).
  //!
  //! @param starting_point Central point of particle cloud
  //! @param covariance Covariance matrix (x, y, theta)
  void initialise(const WeightedParticle::ParticleT &starting_point,
                  const Eigen::Matrix3f &covariance);

  //! Trigger a global localization. Discards all particles and places them
  //! uniformly in the free space in map.
  void global_localization();

  //! Sample using motion model. Updates all particles.
  //!
  //! @param odom_new  Estimated x_t
  //! @param odom_old  Estimated x_(t-1)
  //! @param alpha     Weights for variance
  void handle_odometry(const Odometry &odom_new,
                       const Odometry &odom_old,
                       float alpha[4]);

  //! Update importance from observation data.
  void update_importance_from_observations(const LaserPointCloud &cloud);

  //! Resample (using low variance sampling) and cluster particles
  void resample_and_cluster();

  //! Just cluster particles.
  void cluster();

  //! @brief Set parameters
  //!
  //! Note that this might only take effect at the next call to @a initialise or
  //! @a resample (depending on which parameters are changed).
  void set_parameters(const Parameters::ParticleFilter &parameters);

  //! Returns a single "average" pose, for localisation.
  bool get_estimated_pose(Pose2D<double> &pose,
                          Eigen::Matrix3d &covariance) const;

  //! Access collection (debugging purposes only)
  const ParticleCollection &get_particles() const
  {
    return data_sets[current_data_set].particles;
  }

  //! Get number of particles currently in use
  size_t num_particles() const
  {
    return data_sets[current_data_set].particles.size();
  }

private:
  //! Reference to map object
  const std::shared_ptr<Map> map;

  //! Target particle count.
  Parameters::ParticleFilter parameters;

  //! Structure for data set, we have two sets that we switch between.
  struct DataSet
  {
    //! Particle in this data set
    ParticleCollection particles;
  };

  //! The particle data sets. There are two sets that we switch between to
  //! reduce allocations.
  DataSet data_sets[2];
  //! Index into @a particles for the current cloud.
  uint8_t current_data_set = 0;

  //! Space partitioning, used for KLD buckets & clustering.
  SpacePartitioning space_partitioning;

  //! Random number generator used throughout this filter.
  std::mt19937 rng = std::mt19937{std::random_device()()};

  //! @name Low pass filters for adaptive sampling
  //! @{
  LowPassFilter<WeightedParticle::WeightType> w_slow;
  LowPassFilter<WeightedParticle::WeightType> w_fast;
  //! @}

  //! Implements low variance resampling.
  void resample_low_variance(const ParticleCollection &input_particles,
                             ParticleCollection &output_particles);

  //! Implements adaptive resampling
  void resample_adaptive(const ParticleCollection &input_particles,
                         ParticleCollection &output_particles);

  //! Implements adaptive resampling with KLD.
  void resample_kld(const DataSet &input_set, DataSet &output_set);

  //! Normalise the weights of all particles when the total isn't known
  inline void normalise_weights();

  //! Normalise the weights of all particles when the total is known
  //!
  //! @param total_weight Known total weight
  inline void normalise_weights(double total_weight);

  //! Set all weights to be equal.
  inline void equalise_weights();

  //! Compute the cluster statistics
  //! Since cluster ID 0 is invalid that is used for over-all filter statistics
  std::vector<ParticleCloudStatistics>
  compute_cluster_statistics(ParticleCloudStatistics &global_statistics) const;
};

} // namespace quickmcl
