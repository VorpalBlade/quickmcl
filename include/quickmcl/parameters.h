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

#include "quickmcl/utils.h"

#include <cmath>
#include <cstdint>
#include <string>

//! @file
//! @brief ROS launch file parameter handling.

namespace quickmcl {

//! Types of resampling algorithm to use.
enum class ResampleType {
  //! Low variance resampler
  low_variance,
  //! Adaptive resampler (with fixed particle count)
  adaptive,
  //! KLD resampler + adaptive sampling
  kld,
};

//! Container for all the parameters from the launch file
struct Parameters
{
  //! Struct for parameters for the motion model
  struct MotionModel
  {
    //! Scaling factors for the variance
    float alpha[4] = {1, 1, 1, 1};

    //! Minimum translation for update. Default value from AMCL.
    float min_trans = 0.2f;
    //! Minimum rotation for update. Default value from AMCL.
    float min_rot = static_cast<float>(M_PI / 6);
  };
  //! Parameters for the motion model
  MotionModel motion_model;

  //! Struct for parameters for the particle filter
  struct ParticleFilter
  {
    //! Type of resampler to use.
    ResampleType resample_type = ResampleType::kld;

    //! Number of particles to use (minimum).
    int32_t particle_count_min = 100;

    //! Number of particles to use (maximum).
    int32_t particle_count_max = 5000;

    //! How often (in number of filter updates) to resample.
    int32_t resample_count = 2;

    //! Fast decay for adaptive sampling. Used by adaptive & KLD.
    double alpha_fast = 0.1;
    //! Slow decay for adaptive sampling. Used by adaptive & KLD.
    double alpha_slow = 0.001;

    //! Used by KLD
    double kld_epsilon = 0.01;
    //! Used by KLD
    double kld_z = 0.01;

    //! Bucket resolution for KLD (x and y)
    float space_partitioning_resolution_xy = 0.5;
    //! Bucket resolution for KLD (angle)
    float space_partitioning_resolution_theta = radians(10.0f);
  };
  //! Parameters for the particle filter.
  ParticleFilter particle_filter;

  //! Struct for parameters for the likelihood map
  struct LikelihoodMap
  {
    //! @name Mixing factors
    //!
    //! These should add up to 1
    //! @{
    float z_hit = 0.5f;
    float z_rand = 0.5f;
    //! @}

    //! Number of points from the point clouds to consider, will be evenly
    //! spaced.
    int32_t num_beams = 30;

    //! Maximum laser distance, anything beyond this will be ignored.
    float max_laser_distance = 14.0f;

    //! Maximum distance to consider obstacles at (when using USE_DISTANCE_MAP).
    //! This is in meters.
    float max_obstacle_distance = 2.0f;

    //! Standard deviation of hit distribution
    float sigma_hit = 0.01f;
  };
  //! Parameters for the likelihood map
  LikelihoodMap likelihood_map;

  //! Struct for parameters for ROS communication (TF frame names etc)
  struct Ros
  {
    //! TF2 reference frame to publish transform relative to (map).
    std::string fixed_frame;
    //! TF2 reference frame to localise (base_link).
    std::string localised_frame;
    //! TF2 reference frame to publish transform for odometry (odom).
    std::string odom_frame;
    //! How much to post-date the published transform by (similar to AMCL),
    //! needed for rviz to not get glitchy. Should be a really small value,
    //! similar to the delay between two scans.
    double post_date_transform = 0.1;
    //! How often to store the pose back to the parameter server [s]
    //! Set to 0.0 to disable.
    double save_pose_period = 2.0;
    //! If true, publish particle cloud as markers
    bool publish_particles = false;
    //! If true, process /scan internally instead of using external /cloud
    bool internal_laser_processing = false;
  };
  //! Parameters for ROS communication (TF frame names etc)
  Ros ros;
};

} // namespace quickmcl
