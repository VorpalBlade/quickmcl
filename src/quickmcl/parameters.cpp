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
#include "quickmcl/parameters.h"

#include <ros/node_handle.h>

//! @file
//! @brief ROS launch file parameter handling.

namespace quickmcl {

//! Helper to convert string to enum for resampler type.
static ResampleType to_resample_type(const std::string &value)
{
  if (value == "low_variance") {
    return ResampleType::low_variance;
  } else if (value == "adaptive") {
    return ResampleType::adaptive;
  } else if (value == "kld") {
    return ResampleType::kld;
  } else {
    throw std::runtime_error("Unknown resample type");
  }
}

void Parameters::load()
{
  ros::NodeHandle nh("~");

  //
  // Motion model
  //
  nh.param("motion_model_alpha_1", motion_model.alpha[0], 0.05f);
  nh.param("motion_model_alpha_2", motion_model.alpha[1], 0.1f);
  nh.param("motion_model_alpha_3", motion_model.alpha[2], 0.02f);
  nh.param("motion_model_alpha_4", motion_model.alpha[3], 0.05f);

  nh.param("motion_model_min_trans", motion_model.min_trans, 0.2f);
  nh.param("motion_model_min_rot",
           motion_model.min_rot,
           static_cast<float>(M_PI / 6));

  //
  // Particle filter parameters
  //
  nh.param("particle_filter_particle_count_min",
           particle_filter.particle_count_min,
           100);
  nh.param("particle_filter_particle_count_max",
           particle_filter.particle_count_max,
           5000);
  nh.param("particle_filter_resample_count", particle_filter.resample_count, 2);
  nh.param("particle_filter_alpha_fast", particle_filter.alpha_fast, 0.1);
  nh.param("particle_filter_alpha_slow", particle_filter.alpha_slow, 0.001);
  nh.param("particle_filter_kld_epsilon", particle_filter.kld_epsilon, 0.05);
  nh.param("particle_filter_kld_z", particle_filter.kld_z, 0.95);
  nh.param("space_partitioning_resolution_xy",
           particle_filter.space_partitioning_resolution_xy,
           0.5f);
  nh.param("space_partitioning_resolution_theta",
           particle_filter.space_partitioning_resolution_theta,
           radians(10.0f));

  auto resample_type =
      nh.param<std::string>("particle_filter_resample_type", "kld");
  particle_filter.resample_type = to_resample_type(resample_type);

  //
  // Map parameters
  //
  nh.param("likelihood_z_hit", likelihood_map.z_hit, 0.9f);
  nh.param("likelihood_z_rand", likelihood_map.z_rand, 0.1f);
  nh.param("likelihood_sigma_hit", likelihood_map.sigma_hit, 0.1f);

  // Help the user out and normalise the z parameters to ensure they add up to 1
  auto sum = likelihood_map.z_hit + likelihood_map.z_rand;
  likelihood_map.z_hit /= sum;
  likelihood_map.z_rand /= sum;

  nh.param("likelihood_num_beams", likelihood_map.num_beams, 30);
  nh.param(
      "likelihood_max_laser_distance", likelihood_map.max_laser_distance, 14.f);
  nh.param("likelihood_max_obstacle_distance",
           likelihood_map.max_obstacle_distance,
           2.f);

  //
  // ROS parameters
  //

  // TF frame names
  nh.param<std::string>("fixed_frame", ros.fixed_frame, "map");
  nh.param<std::string>("localised_frame", ros.localised_frame, "base_link");
  nh.param<std::string>("odom_frame", ros.odom_frame, "odom");

  nh.param("save_pose_period", ros.save_pose_period, 2.0);
  nh.param("post_date_transform", ros.post_date_transform, 0.1);
  nh.param("publish_particles", ros.publish_particles, false);
  nh.param("internal_laser_processing", ros.internal_laser_processing, false);

  //
  // Sanity checks
  //

  // Check that theta evenly divides 2*pi
  auto rem =
      std::abs(std::fmod(M_PI, particle_filter.space_partitioning_resolution_theta));
  if (rem > 0.0001) {
    ROS_FATAL_STREAM_NAMED(
        "parameters",
        "Invalid kld_bucket_resolution_theta, needs to divide "
        "pi evenly but reminder is "
            << rem);
    abort();
  }

  if (particle_filter.particle_count_min > particle_filter.particle_count_max) {
    ROS_FATAL_NAMED("parameters",
                    "Particle count min must be less than or equal to max!");
    abort();
  }
}

} // namespace quickmcl
