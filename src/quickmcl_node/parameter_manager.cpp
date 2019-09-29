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

#include "quickmcl_node/parameter_manager.h"

#include "quickmcl/i_particle_filter.h"
#include "quickmcl/parameters.h"

#include <dynamic_reconfigure/server.h>
#include <quickmcl/QuickMCLConfig.h>

namespace quickmcl_node {

//! Helper to convert string to enum for resampler type.
static quickmcl::ResampleType to_resample_type(const std::string &value)
{
  if (value == "low_variance") {
    return quickmcl::ResampleType::low_variance;
  } else if (value == "adaptive") {
    return quickmcl::ResampleType::adaptive;
  } else if (value == "kld") {
    return quickmcl::ResampleType::kld;
  } else {
    ROS_ERROR_STREAM("Unknown resample type \"" << value
                                                << "\", defaulting to kld");
    return quickmcl::ResampleType::kld;
  }
}

//! Implementation class of ParameterManager
class ParameterManager::Impl
{
public:
  //! Constructor. See parent class.
  Impl(const std::shared_ptr<quickmcl::Parameters> &parameters,
       const ros::NodeHandle &nh_priv)
    : nh_priv(nh_priv)
    , parameters(parameters)
    , dyn_srv(nh_priv)
  {
    load_static();
    dyn_srv.setCallback(boost::bind(&Impl::reconfig, this, _1, _2));
  }

  //! See parent class
  void set_particle_filter(const std::shared_ptr<quickmcl::IParticleFilter> &pf)
  {
    this->pf = pf;
    pf->set_parameters(parameters->particle_filter);
  }

private:
  //! Private node handle
  ros::NodeHandle nh_priv;
  //! Pointer to global parameter object
  std::shared_ptr<quickmcl::Parameters> parameters;

  //! Pointer to particle filter (for notification of changed parameters)
  std::shared_ptr<quickmcl::IParticleFilter> pf;

  //! Dynamic reconfiguration server
  dynamic_reconfigure::Server<quickmcl::QuickMCLConfig> dyn_srv;

  //! Load parameters that cannot be reconfigured on the fly
  void load_static()
  {
    //
    // Map parameters
    //
    nh_priv.param(
        "likelihood_sigma_hit", parameters->likelihood_map.sigma_hit, 0.1f);
    nh_priv.param("likelihood_max_obstacle_distance",
                  parameters->likelihood_map.max_obstacle_distance,
                  2.f);

    //
    // ROS parameters
    //
    nh_priv.param("internal_laser_processing",
                  parameters->ros.internal_laser_processing,
                  false);
  }

  //! Callback for dynamic reconfiguration.
  void reconfig(quickmcl::QuickMCLConfig &config, uint32_t level)
  {
    // Copy to parameters
    if (level & quickmcl::QuickMCL_LEVEL_MOTION_MODEL) {
      parameters->motion_model.alpha[0] =
          static_cast<float>(config.motion_model_alpha_1);
      parameters->motion_model.alpha[1] =
          static_cast<float>(config.motion_model_alpha_2);
      parameters->motion_model.alpha[2] =
          static_cast<float>(config.motion_model_alpha_3);
      parameters->motion_model.alpha[3] =
          static_cast<float>(config.motion_model_alpha_4);
      parameters->motion_model.min_trans =
          static_cast<float>(config.motion_model_min_trans);
      parameters->motion_model.min_rot =
          static_cast<float>(config.motion_model_min_rot);
    }
    if (level & quickmcl::QuickMCL_LEVEL_SENSOR_MODEL) {
      parameters->likelihood_map.z_hit =
          static_cast<float>(config.likelihood_z_hit);
      parameters->likelihood_map.z_rand =
          static_cast<float>(config.likelihood_z_rand);
      parameters->likelihood_map.num_beams = config.likelihood_num_beams;
      parameters->likelihood_map.max_laser_distance =
          static_cast<float>(config.likelihood_max_laser_distance);

      // Help the user out and normalise the z parameters to ensure they add up
      // to 1
      auto sum =
          parameters->likelihood_map.z_hit + parameters->likelihood_map.z_rand;
      parameters->likelihood_map.z_hit /= sum;
      parameters->likelihood_map.z_rand /= sum;
    }
    if (level & quickmcl::QuickMCL_LEVEL_PARTICLE_FILTER) {
      if (config.particle_filter_particle_count_min >
          config.particle_filter_particle_count_max) {
        ROS_ERROR_NAMED(
            "parameters",
            "Particle count min must be less than or equal to max!");
      } else {
        parameters->particle_filter.particle_count_min =
            config.particle_filter_particle_count_min;
        parameters->particle_filter.particle_count_max =
            config.particle_filter_particle_count_max;
      }

      parameters->particle_filter.resample_count =
          config.particle_filter_resample_count;
      parameters->particle_filter.alpha_fast =
          config.particle_filter_alpha_fast;
      parameters->particle_filter.alpha_slow =
          config.particle_filter_alpha_slow;
      parameters->particle_filter.kld_epsilon =
          config.particle_filter_kld_epsilon;
      parameters->particle_filter.kld_z = config.particle_filter_kld_z;
      parameters->particle_filter.space_partitioning_resolution_xy =
          static_cast<float>(config.space_partitioning_resolution_xy);

      auto rem =
          std::abs(std::fmod(M_PI, config.space_partitioning_resolution_theta));
      if (rem > 0.0001) {
        ROS_ERROR_STREAM_NAMED("parameters",
                               "Invalid kld_bucket_resolution_theta, needs to "
                               "divide pi evenly but reminder is "
                                   << rem);
      } else {
        parameters->particle_filter.space_partitioning_resolution_theta =
            static_cast<float>(config.space_partitioning_resolution_theta);
      }

      parameters->particle_filter.resample_type =
          to_resample_type(config.particle_filter_resample_type);

      // TODO: Set it
      if (pf) {
        pf->set_parameters(parameters->particle_filter);
      }
    }
    if (level & quickmcl::QuickMCL_LEVEL_ROS) {
      parameters->ros.fixed_frame = config.fixed_frame;
      parameters->ros.localised_frame = config.localised_frame;
      parameters->ros.odom_frame = config.odom_frame;

      parameters->ros.save_pose_period = config.save_pose_period;
      parameters->ros.post_date_transform = config.post_date_transform;
      parameters->ros.publish_particles = config.publish_particles;
    }
  }
};

ParameterManager::ParameterManager(
    const std::shared_ptr<quickmcl::Parameters> &parameters,
    const ros::NodeHandle &nh_priv)
  : impl(new Impl(parameters, nh_priv))
{}

ParameterManager::~ParameterManager()
{}

void ParameterManager::set_particle_filter(
    const std::shared_ptr<quickmcl::IParticleFilter> &pf)
{
  impl->set_particle_filter(pf);
}

} // namespace quickmcl_node
