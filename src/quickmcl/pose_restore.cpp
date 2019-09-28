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
#include "quickmcl/pose_restore.h"

#include "quickmcl/i_particle_filter.h"

#include <cmath>
#include <ros/console.h>

//! @file
//! @brief Functionality for storing and restoring the pose to the parameter
//! server

namespace quickmcl {

PoseRestorer::PoseRestorer(
    const ros::NodeHandle &nh_priv,
    const std::shared_ptr<quickmcl::IParticleFilter> &filter)
  : nh_priv(nh_priv)
  , filter(filter)
{}

void PoseRestorer::store_pose()
{
  Pose2D<double> pose;
  Eigen::Matrix3d covariance;
  if (!filter->get_estimated_pose(&pose, &covariance)) {
    ROS_WARN("Couldn't get pose to store");
    return;
  }

  // AMCL compatible names
  nh_priv.setParam("initial_pose_x", pose.x);
  nh_priv.setParam("initial_pose_y", pose.y);
  nh_priv.setParam("initial_pose_a", pose.theta);
  // This discards some information, but it is likely good enough
  nh_priv.setParam("initial_cov_xx", covariance(0, 0));
  nh_priv.setParam("initial_cov_yy", covariance(1, 1));
  nh_priv.setParam("initial_cov_aa", covariance(2, 2));
}

void PoseRestorer::restore_pose()
{
  Pose2D<double> pose{0, 0, 0};
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

  get_param_nan_safe("initial_pose_x", &pose.x, 0.0);
  get_param_nan_safe("initial_pose_y", &pose.y, 0.0);
  get_param_nan_safe("initial_pose_a", &pose.theta, 0.0);
  get_param_nan_safe("initial_pose_xx", &covariance(0, 0), 0.5 * 0.5);
  get_param_nan_safe("initial_pose_yy", &covariance(1, 1), 0.5 * 0.5);
  get_param_nan_safe("initial_pose_aa", &covariance(2, 2), 0.1 * 0.1);

  filter->initialise(
      quickmcl::WeightedParticle::ParticleT(pose),
      covariance.cast<quickmcl::WeightedParticle::ParticleT::Scalar>());
}

void PoseRestorer::get_param_nan_safe(const std::string &name,
                                      double *output,
                                      double default_value)
{
  double tmp;
  nh_priv.param(name, tmp, default_value);
  if (!std::isnan(tmp)) {
    *output = tmp;
  } else {
    ROS_WARN_STREAM("Ignoring " << name << " because of NaN");
    *output = default_value;
  }
}

} // namespace quickmcl
