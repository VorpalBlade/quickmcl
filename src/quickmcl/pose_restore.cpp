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

#include "quickmcl/timer.h"
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

  quickmcl::CodeTimer timer("Writing parameters");
  std::vector<double> v;
  v.push_back(pose.x);
  v.push_back(pose.y);
  v.push_back(pose.theta);
  v.push_back(covariance(0, 0));
  v.push_back(covariance(1, 1));
  v.push_back(covariance(2, 2));
  nh_priv.setParam("initial_pose", v);
}

void PoseRestorer::restore_pose()
{
  Pose2D<double> pose{0, 0, 0};
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

  std::vector<double> v;
  if (nh_priv.getParam("initial_pose", v))
  {
    get_param_nan_safe(v, 0, &pose.x, 0.0);
    get_param_nan_safe(v, 1, &pose.y, 0.0);
    get_param_nan_safe(v, 2, &pose.theta, 0.0);
    get_param_nan_safe(v, 3, &covariance(0, 0), 0.5 * 0.5);
    get_param_nan_safe(v, 4, &covariance(1, 1), 0.5 * 0.5);
    get_param_nan_safe(v, 5, &covariance(2, 2), 0.1 * 0.1);
  }


  filter->initialise(
      quickmcl::WeightedParticle::ParticleT(pose),
      covariance.cast<quickmcl::WeightedParticle::ParticleT::Scalar>());
}

void PoseRestorer::get_param_nan_safe(const std::vector<double>& v,
                                      size_t index,
                                      double *output,
                                      double default_value)
{
  if (index >= v.size())
  {
    ROS_WARN_STREAM("Ignoring initial_pose[" << index << "] because it is missing");
    *output = default_value;
  }
  else if (!std::isnan(v[index])) {
    *output = v[index];
  } else {
    ROS_WARN_STREAM("Ignoring initial_pose[" << index << "] because of NaN");
    *output = default_value;
  }
}

} // namespace quickmcl
