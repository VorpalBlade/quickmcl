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
#include "quickmcl_node/commands.h"

#include "quickmcl/i_particle_filter.h"
#include "quickmcl/parameters.h"
#include "quickmcl/timer.h"
#include "quickmcl_node/covariance_mappings.h"
#include "quickmcl_node/laser_handler.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <std_srvs/Empty.h>

//! @file
//! @brief Source file for command handling

namespace quickmcl_node {

//! Implementation class (pimpl idiom)
class Commands::Impl
{
public:
  //! Constructor
  Impl(const std::shared_ptr<quickmcl::Parameters> &parameters,
       const std::shared_ptr<quickmcl::IParticleFilter> &filter,
       const std::shared_ptr<quickmcl_node::LaserHandler> &laser_handler)
    : parameters(parameters)
    , filter(filter)
    , laser_handler(laser_handler)
  {}

  //! See parent class for documentation.
  void setup()
  {
    // Subscribers
    initialpose_sub =
        nh.subscribe("initialpose", 100, &Impl::initialpose_callback, this);

    // Services
    global_localization_service = nh.advertiseService(
        "global_localization", &Impl::global_localization_callback, this);
  }

  //! ROS callback for initial pose command
  void initialpose_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    quickmcl::CodeTimer timer("Pose estimate particle initialisation");
    if (msg->header.frame_id != parameters->ros.fixed_frame) {
      ROS_ERROR_STREAM("Initial pose only supported in fixed frame (which is \""
                       << parameters->ros.fixed_frame << "\").");
    }
    // Extract covariance from message
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    ConstRosCovarianceMapping mapping(msg->pose.covariance.data());
    // ROS has (x, y, z, rotX, rotY, rotZ)
    // We want to copy x, y, rotZ and the covariances
    cov.block(0, 0, 2, 2) = mapping.block(0, 0, 2, 2).cast<float>();
    cov.block(0, 2, 2, 1) = mapping.block(0, 5, 2, 1).cast<float>();
    cov.block(2, 0, 1, 2) = mapping.block(5, 0, 1, 2).cast<float>();
    cov(2, 2) = static_cast<float>(mapping(5, 5));
    filter->initialise(quickmcl::WeightedParticle::ParticleT(msg->pose.pose),
                       cov);
    laser_handler->force_pose_reset();
  }

  //! ROS callback for global localisation
  bool global_localization_callback(std_srvs::Empty::Request &,
                                    std_srvs::Empty::Response &)
  {
    quickmcl::CodeTimer timer("Global localisation particle initialisation");
    // Trigger global localisation
    filter->global_localization();
    // Shouldn't there be a call here to: laser_handler->force_pose_reset();
    // Seems to work fine anyway though.
    return true;
  }

private:
  //! Global node handle
  ros::NodeHandle nh;

  //! Global parameters from launch file
  std::shared_ptr<quickmcl::Parameters> parameters;

  //! The particle filter itself.
  std::shared_ptr<quickmcl::IParticleFilter> filter;

  //! Class for laser handling.
  std::shared_ptr<quickmcl_node::LaserHandler> laser_handler;

  //! @name Subscribers
  //! @{
  ros::Subscriber initialpose_sub;
  //! @}

  //! @name Services
  //! @{
  ros::ServiceServer global_localization_service;
  //! @}
};

Commands::Commands(
    const std::shared_ptr<quickmcl::Parameters> &parameters,
    const std::shared_ptr<quickmcl::IParticleFilter> &filter,
    const std::shared_ptr<quickmcl_node::LaserHandler> &laser_handler)
  : impl(new Impl(parameters, filter, laser_handler))
{}

Commands::~Commands()
{
  // Must be here for unique_ptr to work with pimpl.
}

void Commands::setup()
{
  impl->setup();
}

} // namespace quickmcl_node
