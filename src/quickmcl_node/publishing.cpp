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
#include "quickmcl_node/publishing.h"

#include "quickmcl/i_particle_filter.h"
#include "quickmcl/map.h"
#include "quickmcl/parameters.h"
#include "quickmcl/timer.h"
#include "quickmcl_node/covariance_mappings.h"
#include "quickmcl_node/tf_reader.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

//! @file
//! @brief Source file for QuickMCL node publishing code

namespace quickmcl_node {

namespace {
//! Helper to convert a weighted particle to a marker.
static visualization_msgs::Marker
to_marker(const quickmcl::WeightedParticle &particle, double max_weight)
{
  visualization_msgs::Marker marker;
  marker.pose.position.x = particle.data.x;
  marker.pose.position.y = particle.data.y;
  marker.pose.orientation.z = std::sin(particle.data.theta / 2.0f);
  marker.pose.orientation.w = std::cos(particle.data.theta / 2.0f);

  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.2;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;

  marker.color.a = 1;
  marker.color.r = marker.color.b = marker.color.g =
      static_cast<float>(particle.weight / max_weight);

  marker.frame_locked = true;

  return marker;
}

} // anonymous namespace

//! Implementation class (pimpl idiom)
class Publishing::Impl
{
public:
  //! Constructor
  Impl(const std::shared_ptr<quickmcl::Parameters> &parameters,
       const std::shared_ptr<quickmcl::Map> &map,
       const std::shared_ptr<quickmcl::IParticleFilter> &filter,
       const std::shared_ptr<TFReader> &tf_reader)
    : parameters(parameters)
    , map(map)
    , filter(filter)
    , tf_reader(tf_reader)
  {}

  //! See parent class for documentation.
  void setup()
  {
    // Publishers
    if (parameters->ros.publish_particles) {
      particle_pub =
          nh.advertise<visualization_msgs::MarkerArray>("particles", 4);
    }
    likelihood_pub =
        nh.advertise<nav_msgs::OccupancyGrid>("likelihood_map", 2, true);
    estimated_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "quickmcl_pose", 2);
  }

  //! See parent class for documentation.
  void publish_map()
  {
    auto debug_msg = map->get_likelihood_as_gridmap();
    debug_msg.header.stamp = ros::Time::now();
    debug_msg.header.frame_id = parameters->ros.fixed_frame;
    likelihood_pub.publish(debug_msg);
  }

  //! See parent class for documentation.
  void publish_cloud()
  {
    if (!parameters->ros.publish_particles) {
      return;
    }

    quickmcl::CodeTimer resampling_timer("Publishing cloud");
    auto time = ros::Time::now();
    visualization_msgs::MarkerArray cloud_msg;

    double max_weight = 0;

    for (const auto &particle : filter->get_particles()) {
      if (particle.weight > max_weight) {
        max_weight = particle.weight;
      }
    }

    cloud_msg.markers.reserve(
        static_cast<size_t>(parameters->particle_filter.particle_count_max));

    for (const auto &particle : filter->get_particles()) {
      auto marker = to_marker(particle, max_weight);
      marker.id = static_cast<int>(cloud_msg.markers.size());
      marker.header.frame_id = parameters->ros.fixed_frame;
      marker.header.stamp = time;
      cloud_msg.markers.push_back(marker);
    }
    // Remove any extra markers
    for (int32_t i = static_cast<int32_t>(filter->get_particles().size());
         i < parameters->particle_filter.particle_count_max;
         i++) {
      visualization_msgs::Marker marker;
      marker.action = 2;
      marker.id = static_cast<int>(cloud_msg.markers.size());
      marker.header.stamp = time;
      cloud_msg.markers.push_back(marker);
    }

    particle_pub.publish(cloud_msg);
  }

  //! See parent class for documentation.
  void publish_estimated_pose(const ros::Time &t)
  {
    quickmcl::CodeTimer resampling_timer("Publishing pose & transform");
    Eigen::Affine3d odom_transform;
    if (!tf_reader->get_odometry_transform(t, &odom_transform)) {
      ROS_ERROR("Failed to get odom transform while publishing");
      return;
    }

    // Prepare transform from map to localised frame
    quickmcl::Pose2D<double> pose;
    Eigen::Matrix3d covariance;
    if (!filter->get_estimated_pose(&pose, &covariance)) {
      return;
    }
    Eigen::Affine3d transform(pose);
    {
      // Now add in the reverse of the odometry transform, this gives us the
      // transform from map to odom!
      Eigen::Affine3d combined_transform(transform * odom_transform);

      geometry_msgs::TransformStamped trans_msg(
          tf2::eigenToTransform(combined_transform));
      trans_msg.header.frame_id = parameters->ros.fixed_frame;
      // Also set child frame ID.
      trans_msg.child_frame_id = parameters->ros.odom_frame;
      // Post date transform into the future slightly to fix flickering in rviz.
      trans_msg.header.stamp =
          t + ros::Duration(parameters->ros.post_date_transform);
      transform_broadcaster.sendTransform(trans_msg);
    }

    {
      geometry_msgs::PoseWithCovarianceStamped estimated_pose;
      estimated_pose.header.stamp = t;
      estimated_pose.header.frame_id = parameters->ros.fixed_frame;

      estimated_pose.pose.pose = geometry_msgs::Pose(pose);
      // Copy covariance.
      RosCovarianceMapping mapping(estimated_pose.pose.covariance.data());
      // ROS has (x, y, z, rotX, rotY, rotZ)
      // We want to copy x, y, rotZ and the covariances
      //
      // This is exactly the reverse of what we do with the initial pose.
      mapping.block(0, 0, 2, 2) = covariance.block(0, 0, 2, 2);
      mapping.block(0, 5, 2, 1) = covariance.block(0, 2, 2, 1);
      mapping.block(5, 0, 1, 2) = covariance.block(2, 0, 1, 2);
      mapping(5, 5) = covariance(2, 2);

      estimated_pose_pub.publish(estimated_pose);
    }
  }

private:
  //! Global node handle
  ros::NodeHandle nh;

  //! Global parameters from launch file
  std::shared_ptr<quickmcl::Parameters> parameters;

  //! Map object used for importance computations.
  std::shared_ptr<quickmcl::Map> map;
  //! The particle filter itself.
  std::shared_ptr<quickmcl::IParticleFilter> filter;
  //! The TF reader class.
  std::shared_ptr<TFReader> tf_reader;

  //! @name ROS publishers
  //! @{
  //! Publishes particle cloud
  ros::Publisher particle_pub;
  //! Publishes internal map representation for debugging
  ros::Publisher likelihood_pub;
  //! Publish estimated pose (with co-variance)
  ros::Publisher estimated_pose_pub;
  //! Publishes estimated transform.
  tf2_ros::TransformBroadcaster transform_broadcaster;
  //! @}
};

Publishing::Publishing(
    const std::shared_ptr<quickmcl::Parameters> &parameters,
    const std::shared_ptr<quickmcl::Map> &map,
    const std::shared_ptr<quickmcl::IParticleFilter> &filter,
    const std::shared_ptr<TFReader> &tf_reader)
  : impl(new Impl(parameters, map, filter, tf_reader))
{}

Publishing::~Publishing()
{
  // Must be here for unique_ptr to work with pimpl.
}

void Publishing::setup()
{
  impl->setup();
}

void Publishing::publish_map()
{
  impl->publish_map();
}

void Publishing::publish_cloud()
{
  impl->publish_cloud();
}

void Publishing::publish_estimated_pose(const ros::Time &t)
{
  impl->publish_estimated_pose(t);
}

} // namespace quickmcl_node
