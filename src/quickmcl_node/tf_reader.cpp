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
#include "quickmcl_node/tf_reader.h"

#include "quickmcl/parameters.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

//! @file
//! @brief Source file for QuickMCL node TF2 reader code

namespace quickmcl_node {

//! Implementation class (pimpl idiom)
class TFReader::Impl
{
public:
  //! Constructor
  Impl(const std::shared_ptr<quickmcl::Parameters> &parameters)
    : parameters(parameters)
    , tf_listener(tf_buffer)
  {}

  //! See parent class for documentation.
  bool get_odometry_transform(const ros::Time &t, Eigen::Affine3d &transform)
  {
    geometry_msgs::TransformStamped odom_transform_msg;
    try {
      odom_transform_msg = tf_buffer.lookupTransform(
          parameters->ros.localised_frame, parameters->ros.odom_frame, t);
    } catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM("Failed to get odom transform for time " << t << ": "
                                                               << ex.what());
      return false;
    }
    transform = tf2::transformToEigen(odom_transform_msg.transform);
    return true;
  }

  //! See parent class for documentation.
  bool get_odometry_pose(const ros::Time &t, quickmcl::Odometry &odom)
  {
    geometry_msgs::PoseStamped input;
    input.header.frame_id = parameters->ros.localised_frame;
    input.header.stamp = t;
    // Quaternion needs to be valid, everything else is properly default
    // initialised.
    input.pose.orientation.w = 1;
    geometry_msgs::PoseStamped transformed;
    try {
      tf_buffer.transform(input, transformed, parameters->ros.odom_frame);
    } catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM("Failed to get odom pose for time " << t << ": "
                                                          << ex.what());
      return false;
    }
    odom.x = transformed.pose.position.x;
    odom.y = transformed.pose.position.y;
    odom.theta = tf2::getYaw(transformed.pose.orientation);
    return true;
  }

  //! See parent class for documentation.
  tf2_ros::Buffer &get_buffer() { return tf_buffer; }

private:
  //! Global node handle
  ros::NodeHandle nh;

  //! Global parameters from launch file
  std::shared_ptr<quickmcl::Parameters> parameters;

  //! @name ROS TF transform listener
  //! @{
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  //! @}
};

TFReader::TFReader(
    const std::shared_ptr<quickmcl::Parameters> &parameters)
  : impl(new Impl(parameters))
{}

TFReader::~TFReader()
{
  // Must be here for unique_ptr to work with pimpl.
}

bool TFReader::get_odometry_transform(const ros::Time &t,
                                      Eigen::Affine3d &transform)
{
  return impl->get_odometry_transform(t, transform);
}

bool TFReader::get_odometry_pose(const ros::Time &t,
                                 quickmcl::Odometry &odom)
{
  return impl->get_odometry_pose(t, odom);
}

tf2_ros::Buffer &TFReader::get_buffer()
{
  return impl->get_buffer();
}

} // namespace quickmcl_node
