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
#include "quickmcl_node/laser_handler.h"

#include "quickmcl/config.h"
#include "quickmcl/i_particle_filter.h"
#include "quickmcl/laser.h"
#include "quickmcl/odometry.h"
#include "quickmcl/parameters.h"
#include "quickmcl/timer.h"
#include "quickmcl_node/publishing.h"
#include "quickmcl_node/tf_reader.h"

#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

// Warnings inside template instantiation
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <tf2_ros/message_filter.h>
#pragma GCC diagnostic pop

//! @file
//! @brief Source file for handling the laser messages

namespace quickmcl_node {

//! Implementation class (pimpl idiom)
class LaserHandler::Impl
{
public:
  //! Constructor
  Impl(const std::shared_ptr<quickmcl::Parameters> &parameters,
       const std::shared_ptr<quickmcl::IParticleFilter> &filter,
       const std::shared_ptr<TFReader> &tf_reader,
       const std::shared_ptr<Publishing> &publishing)
    : parameters(parameters)
    , filter(filter)
    , tf_reader(tf_reader)
    , publishing(publishing)
  {}

  //! See parent class for documentation.
  void setup()
  {
    // Subscribers
    laser_sub.reset(new LaserSubscriber(nh, "cloud", 10));
    laser_filter.reset(new LaserFilter(*laser_sub,
                                       *tf_reader->get_buffer(),
                                       parameters->ros.odom_frame,
                                       100,
                                       nh));
    laser_filter->registerCallback(&LaserHandler::Impl::laser_scan_callback,
                                   this);
  }

  //! See parent class for documentation.
  void force_pose_reset() { pose_reset = true; }

  //! Callback for ROS laser scan message
  void laser_scan_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    quickmcl::CodeTimer laser_timer("Laser callback");
    // Get odometry
    quickmcl::Odometry odom_new;
    if (!tf_reader->get_odometry_pose(msg->header.stamp, &odom_new)) {
      ROS_ERROR("Failed to get odometry pose while handling laser");
      return;
    }

    // If this is the first odometry, store it into the "last" odometry.
    if (!odom_at_last_filter_run) {
      odom_at_last_filter_run = odom_new;
    }

    // Skip update if we aren't moving to preserve variance.
    //
    // A more exact approach might be to look at the raw motor states and check
    // the tick counters.
    auto diff = odom_new - odom_at_last_filter_run.get();
    diff.normalise();
    bool has_moved = std::abs(diff.x) > parameters->motion_model.min_trans ||
                     std::abs(diff.y) > parameters->motion_model.min_trans ||
                     std::abs(diff.theta) > parameters->motion_model.min_rot;
#if LASER_CALLBACK_IGNORE_NO_MOVE == 0
    if (!pose_reset && !has_moved) {
      ROS_DEBUG("Skipping laser scan: haven't moved");
      // Publish cloud and pose anyway
      filter->cluster();
      publishing->publish_cloud();
      publishing->publish_estimated_pose(msg->header.stamp);
      return;
    }
#endif
    pose_reset = false;

    // Time to actually do something with the data:
    quickmcl::LaserPointCloud cloud;
    quickmcl::from_ros_msg(msg, &cloud);

    // Check that we have at least some particles, this can be false since we
    // discard particles outside of free space in the map. If localisation gets
    // really bad, and takes all the particles outside the map we should trigger
    // global localisation.
    if (filter->get_particles().empty()) {
      ROS_WARN("No particles... Trying global localisation!");
      quickmcl::CodeTimer odom_timer("Global localisation");
      filter->global_localization();
    }

    {
      quickmcl::CodeTimer odom_timer("Odometry");
      // Since the frequency of laser scan data is much lower than that of
      // odometry, now is the time to send the odometry to the filter
      filter->handle_odometry(odom_new,
                              odom_at_last_filter_run.get(),
                              parameters->motion_model.alpha);
      odom_at_last_filter_run = odom_new;
    }
    {
      quickmcl::CodeTimer odom_timer("Sendor model");
      // Now send it to the filter
      filter->update_importance_from_observations(cloud);
    }

    // Resample?
    if (resample_counter++ %
        static_cast<uint32_t>(parameters->particle_filter.resample_count)) {
      quickmcl::CodeTimer resampling_timer("Resampling");
      filter->resample_and_cluster();
      ROS_DEBUG_STREAM_NAMED("resampling",
                             "Particle count after resampling "
                                 << filter->get_particles().size());
    } else {
      ROS_DEBUG_NAMED("resampling", "Not resampling");
      filter->cluster();
    }

    // Publish stuff
    publishing->publish_cloud();
    publishing->publish_estimated_pose(msg->header.stamp);
  }

private:
  //! Global node handle
  ros::NodeHandle nh;

  //! Global parameters from launch file
  std::shared_ptr<quickmcl::Parameters> parameters;

  //! The particle filter itself.
  std::shared_ptr<quickmcl::IParticleFilter> filter;

  //! Class for reading data from TF
  std::shared_ptr<quickmcl_node::TFReader> tf_reader;
  //! Class for publishing.
  std::shared_ptr<quickmcl_node::Publishing> publishing;

  //! Store the previous odometry.
  boost::optional<quickmcl::Odometry> odom_at_last_filter_run = boost::none;

  //! @name Subscribers
  //! @{

  //! Type of laser subscription
  using LaserSubscriber = message_filters::Subscriber<sensor_msgs::PointCloud2>;
  //! Type of laser message filter
  using LaserFilter = tf2_ros::MessageFilter<sensor_msgs::PointCloud2>;

  //! Special subscription used by @a laser_filter to sync the messages with TF
  //! transforms
  std::shared_ptr<LaserSubscriber> laser_sub;
  //! Message filter for point cloud.
  std::shared_ptr<LaserFilter> laser_filter;
  //! @}

  //! Counter for how often to resample.
  uint32_t resample_counter = 0;

  //! Used to force calculating particle weights even when not moving after a
  //! pose reset from rviz.
  bool pose_reset = false;
};

LaserHandler::LaserHandler(
    const std::shared_ptr<quickmcl::Parameters> &parameters,
    const std::shared_ptr<quickmcl::IParticleFilter> &filter,
    const std::shared_ptr<TFReader> &tf_reader,
    const std::shared_ptr<Publishing> &publishing)
  : impl(new Impl(parameters, filter, tf_reader, publishing))
{}

LaserHandler::~LaserHandler()
{
  // Must be here for unique_ptr to work with pimpl.
}

void LaserHandler::setup()
{
  impl->setup();
}

void LaserHandler::force_pose_reset()
{
  impl->force_pose_reset();
}

} // namespace quickmcl_node
