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
#include "quickmcl/pose_restore.h"
#include "quickmcl/timer.h"
#include "quickmcl_node/publishing.h"
#include "quickmcl_node/tf_reader.h"

#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/LaserScan.h>
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

namespace {

//! Type of callback to parent object
using CloudCallback = std::function<void(const sensor_msgs::PointCloud2 &msg)>;

template<typename LaserMessage> struct LaserSub
{
  //! See parent class for documentation.
  LaserSub(const std::string &topic,
           const std::string &odom_frame,
           const std::string &localised_frame,
           tf2_ros::Buffer *tf_buffer,
           const CloudCallback &callback)
    : callback(callback)
    , localised_frame(localised_frame)
    , tf_buffer(tf_buffer)
  {
    // Subscribers
    laser_sub.reset(new LaserSubscriber(nh, topic, 10));
    laser_filter.reset(
        new LaserFilter(*laser_sub, *tf_buffer, odom_frame, 100, nh));
    laser_filter->registerCallback(&LaserSub::laser_callback, this);
  }

  //! Callback for ROS laser scan message
  void laser_callback(const typename LaserMessage::ConstPtr &msg);

  //! Global node handle
  ros::NodeHandle nh;

  //! @name Subscribers
  //! @{

  //! Type of laser subscription
  using LaserSubscriber = message_filters::Subscriber<LaserMessage>;
  //! Type of laser message filter (scan)
  using LaserFilter = tf2_ros::MessageFilter<LaserMessage>;

  //! Special subscription used by @a laser_filter to sync the messages with TF
  //! transforms
  std::shared_ptr<LaserSubscriber> laser_sub;
  //! Message filter for point cloud.
  std::shared_ptr<LaserFilter> laser_filter;
  //! @}

  //! Laser projector class
  laser_geometry::LaserProjection projector;

  //! Callback to parent for processed point cloud
  const CloudCallback callback;

  //! Localised frame
  const std::string localised_frame;

  //! TF2 buffer object
  tf2_ros::Buffer *const tf_buffer;
};

template<>
void LaserSub<sensor_msgs::LaserScan>::laser_callback(
    const sensor_msgs::LaserScan::ConstPtr &msg)
{
  sensor_msgs::PointCloud2 cloud;

  {
    quickmcl::CodeTimer laser_timer("Laser transform");
    projector.transformLaserScanToPointCloud(
        localised_frame, *msg, cloud, *tf_buffer, -1.0, 0);
  }

  callback(cloud);
}

template<>
void LaserSub<sensor_msgs::PointCloud2>::laser_callback(
    const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  callback(*msg);
}

} // anonymous namespace

//! Implementation class (pimpl idiom)
class LaserHandler::Impl
{
public:
  //! Constructor
  Impl(const std::shared_ptr<quickmcl::Parameters> &parameters,
       const std::shared_ptr<quickmcl::IParticleFilter> &filter,
       const std::shared_ptr<TFReader> &tf_reader,
       const std::shared_ptr<Publishing> &publishing,
       const std::shared_ptr<quickmcl::PoseRestorer> &pose_restorer)
    : parameters(parameters)
    , filter(filter)
    , tf_reader(tf_reader)
    , publishing(publishing)
    , pose_restorer(pose_restorer)
  {}

  //! See parent class for documentation.
  void setup()
  {
    // Subscribers
    if (parameters->ros.internal_laser_processing) {
      scan_sub.reset(new ScanSub("scan",
                                 parameters->ros.odom_frame,
                                 parameters->ros.localised_frame,
                                 tf_reader->get_buffer(),
                                 CloudCallback([this](const auto &msg) {
                                   this->cloud_callback(msg);
                                 })));
    } else {
      cloud_sub.reset(new CloudSub("cloud",
                                   parameters->ros.odom_frame,
                                   parameters->ros.localised_frame,
                                   tf_reader->get_buffer(),
                                   CloudCallback([this](const auto &msg) {
                                     this->cloud_callback(msg);
                                   })));
    }
    save_pose_period = ros::Duration(parameters->ros.save_pose_period);
    save_pose_last_ts = ros::Time::now();
  }

  //! See parent class for documentation.
  void force_pose_reset() { pose_reset = true; }

  //! Callback for ROS laser scan message
  void cloud_callback(const sensor_msgs::PointCloud2 &msg)
  {
    quickmcl::CodeTimer laser_timer("Laser cloud processing");
    // Get odometry
    quickmcl::Odometry odom_new;
    if (!tf_reader->get_odometry_pose(msg.header.stamp, &odom_new)) {
      ROS_ERROR("Failed to get odometry pose while handling laser");
      return;
    }

    // Keep track of if we should recompute the transform or re-publish the
    // previous one
    bool recompute_transform = false;

    // If this is the first odometry, store it into the "last" odometry.
    if (!odom_at_last_filter_run) {
      odom_at_last_filter_run = odom_new;
      recompute_transform = true;
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
#if QUICKMCL_LASER_CALLBACK_IGNORE_NO_MOVE == 0
    if (!pose_reset && !has_moved) {
      ROS_DEBUG("Skipping laser scan: haven't moved");
      // Publish cloud and pose anyway
      filter->cluster();
      publishing->publish_cloud();
      publishing->publish_estimated_pose(msg.header.stamp, recompute_transform);
      return;
    }
#endif
    if (pose_reset) {
      // If the pose was reset we need to recompute the transform for it to take
      // effect.
      recompute_transform = true;
    }
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
      recompute_transform = true;
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
      quickmcl::CodeTimer sensor_timer("Sensor model");
      // Now send it to the filter
      filter->update_importance_from_observations(cloud);
    }

    // Resample?
    if (resample_counter++ %
        static_cast<uint32_t>(parameters->particle_filter.resample_count)) {
      recompute_transform = true;
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
    publishing->publish_estimated_pose(msg.header.stamp, recompute_transform);

    ros::Time now = ros::Time::now();
    if (!save_pose_period.isZero() &&
        (now - save_pose_last_ts) >= save_pose_period) {
      pose_restorer->store_pose();
      save_pose_last_ts = now;
    }
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
  //! Class for handling storing/restoring pose
  std::shared_ptr<quickmcl::PoseRestorer> pose_restorer;

  //! Store the previous odometry.
  boost::optional<quickmcl::Odometry> odom_at_last_filter_run = boost::none;

  //! @name Subscribers
  //! @{
  using ScanSub = LaserSub<sensor_msgs::LaserScan>;
  using CloudSub = LaserSub<sensor_msgs::PointCloud2>;

  std::shared_ptr<ScanSub> scan_sub;
  std::shared_ptr<CloudSub> cloud_sub;
  //! @}

  //! @name Pose saving
  //! @{
  ros::Duration save_pose_period;
  ros::Time save_pose_last_ts;
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
    const std::shared_ptr<Publishing> &publishing,
    const std::shared_ptr<quickmcl::PoseRestorer> &pose_restorer)
  : impl(new Impl(parameters, filter, tf_reader, publishing, pose_restorer))
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
