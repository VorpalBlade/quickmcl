# How to use QuickMCL

You need ROS, how to set up that is outside the scope of this documentation, see
the [ROS wiki](https://wiki.ros.org/) for general ROS documentation.

This software expects to have the following topics with relevant information
(these can all be remapped via the launch file in the standard ROS manner of
course):

* One of (depending on the `internal_laser_processing` option):
  * `/cloud` (`sensor_msgs/PointCloud2`): Laser point cloud in the `base_link` TF
    frame. It is recommended to use the
    [laser_filters](https://wiki.ros.org/laser_filters) package to convert a laser
    scan to a point cloud. An [example launch file](../launch/laser_filter.launch)
    for this is available. (`internal_laser_processing = false`)
  * `/scan` (`sensor_msgs/LaserScan`): Laser scan in a TF frame convertable to
    the `base_link` frame. (`internal_laser_processing = true`)
* `/map` (`nav_msgs/OccupancyGrid`): Map to localise in. Consider using
  [map_server](https://wiki.ros.org/map_server) for this.
* TF transforms (can be remapped via parameters, see below):
  * `odom` -> `base_link`: This is the source of the odometry used.


The software will estimate the true location of `base_link` in the map and based
on that publish a transform `map` -> `odom`.

In addition to the above inputs that are required for functionality, there are
optional (and situational) inputs:

* `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`) is a topic used to
  provide the software with a new estimated pose.
* `/global_localization` (`std_srvs/Empty`) is a service used to initiate
  global localisation.

The software will publish:

* `/particles` (`visualization_msgs/MarkerArray`): Cloud of particles with the
  colour representing the weight of the particles. Useful for visualising and
  debugging. Can be turned off via a parameter (see below).
* `/likelihood_map` (`nav_msgs/OccupancyGrid`): Modified internal map used for
  localisation. Useful for debugging. Intensity represents probability.
  distribution around obstacles based on distance from obstacles in `/map`.
* `/localisation_pose` (`geometry_msgs/PoseWithCovarianceStamped`) is the topic
  used to publish the best guess pose as well as covariance for the entire
  filter.
* A TF transform from `map` -> `odom`.

## Parameters

### ROS communication

These parameters are for changing names when communicating with the rest of ROS:

* `~fixed_frame` (`string`, default: map):
  TF frame to work in, map expected to be in this frame.
* `~localised_frame` (`string`, default: base_link):
  TF frame to localise.
* `~odom_frame` (`string`, default: odom):
  TF frame to publish transform for (from fixed frame).

Other ROS communication parameters:

* `~post_date_transform` (`double`, default: 0.1):
   How much to post-date transform, should be similar to delta between scans.
   Helps rviz get less glitchy and in general indicates that this estimate is
   valid a bit into the future.
* `~publish_particles` (`bool`, default: false):
   If true, publish markers as a particle cloud to the topic `/particles`
* `~internal_laser_processing` (`bool`, default: false):
   If true, process laser directly from a scan internally instead of relying on
   external conversion to a point cloud.

### Motion model

Expected noise in odometry:
* `~motion_model_alpha_1` (`double`, default: 0.05):
  Rotation noise from rotation
* `~motion_model_alpha_2` (`double`, default: 0.1):
  Rotation noise from translation
* `~motion_model_alpha_3` (`double`, default: 0.02):
  Translation noise from translation
* `~motion_model_alpha_4` (`double`, default: 0.05):
  Translation noise from rotation

Other motion model parameters:
* `~motion_model_min_trans` (`double`, default: 0.2):
  Minimum translation for filter to update [m]
* `~motion_model_min_rot` (`double`, default: pi/6):
  Minimum rotation for filter to update [rad]

### Sensor model

These z-values should add up to 1, being "mixing factors":
* `~likelihood_z_hit` (`double`, default: 0.9):
  Probability weight to assign to "hit when there is actually something there in
  the map".
* `~likelihood_z_rand` (`double`, default: 0.1):
  Probability weight to assign to "random hit even when there is nothing there
  in the map".

Parameters used to generate the internal likelihood map:
* `~likelihood_sigma_hit` (`double`, default: 0.1):
  Sigma of hit distribution [m]
* `~likelihood_max_obstacle_distance` (`double`, default: 2.0):
  Maximum distance to compute obstacle distances for (for likelihood map) [m].

Other sensor model parameters:
* `~likelihood_max_laser_distance` (`double`, default: 14.0):
  Maximum laser distance [m]. Used to divide likelihood_z_rand to create proper
  probability distribution.
* `~likelihood_num_beams` (`int`, default: 30):
  Number of laser beams to use from the scan. This sub-sampling will be done via
  selecting evenly spaced beams. Set to 0 to use all (not recommended).

### Particle filter

General parameters:
* `~particle_filter_particle_count_min` (`int`, default: 100):
  Number of particles (min)
* `~particle_filter_particle_count_max` (`int`, default: 5000):
  Number of particles (max)
* `~particle_filter_resample_count` (`int`, default: 2):
  How often (in filter updates) to resample.
* `~particle_filter_resample_type` (`string`, default: kld):
  Resample type: `low_variance`, `adaptive`, `kld` are valid choices.
  *Note*: When using `low_variance`, only the min number of particles are used,
  and `particle_filter_alpha_fast`/`particle_filter_alpha_slow `are not used.

Parameters for adaptive sampling and KLD:
* `~particle_filter_alpha_fast` (`double`, default: 0.1):
  Low pass constant for adaptive/kld.
* `~particle_filter_alpha_slow` (`double`, default: 0.001):
  Low pass constant for adaptive/kld.

*Note*: Set both fast and slow to 0 to disable adaptive sampling.

Parameters for KLD:
* `~particle_filter_kld_epsilon` (`double`, default: 0.05):
  KLD epsilon parameter
* `~particle_filter_kld_z` (`double`, default: 0.95):
  KLD z_(1-𝛿) parameter

Parameters for KLD and clustering:
* `~space_partitioning_resolution_xy` (`double`, default: 0.5):
  KLD bucket resolution x and y [m]
* `~space_partitioning_resolution_theta` (`double`, default: 10*pi/180):
  KLD bucket resolution rotation [rad]
