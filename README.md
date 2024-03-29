# QuickMCL

**NOTE**: I no longer work in the field of robotics, this is not maintained. But
hopefully the ideas and techniques presented here might be of use to you. Or the
software itself might even still work!

---

This package implements an MCL localisation node based on (Thrun et al., 2005)
as well as on reverse engineering what [AMCL](https://wiki.ros.org/amcl) does.

The CPU and memory usage of QuickMCL compared to AMCL is analysed in
[this blog post][performance], and it turns out QuickMCL is way better.

This was done as course-work for a robotics master program, but I thought it
turned out good enough it was worth putting it up publicly after the fact. I
might make improvements to it after the fact if there is any interest in the
project.

This has been tested with ROS Kinetic, Melodic & Noetic, but I see no reason
why it shouldn't work with newer ROS.

Just like AMCL this package estimates the location of the `base_link` TF frame
in the `map` frame, but then publishes the difference between `map` and `odom`.

As inputs to the node the change over time in the `odom` to `base_link`
transform is used, as well as the point cloud from the laser scanner. In
addition a map is required.

## Installation

You can build the software as normal ROS 1 software. However, note that
QuickMCL performs poorly in debug builds. This is due to the use of the Eigen
library. An optimised build is essential to good performance. The difference
in processing time is about 20x!

If you use catkin_make build with
`catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release`. If you use catkin-tools
you can set the CMake flags using `catkin config`, for example:
`catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`

If you want to build parts of your software as a debug build still, consider
taking a look at [workspace overlaying](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying).


## Documentation

How to use the software is documented in a [separate file](doc/using.md).

There are also example launch files, in particular the one for
[the node of this package](launch/localiser.launch) may be of interest. All
parameters and ROS names can be overridden here. In addition there are extra
launch files for related nodes that are needed or are useful in combination with
this package.

As for code documentation: Doxygen documentation can be generated to the
directory `doc_generated` by running `doxygen` in the directory of this file.

## Limitations

Compared to AMCL:

* Only the likelihood field model is implemented, not the beam model.
* No support for beam skipping.
* Only odometry type implemented is differential drive.

Shared with AMCL:

* The map offset is taken into account, but not the rotation.

## Extra features compared with AMCL

* QuickMCL can either use a `PointCloud2` topic (in the `base_link` TF frame) or
  a `LaserScan` topic for the sensor data. See the
  [launch file for the laser filter](launch/laser_filter.launch) for an example
  of how a node that converts from the laser scan to a point cloud can look.
* Multiple resampling types implemented (low variance, adaptive, KLD).
* Some parameters that are hard coded in AMCL (such as KLD bucket sizes) are
  tunable via ROS parameters.
* Code is more documented.
* Some bugs in AMCL have not been "reimplemented"!
  * AMCL has incorrect variance handling in the odometry models by default. You
    have to explicitly select "corrected" models with it.
  * Clustering wraparound for the circle cut (pi/-pi) fixed.
    ([AMCL bug](https://github.com/ros-planning/navigation/issues/27))
* Checks if particles are inside free space and penalises particles that aren't.
* [More efficient resampling][performance] with respect to performance when the
  number of particles is large.
* [Lower memory usage][performance] than AMCL.

## References
* Thrun, S., Burgard, W., Fox, D., 2005. Probabilistic robotics, Intelligent
  robotics and autonomous agents. MIT Press, Cambridge, Mass.

[performance]: <https://vorpal.se/posts/2019/apr/07/quickmcl-vs-amcl-performance/>
