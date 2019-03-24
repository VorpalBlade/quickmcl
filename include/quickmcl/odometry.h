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
#pragma once

#include "quickmcl/particle.h"
#include "quickmcl/pose_2d.h"

#include <random>

//! @file
//! @brief Functions and types for odometry

namespace quickmcl {

//! Type for odometry readings
using Odometry = Pose2D<double>;

//! Sample using motion model. Updates all particles.
//!
//! @param odom_new   Estimated x_t
//! @param odom_old   Estimated x_(t-1)
//! @param particles  Particles to process
//! @param rng        Random number generator to use
//! @param alpha      Weights for variance
void sample_odometry(const Odometry &odom_new,
                     const Odometry &odom_old,
                     ParticleCollection &particles,
                     std::mt19937 &rng,
                     float alpha[4]);

} // namespace quickmcl
