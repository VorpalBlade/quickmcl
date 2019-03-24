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

//! @file
//! @brief Build time configuration defines

//! @name Map
//! @{

//! Debugging: Publish z_hit as a hijacked pose array
#define MAP_LIKELIHOOD_DEBUG_PUB 0

//! @}

// ----------------------------------------------------------------------------

//! @name Odometry
//! @{

//! Debugging: Use broken odometry, confusing variance for standard devation.
//! The reason for this is to compare with default AMCL behaviour for debugging.
//! Do not tune for this!
#define ODOMETRY_USE_BROKEN 0

//! @}

// ----------------------------------------------------------------------------

//! @name Particle filter
//! @{

//! Debugging: Generate a single fixed particle
#define PF_FIXED_PARTICLE 0
//! Debugging: disable odometry
#define PF_DISABLE_ODOMETRY 0
//! Debugging: Print particles after sensor update
#define PF_PRINT_PARTICLES_AFTER_SENSOR 0
//! Debugging: Dump weights to csv file
#define PF_DUMP_WEIGHTS_TO_FILE 0

//! @}

// ----------------------------------------------------------------------------

//! @name Laser callback
//! @{

//! Debugging: define to ignore that the robot hasn't moved, and run particle
//! filter anyway.
#define LASER_CALLBACK_IGNORE_NO_MOVE 0

//! @}
