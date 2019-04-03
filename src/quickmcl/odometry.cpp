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
#include "quickmcl/odometry.h"

#include "quickmcl/config.h"
#include "quickmcl/utils.h"

#include <algorithm>
#include <cmath>
#include <random>

//! @file
//! @brief Functions and types for odometry

namespace quickmcl {

namespace {

//! Create a normal distribution
template<typename T> inline auto make_distribution(T variance)
{
#if QUICKMCL_ODOMETRY_USE_BROKEN == 1
  return std::normal_distribution<T>(0, std::abs(variance));
#else
  return std::normal_distribution<T>(0, std::sqrt(variance));
#endif
}

} // anonymous namespace

void sample_odometry(const Odometry &odom_new,
                     const Odometry &odom_old,
                     ParticleCollection *particles,
                     std::mt19937 *rng,
                     float alpha[4])
{
  // Odometry model into turn, drive, turn
  auto xdiff = odom_new.x - odom_old.x;
  auto ydiff = odom_new.y - odom_old.y;
  auto delta_trans = std::sqrt(xdiff * xdiff + ydiff * ydiff);
  // Based on AMCL: avoid numerical instability for close angles. Fixes rotating
  // in spot to some degree.
  decltype(Odometry::theta) delta_rot1 = 0.0;
  if (delta_trans >= 0.01) {
    delta_rot1 = angle_delta(std::atan2(ydiff, xdiff), odom_old.theta);
  }
  auto delta_theta_change = angle_delta(odom_new.theta, odom_old.theta);
  auto delta_rot2 = angle_delta(delta_theta_change, delta_rot1);

  // Based on AMCL: take the minimum noise of |delta_rot1| and |pi-delta_rot1|
  // (same for delta_rot2). This avoids stupidly large noise when driving
  // backwards.
  auto delta_rot1_for_noise =
      std::min(std::abs(delta_rot1), std::abs(angle_delta(M_PI, delta_rot1)));
  auto delta_rot2_for_noise =
      std::min(std::abs(delta_rot2), std::abs(angle_delta(M_PI, delta_rot2)));

  // Compute variances for noise
  auto trans_variance = alpha[2] * delta_trans * delta_trans +
                        alpha[3] * delta_rot1_for_noise * delta_rot1_for_noise +
                        alpha[3] * delta_rot2_for_noise * delta_rot2_for_noise;
  auto rot1_variance = alpha[0] * delta_rot1_for_noise * delta_rot1_for_noise +
                       alpha[1] * delta_trans * delta_trans;
  auto rot2_variance = alpha[0] * delta_rot2_for_noise * delta_rot2_for_noise +
                       alpha[1] * delta_trans * delta_trans;

  // Create the normal distribution objects
  auto trans_dist = make_distribution(trans_variance);
  auto rot1_dist = make_distribution(rot1_variance);
  auto rot2_dist = make_distribution(rot2_variance);

  // Process the particles
  for (WeightedParticle &particle : *particles) {
    // Add the noise
    auto delta_trans_hat = delta_trans - trans_dist(*rng);
    auto delta_rot1_hat = angle_delta(delta_rot1, rot1_dist(*rng));
    auto delta_rot2_hat = angle_delta(delta_rot2, rot2_dist(*rng));

    // Update the particles
    particle.data.x += static_cast<decltype(particle.data.x)>(
        delta_trans_hat * std::cos(particle.data.theta + delta_rot1_hat));
    particle.data.y += static_cast<decltype(particle.data.y)>(
        delta_trans_hat * std::sin(particle.data.theta + delta_rot1_hat));
    particle.data.theta += static_cast<decltype(particle.data.theta)>(
        delta_rot1_hat + delta_rot2_hat);
    // Normalise theta to [-pi,pi]
    particle.data.normalise();
  }
}

} // namespace quickmcl
