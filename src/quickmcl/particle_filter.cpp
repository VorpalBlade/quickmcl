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
#include "quickmcl/particle_filter.h"

#include "quickmcl/config.h"
#include "quickmcl/map.h"
#include "quickmcl/random.h"
#include "quickmcl/statistics.h"

#include <algorithm>
#include <cassert>
#include <random>
#include <ros/console.h>

#if PF_DUMP_WEIGHTS_TO_FILE == 1
#include <fstream>
#endif

//! @file
//! @brief Actual particle filter.

namespace quickmcl {

ParticleFilter::ParticleFilter(const std::shared_ptr<Map> &map)
  : map(map)
{}

void ParticleFilter::initialise(
    const WeightedParticle::ParticleT &starting_point,
    const Eigen::Matrix3f &covariance)
{
  auto particle_count = static_cast<size_t>(parameters.particle_count_min);
  ParticleCollection &cloud = data_sets[current_data_set].particles;
  cloud.clear();
  cloud.reserve(particle_count);

#if PF_FIXED_PARTICLE == 1
  cloud.push_back(WeightedParticle{starting_point, 1});
  return;
#endif
  Eigen::Vector3f initial_pose{
      starting_point.x, starting_point.y, starting_point.theta};
  normal_random_variable<Eigen::Vector3f, Eigen::Matrix3f> pose_dist(
      initial_pose, covariance, &rng);

  double weight = 1.0 / particle_count;
  // Create a cloud around the specified point
  for (size_t i = 0; i < particle_count; i++) {
    auto pose_vec = pose_dist();
    auto particle =
        WeightedParticle::ParticleT{pose_vec(0), pose_vec(1), pose_vec(2)};
    particle.normalise();
    cloud.push_back(WeightedParticle{particle, weight});
  }
}

void ParticleFilter::global_localization()
{
  auto particle_count = static_cast<size_t>(parameters.particle_count_max);
  ParticleCollection &cloud = data_sets[current_data_set].particles;
  cloud.clear();
  cloud.reserve(particle_count);
  double weight = 1.0 / particle_count;
  // Create a cloud around the specified point
  for (size_t i = 0; i < particle_count; i++) {
    auto particle = map->generate_random_pose();
    cloud.push_back(WeightedParticle{particle, weight});
  }
}

void ParticleFilter::handle_odometry(const Odometry &odom_new,
                                     const Odometry &odom_old,
                                     float alpha[4])
{
#if PF_DISABLE_ODOMETRY == 0
  sample_odometry(
      odom_new, odom_old, &data_sets[current_data_set].particles, &rng, alpha);
#endif
}

void ParticleFilter::update_importance_from_observations(
    const LaserPointCloud &cloud)
{
  auto total_weight =
      map->update_importance(cloud, &data_sets[current_data_set].particles);
  // Scale weights, if non-zero total, if we have a zero total
  if (total_weight > 0) {
    normalise_weights(total_weight);
    // For adaptive sampling:
    auto avg_weight =
        total_weight / data_sets[current_data_set].particles.size();
    w_slow.update(avg_weight);
    w_fast.update(avg_weight);
    ROS_DEBUG_NAMED("particle_filter",
                    "Scan report: tot=%f w_slow=%f w_fast=%f",
                    total_weight,
                    w_slow.get(),
                    w_fast.get());
  } else {
    ROS_INFO_NAMED(
        "particle_filter",
        "Oops, zero total weight, equalising and hoping for the best!");
    // Set to average weight, following AMCL on this one.
    // At least this way we will not get NaNs.
    equalise_weights();
  }
#if PF_PRINT_PARTICLES_AFTER_SENSOR == 1
  for (const auto &p : particles[current_cloud]) {
    ROS_INFO_STREAM_NAMED("particle_filter",
                          "Particle " << p.data << ": " << p.weight);
  }
#endif
#if PF_DUMP_WEIGHTS_TO_FILE == 1
  std::ofstream debug_data("weights.csv",
                           std::ios_base::app | std::ios_base::out);
  for (const auto &p : data_sets[current_data_set].particles) {
    debug_data << p.weight << ",";
  }
  debug_data << std::endl;
  debug_data.close();
#endif
}

void ParticleFilter::resample_and_cluster()
{
  const auto &input_set = data_sets[current_data_set];
  auto &output_set = data_sets[(current_data_set + 1) % 2];
  current_data_set = (current_data_set + 1) % 2;

  switch (parameters.resample_type) {
  case ResampleType::low_variance:
    resample_low_variance(input_set.particles, &output_set.particles);
    cluster();
    break;
  case ResampleType::adaptive:
    resample_adaptive(input_set.particles, &output_set.particles);
    cluster();
    break;
  case ResampleType::kld:
    resample_kld(input_set, &output_set);
    // Already clustered as a side effect of KLD sampling
    break;
  }
  // Normalise the weights afterwards, we don't know what state it currently is
  // in, and this will negatively impact get_estimated_pose.
  normalise_weights();
}

void ParticleFilter::cluster()
{
  // Do clustering, used when not using KLD resampling or when not resampling
  // this iteration.
  space_partitioning.reset();

  for (const auto &p : data_sets[current_data_set].particles) {
    space_partitioning.add_point(p.data);
  }

  // Do the clustering
  space_partitioning.assign_clusters();
}

void ParticleFilter::set_parameters(
    const Parameters::ParticleFilter &parameters)
{
  this->parameters = parameters;
  space_partitioning.set_parameters(parameters);
  w_fast.set_alpha(parameters.alpha_fast);
  w_slow.set_alpha(parameters.alpha_slow);
}

bool ParticleFilter::get_estimated_pose(Pose2D<double> *pose,
                                        Eigen::Matrix3d *covariance) const
{
  ParticleCloudStatistics global_stats;
  auto clusters = compute_cluster_statistics(&global_stats);

  double best_weight = -1;
  const decltype(clusters)::value_type *best_cluster = nullptr;
  // Find most likely cluster
  for (const auto &cluster : clusters) {
    if (best_weight < cluster.total_weight) {
      best_weight = cluster.total_weight;
      best_cluster = &cluster;
    }
  }
  if (best_cluster && best_weight > 0) {
    *pose = best_cluster->as_pose();
    // Use global filter statistics for the covariance.
    *covariance = global_stats.covariance;
    return true;
  } else {
    ROS_WARN_NAMED("particle_filter",
                   "ALL particle clusters suck (or there were no valid "
                   "particles)! Not publishing.");
    return false;
  }
}

void ParticleFilter::resample_low_variance(
    const ParticleCollection &input_particles,
    ParticleCollection *output_particles)
{
  const auto M = input_particles.size();
  decltype(WeightedParticle::weight) minv = 1.0f / M;

  output_particles->resize(M);
  // Get the offset
  auto dist = std::uniform_real_distribution<decltype(minv)>(0.f, minv);
  auto r = dist(rng);
  auto c = input_particles.at(0).weight;
  size_t i = 0;
  for (size_t m = 0; m < M; m++) {
    decltype(minv) U = r + m * minv;
    while (U > c) {
      i++;
      // Handle wrap-around? Never seems to happen though...
      // Maybe because the collection size currently never changes and the
      // weights are normalised?
      assert(i < M);
      c += input_particles.at(i).weight;
    }
    (*output_particles)[m] = input_particles.at(i);
  }
}

void ParticleFilter::resample_adaptive(
    const ParticleCollection &input_particles,
    ParticleCollection *output_particles)
{
  // Compute "adaptiveness-factor"
  const auto w_diff = std::max(0.0, 1.0 - w_fast.get() / w_slow.get());

  // if (w_diff > 0.0) {
  const auto output_size = static_cast<size_t>(parameters.particle_count_min);
  output_particles->resize(output_size);
  std::uniform_real_distribution<decltype(WeightedParticle::weight)> uni_dist(
      0, 1);
  // Create a precomputed map of total weights sums.
  ParticlesRunningSum intervals(input_particles);
  if (intervals.get_total_weight() <= 0) {
    ROS_ERROR_NAMED("particle_filter",
                    "No particles have any weights! Failing to resample");
    return;
  }

  for (size_t m = 0; m < output_size; m++) {
    auto p = uni_dist(rng);
    if (p < w_diff) {
      // Draw random particle
      (*output_particles)[m].data = map->generate_random_pose();
      // Don't want the weight to affect the estimated pose, until next filter
      // iteration.
      (*output_particles)[m].weight = 0;
    } else {
      // Draw as normal, using naive resampler. This relies on the weights
      // summing to 1.
      auto r = uni_dist(rng);
      auto index = intervals.lookup_index(r);
      (*output_particles)[m] = input_particles.at(index);
    }
  }
  //} else {
  //  // No need for adaptive sampling, just run low_variance sampling instead,
  //  // to get the advantages of it.
  //  resample_low_variance(input_particles, output_particles);
  //}

  // Reset filters if we were adaptively resampling
  if (w_diff > 0.0) {
    ROS_INFO_NAMED("particle_filter",
                   "Adaptive resampling took place since w_diff=%f",
                   w_diff);
    w_fast.reset(0);
    w_slow.reset(0);
  }
}

void ParticleFilter::resample_kld(const DataSet &input_set, DataSet *output_set)
{
  // Reset KLD state.
  space_partitioning.reset();
  // Compute "adaptiveness-factor"
  const auto w_diff = std::max(0.0, 1.0 - w_fast.get() / w_slow.get());

  const auto max_particles = static_cast<size_t>(parameters.particle_count_max);
  output_set->particles.clear();
  output_set->particles.reserve(max_particles);
  std::uniform_real_distribution<decltype(WeightedParticle::weight)> uni_dist(
      0, 1);
  // Create a precomputed map of total weights sums.
  ParticlesRunningSum intervals(input_set.particles);
  if (intervals.get_total_weight() <= 0) {
    ROS_ERROR_NAMED("particle_filter",
                    "No particles have any weights! Failing to resample");
    return;
  }

  for (size_t m = 0; m < max_particles; m++) {
    auto p = uni_dist(rng);
    if (p < w_diff) {
      // Draw random particle
      // Note: We don't want the weight to affect the estimated pose, until next
      // filter iteration.
      WeightedParticle particle{map->generate_random_pose(), 0};
      output_set->particles.push_back(particle);
    } else {
      // Draw as normal, using naive resampler. This relies on the weights
      // summing to 1.
      auto r = uni_dist(rng);
      auto index = intervals.lookup_index(r);
      output_set->particles.push_back(input_set.particles.at(index));
    }
    // KLD stuff:
    space_partitioning.add_point(output_set->particles.rbegin()->data);
    // Check if we are done according to KLD.
    // ROS_INFO("KLD: %zu count=%zu", space_partitioning.limit(), m);
    if (m >= space_partitioning.limit()) {
      break;
    }
  }

  // Reset filters if we were adaptively resampling
  if (w_diff > 0.0) {
    ROS_INFO_NAMED("particle_filter",
                   "Adaptive resampling took place since w_diff=%f",
                   w_diff);
    w_fast.reset(0);
    w_slow.reset(0);
  }

  // Do the clustering
  space_partitioning.assign_clusters();
}

void ParticleFilter::normalise_weights()
{
  normalise_particle_weights(&data_sets[current_data_set].particles);
}

void ParticleFilter::normalise_weights(double total_weight)
{
  normalise_particle_weights(&data_sets[current_data_set].particles,
                             total_weight);
}

void ParticleFilter::equalise_weights()
{
  equalise_particle_weights(&data_sets[current_data_set].particles);
}

std::vector<ParticleCloudStatistics> ParticleFilter::compute_cluster_statistics(
    ParticleCloudStatistics *global_statistics) const
{
  std::vector<ParticleCloudStatistics> clusters(
      space_partitioning.get_cluster_count());

  // Update all the cluster statistics
  for (const auto &particle : data_sets[current_data_set].particles) {
    auto cluster_id = space_partitioning.get_cluster_id(particle.data);
    if (cluster_id == SpacePartitioning::INVALID_CLUSTER_ID) {
      ROS_FATAL_NAMED("particle_filter",
                      "Invalid cluster ID. Shouldn't happen!");
      abort();
    }
    clusters[cluster_id].add(particle);
  }

  // Compute global cluster and finalise.
  for (auto &cluster : clusters) {
    global_statistics->add(cluster);
    cluster.finalise();
  }
  global_statistics->finalise();

  return clusters;
}

} // namespace quickmcl
