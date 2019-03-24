#include "quickmcl/statistics.h"

#include <gtest/gtest.h>

using namespace quickmcl;

static void compare_statistics(const ParticleCloudStatistics &a,
                               const ParticleCloudStatistics &b)
{
  EXPECT_DOUBLE_EQ(a.total_weight, a.total_weight);
  EXPECT_EQ(a.sample_count, b.sample_count);

  for (Eigen::Index i = 0; i < 3; i++) {
    EXPECT_DOUBLE_EQ(a.mean(i), b.mean(i));
  }

  for (Eigen::Index x = 0; x < 3; x++) {
    for (Eigen::Index y = 0; y < 3; y++) {
      EXPECT_DOUBLE_EQ(a.covariance(y, x), b.covariance(y, x));
    }
  }
  auto a_pose = a.as_pose();
  auto b_pose = b.as_pose();
  EXPECT_DOUBLE_EQ(a_pose.x, b_pose.x);
  EXPECT_DOUBLE_EQ(a_pose.y, b_pose.y);
  EXPECT_DOUBLE_EQ(a_pose.theta, b_pose.theta);
}

TEST(TestStatistics, functionality)
{
  ParticleCloudStatistics stats_a;
  ParticleCloudStatistics stats_b;
  ParticleCloudStatistics stats_c;
  ParticleCloudStatistics stats_d;
  ParticleCollection particles_a{
      WeightedParticle{{1, 0, 0}, 0.2 / 3},
      WeightedParticle{{0, 1, 0}, 0.3 / 3},
      WeightedParticle{{0, 0, 1}, 0.5 / 3},
      WeightedParticle{{2, 0, 0.5}, 0.0},
      WeightedParticle{{0, 2, 0.5}, 2.0 / 3},
  };
  ParticleCollection particles_b{
      WeightedParticle{{0.5, 2.0, -1.5}, 0.2},
      WeightedParticle{{3, 0.5, 0}, 0.5},
  };

  // Test adding individual particles as well as entire clouds.
  for (const auto &particle : particles_a) {
    stats_a.add(particle);
  }
  stats_b.add(particles_a);

  // Test adding statistics objects
  stats_c.add(particles_a);
  stats_c.add(particles_b);

  stats_d.add(stats_a);
  stats_d.add(particles_b);

  // Finalise all
  stats_a.finalise();
  stats_b.finalise();
  stats_c.finalise();
  stats_d.finalise();

  // A and B should be equal
  EXPECT_EQ(5, stats_a.sample_count);
  EXPECT_EQ(5, stats_b.sample_count);
  compare_statistics(stats_a, stats_b);

  // C and D should be equal
  EXPECT_EQ(7, stats_c.sample_count);
  EXPECT_EQ(7, stats_d.sample_count);
  compare_statistics(stats_c, stats_d);

  // And then compare to the expected values
  Eigen::Vector3d expected_mean{
      0.98039215686274517, 1.2254901960784315, 0.18971050466256337};
  for (Eigen::Index i = 0; i < 3; i++) {
    EXPECT_DOUBLE_EQ(expected_mean(i), stats_c.mean(i));
  }
  Eigen::Matrix3d expected_covariance;
  expected_covariance << 1.7545174932718179, -0.64263744713571724, 0,
      -0.64263744713571724, 0.66974240676662777, 0, 0, 0, -0.38069067304713533;
  for (Eigen::Index x = 0; x < 3; x++) {
    for (Eigen::Index y = 0; y < 3; y++) {
      EXPECT_DOUBLE_EQ(expected_covariance(y, x), stats_c.covariance(y, x))
          << "(" << x << ", " << y << ")";
    }
  }

  // Check getting the pose
  Pose2D<double> expected_pose(expected_mean(0), expected_mean(1), expected_mean(2));
  auto actual_pose = stats_c.as_pose();

  EXPECT_DOUBLE_EQ(expected_pose.x, actual_pose.x);
  EXPECT_DOUBLE_EQ(expected_pose.y, actual_pose.y);
  EXPECT_DOUBLE_EQ(expected_pose.theta, actual_pose.theta);
}
