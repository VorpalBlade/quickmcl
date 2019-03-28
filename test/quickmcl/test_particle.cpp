#include "quickmcl/particle.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace quickmcl;

using testing::DoubleEq;
using testing::Each;
using testing::Field;

TEST(TestParticle, effective_particles)
{
  ParticleCollection particles{
      WeightedParticle{{0, 0, 0}, 1},
      WeightedParticle{{0, 0, 0}, 1},
      WeightedParticle{{0, 0, 0}, 1},
  };
  ParticleCollection particles_2{
      WeightedParticle{{0, 0, 0}, 1},
      WeightedParticle{{0, 0, 0}, 0},
      WeightedParticle{{0, 0, 0}, 0},
  };
  equalise_particle_weights(&particles);
  EXPECT_DOUBLE_EQ(3, effective_particles(particles));
  EXPECT_DOUBLE_EQ(1, effective_particles(particles_2));
}

TEST(TestParticle, normalise_particle_weights)
{
  ParticleCollection particles{
      WeightedParticle{{0, 0, 0}, 0.2},
      WeightedParticle{{0, 0, 0}, 0.3},
      WeightedParticle{{0, 0, 0}, 0.5},
      WeightedParticle{{0, 0, 0}, 0.0},
      WeightedParticle{{0, 0, 0}, 2.0},
  };
  normalise_particle_weights(&particles);

  ParticleCollection normalised_particles{
      WeightedParticle{{0, 0, 0}, 0.2 / 3},
      WeightedParticle{{0, 0, 0}, 0.3 / 3},
      WeightedParticle{{0, 0, 0}, 0.5 / 3},
      WeightedParticle{{0, 0, 0}, 0.0},
      WeightedParticle{{0, 0, 0}, 2.0 / 3},
  };

  for (size_t i = 0; i < particles.size(); i++) {
    EXPECT_DOUBLE_EQ(normalised_particles[i].weight, particles[i].weight)
        << "i = " << i;
  }
}

TEST(TestParticle, equalise_particle_weights)
{
  ParticleCollection particles{
      WeightedParticle{{0, 0, 0}, 0.2},
      WeightedParticle{{0, 0, 0}, 0.3},
      WeightedParticle{{0, 0, 0}, 0.4},
      WeightedParticle{{0, 0, 0}, 0.0},
      WeightedParticle{{0, 0, 0}, 231.0},
  };
  equalise_particle_weights(&particles);
  EXPECT_THAT(particles,
              Each(Field(&WeightedParticle::weight, DoubleEq(1.0 / 5.0))));
}

TEST(TestParticle, running_sum)
{
  ParticleCollection particles{
      WeightedParticle{{0, 0, 0}, 0.2},
      WeightedParticle{{0, 0, 0}, 0.3},
      WeightedParticle{{0, 0, 0}, 0.4},
      WeightedParticle{{0, 0, 0}, 0.0},
      WeightedParticle{{0, 0, 0}, 0.1},
  };

  ParticlesRunningSum sum_map(particles);

  EXPECT_EQ(0, sum_map.lookup_index(0.0));
  EXPECT_EQ(0, sum_map.lookup_index(0.1));
  EXPECT_EQ(0, sum_map.lookup_index(0.199));

  EXPECT_EQ(1, sum_map.lookup_index(0.2));
  EXPECT_EQ(1, sum_map.lookup_index(0.499));

  EXPECT_EQ(2, sum_map.lookup_index(0.5));
  EXPECT_EQ(2, sum_map.lookup_index(0.899));

  EXPECT_EQ(4, sum_map.lookup_index(0.9));
  EXPECT_EQ(4, sum_map.lookup_index(1));

  EXPECT_DOUBLE_EQ(1.0, sum_map.get_total_weight());
}
