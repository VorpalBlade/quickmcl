#include "quickmcl/utils.h"

#include <gtest/gtest.h>

using namespace quickmcl;

TEST(TestUtils, normalise_angle)
{
  EXPECT_FLOAT_EQ(0.f, normalise_angle(0.f));
  EXPECT_FLOAT_EQ(0, normalise_angle(0));
  EXPECT_FLOAT_EQ(0, normalise_angle(2 * M_PI));
  EXPECT_FLOAT_EQ(M_PI - 0.1, normalise_angle(3 * M_PI - 0.1));
  EXPECT_FLOAT_EQ(0, normalise_angle(4 * M_PI));
  EXPECT_FLOAT_EQ(0, normalise_angle(-2 * M_PI));
  EXPECT_FLOAT_EQ(-M_PI + 0.1, normalise_angle(-3 * M_PI + 0.1));
  EXPECT_FLOAT_EQ(0, normalise_angle(-4 * M_PI));
}

TEST(TestUtils, angle_delta)
{
  EXPECT_FLOAT_EQ(-0.1, angle_delta(0.1, 0.2));
  EXPECT_FLOAT_EQ(0.1, angle_delta(0.2, 0.1));

  EXPECT_FLOAT_EQ(-0.1, angle_delta(-0.2, -0.1));
  EXPECT_FLOAT_EQ(0.1, angle_delta(-0.1, -0.2));

  EXPECT_FLOAT_EQ(-0.2, angle_delta(-0.1, 0.1));
  EXPECT_FLOAT_EQ(0.2, angle_delta(0.1, -0.1));

  EXPECT_FLOAT_EQ(0.0, angle_delta(0.1, 0.1));
  EXPECT_FLOAT_EQ(0.0, angle_delta(-0.1, -0.1));

  EXPECT_FLOAT_EQ(-0.2, angle_delta(M_PI - 0.1, -M_PI + 0.1));
  EXPECT_FLOAT_EQ(0.2, angle_delta(-M_PI + 0.1, M_PI - 0.1));

  EXPECT_FLOAT_EQ(0.1, angle_delta(-M_PI, M_PI - 0.1));
  EXPECT_FLOAT_EQ(-0.1, angle_delta(-M_PI, -M_PI + 0.1));

  EXPECT_FLOAT_EQ(0.1, angle_delta(M_PI, M_PI - 0.1));
  EXPECT_FLOAT_EQ(-0.1, angle_delta(M_PI, -M_PI + 0.1));
}
