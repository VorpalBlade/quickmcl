#include "quickmcl/pose_2d.h"

#include <type_traits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace quickmcl;

using testing::DoubleEq;
using testing::Each;
using testing::Field;

template<typename T> class TestPose2D : public ::testing::Test
{
public:
  using PoseT = Pose2D<T>;
  using ScalarType = typename PoseT::Scalar;

  static_assert(std::is_same<T, ScalarType>::value, "Unexpected scalar type");
};

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_CASE(TestPose2D, MyTypes);

TYPED_TEST(TestPose2D, constructor)
{
  typename TestFixture::PoseT default_constructed;

  auto v = typename TestFixture::PoseT(0, 1, 2);

  EXPECT_FLOAT_EQ(0, v.x);
  EXPECT_FLOAT_EQ(1, v.y);
  EXPECT_FLOAT_EQ(2, v.theta);

  // Copy constructor
  auto u(v);
  auto t = u;

  EXPECT_FLOAT_EQ(0, t.x);
  EXPECT_FLOAT_EQ(1, t.y);
  EXPECT_FLOAT_EQ(2, t.theta);

  geometry_msgs::Pose p;
  p.position.x = 1;
  p.position.y = 2;
  p.orientation.z = std::sin(3.0 / 2.0);
  p.orientation.w = std::cos(3.0 / 2.0);

  auto pp = typename TestFixture::PoseT(p);
  EXPECT_FLOAT_EQ(1, pp.x);
  EXPECT_FLOAT_EQ(2, pp.y);
  EXPECT_FLOAT_EQ(3, pp.theta);
}

TYPED_TEST(TestPose2D, math)
{
  auto a = typename TestFixture::PoseT(0, 1, 0);
  auto b = typename TestFixture::PoseT(1, 0, M_PI / 4.0);

  auto c = a + b;
  c = c - static_cast<typename TestFixture::ScalarType>(2.0) * b;
  c.normalise();
  EXPECT_FLOAT_EQ(-1, c.x);
  EXPECT_FLOAT_EQ(1, c.y);
  EXPECT_FLOAT_EQ(-M_PI / 4.0, c.theta);
}

TYPED_TEST(TestPose2D, conversion)
{
  auto v = typename TestFixture::PoseT(3, 1, 2);

  geometry_msgs::Pose pose(v);
  EXPECT_FLOAT_EQ(3, pose.position.x);
  EXPECT_FLOAT_EQ(1, pose.position.y);
  EXPECT_FLOAT_EQ(2, tf2::getYaw(pose.orientation));

  // Check eigen transforms
  Eigen::Affine3d aff3(v);
  Eigen::Affine2f aff2(v);
  Eigen::Isometry2f iso2(v);

  Eigen::Vector2f r1 = aff2 * Eigen::Vector2f{1, 2};
  EXPECT_FLOAT_EQ(0.76525831, r1(0));
  EXPECT_FLOAT_EQ(1.0770037, r1(1));

  Eigen::Vector3d r2 = aff3 * Eigen::Vector3d{1, 2, 0};
  EXPECT_FLOAT_EQ(0.76525831, r2(0));
  EXPECT_FLOAT_EQ(1.0770037, r2(1));
  EXPECT_FLOAT_EQ(0, r2(2));

  Eigen::Vector2f r3 = iso2 * Eigen::Vector2f{1, 2};
  EXPECT_FLOAT_EQ(0.76525831, r3(0));
  EXPECT_FLOAT_EQ(1.0770037, r3(1));
}

TYPED_TEST(TestPose2D, to_decomposed)
{
  auto v = typename TestFixture::PoseT(2, 1, 2);

  auto dec = v.to_decomposed();

  EXPECT_FLOAT_EQ(2, dec(0));
  EXPECT_FLOAT_EQ(1, dec(1));
  EXPECT_FLOAT_EQ(std::sin(2.0), dec(2));
  EXPECT_FLOAT_EQ(std::cos(2.0), dec(3));
}

TYPED_TEST(TestPose2D, normalise)
{
  auto v = typename TestFixture::PoseT(0, 1, M_PI + 2);

  v.normalise();

  EXPECT_FLOAT_EQ(0, v.x);
  EXPECT_FLOAT_EQ(1, v.y);
  EXPECT_FLOAT_EQ(-M_PI + 2, v.theta);
}
