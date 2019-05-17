#include "quickmcl/distance_calculator.h"

#include <gtest/gtest.h>

using namespace quickmcl;

TEST(TestDistanceCalculator, create_distance_cache)
{
  auto cache = create_distance_cache(1, 4);
  EXPECT_EQ(6, cache.cols());
  EXPECT_EQ(6, cache.rows());

  // Sample some positions:
  EXPECT_FLOAT_EQ(0.f, cache(0, 0));

  EXPECT_FLOAT_EQ(1.f, cache(0, 1));
  EXPECT_FLOAT_EQ(1.f, cache(1, 0));

  EXPECT_FLOAT_EQ(std::sqrt(2), cache(1, 1));
  EXPECT_FLOAT_EQ(std::sqrt(5), cache(1, 2));
  EXPECT_FLOAT_EQ(std::sqrt(5), cache(2, 1));
}

static nav_msgs::OccupancyGrid make_test_map()
{
  nav_msgs::OccupancyGrid map;
  map.info.resolution = 0.5;
  map.info.width = 5;
  map.info.height = 5;
  map.data = {
      -1, -1, -1,  -1, 100, 100, 100, 100, 100, 100, 100, 0, 0,
      0,  0,  100, 0,  0,   0,   0,   0,   0,   0,   0,   0,
  };
  return map;
}

TEST(TestDistanceCalculator, compute_distance_map)
{
  float max_dist = 2;
  const nav_msgs::OccupancyGrid map = make_test_map();
  MapContainer output;

  MapContainer expected_output(5, 5);
  expected_output << 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0, 0, 0.5, 0.5, 0.5,
      0.5, 0, 0.5, 1, 1, 1, 0.5, sqrt(0.5 * 0.5 + 0.5 * 0.5),
      sqrt(0.5 * 0.5 + 1), 1.5, 1.5;

  compute_distance_map(max_dist, map, &output);
  for (Eigen::Index y = 0; y < expected_output.cols(); y++) {
    for (Eigen::Index x = 0; x < expected_output.rows(); x++) {
      EXPECT_FLOAT_EQ(expected_output(y, x), output(y, x))
          << "x=" << x << " y=" << y;
    }
  }
}

TEST(TestDistanceCalculator, compute_state_map)
{
  const nav_msgs::OccupancyGrid map = make_test_map();
  MapStateContainer map_state;

  MapStateContainer expected_state(5, 5);
  expected_state << MapState::Unknown, MapState::Unknown, MapState::Unknown,
      MapState::Unknown, MapState::Occupied, MapState::Occupied,
      MapState::Occupied, MapState::Occupied, MapState::Occupied,
      MapState::Occupied, MapState::Occupied, MapState::Free, MapState::Free,
      MapState::Free, MapState::Free, MapState::Occupied, MapState::Free,
      MapState::Free, MapState::Free, MapState::Free, MapState::Free,
      MapState::Free, MapState::Free, MapState::Free, MapState::Free;

  compute_state_map(map, &map_state);
  for (Eigen::Index y = 0; y < expected_state.cols(); y++) {
    for (Eigen::Index x = 0; x < expected_state.rows(); x++) {
      EXPECT_EQ(expected_state(y, x), map_state(y, x))
          << "x=" << x << " y=" << y;
    }
  }
}
