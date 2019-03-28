#include "quickmcl/low_pass_filter.h"

#include <type_traits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace quickmcl;

using testing::DoubleEq;
using testing::Each;
using testing::Field;

template<typename T> class TestLowPassFilter : public ::testing::Test
{
public:
  using FilterT = LowPassFilter<T>;
  using ValueT = typename FilterT::value_type;

  static_assert(std::is_same<T, ValueT>::value, "Unexpected value_type");
};

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_CASE(TestLowPassFilter, MyTypes);

TYPED_TEST(TestLowPassFilter, functionality)
{
  typename TestFixture::FilterT filter;

  filter.set_alpha(0.1);
  filter.update(0.7);
  for (int i = 0; i < 15; i++) {
    filter.update(0.9);
  }
  filter.update(0.8);
  EXPECT_FLOAT_EQ(0.85293961, filter.get());
  filter.update(0.2);
  EXPECT_FLOAT_EQ(0.78764564, filter.get());

  filter.reset(0.5);
  EXPECT_FLOAT_EQ(0.5, filter.get());
}
