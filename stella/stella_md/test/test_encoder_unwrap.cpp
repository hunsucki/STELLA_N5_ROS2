#include "encoder_unwrap.hpp"

#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

TEST(EncoderUnwrap, PreservesOrdinaryMovement)
{
  EXPECT_EQ(unwrap_signed_32bit_delta(105, 100), 5);
  EXPECT_EQ(unwrap_signed_32bit_delta(95, 100), -5);
}

TEST(EncoderUnwrap, CrossesPositiveBoundaryInBothDirections)
{
  constexpr auto high = std::numeric_limits<std::int32_t>::max();
  constexpr auto low = std::numeric_limits<std::int32_t>::min();
  EXPECT_EQ(unwrap_signed_32bit_delta(low + 2L, high - 2L), 5);
  EXPECT_EQ(unwrap_signed_32bit_delta(high - 2L, low + 2L), -5);
}

TEST(EncoderUnwrap, TreatsWireValueAs32BitOn64BitLinux)
{
  constexpr long wrapped_high = (std::int64_t{1} << 32) - 3;
  EXPECT_EQ(unwrap_signed_32bit_delta(2, wrapped_high), 5);
}
