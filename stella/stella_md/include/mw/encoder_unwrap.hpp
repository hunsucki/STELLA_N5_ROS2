#pragma once

#include <cstdint>
#include <limits>

// The MD controller exposes its position counter through a C `long`, but the
// wire value is a signed 32-bit counter. Return the shortest count delta so a
// crossing from INT32_MAX to INT32_MIN remains a small forward movement.
inline std::int64_t unwrap_signed_32bit_delta(long current, long previous)
{
  const auto current_32 = static_cast<std::int32_t>(current);
  const auto previous_32 = static_cast<std::int32_t>(previous);
  auto delta = static_cast<std::int64_t>(current_32) - previous_32;
  constexpr auto full_range = std::int64_t{1} << 32;
  if (delta > std::numeric_limits<std::int32_t>::max()) {
    delta -= full_range;
  } else if (delta < std::numeric_limits<std::int32_t>::min()) {
    delta += full_range;
  }
  return delta;
}
