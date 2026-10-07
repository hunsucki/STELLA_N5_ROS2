#pragma once

#include <chrono>
#include <stdexcept>

namespace motor_serial
{
// The vendor ReadPacket loop polls Serial::available() and does not enforce
// its timeout argument. Apply a deadline at that boundary, including retries.
struct DeadlineState
{
  bool active = false;
  std::chrono::steady_clock::time_point end;
};

inline DeadlineState & deadline_state()
{
  static thread_local DeadlineState state;
  return state;
}

class Deadline
{
public:
  explicit Deadline(std::chrono::milliseconds timeout)
  : previous_(deadline_state())
  {
    auto & state = deadline_state();
    state.active = true;
    state.end = std::chrono::steady_clock::now() + timeout;
    if (previous_.active && previous_.end < state.end) {
      state.end = previous_.end;
    }
  }
  ~Deadline() {deadline_state() = previous_;}
  Deadline(const Deadline &) = delete;
  Deadline & operator=(const Deadline &) = delete;

private:
  DeadlineState previous_;
};

inline void check_deadline()
{
  const auto & state = deadline_state();
  if (state.active && std::chrono::steady_clock::now() >= state.end) {
    throw std::runtime_error("Motor serial transaction deadline exceeded");
  }
}
}  // namespace motor_serial
