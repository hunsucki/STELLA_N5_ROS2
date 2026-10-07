#include <gtest/gtest.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <thread>
#include "serial.h"
#include "serial_deadline.hpp"

TEST(MotorSerial, SilentControllerTimesOutAndReleasesSerialLock)
{
  const int master = posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  serial::Serial port(ptsname(master), 115200, serial::Timeout::simpleTimeout(10));
  ASSERT_TRUE(port.isOpen());
  const auto start = std::chrono::steady_clock::now();
  {
    motor_serial::Deadline deadline{std::chrono::milliseconds(40)};
    EXPECT_THROW(while (port.available() == 0) {}, std::runtime_error);
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_GE(elapsed, std::chrono::milliseconds(40));
  EXPECT_LT(elapsed, std::chrono::milliseconds(250));
  const char packet = 'x';
  ASSERT_EQ(write(master, &packet, 1), 1);
  uint8_t received = 0;
  EXPECT_EQ(port.read(&received, 1), 1U);
  EXPECT_EQ(received, 'x');
  port.close();
  close(master);
}

TEST(MotorSerial, CommandAcknowledgementMustBeCompleteAndFitVendorBuffer)
{
  const int master = posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  serial::Serial port(ptsname(master), 115200, serial::Timeout::simpleTimeout(10));
  std::string reply;
  {
    motor_serial::Deadline deadline{std::chrono::milliseconds(100)};
    ASSERT_EQ(write(master, "OK\r", 3), 3);
    EXPECT_EQ(port.readline(reply, 65536, "\r"), 3U);
    EXPECT_EQ(reply, "OK\r");
  }
  reply.clear();
  {
    motor_serial::Deadline deadline{std::chrono::milliseconds(100)};
    ASSERT_EQ(write(master, "partial", 7), 7);
    EXPECT_THROW(port.readline(reply, 65536, "\r"), std::runtime_error);
    EXPECT_TRUE(reply.empty());
  }
  {
    motor_serial::Deadline deadline{std::chrono::milliseconds(100)};
    const std::string oversized(97, 'x');
    ASSERT_EQ(write(master, oversized.data(), oversized.size()), 97);
    EXPECT_THROW(port.readline(reply, 65536, "\r"), std::runtime_error);
    EXPECT_TRUE(reply.empty());
  }
  close(master);
}

TEST(MotorSerial, DeadlineDoesNotLeakIntoAnotherThreadOrAfterException)
{
  {
    motor_serial::Deadline deadline{std::chrono::milliseconds(0)};
    EXPECT_THROW(motor_serial::check_deadline(), std::runtime_error);
    std::thread other([] {EXPECT_NO_THROW(motor_serial::check_deadline());});
    other.join();
  }
  EXPECT_NO_THROW(motor_serial::check_deadline());
}

TEST(MotorSerial, NestedDeadlineCannotExtendOuterDeadline)
{
  motor_serial::Deadline outer{std::chrono::milliseconds(0)};
  motor_serial::Deadline inner{std::chrono::seconds(1)};
  EXPECT_THROW(motor_serial::check_deadline(), std::runtime_error);
}
