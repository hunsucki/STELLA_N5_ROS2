#include <gtest/gtest.h>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

extern "C" ssize_t dds_test_send(int, const msghdr *, int);
extern "C" int sendmsg_test_flags();
extern "C" void sendmsg_test_error(int);

TEST(DdsUdpGuard, CongestedUdpReturnsImmediatelyWithoutInfiniteRetry)
{
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(fd, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  msghdr msg{};
  msg.msg_name = &address;
  msg.msg_namelen = sizeof(address);
  sendmsg_test_error(EAGAIN);
  EXPECT_EQ(dds_test_send(fd, &msg, MSG_NOSIGNAL), -1);
  EXPECT_EQ(errno, ENOBUFS);
  EXPECT_EQ(sendmsg_test_flags(), MSG_NOSIGNAL | MSG_DONTWAIT);
  timeval timeout{};
  socklen_t size = sizeof(timeout);
  ASSERT_EQ(getsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, &size), 0);
  EXPECT_EQ(timeout.tv_sec, 0);
  EXPECT_EQ(timeout.tv_usec, 0);  // No socket-wide timeout changes.
  sendmsg_test_error(EMSGSIZE);
  EXPECT_EQ(dds_test_send(fd, &msg, 0), -1);
  EXPECT_EQ(errno, EMSGSIZE);
  close(fd);
}

TEST(DdsUdpGuard, OtherCallersAndTcpKeepTheirOriginalBehavior)
{
  const int udp = socket(AF_INET, SOCK_DGRAM, 0);
  const int tcp = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  msghdr msg{};
  msg.msg_name = &address;
  msg.msg_namelen = sizeof(address);
  sendmsg_test_error(EAGAIN);
  EXPECT_EQ(sendmsg(udp, &msg, MSG_NOSIGNAL), -1);
  EXPECT_EQ(errno, EAGAIN);
  EXPECT_EQ(sendmsg_test_flags(), MSG_NOSIGNAL);
  timeval timeout{};
  socklen_t size = sizeof(timeout);
  ASSERT_EQ(getsockopt(udp, SOL_SOCKET, SO_SNDTIMEO, &timeout, &size), 0);
  EXPECT_EQ(timeout.tv_sec, 0);
  EXPECT_EQ(timeout.tv_usec, 0);
  EXPECT_EQ(dds_test_send(tcp, &msg, MSG_NOSIGNAL), -1);
  EXPECT_EQ(errno, EAGAIN);
  EXPECT_EQ(sendmsg_test_flags(), MSG_NOSIGNAL);
  close(udp);
  close(tcp);
}

#include <cstdlib>
#include <string>
#include <thread>
#include <vector>
extern "C" void dds_test_log(const char *, const char *, int);
extern "C" void dds_test_long_log(const char *);
extern "C" void dds_log_test_reset();
extern "C" int dds_log_test_calls();
extern "C" uint32_t dds_log_test_priority();
extern "C" const char * dds_log_test_message();

TEST(DdsLogThrottle, OtherErrorsAndLongMessagesKeepContentAndSeverity)
{
  dds_log_test_reset();
  dds_test_log("ddsi_udp_conn_write", "udp/192.168.0.19:7410", -3);
  EXPECT_EQ(dds_log_test_calls(), 1);
  EXPECT_EQ(dds_log_test_priority(), 2u);
  EXPECT_STREQ(dds_log_test_message(),
    "ddsi_udp_conn_write to udp/192.168.0.19:7410 failed with retcode -3\n");
  dds_test_log("unrelated", "udp/192.168.0.19:7410", -5);
  EXPECT_EQ(dds_log_test_calls(), 2);
  EXPECT_EQ(dds_log_test_priority(), 2u);
  const std::string long_message(8192, 'a');
  dds_test_long_log(long_message.c_str());
  EXPECT_EQ(std::string(dds_log_test_message()), long_message);
}

TEST(DdsLogThrottle, ConcurrentResourceFloodIsSummarizedWithoutChangingSendFailure)
{
  dds_log_test_reset();
  std::vector<std::thread> workers;
  for (int i = 0; i < 8; ++i) {
    workers.emplace_back([]() {
        for (int n = 0; n < 500; ++n) {
          dds_test_log("ddsi_udp_conn_write", "udp/192.168.0.19:7410", -5);
        }
      });
  }
  for (auto & worker : workers) {worker.join();}
  const char * setting = std::getenv("STELLA_DDS_LOG_THROTTLE");
  if (setting && std::string(setting) == "0") {
    EXPECT_EQ(dds_log_test_calls(), 4000);
    EXPECT_EQ(dds_log_test_priority(), 2u);
  } else {
    EXPECT_EQ(dds_log_test_calls(), 1);
    EXPECT_EQ(dds_log_test_priority(), 4u);
    EXPECT_NE(std::string(dds_log_test_message()).find("UDP send resource pressure"),
      std::string::npos);
  }
  // The independent existing transport tests still require -1/ENOBUFS,
  // never pretend a dropped packet was transmitted successfully.
}
