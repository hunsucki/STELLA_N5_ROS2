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
