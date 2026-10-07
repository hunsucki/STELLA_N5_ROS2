#include <sys/socket.h>

extern "C" ssize_t dds_test_send(int fd, const msghdr * msg, int flags)
{
  return sendmsg(fd, msg, flags);
}
