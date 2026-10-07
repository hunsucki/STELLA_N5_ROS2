#include <cerrno>
#include <sys/socket.h>

static int recorded_flags;
static int injected_error = EAGAIN;
extern "C" int sendmsg_test_flags() {return recorded_flags;}
extern "C" void sendmsg_test_error(int value) {injected_error = value;}
extern "C" ssize_t sendmsg(int, const msghdr *, int flags)
{
  recorded_flags = flags;
  errno = injected_error;
  return -1;
}
