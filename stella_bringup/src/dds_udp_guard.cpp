// Keep CycloneDDS UDP transmission from blocking sensor executors on Linux.
// Loaded only into bringup children; it does not change the system DDS library.
#include <cerrno>
#include <cstring>
#include <dlfcn.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/syscall.h>

namespace
{
using SendMessage = ssize_t (*)(int, const struct msghdr *, int);
SendMessage real_sendmsg = nullptr;

// Resolve before Python imports rclpy/LTTng. Lazy dlsym inside an UST send can
// invert the loader/UST locks and prevent Python nodes from initializing.
__attribute__((constructor)) void resolve_sendmsg()
{
  real_sendmsg = reinterpret_cast<SendMessage>(dlsym(RTLD_NEXT, "sendmsg"));
}
}

extern "C" ssize_t sendmsg(int fd, const struct msghdr * msg, int flags)
{
  Dl_info caller{};
  bool guarded = false;
  int send_flags = flags;
  const int saved_errno = errno;
  // Restrict this to the CycloneDDS transport, including both reliable and
  // best-effort traffic. TCP and other libraries retain their socket semantics.
  if (msg && msg->msg_name &&
    msg->msg_namelen >= sizeof(sa_family_t))
  {
    const auto family = static_cast<const sockaddr *>(msg->msg_name)->sa_family;
    int type = 0;
    socklen_t size = sizeof(type);
    // UST uses UNIX sockets. Never call dladdr under its internal locks.
    guarded = (family == AF_INET || family == AF_INET6) &&
      getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &size) == 0 && type == SOCK_DGRAM &&
      dladdr(__builtin_return_address(0), &caller) && caller.dli_fname &&
      std::strstr(caller.dli_fname, "libddsc.so");
    if (guarded) {
      // A per-datagram timeout still adds up across many remote destinations.
      // Sensor executors must return immediately when the UDP queue is full.
      send_flags |= MSG_DONTWAIT;
    }
  }
  errno = saved_errno;
  const ssize_t result = real_sendmsg
    ? real_sendmsg(fd, msg, send_flags)
    : syscall(SYS_sendmsg, fd, msg, send_flags);
  if (guarded && result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    // CycloneDDS 0.10's Linux transport retries EAGAIN indefinitely without a
    // sleep. Report queue exhaustion as ENOBUFS so that it drops this datagram
    // and returns. Reliable DDS can retransmit later; local processing proceeds.
    errno = ENOBUFS;
  }
  return result;
}
