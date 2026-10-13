#include <sys/socket.h>

extern "C" ssize_t dds_test_send(int fd, const msghdr * msg, int flags)
{
  return sendmsg(fd, msg, flags);
}

#include <cstdint>
struct ddsrt_log_cfg;
extern "C" void dds_log_cfg(const ddsrt_log_cfg *, uint32_t, const char *, uint32_t,
  const char *, const char *, ...);
extern "C" void dds_test_log(const char * function, const char * destination, int code)
{
  dds_log_cfg(nullptr, 2u, "ddsi_udp.c", 201, function,
    "ddsi_udp_conn_write to %s failed with retcode %d\n", destination, code);
}
extern "C" void dds_test_long_log(const char * text)
{
  dds_log_cfg(nullptr, 2u, "other.c", 1, "unrelated", "%s", text);
}
