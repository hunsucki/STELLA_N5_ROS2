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

#include <cstdarg>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
static std::mutex log_mutex;
static int log_calls;
static uint32_t log_priority;
static std::string log_message;
struct ddsrt_log_cfg;
extern "C" void dds_log_cfg(const ddsrt_log_cfg *, uint32_t priority, const char *,
  uint32_t, const char *, const char * format, ...)
{
  va_list arguments;
  va_start(arguments, format);
  char * message = nullptr;
  vasprintf(&message, format, arguments);
  va_end(arguments);
  std::lock_guard<std::mutex> lock(log_mutex);
  ++log_calls;
  log_priority = priority;
  log_message = message;
  free(message);
}
extern "C" void dds_log_test_reset() {log_calls = 0; log_message.clear();}
extern "C" int dds_log_test_calls() {return log_calls;}
extern "C" uint32_t dds_log_test_priority() {return log_priority;}
extern "C" const char * dds_log_test_message() {return log_message.c_str();}
