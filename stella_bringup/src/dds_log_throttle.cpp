// Limit only CycloneDDS 0.10 UDP resource-pressure log floods. Transport
// return codes, retransmission and the nonblocking send guard stay unchanged.
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dlfcn.h>

// Public signature from dds/ddsrt/log.h; keep DDS optional for non-DDS children.
struct ddsrt_log_cfg;
using LogConfig = void (*)(const ddsrt_log_cfg *, uint32_t, const char *,
  uint32_t, const char *, const char *, ...);

extern "C" void dds_log_cfg(const ddsrt_log_cfg * cfg, uint32_t priority,
  const char * file, uint32_t line, const char * function, const char * format, ...)
{
  // This is a DDS logging entry, never an intercepted UST socket operation.
  // Python loads DDS after the preload's constructor, so resolve here once.
  static const auto original = reinterpret_cast<LogConfig>(dlsym(RTLD_NEXT, "dds_log_cfg"));
  if (!original) {return;}
  static const bool enabled = []() {
      const char * setting = std::getenv("STELLA_DDS_LOG_THROTTLE");
      return !setting || std::strcmp(setting, "0") != 0;
    }();
  va_list arguments;
  va_start(arguments, format);
  if (enabled && function && format &&
    std::strcmp(function, "ddsi_udp_conn_write") == 0 &&
    std::strcmp(format, "ddsi_udp_conn_write to %s failed with retcode %d\n") == 0)
  {
    va_list copy;
    va_copy(copy, arguments);
    const char * destination = va_arg(copy, const char *);
    const int code = va_arg(copy, int);
    va_end(copy);
    if (code == -5) {
      static std::atomic<unsigned long> total{0};
      static std::atomic<long long> next_report{0};
      const auto count = total.fetch_add(1, std::memory_order_relaxed)+1;
      timespec now{};
      clock_gettime(CLOCK_MONOTONIC, &now);
      const long long seconds = now.tv_sec;
      auto next = next_report.load(std::memory_order_relaxed);
      if (seconds >= next && next_report.compare_exchange_strong(next, seconds+30)) {
        original(cfg, 4u /* DDS_LC_WARNING */, file, line, function,
          "WARN STELLA DDS: UDP send resource pressure; failures=%lu, last=%s; "
          "transport still reports failure, repeated logs limited to once/30s per process\n",
          count, destination);
      }
      va_end(arguments);
      return;
    }
  }
  // Preserve all other diagnostics, metadata and severity, including other UDP
  // errors and arbitrarily long messages. Do not disable DDS logging globally.
  char * message = nullptr;
  const int size = vasprintf(&message, format, arguments);
  va_end(arguments);
  if (size >= 0) {
    // DDS flushes by the format's trailing newline, not the rendered text.
    // Preserve fragment assembly and immediate emission of complete errors.
    const auto format_size = std::strlen(format);
    if (format_size && format[format_size-1] == '\n' && size > 0) {
      message[size-1] = '\0';
      original(cfg, priority, file, line, function, "%s\n", message);
    } else {
      original(cfg, priority, file, line, function, "%s", message);
    }
    std::free(message);
  } else {
    original(cfg, priority, file, line, function,
      "%s [DDS log formatting failed]\n", format);
  }
}
