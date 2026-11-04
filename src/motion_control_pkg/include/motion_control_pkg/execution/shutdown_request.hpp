#pragma once
#include <atomic>
#include <csignal>
#include <stdexcept>

namespace motion_control_pkg::execution {
// Only a lock-free store runs in the signal handler. No ROS shutdown, SDK call,
// allocation or logging is permitted there. The latch is never reset in-process.
inline std::atomic<bool> shutdown_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free, "signal latch must be lock-free");
inline bool shutdownRequested() noexcept { return shutdown_requested.load(); }
inline void requestShutdown() noexcept { shutdown_requested.store(true); }
inline void shutdownSignalHandler(int) noexcept { requestShutdown(); }
inline void installShutdownSignalHandlers() {
  struct sigaction action{};
  action.sa_handler = shutdownSignalHandler;
  sigemptyset(&action.sa_mask);
  if (sigaction(SIGINT, &action, nullptr) || sigaction(SIGTERM, &action, nullptr))
    throw std::runtime_error("cannot install orderly shutdown signal handlers");
}
}  // namespace motion_control_pkg::execution
