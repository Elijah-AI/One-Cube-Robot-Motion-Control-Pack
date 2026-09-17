#ifndef MOTION_CONTROL_PKG__EXECUTION__GRIPPER_WORKER_THREADS_HPP_
#define MOTION_CONTROL_PKG__EXECUTION__GRIPPER_WORKER_THREADS_HPP_

#include <array>
#include <cstddef>
#include <thread>
#include <utility>

namespace motion_control_pkg::execution
{

enum class GripperWorkerSlot : std::size_t
{
  kLeft = 0U,
  kRight = 1U,
  kBoth = 2U,
  kCount = 3U,
};

/// Owns one reusable worker per independent gripper execution lane.
/// Calls to start() must be serialized by the owner.
class GripperWorkerThreads
{
public:
  GripperWorkerThreads() = default;
  ~GripperWorkerThreads()
  {
    joinAll();
  }

  GripperWorkerThreads(const GripperWorkerThreads &) = delete;
  GripperWorkerThreads & operator=(const GripperWorkerThreads &) = delete;

  template<typename FunctionT, typename... ArgumentTs>
  void start(
    GripperWorkerSlot slot, FunctionT && function, ArgumentTs &&... arguments)
  {
    auto & worker = workers_[static_cast<std::size_t>(slot)];
    if (worker.joinable())
    {
      worker.join();
    }
    worker = std::thread(
      std::forward<FunctionT>(function),
      std::forward<ArgumentTs>(arguments)...);
  }

  void joinAll() noexcept
  {
    for (auto & worker : workers_)
    {
      if (worker.joinable())
      {
        worker.join();
      }
    }
  }

private:
  static constexpr std::size_t kWorkerCount =
    static_cast<std::size_t>(GripperWorkerSlot::kCount);
  std::array<std::thread, kWorkerCount> workers_;
};

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG__EXECUTION__GRIPPER_WORKER_THREADS_HPP_
