#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace motion_control_pkg::execution {

// One bounded latest-value slot. update() never allocates, waits, logs or calls
// ROS/DDS. The independent periodic worker keeps publication off both the motor
// sampling thread and the node's measured-feedback executor callbacks.
class LatestProgressPublisher {
public:
  using Publish = std::function<void(float)>;
  explicit LatestProgressPublisher(Publish publish,
      std::chrono::milliseconds period=std::chrono::milliseconds(100))
      : publish_(std::move(publish)), period_(period) {
    static_assert(std::atomic<float>::is_always_lock_free,
        "motor-loop progress updates require a lock-free float");
    if (!publish_ || period_.count()<=0) throw std::invalid_argument("invalid progress publisher");
    try {worker_=std::thread([this]{run();});}
    catch (...) {error_=std::current_exception();failed_.store(true,std::memory_order_release);}
  }
  ~LatestProgressPublisher() {stop();}
  LatestProgressPublisher(const LatestProgressPublisher&)=delete;
  LatestProgressPublisher& operator=(const LatestProgressPublisher&)=delete;

  void update(float progress) noexcept {
    if (stopped_.load(std::memory_order_relaxed)) return;
    latest_.store(progress,std::memory_order_relaxed);
    dirty_.store(true,std::memory_order_release);
  }
  bool failed() const noexcept {return failed_.load(std::memory_order_acquire);}
  std::string error() const {
    if (!failed()) return {};
    try {std::rethrow_exception(error_);}
    catch (const std::exception& e) {return std::string("asynchronous Action feedback failed: ")+e.what();}
    catch (...) {return "asynchronous Action feedback failed: unknown publication exception";}
  }
  // Call outside the sample loop, before publishing an Action terminal state or
  // reusing its feedback message. Joins any in-flight publication and discards
  // queued progress; the Action result remains the authoritative final outcome.
  void stop() noexcept {
    {
      std::lock_guard<std::mutex> lock(wait_mutex_);
      stopped_.store(true,std::memory_order_release);
    }
    wake_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

private:
  void run() noexcept {
    using Clock=std::chrono::steady_clock;
    auto next=Clock::now()+period_;
    std::unique_lock<std::mutex> lock(wait_mutex_);
    while (!wake_.wait_until(lock,next,[this]{return stopped_.load(std::memory_order_acquire);})) {
      lock.unlock();
      if (dirty_.exchange(false,std::memory_order_acq_rel)) {
        const float value=latest_.load(std::memory_order_acquire);
        try {publish_(value);}
        catch (...) {
          error_=std::current_exception();failed_.store(true,std::memory_order_release);return;
        }
      }
      lock.lock();
      // A slow consumer never drains a backlog or causes a catch-up burst.
      next=Clock::now()+period_;
    }
  }
  Publish publish_;
  const std::chrono::milliseconds period_;
  std::atomic<float> latest_{0};
  std::atomic<bool> dirty_{false},stopped_{false},failed_{false};
  std::exception_ptr error_;
  std::mutex wait_mutex_;
  std::condition_variable wake_;
  std::thread worker_;
};
}  // namespace motion_control_pkg::execution
