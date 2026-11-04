#pragma once

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <stdexcept>
#ifdef __linux__
#include <pthread.h>
#endif

namespace motion_control_pkg::motion {

// The owner serializes run() calls. Always drain ALL active jobs before returning or
// propagating an exception: jobs may reference the caller's stack and scene.
class PlanningWorkerPool {
public:
    explicit PlanningWorkerPool(std::size_t count = 2) : threads_(count), errors_(count) {
        if (count != 2 && count != 4) throw std::invalid_argument("planning pool requires 2 or 4 workers");
        try {
            for (std::size_t i = 0; i < threads_.size(); ++i)
                threads_[i] = std::thread([this, i] { worker(i); });
        } catch (...) {
            stop();
            throw;
        }
    }
    ~PlanningWorkerPool() { stop(); }
    PlanningWorkerPool(const PlanningWorkerPool&) = delete;
    PlanningWorkerPool& operator=(const PlanningWorkerPool&) = delete;

    void run(std::function<void(std::size_t)> job, std::size_t active_workers = 0) {
        if (active_workers == 0) active_workers = threads_.size();
        if (active_workers > threads_.size()) throw std::invalid_argument("too many active planning workers");
        auto task = std::make_shared<const std::function<void(std::size_t)>>(std::move(job));
        std::unique_lock<std::mutex> lock(mutex_);
        std::fill(errors_.begin(), errors_.end(), std::exception_ptr{});
        task_ = std::move(task);
        active_ = active_workers;
        remaining_ = active_;
        ++generation_;
        ready_.notify_all();
        done_.wait(lock, [this] { return remaining_ == 0; });
        task_.reset();
        for (const auto& error : errors_)
            if (error) std::rethrow_exception(error);
    }

private:
    void stop() noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_all();
        for (auto& thread : threads_)
            if (thread.joinable()) thread.join();
    }
    void worker(std::size_t index) {
#ifdef __linux__
        constexpr const char* names[]{"wbc_plan_0", "wbc_plan_1", "wbc_plan_2", "wbc_plan_3"};
        pthread_setname_np(pthread_self(), names[index]);
#endif
        std::size_t seen = 0;
        std::unique_lock<std::mutex> lock(mutex_);
        while (true) {
            ready_.wait(lock, [&] { return stopping_ || generation_ != seen; });
            if (stopping_) return;
            seen = generation_;
            if (index >= active_) continue;
            auto task = task_;
            lock.unlock();
            std::exception_ptr error;
            try { (*task)(index); } catch (...) { error = std::current_exception(); }
            lock.lock();
            errors_[index] = error;
            if (--remaining_ == 0) done_.notify_one();
        }
    }
    std::mutex mutex_;
    std::condition_variable ready_, done_;
    std::vector<std::thread> threads_;
    std::vector<std::exception_ptr> errors_;
    std::shared_ptr<const std::function<void(std::size_t)>> task_;
    std::size_t generation_{0}, remaining_{0}, active_{0};
    bool stopping_{false};
};
}  // namespace motion_control_pkg::motion
