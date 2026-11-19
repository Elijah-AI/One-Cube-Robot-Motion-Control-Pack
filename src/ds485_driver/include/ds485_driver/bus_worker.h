#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace ds485
{
// 每条总线只允许这里的线程调用驱动。ROS 回调提交有限信箱后立即返回。
class BusWorker
{
  public:
    using Clock = std::chrono::steady_clock;
    enum class Priority
    {
        Stop,
        Goal,
        Service
    };
    struct Work
    {
        std::function<void()> run;
        std::function<void(const char *)> reject;
        double queued_ms = 0;
    };
    void Start(std::function<void()> tick, std::function<double()> period);
    void Submit(Priority priority, Work work);
    void RequestClose(std::function<void()> final);
    void Join();
    bool Interrupted();
    double QueuedMs() const
    {
        return queued_ms_;
    }
    static double Now();

  private:
    void Loop();
    std::mutex mutex_;
    std::condition_variable wake_;
    // 提交目标/停止或关闭时递增；工作线程取出请求时保存代号，收发等待中比较代号即可抢占。
    unsigned long epoch_ = 0, active_epoch_ = 0;
    bool closing_ = false, stopping_ = false;
    std::optional<Work> stop_, goal_, service_;
    std::function<void()> tick_, final_;
    std::function<double()> period_;
    std::thread thread_;
    double queued_ms_ = 0; // 仅总线线程维护，供本项工作内的所有事务关联入队时刻。
};
} // namespace ds485
