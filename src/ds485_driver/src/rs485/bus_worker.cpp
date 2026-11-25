#include "ds485_driver/bus_worker.h"
#include <algorithm>
#include <utility>

namespace ds485
{
double BusWorker::Now()
{
    return std::chrono::duration<double, std::milli>(Clock::now().time_since_epoch()).count();
}

void BusWorker::Start(std::function<void()> tick, std::function<double()> period)
{
    tick_ = std::move(tick);
    period_ = std::move(period);
    thread_ = std::thread(&BusWorker::Loop, this);
}

void BusWorker::Submit(Priority priority, Work work)
{
    work.queued_ms = Now();
    std::optional<Work> displaced, canceled;
    const char *why = "BUSY";
    bool accepted = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closing_)
        {
            why = "SHUTDOWN";
        }
        else if (priority == Priority::Stop)
        {
            ++epoch_;
            displaced = std::move(stop_);
            canceled = std::move(goal_);
            goal_.reset();
            stop_ = std::move(work);
            accepted = true;
            why = "CANCELED";
        }
        else if (priority == Priority::Goal && !stopping_ && !stop_)
        {
            ++epoch_;
            displaced = std::move(goal_);
            goal_ = std::move(work);
            accepted = true;
            why = "PREEMPTED";
        }
        else if (priority == Priority::Service && !service_ && !stop_ && !goal_ && !stopping_)
        {
            service_ = std::move(work);
            accepted = true;
        }
        else if (priority == Priority::Goal)
        {
            why = "STOP_PENDING";
        }
    }
    wake_.notify_one();
    if (!accepted && work.reject)
    {
        work.reject(why);
    }
    if (displaced && displaced->reject)
    {
        displaced->reject(why);
    }
    if (canceled && canceled->reject)
    {
        canceled->reject("CANCELED");
    }
}

bool BusWorker::Interrupted()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return !stopping_ && (closing_ || active_epoch_ != epoch_);
}

void BusWorker::RequestClose(std::function<void()> final)
{
    if (!thread_.joinable())
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closing_ = true;
        ++epoch_;
        final_ = std::move(final);
    }
    wake_.notify_one();
}
void BusWorker::Join()
{
    if (!thread_.joinable())
    {
        return;
    }
    thread_.join();
    for (auto *item : {&stop_, &goal_, &service_})
    {
        if (*item && (*item)->reject)
        {
            (*item)->reject("SHUTDOWN");
        }
        item->reset();
    }
}

void BusWorker::Loop()
{
    double due = Now();
    for (;;)
    {
        Work work;
        bool tick = false, reset_period = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            stopping_ = false;
            while (!closing_ && !stop_ && !goal_ && !service_ && Now() < due)
            {
                wake_.wait_until(lock, Clock::time_point(std::chrono::duration_cast<Clock::duration>(
                                           std::chrono::duration<double, std::milli>(due))));
            }
            active_epoch_ = epoch_;
            if (closing_)
            {
                stopping_ = true;
                break;
            }
            std::optional<Work> *slot = nullptr;
            if (stop_)
            {
                slot = &stop_;
                stopping_ = true;
            }
            else if (goal_)
            {
                slot = &goal_;
                reset_period = true;
            }
            else if (Now() >= due)
            {
                tick = true;
            }
            else if (service_)
            {
                slot = &service_;
            }
            if (slot)
            {
                work = std::move(**slot);
                slot->reset();
            }
        }
        if (tick)
        {
            queued_ms_ = due;
            const double start = Now();
            tick_();
            // 超周期只留一个下一周期；不能用积压周期追赶位移。
            const double period = period_();
            due = start + period;
            if (due <= Now())
            {
                due = Now() + period;
            }
        }
        else if (work.run)
        {
            queued_ms_ = work.queued_ms;
            work.run();
            if (reset_period)
            {
                due = Now();
            }
        }
    }
    if (final_)
    {
        final_();
    }
}
} // namespace ds485
