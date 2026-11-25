#include "ds485_driver/bus_worker.h"
#include <cassert>
#include <condition_variable>
#include <future>
#include <string>
#include <vector>
using Worker = ds485::BusWorker;

// 固定第一个读请求在途，依次排队写、两个目标和停止。只允许停止先于保留的服务出队。
void TestBusWorker()
{
    Worker bus;
    std::promise<void> active, release, drained;
    auto gate = release.get_future();
    std::vector<std::string> events;
    bool first = true;
    bus.Start([&]
    {
        if (!first)
        {
            return;
        }
        first = false;
        active.set_value();
        gate.wait();
        assert(bus.Interrupted());
        events.emplace_back("interrupted");
    }, [] { return 1000.; });
    active.get_future().wait();
    auto work = [](std::function<void()> f, std::function<void(const char *)> reject = {})
    {
        Worker::Work w;
        w.run = std::move(f);
        w.reject = std::move(reject);
        return w;
    };
    bus.Submit(Worker::Priority::Service, work([&]
    {
        events.emplace_back("service");
        drained.set_value();
    }));
    std::string busy, replaced, canceled, late;
    bus.Submit(Worker::Priority::Service, work([] { assert(false); }, [&](auto why) { busy = why; }));
    bus.Submit(Worker::Priority::Goal, work([] { assert(false); }, [&](auto why) { replaced = why; }));
    bus.Submit(Worker::Priority::Goal, work([] { assert(false); }, [&](auto why) { canceled = why; }));
    bus.Submit(Worker::Priority::Stop, work([&] { events.emplace_back("stop"); }));
    bus.Submit(Worker::Priority::Goal, work([] { assert(false); }, [&](auto why) { late = why; }));
    assert(busy == "BUSY" && replaced == "PREEMPTED" && canceled == "CANCELED" && late == "STOP_PENDING");
    release.set_value();
    assert(drained.get_future().wait_for(std::chrono::seconds(2)) == std::future_status::ready);
    bus.RequestClose([&] { events.emplace_back("closed"); });
    bus.Join();
    assert((events == std::vector<std::string>{"interrupted", "stop", "service", "closed"}));
}
