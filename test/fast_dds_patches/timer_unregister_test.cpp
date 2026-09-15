// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy of
// the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.
//
// Guards cmake/fast_dds/resource_event_per_timer_wait.cmake, which replaces Fast-DDS'
// ResourceEvent::unregister_timer() so it waits for THIS timer's callback instead of for the
// execution thread's whole pass (eProsima/Fast-DDS#6502). That wait releases the mutex, and a
// callback is free to restart its own timer while it runs -- ResourceEvent::notify() then puts
// the pointer back into pending_timers_ after unregister had already removed it. The caller
// deletes the timer as soon as unregister_timer returns (TimedEvent::~TimedEvent), so the
// execution thread's next pass sorts a container holding freed memory, reading it through
// event_compare() and TimedEventImpl::next_trigger_time().
//
// This exercises exactly that ordering and then forces the pass that would touch the corpse.
// Under the sanitizers the Debug builds enable it fails as a heap-use-after-free; without them
// it is a silent memory error, which is why the case exists rather than relying on the
// behavioural suites to notice.
//
// Built only where Fast-DDS is built from source, since that is the only configuration the
// patch is applied in -- see the CMakeLists. It reaches into Fast-DDS' PRIVATE headers
// deliberately: the patch reaches into those same sources, and a FAST_DDS_VERSION bump that
// moves them has to re-check this patch anyway (see AGENTS.md).

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include "rtps/resources/ResourceEvent.h"
#include "rtps/resources/TimedEventImpl.h"

namespace
{
    constexpr auto k_wait = std::chrono::seconds{10};
    // Far enough out that the rearmed timer never fires on its own: the point is that it is
    // QUEUED when it should not be, not that it runs.
    constexpr unsigned int k_far_future_ms = 3600000;
}  // namespace

int main()
{
    using eprosima::fastdds::rtps::ResourceEvent;
    using eprosima::fastdds::rtps::TimedEventImpl;

    ResourceEvent service;
    service.init_thread();

    std::promise<void> callback_entered;
    std::promise<void> release_callback;
    auto released = release_callback.get_future();

    TimedEventImpl *rearming = nullptr;
    rearming = new TimedEventImpl(
        [&]() {
            callback_entered.set_value();
            released.wait();
            // What a real callback is entitled to do, and what WriterProxy does: rearm itself.
            // It lands in pending_timers_ through notify(), behind an unregister_timer() that
            // has already taken it out.
            rearming->update_interval_millisec(k_far_future_ms);
            rearming->go_ready();
            service.notify(rearming);
            return false;
        },
        std::chrono::microseconds{1000});

    service.register_timer(rearming);
    rearming->go_ready();
    service.notify(rearming);

    if (callback_entered.get_future().wait_for(k_wait) != std::future_status::ready)
    {
        std::cout << "timer_unregister: FAIL (the callback never ran)" << '\n';
        return 1;
    }

    // Unregister while the callback is still inside itself, so unregister_timer reaches its
    // wait and releases the mutex the rearm needs.
    auto unregistering = std::async(std::launch::async, [&]() { service.unregister_timer(rearming); });
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    release_callback.set_value();

    if (unregistering.wait_for(k_wait) != std::future_status::ready)
    {
        std::cout << "timer_unregister: FAIL (unregister_timer did not return -- deadlock)" << '\n';
        std::abort();  // A hung service thread would otherwise hang the whole test binary.
    }
    unregistering.get();
    delete rearming;

    // The pass that would read the corpse. A second timer, registered and fired after the
    // deletion, makes the execution thread walk and SORT its collections -- which is where a
    // pointer left behind by the rearm is dereferenced. Waiting for it to fire also proves the
    // service is still functional, not merely quiet.
    std::mutex mutex;
    std::condition_variable fired_cv;
    bool fired = false;
    auto *survivor = new TimedEventImpl(
        [&]() {
            const std::lock_guard<std::mutex> lock{mutex};
            fired = true;
            fired_cv.notify_all();
            return false;
        },
        std::chrono::microseconds{1000});
    service.register_timer(survivor);
    survivor->go_ready();
    service.notify(survivor);

    bool survivor_fired = false;
    {
        std::unique_lock<std::mutex> lock{mutex};
        survivor_fired = fired_cv.wait_for(lock, k_wait, [&] { return fired; });
    }
    service.unregister_timer(survivor);
    delete survivor;

    if (!survivor_fired)
    {
        std::cout << "timer_unregister: FAIL (the event service stopped serving timers)" << '\n';
        return 1;
    }

    std::cout << "timer_unregister: PASS (a rearm during unregister left nothing queued)" << '\n';
    return 0;
}
