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
#include <memory>
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
    // How often the timers are armed to fire: soon enough that a test waiting for one never waits long.
    constexpr auto k_timer_interval = std::chrono::microseconds{1000};
    // How long the unregistering thread is given to reach its wait inside unregister_timer() before the
    // callback is released.
    constexpr auto k_unregister_head_start = std::chrono::milliseconds{100};
}  // namespace

int main()
{
    using eprosima::fastdds::rtps::ResourceEvent;
    using eprosima::fastdds::rtps::TimedEventImpl;

    // Everything the callback below touches is declared ahead of the service, so that wherever the function
    // returns the service's thread is stopped before any of it is freed: a late callback must never reach a
    // destroyed promise, future or timer. (The failure paths that cannot return safely terminate instead.)
    std::promise<void> callback_entered;
    std::promise<void> release_callback;
    auto released = release_callback.get_future();
    std::unique_ptr<TimedEventImpl> rearming;

    ResourceEvent service;
    service.init_thread();

    rearming = std::make_unique<TimedEventImpl>(
        [&]() {
            callback_entered.set_value();
            released.wait();
            // What a real callback is entitled to do, and what WriterProxy does: rearm itself.
            // It lands in pending_timers_ through notify(), behind an unregister_timer() that
            // has already taken it out.
            rearming->update_interval_millisec(k_far_future_ms);
            rearming->go_ready();
            service.notify(rearming.get());
            return false;
        },
        k_timer_interval);

    service.register_timer(rearming.get());
    rearming->go_ready();
    service.notify(rearming.get());

    if (callback_entered.get_future().wait_for(k_wait) != std::future_status::ready)
    {
        std::cout << "timer_unregister: FAIL (the callback never ran)" << '\n';
        // Not a return: the worker may be entering the callback right now, where it blocks until release_callback
        // is set -- which nothing will do on this path -- and destroying the service would then wait for it
        // forever. Terminating, like the deadlock case below, ends the binary whatever the worker is doing.
        std::abort();
    }

    // Unregister while the callback is still inside itself, so unregister_timer reaches its
    // wait and releases the mutex the rearm needs.
    auto unregistering = std::async(std::launch::async, [&]() { service.unregister_timer(rearming.get()); });
    std::this_thread::sleep_for(k_unregister_head_start);
    release_callback.set_value();

    if (unregistering.wait_for(k_wait) != std::future_status::ready)
    {
        std::cout << "timer_unregister: FAIL (unregister_timer did not return -- deadlock)" << '\n';
        std::abort();  // A hung service thread would otherwise hang the whole test binary.
    }
    unregistering.get();
    rearming.reset();

    // The pass that would read the corpse. A second timer, registered and fired after the
    // deletion, makes the execution thread walk and SORT its collections -- which is where a
    // pointer left behind by the rearm is dereferenced. Waiting for it to fire also proves the
    // service is still functional, not merely quiet.
    std::mutex mutex;
    std::condition_variable fired_cv;
    bool fired = false;
    auto survivor = std::make_unique<TimedEventImpl>(
        [&]() {
            const std::lock_guard<std::mutex> lock{mutex};
            fired = true;
            fired_cv.notify_all();
            return false;
        },
        k_timer_interval);
    service.register_timer(survivor.get());
    survivor->go_ready();
    service.notify(survivor.get());

    bool survivor_fired = false;
    {
        std::unique_lock<std::mutex> lock{mutex};
        survivor_fired = fired_cv.wait_for(lock, k_wait, [&] { return fired; });
    }
    service.unregister_timer(survivor.get());
    survivor.reset();

    if (!survivor_fired)
    {
        std::cout << "timer_unregister: FAIL (the event service stopped serving timers)" << '\n';
        return 1;
    }

    std::cout << "timer_unregister: PASS (a rearm during unregister left nothing queued)" << '\n';
    return 0;
}
