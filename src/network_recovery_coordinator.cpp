// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "provizio/dds/detail/network_recovery_coordinator.h"

#include <algorithm>
#include <chrono>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/logging.h"

namespace provizio::dds::detail
{
    network_recovery_coordinator &network_recovery_coordinator::instance()
    {
        // Meyers' singleton — lazy, thread-safe (C++11+), destroyed at process exit.
        static network_recovery_coordinator the_instance;
        return the_instance;
    }

    network_recovery_coordinator::network_recovery_coordinator() = default;

    network_recovery_coordinator::~network_recovery_coordinator()
    {
        {
            const std::lock_guard<std::mutex> lock{coalescer_mutex};
            stop_requested = true;
        }
        coalescer_cv.notify_all();
        if (coalescer_thread.joinable())
        {
            coalescer_thread.join();
        }
        // network_monitor's destructor closes the kernel channel and joins its
        // worker; this is a no-op if the monitor was never lazily constructed.
        monitor.reset();
    }

    void network_recovery_coordinator::register_participant(const std::shared_ptr<domain_participant> &participant)
    {
        if (!participant)
        {
            return;
        }

        // The user-installed log callback can call back into provizio_dds
        // (e.g. construct another domain_participant), so log_info /
        // log_error must NOT run while we hold registry_mutex — otherwise a
        // callback that ends up here transitively would deadlock on the
        // recursive acquire. We capture the outcome inside the locked
        // section and emit the corresponding log line after we release.
        bool initialised_monitor_now = false;
        std::size_t initial_snapshot_size = 0;
        std::string init_error;

        {
            const std::lock_guard<std::mutex> lock{registry_mutex};

            // Garbage-collect expired entries inline; the registry is small and walked
            // on every reset, so amortised this is cheaper than a periodic sweep.
            registered_participants.erase(
                std::remove_if(registered_participants.begin(), registered_participants.end(),
                               [](const std::weak_ptr<domain_participant> &weak) { return weak.expired(); }),
                registered_participants.end());

            // Idempotent: already registered?
            for (const auto &existing : registered_participants)
            {
                if (existing.lock() == participant)
                {
                    return;
                }
            }
            registered_participants.emplace_back(participant);

            // Lazily start the monitor on first real registration. Open the monitor
            // BEFORE capturing the snapshot: any kernel event that fires between
            // these two operations will be queued on the (open) kernel channel and
            // delivered to the coalescer, which will then capture a fresh snapshot
            // and diff against last_known_snapshot. If we captured the snapshot
            // first and the event fired in between, the snapshot would already
            // reflect the post-event state — the change would silently be lost
            // unless another future event fires.
            if (!monitor)
            {
                try
                {
                    monitor = std::make_unique<network_monitor>([this] { on_kernel_event(); });
                    last_known_snapshot = capture_address_snapshot();
                    coalescer_thread = std::thread{[this] { coalescer_loop(); }};
                    initialised_monitor_now = true;
                    initial_snapshot_size = last_known_snapshot.size();
                }
                catch (const std::exception &exception)
                {
                    // Roll back any half-initialised state so a future
                    // register_participant call (e.g. from another participant on
                    // a fresh attempt) can try again. Tear-down order matters:
                    // stop and join the coalescer FIRST — otherwise leaving it
                    // joinable would cause the next assignment to std::thread
                    // (on a retry) to std::terminate, and resetting `monitor`
                    // while the coalescer might still be reading it could race.
                    if (coalescer_thread.joinable())
                    {
                        {
                            const std::lock_guard<std::mutex> coalescer_lock{coalescer_mutex};
                            stop_requested = true;
                        }
                        coalescer_cv.notify_all();
                        coalescer_thread.join();
                        // Reset the coalescer's request flags so a future
                        // re-init starts from a clean state.
                        stop_requested = false;
                        has_pending_burst = false;
                        reset_in_progress = false;
                    }
                    monitor.reset();
                    // Drop the registration appended above: the monitor failed
                    // to come up, so this participant must not linger in the
                    // registry advertising recovery it would never actually
                    // receive. It is the last entry (we hold registry_mutex
                    // throughout, so nothing else touched the vector since the
                    // emplace_back). A later register_participant retries init.
                    registered_participants.pop_back();
                    init_error = exception.what();
                }
            }
        }

        // Emit the outcome log AFTER releasing registry_mutex. A user log
        // callback that re-enters register_participant via
        // make_domain_participant would otherwise deadlock on the recursive
        // acquire above.
        if (initialised_monitor_now)
        {
            log_info() << "network auto-recovery: enabled (initial snapshot: " << initial_snapshot_size
                       << " interface address(es))";
        }
        else if (!init_error.empty())
        {
            log_error() << "network auto-recovery: monitor failed to start (" << init_error
                        << "); auto-recovery unavailable until the next recovery-enabled "
                           "participant creation retries the initialization";
        }
    }

    void network_recovery_coordinator::wait_for_idle()
    {
        // Wait for both: the coalescer has consumed any pending burst AND
        // there's no reset currently running. Either alone is insufficient —
        // a test that observes !has_pending_burst could still race with the
        // tail of an in-progress reset.
        std::unique_lock<std::mutex> lock{coalescer_mutex};
        coalescer_cv.wait(lock, [this] { return (!has_pending_burst && !reset_in_progress) || stop_requested; });
    }

    void network_recovery_coordinator::inject_kernel_event_for_test()
    {
        // Out-of-line so the call into the private on_kernel_event resolves
        // in this TU rather than crossing the DLL boundary from a test
        // executable. See the header for the rationale.
        on_kernel_event();
    }

    void network_recovery_coordinator::on_kernel_event()
    {
        const auto now = std::chrono::steady_clock::now();
        {
            const std::lock_guard<std::mutex> lock{coalescer_mutex};
            if (!has_pending_burst)
            {
                first_event_time = now;
                has_pending_burst = true;
            }
            last_event_time = now;
        }
        coalescer_cv.notify_all();
    }

    void network_recovery_coordinator::coalescer_loop()
    {
        std::unique_lock<std::mutex> lock{coalescer_mutex};
        while (!stop_requested)
        {
            if (!has_pending_burst)
            {
                coalescer_cv.wait(lock, [this] { return has_pending_burst || stop_requested; });
                continue;
            }

            // Wait until the burst is quiet for quiet_period OR has been going on for
            // max_debounce since the first event.
            const auto now = std::chrono::steady_clock::now();
            const auto quiet_until = last_event_time + quiet_period;
            const auto max_wait_until = first_event_time + max_debounce;
            const auto wake_at = (quiet_until < max_wait_until) ? quiet_until : max_wait_until;

            if (now < wake_at)
            {
                coalescer_cv.wait_until(lock, wake_at);
                continue;
            }

            // Re-check: if any new event arrived during our wait, recompute.
            if (last_event_time + quiet_period > std::chrono::steady_clock::now() &&
                first_event_time + max_debounce > std::chrono::steady_clock::now())
            {
                continue;
            }

            // Mark that we're starting a reset BEFORE clearing has_pending_burst,
            // so wait_for_idle observes the "still busy" state through the whole
            // duration. The CV is notified after the reset finishes (under the
            // lock) to wake anyone waiting.
            has_pending_burst = false;
            reset_in_progress = true;
            lock.unlock();
            run_reset();
            lock.lock();
            reset_in_progress = false;
            coalescer_cv.notify_all();
        }
    }

    void network_recovery_coordinator::run_reset()
    {
        // last_known_snapshot is only mutated here (inside the coalescer
        // thread) and once during the lazy monitor-start in
        // register_participant — and that single initial write happens
        // BEFORE the coalescer thread is constructed, so no actual race
        // exists. Documented to forestall future readers wondering
        // whether the access pattern needs additional synchronisation.
        const auto new_snapshot = capture_address_snapshot();

        if (new_snapshot == last_known_snapshot)
        {
            log_info() << "network event burst — snapshot unchanged (" << new_snapshot.size()
                       << " interface address(es)), no reset";
            skipped_reset_count.fetch_add(1, std::memory_order_acq_rel);
            return;
        }

        // Quick diff summary for the log: counts of additions and removals
        // give a more useful one-liner than "old.size() → new.size()" which
        // could be the same even when contents fully differ.
        std::size_t added = 0;
        for (const auto &address : new_snapshot)
        {
            if (last_known_snapshot.find(address) == last_known_snapshot.end())
            {
                ++added;
            }
        }
        std::size_t removed = 0;
        for (const auto &address : last_known_snapshot)
        {
            if (new_snapshot.find(address) == new_snapshot.end())
            {
                ++removed;
            }
        }

        log_info() << "network change detected: +" << added << " / -" << removed << " interface address(es) ("
                   << last_known_snapshot.size() << " → " << new_snapshot.size()
                   << "); resetting recovery-enabled participants";

        last_known_snapshot = new_snapshot;

        // Snapshot the registry under the lock, then operate on the snapshot. This
        // releases the registry mutex during the (potentially long) per-participant
        // reset, so a participant being destroyed concurrently isn't blocked.
        std::vector<std::shared_ptr<domain_participant>> live_participants;
        {
            const std::lock_guard<std::mutex> lock{registry_mutex};
            live_participants.reserve(registered_participants.size());
            for (const auto &weak : registered_participants)
            {
                if (auto strong = weak.lock())
                {
                    live_participants.push_back(std::move(strong));
                }
            }
        }

        for (const auto &participant : live_participants)
        {
            try
            {
                participant->trigger_network_recovery_reset();
            }
            catch (const std::exception &exception)
            {
                log_error() << "participant reset failed: " << exception.what();
            }
        }

        log_info() << "reset complete (" << live_participants.size() << " participant(s))";
        reset_count.fetch_add(1, std::memory_order_acq_rel);
    }
}  // namespace provizio::dds::detail
