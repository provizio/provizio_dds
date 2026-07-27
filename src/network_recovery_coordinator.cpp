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
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <string>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/logging.h"

namespace provizio::dds::detail
{
    namespace
    {
        // Consumed (and cleared) by the next create_fastdds_participant call — see
        // fail_next_participant_creation_for_test. Wrapped in an accessor so the flag is
        // a function-local static rather than a mutable global (which
        // cppcoreguidelines-avoid-non-const-global-variables rightly rejects).
        std::atomic<bool> &forced_participant_creation_failure()
        {
            static std::atomic<bool> flag{false};
            return flag;
        }

        constexpr const char *safety_net_env_var_name = "PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC";

        // Upper bound on the safety-net period. Beyond roughly 9.2e9 seconds, converting
        // the duration to the nanoseconds that condition_variable::wait_for works in
        // overflows int64: libstdc++ then returns from wait_for IMMEDIATELY, every time,
        // which turns the coalescer's "wait for the period, then tick" into an unbounded
        // busy loop doing a getifaddrs plus a full RTM_GETLINK dump per iteration. A day
        // is far past any sane cadence, so clamping there keeps the arithmetic safe and
        // the failure mode impossible. Callers who want "never" have the documented 0.
        constexpr std::chrono::seconds max_safety_net_period{24 * 60 * 60};

        // Env values are echoed back in warnings; cap what we quote so a pathological
        // value cannot flood the log, and drop control characters so it cannot forge log
        // lines in whatever ingests them.
        std::string sanitise_env_value_for_log(const std::string &raw)
        {
            // ASCII C0 controls are everything below the first printable character
            // (space); DEL sits just past the printable range.
            constexpr unsigned char first_printable_ascii = 0x20;
            constexpr unsigned char ascii_delete = 0x7F;
            constexpr std::size_t max_quoted_length = 32;

            std::string result = raw.substr(0, std::min(raw.size(), max_quoted_length));
            for (auto &chr : result)
            {
                const auto value = static_cast<unsigned char>(chr);
                if (value < first_printable_ascii || value == ascii_delete)
                {
                    chr = '?';
                }
            }
            if (raw.size() > max_quoted_length)
            {
                result += "...";
            }
            return result;
        }

        // Resolves the safety-net period from the environment. Any warning is reported
        // through @p warning rather than logged here: the only caller holds
        // registry_mutex, and a user log callback that re-enters make_domain_participant
        // would deadlock on it (see register_participant).
        std::chrono::seconds resolve_safety_net_period(std::string &warning)
        {
            const auto default_period = network_recovery_coordinator::default_safety_net_period;

            // NOLINTNEXTLINE(concurrency-mt-unsafe): startup-only probe, as elsewhere.
            const auto *raw = std::getenv(safety_net_env_var_name);
            if (raw == nullptr || *raw == '\0')
            {
                return default_period;
            }

            const std::string value{raw};
            const auto quoted = sanitise_env_value_for_log(value);
            const auto reject = [&](const char *reason) {
                warning = std::string{safety_net_env_var_name} + "=" + quoted + " " + reason +
                          "; using the default of " + std::to_string(default_period.count()) + "s";
                return default_period;
            };

            std::size_t consumed = 0;
            std::int64_t parsed = 0;
            try
            {
                parsed = std::stoll(value, &consumed);
            }
            catch (const std::exception &)
            {
                return reject("is not an integer number of seconds");
            }
            // stoll stops at the first non-digit and reports success, so "30s" or "0x10"
            // would otherwise be silently accepted as 30 and 0 — and 0 DISABLES the
            // safety net, the opposite of what someone writing "0x10" intends.
            if (consumed != value.size())
            {
                return reject("has trailing characters after the number of seconds");
            }
            if (parsed < 0)
            {
                return reject("is negative");
            }
            if (std::chrono::seconds{parsed} > max_safety_net_period)
            {
                warning = std::string{safety_net_env_var_name} + "=" + quoted + " exceeds the maximum of " +
                          std::to_string(max_safety_net_period.count()) + "s; clamping to it (use 0 to disable the " +
                          "periodic check entirely)";
                return max_safety_net_period;
            }
            return std::chrono::seconds{parsed};
        }
    }  // namespace

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
        std::string env_warning;

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
            const std::lock_guard<std::mutex> monitor_lock{monitor_mutex};
            if (!monitor)
            {
                try
                {
                    safety_net_period = resolve_safety_net_period(env_warning);
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
        if (!env_warning.empty())
        {
            log_warning() << env_warning;
        }
        if (initialised_monitor_now)
        {
            auto log = log_info();
            log << "network auto-recovery: enabled (initial snapshot: " << initial_snapshot_size
                << " interface address(es)";
            if (safety_net_period.count() > 0)
            {
                log << ", safety-net check every " << safety_net_period.count() << "s";
            }
            else
            {
                log << ", periodic safety-net check disabled";
            }
            // Reported from here, not from force_included_interfaces() itself: that runs
            // inside capture_address_snapshot(), which the block above calls while
            // holding registry_mutex and monitor_mutex, and logging under those invites a
            // re-entrant user callback to deadlock (see address_snapshot_env.cpp).
            const auto &force_included = force_included_interfaces();
            if (!force_included.empty())
            {
                log << ", force-including " << force_included.size() << " interface(s):";
                for (const auto &name : force_included)
                {
                    log << " " << name;
                }
            }
            log << ")";
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

    void network_recovery_coordinator::inject_transient_for_test(const address_snapshot &simulated_burst_start)
    {
        // Stage a synthetic burst and let the COALESCER THREAD run the reset, rather
        // than calling run_reset() inline here. run_reset() reads and writes
        // last_known_snapshot lock-free under the invariant that only the coalescer
        // thread ever mutates it (see run_reset); driving it from the test thread
        // while the coalescer is live would race that single-writer access. So we
        // seed the burst-start snapshot (= simulated_burst_start) and mark a pending
        // burst — the end snapshot is whatever the coalescer captures live, exactly
        // as in the real path. The event timers are backdated past max_debounce so
        // the coalescer fires on its next wake without waiting out the window. The
        // caller wait_for_idle()s for completion. Out-of-line / exported for the
        // same reason as inject_kernel_event_for_test.
        {
            const std::lock_guard<std::mutex> lock{coalescer_mutex};
            const auto now = std::chrono::steady_clock::now();
            first_event_time = now - max_debounce;
            last_event_time = now - max_debounce;
            has_pending_burst = true;
            burst_start_snapshot = simulated_burst_start;
            burst_start_valid = true;
        }
        coalescer_cv.notify_all();
    }

    void fail_next_participant_creation_for_test() noexcept
    {
        forced_participant_creation_failure().store(true, std::memory_order_release);
    }

    bool consume_forced_participant_creation_failure() noexcept
    {
        return forced_participant_creation_failure().exchange(false, std::memory_order_acq_rel);
    }

    void network_recovery_coordinator::run_safety_net_tick_for_test()
    {
        // Out-of-line / exported for the same reason as the other test hooks: the body
        // calls private members the class does not export wholesale.
        safety_net_tick();
    }

    void network_recovery_coordinator::seed_last_known_snapshot_for_test(const address_snapshot &snapshot)
    {
        // No lock: last_known_snapshot is single-writer state owned by the coalescer
        // thread, and this hook is documented as idle-only (the caller wait_for_idle()s
        // first), so there is no concurrent access to guard against.
        last_known_snapshot = snapshot;
    }

    bool network_recovery_coordinator::kill_monitor_for_test()
    {
        const std::lock_guard<std::mutex> lock{monitor_mutex};
        return monitor && monitor->kill_for_test();
    }

    bool network_recovery_coordinator::monitor_alive_for_test()
    {
        const std::lock_guard<std::mutex> lock{monitor_mutex};
        return monitor && monitor->is_alive();
    }

    void network_recovery_coordinator::on_kernel_event()
    {
        const auto now = std::chrono::steady_clock::now();
        bool first_of_burst = false;
        {
            const std::lock_guard<std::mutex> lock{coalescer_mutex};
            if (!has_pending_burst)
            {
                first_event_time = now;
                has_pending_burst = true;
                first_of_burst = true;
            }
            last_event_time = now;
        }

        // On the FIRST event of a burst, snapshot the interfaces immediately, before
        // a quick flap can revert. The coalescer captures the END snapshot ~quiet_period
        // later; if an address left and returned within that window the two ends match
        // and the end-only diff would skip the reset — yet the Fast-DDS sockets bound to
        // that address were torn down while it was gone. This start snapshot is what lets
        // run_reset() catch that transient. capture_address_snapshot() is too slow to hold
        // coalescer_mutex across, so capture outside it and then store under the lock.
        if (first_of_burst)
        {
            auto start_snapshot = capture_address_snapshot();
            const std::lock_guard<std::mutex> lock{coalescer_mutex};
            // Guard against a racing coalescer having already consumed this burst.
            if (has_pending_burst && !burst_start_valid)
            {
                burst_start_snapshot = std::move(start_snapshot);
                burst_start_valid = true;
            }
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
                if (safety_net_period.count() <= 0)
                {
                    coalescer_cv.wait(lock, [this] { return has_pending_burst || stop_requested; });
                    continue;
                }

                // wait_for returns the predicate's value, so false means "timed out
                // with still nothing to do" — time for a safety-net tick.
                const bool woken_by_work = coalescer_cv.wait_for(
                    lock, safety_net_period, [this] { return has_pending_burst || stop_requested; });
                if (woken_by_work)
                {
                    continue;
                }

                // Hold reset_in_progress across the tick so wait_for_idle() cannot
                // observe an idle coordinator while the tick is rebuilding participants.
                reset_in_progress = true;
                lock.unlock();
                try
                {
                    safety_net_tick();
                }
                catch (const std::exception &exception)
                {
                    // The tick emits log lines, and a user log callback can throw; a
                    // snapshot capture can throw bad_alloc. Letting either escape this
                    // thread function would call std::terminate AND leave
                    // reset_in_progress stuck true, hanging every wait_for_idle() for
                    // the rest of the process' life. Swallow, report, tick again next
                    // period.
                    log_error() << "network auto-recovery: periodic safety-net check failed (" << exception.what()
                                << "); will retry next period";
                }
                catch (...)
                {
                    log_error() << "network auto-recovery: periodic safety-net check failed (unknown exception); "
                                   "will retry next period";
                }
                lock.lock();
                reset_in_progress = false;
                coalescer_cv.notify_all();
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
            // Consume the burst-start snapshot under the lock so a subsequent
            // burst's on_kernel_event can't clobber the value run_reset will use.
            const bool had_burst_start = burst_start_valid;
            address_snapshot burst_start;
            if (had_burst_start)
            {
                burst_start = std::move(burst_start_snapshot);
            }
            burst_start_valid = false;
            lock.unlock();
            run_reset(burst_start, had_burst_start);
            lock.lock();
            reset_in_progress = false;
            coalescer_cv.notify_all();
        }
    }

    void network_recovery_coordinator::run_reset(const address_snapshot &burst_start, bool had_burst_start)
    {
        // last_known_snapshot is only mutated here (inside the coalescer
        // thread) and once during the lazy monitor-start in
        // register_participant — and that single initial write happens
        // BEFORE the coalescer thread is constructed, so no actual race
        // exists. Documented to forestall future readers wondering
        // whether the access pattern needs additional synchronisation.
        const auto new_snapshot = capture_address_snapshot();

        const bool end_changed = (new_snapshot != last_known_snapshot);
        // Transient flap: the burst OPENED with a different DDS-interesting address
        // set than last known (e.g. an address had just been removed) but the set is
        // back to normal by the time the burst settles. An end-snapshot-only diff
        // treats this as "nothing changed" and skips the rebuild — but the Fast-DDS
        // sockets bound to that address were torn down while it was gone, so a rebuild
        // is still required. (Container/veth/link-local churn can't trigger this: it is
        // filtered out of BOTH the start and end snapshots by capture_address_snapshot.)
        const bool transient_changed = had_burst_start && (burst_start != last_known_snapshot);

        if (!end_changed && !transient_changed)
        {
            log_info() << "network event burst — snapshot unchanged (" << new_snapshot.size()
                       << " interface address(es)), no reset";
            skipped_reset_count.fetch_add(1, std::memory_order_acq_rel);
            // A burst that changed nothing does not clear a pending retry: the
            // participant it refers to is still torn down, and the next safety-net
            // tick is what will pick it up.
            return;
        }

        if (end_changed)
        {
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
        }
        else
        {
            // Transient: end-state matches last known (added/removed both 0), so log
            // the actual reason rather than a misleading "+0 / -0".
            log_info() << "network change detected: transient interface change within the debounce window "
                          "(a DDS-relevant address left and returned, end-state unchanged); "
                          "resetting recovery-enabled participants";
        }

        last_known_snapshot = new_snapshot;
        // As in safety_net_tick: a real change re-arms bounded retrying.
        consecutive_retry_passes = 0;
        retry_exhaustion_reported = false;
        apply_reset(reset_scope::all);
    }

    void network_recovery_coordinator::apply_reset(const reset_scope scope)
    {
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

        std::size_t reset_participants = 0;
        std::size_t still_unrecovered = 0;
        for (const auto &participant : live_participants)
        {
            if (scope == reset_scope::retry_only && !participant->needs_network_recovery_retry())
            {
                continue;  // Intact — a retry pass must not disturb it.
            }

            ++reset_participants;
            try
            {
                participant->trigger_network_recovery_reset();
            }
            catch (const std::exception &exception)
            {
                log_error() << "participant reset failed: " << exception.what();
            }

            // The participant reports this itself: an exception is not the only way a
            // rebuild can fail (create_participant returning nullptr leaves it dead
            // without throwing), and only the participant knows whether its endpoints
            // came back.
            if (participant->needs_network_recovery_retry())
            {
                ++still_unrecovered;
            }
        }

        // No bookkeeping of who still needs a retry: the next safety-net tick asks the
        // participants themselves (any_participant_needs_retry). A cached flag here
        // could only ever go stale — a reset driven straight through
        // domain_participant::trigger_network_recovery_reset, for instance, never
        // reaches this function at all, yet can leave a participant torn down.
        auto log = log_info();
        log << "reset complete (" << reset_participants << " participant(s)";
        if (still_unrecovered > 0)
        {
            log << ", " << still_unrecovered << " still unrecovered — will retry";
        }
        log << ")";
        reset_count.fetch_add(1, std::memory_order_acq_rel);
    }

    void network_recovery_coordinator::safety_net_tick()
    {
        // 1. Bring the kernel notification channel back if its worker died. Do this
        //    first: everything below is a poor substitute for real events.
        ensure_monitor_alive();

        // 2. Retry participants a previous reset left torn down. They are broken right
        //    now, independently of whether the network has moved since — but bounded, so
        //    an endpoint that can never come back does not churn its healthy siblings
        //    once per period forever (see max_consecutive_retry_passes).
        if (!any_participant_needs_retry())
        {
            consecutive_retry_passes = 0;
            retry_exhaustion_reported = false;
        }
        else if (consecutive_retry_passes < max_consecutive_retry_passes)
        {
            ++consecutive_retry_passes;
            log_info() << "network auto-recovery: retrying participant(s) left unrecovered by a failed rebuild "
                          "(attempt "
                       << consecutive_retry_passes << " of " << max_consecutive_retry_passes << ")";
            apply_reset(reset_scope::retry_only);
        }
        else if (!retry_exhaustion_reported)
        {
            retry_exhaustion_reported = true;
            log_error() << "network auto-recovery: gave up rebuilding participant(s) after "
                        << max_consecutive_retry_passes
                        << " consecutive attempts; they stay inactive (publish/take report failure) until the next "
                           "network change. Retrying further would keep tearing down their healthy siblings.";
        }

        // 3. Re-verify the snapshot directly, catching any change no event reported —
        //    a dropped netlink datagram (ENOBUFS), an interface transition on a channel
        //    we do not subscribe to, or a change that raced the monitor's startup.
        auto new_snapshot = capture_address_snapshot();
        if (new_snapshot == last_known_snapshot)
        {
            // Deliberately silent: this runs on a timer for the life of the process.
            return;
        }

        log_info() << "network change detected by the periodic safety-net check — no kernel event reported it ("
                   << last_known_snapshot.size() << " → " << new_snapshot.size()
                   << " interface address(es)); resetting recovery-enabled participants";
        last_known_snapshot = std::move(new_snapshot);
        // A real change re-arms retrying: the reason a rebuild failed before may well be
        // gone now, so the give-up above must not be permanent.
        consecutive_retry_passes = 0;
        retry_exhaustion_reported = false;
        apply_reset(reset_scope::all);
    }

    bool network_recovery_coordinator::any_participant_needs_retry()
    {
        const std::lock_guard<std::mutex> lock{registry_mutex};
        return std::any_of(registered_participants.begin(), registered_participants.end(),
                           [](const std::weak_ptr<domain_participant> &weak) {
                               const auto strong = weak.lock();
                               return strong && strong->needs_network_recovery_retry();
                           });
    }

    void network_recovery_coordinator::ensure_monitor_alive()
    {
        // Logs are emitted after the lock is released: a user log callback may re-enter
        // register_participant, which takes registry_mutex → monitor_mutex.
        bool reopened = false;
        bool needed_reopen = false;
        std::string reopen_error;

        {
            const std::lock_guard<std::mutex> lock{monitor_mutex};
            if (monitor && monitor->is_alive())
            {
                return;
            }

            needed_reopen = true;
            try
            {
                // Destroy first: the dead worker has already returned, so the join in
                // ~network_monitor is immediate, and this releases the old channel's fds
                // before we ask the kernel for new ones.
                monitor.reset();
                monitor = std::make_unique<network_monitor>([this] { on_kernel_event(); });
                reopened = true;
            }
            catch (const std::exception &exception)
            {
                monitor.reset();
                reopen_error = exception.what();
            }
        }

        if (!needed_reopen)
        {
            return;
        }
        if (reopened)
        {
            log_warning() << "network auto-recovery: the kernel notification channel had died and was reopened; "
                             "events missed while it was down are covered by this periodic check";
        }
        else
        {
            log_error() << "network auto-recovery: the kernel notification channel died and could not be reopened ("
                        << reopen_error << "); recovery continues on the periodic check alone";
        }
    }
}  // namespace provizio::dds::detail
