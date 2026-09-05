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

#include "detail/env_utils.h"
#include "provizio/dds/detail/vpn_interfaces.h"
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
            const auto quoted = detail::sanitise_env_value_for_log(value);
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
        std::string init_error;
        std::string env_warning;
        // The third deferred diagnostic, for the same reason as the two above: see
        // current_snapshot, which cannot log for itself while this holds registry_mutex.
        std::string snapshot_warning;

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
            // The joinable() check is load-bearing, not belt-and-braces: after a monitor
            // reopen failure in ensure_monitor_alive(), `monitor` is null while the
            // coalescer thread is alive and JOINABLE — re-initialising here would then
            // assign to a joinable std::thread (std::terminate) and race the plain writes
            // to safety_net_period / last_known_snapshot against the live coalescer's
            // reads. It would also be redundant: while the coalescer runs, every
            // safety-net tick already retries reopening the kernel channel.
            if (!monitor && !coalescer_thread.joinable())
            {
                try
                {
                    safety_net_period = resolve_safety_net_period(env_warning);
                    monitor = std::make_unique<network_monitor>([this] { on_kernel_event(); });
                    // Through current_snapshot(), not capture_address_snapshot(), so a failed
                    // read leaves the baseline UNSET (nullopt) instead of seeding the empty
                    // set: seeding it would make the first successful read look like every
                    // address arriving at once and rebuild every participant for nothing. Using
                    // the same accessor as every other read is also what keeps the two in step.
                    last_known_snapshot = current_snapshot(&snapshot_warning);
                    coalescer_thread = std::thread{[this] { coalescer_loop(); }};
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
        if (!snapshot_warning.empty())
        {
            log_warning() << snapshot_warning;
        }
        // Nothing is logged when the monitor starts successfully. That auto-recovery is
        // enabled, how many addresses it saw and how often it re-checks are this library's
        // internals; a working feature has nothing to tell the user. Only its FAILURE to
        // start does, because that leaves auto-recovery unavailable.
        if (!init_error.empty())
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

    void network_recovery_coordinator::force_enumeration_failure_for_test(const bool fail)
    {
        // Only what its name says: it makes the read fail, and touches nothing else. In
        // particular it does NOT re-arm the once-per-streak warning (see current_snapshot),
        // tempting though that is -- a test could then never observe the production re-arm,
        // because no second failure streak is reachable without coming back through here and
        // being handed a fresh latch either way. A hook that quietly repairs the state under
        // test can only hide a regression in it.
        forced_enumeration_failure_for_test = fail;
    }

    void network_recovery_coordinator::force_snapshot_for_test(std::optional<address_snapshot> snapshot)
    {
        // No lock, for the same reason as seed_last_known_snapshot_for_test: idle-only by
        // contract, so nothing reads it concurrently.
        forced_snapshot_for_test = std::move(snapshot);
    }

    std::optional<address_snapshot> network_recovery_coordinator::current_snapshot(std::string *deferred_warning)
    {
        // The forced failure joins the real one rather than returning early, so a test
        // exercises everything a failed read does -- the diagnostic included, which is the
        // only way to reach the callback re-entrancy this path has to survive.
        bool enumeration_failed = forced_enumeration_failure_for_test;
        address_snapshot snapshot;
        if (!enumeration_failed)
        {
            if (forced_snapshot_for_test)
            {
                // Returned as the optional it already is: dereferencing it here only to have
                // the return type wrap it again is what bugprone-optional-value-conversion
                // objects to.
                return forced_snapshot_for_test;
            }
            snapshot = capture_address_snapshot(&enumeration_failed);
        }
        if (enumeration_failed)
        {
            // Warned once per streak, not once per attempt: a poller asks every few
            // seconds, and a host that has genuinely lost the ability to enumerate would
            // otherwise fill the log with the same line for the life of the process. The
            // exchange makes the "first of the streak" decision atomic, because this runs on
            // whichever thread is reading the interfaces — the coalescer for a reset
            // decision, the notification thread for a burst-start snapshot.
            if (!enumeration_failure_reported.exchange(true, std::memory_order_relaxed))
            {
                constexpr const char *const message =
                    "could not read this host's network interfaces; keeping the last known "
                    "address set and making no participant rebuild decision until it can be read "
                    "again (an unreadable interface list is not an interface change)";
                if (deferred_warning != nullptr)
                {
                    // A caller holding a lifecycle lock takes the text and emits it once it
                    // does not: the log callback is documented as free to create a
                    // participant, which would re-enter register_participant and block on
                    // the non-recursive registry_mutex this runs under. Every other
                    // diagnostic on that path (init_error, env_warning) is deferred for the
                    // same reason, and this is the one read that can fail on it -- the
                    // macOS sysctl(NET_RT_IFLIST) race this feature exists to tolerate.
                    *deferred_warning = message;
                }
                else
                {
                    log_warning() << message;
                }
            }
            return std::nullopt;
        }
        enumeration_failure_reported.store(false, std::memory_order_relaxed);
        return snapshot;
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
            // An unreadable interface list leaves the burst with no start snapshot rather
            // than an empty one: run_reset() would otherwise see every address as having
            // "returned" during the burst and rebuild for it.
            auto start_snapshot = current_snapshot();
            const std::lock_guard<std::mutex> lock{coalescer_mutex};
            // Guard against a racing coalescer having already consumed this burst.
            if (start_snapshot && has_pending_burst && !burst_start_valid)
            {
                burst_start_snapshot = std::move(*start_snapshot);
                burst_start_valid = true;
            }
        }

        coalescer_cv.notify_all();
    }

    void network_recovery_coordinator::coalescer_loop()
    {
        std::unique_lock<std::mutex> lock{coalescer_mutex};
        // Absolute next-tick deadline, re-armed only when a tick actually runs. A
        // relative wait_for would restart the full period on EVERY burst wake-up, and
        // the kernel subscription is unfiltered (every veth/docker link event on the
        // host raises a burst), so sustained interface churn could postpone the tick
        // indefinitely — and with it the failed-rebuild retry, which runs only from
        // ticks. An absolute deadline caps the gap between ticks at one period no
        // matter how busy the event stream is.
        auto next_tick = std::chrono::steady_clock::now() + safety_net_period;
        while (!stop_requested)
        {
            if (!has_pending_burst)
            {
                if (safety_net_period.count() <= 0)
                {
                    coalescer_cv.wait(lock, [this] { return has_pending_burst || stop_requested; });
                    continue;
                }

                // wait_until returns the predicate's value, so false means "the tick
                // deadline passed with still nothing to do" — time for a safety-net tick.
                const bool woken_by_work =
                    coalescer_cv.wait_until(lock, next_tick, [this] { return has_pending_burst || stop_requested; });
                if (woken_by_work)
                {
                    continue;
                }
                next_tick = std::chrono::steady_clock::now() + safety_net_period;

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

    void network_recovery_coordinator::adopt_first_readable_snapshot(const address_snapshot &new_snapshot)
    {
        last_known_snapshot = new_snapshot;

        if (new_snapshot.empty())
        {
            // Nothing to bind, so nothing a rebuild could achieve -- the same judgement the
            // added == 0 path below makes. A host that genuinely has no usable address reads
            // exactly this (a container whose only device is a filtered-out veth), and so does
            // one whose addresses arrive a moment later; either way the next change decides.
            skipped_reset_count.fetch_add(1, std::memory_order_acq_rel);
            return;
        }

        log_info() << "no interface baseline to compare against (the list could not be read at "
                      "startup), so all "
                   << new_snapshot.size()
                   << " interface address(es) now visible are treated as new; resetting "
                      "recovery-enabled participants";
        // A real change re-arms bounded retrying, as in the paths below.
        consecutive_retry_passes = 0;
        retry_exhaustion_reported = false;
        apply_reset(reset_scope::all);
    }

    void network_recovery_coordinator::run_reset(const address_snapshot &burst_start, bool had_burst_start)
    {
        // last_known_snapshot is only mutated here (inside the coalescer
        // thread) and once during the lazy monitor-start in
        // register_participant — and that single initial write happens
        // BEFORE the coalescer thread is constructed, so no actual race
        // exists. Documented to forestall future readers wondering
        // whether the access pattern needs additional synchronisation.
        const auto captured = current_snapshot();
        if (!captured)
        {
            // Could not read the interfaces — see current_snapshot. Deliberately does NOT
            // count as a skipped reset: nothing was decided, and last_known_snapshot is
            // left alone so the next readable snapshot is compared against the last one
            // this actually saw.
            return;
        }
        const auto &new_snapshot = *captured;

        decide_and_apply(new_snapshot, burst_start, had_burst_start, change_source::kernel_event);
    }

    void network_recovery_coordinator::decide_and_apply(const address_snapshot &new_snapshot,
                                                        const address_snapshot &burst_start, const bool had_burst_start,
                                                        const change_source source)
    {
        if (!last_known_snapshot)
        {
            // No baseline to measure against: the read at startup failed, so every address
            // now visible counts as new. Handled here rather than in each caller so both
            // paths reach it, and so the counts below always have a baseline to subtract.
            adopt_first_readable_snapshot(new_snapshot);
            return;
        }
        const address_snapshot &last_known = *last_known_snapshot;
        // Taken now, not read from last_known where the log lines need it: the branches
        // below adopt the new set (assigning last_known_snapshot, which leaves the
        // reference above dangling), so a future edit that logged after adopting would
        // otherwise be reading freed memory.
        const std::size_t previous_size = last_known.size();

        // Three counts decide this, and the same three describe it in the log. Additions
        // and removals are measured against the last set we rebuilt for; `returned` is
        // measured against the start of this burst.
        const auto count_missing_from = [](const address_snapshot &addresses, const address_snapshot &reference) {
            return static_cast<std::size_t>(std::count_if(addresses.begin(), addresses.end(),
                                                          [&reference](const address_snapshot::value_type &address) {
                                                              return reference.find(address) == reference.end();
                                                          }));
        };
        const std::size_t added = count_missing_from(new_snapshot, last_known);
        const std::size_t removed = count_missing_from(last_known, new_snapshot);
        // Transient flap: an address the host has NOW was missing when the burst opened,
        // so it left and came back inside the debounce window. An end-snapshot-only diff
        // treats that as "nothing changed" and skips the rebuild — but the Fast-DDS
        // sockets bound to that address were torn down while it was gone, so a rebuild is
        // still required. Only an event-driven backend can observe it; a poller, and the
        // safety-net tick, pass no burst-start snapshot. (Container/veth/link-local churn
        // can't trigger this: it is filtered out of BOTH the start and end snapshots by
        // capture_address_snapshot.)
        const std::size_t returned = had_burst_start ? count_missing_from(new_snapshot, burst_start) : 0U;

        // Names the path that noticed the change, and nothing more. Worth saying for the
        // safety net: a change that reached us without a kernel event means notifications
        // were lost, which is the one thing that distinguishes a healthy monitor from a
        // silently broken one in a log.
        const char *const noticed_by = (source == change_source::safety_net)
                                           ? " by the periodic safety-net check -- no kernel event reported it"
                                           : "";

        // A rebuild is worth doing only when the host has GAINED something to bind — a
        // new address, or one that went away and came back. Addresses merely going away
        // are handled below.
        if (added == 0 && returned == 0)
        {
            if (removed == 0)
            {
                // Silent: a change that changed nothing is a non-event. The counter below is
                // what the tests observe.
                skipped_reset_count.fetch_add(1, std::memory_order_acq_rel);
                // A burst that changed nothing does not clear a pending retry: the
                // participant it refers to is still torn down, and the next safety-net
                // tick is what will pick it up.
                return;
            }

            // Addresses only went away. Rebuilding cannot bind what is gone, and it would
            // tear down endpoints still working over the interfaces that remain — so the
            // smaller set is adopted and nothing is rebuilt. This is not a lost signal: if
            // an address comes back, even the very same one, it counts as `added` here (or
            // as `returned` when it happens inside one burst) and the rebuild happens then,
            // which is the moment it can actually achieve something. The cost of waiting is
            // that this participant keeps announcing a locator that no longer answers until
            // the next real change — cheaper than dropping every in-flight sample now.
            log_info() << "network change detected" << noticed_by << ": +0 / -" << removed << " interface address(es) ("
                       << previous_size << " -> " << new_snapshot.size()
                       << "); not rebuilding for a loss alone -- will rebuild if address(es) return";
            // Adopted so a return reads as a gain rather than as "no change".
            last_known_snapshot = new_snapshot;
            skipped_reset_count.fetch_add(1, std::memory_order_acq_rel);
            return;
        }

        if (added != 0 || removed != 0)
        {
            // Quick diff summary for the log: counts of additions and removals
            // give a more useful one-liner than "old.size() → new.size()" which
            // could be the same even when contents fully differ.
            log_info() << "network change detected" << noticed_by << ": +" << added << " / -" << removed
                       << " interface address(es) (" << previous_size << " -> " << new_snapshot.size()
                       << "); resetting recovery-enabled participants";
        }
        else
        {
            // Transient: end-state matches last known (added/removed both 0), so log
            // the actual reason rather than a misleading "+0 / -0".
            log_info() << "network change detected" << noticed_by
                       << ": transient interface change within the debounce window "
                          "(a DDS-relevant address left and returned, end-state unchanged); "
                          "resetting recovery-enabled participants";
        }

        last_known_snapshot = new_snapshot;
        // A real change re-arms bounded retrying: the reason a rebuild failed before may
        // well be gone now, so an earlier give-up must not be permanent.
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

        // One enumeration of the host's VPN / tunnel interfaces for the whole pass rather
        // than one per participant rebuilt: every participant here is being rebuilt for the
        // SAME network change, so they should be configured for the same interface set, and
        // asking the OS again for each of them costs an rtnetlink dump per participant to
        // answer a question that has not changed in between.
        const detail::scoped_vpn_blocklist_cache vpn_blocklist_cache;

        // Filled HERE, before the first participant takes its reset lock, and not left to
        // the first refresh that needs it. On Linux that call is an rtnetlink RTM_GETLINK
        // dump with a receive timeout, and a participant's refresh runs inside
        // reset_mutex held exclusively -- so priming it there would stall every concurrent
        // publish(), take() and register_topic() on that participant for the length of a
        // netlink round trip. Out here it blocks nobody: the participants are still live and
        // unlocked. The result is what the cache above hands to all of them.
        (void)detail::vpn_interface_blocklist_entries();

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
        // No "reset complete" line: the reset was already announced by the "network change
        // detected" message above, and a rebuild that FAILED logs its own error. A success
        // confirmation would only repeat what silence already says.
        static_cast<void>(reset_participants);
        static_cast<void>(still_unrecovered);
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
            // Silent: retrying is internal bookkeeping. The user hears about it only if every
            // attempt is exhausted, which the error below reports.
            ++consecutive_retry_passes;
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
        const auto captured = current_snapshot();
        if (!captured)
        {
            return;  // Unreadable interfaces are not a change — see current_snapshot.
        }
        const auto &new_snapshot = *captured;
        if (last_known_snapshot && new_snapshot == *last_known_snapshot)
        {
            // Deliberately silent, and deliberately not counted as a skipped reset: this
            // runs on a timer for the life of the process, so an unchanged host would
            // otherwise emit a line and move a counter once per period forever.
            return;
        }

        // The same decision the event path makes, for the same reasons — including not
        // rebuilding for a loss alone. The tick exists to catch changes whose events were
        // never delivered, and such a change is no more deserving of a rebuild than one
        // that arrived normally. It passes no burst-start snapshot: a periodic re-read
        // cannot say what the host looked like when the change began, so a flap that
        // started and ended between two ticks is invisible to it either way.
        decide_and_apply(new_snapshot, address_snapshot{}, /*had_burst_start=*/false, change_source::safety_net);
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
            // Silent: the channel died and this repaired it, and the events missed meanwhile
            // are covered by the very check that repaired it. Self-healed, so nothing to say.
            return;
        }
        {
            log_error() << "network auto-recovery: the kernel notification channel died and could not be reopened ("
                        << reopen_error << "); recovery continues on the periodic check alone";
        }
    }
}  // namespace provizio::dds::detail
