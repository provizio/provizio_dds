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

#ifndef DDS_DETAIL_LISTENER_DRAIN
#define DDS_DETAIL_LISTENER_DRAIN

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include "provizio/dds/common.h"
#include "provizio/dds/logging.h"

namespace provizio::dds::detail
{
    /**
     * @file listener_drain.h
     * @brief Detach + drain primitive for Fast-DDS listener callbacks.
     *
     * Background: during a network-recovery reset, the participant's @c reset_mutex
     * is held exclusively. If a Fast-DDS listener callback (e.g. our @c on_data_available
     * forwarding a sample to the user's lambda) re-enters provizio_dds — typically via
     * @c publish() on a sibling publisher — that re-entry tries to acquire
     * @c reset_mutex shared and blocks. The reset then calls
     * @c DataReader::delete_datareader, which waits for the in-flight callback to
     * return — deadlock.
     *
     * The fix is to detach the listener and wait for in-flight callbacks to drain
     * BEFORE taking @c reset_mutex exclusively. While in this drain phase no provizio
     * lock is held, so a callback that re-enters @c publish() / @c get_guid() can
     * complete and the drain finishes.
     *
     * Usage: every provizio-owned Fast-DDS listener holds a @c listener_drain
     * member. The listener wraps its callback body with @c entered() / @c left()
     * (or uses @c scoped_call as an RAII pair) and bails out early if @c detached()
     * is observed. The reset path calls @c detach_and_drain() once per affected
     * endpoint before acquiring the exclusive lock, and @c reattach() after the
     * new listener is in place.
     */
    class listener_drain
    {
      public:
        listener_drain() noexcept = default;

        /**
         * @brief Construct with a non-default stall-reporting slice.
         *
         * @param slice How long the drain waits before reporting that it is still waiting,
         * and between repeats of that report. Purely diagnostic: it never bounds the total
         * wait (see @c detach_and_drain). Every shipped endpoint takes the default; the
         * parameter exists so a test can observe the reporting without spending the default
         * slice per line.
         */
        explicit listener_drain(const std::chrono::milliseconds slice) noexcept : stall_warning_period{slice}
        {
        }

        listener_drain(const listener_drain &) = delete;
        listener_drain &operator=(const listener_drain &) = delete;
        listener_drain(listener_drain &&) = delete;
        listener_drain &operator=(listener_drain &&) = delete;

        /**
         * @brief Record entry to a callback. Returns @c true if the callback body
         * should proceed, @c false if a reset is in progress and the body must
         * return without touching user state. The counter is incremented either way;
         * callers must pair every @c entered() call with @c left().
         *
         * Memory ordering: both the @c in_flight increment and the @c detached
         * load use @c seq_cst so the symmetric Dekker-style race against
         * @c detach_and_drain (which stores @c detached then loads
         * @c in_flight) cannot have both sides miss the other's write. A
         * weaker acquire/release pair would be legal under IRIW on
         * non-multi-copy-atomic architectures.
         */
        bool entered() noexcept
        {
            in_flight.fetch_add(1, std::memory_order_seq_cst);
            if (detached.load(std::memory_order_seq_cst))
            {
                // Still counts as "in flight" for the purposes of the drain — a
                // racing detach_and_drain() that sees the bump must wait for the
                // matching left().
                return false;
            }
            return true;
        }

        /**
         * @brief Record exit from a callback. Wakes any thread blocked in
         * @c detach_and_drain() once the counter reaches zero.
         */
        void left() noexcept
        {
            if (in_flight.fetch_sub(1, std::memory_order_acq_rel) == 1)
            {
                const std::lock_guard<std::mutex> lock{drain_mutex};
                drain_cv.notify_all();
            }
        }

        /**
         * @brief RAII pair for a single callback invocation. Destructor always
         * decrements the in-flight counter; @c should_run() tells the caller
         * whether to execute the callback body.
         */
        class scoped_call
        {
          public:
            explicit scoped_call(listener_drain &drain) noexcept : the_drain(&drain)
            {
                run = drain.entered();
            }
            ~scoped_call()
            {
                the_drain->left();
            }
            scoped_call(const scoped_call &) = delete;
            scoped_call &operator=(const scoped_call &) = delete;
            scoped_call(scoped_call &&) = delete;
            scoped_call &operator=(scoped_call &&) = delete;

            bool should_run() const noexcept
            {
                return run;
            }

          private:
            listener_drain *the_drain;
            bool run{false};
        };

        /**
         * @brief Set the detached flag and block until every in-flight callback
         * has returned. Must NOT be called while holding any lock that an
         * in-flight callback might want — in particular the participant's
         * @c reset_mutex (shared or exclusive).
         *
         * @note The participant's @c registration_mutex, by contrast, IS held across
         * this call by @c trigger_network_recovery_reset, deliberately: it is what stops
         * an endpoint being registered against a participant that is being torn down. A
         * data callback may not take it either way — creating a publisher, subscriber or
         * service from one is already forbidden, and is what the stall warning below
         * reports. The consequence worth stating is for the LOG callback, which the
         * warning invokes on this thread with that mutex held: creating an endpoint from
         * it would self-deadlock. That is why @c log_callback's contract rules endpoint
         * creation out (see logging.h). Deferring this one message until the locks are
         * released — the discipline every other diagnostic in the library follows — is not
         * available here: the stall it reports is unbounded, so a deferred message could
         * be emitted only once the drain finished, which is exactly what has not happened.
         *
         * Memory ordering: see @c entered — both atomic operations on this
         * side use @c seq_cst to pair with the callback side and guarantee
         * the Dekker invariant.
         */
        void detach_and_drain() noexcept
        {
            detached.store(true, std::memory_order_seq_cst);

            // Waits in bounded slices with an unbounded total: the contract is still
            // "block until every in-flight callback has returned", but a callback that
            // never returns (one that blocks on a lock the caller holds, or that creates
            // an endpoint during a reset — both forbidden, see the class docs) would
            // otherwise hang here forever with nothing in the log to say why. The slice
            // only decides how soon that is reported; it never ends the wait early,
            // because proceeding while a callback is still running would tear down the
            // Fast-DDS entity underneath it.
            //
            // Reported once when the stall starts and again on every slice after it,
            // because one line cannot say whether the stall is over: the wait is unbounded
            // and the caller holds registration_mutex throughout, so an operator who sees a
            // single warning has no way to tell a stall that cleared ten seconds later from
            // one still going hours on. Repeating turns it into a heartbeat, and the slice
            // is long enough (see stall_warning_period) that a wedged process produces a
            // line a few times a minute rather than a flood.
            std::size_t stalled_slices = 0;
            // The real elapsed time, because the completion line below must report what the
            // reset actually cost. Deriving it from the slice count would undercount by
            // however far into the final slice the drain finished -- a stall cleared 2.5 s
            // into its second slice would be reported as 5 s rather than 7.5 s.
            const auto stall_started = std::chrono::steady_clock::now();
            while (true)
            {
                bool drained = false;
                {
                    std::unique_lock<std::mutex> lock{drain_mutex};
                    drained = drain_cv.wait_for(lock, stall_warning_period,
                                                [this] { return in_flight.load(std::memory_order_seq_cst) == 0; });
                }

                // Reported with drain_mutex released: log_warning invokes the user's log
                // callback, which is free to use provizio APIs (publishing a log line onto
                // a topic is a supported use), and holding this class' mutex across that
                // would silently add it to that contract. The participant's
                // registration_mutex IS still held here — see the note on this function for
                // why that cannot be helped, and why creating an endpoint from a log
                // callback is ruled out because of it. try/catch keeps the noexcept promise
                // if such a callback throws.
                if (drained)
                {
                    if (stalled_slices != 0)
                    {
                        // Only where a stall was reported: the log needs an end as well as
                        // a beginning, or the last warning stands as the final word on a
                        // reset that in fact completed.
                        try
                        {
                            log_warning() << "listener drain completed after " << elapsed_seconds_since(stall_started)
                                          << " s; the reset is no longer blocked.";
                        }
                        catch (...)
                        {
                            // As below.
                        }
                    }
                    return;
                }

                ++stalled_slices;
                try
                {
                    log_warning() << "listener drain has been waiting for " << in_flight.load(std::memory_order_seq_cst)
                                  << " in-flight callback(s) for over " << waited_seconds(stalled_slices)
                                  << " s. A callback that never returns blocks the "
                                     "network-recovery reset (and endpoint teardown) "
                                     "indefinitely: callbacks must not block and must "
                                     "not create publishers / subscribers / services.";
                }
                catch (...)
                {
                    // Nothing useful to do — a logging failure must not turn a
                    // diagnostic into a crash.
                }
            }
        }

        /**
         * @brief Clear the detached flag so callback bodies resume executing.
         * Safe to call from any thread; intended for the reset path after the new
         * Fast-DDS listener has been bound to its DataReader / DataWriter.
         */
        void reattach() noexcept
        {
            detached.store(false, std::memory_order_release);
        }

        /**
         * @brief Whether the drain is currently detached. Exposed for tests and
         * for callbacks that prefer the explicit @c entered() / @c left() form.
         */
        bool is_detached() const noexcept
        {
            return detached.load(std::memory_order_acquire);
        }

      private:
        /// @brief How long ago @p started was, in seconds. What the completion report needs:
        /// the drain can finish part way through a slice, so the number of slices that
        /// expired is a floor rather than the answer.
        static double elapsed_seconds_since(const std::chrono::steady_clock::time_point started) noexcept
        {
            return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - started)
                .count();
        }

        /// @brief How long @p slices of stalled waiting add up to, in seconds. Seconds
        /// because that is the unit an operator reading the log thinks in; a double because
        /// a slice need not be a whole number of them.
        double waited_seconds(const std::size_t slices) const noexcept
        {
            constexpr double milliseconds_per_second = 1000.0;
            return static_cast<double>(stall_warning_period.count()) * static_cast<double>(slices) /
                   milliseconds_per_second;
        }

        /// The default slice: long enough that a wedged process reports itself a few times
        /// a minute rather than flooding the log.
        static constexpr std::chrono::milliseconds default_stall_warning_period{5000};

        /// How long the drain waits before reporting that it is still waiting, and between
        /// repeats of that report. Purely diagnostic: it does not bound the total wait (see
        /// detach_and_drain). Const, so it needs no synchronisation against the callback
        /// threads that read nothing but the atomics below.
        const std::chrono::milliseconds stall_warning_period{default_stall_warning_period};

        std::atomic<int> in_flight{0};
        std::atomic<bool> detached{false};
        std::mutex drain_mutex;
        std::condition_variable drain_cv;
    };
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_LISTENER_DRAIN
