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
#include <condition_variable>
#include <mutex>

#include "provizio/dds/common.h"

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
         * Memory ordering: see @c entered — both atomic operations on this
         * side use @c seq_cst to pair with the callback side and guarantee
         * the Dekker invariant.
         */
        void detach_and_drain() noexcept
        {
            detached.store(true, std::memory_order_seq_cst);
            std::unique_lock<std::mutex> lock{drain_mutex};
            drain_cv.wait(lock, [this] { return in_flight.load(std::memory_order_seq_cst) == 0; });
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
        std::atomic<int> in_flight{0};
        std::atomic<bool> detached{false};
        std::mutex drain_mutex;
        std::condition_variable drain_cv;
    };
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_LISTENER_DRAIN
