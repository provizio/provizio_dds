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

#ifndef DDS_DETAIL_NETWORK_RECOVERY_COORDINATOR
#define DDS_DETAIL_NETWORK_RECOVERY_COORDINATOR

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "provizio/dds/common.h"
#include "provizio/dds/detail/address_snapshot.h"
#include "provizio/dds/detail/network_monitor.h"

namespace provizio::dds
{
    class domain_participant;
}  // namespace provizio::dds

namespace provizio::dds::detail
{
    /**
     * @file network_recovery_coordinator.h
     * @brief Process-wide registry of recovery-enabled @c domain_participant instances,
     * coalescing logic, and the orchestrator that performs the actual reset on
     * network-interface address changes.
     *
     * Single instance per process, created lazily on the first
     * @c make_domain_participant call with auto-recovery enabled. The monitor thread
     * and the coalescer thread are started at that point and run until process exit.
     *
     * Lifecycle:
     *   - When a participant is created with auto-recovery enabled, it calls
     *     @c network_recovery_coordinator::instance().register_participant(self).
     *   - The coordinator starts its @c network_monitor on first registration,
     *     then captures the initial address snapshot. Opening the monitor first
     *     ensures any events fired while we capture the snapshot are still
     *     queued for delivery and won't be silently dropped.
     *   - On each raw kernel event, the monitor calls @c on_kernel_event() which
     *     records the wall-clock time and notifies the coalescer thread.
     *   - The coalescer thread waits up to @c quiet_period of silence since the
     *     last event (or up to @c max_debounce since the first event in the burst)
     *     and then captures a fresh @c address_snapshot.
     *   - If the snapshot differs from the last known snapshot, the coalescer
     *     walks the live participant list and triggers reset on each.
     */
    // No class-level PROVIZIO_DDS_API: the class has std::mutex /
    // std::condition_variable / std::thread / std::vector<std::weak_ptr<...>> /
    // std::atomic members. Class-level dllexport on MSVC triggers C4251 for
    // each of those and locks any consumer to the exact same MSVC STL build.
    // Per-method PROVIZIO_DDS_API on the symbols that cross the DLL boundary
    // achieves the same export without the STL-export liability — the
    // members themselves remain implementation detail.
    class network_recovery_coordinator
    {
      public:
        /**
         * @brief How long the kernel channel must be silent after the last event
         * before we trigger a reset. 3 seconds — short enough to feel responsive when
         * the network actually settles; long enough to coalesce a DHCP/dual-stack
         * handshake burst into one reset.
         */
        static constexpr std::chrono::seconds quiet_period{3};

        /**
         * @brief Hard ceiling on coalescing: even if events keep arriving we won't
         * delay the reset past this many seconds from the first event in the burst.
         * Without this, a constantly flapping interface could starve consumers of
         * any recovery at all.
         */
        static constexpr std::chrono::seconds max_debounce{60};

        /**
         * @brief Returns the per-process singleton, constructing it on first call.
         * Construction is fully lazy: a process that never creates a
         * recovery-enabled participant pays nothing.
         */
        static PROVIZIO_DDS_API network_recovery_coordinator &instance();

        /**
         * @brief Register a domain_participant for auto-recovery. Stored as a
         * @c weak_ptr so that natural shared_ptr lifetime ends the registration.
         * Idempotent for the same participant. Starts the network monitor on first
         * registration.
         */
        PROVIZIO_DDS_API void register_participant(const std::shared_ptr<domain_participant> &participant);

        // ----------------------------------------------------------------------
        // Observability / test hooks. Always present in the ABI (rather than
        // gated behind a compile-time flag — that would change the class layout
        // between consumers configured with and without the flag and produce
        // memory-corruption-class bugs on mismatch). Cost is negligible: two
        // atomics and a handful of method symbols.
        //
        // These are documented as test-only; production code does not need
        // them.
        // ----------------------------------------------------------------------

        /**
         * @brief Block until the coordinator is idle (no burst pending AND no
         * reset in progress). Test-only entry point — production code interacts
         * with the coordinator via the participant lifecycle.
         */
        PROVIZIO_DDS_API void wait_for_idle();

        /**
         * @brief Inject a synthetic kernel event from a test. Lets the test
         * drive coalescing without actually changing the host's network state.
         *
         * Out-of-line and @c PROVIZIO_DDS_API'd because the body calls the
         * private @c on_kernel_event(), which the class deliberately does not
         * export wholesale (its STL members would trigger C4251 on MSVC).
         * An inline body would emit the call to a non-exported symbol from
         * the test TU and fail to link on Windows.
         */
        PROVIZIO_DDS_API void inject_kernel_event_for_test();

        /**
         * @brief Inject a synthetic transient flap from a test: drive the reset
         * decision as if a coalesced burst had STARTED with
         * @p simulated_burst_start and ended at the current real snapshot. Lets
         * a test exercise the transient path (a DDS-relevant address that left
         * and returned within the debounce window, netting to an unchanged
         * end-snapshot) without manipulating the host's interfaces.
         *
         * The reset is performed on the coalescer thread (not inline), so it is
         * asynchronous: call @c wait_for_idle() afterwards to await completion.
         *
         * Out-of-line / @c PROVIZIO_DDS_API'd for the same reason as
         * @c inject_kernel_event_for_test (the body calls private members the
         * class does not export wholesale).
         */
        PROVIZIO_DDS_API void inject_transient_for_test(const address_snapshot &simulated_burst_start);

        /**
         * @brief Number of resets the coordinator has run so far. Counts only
         * the "snapshot changed → participant rebuild" outcomes (see
         * @c skipped_reset_count_for_test for "snapshot unchanged → no rebuild").
         */
        std::uint64_t reset_count_for_test() const noexcept
        {
            return reset_count.load(std::memory_order_acquire);
        }

        /**
         * @brief Number of coalesced bursts whose snapshot equalled the last
         * known one — those that did NOT result in a participant rebuild.
         */
        std::uint64_t skipped_reset_count_for_test() const noexcept
        {
            return skipped_reset_count.load(std::memory_order_acquire);
        }

        network_recovery_coordinator(const network_recovery_coordinator &) = delete;
        network_recovery_coordinator &operator=(const network_recovery_coordinator &) = delete;
        network_recovery_coordinator(network_recovery_coordinator &&) = delete;
        network_recovery_coordinator &operator=(network_recovery_coordinator &&) = delete;

      private:
        network_recovery_coordinator();
        ~network_recovery_coordinator();

        void on_kernel_event();
        void coalescer_loop();
        void run_reset(const address_snapshot &burst_start, bool had_burst_start);

        // Lazily constructed when the first recovery-enabled participant registers.
        // The monitor's constructor opens the kernel channel and starts its worker;
        // its destructor cleans both up. Held by unique_ptr so the coordinator's
        // life doesn't pay for monitor construction in processes that never register
        // a recovery-enabled participant.
        std::unique_ptr<network_monitor> monitor;

        std::mutex registry_mutex;
        std::vector<std::weak_ptr<domain_participant>> registered_participants;

        std::mutex coalescer_mutex;
        std::condition_variable coalescer_cv;
        std::chrono::steady_clock::time_point first_event_time;
        std::chrono::steady_clock::time_point last_event_time;
        bool has_pending_burst{false};
        bool reset_in_progress{false};
        bool stop_requested{false};
        std::thread coalescer_thread;

        address_snapshot last_known_snapshot;

        // Snapshot captured at the FIRST event of the current coalescing burst,
        // before a quick flap can revert. A burst whose post-debounce end-snapshot
        // equals last_known_snapshot but whose start-snapshot differs means a
        // DDS-relevant address left and returned inside the debounce window — the
        // Fast-DDS sockets bound to it were torn down while it was gone, so a
        // rebuild is still required even though the end-state "looks unchanged".
        // Without this, such a transient is silently coalesced away (the bug this
        // guards against). Both fields are guarded by coalescer_mutex.
        address_snapshot burst_start_snapshot;
        bool burst_start_valid{false};

        // Observability counters — always present (see test-hooks comment above
        // for why they are unconditional rather than #ifdef'd).
        std::atomic<std::uint64_t> reset_count{0};
        std::atomic<std::uint64_t> skipped_reset_count{0};
    };
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_NETWORK_RECOVERY_COORDINATOR
