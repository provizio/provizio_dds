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

#ifndef DDS_NETWORK_RECOVERY
#define DDS_NETWORK_RECOVERY

#include "provizio/dds/common.h"

namespace provizio::dds
{
    /**
     * @file network_recovery.h
     * @brief Per-participant opt-in/out for automatic Fast-DDS participant recreation
     * on observed network-interface address changes.
     *
     * Fast-DDS binds its transports (UDPv4/v6) to the set of network interfaces present
     * at participant-creation time and does not refresh those bindings when the host
     * later acquires a new address or sees a new link come up. Without recovery,
     * a participant created before its host's primary interface had a DHCP lease
     * will fail to receive messages from off-host peers forever, even after the
     * lease is acquired.
     *
     * When auto-recovery is enabled for a participant, provizio_dds starts a process-wide
     * background monitor (one per process, started lazily) that watches the kernel's
     * address-change notifications (netlink on Linux, PF_ROUTE on macOS,
     * NotifyUnicastIpAddressChange on Windows). Bursts of events are coalesced into a
     * single quiescence-detected reset; a snapshot of the relevant interface→IP set is
     * diffed against the last known state to avoid resets when the change was
     * inconsequential (e.g. a Docker / veth bridge churn whose adapter name is
     * filtered out, or any change that nets out to the same set of routable addresses).
     *
     * Note: IPv6 RFC 4941 temporary / privacy addresses are NOT currently filtered out
     * by their @c IFA_F_* flags — they would only show up here if they were preferred
     * addresses on a non-filtered interface, in which case rotation does cause a
     * snapshot delta. On hosts where this matters, set @c net.ipv6.conf.all.use_tempaddr=0
     * for DDS interfaces, or use the @c off mode (or env var) to opt out.
     *
     * On a confirmed change, every recovery-enabled participant is torn down and
     * rebuilt with its original QoS; publisher / subscriber handles created against it
     * stay valid (their internal Fast-DDS objects are swapped under the user's
     * shared_ptr), and the user-supplied subscriber callbacks remain registered.
     * Every recreation is logged (see @c logging.h to redirect log output).
     */
    enum class network_recovery_mode
    {
        /**
         * @brief Honour the @c PROVIZIO_DDS_NETWORK_RECOVERY environment variable
         * for the whole process. Recognised values (case-insensitive):
         *   - @c on / @c 1 / @c true / @c yes — enable auto-recovery (also the default
         *     when the variable is unset).
         *   - @c off / @c 0 / @c false / @c no — disable auto-recovery.
         * Any other value is treated as the default (enabled) with a one-time warning.
         */
        env_var_controlled,

        /**
         * @brief Force auto-recovery on for this participant, regardless of env var.
         */
        on,

        /**
         * @brief Force auto-recovery off for this participant, regardless of env var.
         * The participant is excluded from the registry; the background monitor still
         * runs if any other participant enables recovery.
         */
        off,
    };

    /**
     * @brief Resolve a @c network_recovery_mode to a concrete on/off decision by
     * consulting the @c PROVIZIO_DDS_NETWORK_RECOVERY env variable when the mode is
     * @c env_var_controlled. Exposed for testing and for callers that want to log the
     * effective decision themselves.
     *
     * @note For @c env_var_controlled mode the env variable is read exactly once
     * (on the first call from this process), and the result is cached for the
     * lifetime of the process. Changing @c PROVIZIO_DDS_NETWORK_RECOVERY at
     * runtime after that first call has no effect — to switch the default,
     * relaunch the process. Tests that need to verify multiple env values must
     * exercise each in a separate process (the CTest harness does this).
     *
     * @param mode Mode as supplied to @c make_domain_participant.
     * @return true if auto-recovery is effectively enabled for this participant.
     */
    PROVIZIO_DDS_API bool resolve_network_recovery_enabled(network_recovery_mode mode);

    /**
     * @brief Force Fast-DDS to re-enumerate the host's network interfaces and
     * repopulate its process-wide interface cache.
     *
     * Fast-DDS keeps a static @c eprosima::SystemInfo cache of network interfaces
     * that is populated exactly once — at the first call to @c SystemInfo::instance()
     * which is invoked deep inside the first @c DomainParticipantFactory::create_participant.
     * Every subsequent locator lookup (in @c UDPv4Transport, @c UDPv6Transport,
     * @c TCPv4Transport, etc.) goes through the cache via @c get_ips(force_lookup=false)
     * and so observes the same fixed interface set for the lifetime of the process.
     *
     * The auto-recovery path (destroy + recreate the @c DomainParticipant) does
     * NOT refresh that cache on its own. Without an explicit refresh, a participant
     * rebuilt after wifi reconnects / DHCP lease arrives / a new interface comes up
     * binds to exactly the same (stale) interfaces the original participant did.
     * The visible effect: discovery announcements stop reaching the network, no
     * publishers ever match, and the only way to recover is a full process
     * restart.
     *
     * Calling this function immediately before every @c create_participant — which
     * @c domain_participant::create_fastdds_participant does internally — closes
     * that gap. It is a thin wrapper around the (otherwise internal)
     * @c eprosima::SystemInfo::update_interfaces, made portable across Linux /
     * macOS (default-visibility ELF / Mach-O exports the symbol already) and
     * Windows (a targeted Fast-DDS ExternalProject patch step,
     * @c cmake/fast_dds/export_system_info.cmake, decorates just this one
     * declaration with @c FASTDDS_EXPORTED_API so it lands in the DLL's
     * export table — preferred over @c CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS,
     * which would overflow the 65535-symbol Windows DLL export ceiling on a
     * library Fast-DDS' size).
     *
     * Thread-safe: @c SystemInfo guards the cache with its own internal mutex.
     * Cost: one @c getifaddrs / @c GetAdaptersAddresses syscall, on the order
     * of microseconds.
     *
     * @return true on success, false if Fast-DDS reported the underlying OS
     *         enumeration failed (in which case the cache is left untouched
     *         and the next locator lookup will continue to use the previous
     *         contents).
     */
    PROVIZIO_DDS_API bool refresh_fastdds_interface_cache() noexcept;
}  // namespace provizio::dds

// C-ABI shim for the Python wrapper. Python's @c provizio_dds module loads
// libprovizio_dds via @c ctypes and calls this symbol before its own direct
// @c factory.create_participant calls — the Python side bypasses the C++
// @c domain_participant::create_fastdds_participant and so needs to drive the
// refresh itself. Kept @c extern "C" so the symbol name is stable and matches
// across compilers without name-mangling differences.
extern "C"
{
    PROVIZIO_DDS_API bool provizio_dds_refresh_fastdds_interface_cache(void);
}

#endif  // DDS_NETWORK_RECOVERY
