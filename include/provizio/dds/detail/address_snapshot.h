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

#ifndef DDS_DETAIL_ADDRESS_SNAPSHOT
#define DDS_DETAIL_ADDRESS_SNAPSHOT

#include <functional>
#include <string>
#include <unordered_set>

#include "provizio/dds/common.h"

namespace provizio::dds::detail
{
    /**
     * @file address_snapshot.h
     * @brief Process-portable representation of the host's current "DDS-interesting"
     * interface→address set, with platform-specific filters baked in.
     *
     * The snapshot is intentionally narrow: only addresses that a Fast-DDS UDP transport
     * could plausibly bind to AND that signal a meaningful network state change. This
     * excludes loopback, IPv6 link-local @c fe80:: addresses, and interfaces operated by
     * container/hypervisor runtimes (Docker bridges, veth pairs, Hyper-V Virtual Ethernet,
     * WSL adapters, etc.). Those are the noise sources that would otherwise cause spurious
     * participant recreations. See the "Limitation" note in @c capture_address_snapshot for
     * the one exclusion we don't currently apply (IPv6 RFC 4941 temporary / privacy
     * addresses — @c getifaddrs has no per-address flag).
     *
     * The snapshot is computed on demand at the end of each coalesced burst; comparing
     * two snapshots is what gates the actual reset.
     */

    /**
     * @brief Element of an @c address_snapshot.
     */
    struct interface_address
    {
        std::string interface_name;  ///< Kernel-reported name, e.g. "eth0", "wlp3s0".
        std::string address_text;    ///< Canonical text form, e.g. "192.168.1.42" / "2001:db8::1".

        /**
         * @brief Length of the address's subnet prefix in bits, e.g. 24 for a
         * 255.255.255.0 netmask. Part of the snapshot identity because Fast-DDS
         * derives its netmask-based locator filtering from it: re-subnetting an
         * interface without changing its address (192.168.1.10/24 →
         * 192.168.1.10/16) changes which peers Fast-DDS considers on-link, so it
         * must count as a network change. 0 when the OS reported no netmask.
         */
        unsigned int prefix_length{0};

        /**
         * @brief Value equality over all three fields — the identity the snapshot set is
         * built on, so any one of them changing counts as a network change.
         */
        bool operator==(const interface_address &other) const noexcept
        {
            return interface_name == other.interface_name && address_text == other.address_text &&
                   prefix_length == other.prefix_length;
        }
    };

    /**
     * @brief Hash functor for @c interface_address, so it can key an unordered_set.
     * Consistent with @c interface_address::operator== by construction: it mixes exactly
     * the three fields that equality compares.
     */
    struct interface_address_hash
    {
        /// @brief Combined hash of the interface name, address text and prefix length.
        std::size_t operator()(const interface_address &address) const noexcept
        {
            const std::size_t name_hash = std::hash<std::string>{}(address.interface_name);
            const std::size_t addr_hash = std::hash<std::string>{}(address.address_text);
            // Boost-style combine; the snapshot is small (typ. <20 entries) so the only
            // requirement is that the same (name, address, prefix) triple hashes to the
            // same value, which it does — collision resistance is irrelevant at this scale.
            std::size_t combined = name_hash ^ (addr_hash + 0x9e3779b9 + (name_hash << 6) + (name_hash >> 2));
            const std::size_t prefix_hash = std::hash<unsigned int>{}(address.prefix_length);
            combined ^= prefix_hash + 0x9e3779b9 + (combined << 6) + (combined >> 2);
            return combined;
        }
    };

    /**
     * @brief Unordered set of DDS-interesting (interface, address) pairs at one instant.
     * Empty when no interface qualifies (e.g. host has only loopback up).
     *
     * @c std::unordered_set::operator== compares contents order-independently
     * (see C++17 [unord.req]/9), which is exactly the comparison we need to gate
     * a reset on a confirmed change.
     */
    using address_snapshot = std::unordered_set<interface_address, interface_address_hash>;

    /**
     * @brief Capture the current "DDS-interesting" interface→address set.
     *
     * Excluded from the result, by platform:
     *
     * Common to all platforms:
     *  - Loopback interfaces.
     *  - IPv6 link-local @c fe80::/10 addresses (RFC 4291) — not useful for
     *    cross-host DDS.
     *  - Interfaces that are not operationally up, i.e. carrier-down as well as
     *    administratively down (Linux/macOS: @c IFF_RUNNING not set; Windows:
     *    @c OperStatus != @c IfOperStatusUp). This deliberately mirrors
     *    Fast-DDS' own @c IPFinder::getIPs, which also keys on @c IFF_RUNNING:
     *    the snapshot exists to model the interface set Fast-DDS will bind to,
     *    so any flag it filters on must be filtered on here too. Checking the
     *    weaker @c IFF_UP (administrative state) instead would make a
     *    cable-unplug / switch-power-cycle invisible to the diff — the address
     *    stays in the snapshot across the outage while Fast-DDS silently stops
     *    binding a locator to it — and the participant would never be rebuilt.
     *    See @c network_monitor for the matching link-state event subscription
     *    that makes carrier transitions observable in the first place.
     *
     * Linux:
     *  - Interfaces whose @c IFLA_INFO_KIND from rtnetlink matches one of:
     *    @c bridge, @c veth, @c dummy, @c vxlan, @c macvlan, @c ipvlan
     *    (physical Ethernet and Wi-Fi have no kind name and pass through).
     *    Tunnel kinds (@c tun, @c ip6tnl) are deliberately NOT excluded: a
     *    VPN / tunnel endpoint routinely carries real DDS traffic and, unlike
     *    container plumbing, is not a churn source.
     *  - Interfaces whose name starts with one of @c docker, @c br-,
     *    @c cni, @c kube, @c lxc, @c flannel, @c weave, @c veth — the
     *    name-prefix list catches container/Kubernetes/LXC adapters whose
     *    @c IFLA_INFO_KIND may legitimately be absent on older kernels or
     *    when created via @c ip link without a @c type argument. (@c veth
     *    appears in both filters because the @c IFLA_INFO_KIND value
     *    matches the @c veth/* name convention; either filter alone is
     *    sufficient.)
     *
     * Limitation: IPv6 RFC 4941 temporary (privacy) addresses are NOT filtered
     * by their @c IFA_F_* flags — @c getifaddrs returns only interface-level
     * flags, not per-address ones. On a host with default
     * @c net.ipv6.conf.all.use_tempaddr=2 the rotated address WILL produce
     * snapshot deltas. Users can either disable @c use_tempaddr on DDS
     * interfaces or run with @c PROVIZIO_DDS_NETWORK_RECOVERY=off.
     *
     * macOS:
     *  - Interface name prefix in {@c lo, @c awdl, @c llw, @c utun, @c bridge,
     *    @c gif, @c stf, @c anpi, @c ap}.
     *
     * Windows:
     *  - @c IfType not in {@c IF_TYPE_ETHERNET_CSMACD, @c IF_TYPE_IEEE80211,
     *    @c IF_TYPE_PPP}.
     *  - Friendly-name / description substring match for known virtual
     *    adapters: @c "Hyper-V Virtual", @c "WSL", @c "VirtualBox Host-Only",
     *    @c "VMware Virtual", @c "TAP-Windows", @c "Loopback Pseudo-Interface",
     *    @c "Tunnel adapter".
     *
     * @return Snapshot of the interesting (interface, address, prefix) triples. Two
     * equal snapshots produced at different times mean the set of interesting
     * addresses hasn't changed; @c operator== on @c std::unordered_set compares
     * contents order-independently and is the no-op-reset signal.
     */
    PROVIZIO_DDS_API address_snapshot capture_address_snapshot();

    /**
     * @brief Interface names the user has force-included via the
     * @c PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES environment variable
     * (comma-separated, e.g. @c "br0,virbr2"). An interface named here bypasses
     * every name-prefix / kind / adapter-type exclusion listed on
     * @c capture_address_snapshot, but still has to be operationally up, carry a
     * non-link-local address, and not be loopback.
     *
     * The escape hatch exists because the exclusions are heuristics: a host whose
     * primary NIC *is* a bridge (@c br0 on a vehicle PC, @c br-lan on a router-like
     * unit) would otherwise have its only DDS-relevant interface filtered out of
     * the snapshot, and no address change on it would ever trigger a recovery.
     * Broadening the defaults instead is not an option — @c docker0 / @c virbr0 are
     * bridges too, and their churn is exactly the noise the filters exist to drop.
     *
     * @note The variable is read exactly once per process, on the first call.
     * @return Set of interface names; empty when the variable is unset or blank.
     */
    PROVIZIO_DDS_API const std::unordered_set<std::string> &force_included_interfaces();
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_ADDRESS_SNAPSHOT
