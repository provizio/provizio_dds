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

        bool operator==(const interface_address &other) const noexcept
        {
            return interface_name == other.interface_name && address_text == other.address_text;
        }
    };

    struct interface_address_hash
    {
        std::size_t operator()(const interface_address &address) const noexcept
        {
            const std::size_t name_hash = std::hash<std::string>{}(address.interface_name);
            const std::size_t addr_hash = std::hash<std::string>{}(address.address_text);
            // Boost-style combine; the snapshot is small (typ. <20 entries) so the only
            // requirement is that the same (name, address) pair hashes to the same value,
            // which it does — collision resistance is irrelevant at this scale.
            return name_hash ^ (addr_hash + 0x9e3779b9 + (name_hash << 6) + (name_hash >> 2));
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
     *  - Interfaces that are administratively down (Linux/macOS: @c IFF_UP
     *    not set; Windows: @c OperStatus != @c IfOperStatusUp). Carrier
     *    state (cable-unplugged but interface still UP) is NOT checked
     *    today — interfaces in that state will be included in the snapshot;
     *    transitions to / from carrier-up may then produce snapshot deltas
     *    via DHCP address churn, which is handled by the normal reset path.
     *
     * Linux:
     *  - Interfaces whose @c IFLA_INFO_KIND from rtnetlink matches one of:
     *    @c bridge, @c veth, @c dummy, @c vxlan, @c macvlan, @c ipvlan,
     *    @c ip6tnl, @c tun (best-effort: @c tun covers TAP/TUN VPN endpoints;
     *    physical Ethernet and Wi-Fi have no kind name and pass through).
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
     * @return Snapshot of the interesting (interface, address) pairs. Two equal
     * snapshots produced at different times mean the set of interesting addresses
     * hasn't changed; @c operator== on @c std::unordered_set compares contents
     * order-independently and is the no-op-reset signal.
     */
    PROVIZIO_DDS_API address_snapshot capture_address_snapshot();
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_ADDRESS_SNAPSHOT
