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

#include "provizio/dds/detail/address_snapshot.h"
#include "provizio/dds/detail/vpn_interfaces.h"

#if defined(__APPLE__)

#include "detail/posix_interface_walk.h"

#include <array>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace provizio::dds::detail
{
    namespace
    {
        // Apple invents interfaces for AWDL (peer-to-peer Wi-Fi), Apple Wireless
        // Direct, internal proxy NICs, and per-VPN utun*. None are useful for DDS
        // and most rotate addresses far more often than physical NICs would.
        //
        // Prefix-followed-by-digit list: "ap" alone would falsely match user names
        // like "appliance0" or hypothetical vendor adapters; require a digit after
        // the short prefixes so we only catch Apple's enumerated forms (ap0, ap1,
        // ...). Multi-character distinctive prefixes (awdl, utun, bridge, ...) can
        // safely match without a trailing digit since they're unambiguous.
        constexpr std::array<std::string_view, 1> excluded_name_prefixes_digit_suffix{
            "ap",  // Wi-Fi AP mode (host AP) — ap0, ap1, …
        };
        constexpr std::array<std::string_view, 8> excluded_name_prefixes{
            "lo",      // loopback (lo0)
            "awdl",    // Apple Wireless Direct Link
            "llw",     // Low-latency wifi
            "utun",    // user-space tunnel (VPN, IPsec)
            "bridge",  // user bridges (often used by VirtualBox / Parallels)
            "gif",     // generic tunnel
            "stf",     // 6to4 tunnel
            "anpi",    // Apple Wireless internal
        };

        bool starts_with(const std::string &name, std::string_view prefix)
        {
            return name.size() >= prefix.size() && std::string_view{name.data(), prefix.size()} == prefix;
        }

        bool name_excluded(const std::string &name)
        {
            for (const auto &prefix : excluded_name_prefixes)
            {
                if (starts_with(name, prefix))
                {
                    return true;
                }
            }
            for (const auto &prefix : excluded_name_prefixes_digit_suffix)
            {
                // Require at least one digit immediately after the prefix; e.g.
                // "ap" alone or "appliance0" do not match, but "ap0" does.
                if (name.size() > prefix.size() && starts_with(name, prefix) &&
                    static_cast<unsigned char>(name[prefix.size()]) >= '0' &&
                    static_cast<unsigned char>(name[prefix.size()]) <= '9')
                {
                    return true;
                }
            }
            return false;
        }

        // The walk itself lives in detail/posix_interface_walk.h, shared with the Linux
        // backend: same getifaddrs call, same operational filters. macOS has no
        // rtnetlink-style interface "kind", so posix_interface_address IS the whole entry
        // here — the name is the entire identity — and no per-platform wrapper is needed.
        std::vector<posix_interface_address> enumerate_interface_addresses(bool *const enumeration_failed = nullptr)
        {
            std::vector<posix_interface_address> entries;

            const bool readable = walk_posix_interface_addresses(
                [&entries](posix_interface_address address) { entries.push_back(std::move(address)); });

            // Assigned on every call, failure or not: the contract on
            // capture_address_snapshot promises that, and a caller reading a stale true
            // would treat a perfectly readable host as unreadable and stop deciding
            // altogether. An unreadable list is reported rather than returned as an empty
            // one — see walk_posix_interface_addresses for what an empty reading legitimately
            // means.
            if (enumeration_failed != nullptr)
            {
                *enumeration_failed = !readable;
            }

            return entries;
        }
    }  // namespace

    bool snapshot_policy_excludes_interface(const interface_identity &identity)
    {
        // Whether a tunnel may be dropped from the snapshot at all. It may only when the
        // exclusion actually reached the transports: a tunnel this library kept them off
        // cannot move any locator, so its churn must not rebuild anything -- but where the
        // exclusion could not be applied (the caller owns the transports, a
        // participant-level netmask filter of OFF rules a blocklist out, or this host's
        // interfaces could not be read when the participant was configured) DDS binds and
        // announces the tunnel after all, and dropping it here would leave a re-auth or a
        // reconnect with a dead locator that no rebuild replaces. Before the exclusion
        // existed, tunnel interfaces stayed in the snapshot for exactly that reason. The
        // two filters disagreeing
        // about one interface is the one outcome neither may produce.
        const bool may_drop_tunnels = vpn_exclusion_applies_to_transports();

        // A VPN interface is excluded before anything else and regardless of
        // force-inclusion: the transports refuse to bind it (see
        // vpn_interface_blocklist_entries), so its address churn can no longer change any
        // locator. PROVIZIO_DDS_ALLOW_VPN_INTERFACES re-admits it here and in the
        // transports together. macOS reports no interface "kind", so the name carries the
        // whole signal and there is no platform flag to pass.
        if (may_drop_tunnels && excluded_as_vpn_interface(identity.name, /*platform_says_vpn=*/false))
        {
            return true;
        }

        // Reached for a tunnel that stays in the snapshot (the override re-admitted it,
        // or the exclusion never reached the transports), and the name-prefix
        // list below must not then drop it again: "utun" is in that list, so consulting it
        // for an allowed utunN would leave the snapshot ignoring an interface the
        // transports do bind — the one disagreement between the two filters that must
        // never happen, and one no runner without a live tunnel would notice.
        if (is_vpn_interface_name(identity.name))
        {
            return false;
        }

        // A force-included interface skips the name heuristics (see
        // force_included_interfaces) but not the loopback / carrier / link-local checks
        // applied by the walk.
        const auto &force_included = force_included_interfaces();
        if (force_included.find(identity.name) != force_included.end())
        {
            return false;
        }

        return name_excluded(identity.name);
    }

    address_snapshot capture_address_snapshot(bool *const enumeration_failed)
    {
        address_snapshot snapshot;

        for (const auto &entry : enumerate_interface_addresses(enumeration_failed))
        {
            interface_identity identity;
            identity.name = entry.name;
            if (snapshot_policy_excludes_interface(identity))
            {
                continue;
            }

            snapshot.insert({entry.name, entry.address_text, entry.prefix_length});
        }

        return snapshot;
    }

    std::unordered_set<std::string> enumerate_vpn_interface_blocklist_entries(bool *const enumeration_failed)
    {
        // Shared with the Linux backend, which passes a kind where macOS has none: the
        // name is the whole of the signal here.
        return vpn_blocklist_entries_from(
            enumerate_interface_addresses(enumeration_failed),
            [](const posix_interface_address &entry) { return excluded_as_vpn_interface(entry.name, false); });
    }
}  // namespace provizio::dds::detail

#endif  // __APPLE__
