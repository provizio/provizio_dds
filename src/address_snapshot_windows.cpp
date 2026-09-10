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

// Symmetric with network_monitor_windows.cpp — pull the Windows SDK
// headers in here so they get processed before address_snapshot.h /
// common.h / Fast-DDS can include them with a different include order.
// _WIN32_WINNT / NTDDI_VERSION / NOMINMAX / WIN32_LEAN_AND_MEAN are
// pinned project-wide via add_compile_definitions in CMakeLists.txt.
#if defined(_WIN32)

#include <winsock2.h>
// clang-format off
#include <ws2tcpip.h>
#include <iphlpapi.h>
// clang-format on
#endif  // _WIN32

#include "provizio/dds/detail/address_snapshot.h"
#include "provizio/dds/detail/vpn_interfaces.h"

#include "detail/snapshot_policy.h"

#if defined(_WIN32)

#pragma comment(lib, "Iphlpapi.lib")
#pragma comment(lib, "Ws2_32.lib")

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace provizio::dds::detail
{
    namespace
    {
        // Friendly-name / description substrings that flag a virtual / container
        // adapter on Windows. Matched case-sensitively against the Windows-localised
        // FriendlyName and Description fields; vendors are stable enough that this
        // works in practice. Extended as new container runtimes are encountered.
        constexpr std::array<std::string_view, 7> excluded_description_substrings{
            "Hyper-V Virtual", "WSL",         "VirtualBox Host-Only",
            "VMware Virtual",  "TAP-Windows", "Loopback Pseudo-Interface",
            "Tunnel adapter",
        };

        bool description_excluded(const std::string &description)
        {
            for (const auto &needle : excluded_description_substrings)
            {
                if (description.find(needle) != std::string::npos)
                {
                    return true;
                }
            }
            return false;
        }

        std::string wide_to_utf8(const wchar_t *wide)
        {
            if (wide == nullptr || *wide == L'\0')
            {
                return {};
            }
            // WideCharToMultiByte with cchWideChar=-1 (NUL-terminated input)
            // returns the byte count INCLUDING the NUL terminator and writes
            // the NUL into the output buffer. Allocate the full `len` bytes,
            // let the API write the NUL, then trim it — sizing the buffer to
            // `len - 1` and asking the API for `len` bytes would write one
            // byte past the std::string's storage (heap corruption).
            const int len = ::WideCharToMultiByte(CP_UTF8, 0, wide, -1, nullptr, 0, nullptr, nullptr);
            if (len <= 1)
            {
                return {};
            }
            std::string out(static_cast<std::size_t>(len), '\0');
            ::WideCharToMultiByte(CP_UTF8, 0, wide, -1, out.data(), len, nullptr, nullptr);
            out.resize(static_cast<std::size_t>(len) - 1);  // drop the NUL std::string already has.
            return out;
        }

        // Fills `buffer` with the IP Helper adapter list, growing it while Windows says
        // it is too small. Shared by capture_address_snapshot() and
        // vpn_interface_blocklist_entries() so the two agree on what the OS reported.
        bool query_adapters(std::vector<std::uint8_t> &buffer)
        {
            ULONG buffer_size = static_cast<ULONG>(buffer.size());
            DWORD result = 0;

            // GetAdaptersAddresses may need a larger buffer; loop while it tells us so.
            for (int attempts = 0; attempts < 3; ++attempts)
            {
                result = ::GetAdaptersAddresses(
                    AF_UNSPEC, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER, nullptr,
                    reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data()), &buffer_size);
                if (result == ERROR_BUFFER_OVERFLOW)
                {
                    buffer.resize(buffer_size);
                    continue;
                }
                break;
            }

            return result == NO_ERROR;
        }

        /// Whether an adapter is a VPN / tunnel endpoint: an IF_TYPE_TUNNEL adapter, one
        /// whose driver-supplied description names a tunnel, or one whose friendly name
        /// names a VPN product.
        ///
        /// Neither string gets the device-name classifier, and for the same reason: both
        /// are prose. A user can rename the friendly name ("WG-LAN" on an Ethernet
        /// adapter), and a description is the driver's own vendor string, where a model
        /// number can begin with the very letters a device convention anchors on -- a
        /// NETGEAR WG-series radio describes itself "WG111v3 54Mbps Wireless USB 2.0
        /// Adapter". Taking either for a tunnel would blocklist a real NIC and every
        /// address on it, the worst outcome this feature can produce. is_vpn_description
        /// still catches what the description exists to catch: OpenVPN's "TAP-Windows
        /// Adapter V9", an IF_TYPE_ETHERNET_CSMACD device that names no vendor.
        bool adapter_is_vpn(const interface_identity &identity)
        {
            return identity.platform_says_tunnel || is_vpn_description(identity.description) ||
                   is_vpn_product_name(identity.friendly_name);
        }

        /// Whether an adapter classified as a VPN endpoint stays excluded — i.e.
        /// PROVIZIO_DDS_ALLOW_VPN_INTERFACES named none of the three identities Windows
        /// reports for it. All three are consulted because any of them can be the one that
        /// classified it and the one a user would name it by: the device name is a GUID
        /// nobody would type, the friendly name is what the OS' own network settings show,
        /// and the driver-supplied description is the only identity for an adapter matched
        /// on it alone ("TAP-Windows Adapter V9" on a NIC called "Ethernet 3"), which no
        /// value of the variable short of the blanket form could otherwise bring back.
        ///
        /// One predicate rather than the same test written out at each site: the snapshot
        /// needs it as-is and the blocklist enumeration needs its negation, and a
        /// hand-negated second copy is how the two would eventually come to disagree about
        /// one adapter — the snapshot ignoring an interface the transports bind, or watching
        /// one they refuse to.
        bool excluded_as_vpn_adapter(const interface_identity &identity)
        {
            if (!adapter_is_vpn(identity))
            {
                return false;
            }

            // All three are evaluated before they are combined, rather than chained with &&.
            // excluded_as_vpn_interface records which PROVIZIO_DDS_ALLOW_VPN_INTERFACES name
            // re-admitted an interface, and take_unmatched_vpn_allow_override_names warns
            // about every name that re-admitted nothing. A && chain stops at the first
            // identity the override re-admits, leaving the other two unrecorded -- so a value
            // naming the SAME adapter by two of its identities, its friendly name and its
            // GUID say, would have the second reported as matching no VPN / tunnel interface
            // on this host, sending an operator to hunt a typo in a setting that is working.
            const bool friendly_name_excluded =
                excluded_as_vpn_interface(identity.friendly_name, /*platform_says_vpn=*/true);
            const bool device_name_excluded = excluded_as_vpn_interface(identity.name, /*platform_says_vpn=*/true);
            const bool description_excluded_as_vpn =
                excluded_as_vpn_interface(identity.description, /*platform_says_vpn=*/true);
            return friendly_name_excluded && device_name_excluded && description_excluded_as_vpn;
        }

        /// One preferred unicast address of an adapter, in the form both walks below want
        /// it: the family they filter on, the text they store, and the prefix length the
        /// snapshot records.
        struct unicast_address
        {
            int family{AF_UNSPEC};          ///< @c AF_INET or @c AF_INET6.
            std::string text;               ///< Canonical text form of the address.
            unsigned int prefix_length{0};  ///< CIDR prefix length, from @c OnLinkPrefixLength.
        };

        /// Hands @p on_address every address of @p adapter that Fast-DDS could bind a
        /// locator to.
        ///
        /// Applies only the filters that decide whether a locator could exist for the
        /// address at all -- a DAD state other than preferred, a null or foreign-family
        /// sockaddr, and IPv6 link-local @c fe80::/10, which is unusable for cross-host DDS
        /// and which the OS rotates on link-state churn. Every POLICY filter is left to the
        /// caller, because the two callers want opposite ones: the snapshot takes both
        /// families, while the blocklist enumeration takes IPv4 alone. Split from the
        /// policies for the same reason the POSIX backends split theirs -- see
        /// walk_posix_interface_addresses -- so a fix to the walk cannot reach one caller
        /// and miss the other.
        template <typename on_address_fn>
        void walk_windows_unicast_addresses(const IP_ADAPTER_ADDRESSES *adapter, on_address_fn on_address)
        {
            for (const auto *uni = adapter->FirstUnicastAddress; uni != nullptr; uni = uni->Next)
            {
                if (uni->DadState != IpDadStatePreferred)
                {
                    continue;  // tentative / deprecated / duplicate
                }

                const auto *sa = uni->Address.lpSockaddr;
                if (sa == nullptr || (sa->sa_family != AF_INET && sa->sa_family != AF_INET6))
                {
                    continue;
                }

                std::array<char, INET6_ADDRSTRLEN> addr_text{};
                if (sa->sa_family == AF_INET)
                {
                    const auto *sin = reinterpret_cast<const sockaddr_in *>(sa);
                    if (::inet_ntop(AF_INET, &sin->sin_addr, addr_text.data(),
                                    static_cast<std::size_t>(addr_text.size())) == nullptr)
                    {
                        continue;
                    }
                }
                else
                {
                    const auto *sin6 = reinterpret_cast<const sockaddr_in6 *>(sa);
                    if (IN6_IS_ADDR_LINKLOCAL(&sin6->sin6_addr))
                    {
                        continue;
                    }
                    if (::inet_ntop(AF_INET6, &sin6->sin6_addr, addr_text.data(),
                                    static_cast<std::size_t>(addr_text.size())) == nullptr)
                    {
                        continue;
                    }
                }

                on_address(unicast_address{static_cast<int>(sa->sa_family), std::string{addr_text.data()},
                                           static_cast<unsigned int>(uni->OnLinkPrefixLength)});
            }
        }

        /// What the policy filters read about one adapter, gathered from the fields
        /// GetAdaptersAddresses reports.
        interface_identity identity_of(const IP_ADAPTER_ADDRESSES *adapter)
        {
            interface_identity identity;
            identity.name = adapter->AdapterName != nullptr ? adapter->AdapterName : "";
            identity.friendly_name = wide_to_utf8(adapter->FriendlyName);
            identity.description = wide_to_utf8(adapter->Description);
            identity.platform_says_tunnel = adapter->IfType == IF_TYPE_TUNNEL;
            identity.platform_says_physical = adapter->IfType == IF_TYPE_ETHERNET_CSMACD ||
                                              adapter->IfType == IF_TYPE_IEEE80211 || adapter->IfType == IF_TYPE_PPP;
            return identity;
        }
    }  // namespace

    bool snapshot_policy_excludes_interface(const interface_identity &identity)
    {
        // The order these four questions are asked in, and why, is stated once in
        // detail/snapshot_policy.h. Windows contributes three identities per adapter and an
        // IfType, and its step 2 covers one gate more than the other platforms': both the
        // adapter-type gate (IF_TYPE_TUNNEL is none of Ethernet / Wi-Fi / PPP) and the
        // description gate, since "Tunnel adapter ..." is exactly what such an adapter is
        // called.
        return snapshot_policy_excludes(
            /*excluded_as_vpn=*/[&] { return excluded_as_vpn_adapter(identity); },
            /*is_vpn=*/[&] { return adapter_is_vpn(identity); },
            /*force_included=*/
            [&] {
                // Either the GUID-ish AdapterName or the friendly name — a user cannot
                // reasonably be expected to know the former.
                const auto &force_included = force_included_interfaces();
                return force_included.count(identity.name) != 0 || force_included.count(identity.friendly_name) != 0;
            },
            /*platform_excludes=*/
            [&] {
                return !identity.platform_says_physical || description_excluded(identity.friendly_name) ||
                       description_excluded(identity.description);
            });
    }

    address_snapshot capture_address_snapshot(bool *const enumeration_failed)
    {
        address_snapshot snapshot;

        // Cleared up front, so the answer is this call's rather than whatever the caller's
        // variable held: the contract on capture_address_snapshot promises an assignment on
        // every call, and a caller reading a stale true would treat a perfectly readable
        // host as unreadable and stop deciding altogether.
        if (enumeration_failed != nullptr)
        {
            *enumeration_failed = false;
        }

        std::vector<std::uint8_t> buffer(16 * 1024);
        if (!query_adapters(buffer))
        {
            // Reported rather than returned as an empty snapshot: an empty snapshot is a
            // legitimate reading, so a caller comparing sets would take this for "every
            // address disappeared" and rebuild every participant for nothing — twice, once
            // on the way out and once when the next successful call restores the set.
            if (enumeration_failed != nullptr)
            {
                *enumeration_failed = true;
            }
            return snapshot;
        }

        for (auto *adapter = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data()); adapter != nullptr;
             adapter = adapter->Next)
        {
            // OperStatus is Windows' operational (carrier-aware) state, the counterpart of
            // POSIX IFF_RUNNING — a disconnected adapter reports IfOperStatusDown and is
            // dropped here, matching what the IP Helper API reports to Fast-DDS.
            if (adapter->OperStatus != IfOperStatusUp)
            {
                continue;
            }

            // Loopback is excluded unconditionally, BEFORE the force-include bypass below
            // — the POSIX backends check IFF_LOOPBACK the same way, and the documented
            // contract for force-including an interface is that it still cannot be
            // loopback. Without this, force-including "Loopback Pseudo-Interface 1" would
            // put 127.0.0.1 in the snapshot, since the only other loopback guards are the
            // IfType whitelist and the description match that the bypass skips.
            if (adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK)
            {
                continue;
            }

            const interface_identity identity = identity_of(adapter);
            if (snapshot_policy_excludes_interface(identity))
            {
                continue;
            }

            walk_windows_unicast_addresses(adapter, [&snapshot, &identity](const unicast_address &address) {
                // Both families: change detection watches whatever the host binds. The walk
                // has already dropped IPv6 link-local, and OnLinkPrefixLength is already the
                // CIDR prefix length, so unlike the POSIX backends there is no netmask to
                // convert.
                snapshot.insert({identity.name.empty() ? identity.friendly_name : identity.name, address.text,
                                 address.prefix_length});
            });
        }

        return snapshot;
    }

    std::unordered_set<std::string> enumerate_vpn_interface_blocklist_entries(bool *const enumeration_failed)
    {
        std::unordered_set<std::string> entries;

        std::vector<std::uint8_t> buffer(16 * 1024);
        const bool readable = query_adapters(buffer);
        if (enumeration_failed != nullptr)
        {
            // Assigned on every path, so a caller that declares the flag without
            // initialising it reads an answer rather than whatever was on the stack.
            *enumeration_failed = !readable;
        }
        if (!readable)
        {
            return entries;
        }

        for (auto *adapter = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data()); adapter != nullptr;
             adapter = adapter->Next)
        {
            // Only what Fast-DDS would bind needs blocking: operationally up, not
            // loopback. Mirrors the operational filters in capture_address_snapshot.
            if (adapter->OperStatus != IfOperStatusUp || adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK)
            {
                continue;
            }

            const interface_identity identity = identity_of(adapter);
            if (!excluded_as_vpn_adapter(identity))
            {
                continue;
            }

            // Addresses first, then the name — and only if the adapter had at least one
            // IPv4 address, for the same reason the POSIX backends require one: the name
            // of an adapter a UDPv4 transport never enumerates could not match anything it
            // binds, while still forcing whitelist mode and netmask filtering on.
            bool has_ipv4 = false;

            walk_windows_unicast_addresses(adapter, [&entries, &has_ipv4](const unicast_address &address) {
                // IPv4 only — see the rationale in the POSIX backends: a UDPv4 transport
                // enumerates no IPv6 interface, so an IPv6 entry could never match while
                // still forcing whitelist mode and netmask filtering on.
                if (address.family != AF_INET)
                {
                    return;
                }

                entries.insert(address.text);
                has_ipv4 = true;
            });

            // The GUID-ish AdapterName is what Fast-DDS compares a blocklist entry to as
            // IPFinder::info_IP::dev, and unlike an address it cannot later belong to a
            // real interface — see vpn_interface_blocklist_entries in the header. Falls
            // back to the friendly name when Windows reported no AdapterName, matching
            // _vpn_blocklist_entries_windows in python/network_recovery.py.
            if (has_ipv4)
            {
                const std::string &device_name = identity.name.empty() ? identity.friendly_name : identity.name;
                if (!device_name.empty())
                {
                    entries.insert(device_name);
                }
            }
        }

        return entries;
    }
}  // namespace provizio::dds::detail

#endif  // _WIN32
