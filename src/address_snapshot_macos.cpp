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

#if defined(__APPLE__)

#include "detail/netmask_prefix.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/socket.h>

#include <array>
#include <string>
#include <string_view>

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
    }  // namespace

    address_snapshot capture_address_snapshot()
    {
        address_snapshot snapshot;

        // Hoisted: one lookup of the (immutable) force-include set for the whole walk.
        const auto &force_included = force_included_interfaces();

        ifaddrs *ifa_head = nullptr;
        if (::getifaddrs(&ifa_head) != 0)
        {
            return snapshot;
        }

        for (const ifaddrs *ifa = ifa_head; ifa != nullptr; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == nullptr || ifa->ifa_name == nullptr)
            {
                continue;
            }
            // IFF_RUNNING (operationally up: administratively up AND carrier present),
            // NOT the weaker IFF_UP — see the rationale on capture_address_snapshot in
            // detail/address_snapshot.h: Fast-DDS' IPFinder::getIPs keys on IFF_RUNNING,
            // so a snapshot that keys on IFF_UP would treat a carrier outage as "nothing
            // changed" and never rebuild the participant.
            if ((ifa->ifa_flags & IFF_LOOPBACK) != 0 || (ifa->ifa_flags & IFF_RUNNING) == 0)
            {
                continue;
            }

            const int family = ifa->ifa_addr->sa_family;
            if (family != AF_INET && family != AF_INET6)
            {
                continue;
            }

            const std::string name{ifa->ifa_name};
            // A force-included interface skips the name heuristics (see
            // force_included_interfaces) but not the loopback / carrier / link-local checks.
            if (force_included.find(name) == force_included.end() && name_excluded(name))
            {
                continue;
            }

            std::array<char, INET6_ADDRSTRLEN> addr_text{};
            if (family == AF_INET)
            {
                const auto *sin = reinterpret_cast<const sockaddr_in *>(ifa->ifa_addr);
                if (::inet_ntop(AF_INET, &sin->sin_addr, addr_text.data(), addr_text.size()) == nullptr)
                {
                    continue;
                }
            }
            else
            {
                const auto *sin6 = reinterpret_cast<const sockaddr_in6 *>(ifa->ifa_addr);
                if (IN6_IS_ADDR_LINKLOCAL(&sin6->sin6_addr))
                {
                    continue;
                }
                if (::inet_ntop(AF_INET6, &sin6->sin6_addr, addr_text.data(), addr_text.size()) == nullptr)
                {
                    continue;
                }
            }

            snapshot.insert({name, std::string{addr_text.data()}, prefix_length_from_netmask(ifa->ifa_netmask)});
        }

        ::freeifaddrs(ifa_head);
        return snapshot;
    }
}  // namespace provizio::dds::detail

#endif  // __APPLE__
