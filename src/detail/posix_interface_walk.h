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

#ifndef DDS_DETAIL_POSIX_INTERFACE_WALK
#define DDS_DETAIL_POSIX_INTERFACE_WALK

#if !defined(_WIN32)

#include <arpa/inet.h>
#include <array>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unordered_set>
#include <utility>

#include "netmask_prefix.h"

namespace provizio::dds::detail
{
    /**
     * @file posix_interface_walk.h
     * @brief The @c getifaddrs walk shared by the Linux and macOS address-snapshot
     * backends, for the same reason @c netmask_prefix.h is shared: both derive the set of
     * addresses Fast-DDS could bind from the same call and the same operational filters, and
     * a fix applied to one copy only would make otherwise-identical hosts disagree about
     * which addresses exist at all. Only Linux's extra rtnetlink "kind" lookup differs, and
     * that lives in its own backend, where it is used.
     */

    /// One (interface, address) pair @c getifaddrs reported that survived the operational
    /// filters — the raw material both public functions of a backend then filter by policy.
    struct posix_interface_address
    {
        std::string name;               ///< Kernel-reported device name, e.g. @c "eth0", @c "utun3".
        std::string address_text;       ///< Canonical text form of the address.
        unsigned int prefix_length{0};  ///< CIDR prefix length, 0 when the OS reported no netmask.
        bool is_ipv4{false};            ///< Whether the address is IPv4 rather than IPv6.
    };

    /**
     * @brief Turn a walk's entries into the form Fast-DDS matches an @c interface_blocklist
     * entry by: each excluded interface's device name AND each of its addresses.
     *
     * Both forms, because Fast-DDS compares an entry to @c IPFinder::info_IP's @c dev as
     * well as to its address text. The device name is the stable identity, so it keeps
     * blocking the right interface when its address changes between rebuilds; the address
     * covers a tunnel whose name a platform cannot report usefully.
     *
     * IPv4 only: that is what a UDPv4 transport can be told to block. Shared by the Linux
     * and macOS backends, which differ in one thing only -- whether the platform has a kind
     * to consult -- so the policy and this shape are stated once rather than per platform.
     *
     * @param entries What the platform's walk found.
     * @param is_vpn Whether one entry names a VPN / tunnel interface.
     * @return Device names and address texts to hand Fast-DDS.
     */
    template <typename entries_t, typename is_vpn_fn>
    std::unordered_set<std::string> vpn_blocklist_entries_from(const entries_t &entries, is_vpn_fn is_vpn)
    {
        std::unordered_set<std::string> result;
        for (const auto &entry : entries)
        {
            if (entry.is_ipv4 && is_vpn(entry))
            {
                if (!entry.name.empty())
                {
                    result.insert(entry.name);
                }
                result.insert(entry.address_text);
            }
        }
        return result;
    }

    /**
     * @brief Walk every address @c getifaddrs reports and hand each one that Fast-DDS could
     * bind a locator to to @p on_address.
     *
     * Applies only the OPERATIONAL filters — the ones that decide whether a locator could
     * exist for the address at all:
     *  - a null @c ifa_addr or @c ifa_name (which @c getifaddrs may legitimately report),
     *  - loopback interfaces,
     *  - interfaces without @c IFF_RUNNING, i.e. carrier-down as well as administratively
     *    down. NOT the weaker @c IFF_UP: Fast-DDS' @c IPFinder::getIPs — the only
     *    enumeration behind its UDP locators — filters on exactly this flag, and a snapshot
     *    keying on @c IFF_UP instead would keep a carrier-down interface's address, so a
     *    switch power-cycle or a cable unplug/replug would net out to "nothing changed"
     *    while Fast-DDS had in fact stopped binding a locator to it, and no participant
     *    rebuild would ever be triggered,
     *  - address families other than @c AF_INET / @c AF_INET6,
     *  - IPv6 link-local @c fe80::/10, which is not usable for cross-host DDS and which the
     *    kernel rotates on link-state churn, generating noise.
     *
     * Every POLICY filter is left to the caller, because the two callers in each backend
     * want opposite ones: the snapshot drops container plumbing and tunnels, while the
     * blocklist enumeration wants exactly the tunnels.
     *
     * @param on_address Invoked once per surviving address with a @c posix_interface_address.
     * @return @c true when @c getifaddrs answered, @c false when the OS could not be asked
     * at all. The distinction matters to every caller: no address surviving the filters is a
     * legitimate reading (a container whose only device is a filtered-out veth), so a
     * failure reported as an empty result would read as "every address disappeared" and
     * rebuild every participant for nothing — twice, once on the way out and once when the
     * next successful call restores the set.
     */
    template <typename on_address_fn> bool walk_posix_interface_addresses(on_address_fn on_address)
    {
        ifaddrs *ifa_head = nullptr;
        if (::getifaddrs(&ifa_head) != 0)
        {
            return false;
        }

        // RAII rather than a freeifaddrs at the end: on_address allocates (a snapshot
        // insert, a vector push_back), so a std::bad_alloc from it would otherwise carry
        // the whole ifaddrs list past the free. This runs on every participant creation
        // and every rebuild, which is exactly when a leak per pass compounds -- and it is
        // reached when the process is already short of memory. The Python mirror uses
        // try/finally for the same reason.
        struct ifaddrs_owner
        {
            ifaddrs *head;
            ifaddrs_owner(const ifaddrs_owner &) = delete;
            ifaddrs_owner(ifaddrs_owner &&) = delete;
            ifaddrs_owner &operator=(const ifaddrs_owner &) = delete;
            ifaddrs_owner &operator=(ifaddrs_owner &&) = delete;
            ~ifaddrs_owner()
            {
                ::freeifaddrs(head);
            }
        } const owner{ifa_head};

        for (const ifaddrs *ifa = ifa_head; ifa != nullptr; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == nullptr || ifa->ifa_name == nullptr)
            {
                continue;
            }

            if ((ifa->ifa_flags & IFF_LOOPBACK) != 0 || (ifa->ifa_flags & IFF_RUNNING) == 0)
            {
                continue;
            }

            const int family = ifa->ifa_addr->sa_family;
            if (family != AF_INET && family != AF_INET6)
            {
                continue;
            }

            std::array<char, INET6_ADDRSTRLEN> addr_text{};
            if (family == AF_INET)
            {
                // sockaddr -> sockaddr_in is the canonical BSD-sockets idiom; there is no
                // safe alternative at this layer.
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                const auto *sin = reinterpret_cast<const sockaddr_in *>(ifa->ifa_addr);
                if (::inet_ntop(AF_INET, &sin->sin_addr, addr_text.data(), addr_text.size()) == nullptr)
                {
                    continue;
                }
            }
            else
            {
                // Same canonical BSD-sockets cast for the IPv6 family.
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
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

            on_address(posix_interface_address{std::string{ifa->ifa_name}, std::string{addr_text.data()},
                                               prefix_length_from_netmask(ifa->ifa_netmask), family == AF_INET});
        }

        return true;
    }
}  // namespace provizio::dds::detail

#endif  // !_WIN32

#endif  // DDS_DETAIL_POSIX_INTERFACE_WALK
