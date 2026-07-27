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
#include "provizio/dds/logging.h"

#if defined(__linux__)

#include "detail/netmask_prefix.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <ifaddrs.h>
#include <linux/if_addr.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace provizio::dds::detail
{
    namespace
    {
        // Name-prefix block-list, applied in addition to IFLA_INFO_KIND. Catches
        // container/Kubernetes interfaces whose IFLA_INFO_KIND is sometimes absent
        // (e.g. when created via `ip link add` without `type` on older kernels) but
        // whose conventional names are well-known noise sources.
        // veth is included alongside the IFLA_INFO_KIND filter as a
        // defensive second layer — older kernels and adapters created via
        // `ip link add` without an explicit `type` may have no kind, and
        // the name convention catches them anyway.
        constexpr std::array<std::string_view, 8> excluded_name_prefixes{
            "docker", "br-", "cni", "kube", "lxc", "flannel", "weave", "veth",
        };

        bool name_excluded_by_prefix(const std::string &name)
        {
            return std::any_of(
                excluded_name_prefixes.begin(), excluded_name_prefixes.end(), [&name](const std::string_view prefix) {
                    return name.size() >= prefix.size() && std::string_view{name.data(), prefix.size()} == prefix;
                });
        }

        bool kind_excluded(const std::string &kind)
        {
            if (kind.empty())
            {
                return false;  // No kind reported → assume physical Ethernet/Wi-Fi.
            }
            // Interface "kinds" reported via IFLA_INFO_KIND that are excluded from snapshots.
            // Physical Ethernet and Wi-Fi have NO IFLA_INFO_KIND attribute, so the "no kind"
            // case is the inclusion default (handled above).
            //
            // Tunnel kinds (tun, ip6tnl) are deliberately absent: a VPN or tunnel endpoint
            // routinely carries real DDS traffic, and unlike container plumbing it is not a
            // churn source (libvirt's per-VM vnetN devices are of kind tun but hold no
            // address of their own, so they never enter the snapshot anyway).
            static const std::unordered_set<std::string_view> excluded_kinds{
                "bridge", "veth", "dummy", "vxlan", "macvlan", "ipvlan",
            };
            return excluded_kinds.find(kind) != excluded_kinds.end();
        }

        // Issues a single RTM_GETLINK dump request and parses IFLA_LINKINFO / IFLA_IFNAME
        // out of every reply. The map returned is keyed by ifindex; missing entries are
        // treated as "no kind" by callers.
        std::unordered_map<int, std::string> fetch_link_kinds()
        {
            std::unordered_map<int, std::string> kinds;

            const int sock = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
            if (sock < 0)
            {
                return kinds;
            }

            // Bound the dump so a stalled/lost kernel reply can't wedge the caller.
            // fetch_link_kinds runs synchronously under the coordinator's
            // registry_mutex during the first participant registration
            // (network_recovery_coordinator.cpp), so an untimed recv would block
            // every other participant. Mirrors the Python _NETLINK_RECV_TIMEOUT_SEC.
            // Best-effort: if SO_RCVTIMEO can't be set we fall through to a blocking
            // recv, no worse than before.
            timeval recv_timeout{};
            recv_timeout.tv_sec = 2;
            recv_timeout.tv_usec = 0;
            (void)::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));

            struct
            {
                nlmsghdr nh;
                ifinfomsg ifi;
            } request{};
            request.nh.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
            request.nh.nlmsg_type = RTM_GETLINK;
            request.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
            request.nh.nlmsg_seq = 1;
            request.ifi.ifi_family = AF_UNSPEC;

            if (::send(sock, &request, request.nh.nlmsg_len, 0) < 0)
            {
                ::close(sock);
                return kinds;
            }

            // Recv buffer large enough for a small fleet of interfaces — netlink will
            // split into multiple datagrams if a single fits doesn't, and we loop.
            constexpr std::size_t recv_buffer_bytes = std::size_t{16} * 1024;
            std::vector<char> buffer(recv_buffer_bytes);
            // Only a dump terminated by a clean NLMSG_DONE is authoritative. A
            // dump cut short by NLMSG_ERROR, a truncated datagram, or a recv
            // failure leaves a partial map that misclassifies virtual
            // interfaces as "no kind" and admits them into the snapshot — the
            // very asymmetry this function exists to prevent. On any such
            // failure we return an empty map so the caller falls back to
            // name-prefix filtering, mirroring the Python implementation.
            bool dump_complete = false;
            bool stop = false;
            while (!stop)
            {
                // MSG_TRUNC: on a netlink socket recv() then returns the real
                // datagram length even when it exceeds the buffer, letting us
                // detect an oversized reply instead of silently dropping the
                // links that didn't fit.
                const ssize_t got = ::recv(sock, buffer.data(), buffer.size(), MSG_TRUNC);
                if (got < 0)
                {
                    // Retry only a genuine signal interruption. SO_RCVTIMEO surfaces
                    // a stalled dump as EAGAIN/EWOULDBLOCK, which must NOT be retried
                    // — looping there would reinstate the unbounded block the timeout
                    // exists to prevent. Bail to name-prefix filtering instead.
                    if (errno == EINTR)
                    {
                        continue;
                    }
                    log_warning() << "address_snapshot: netlink RTM_GETLINK recv failed or timed out: "
                                  << std::system_category().message(errno);
                    break;
                }
                if (got == 0)
                {
                    // A dump is terminated by NLMSG_DONE, not a 0-length
                    // datagram — treat this as incomplete.
                    break;
                }
                if (static_cast<std::size_t>(got) > buffer.size())
                {
                    log_warning() << "address_snapshot: netlink RTM_GETLINK reply truncated "
                                     "(buffer too small); falling back to name-prefix filtering";
                    break;
                }

                // NLMSG_NEXT mutates the remaining-length argument in place, so it
                // must be a non-const lvalue. We use `unsigned int` to avoid the
                // signed/unsigned comparison inside NLMSG_OK on newer kernel
                // headers; `got > 0` was verified above so the cast is safe.
                auto remaining_bytes = static_cast<unsigned int>(got);
                // reinterpret_cast is unavoidable here: NLMSG_OK / NLMSG_NEXT
                // require an `nlmsghdr *` pointing into a raw byte buffer.
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                for (auto *nh = reinterpret_cast<nlmsghdr *>(buffer.data()); NLMSG_OK(nh, remaining_bytes);
                     nh = NLMSG_NEXT(nh, remaining_bytes))
                {
                    if (nh->nlmsg_type == NLMSG_DONE)
                    {
                        dump_complete = true;
                        stop = true;
                        break;
                    }
                    if (nh->nlmsg_type == NLMSG_ERROR)
                    {
                        log_warning() << "address_snapshot: netlink RTM_GETLINK dump returned an error; "
                                         "falling back to name-prefix filtering";
                        stop = true;
                        break;
                    }
                    if (nh->nlmsg_type != RTM_NEWLINK)
                    {
                        continue;
                    }

                    // NLMSG_OK only guarantees room for the nlmsghdr, not the
                    // ifinfomsg that follows it. Skip a short/malformed RTM_NEWLINK
                    // rather than reading ifi_index past the message end (mirrors the
                    // Python check before unpacking the ifinfomsg).
                    if (nh->nlmsg_len < NLMSG_LENGTH(sizeof(ifinfomsg)))
                    {
                        continue;
                    }

                    const auto *ifi = static_cast<const ifinfomsg *>(NLMSG_DATA(nh));
                    std::string kind;

                    // Walk top-level IFLA_* attributes searching for IFLA_LINKINFO,
                    // which contains nested IFLA_INFO_KIND.
                    int remaining = static_cast<int>(IFLA_PAYLOAD(nh));
                    for (const rtattr *rta = IFLA_RTA(ifi); RTA_OK(rta, remaining); rta = RTA_NEXT(rta, remaining))
                    {
                        if (rta->rta_type != IFLA_LINKINFO)
                        {
                            continue;
                        }

                        int nested_remaining = static_cast<int>(RTA_PAYLOAD(rta));
                        for (const auto *nested = static_cast<const rtattr *>(RTA_DATA(rta));
                             RTA_OK(nested, nested_remaining); nested = RTA_NEXT(nested, nested_remaining))
                        {
                            if (nested->rta_type == IFLA_INFO_KIND)
                            {
                                const auto *data = static_cast<const char *>(RTA_DATA(nested));
                                const auto len = static_cast<std::size_t>(RTA_PAYLOAD(nested));
                                kind.assign(data, ::strnlen(data, len));
                                break;
                            }
                        }
                        break;
                    }

                    kinds[ifi->ifi_index] = std::move(kind);
                }
            }

            ::close(sock);
            if (!dump_complete)
            {
                kinds.clear();
            }
            return kinds;
        }
    }  // namespace

    address_snapshot capture_address_snapshot()
    {
        address_snapshot snapshot;

        // First pass: which interfaces are virtual / container by kind?
        const auto kinds_by_index = fetch_link_kinds();
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

            if ((ifa->ifa_flags & IFF_LOOPBACK) != 0)
            {
                continue;
            }
            // IFF_RUNNING (operationally up: administratively up AND carrier present),
            // NOT the weaker IFF_UP. Fast-DDS' IPFinder::getIPs — the only enumeration
            // behind its UDP locators — filters on exactly this flag, and the snapshot's
            // whole job is to model that interface set. Keying on IFF_UP instead would
            // leave a carrier-down interface's address in the snapshot, so a switch
            // power-cycle or cable unplug/replug would net out to "nothing changed"
            // while Fast-DDS had in fact stopped (and later could resume) binding a
            // locator to it — and no participant rebuild would ever be triggered.
            if ((ifa->ifa_flags & IFF_RUNNING) == 0)
            {
                continue;
            }

            const int family = ifa->ifa_addr->sa_family;
            if (family != AF_INET && family != AF_INET6)
            {
                continue;
            }

            const std::string name{ifa->ifa_name};
            // A force-included interface skips the name-prefix and kind heuristics
            // (see force_included_interfaces) but not the loopback / carrier /
            // link-local checks above and below.
            const bool is_force_included = force_included.find(name) != force_included.end();

            if (!is_force_included)
            {
                if (name_excluded_by_prefix(name))
                {
                    continue;
                }

                // if_nametoindex returns 0 on failure; kernel-assigned indices
                // start at 1, so 0 cannot legitimately appear in kinds_by_index.
                // Treat the failure as "no kind known" (kind_excluded("") below
                // is false for an empty string) rather than risk a spurious hit.
                const unsigned int raw_idx = ::if_nametoindex(name.c_str());
                const int idx = raw_idx == 0 ? -1 : static_cast<int>(raw_idx);
                const auto kind_it = kinds_by_index.find(idx);
                // Value (not const&): one arm of the ternary is a prvalue
                // `std::string{}`, so the result is a prvalue regardless. Binding
                // it to a `const std::string&` would technically lifetime-extend
                // the temporary, but the rules are fiddly enough that some
                // compilers (and lint passes) flag it; using a plain value here
                // is the same cost (one move) and obviously correct.
                const std::string kind = (kind_it == kinds_by_index.end()) ? std::string{} : kind_it->second;
                if (kind_excluded(kind))
                {
                    continue;
                }
            }

            std::array<char, INET6_ADDRSTRLEN> addr_text{};
            if (family == AF_INET)
            {
                // sockaddr → sockaddr_in is the canonical BSD-sockets idiom; no
                // safe alternative exists at this layer.
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

                // IPv6 link-local fe80::/10 — not used for cross-host DDS, and the
                // kernel rotates it on link state churn, generating noise.
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

#endif  // __linux__
