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
#include "provizio/dds/logging.h"

#if defined(__linux__)

#include "detail/posix_interface_walk.h"

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
            // Tunnel kinds are deliberately absent HERE: they are handled by the dedicated
            // VPN filter (excluded_as_vpn_interface), which the
            // PROVIZIO_DDS_ALLOW_VPN_INTERFACES override flows through, so that one variable
            // governs both change detection and the transports.
            static const std::unordered_set<std::string_view> excluded_kinds{
                "bridge", "veth", "dummy", "vxlan", "macvlan", "ipvlan",
            };
            return excluded_kinds.find(kind) != excluded_kinds.end();
        }

        /// Closes a file descriptor however the scope it was opened in ends. Only the
        /// netlink socket below needs it, so it stays here rather than becoming a utility:
        /// every other descriptor in this file is opened and closed by getifaddrs.
        class owned_descriptor final
        {
          public:
            explicit owned_descriptor(const int descriptor) noexcept : descriptor{descriptor}
            {
            }
            ~owned_descriptor()
            {
                if (descriptor >= 0)
                {
                    ::close(descriptor);
                }
            }
            owned_descriptor(const owned_descriptor &) = delete;
            owned_descriptor &operator=(const owned_descriptor &) = delete;
            owned_descriptor(owned_descriptor &&) = delete;
            owned_descriptor &operator=(owned_descriptor &&) = delete;

            [[nodiscard]] int get() const noexcept
            {
                return descriptor;
            }

          private:
            int descriptor;
        };

        // What every exit that could not produce an authoritative kind map returns, so the
        // report cannot be missed by a branch added later -- and so the fallback is never
        // silent. Empty rather than partial for the reason spelled out at the recv loop: a
        // half-parsed dump misclassifies virtual interfaces as "no kind" instead of simply
        // leaving them to the name prefixes.
        //
        // Reported rather than logged here: this runs under the participant's lifecycle
        // locks (see the SO_RCVTIMEO comment below), where invoking the caller's log
        // callback could deadlock. refresh_vpn_interface_blocklist takes the report and
        // stashes the line with its own.
        std::unordered_map<int, std::string> no_link_kinds()
        {
            report_interface_kind_lookup_failed();
            return {};
        }

        // Issues a single RTM_GETLINK dump request and parses IFLA_LINKINFO / IFLA_IFNAME
        // out of every reply. The map returned is keyed by ifindex; missing entries are
        // treated as "no kind" by callers.
        std::unordered_map<int, std::string> fetch_link_kinds()
        {
            std::unordered_map<int, std::string> kinds;

            // Checked before the socket, so a test can drive the whole route a real failure
            // takes -- funnel, process-wide report, and the participant warning that reads it
            // -- rather than only the flag's own semantics. Never set in a shipped
            // configuration.
            if (link_kind_lookup_forced_to_fail())
            {
                return no_link_kinds();
            }

            // Owned by a guard rather than closed on each exit path: the parse loop below
            // allocates (the kind strings and the map holding them), so a bad_alloc from it
            // would leave the descriptor behind — and this runs on every participant creation
            // and every rebuild, so the leak would be unbounded exactly when the process is
            // already short of memory.
            const owned_descriptor sock_guard{::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE)};
            const int sock = sock_guard.get();
            if (sock < 0)
            {
                return no_link_kinds();
            }

            // Bound the dump so a stalled/lost kernel reply can't wedge the caller.
            // fetch_link_kinds runs synchronously under the coordinator's
            // registry_mutex during participant registration, and on every participant
            // creation via vpn_interface_blocklist_entries
            // (network_recovery_coordinator.cpp), so an untimed recv would block
            // every other participant. Mirrors the Python _NETLINK_RECV_TIMEOUT_SEC.
            // Best-effort: if SO_RCVTIMEO can't be set we fall through to a blocking
            // recv, no worse than before.
            timeval recv_timeout{};
            recv_timeout.tv_sec = 2;
            recv_timeout.tv_usec = 0;
            (void)::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));

            // Bound explicitly so this socket's port id is known (getsockname below),
            // which is what lets every reply be checked against THIS request from THIS
            // socket. Note what the bind does not buy: nl_pid = 0 asks for the same
            // netlink_autobind() the implicit bind performs, so the port id is no less
            // guessable than before — and rtnetlink is registered without
            // NL_CFG_F_NONROOT_SEND, so an unprivileged process cannot unicast to another
            // socket's port id at all (verified: sendto() to a non-zero port id returns
            // EPERM). The checks are therefore about correctness rather than defence: a
            // reply that answers someone else's dump, or that did not come from the
            // kernel, must not be parsed as if it described this host's links — and a
            // CAP_NET_ADMIN process that could forge one could simply create a real
            // wireguard device instead.
            sockaddr_nl local{};
            local.nl_family = AF_NETLINK;
            if (::bind(sock, reinterpret_cast<const sockaddr *>(&local),  // NOLINT: sockets idiom
                       sizeof(local)) < 0)
            {
                return no_link_kinds();
            }
            sockaddr_nl bound{};
            socklen_t bound_len = sizeof(bound);
            if (::getsockname(sock, reinterpret_cast<sockaddr *>(&bound), &bound_len) < 0)  // NOLINT: sockets idiom
            {
                return no_link_kinds();
            }

            constexpr std::uint32_t request_sequence = 1;
            struct
            {
                nlmsghdr nh;
                ifinfomsg ifi;
            } request{};
            request.nh.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
            request.nh.nlmsg_type = RTM_GETLINK;
            request.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
            request.nh.nlmsg_seq = request_sequence;
            request.ifi.ifi_family = AF_UNSPEC;

            if (::send(sock, &request, request.nh.nlmsg_len, 0) < 0)
            {
                return no_link_kinds();
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
            // Bound on datagrams discarded as not-from-the-kernel. SO_RCVTIMEO above only
            // fires while the socket is IDLE, so a local process writing to this socket's
            // netlink port keeps recvfrom returning data and the discard below would spin
            // forever — wedging the whole process, since fetch_link_kinds runs
            // synchronously under the coordinator's registry_mutex during the first
            // participant registration. A real dump is a handful of datagrams, so any host
            // reaches NLMSG_DONE long before this; exceeding it means someone else is
            // filling the socket, and bailing out degrades to name-prefix filtering exactly
            // as every other failure here does. Mirrors the Python _MAX_FOREIGN_NETLINK_DATAGRAMS.
            constexpr int max_foreign_datagrams = 64;
            int foreign_datagrams = 0;
            while (!stop)
            {
                // MSG_TRUNC: on a netlink socket recv() then returns the real
                // datagram length even when it exceeds the buffer, letting us
                // detect an oversized reply instead of silently dropping the
                // links that didn't fit.
                sockaddr_nl from{};
                socklen_t from_len = sizeof(from);
                const ssize_t got = ::recvfrom(sock, buffer.data(), buffer.size(), MSG_TRUNC,
                                               reinterpret_cast<sockaddr *>(&from),  // NOLINT: sockets idiom
                                               &from_len);
                // Checked before the length handling below, so a zero-length datagram
                // cannot reach the dump-terminated-early path without having passed it.
                // Only the kernel (port id 0) may answer; anything else is discarded
                // rather than parsed — see the bind() above.
                if (got >= 0 && (from_len != sizeof(from) || from.nl_pid != 0))
                {
                    if (++foreign_datagrams > max_foreign_datagrams)
                    {
                        // Silent: falls back to name-prefix filtering, which is correct.
                        break;
                    }
                    continue;
                }
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
                    // Silent: the interface-kind lookup is an optimisation over name-prefix
                    // filtering, and the fallback below produces a correct snapshot without it.
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
                    // Silent: falls back to name-prefix filtering, which is correct.
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
                    // Belongs to THIS request from THIS socket. With the source-port
                    // check on recvfrom above, this is what keeps a reply that answers
                    // someone else's dump — or that is addressed to another socket's port
                    // id — from being parsed as if it described this host's links.
                    if (nh->nlmsg_seq != request_sequence || nh->nlmsg_pid != bound.nl_pid)
                    {
                        continue;
                    }
                    if (nh->nlmsg_type == NLMSG_DONE)
                    {
                        dump_complete = true;
                        stop = true;
                        break;
                    }
                    if (nh->nlmsg_type == NLMSG_ERROR)
                    {
                        // Silent: falls back to name-prefix filtering, which is correct.
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

            if (!dump_complete)
            {
                return no_link_kinds();
            }
            return kinds;
        }
    }  // namespace

    namespace
    {
        /// One (interface, address) pair the kernel reported, with the interface's
        /// rtnetlink kind attached — the raw material both public functions filter.
        struct interface_entry
        {
            std::string name;
            std::string kind;  ///< IFLA_INFO_KIND, empty for physical Ethernet / Wi-Fi.
            std::string address_text;
            unsigned int prefix_length{0};
            bool is_ipv4{false};
        };

        // The getifaddrs walk itself — the operational filters, shared with the macOS
        // backend (see detail/posix_interface_walk.h) — plus the one thing only Linux has:
        // each interface's rtnetlink IFLA_INFO_KIND, which is what identifies a tunnel whose
        // name follows no convention. Every POLICY filter is left to the callers, which want
        // different ones: the snapshot drops container plumbing and tunnels, while the
        // blocklist wants exactly the tunnels.
        std::vector<interface_entry> enumerate_interface_addresses(bool *const enumeration_failed = nullptr)
        {
            std::vector<interface_entry> entries;

            const auto kinds_by_index = fetch_link_kinds();

            const bool readable =
                walk_posix_interface_addresses([&entries, &kinds_by_index](posix_interface_address address) {
                    // if_nametoindex returns 0 on failure; kernel-assigned indices start at 1,
                    // so 0 cannot legitimately appear in kinds_by_index. Treat the failure as
                    // "no kind known" rather than risk a spurious hit.
                    const unsigned int raw_idx = ::if_nametoindex(address.name.c_str());
                    const int idx = raw_idx == 0 ? -1 : static_cast<int>(raw_idx);
                    const auto kind_it = kinds_by_index.find(idx);
                    std::string kind = (kind_it == kinds_by_index.end()) ? std::string{} : kind_it->second;

                    entries.push_back(interface_entry{std::move(address.name), std::move(kind),
                                                      std::move(address.address_text), address.prefix_length,
                                                      address.is_ipv4});
                });

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

        const bool platform_says_vpn = is_vpn_interface_kind(identity.kind);

        // A VPN interface is excluded before anything else and regardless of
        // force-inclusion: the transports refuse to bind it (see
        // vpn_interface_blocklist_entries), so its address churn — a Tailscale re-auth, a
        // tunnel reconnect — can no longer change any locator, and rebuilding every
        // participant over it would be pure disruption. PROVIZIO_DDS_ALLOW_VPN_INTERFACES
        // re-admits it here and in the transports together.
        if (may_drop_tunnels && excluded_as_vpn_interface(identity.name, platform_says_vpn))
        {
            return true;
        }

        // Reached for a tunnel that stays in the snapshot -- the override re-admitted it,
        // or the exclusion never reached the transports -- and such an interface must not
        // then be dropped by the heuristics below: DDS binds it either way, so change
        // detection has to watch it.
        if (platform_says_vpn || is_vpn_interface_name(identity.name))
        {
            return false;
        }

        // A force-included interface skips the name-prefix and kind heuristics
        // (see force_included_interfaces) but not the loopback / carrier /
        // link-local checks applied by the walk.
        const auto &force_included = force_included_interfaces();
        if (force_included.find(identity.name) != force_included.end())
        {
            return false;
        }

        return name_excluded_by_prefix(identity.name) || kind_excluded(identity.kind);
    }

    address_snapshot capture_address_snapshot(bool *const enumeration_failed)
    {
        address_snapshot snapshot;

        for (const auto &entry : enumerate_interface_addresses(enumeration_failed))
        {
            if (snapshot_policy_excludes_interface(interface_identity{entry.name, "", "", entry.kind, false, false}))
            {
                continue;
            }

            snapshot.insert({entry.name, entry.address_text, entry.prefix_length});
        }

        return snapshot;
    }

    std::unordered_set<std::string> enumerate_vpn_interface_blocklist_entries(bool *const enumeration_failed)
    {
        // The filtering and the two entry forms are vpn_blocklist_entries_from's, shared
        // with the macOS backend: they read nothing platform-specific, and stating them
        // twice is how the two would come to disagree about what a blocklist entry is.
        // Linux contributes the one thing macOS has not got -- the rtnetlink kind, which
        // catches a tunnel whose device someone renamed.
        return vpn_blocklist_entries_from(
            enumerate_interface_addresses(enumeration_failed), [](const interface_entry &entry) {
                return excluded_as_vpn_interface(entry.name, is_vpn_interface_kind(entry.kind));
            });
    }
}  // namespace provizio::dds::detail

#endif  // __linux__
