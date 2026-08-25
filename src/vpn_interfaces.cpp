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

#include "provizio/dds/detail/vpn_interfaces.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fastdds/utils/IPFinder.hpp>

#include "detail/env_utils.h"

namespace provizio::dds::detail
{
    namespace
    {
        // Distinctive vendor words, matched ANYWHERE in the string. They are long and
        // specific enough that a physical NIC's name or description cannot contain one,
        // which is what makes free substring matching safe here — and necessary, since
        // Windows reports things like "OpenVPN TAP-Windows6" and "Tailscale Tunnel"
        // where the identifying word is not at the start.
        constexpr std::array<std::string_view, 4> vpn_vendor_needles{
            "tailscale",  // Tailscale
            "wireguard",  // WireGuard (incl. the WireGuard NT Windows adapter)
            "openvpn",    // OpenVPN
            "zerotier",   // ZeroTier
        };

        // Generic tunnel conventions, matched only as a PREFIX. Anchoring matters: as
        // free substrings these are short enough to appear inside an unrelated vendor
        // string, and a false positive is the worst outcome this feature can produce —
        // it would drop ALL DDS traffic on a real interface, where a false negative
        // merely leaves the duplicate traffic this change exists to remove. Prefix form
        // still covers every convention in use: tun0, tap0, utun3, ipsec1, wg0, and
        // Windows' own "TAP-Windows Adapter V9" / "Tunnel adapter ...".
        constexpr std::array<std::string_view, 7> vpn_name_prefixes{
            "utun",   // macOS user-space tunnel (VPN, IPsec)
            "ipsec",  // IPsec / strongSwan
            "tun",    // OpenVPN & friends, L3; also "Tunnel adapter ..." on Windows
            "tap",    // OpenVPN & friends, L2; also "TAP-Windows ..." on Windows
            "wg",     // WireGuard device convention
            // The two kernel tunnels macOS names itself. Linux's equivalents (ipip, sit,
            // gre) are caught by their rtnetlink kind instead; macOS reports no kind, so
            // the name is the only signal there is. They must be classified here and not
            // only dropped from the address snapshot: the snapshot drops exactly what the
            // transports refuse to bind, and an interface excluded from one but bound by
            // the other is the disagreement address_snapshot.h says must never happen.
            "gif",  // generic IPv4/IPv6-in-IP tunnel (gif0)
            "stf",  // 6to4 tunnel (stf0)
        };

        // Tunnel forms as Windows' own strings spell them, matched as a PREFIX of a
        // driver-supplied description. Whole words, unlike the device conventions above:
        // that is what lets them be applied to a vendor string at all (see
        // is_vpn_description -- a description beginning "WG111v3" is a NETGEAR radio, not
        // a WireGuard device).
        constexpr std::array<std::string_view, 2> vpn_description_prefixes{
            "tap-windows",     // OpenVPN's TAP driver: "TAP-Windows Adapter V9"
            "tunnel adapter",  // Windows' own naming: "Tunnel adapter Teredo"
        };

        // "zt" is deliberately absent from the needle lists above and handled here instead:
        // as a two-letter substring it would match far too much of a vendor string, so
        // ZeroTier's device convention is matched only as a name PREFIX followed by its
        // node-derived suffix — "zt" plus eight alphanumeric characters (ztppmkbrz2),
        // which is long and specific enough that no physical NIC name collides with it.
        bool is_zerotier_device_name(const std::string &lowered)
        {
            constexpr std::string_view prefix{"zt"};
            constexpr std::string::size_type suffix_length = 8;
            if (lowered.size() != prefix.size() + suffix_length || lowered.compare(0, prefix.size(), prefix) != 0)
            {
                return false;
            }
            return std::all_of(std::next(lowered.begin(), static_cast<std::ptrdiff_t>(prefix.size())), lowered.end(),
                               [](const char chr) {
                                   const auto uchr = static_cast<unsigned char>(chr);
                                   return (uchr >= '0' && uchr <= '9') || (uchr >= 'a' && uchr <= 'z');
                               });
        }

        // Parsed once (see vpn_interface_excluded): the blanket allow flag and the
        // explicitly allowed interface names.
        struct allow_override
        {
            bool allow_all{false};
            std::unordered_set<std::string> names;
        };

        allow_override parse_allow_override_once()
        {
            allow_override override_values;

            // Startup-only probe, same as every other PROVIZIO_DDS_* read.
            const auto *raw = std::getenv(allow_vpn_interfaces_env);  // NOLINT(concurrency-mt-unsafe)
            if (raw == nullptr || *raw == '\0')
            {
                return override_values;
            }

            for (const auto &entry : split_comma_separated(std::string{raw}))
            {
                const std::string lowered = to_lower_ascii(entry);
                if (lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on" || lowered == "all")
                {
                    override_values.allow_all = true;
                }
                else if (lowered == "0" || lowered == "false" || lowered == "off" || lowered == "no")
                {
                    // Documented as leaving the default in place, so they are dropped here
                    // rather than kept as names that happen to match nothing. Keeping them
                    // would have take_unmatched_vpn_allow_override_names report a
                    // deliberately inert setting as if it were a typo, on every host with a
                    // tunnel up -- telling an operator who asked for the default that their
                    // value "matched no VPN / tunnel interface".
                }
                else
                {
                    // Everything that is neither a blanket keyword nor one of the inert
                    // values above is an interface name.
                    //
                    // Stored lower-cased, and compared against a lower-cased name in
                    // excluded_as_vpn_interface: the classifier that decided this was a
                    // tunnel lower-cased too, and on Windows the identity it classified is
                    // the adapter's friendly name or description ("Tailscale", "WireGuard
                    // Tunnel"). Matching the raw strings here would make
                    // PROVIZIO_DDS_ALLOW_VPN_INTERFACES=tailscale classify the adapter as a
                    // VPN and then fail to re-admit it — an escape hatch that silently does
                    // nothing.
                    override_values.names.insert(lowered);
                }
            }

            // Deliberately silent, for the same reason force_included_interfaces() is:
            // this runs inside a function-local static initializer that
            // capture_address_snapshot() reaches while the coordinator holds its
            // registry and monitor mutexes, and a log line there would invoke the
            // user's callback under those locks.
            return override_values;
        }

        // Which PROVIZIO_DDS_ALLOW_VPN_INTERFACES names have actually re-admitted an
        // interface, and whether the mismatch between those and the names given has been
        // reported. Kept apart from allow_override, which is immutable once parsed: this is
        // what the classifier learns as it runs, and it is the only evidence that a name
        // given means anything on this host.
        //
        // Function-local statics rather than namespace-scope objects, for the reason every
        // other piece of state in this file is: the classifier is reachable from a static
        // initializer in another translation unit, and an unordered_set that has not been
        // constructed yet would be read as one that has.
        std::mutex &matched_override_names_mutex()
        {
            static std::mutex mutex;
            return mutex;
        }

        std::unordered_set<std::string> &matched_override_names()
        {
            static std::unordered_set<std::string> names;
            return names;
        }

        std::atomic<bool> &unmatched_override_reported()
        {
            static std::atomic<bool> reported{false};
            return reported;
        }

        const allow_override &allow_override_values()
        {
            // Meyers' singleton: parsed on first use, then immutable, so the transports
            // of every participant (including ones rebuilt on a network change) and the
            // change-detection snapshot all agree for the process' lifetime.
            static const allow_override values = parse_allow_override_once();
            return values;
        }

        // Substitution installed by force_vpn_blocklist_entries_for_test. Unset in every
        // shipped configuration; guarded by a mutex rather than left plain because a test
        // installs it from its own thread while participants read it from theirs.
        std::mutex &forced_blocklist_mutex()
        {
            static std::mutex mutex;
            return mutex;
        }

        std::optional<std::unordered_set<std::string>> &forced_blocklist_entries()
        {
            static std::optional<std::unordered_set<std::string>> entries;
            return entries;
        }

        // Substitution installed by force_allowed_interfaces_for_test, guarded for the
        // same reason as the one above: a test installs it from its own thread while
        // participants read it from theirs.
        std::mutex &forced_interface_count_mutex()
        {
            static std::mutex mutex;
            return mutex;
        }

        /// Whether a test has made the interface enumeration behind vpn_allowed_interfaces
        /// report failure. Separate from forced_interfaces, deliberately: substituting a
        /// list can only ever express success, so without this the failure branch -- and the
        /// caller's bail-out that depends on telling it apart from an empty host -- is
        /// unreachable from a test. Guarded by the same mutex as the substitution it sits
        /// beside, for the same reason: a test installs it from its own thread.
        bool &forced_allowed_enumeration_failure()
        {
            static bool fail = false;
            return fail;
        }

        std::optional<std::vector<allowed_interface>> &forced_interfaces()
        {
            static std::optional<std::vector<allowed_interface>> interfaces;
            return interfaces;
        }

        // Per-thread enumeration cache, active only inside a scoped_vpn_blocklist_cache.
        // depth counts nested scopes so an inner one does not release the outer one's
        // cache; entries is what the enumeration returned for this scope, filled on first
        // use.
        struct blocklist_cache_state
        {
            std::size_t depth{0};
            std::optional<std::unordered_set<std::string>> entries;
        };

        blocklist_cache_state &blocklist_cache()
        {
            thread_local blocklist_cache_state state;
            return state;
        }
        // One-way, process-wide, and deliberately not part of blocklist_cache_state: that
        // is thread-local and released per scope, while this answer belongs to the process
        // and must outlive every scope. Relaxed ordering is enough -- it guards no other
        // state, and a reader that sees the old value once simply keeps watching an
        // interface for one more snapshot.
        std::atomic<bool> &vpn_exclusion_not_applied()
        {
            static std::atomic<bool> not_applied{false};
            return not_applied;
        }
    }  // namespace

    scoped_vpn_blocklist_cache::scoped_vpn_blocklist_cache() noexcept
    {
        ++blocklist_cache().depth;
    }

    scoped_vpn_blocklist_cache::~scoped_vpn_blocklist_cache()
    {
        auto &state = blocklist_cache();
        if (--state.depth == 0)
        {
            // Released with the scope, so the next pass reads the host again: the whole
            // point of re-enumerating per event is that a tunnel can come and go between
            // them.
            state.entries.reset();
        }
    }

    bool is_vpn_product_name(std::string_view name)
    {
        const std::string lowered = to_lower_ascii(name);
        return std::any_of(
            vpn_vendor_needles.begin(), vpn_vendor_needles.end(),
            [&lowered](const std::string_view needle) { return lowered.find(needle) != std::string::npos; });
    }

    void report_vpn_exclusion_not_applied() noexcept
    {
        vpn_exclusion_not_applied().store(true, std::memory_order_relaxed);
    }

    bool vpn_exclusion_applies_to_transports() noexcept
    {
        return !vpn_exclusion_not_applied().load(std::memory_order_relaxed);
    }

    bool is_vpn_description(std::string_view description)
    {
        const std::string lowered = to_lower_ascii(description);
        if (std::any_of(
                vpn_vendor_needles.begin(), vpn_vendor_needles.end(),
                [&lowered](const std::string_view needle) { return lowered.find(needle) != std::string::npos; }))
        {
            return true;
        }
        return std::any_of(
            vpn_description_prefixes.begin(), vpn_description_prefixes.end(),
            [&lowered](const std::string_view prefix) { return lowered.compare(0, prefix.size(), prefix) == 0; });
    }

    bool is_vpn_interface_name(std::string_view name)
    {
        const std::string lowered = to_lower_ascii(name);
        if (is_zerotier_device_name(lowered))
        {
            return true;
        }
        if (std::any_of(
                vpn_vendor_needles.begin(), vpn_vendor_needles.end(),
                [&lowered](const std::string_view needle) { return lowered.find(needle) != std::string::npos; }))
        {
            return true;
        }
        return std::any_of(vpn_name_prefixes.begin(), vpn_name_prefixes.end(),
                           [&lowered](const std::string_view prefix) {
                               return lowered.size() >= prefix.size() && lowered.compare(0, prefix.size(), prefix) == 0;
                           });
    }

#if defined(__linux__)
    bool is_vpn_interface_kind(std::string_view kind)
    {
        if (kind.empty())
        {
            return false;  // Physical Ethernet / Wi-Fi report no kind at all.
        }
        // IFLA_INFO_KIND values that denote an overlay or tunnel device. "tun" also
        // covers libvirt's per-VM vnetN devices, which hold no address of their own and
        // so never reach a locator or a snapshot either way.
        static const std::unordered_set<std::string_view> vpn_kinds{
            "tun", "wireguard", "xfrm", "vti", "vti6", "ipip", "ip6tnl", "gre", "gretap", "ip6gre", "sit",
        };
        return vpn_kinds.find(kind) != vpn_kinds.end();
    }
#endif

    bool excluded_as_vpn_interface(const std::string &name, const bool platform_says_vpn)
    {
        if (!platform_says_vpn && !is_vpn_interface_name(name))
        {
            return false;
        }

        const auto &values = allow_override_values();
        if (values.allow_all)
        {
            return false;
        }

        const std::string lowered = to_lower_ascii(name);
        if (values.names.find(lowered) == values.names.end())
        {
            return true;
        }

        // Recorded because the opposite -- a name that never matches anything -- is
        // otherwise indistinguishable from the variable not being set at all, and that is
        // exactly what a typo produces (see take_unmatched_vpn_allow_override_names).
        // Recorded here rather than at parse time because only the classifier knows the
        // identities this host actually has: the same name means different things on
        // different machines, and on Windows the identity compared is an adapter's
        // friendly name or description rather than a device name.
        {
            const std::lock_guard<std::mutex> lock{matched_override_names_mutex()};
            matched_override_names().insert(lowered);
        }
        return false;
    }

    std::vector<std::string> take_unmatched_vpn_allow_override_names()
    {
        const auto &values = allow_override_values();
        if (values.allow_all || values.names.empty())
        {
            return {};  // Nothing named, or everything allowed: no name can be wrong.
        }

        std::vector<std::string> unmatched;
        {
            const std::lock_guard<std::mutex> lock{matched_override_names_mutex()};
            const auto &matched = matched_override_names();
            for (const auto &name : values.names)
            {
                if (matched.find(name) == matched.end())
                {
                    unmatched.push_back(name);
                }
            }
        }

        if (unmatched.empty())
        {
            return {};
        }

        // The latch is spent only on a call that has something to hand back, so a host
        // where every name matched leaves the report available for whichever participant
        // later finds one that did not.
        if (unmatched_override_reported().exchange(true, std::memory_order_relaxed))
        {
            return {};
        }

        // Sorted so the same host always reports the same line: the names live in an
        // unordered_set.
        std::sort(unmatched.begin(), unmatched.end());
        return unmatched;
    }

    std::vector<allowed_interface> vpn_allowed_interfaces(const std::unordered_set<std::string> &blocked,
                                                          bool *enumeration_failed)
    {
        if (enumeration_failed != nullptr)
        {
            // Assigned on every path, never only on failure: a caller reusing the flag
            // across refreshes would otherwise read the previous call's answer.
            *enumeration_failed = false;
        }

        {
            const std::lock_guard<std::mutex> lock{forced_interface_count_mutex()};
            if (forced_allowed_enumeration_failure())
            {
                // Checked BEFORE the substitution below, so a test can reach the failure
                // branch whether or not it also substitutes a list.
                if (enumeration_failed != nullptr)
                {
                    *enumeration_failed = true;
                }
                return {};
            }

            const auto &forced = forced_interfaces();
            if (forced.has_value())
            {
                return *forced;
            }
        }

        std::vector<eprosima::fastdds::rtps::IPFinder::info_IP> found;
        // Loopback included, matching the get_ipv4s(local_interfaces, true, false) call
        // UDPv4Transport's constructor makes: what this answers is which interfaces THAT
        // will hand a sender socket to, not which ones the host finds interesting.
        if (!eprosima::fastdds::rtps::IPFinder::getIPs(&found, true))
        {
            // Reported rather than returned as an empty vector: the caller cannot tell the
            // two apart otherwise, and it must leave the interface lists alone for one while
            // treating the other as a legitimate reading. See the header.
            if (enumeration_failed != nullptr)
            {
                *enumeration_failed = true;
            }
            return {};
        }

        std::vector<allowed_interface> allowed;
        std::unordered_set<std::string> seen_addresses;
        for (const auto &info : found)
        {
            const bool is_loopback = info.type == eprosima::fastdds::rtps::IPFinder::IP4_LOCAL;
            if (info.type != eprosima::fastdds::rtps::IPFinder::IP4 && !is_loopback)
            {
                // IPv4 only: an interface holding no IPv4 address can neither be blocked by
                // an IPv4 entry nor own a UDPv4 sender socket, so counting it would answer a
                // question about a transport that does not exist.
                continue;
            }
            if (blocked.count(info.dev) != 0 || blocked.count(info.name) != 0)
            {
                continue;
            }
            if (seen_addresses.insert(info.name).second)
            {
                allowed.push_back(allowed_interface{info.name, is_loopback});
            }
        }
        return allowed;
    }

    void force_allowed_interfaces_for_test(std::optional<std::vector<allowed_interface>> interfaces)
    {
        const std::lock_guard<std::mutex> lock{forced_interface_count_mutex()};
        forced_interfaces() = std::move(interfaces);
    }

    void force_allowed_interfaces_enumeration_failure_for_test(const bool fail)
    {
        const std::lock_guard<std::mutex> lock{forced_interface_count_mutex()};
        forced_allowed_enumeration_failure() = fail;
    }

    void force_vpn_blocklist_entries_for_test(std::optional<std::unordered_set<std::string>> entries)
    {
        const std::lock_guard<std::mutex> lock{forced_blocklist_mutex()};
        forced_blocklist_entries() = std::move(entries);
    }

    std::unordered_set<std::string> vpn_interface_blocklist_entries(bool *const enumeration_failed)
    {
        if (enumeration_failed != nullptr)
        {
            *enumeration_failed = false;
        }

        {
            const std::lock_guard<std::mutex> lock{forced_blocklist_mutex()};
            const auto &forced = forced_blocklist_entries();
            if (forced)
            {
                // Checked first, so a test's substitution is what every caller sees
                // whether or not a cache scope happens to be open around it. A substituted
                // enumeration never fails: it is the answer, not a reading of the host.
                return *forced;
            }
        }

        auto &state = blocklist_cache();
        if (state.depth != 0)
        {
            if (!state.entries)
            {
                bool failed = false;
                auto entries = enumerate_vpn_interface_blocklist_entries(&failed);
                if (failed)
                {
                    // Deliberately NOT cached. A cache exists to collapse one network
                    // event's repeated enumerations into one reading of the host -- caching
                    // a failure would instead hand every participant rebuilt for that event
                    // the same wrong answer, "this host has no tunnels", and each of them
                    // would then unblock one that is still up.
                    if (enumeration_failed != nullptr)
                    {
                        *enumeration_failed = true;
                    }
                    return entries;
                }
                state.entries = std::move(entries);
            }
            return *state.entries;
        }

        return enumerate_vpn_interface_blocklist_entries(enumeration_failed);
    }
}  // namespace provizio::dds::detail
