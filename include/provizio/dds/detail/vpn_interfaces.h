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

#ifndef DDS_DETAIL_VPN_INTERFACES
#define DDS_DETAIL_VPN_INTERFACES

#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "provizio/dds/common.h"

/**
 * @file vpn_interfaces.h
 * @brief Identification of VPN / overlay-tunnel network interfaces, which are kept
 * out of the DDS transports by default.
 *
 * Why this exists: Fast-DDS binds its UDP transports to every operationally-up
 * interface the OS reports (@c IPFinder::getIPs) and announces an address on each
 * one as both a metatraffic and a user-data locator. A remote reader's writer then
 * sends every sample to ALL of that reader's announced unicast locators
 * (@c UDPTransportInterface::select_locators), so two hosts that share a LAN and
 * are BOTH on a VPN exchange every sample twice: once over the LAN and once through
 * the tunnel. On a vehicle whose tunnel egress is a metered cellular uplink, that
 * duplicate is charged for and can saturate the link.
 *
 * A VPN interface can never be the path that *establishes* DDS communication: SPDP
 * discovery is link-local multicast and no VPN carries multicast, so a tunnel only
 * ever duplicates a peer that was already discovered on a real network. That is why
 * excluding tunnels is the default rather than an opt-in, and why the escape hatch
 * (@c PROVIZIO_DDS_ALLOW_VPN_INTERFACES) also requires unicast discovery to be
 * configured by whoever uses it.
 *
 * Classification is by interface identity, never by address range. 100.64.0.0/10
 * would catch Tailscale, but it is the RFC 6598 carrier-NAT range: a cellular or
 * ISP-CGNAT uplink legitimately holds an address in it, and so, therefore, can a
 * peer's real LAN interface behind such a router. Blocking the range would break
 * exactly the deployments this feature is meant to protect.
 */

namespace provizio::dds::detail
{
    /// @brief Name of the environment variable that re-admits VPN interfaces; see
    /// @c excluded_as_vpn_interface for the accepted values.
    constexpr const char *allow_vpn_interfaces_env = "PROVIZIO_DDS_ALLOW_VPN_INTERFACES";

    /**
     * @brief Whether an interface name looks like a VPN / overlay tunnel endpoint.
     *
     * Matched against the name the platform reports: the device name on POSIX
     * (@c tailscale0, @c wg0, @c utun3), and the adapter's friendly name or
     * description on Windows, where the device name is a GUID and carries no hint
     * (@c "Tailscale", @c "WireGuard Tunnel").
     *
     * Deliberately NOT matched: @c ppp (a PPP link is a real WAN uplink — cellular
     * modems and DSL — not an overlay), bridges (@c br0 / @c br-lan can be the only
     * interface a vehicle PC carries DDS on) and container plumbing (@c docker0,
     * @c veth*), whose traffic is genuinely local and free.
     *
     * @param name Interface name / friendly name / description; case-insensitive.
     * @return true when the name matches a known VPN or tunnel convention.
     */
    PROVIZIO_DDS_API bool is_vpn_interface_name(std::string_view name);

    /**
     * @brief Whether a string names a VPN *product* — the vendor half of
     * @c is_vpn_interface_name, without the generic @c tun / @c tap / @c wg prefixes.
     *
     * For strings a user can rename, which on Windows means the adapter's friendly
     * name. Renaming a real NIC to something starting with one of those prefixes
     * ("WG-LAN") would otherwise drop it from the transports and take all of that
     * host's DDS traffic with it — the worst outcome this feature can produce, and one
     * a driver-supplied description or a POSIX device name cannot be talked into.
     *
     * @param name Friendly name or other user-editable label; case-insensitive.
     * @return true when the name contains a known VPN vendor's name.
     */
    PROVIZIO_DDS_API bool is_vpn_product_name(std::string_view name);

    /**
     * @brief Whether a driver-supplied adapter DESCRIPTION names a tunnel.
     *
     * A third question, between the other two, and Windows' alone: a description is a
     * vendor string rather than a device name, so the short conventions
     * @c is_vpn_interface_name anchors — @c tun, @c tap, @c wg — match real hardware on
     * it. NETGEAR's WG series is the example that matters: a description reading
     * "WG111v3 54Mbps Wireless USB 2.0 Adapter" begins with @c wg, and taking it for a
     * tunnel would blocklist that host's only NIC and every address on it.
     *
     * What is matched instead: a VPN vendor's name anywhere in the string, as in
     * @c is_vpn_product_name, plus the two forms Windows itself uses to describe a
     * tunnel — "TAP-Windows Adapter V9" (OpenVPN's driver, an @c IF_TYPE_ETHERNET_CSMACD
     * device naming no vendor, which is why the description has to be read at all) and
     * "Tunnel adapter ...". Both are whole words no model number begins with.
     *
     * @param description Driver-supplied adapter description; case-insensitive.
     * @return true when the description names a known VPN product or a Windows tunnel.
     */
    PROVIZIO_DDS_API bool is_vpn_description(std::string_view description);

#if defined(__linux__)
    /**
     * @brief Whether an rtnetlink @c IFLA_INFO_KIND identifies a VPN / tunnel device.
     *
     * The kind is the reliable half of the signal on Linux: it survives renaming (a
     * @c wireguard device called @c office0) and covers tunnel types that follow no
     * naming convention at all (@c xfrm, @c vti). Physical Ethernet and Wi-Fi report
     * no kind, which is why an empty kind is never a match.
     *
     * @param kind Value of @c IFLA_INFO_KIND, empty when the kernel reported none.
     * @return true when the kind is a tunnel/overlay kind.
     */
    PROVIZIO_DDS_API bool is_vpn_interface_kind(std::string_view kind);
#endif

    /**
     * @brief Whether an interface must be kept out of the DDS transports (and out of
     * network-recovery change detection) as a VPN endpoint: it is classified as one
     * and the user has not allowed it back via @c PROVIZIO_DDS_ALLOW_VPN_INTERFACES.
     *
     * The variable takes either a blanket form (@c 1 / @c true / @c yes / @c on /
     * @c all — every VPN interface is used exactly as before this feature existed) or
     * a comma-separated interface-name list (@c "tailscale0,wg0" — only those are
     * used, every other tunnel stays excluded). Anything else, including @c 0 /
     * @c false / @c off and an empty value, leaves the default in place. It is the
     * only knob that re-admits a tunnel: naming one in
     * @c PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES does not, because that would
     * put an interface back into change detection whose addresses the transports
     * still refuse to bind.
     *
     * @note The variable is read exactly once per process, on the first call — so the
     * transports, the change-detection snapshot and every rebuilt participant agree
     * for the process' lifetime.
     *
     * @note Names are matched case-insensitively, on the same terms the classifier
     * used: it lower-cases before matching, and on Windows the identity being
     * classified is the adapter's friendly name or description (@c "Tailscale",
     * @c "WireGuard Tunnel"), whose capitalisation nobody should have to reproduce
     * exactly for the escape hatch to work.
     *
     * @note On Windows the variable is matched against every identity the adapter has —
     * friendly name, GUID-style device name AND driver-supplied description — because any
     * of them can be the one that classified it, and an adapter matched on its description
     * alone (@c "TAP-Windows Adapter V9" on a NIC called @c "Ethernet 3") has no other
     * identity to name it by. See @c excluded_as_vpn_adapter in
     * src/address_snapshot_windows.cpp, which is where that decision is made.
     *
     * @param name Interface name as reported by the platform.
     * @param platform_says_vpn Platform-specific signal that this is a tunnel device
     * (Linux: @c is_vpn_interface_kind of its @c IFLA_INFO_KIND; Windows: an
     * @c IF_TYPE_TUNNEL adapter). Pass false where the platform offers no such signal.
     * @return true when the interface must be excluded.
     */
    PROVIZIO_DDS_API bool excluded_as_vpn_interface(const std::string &name, bool platform_says_vpn);

    /**
     * @brief Whether an address text names the loopback interface: anything in
     * 127.0.0.0/8, or @c ::1.
     *
     * The whole /8, not just 127.0.0.1: an alias such as 127.0.1.1 or a service address on
     * @c lo can reach no other host either, so for @c vpn_allowed_interfaces it needs the
     * same netmask filter as 127.0.0.1 and must not count as a real interface. This is
     * deliberately NOT Fast-DDS' @c IPFinder::IP4_LOCAL, which is an exact 127.0.0.1
     * compare. Mirrors @c _is_loopback_address in python/network_recovery.py, and the two
     * are held to the same verdicts by test/python/vpn_classifier_parity_test.py.
     */
    PROVIZIO_DDS_API bool is_loopback_address(std::string_view address);

    /**
     * @brief The @c PROVIZIO_DDS_ALLOW_VPN_INTERFACES names that have re-admitted nothing,
     * lower-cased and sorted; empty when every name given has matched an interface this
     * process classified as a tunnel, when the variable names none, or when it was set to a
     * blanket keyword.
     *
     * Exists because such a name is silent otherwise. @c PROVIZIO_DDS_ALLOW_VPN_INTERFACES
     * @c =tailscale on a host whose device is @c tailscale0 -- or a Windows adapter named
     * under its device identity rather than the friendly name the classifier saw -- leaves
     * the tunnel excluded for the life of the process, exactly as if the variable had never
     * been set, and the deployment that needed DDS over the tunnel silently does not get it.
     *
     * @note "Matched" means the name re-admitted an interface that WAS classified as a
     * tunnel, which is the only thing the variable can do. A name that matches a real but
     * ordinary interface counts as unmatched -- naming it changes nothing either way.
     *
     * @note Takes: the names are handed back at most once per process, so a caller need not
     * latch. Only a call that returns something spends that -- a host where every name
     * matched leaves the report available for whichever participant later finds one that did
     * not. Ask only where the answer is actionable, i.e. AFTER an enumeration that classified
     * this host's interfaces and only where the exclusion actually excluded something (see
     * @c domain_participant::refresh_vpn_interface_blocklist); asking first would report every
     * name as unmatched, because none of them has been offered an interface yet.
     *
     * @return Names that matched nothing, or an empty vector.
     */
    PROVIZIO_DDS_API std::vector<std::string> take_unmatched_vpn_allow_override_names();

    /**
     * @brief Record that a participant could NOT apply the VPN exclusion to its
     * transports, so nothing in this process may assume a tunnel is unbound.
     *
     * Called by @c domain_participant::refresh_vpn_interface_blocklist wherever it steps
     * aside: the caller owns the transport configuration, a participant-level netmask
     * filter of @c OFF rules a blocklist out, or the host's interfaces could not be read
     * (which is not the same as having no tunnel, and on a first creation leaves the
     * transports with no blocklist at all). Process-wide and one-way, because the consumer
     * is process-wide (the address snapshot the recovery coordinator keeps) and because the
     * safe direction is to watch an interface that might be bound rather than to ignore one
     * that is.
     *
     * Not called for @c transport_mode::localhost_only: such a participant announces
     * nothing off this host and takes no part in auto-recovery, so it has no opinion on
     * whether the process should watch tunnels.
     */
    PROVIZIO_DDS_API void report_vpn_exclusion_not_applied() noexcept;

    /**
     * @brief Whether the VPN exclusion is believed to reach the transports of every
     * participant in this process — i.e. @c report_vpn_exclusion_not_applied was never
     * called.
     *
     * What the address snapshot keys on. A tunnel this library keeps out of the
     * transports cannot change any locator, so its address churn must not rebuild
     * anything (that is the whole point of dropping it from the snapshot); a tunnel the
     * transports DO bind must be watched exactly as any other interface, or a re-auth or
     * a reconnect leaves a dead locator that nothing replaces. The two filters
     * disagreeing about one interface is the one outcome neither may produce, and this is
     * what keeps them in step where the exclusion does not apply.
     *
     * Conservative when a process mixes the two: one participant that could not exclude
     * makes the whole process watch tunnels, which costs an unnecessary rebuild at worst.
     */
    PROVIZIO_DDS_API bool vpn_exclusion_applies_to_transports() noexcept;

    /**
     * @brief Record that an interface classification ran without the platform's own
     * interface-kind information, so every verdict it produced came from names alone.
     *
     * Linux identifies a tunnel by its rtnetlink @c IFLA_INFO_KIND, which is what keeps
     * the classification working for a device renamed away from the conventional prefixes
     * (a WireGuard interface called @c office0). That dump can fail -- no @c AF_NETLINK
     * socket, a kernel reply that never arrives, a dump cut short -- and the classifier
     * then falls back to name prefixes, which is correct for every conventionally named
     * tunnel and wrong for a renamed one: DDS binds and announces it as if it were
     * ordinary hardware, which is the duplication this whole feature exists to remove.
     *
     * Recorded rather than logged, and process-wide, for the same reason
     * @c take_unmatched_vpn_allow_override_names is: this runs deep inside a participant
     * creation, under the lifecycle locks, where invoking the caller's log callback could
     * deadlock. The participant takes the report once its locks are released.
     */
    PROVIZIO_DDS_API void report_interface_kind_lookup_failed() noexcept;

    /**
     * @brief Whether an interface classification has run without the platform's
     * interface-kind information since this was last asked.
     *
     * Takes: answers @c true at most once per occurrence, so the caller need not latch and
     * a process that keeps re-enumerating a host whose kind lookup keeps failing still says
     * it once. Re-arms if the lookup fails again after being reported, because by then the
     * operator has acted on the first line or the condition has outlived it.
     *
     * @return Whether a classification since the last call ran on names alone.
     */
    PROVIZIO_DDS_API bool take_interface_kind_lookup_failure_report() noexcept;

    /**
     * @brief Everything Fast-DDS should be handed as its transports'
     * @c interface_blocklist for the host's excluded VPN interfaces: each one's device
     * name AND each of its addresses in canonical text form.
     *
     * Both forms, because a blocklist entry is matched against either the device name
     * or the address (Fast-DDS compares it to @c IPFinder::info_IP's @c dev, which is
     * @c ifa_name on POSIX and @c AdapterName on Windows, as well as to the address).
     * The device name is the stable identity, so it keeps blocking the right interface
     * even if its address changes between rebuilds; the addresses cover the reverse
     * case of a tunnel whose name a platform cannot report usefully. The address form
     * alone would also carry a hazard the name form does not: an address a tunnel has
     * released can later be handed to a real interface, and blocking it then would drop
     * genuine traffic.
     *
     * Interfaces are enumerated with the same operational filters Fast-DDS applies
     * (non-loopback, carrier up, no IPv6 link-local), since anything it would not bind
     * needs no blocking.
     *
     * @param enumeration_failed Optional out-parameter, assigned on every call: @c true
     *        when the host's interfaces could not be read at all. It matters because an
     *        empty set is also what a host with no tunnel returns, and the two must not be
     *        confused where entries have ALREADY been applied -- erasing them and re-adding
     *        from a failed read would unblock a tunnel that is still up, on exactly the
     *        rebuild a network change triggered.
     * @return Set of device names and address strings; empty when the host has no VPN
     * interface up, when @c PROVIZIO_DDS_ALLOW_VPN_INTERFACES allows them all, or when the
     * interfaces could not be read -- which @p enumeration_failed tells apart.
     */
    PROVIZIO_DDS_API std::unordered_set<std::string> vpn_interface_blocklist_entries(
        bool *enumeration_failed = nullptr);

    /**
     * @brief The same set, always enumerated from the OS — the platform-specific half
     * of @c vpn_interface_blocklist_entries, which production code should call instead.
     *
     * Split out so the substitution below has somewhere to stand.
     */
    PROVIZIO_DDS_API std::unordered_set<std::string> enumerate_vpn_interface_blocklist_entries(
        bool *enumeration_failed = nullptr);

    /**
     * @brief Collapses the host enumeration behind @c vpn_interface_blocklist_entries to
     * (at most) one for as long as this object lives, on the thread that created it.
     *
     * A single network event rebuilds every recovery-enabled participant in the process,
     * and each rebuild asks for the blocklist again — so the same rtnetlink dump (or
     * @c GetAdaptersAddresses call) ran once per participant to answer one question about
     * one host. With this held for the pass, it runs once. Rebuilding several participants
     * from one enumeration is also the more consistent answer: they are being rebuilt for
     * the same network change, and should be configured for the same interface set.
     *
     * Thread-local rather than process-wide on purpose. A participant being constructed on
     * another thread while a reset pass runs must still read the host for itself — it is
     * not part of this event — and a cache visible to it would hand it a set captured for
     * somebody else's. Nesting is counted, so an inner scope does not release an outer
     * one's cache.
     */
    class PROVIZIO_DDS_API scoped_vpn_blocklist_cache final
    {
      public:
        scoped_vpn_blocklist_cache() noexcept;
        ~scoped_vpn_blocklist_cache();
        scoped_vpn_blocklist_cache(const scoped_vpn_blocklist_cache &) = delete;
        scoped_vpn_blocklist_cache &operator=(const scoped_vpn_blocklist_cache &) = delete;
        scoped_vpn_blocklist_cache(scoped_vpn_blocklist_cache &&) = delete;
        scoped_vpn_blocklist_cache &operator=(scoped_vpn_blocklist_cache &&) = delete;
    };

    /**
     * @brief One interface the transports may still use once @p blocked is excluded, in the
     * form Fast-DDS matches an interface list entry by.
     */
    struct allowed_interface
    {
        /// IPv4 address text (@c "192.168.1.10", @c "127.0.0.1").
        std::string address;
        /// Whether this is the loopback interface, which needs a netmask filter of its own:
        /// it cannot carry a datagram to any other host, so every attempt it makes at one
        /// costs a failed @c sendto and a Fast-DDS warning, per datagram.
        bool is_loopback{false};
    };

    /**
     * @brief The interfaces Fast-DDS will let the transports use once @p blocked is excluded.
     *
     * Enumerated the way @c UDPv4Transport enumerates them -- @c IPFinder with loopback
     * included, IPv4 only -- because the answer has to match what that transport will do
     * rather than what the host merely has. Loopback is IN: a non-empty interface list puts
     * UDPv4 into whitelist mode, where every interface that is not blocked gets a sender
     * socket of its own, and loopback is not blocked (it is how same-host participants talk
     * wherever shared memory is off). An interface holding no IPv4 address is OUT: a UDPv4
     * transport can neither bind it nor own a sender socket for it.
     *
     * @param blocked Interface names and addresses the blocklist excludes.
     * @param enumeration_failed Set to @c true when the host could not be asked at all, and
     * to @c false on every successful call. Distinguishing that from an empty result is the
     * caller's whole reason for asking: an empty vector is a legitimate reading, while a
     * failed one must leave the interface lists alone -- applying a blocklist without the
     * matching per-interface netmask filters is the one state this exclusion may never
     * produce, because it puts UDPv4 into whitelist mode with a sender socket per interface
     * and no filter to stop each of them sending its own copy.
     * @return One entry per usable IPv4 address, loopback included; empty on failure.
     */
    PROVIZIO_DDS_API std::vector<allowed_interface> vpn_allowed_interfaces(
        const std::unordered_set<std::string> &blocked, bool *enumeration_failed = nullptr);

    /**
     * @brief Test hook: make @c vpn_allowed_interfaces report @p interfaces instead of
     * asking the host, or restore the real enumeration with @c std::nullopt.
     *
     * The host's interfaces are no more controllable an input than its tunnels are, and they
     * decide the same thing: a runner with a single NIC can only ever observe one branch of
     * the per-interface filter decision, so the other -- collapsing one copy of every unicast
     * datagram per interface back to one in total -- would be asserted nowhere. Substituting
     * the enumeration is what lets one case pin each branch on every platform, exactly as
     * @c force_vpn_blocklist_entries_for_test does for the entries.
     *
     * @param interfaces Substitute interfaces, or @c std::nullopt to ask the host again.
     */
    PROVIZIO_DDS_API void force_allowed_interfaces_for_test(std::optional<std::vector<allowed_interface>> interfaces);

    /**
     * @brief Test hook: make @c vpn_allowed_interfaces report that the host could not be
     * enumerated at all, or stop doing so.
     *
     * Separate from @c force_allowed_interfaces_for_test because substituting a list can only
     * ever express success. Without this the failure branch is unreachable from a test, and
     * so is what depends on it: the caller must tell a failed reading apart from an empty one
     * and leave every interface list untouched for the former, because applying a blocklist
     * with no matching per-interface netmask filters is worse than applying nothing at all
     * (see @c domain_participant::refresh_vpn_interface_blocklist).
     *
     * @param fail Whether the enumeration should report failure.
     */
    PROVIZIO_DDS_API void force_allowed_interfaces_enumeration_failure_for_test(bool fail);

    /**
     * @brief Test hook: make @c vpn_interface_blocklist_entries report @p entries
     * instead of enumerating the host, or restore enumeration with @c std::nullopt.
     *
     * Exists because the host is not a controllable input: on a runner with no tunnel
     * up, every assertion about a non-empty blocklist degenerates into @c 0 @c == @c 0
     * and a regression that stopped configuring the transports altogether would pass
     * the suite green. Substituting the enumeration is what lets one case observe the
     * populated path on every platform, exactly as the Python suite does by replacing
     * the same function.
     *
     * @param entries Substitute entries, or @c std::nullopt to enumerate the host again.
     */
    PROVIZIO_DDS_API void force_vpn_blocklist_entries_for_test(std::optional<std::unordered_set<std::string>> entries);

    /**
     * @brief Test hook: make @c vpn_interface_blocklist_entries report that the host could
     * not be enumerated at all, or stop doing so.
     *
     * Separate from @c force_vpn_blocklist_entries_for_test for the same reason
     * @c force_allowed_interfaces_enumeration_failure_for_test is separate from its
     * substitution: a substituted set is an answer, not a reading of the host, so it can
     * only ever express success. Without this the first enumeration's failure branch is
     * unreachable from a test, and so is what depends on it -- leaving every interface list
     * untouched AND telling change detection that this participant's transports may be
     * binding a tunnel after all (see @c domain_participant::refresh_vpn_interface_blocklist).
     *
     * @param fail Whether the enumeration should report failure.
     */
    PROVIZIO_DDS_API void force_vpn_blocklist_enumeration_failure_for_test(bool fail);

    /**
     * @brief Test hook: make the platform's interface-kind lookup fail, or stop doing so.
     *
     * The third hook of the same shape as the two above, and for the same reason: a
     * substituted answer can only express success, so a failure branch needs its own way in.
     * Without it the only thing a test can do is call @c report_interface_kind_lookup_failed
     * by hand, which pins the flag's own semantics and nothing about how a failing dump
     * reaches it -- so deleting the report from the funnel every failure exit goes through
     * would leave the suite green.
     *
     * Honoured only where such a lookup exists (Linux's rtnetlink dump). On the platforms
     * that classify by name alone there is nothing to fail, so setting it changes nothing.
     *
     * @param fail Whether the interface-kind lookup should report failure.
     */
    PROVIZIO_DDS_API void force_link_kind_lookup_failure_for_test(bool fail);

    /**
     * @brief Whether @c force_link_kind_lookup_failure_for_test has been set.
     *
     * For the platform backends that perform a kind lookup, so they can take their failure
     * path on demand. Not for general use.
     *
     * @return Whether the lookup should behave as though it failed.
     */
    PROVIZIO_DDS_API bool link_kind_lookup_forced_to_fail() noexcept;
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_VPN_INTERFACES
