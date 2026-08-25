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

#include <array>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#if !defined(_WIN32)
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/socket.h>
#endif

#include "provizio/dds/detail/address_snapshot.h"
#include "provizio/dds/detail/vpn_interfaces.h"

#if !defined(_WIN32)
// A library-private header, reached through the src/ include directory this test target
// is given. Its blocklist-shaping helper is a template with no exported symbol, and the
// two backends that instantiate it (Linux and macOS) feed it the host's real interfaces
// -- which no CI runner has a tunnel on, so the only way to exercise the shaping itself
// is to call it directly with entries chosen for the purpose.
#include "detail/posix_interface_walk.h"
#endif
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/network_recovery.h"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/SocketTransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/network/NetmaskFilterKind.hpp>

namespace
{
    // Not the default domain: Provizio's self-hosted runners share a LAN where domain 0
    // carries every other suite's participants. These cases only read QoS back, but a
    // participant on domain 0 still joins that traffic.
    constexpr auto k_domain = 44;
    constexpr const char *k_allow_env_name = "PROVIZIO_DDS_ALLOW_VPN_INTERFACES";

    bool expect(bool condition, const char *expression, const char *file, int line)
    {
        if (!condition)
        {
            std::cerr << "FAIL " << file << ":" << line << ": " << expression << '\n';
        }
        return condition;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage): captures #cond + __FILE__/__LINE__.
#define EXPECT(cond) expect((cond), #cond, __FILE__, __LINE__)

    void set_env(const char *name, const char *value)
    {
#if defined(_WIN32)
        _putenv_s(name, value);
#else
        // NOLINTNEXTLINE: POSIX env API
        setenv(name, value, /*overwrite=*/1);
#endif
    }

    void unset_env(const char *name)
    {
#if defined(_WIN32)
        _putenv_s(name, "");
#else
        // NOLINTNEXTLINE: POSIX env API
        unsetenv(name);
#endif
    }

    // The host's VPN-interface IPv4 addresses, found WITHOUT going through
    // vpn_interface_blocklist_entries() — a plain getifaddrs walk classified by interface name
    // only. Two reasons it is worth the duplication: the blocklist assertions compare
    // the library against something other than itself, and this touches neither the
    // PROVIZIO_DDS_ALLOW_VPN_INTERFACES cache nor the platform kind lookup, so a case
    // can read it before deciding what to set the override to. It is a SUBSET of what
    // the library finds (a renamed tunnel is caught by its rtnetlink kind, not its
    // name), so assertions below only ever require containment, never equality.
    std::unordered_set<std::string> host_vpn_addresses_by_name()
    {
        std::unordered_set<std::string> addresses;
#if !defined(_WIN32)
        ifaddrs *head = nullptr;
        if (::getifaddrs(&head) != 0)
        {
            return addresses;
        }
        for (const ifaddrs *ifa = head; ifa != nullptr; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == nullptr || ifa->ifa_name == nullptr || (ifa->ifa_flags & IFF_LOOPBACK) != 0 ||
                (ifa->ifa_flags & IFF_RUNNING) == 0 || !provizio::dds::detail::is_vpn_interface_name(ifa->ifa_name))
            {
                continue;
            }
            std::array<char, INET6_ADDRSTRLEN> text{};
            if (ifa->ifa_addr->sa_family == AF_INET)
            {
                const auto *sin = reinterpret_cast<const sockaddr_in *>(ifa->ifa_addr);  // NOLINT: sockets idiom
                if (::inet_ntop(AF_INET, &sin->sin_addr, text.data(), text.size()) != nullptr)
                {
                    addresses.insert(std::string{text.data()});
                }
            }
            // IPv4 only, matching what the library hands to the transports: a UDPv4
            // transport enumerates no IPv6 interface, so an IPv6 entry could never match
            // anything it binds (see vpn_interface_blocklist_entries).
        }
        ::freeifaddrs(head);
#endif
        return addresses;
    }

    // What a fresh participant's socket transports (UDPv4/UDPv6) ended up configured
    // with. Shared memory has no interfaces, so it contributes nothing and is not
    // counted.
    struct socket_transports
    {
        std::unordered_set<std::string> blocked;
        std::size_t descriptors{0};
        std::size_t netmask_filter_on{0};
        /// The netmask filter each allowlist entry carries, keyed by the address it names.
        /// Per entry rather than per descriptor because that is where the decision lives:
        /// loopback has to be filtered (it can reach no other host, so every attempt at one
        /// costs a failed send and a warning) while a lone real interface must not be (its
        /// peers outside the subnet would silently stop receiving).
        std::map<std::string, eprosima::fastdds::rtps::NetmaskFilterKind> allowlist;
    };

    socket_transports socket_transports_of_fresh_participant()
    {
        socket_transports configured;
        const auto participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        for (const auto &descriptor : participant->fastdds_participant()->get_qos().transport().user_transports)
        {
            const auto socket_descriptor =
                std::dynamic_pointer_cast<eprosima::fastdds::rtps::SocketTransportDescriptor>(descriptor);
            if (socket_descriptor)
            {
                ++configured.descriptors;
                if (socket_descriptor->netmask_filter == eprosima::fastdds::rtps::NetmaskFilterKind::ON)
                {
                    ++configured.netmask_filter_on;
                }
                for (const auto &entry : socket_descriptor->interface_blocklist)
                {
                    configured.blocked.insert(entry.name);
                }
                for (const auto &entry : socket_descriptor->interface_allowlist)
                {
                    configured.allowlist.emplace(entry.name, entry.netmask_filter);
                }
            }
        }
        return configured;
    }

    // Every interface_blocklist entry across a participant's socket transports.
    std::unordered_set<std::string> blocklist_of_fresh_participant()
    {
        return socket_transports_of_fresh_participant().blocked;
    }

    // Case: the classifier recognises VPN / overlay tunnel endpoints by interface
    // name, and leaves everything a Provizio device actually carries DDS on alone.
    // Names are the portable half of the signal (Windows reports adapter GUIDs as
    // the device name, so the friendly name / description is what gets matched
    // there); the Linux kind check below is the platform half.
    // Synthetic interface identities, one per role, built for whatever platform the test
    // runs on. They exist because the host is not a controllable input: no CI runner has a
    // tunnel up, so every assertion about how a tunnel is treated would otherwise be
    // vacuous — which is exactly how a broken PROVIZIO_DDS_ALLOW_VPN_INTERFACES override
    // shipped green on two platforms. snapshot_policy_excludes_interface answers for an
    // identity rather than for a live device, so these cases assert on the real production
    // decision without needing the interface to exist.
    provizio::dds::detail::interface_identity tunnel_identity()
    {
        provizio::dds::detail::interface_identity identity;
#if defined(_WIN32)
        // Windows names devices by GUID, so the classifier reads the friendly name and the
        // driver description; IF_TYPE_TUNNEL is the platform's own signal.
        identity.name = "{00000000-0000-0000-0000-provizio-test}";
        identity.friendly_name = "Tailscale";
        identity.description = "Tailscale Tunnel";
        identity.platform_says_tunnel = true;
#elif defined(__APPLE__)
        // macOS reports no interface kind, and every VPN lands on a utunN device — the
        // prefix that is ALSO in the snapshot's own name-exclusion list, which is what made
        // the override unable to re-admit it.
        identity.name = "utun9";
#else
        identity.name = "tailscale0";
        identity.kind = "wireguard";
#endif
        return identity;
    }

    provizio::dds::detail::interface_identity ordinary_identity()
    {
        provizio::dds::detail::interface_identity identity;
#if defined(_WIN32)
        identity.name = "{11111111-1111-1111-1111-provizio-test}";
        identity.friendly_name = "Ethernet 2";
        identity.description = "Intel(R) Ethernet Connection I219-LM";
        identity.platform_says_physical = true;
#elif defined(__APPLE__)
        identity.name = "en0";
#else
        identity.name = "eth0";
#endif
        return identity;
    }

    // Excluded by a heuristic that has nothing to do with VPNs (container plumbing,
    // hypervisor adapters). Included in both cases below to pin the boundary: the override
    // re-admits tunnels, and must not quietly re-admit everything else with them.
    provizio::dds::detail::interface_identity virtual_identity()
    {
        provizio::dds::detail::interface_identity identity;
#if defined(_WIN32)
        identity.name = "{22222222-2222-2222-2222-provizio-test}";
        identity.friendly_name = "vEthernet (WSL)";
        identity.description = "Hyper-V Virtual Ethernet Adapter";
        identity.platform_says_physical = true;
#elif defined(__APPLE__)
        identity.name = "bridge0";
#else
        identity.name = "docker0";
        identity.kind = "bridge";
#endif
        return identity;
    }

    int test_classifier()
    {
        bool passed = true;

        for (const auto *vpn_name :
             {"tailscale0", "tun0", "tun9", "tap0", "utun3", "ipsec1", "wg0", "ztabcdefgh", "Tailscale",
              "WireGuard Tunnel", "OpenVPN TAP-Windows6", "TAP-Windows Adapter V9", "Tunnel adapter Teredo"})
        {
            passed &= EXPECT(provizio::dds::detail::is_vpn_interface_name(vpn_name));
        }

        // Physical NICs, container plumbing and loopback must never be classified as
        // VPNs: blocking a Docker bridge or a vehicle PC's br0 would break real
        // same-host / same-LAN DDS paths, which is not what this feature is for.
        // The vendor strings are the reason the short conventions (tun / tap / wg) are
        // matched only as prefixes: "NETGEAR WG111v3" contains "wg", and misclassifying a
        // host's only NIC would drop ALL of its DDS traffic — a far worse outcome than the
        // duplicate traffic this feature removes.
        for (const auto *ordinary_name : {"eth0",
                                          "eno1",
                                          "enp8s0",
                                          "wlp9s0",
                                          "wlan0",
                                          "docker0",
                                          "br0",
                                          "br-lan",
                                          "veth1a2b",
                                          "lo",
                                          "l4tbr0",
                                          "ppp0",
                                          "usb0",
                                          "rndis0",
                                          "can0",
                                          "Ethernet",
                                          "Wi-Fi",
                                          "NETGEAR WG111v3 54Mbps Wireless USB 2.0 Adapter",
                                          "NETGEAR WGA600N Wireless Gaming Adapter",
                                          "Intel(R) Ethernet Connection I219-LM",
                                          "Marvell AQtion 10Gbit Network Adapter"})
        {
            passed &= EXPECT(!provizio::dds::detail::is_vpn_interface_name(ordinary_name));
        }

#if defined(__linux__)
        // rtnetlink IFLA_INFO_KIND is what identifies a tunnel whose name follows no
        // convention (a renamed wg device, an xfrm interface); physical Ethernet and
        // Wi-Fi report no kind at all, and container plumbing reports its own kinds.
        for (const auto *vpn_kind : {"tun", "wireguard", "ip6tnl", "gre", "vti", "xfrm"})
        {
            passed &= EXPECT(provizio::dds::detail::is_vpn_interface_kind(vpn_kind));
        }
        for (const auto *ordinary_kind : {"", "bridge", "veth", "dummy", "vxlan", "macvlan", "ipvlan", "bond"})
        {
            passed &= EXPECT(!provizio::dds::detail::is_vpn_interface_kind(ordinary_kind));
        }
#endif

        // A user-editable label (Windows' friendly name) is matched only against vendor
        // names: an Ethernet adapter renamed "WG-LAN" must not be mistaken for a tunnel,
        // because dropping it would take every bit of that host's DDS traffic with it.
        for (const auto *product_name : {"Tailscale", "WireGuard Tunnel", "OpenVPN TAP-Windows6", "ZeroTier One"})
        {
            passed &= EXPECT(provizio::dds::detail::is_vpn_product_name(product_name));
        }
        for (const auto *renamed_nic : {"WG-LAN", "TUN-uplink", "tap-office", "Ethernet 2", "LAN", "Wi-Fi"})
        {
            passed &= EXPECT(!provizio::dds::detail::is_vpn_product_name(renamed_nic));
        }
        // The driver-supplied description keeps the full classifier, which is what still
        // catches OpenVPN's TAP adapter — an Ethernet-type device naming no vendor.
        passed &= EXPECT(provizio::dds::detail::is_vpn_interface_name("TAP-Windows Adapter V9"));

        std::cout << "classifier: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: with no override, every address of every VPN interface on the host is
    // handed to Fast-DDS as a blocked interface, so the participant neither binds
    // nor announces a locator on it. On a host with no VPN up this asserts the
    // empty case (the plumbing still has to run without throwing).
    int test_blocklist()
    {
        unset_env(k_allow_env_name);

        const auto expected = provizio::dds::detail::vpn_interface_blocklist_entries();
        const auto blocked = blocklist_of_fresh_participant();

        bool passed = true;
        for (const auto &address : expected)
        {
            passed &= EXPECT(blocked.find(address) != blocked.end());
        }
        passed &= EXPECT(blocked.size() == expected.size());
        // Independent of the library's own enumeration: every VPN address a plain
        // name scan of getifaddrs finds must be blocked as well.
        for (const auto &address : host_vpn_addresses_by_name())
        {
            passed &= EXPECT(blocked.find(address) != blocked.end());
        }

        std::cout << "blocklist: " << (passed ? "PASS" : "FAIL") << " (" << blocked.size() << " blocked address(es)";
        if (expected.empty())
        {
            std::cout << "; host has no VPN interface up, empty case only";
        }
        std::cout << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: the populated path, on every runner — including the many with no tunnel
    // up, where every other blocklist assertion here degenerates into 0 == 0 and a
    // regression that stopped configuring the transports altogether would pass the
    // suite green. Substituting the enumeration is the only way to reach it, since the
    // host's interfaces are not an input the suite controls; the Python suite reaches
    // the same path by replacing the same function.
    int test_blocklist_forced()
    {
        unset_env(k_allow_env_name);

        // Documentation-range address (RFC 5737) and a name no host carries, so nothing
        // here can collide with a real interface if the substitution were ever to leak.
        const std::unordered_set<std::string> synthetic{"provizio-test-tunnel0", "198.51.100.7"};
        provizio::dds::detail::force_vpn_blocklist_entries_for_test(synthetic);

        // What the exclusion leaves behind, forced for the same reason the entries are: the
        // host decides the per-interface netmask filter below, and on a single-NIC runner --
        // which is most of them -- one branch of that decision would be asserted nowhere.
        // The host's real loopback is kept in every substituted set on purpose: an allowlist
        // that matches no interface at all leaves Fast-DDS' whitelist empty, and it then
        // fills it with a sentinel that allows nothing through.
        const provizio::dds::detail::allowed_interface loopback{"127.0.0.1", true};
        const provizio::dds::detail::allowed_interface first_real{"198.51.100.10", false};
        const provizio::dds::detail::allowed_interface second_real{"203.0.113.10", false};
        using kind = eprosima::fastdds::rtps::NetmaskFilterKind;

        // Two real interfaces: each one sends its own copy of every unicast datagram, so
        // filtering is what collapses them back to the socket whose subnet holds the
        // destination -- ON everywhere, loopback included.
        provizio::dds::detail::force_allowed_interfaces_for_test(
            std::vector<provizio::dds::detail::allowed_interface>{loopback, first_real, second_real});
        const auto duplicating = socket_transports_of_fresh_participant();
        bool passed = EXPECT(duplicating.descriptors > 0);
        passed &= EXPECT(duplicating.blocked == synthetic);
        passed &= EXPECT(duplicating.allowlist.size() == 3);
        passed &= EXPECT(duplicating.allowlist.at("127.0.0.1") == kind::ON);
        passed &= EXPECT(duplicating.allowlist.at("198.51.100.10") == kind::ON);
        passed &= EXPECT(duplicating.allowlist.at("203.0.113.10") == kind::ON);
        // The descriptor stays AUTO throughout: it is the container the entries are
        // validated against, and ON there would override every entry's own answer.
        passed &= EXPECT(duplicating.netmask_filter_on == 0);

        // One real interface, the common vehicle host. Nothing duplicates, so the real
        // interface keeps AUTO and its peers behind a gateway keep receiving unicast --
        // while loopback is still filtered, because it can reach no other host at all and
        // every attempt it makes at one costs a failed sendto and a warning per datagram.
        provizio::dds::detail::force_allowed_interfaces_for_test(
            std::vector<provizio::dds::detail::allowed_interface>{loopback, first_real});
        const auto single_interface = socket_transports_of_fresh_participant();
        passed &= EXPECT(single_interface.blocked == synthetic);
        passed &= EXPECT(single_interface.allowlist.size() == 2);
        passed &= EXPECT(single_interface.allowlist.at("127.0.0.1") == kind::ON);
        passed &= EXPECT(single_interface.allowlist.at("198.51.100.10") == kind::AUTO);
        passed &= EXPECT(single_interface.netmask_filter_on == 0);
        provizio::dds::detail::force_allowed_interfaces_for_test(std::nullopt);

        // And the reverse, which is what makes the assertion above mean something: with
        // nothing to block, the descriptors are left as Fast-DDS built them, so a host
        // without a tunnel keeps the any-address output socket and the cheaper send path
        // that comes with it.
        provizio::dds::detail::force_vpn_blocklist_entries_for_test(std::unordered_set<std::string>{});
        const auto unconfigured = socket_transports_of_fresh_participant();
        passed &= EXPECT(unconfigured.descriptors == duplicating.descriptors);
        passed &= EXPECT(unconfigured.blocked.empty());
        passed &= EXPECT(unconfigured.netmask_filter_on == 0);
        // No interface list of any kind, which is what keeps the any-address socket: an
        // allowlist alone would put UDPv4 into whitelist mode just as a blocklist does.
        passed &= EXPECT(unconfigured.allowlist.empty());

        provizio::dds::detail::force_vpn_blocklist_entries_for_test(std::nullopt);

        std::cout << "blocklist_forced: " << (passed ? "PASS" : "FAIL") << " (" << duplicating.descriptors
                  << " socket transport descriptor(s))" << '\n';
        return passed ? 0 : 1;
    }

    // Case: PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1 restores the pre-2.0 behaviour of
    // binding and announcing every interface the OS offers — for the rare
    // deployment that genuinely carries DDS over a tunnel (which needs unicast
    // discovery to be configured too, since no VPN carries multicast).
    int test_blocklist_override_all()
    {
        set_env(k_allow_env_name, "1");

        bool passed = EXPECT(provizio::dds::detail::vpn_interface_blocklist_entries().empty());
        passed &= EXPECT(blocklist_of_fresh_participant().empty());

        std::cout << "blocklist_override_all: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: a named interface is allowed back individually, so a host that carries
    // real DDS traffic over one tunnel can keep every other tunnel excluded.
    int test_blocklist_override_named()
    {
        // Set before the first read: the override is parsed once per process (so the
        // transports, the snapshot and every rebuilt participant agree), which is also
        // why every case here is its own ctest entry. Two names, to pin both halves of
        // the contract in one process: "tailscale0" is re-admitted, "wg0" is not.
        set_env(k_allow_env_name, " tailscale0 ,definitely-not-an-interface");

        bool passed = true;
        // The named interface is allowed back — this is the documented escape hatch, and
        // the assertion holds on every host because the predicate takes the name
        // directly rather than requiring such an interface to exist.
        passed &= EXPECT(!provizio::dds::detail::excluded_as_vpn_interface("tailscale0", false));
        // Whitespace around a list entry is ignored, as for every other list-valued
        // PROVIZIO_DDS_* variable.
        passed &= EXPECT(!provizio::dds::detail::excluded_as_vpn_interface("definitely-not-an-interface", true));
        // Every other tunnel stays excluded.
        passed &= EXPECT(provizio::dds::detail::excluded_as_vpn_interface("wg0", false));
        passed &= EXPECT(provizio::dds::detail::excluded_as_vpn_interface("office0", /*platform_says_vpn=*/true));
        // Whole-name matching, not prefix matching: the value names an interface, and a
        // loose match here would silently re-admit a different tunnel.
        passed &= EXPECT(provizio::dds::detail::excluded_as_vpn_interface("tailscale1", false));
        // Case, on the other hand, is ignored — on the same terms the classifier used.
        // On Windows the identity being classified is the adapter's friendly name or
        // description ("Tailscale"), and requiring its exact capitalisation would leave
        // the escape hatch silently doing nothing there.
        passed &= EXPECT(!provizio::dds::detail::excluded_as_vpn_interface("TailScale0", false));

        // And a host that actually has the named interface up no longer blocks it.
        const auto entries = provizio::dds::detail::vpn_interface_blocklist_entries();
        passed &= EXPECT(entries.find("tailscale0") == entries.end());

        std::cout << "blocklist_override_named: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

#if !defined(_WIN32)
    // Case: which of a walk's entries become blocklist entries, and in what form. The
    // shaping is a template fed by the Linux and macOS backends from getifaddrs, so on a
    // CI runner -- none of which has a tunnel up -- every other test of the blocklist sees
    // it produce an empty set no matter what it does. Synthetic entries are the only input
    // that distinguishes "excludes exactly the IPv4 addresses of tunnels" from "excludes
    // nothing", from "excludes everything that is not a tunnel", and from "excludes
    // tunnels' IPv6 addresses too" -- which a UDPv4 transport cannot be told to block.
    int test_blocklist_entries_from_walk()
    {
        using provizio::dds::detail::posix_interface_address;

        const std::vector<posix_interface_address> entries{
            // A tunnel, in the form the transports can actually use.
            posix_interface_address{"tun0", "10.8.0.2", 24, true},
            // The same tunnel's IPv6 address: classified identically, but a UDPv4
            // interface_blocklist entry cannot express it.
            posix_interface_address{"tun0", "fd00::2", 64, false},
            // An ordinary interface, which must survive: blocking it would take DDS off
            // the LAN entirely.
            posix_interface_address{"eth0", "192.168.1.7", 24, true},
            // A tunnel the platform could not name. The address alone still has to reach
            // the blocklist -- it is the only identity there is.
            posix_interface_address{"", "10.9.0.3", 24, true},
        };

        const auto blocked = provizio::dds::detail::vpn_blocklist_entries_from(
            entries, [](const posix_interface_address &entry) { return entry.name.empty() || entry.name == "tun0"; });

        bool passed = true;
        // Both forms for a named tunnel: Fast-DDS matches a blocklist entry against
        // IPFinder::info_IP's dev as well as its address.
        passed &= EXPECT(blocked.find("tun0") != blocked.end());
        passed &= EXPECT(blocked.find("10.8.0.2") != blocked.end());
        // The nameless tunnel contributes its address and no empty string.
        passed &= EXPECT(blocked.find("10.9.0.3") != blocked.end());
        passed &= EXPECT(blocked.find("") == blocked.end());
        // IPv4 only.
        passed &= EXPECT(blocked.find("fd00::2") == blocked.end());
        // And nothing else at all: not the ordinary interface, and not some third form.
        passed &= EXPECT(blocked.find("eth0") == blocked.end());
        passed &= EXPECT(blocked.find("192.168.1.7") == blocked.end());
        passed &= EXPECT(blocked.size() == 3);

        std::cout << "blocklist_entries_from_walk: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }
#endif

    // Case: the PARTICIPANT-level netmask filter, set to OFF, makes the exclusion step
    // aside entirely -- and say so. Distinct from the caller-descriptor case below, which
    // sets the filter on a SocketTransportDescriptor of its own: this one is
    // DomainParticipantQos::transport().netmask_filter, a different object, and it is the
    // one refresh_vpn_interface_blocklist actually reads.
    //
    // Stepping aside is not a nicety here. Fast-DDS validates a descriptor's netmask filter
    // against the participant's and REFUSES to register a transport whose descriptor asks
    // for ON while the participant says OFF, so applying the exclusion to such a participant
    // would take UDP away altogether -- no cross-host communication at all, reported only as
    // a Fast-DDS error line -- which is far worse than the duplicate traffic it saves.
    int test_blocklist_participant_netmask_off()
    {
        unset_env(k_allow_env_name);

        // Something to exclude, so the case tests the step-aside rather than an empty host.
        const std::unordered_set<std::string> forced{"provizio_test_tunnel0", "203.0.113.9"};
        provizio::dds::detail::force_vpn_blocklist_entries_for_test(forced);

        const auto factory = eprosima::fastdds::dds::DomainParticipantFactory::get_shared_instance();
        eprosima::fastdds::dds::DomainParticipantQos qos;
        factory->get_default_participant_qos(qos);
        qos.transport().netmask_filter = eprosima::fastdds::rtps::NetmaskFilterKind::OFF;
        bool passed = EXPECT(factory->set_default_participant_qos(qos) == eprosima::fastdds::dds::RETCODE_OK);

        // Asserted before the participant exists: the latch is process-wide and one-way, so
        // reading it first is what makes the assertion below about THIS participant.
        passed &= EXPECT(provizio::dds::detail::vpn_exclusion_applies_to_transports());

        const auto configured = socket_transports_of_fresh_participant();

        // Nothing of ours was written to any descriptor: neither the blocklist nor the
        // allowlist that carries the per-interface filters.
        passed &= EXPECT(configured.blocked.empty());
        passed &= EXPECT(configured.allowlist.empty());
        passed &= EXPECT(configured.netmask_filter_on == 0);
        // Vacuity guard: the participant did get socket transports, so "nothing applied"
        // is a decision rather than an absence of anything to decide about.
        passed &= EXPECT(configured.descriptors > 0);

        // And change detection was told, so a tunnel this participant does bind keeps being
        // watched -- the two filters must never disagree about an interface.
        passed &= EXPECT(!provizio::dds::detail::vpn_exclusion_applies_to_transports());

        provizio::dds::detail::force_vpn_blocklist_entries_for_test(std::nullopt);
        std::cout << "blocklist_participant_netmask_off: " << (passed ? "PASS" : "FAIL") << " ("
                  << configured.descriptors << " socket transport(s))" << '\n';
        return passed ? 0 : 1;
    }

#if defined(__linux__)
    // Case: a real interface-kind lookup failure reaches the participant's log, naming the way
    // out. The Python suite pinned this from the start; the C++ one pinned only the flag, so
    // deleting the whole warning block from refresh_vpn_interface_blocklist left every C++ case
    // green -- and the C++ library is the surface more consumers use.
    //
    // Drives the real route rather than raising the flag by hand: the forcing hook makes
    // fetch_link_kinds take a failure exit, so the funnel every such exit goes through, the
    // process-wide report, the participant's take and the log line are all under test. Raising
    // the flag directly, which the case above does, pins none of that -- deleting the report
    // from the funnel would not fail it.
    //
    // Linux-only, because that is where an interface-kind lookup exists at all; the other
    // platforms classify by name and have nothing to fail.
    int test_interface_kind_lookup_failure_warns()
    {
        unset_env(k_allow_env_name);

        // Drained first: this process may have enumerated during start-up, and a stale report
        // would make the assertion below pass without the forced failure doing anything.
        (void)provizio::dds::detail::take_interface_kind_lookup_failure_report();

        provizio::dds::detail::force_link_kind_lookup_failure_for_test(true);

        std::vector<std::string> warnings;
        std::mutex warnings_mutex;
        auto previous = provizio::dds::set_log_callback(
            [&warnings, &warnings_mutex](provizio::dds::log_level level, std::string_view message) {
                if (level == provizio::dds::log_level::warning)
                {
                    const std::lock_guard<std::mutex> lock{warnings_mutex};
                    warnings.emplace_back(message);
                }
            });

        {
            const auto participant =
                provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        }

        provizio::dds::set_log_callback(std::move(previous));
        provizio::dds::detail::force_link_kind_lookup_failure_for_test(false);

        std::string reported;
        {
            const std::lock_guard<std::mutex> lock{warnings_mutex};
            for (const auto &message : warnings)
            {
                if (message.find("could not read this host's interface kinds") != std::string::npos)
                {
                    reported = message;
                    break;
                }
            }
        }

        bool passed = EXPECT(!reported.empty());
        // Actionable, not merely present: an operator who wants DDS over that tunnel has to be
        // told what to set.
        passed &= EXPECT(reported.find(k_allow_env_name) != std::string::npos);

        std::cout << "interface_kind_lookup_failure_warns: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }
#endif

    // Case: an unreadable interface list on the SECOND enumeration leaves every list alone.
    //
    // Two independent IPFinder walks decide this: one produces the blocklist, the other the
    // interfaces left to bind (and therefore the per-interface netmask filters). The second
    // can fail where the first succeeded, and the outcome that must never follow is a
    // non-empty blocklist with no allowlist beside it: any non-empty interface list puts
    // UDPv4 into whitelist mode, giving every remaining interface its own sender socket,
    // and without the filters each of them sends its own copy of every unicast datagram --
    // the duplication this feature removes, relocated from the tunnel onto the LAN.
    //
    // Reachable only through the forcing hook: substituting a list can express success and
    // nothing else, so before that hook existed this branch had no test at all.
    //
    // Its own subcommand, like every other case that asserts on the transports-owning latch:
    // that latch is process-wide and one-way, and this case sets it.
    int test_blocklist_allowed_enumeration_failure_changes_nothing()
    {
        unset_env(k_allow_env_name);

        // Something to block, so the case tests the bail-out rather than an empty host.
        const std::unordered_set<std::string> forced{"provizio_test_tunnel0", "203.0.113.9"};
        provizio::dds::detail::force_vpn_blocklist_entries_for_test(forced);
        provizio::dds::detail::force_allowed_interfaces_enumeration_failure_for_test(true);

        // Asserted before the participant exists, so the assertion after it is about THIS
        // participant rather than about whatever the process had already recorded.
        bool passed = EXPECT(provizio::dds::detail::vpn_exclusion_applies_to_transports());

        const auto configured = socket_transports_of_fresh_participant();

        // Nothing applied: not the blocklist the first enumeration justified, and not the
        // allowlist the second could not supply.
        passed &= EXPECT(configured.blocked.empty());
        passed &= EXPECT(configured.allowlist.empty());
        passed &= EXPECT(configured.netmask_filter_on == 0);
        // Vacuity guard: the participant did get socket transports, so "nothing applied" is
        // a decision rather than an absence of anything to decide about.
        passed &= EXPECT(configured.descriptors > 0);

        // And change detection was told. Nothing was applied, so the transports bind the
        // tunnel this reading found; a snapshot that dropped it anyway would leave a
        // reconnect with a dead locator no rebuild replaces -- the two filters must never
        // disagree about one interface.
        passed &= EXPECT(!provizio::dds::detail::vpn_exclusion_applies_to_transports());

        provizio::dds::detail::force_allowed_interfaces_enumeration_failure_for_test(false);
        provizio::dds::detail::force_vpn_blocklist_entries_for_test(std::nullopt);

        std::cout << "blocklist_allowed_enumeration_failure_changes_nothing: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: an unreadable interface list on the FIRST enumeration -- the one that produces
    // the blocklist -- leaves every list alone AND tells change detection that this
    // participant's transports may be binding a tunnel after all.
    //
    // The second half is what this case exists for. On a first creation there is nothing in
    // the descriptors for "leave every list exactly as it stands" to preserve, so they keep
    // Fast-DDS' default -- no interface_blocklist at all -- and DDS binds and announces
    // every tunnel the host has. A snapshot filter that went on dropping tunnels would then
    // stop watching an interface the transports do bind, which is the disagreement
    // address_snapshot.h calls the one outcome neither filter may produce.
    //
    // Reachable only through the forcing hook, for the same reason its allowed-interfaces
    // sibling is: a substituted set of entries is an answer, not a reading of the host.
    int test_blocklist_enumeration_failure_changes_nothing()
    {
        unset_env(k_allow_env_name);

        provizio::dds::detail::force_vpn_blocklist_enumeration_failure_for_test(true);

        bool passed = EXPECT(provizio::dds::detail::vpn_exclusion_applies_to_transports());

        const auto configured = socket_transports_of_fresh_participant();

        passed &= EXPECT(configured.blocked.empty());
        passed &= EXPECT(configured.allowlist.empty());
        passed &= EXPECT(configured.netmask_filter_on == 0);
        // Vacuity guard, as above.
        passed &= EXPECT(configured.descriptors > 0);

        passed &= EXPECT(!provizio::dds::detail::vpn_exclusion_applies_to_transports());

        provizio::dds::detail::force_vpn_blocklist_enumeration_failure_for_test(false);

        std::cout << "blocklist_enumeration_failure_changes_nothing: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: a classification that ran without the platform's interface-kind information
    // reports itself, once, rather than silently degrading to name-prefix matching.
    //
    // What it guards: on Linux the rtnetlink IFLA_INFO_KIND is what identifies a tunnel
    // renamed away from the conventional prefixes (a WireGuard device called office0). When
    // the dump cannot be issued or does not complete, the classifier falls back to names,
    // that device is taken for ordinary hardware, and DDS binds and announces it -- the
    // duplication this whole feature exists to remove, reinstated with nothing anywhere
    // saying so. The report is what an operator has to go on.
    int test_interface_kind_lookup_failure_reported()
    {
        // Nothing has failed yet, so nothing is reported. Asserted first because the take
        // is destructive: without it a stale report from this process' own start-up
        // enumeration would make the assertions below pass for the wrong reason.
        bool passed = EXPECT(!provizio::dds::detail::take_interface_kind_lookup_failure_report());

        provizio::dds::detail::report_interface_kind_lookup_failed();
        passed &= EXPECT(provizio::dds::detail::take_interface_kind_lookup_failure_report());

        // Taken once: the caller does not latch for itself, and a process that keeps
        // re-enumerating a host whose lookup keeps failing must still say it once.
        passed &= EXPECT(!provizio::dds::detail::take_interface_kind_lookup_failure_report());

        // Re-arms, unlike the transports-owning latch: this describes one enumeration rather
        // than a property of the process, so a lookup that starts failing again after the
        // first report is worth a second line.
        provizio::dds::detail::report_interface_kind_lookup_failed();
        passed &= EXPECT(provizio::dds::detail::take_interface_kind_lookup_failure_report());

        std::cout << "interface_kind_lookup_failure_reported: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: a name that re-admits nothing is reported, once, rather than leaving the
    // escape hatch silently doing nothing. The gap this covers is a typo -- "tailscale"
    // for a device called "tailscale0" -- or the wrong identity for the platform, both of
    // which otherwise leave the tunnel excluded for the life of the process exactly as if
    // the variable had never been set.
    int test_blocklist_override_unmatched()
    {
        // One name that will match an interface offered to the classifier below and one
        // that will not, so the report has to distinguish them rather than simply echo
        // what was set.
        set_env(k_allow_env_name, "wg0,tailscale");

        bool passed = true;
        // Nothing has been classified yet, so nothing has matched. The report is not asked
        // for here -- see the note on take_unmatched_vpn_allow_override_names: before an
        // enumeration every name looks unmatched, which is why the caller asks only after
        // one.
        passed &= EXPECT(!provizio::dds::detail::excluded_as_vpn_interface("wg0", false));
        // "tailscale" does not name this one: whole-name matching, so the tunnel stays out.
        passed &= EXPECT(provizio::dds::detail::excluded_as_vpn_interface("tailscale0", false));

        const auto unmatched = provizio::dds::detail::take_unmatched_vpn_allow_override_names();
        passed &= EXPECT(unmatched.size() == 1);
        passed &= EXPECT(!unmatched.empty() && unmatched.front() == "tailscale");

        // Once per process: a second participant creation on a host with a tunnel up must
        // not repeat it, and the caller does not latch for itself.
        passed &= EXPECT(provizio::dds::detail::take_unmatched_vpn_allow_override_names().empty());

        std::cout << "blocklist_override_unmatched: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: every name matched, so there is nothing to report. The other half of the case
    // above, and what keeps the report from degenerating into "echo whatever was set":
    // a working escape hatch must stay silent.
    int test_blocklist_override_all_matched()
    {
        set_env(k_allow_env_name, "wg0");

        bool passed = true;
        passed &= EXPECT(!provizio::dds::detail::excluded_as_vpn_interface("wg0", false));
        passed &= EXPECT(provizio::dds::detail::take_unmatched_vpn_allow_override_names().empty());

        std::cout << "blocklist_override_all_matched: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: when the caller owns the transport configuration via
    // FASTDDS_BUILTIN_TRANSPORTS, Fast-DDS builds the descriptors itself and this
    // library must not touch them — no blocklist, no attempt to reach into a QoS that
    // holds none. The README promises exactly this.
    // Case: a transport descriptor the caller configured is left exactly as they set it.
    // Nothing this library can probe sees transports a caller configured through
    // DomainParticipantFactory (load_XML_profiles_file/_string, or set_default_participant_qos
    // as here): there is no FASTDDS_DEFAULT_PROFILES_FILE and no DEFAULT_FASTDDS_PROFILES.xml,
    // yet get_default_participant_qos hands their descriptor back as the very shared_ptr the
    // factory holds, and setup_transports appends ours alongside it rather than replacing it.
    // Rewriting that descriptor would flip a netmask_filter they chose — process-wide, since
    // the object is shared — and delete a blocklist entry of theirs as soon as the tunnel went
    // away, re-exposing DDS on an interface they had deliberately excluded.
    int test_leaves_caller_transports_alone()
    {
        unset_env(k_allow_env_name);

        // Forced, so the case cannot pass vacuously: on a runner with no tunnel nothing is
        // applied to any descriptor and every assertion below would hold with the guard
        // deleted.
        const std::unordered_set<std::string> forced{"203.0.113.9", "tun-forced"};
        provizio::dds::detail::force_vpn_blocklist_entries_for_test(forced);
        // What the exclusion leaves behind, forced so the allowlist assertions below are the
        // same on every runner. Loopback plus one real interface: the shape whose filters
        // differ from each other, and so the shape that proves the entries carry their own.
        provizio::dds::detail::force_allowed_interfaces_for_test(
            std::vector<provizio::dds::detail::allowed_interface>{{"127.0.0.1", true}, {"198.51.100.10", false}});

        static constexpr const char *const caller_entry = "utun9-configured-by-the-caller";
        const auto caller_descriptor = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        caller_descriptor->netmask_filter = eprosima::fastdds::rtps::NetmaskFilterKind::OFF;
        caller_descriptor->interface_blocklist.emplace_back(caller_entry);

        const auto factory = eprosima::fastdds::dds::DomainParticipantFactory::get_shared_instance();
        eprosima::fastdds::dds::DomainParticipantQos caller_qos;
        factory->get_default_participant_qos(caller_qos);
        caller_qos.transport().user_transports.push_back(caller_descriptor);
        bool passed = EXPECT(factory->set_default_participant_qos(caller_qos) == eprosima::fastdds::dds::RETCODE_OK);

        const auto configured = socket_transports_of_fresh_participant();

        // Theirs, untouched: the one entry they wrote, the netmask filter they chose, and an
        // interface_allowlist this library never added to. That last one matters as much as
        // the others: an allowlist entry appended here would confine THEIR transport to the
        // interfaces this library picked, which is a far larger change to a descriptor they
        // own than a blocklist entry would have been.
        passed &= EXPECT(caller_descriptor->interface_blocklist.size() == 1);
        passed &= EXPECT(caller_descriptor->interface_blocklist.front().name == caller_entry);
        passed &= EXPECT(caller_descriptor->netmask_filter == eprosima::fastdds::rtps::NetmaskFilterKind::OFF);
        passed &= EXPECT(caller_descriptor->interface_allowlist.empty());

        // Ours, configured: the exclusion still applied where it is allowed to, so this
        // case pins a boundary rather than an inert code path.
        for (const auto &address : forced)
        {
            passed &= EXPECT(configured.blocked.find(address) != configured.blocked.end());
        }
        passed &= EXPECT(configured.descriptors >= 2);
        // The allowlist seen across every descriptor is ours alone, since the caller's has
        // none: loopback filtered, the lone real interface left alone.
        passed &= EXPECT(configured.allowlist.size() == 2);
        passed &= EXPECT(configured.allowlist.at("127.0.0.1") == eprosima::fastdds::rtps::NetmaskFilterKind::ON);
        passed &= EXPECT(configured.allowlist.at("198.51.100.10") == eprosima::fastdds::rtps::NetmaskFilterKind::AUTO);

        provizio::dds::detail::force_vpn_blocklist_entries_for_test(std::nullopt);
        provizio::dds::detail::force_allowed_interfaces_for_test(std::nullopt);

        std::cout << "leaves_caller_transports_alone: " << (passed ? "PASS" : "FAIL") << " (" << configured.descriptors
                  << " socket transport descriptor(s), " << configured.allowlist.size()
                  << " allowlist entry(ies) of ours)" << '\n';
        return passed ? 0 : 1;
    }

    int test_no_op_with_builtin_transports_env()
    {
        unset_env(k_allow_env_name);
        set_env("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4");

        // Forced, so the case cannot pass vacuously. Without a substitute set, a runner
        // with no tunnel produces an empty blocklist anyway and the assertion below would
        // hold even if the guard were deleted; with one, the only reason nothing is
        // blocked is that the guard saw the caller's own transport selection.
        provizio::dds::detail::force_vpn_blocklist_entries_for_test(
            std::unordered_set<std::string>{"203.0.113.7", "tun-forced"});

        const bool passed = EXPECT(blocklist_of_fresh_participant().empty());
        provizio::dds::detail::force_vpn_blocklist_entries_for_test(std::nullopt);

        std::cout << "no_op_with_builtin_transports_env: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: with no override, an interface the transports refuse to bind is kept out of
    // the change-detection snapshot too — asserted on a synthetic identity, so it holds on
    // every runner rather than only on a developer machine with a tunnel up.
    int test_snapshot_policy_excludes_tunnel()
    {
        unset_env(k_allow_env_name);

        bool passed = EXPECT(provizio::dds::detail::snapshot_policy_excludes_interface(tunnel_identity()));
        passed &= EXPECT(!provizio::dds::detail::snapshot_policy_excludes_interface(ordinary_identity()));
        passed &= EXPECT(provizio::dds::detail::snapshot_policy_excludes_interface(virtual_identity()));

        std::cout << "snapshot_policy_excludes_tunnel: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: where the exclusion could NOT be applied to the transports, the snapshot keeps
    // tunnels — because DDS binds and announces them there, so a re-auth or a reconnect
    // moves a locator that a rebuild has to follow. This is the half the first draft of the
    // feature got wrong in the other direction: the snapshot dropped tunnels
    // unconditionally, so on a host whose transports came from the caller's own XML the
    // tunnel was bound and announced while its churn rebuilt nothing at all.
    //
    // Its own subcommand because report_vpn_exclusion_not_applied is process-wide and
    // one-way, by design: a process is either watching tunnels or it is not, and no test
    // may leave the other cases running under the flag this one sets.
    int test_snapshot_policy_follows_transports()
    {
        unset_env(k_allow_env_name);

        // Before: the exclusion is believed to apply, so the tunnel is dropped.
        bool passed = EXPECT(provizio::dds::detail::snapshot_policy_excludes_interface(tunnel_identity()));

        provizio::dds::detail::report_vpn_exclusion_not_applied();

        // After: the tunnel is watched like any other interface...
        passed &= EXPECT(!provizio::dds::detail::snapshot_policy_excludes_interface(tunnel_identity()));
        // ...and nothing else changes with it. Container plumbing has nothing to do with
        // the transports owning question and must still be dropped, or every veth churn on
        // a Docker host would rebuild every participant.
        passed &= EXPECT(provizio::dds::detail::snapshot_policy_excludes_interface(virtual_identity()));
        passed &= EXPECT(!provizio::dds::detail::snapshot_policy_excludes_interface(ordinary_identity()));

        std::cout << "snapshot_policy_follows_transports: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: PROVIZIO_DDS_ALLOW_VPN_INTERFACES puts tunnels back into change detection, and
    // nothing else with them. The variable is set by the ctest registration rather than
    // here, because it is parsed once per process on first use — which is inside the very
    // call being asserted on.
    //
    // This is the regression test for a divergence that shipped in the first draft of the
    // feature on two platforms: the override correctly bypassed the VPN filter, and the
    // interface was then dropped anyway by the macOS "utun" name prefix / the Windows
    // adapter-type gate. The transports bound it while change detection ignored it — the
    // one disagreement the two filters may never have.
    int test_snapshot_policy_honours_override()
    {
        bool passed = EXPECT(!provizio::dds::detail::snapshot_policy_excludes_interface(tunnel_identity()));
        passed &= EXPECT(!provizio::dds::detail::snapshot_policy_excludes_interface(ordinary_identity()));
        // Still excluded: the override is about tunnels, not about the heuristics.
        passed &= EXPECT(provizio::dds::detail::snapshot_policy_excludes_interface(virtual_identity()));

        std::cout << "snapshot_policy_honours_override: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Not an assertion of its own: prints this build's classifier verdict for every name it
    // is given (or for a built-in sample when given none), so
    // test/python/vpn_classifier_parity_test.py can compare the two languages' answers over
    // one table of names it owns. Machine-readable and stable: one line per name,
    // "name|vpn_name|vpn_product|vpn_kind", with 0/1 verdicts.
    //
    // Takes the whole argument vector and gathers the names into one list, rather than being
    // handed a sub-range and branching between it and the sample. Written the other way --
    // `if (names.empty()) walk the sample; else walk names;` over a sub-range vector -- gcc
    // 11.4 at -O2 and -O3 enters the `names` loop unconditionally and dereferences the empty
    // vector's null _M_start, so every ubuntu-22.04 Release job segfaulted here while -O0,
    // -O1, gcc 12 and gcc 13 all ran it correctly (reproduced and fixed against that exact
    // toolchain in a container). One list walked once is both immune and simpler; please
    // don't "simplify" it back.
    // The loopback predicate the allowlist's netmask decision rests on, pinned on both sides
    // of the line Fast-DDS draws differently (IP4_LOCAL is an exact 127.0.0.1 compare).
    int test_loopback_address()
    {
        bool passed = true;
        for (const auto *const loopback : {"127.0.0.1", "127.0.0.2", "127.0.1.1", "127.255.255.254", "::1"})
        {
            if (!provizio::dds::detail::is_loopback_address(loopback))
            {
                std::cerr << "FAIL: " << loopback << " was not classified as loopback\n";
                passed = false;
            }
        }
        for (const auto *const ordinary : {"192.0.2.1", "10.0.0.1", "12.7.0.1", "1270.0.0.1", "::1:1", "fe80::1", ""})
        {
            if (provizio::dds::detail::is_loopback_address(ordinary))
            {
                std::cerr << "FAIL: '" << ordinary << "' was classified as loopback\n";
                passed = false;
            }
        }
        std::cout << "loopback_address: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Machine-readable "<address>|<0/1>" per address given (or a built-in sample), for
    // test/python/vpn_classifier_parity_test.py to compare with _is_loopback_address.
    int test_loopback_table(const std::vector<std::string_view> &args)
    {
        static constexpr std::array<std::string_view, 4> sample{"127.0.0.1", "127.0.0.2", "192.0.2.1", "::1"};
        std::vector<std::string_view> addresses;
        for (std::size_t index = 2; index < args.size(); ++index)
        {
            addresses.push_back(args[index]);
        }
        if (addresses.empty())
        {
            addresses.assign(sample.begin(), sample.end());
        }
        for (const auto &address : addresses)
        {
            std::cout << address << "|" << (provizio::dds::detail::is_loopback_address(address) ? 1 : 0) << "\n";
        }
        return 0;
    }

    int test_classifier_table(const std::vector<std::string_view> &args)
    {
        static constexpr std::array<std::string_view, 6> sample{
            "tailscale0", "eth0", "wireguard", "TAP-Windows Adapter V9", "ztppmkbrz2", "br-lan",
        };

        // Everything after the subcommand, which the parity test fills with its own table.
        std::vector<std::string_view> names;
        for (std::size_t index = 2; index < args.size(); ++index)
        {
            names.push_back(args[index]);
        }
        if (names.empty())
        {
            names.assign(sample.begin(), sample.end());
        }

        for (const auto &name : names)
        {
            const std::string text{name};
            const bool is_kind =
#if defined(__linux__)
                provizio::dds::detail::is_vpn_interface_kind(text);
#else
                // Only Linux has an interface "kind"; reported as 0 elsewhere so the
                // parity test can compare the same three columns on every platform.
                false;
#endif
            // Flushed per line rather than left to the stream's buffering: stdout is a pipe
            // when ctest captures it, so anything still buffered is lost if the process dies
            // -- and this subcommand exists to be read after the fact. One flush per name
            // costs nothing at this scale and means the table always shows how far it got.
            std::cout << text << "|" << (provizio::dds::detail::is_vpn_interface_name(text) ? 1 : 0) << "|"
                      << (provizio::dds::detail::is_vpn_product_name(text) ? 1 : 0) << "|"
                      << (provizio::dds::detail::is_vpn_description(text) ? 1 : 0) << "|" << (is_kind ? 1 : 0) << "\n";
        }
        return 0;
    }

    // Case: an interface the transports refuse to bind must not drive
    // network-recovery either — its address churn (a Tailscale re-auth, a VPN
    // reconnect) can no longer change any locator, so rebuilding every participant
    // over it is pure disruption. The two filters have to model the same set.
    int test_snapshot_excludes_vpn()
    {
        unset_env(k_allow_env_name);

        const auto vpn_addresses = provizio::dds::detail::vpn_interface_blocklist_entries();
        const auto snapshot = provizio::dds::detail::capture_address_snapshot();

        bool passed = true;
        for (const auto &entry : snapshot)
        {
            passed &= EXPECT(vpn_addresses.find(entry.address_text) == vpn_addresses.end());
        }

        std::cout << "snapshot_excludes_vpn: " << (passed ? "PASS" : "FAIL") << " (" << snapshot.size()
                  << " snapshot entry(ies), " << vpn_addresses.size() << " VPN address(es)";
        if (vpn_addresses.empty())
        {
            std::cout << "; host has no VPN interface up, empty case only";
        }
        std::cout << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: the override puts VPN interfaces back into change detection as well —
    // once they carry locators again, their churn matters again.
    int test_snapshot_honours_override()
    {
        // Set before the first read, as in test_blocklist_override_named.
        set_env(k_allow_env_name, "1");

        const auto vpn_addresses = host_vpn_addresses_by_name();
        if (vpn_addresses.empty())
        {
            // Nothing to assert on this host — the case is the LIVE half of the override
            // check, and snapshot_policy_honours_override is the half that runs everywhere.
            // Said explicitly, because a bare PASS here reads as coverage that did not
            // happen.
            std::cout << "snapshot_honours_override: PASS (nothing to check: no VPN interface is up on this "
                         "host; snapshot_policy_honours_override covers the same rule without one)"
                      << '\n';
            return 0;
        }

        const auto snapshot = provizio::dds::detail::capture_address_snapshot();

        bool passed = true;
        for (const auto &address : vpn_addresses)
        {
            bool found = false;
            for (const auto &entry : snapshot)
            {
                found = found || entry.address_text == address;
            }
            passed &= EXPECT(found);
        }

        std::cout << "snapshot_honours_override: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }
}  // namespace

int main(int argc, char **argv)
{
    const std::vector<std::string_view> args(
        argv, std::next(argv, argc));  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    if (args.size() < 2)
    {
        std::cerr << "usage: " << args[0] << " <subcommand>\n";
        return 1;
    }
    const std::string_view subcommand = args[1];

    // Hermetic: an ambient FASTDDS_DEFAULT_PROFILES_FILE makes the library defer
    // entirely to that profile, transports included, so these cases would assert
    // against settings that were never applied (see domain_participant.cpp).
    unset_env("FASTDDS_DEFAULT_PROFILES_FILE");
    // Unsetting that variable is not enough on its own: Fast-DDS also auto-loads a
    // DEFAULT_FASTDDS_PROFILES.xml from the working directory, which counts as the caller's
    // profile just the same. A developer who happens to have one there would see these cases
    // fail for a reason that has nothing to do with the code under test, so the auto-load is
    // turned off rather than hoped against.
    set_env("SKIP_DEFAULT_XML_FILE", "1");
    // Same for a user-chosen transport set: with FASTDDS_BUILTIN_TRANSPORTS set,
    // Fast-DDS builds the transports and there are no descriptors of ours to
    // blocklist.
    unset_env("FASTDDS_BUILTIN_TRANSPORTS");

    if (subcommand == "classifier")
    {
        return test_classifier();
    }
    if (subcommand == "blocklist")
    {
        return test_blocklist();
    }
    if (subcommand == "blocklist_forced")
    {
        return test_blocklist_forced();
    }
    if (subcommand == "blocklist_override_all")
    {
        return test_blocklist_override_all();
    }
    if (subcommand == "blocklist_override_named")
    {
        return test_blocklist_override_named();
    }
#if !defined(_WIN32)
    if (subcommand == "blocklist_entries_from_walk")
    {
        return test_blocklist_entries_from_walk();
    }
#endif
    if (subcommand == "blocklist_allowed_enumeration_failure_changes_nothing")
    {
        return test_blocklist_allowed_enumeration_failure_changes_nothing();
    }
    if (subcommand == "blocklist_enumeration_failure_changes_nothing")
    {
        return test_blocklist_enumeration_failure_changes_nothing();
    }
    if (subcommand == "interface_kind_lookup_failure_reported")
    {
        return test_interface_kind_lookup_failure_reported();
    }
#if defined(__linux__)
    if (subcommand == "interface_kind_lookup_failure_warns")
    {
        return test_interface_kind_lookup_failure_warns();
    }
#endif
    if (subcommand == "blocklist_participant_netmask_off")
    {
        return test_blocklist_participant_netmask_off();
    }
    if (subcommand == "blocklist_override_unmatched")
    {
        return test_blocklist_override_unmatched();
    }
    if (subcommand == "blocklist_override_all_matched")
    {
        return test_blocklist_override_all_matched();
    }
    if (subcommand == "leaves_caller_transports_alone")
    {
        return test_leaves_caller_transports_alone();
    }
    if (subcommand == "no_op_with_builtin_transports_env")
    {
        return test_no_op_with_builtin_transports_env();
    }
    if (subcommand == "snapshot_excludes_vpn")
    {
        return test_snapshot_excludes_vpn();
    }
    if (subcommand == "snapshot_honours_override")
    {
        return test_snapshot_honours_override();
    }
    if (subcommand == "snapshot_policy_excludes_tunnel")
    {
        return test_snapshot_policy_excludes_tunnel();
    }
    if (subcommand == "snapshot_policy_follows_transports")
    {
        return test_snapshot_policy_follows_transports();
    }

    if (subcommand == "snapshot_policy_honours_override")
    {
        return test_snapshot_policy_honours_override();
    }
    if (subcommand == "classifier_table")
    {
        return test_classifier_table(args);
    }
    if (subcommand == "loopback_address")
    {
        return test_loopback_address();
    }
    if (subcommand == "loopback_table")
    {
        return test_loopback_table(args);
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
