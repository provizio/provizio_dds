// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy of
// the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.
//
// Guards cmake/fast_dds/local_address_by_current_interfaces.cmake, which makes
// UDPTransportInterface::transform_remote_locator -- asked by Fast-DDS whether an address a
// participant of this host announces is this host's, and so to be reached over loopback when that
// participant accepts localhost --
// refresh Fast-DDS's interface list when the list lacks the address, rate-limited, instead of
// answering from the list as it was last refreshed.
//
// A process whose last participant was created before the network came up holds a list without
// the host's addresses, and an unprivileged test cannot bring an interface up to produce that.
// It can empty the list, though, which is the very state such a process is in, and which
// network_recovery_test.cpp's reset_refreshes_fastdds_interface_cache already does. Then, on a UDPv4
// transport confined to 127.0.0.1 as transport_mode::localhost_only builds it:
//   - the host's own address is turned into 127.0.0.1 (unpatched, it was left as it is, and such a
//     transport could not use it);
//   - an address of no interface of the host, and a multicast address, are left as they are, the
//     multicast one without refreshing the list;
//   - a stream of addresses the host does not have refreshes the list at most as often as the rate
//     limit allows, and once the limit has refilled, the host's own address is recognised again.
//
// Built only where Fast-DDS is built from source, since that is the only configuration the patch
// is applied in -- see the CMakeLists -- and not on Windows, whose fastdds.dll exports none of
// SystemInfo's data members (only update_interfaces, through export_system_info.cmake), so the
// list cannot be emptied there.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fastdds/rtps/common/Locator.hpp>
#include <fastdds/rtps/transport/TransportInterface.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/utils/IPFinder.hpp>
#include <fastdds/utils/IPLocator.hpp>

// The members of Fast-DDS' internal SystemInfo this needs: the cached interface list, the flag
// saying it is cached, and the mutex guarding them. Declared rather than included, as
// network_recovery_test.cpp declares them: they are in libfastdds's default-visibility export table
// on Linux and macOS, and only the mangled names have to match.
namespace eprosima
{
    // NOLINTBEGIN(readability-identifier-naming,cppcoreguidelines-special-member-functions,cppcoreguidelines-avoid-non-const-global-variables)
    // -- the names must match Fast-DDS' class verbatim for the static members to resolve.
    class SystemInfo
    {
      public:
        static bool cached_interfaces_;
        static std::vector<eprosima::fastdds::rtps::IPFinder::info_IP> interfaces_;
        static std::mutex interfaces_mtx_;
    };
    // NOLINTEND(readability-identifier-naming,cppcoreguidelines-special-member-functions,cppcoreguidelines-avoid-non-const-global-variables)
}  // namespace eprosima

namespace
{
    using eprosima::fastdds::rtps::IPFinder;
    using eprosima::fastdds::rtps::IPLocator;
    using eprosima::fastdds::rtps::Locator_t;

    constexpr std::uint32_t test_port = 7412;
    constexpr int skip_return_code = 77;

    // The rate limit the patch sets: a burst of this many refreshes...
    constexpr double refresh_burst = 8.0;
    // ...then this many a second
    constexpr double refreshes_per_second = 20.0;
    // How many addresses the host does not have are asked about in a row
    constexpr int unknown_address_count = 400;
    // Addresses of TEST-NET-3 (RFC 5737), which no host has: 203.0.113.<1..last_test_net_host>
    constexpr int last_test_net_host = 250;
    // Long enough for the limit to refill a token many times over
    constexpr std::chrono::milliseconds refill_wait{600};

    int fail(const std::string &reason)
    {
        std::cout << "local_address: FAIL (" << reason << ")" << '\n';
        return 1;
    }

    // Empties Fast-DDS's interface list, keeping it marked as cached: the state of a process whose
    // last participant was created before any interface had carrier
    void forget_interfaces()
    {
        const std::lock_guard<std::mutex> lock{eprosima::SystemInfo::interfaces_mtx_};
        eprosima::SystemInfo::interfaces_.clear();
        eprosima::SystemInfo::cached_interfaces_ = true;
    }

    // Whether the list holds anything, which after forget_interfaces() means it was refreshed since
    bool interfaces_listed()
    {
        const std::lock_guard<std::mutex> lock{eprosima::SystemInfo::interfaces_mtx_};
        return !eprosima::SystemInfo::interfaces_.empty();
    }

    Locator_t udpv4_locator(const std::string &address)
    {
        Locator_t locator;
        locator.kind = LOCATOR_KIND_UDPv4;
        IPLocator::setIPv4(locator, address);
        locator.port = test_port;
        return locator;
    }

    // What <transport> makes of <remote>, as announced by a participant of this host that accepts
    // localhost: whether it may be used, and as what
    struct transformed
    {
        bool usable{false};
        Locator_t locator;
    };

    transformed transform(const eprosima::fastdds::rtps::TransportInterface &transport, const Locator_t &remote)
    {
        transformed result;
        result.usable = transport.transform_remote_locator(remote, result.locator, true, false);
        return result;
    }

    bool is_loopback_of(const transformed &result, const Locator_t &remote)
    {
        return result.usable && result.locator.kind == LOCATOR_KIND_UDPv4 && IPLocator::isLocal(result.locator) &&
               result.locator.port == remote.port;
    }

    bool is_unchanged(const transformed &result, const Locator_t &remote)
    {
        return result.usable && result.locator == remote;
    }
}  // namespace

int main()
{
    // An IPv4 address of this host's other than loopback, on an interface with carrier
    std::vector<IPFinder::info_IP> addresses;
    if (!IPFinder::getIPs(&addresses, false))
    {
        return fail("the host's interfaces could not be listed");
    }
    std::string own_address;
    for (const auto &address : addresses)
    {
        if (address.type == IPFinder::IP4)
        {
            own_address = address.name;
            break;
        }
    }
    if (own_address.empty())
    {
        std::cout << "local_address: SKIP (no interface with carrier has an IPv4 address besides loopback)" << '\n';
        return skip_return_code;
    }

    // A UDPv4 transport confined to loopback, as transport_mode::localhost_only builds one, while
    // the list still holds the host's interfaces, as a participant's is
    eprosima::fastdds::rtps::UDPv4TransportDescriptor descriptor;
    descriptor.interface_allowlist = {eprosima::fastdds::rtps::AllowedNetworkInterface{"127.0.0.1"}};
    const std::unique_ptr<eprosima::fastdds::rtps::TransportInterface> transport{descriptor.create_transport()};
    if (!transport)
    {
        return fail("no UDPv4 transport could be created");
    }

    // The host's own address, judged by a list without it: turned into loopback, the list refreshed
    const Locator_t own = udpv4_locator(own_address);
    forget_interfaces();
    if (!is_loopback_of(transform(*transport, own), own))
    {
        return fail(own_address + ", an address of this host, was not turned into 127.0.0.1 with the interface list "
                                  "emptied, as a process created before the network came up has it");
    }
    if (!interfaces_listed())
    {
        return fail("the interface list was not refreshed for an address it lacked");
    }

    // An address of no interface of the host: left as it is
    const Locator_t elsewhere = udpv4_locator("203.0.113.77");
    forget_interfaces();
    if (!is_unchanged(transform(*transport, elsewhere), elsewhere))
    {
        return fail("203.0.113.77, an address this host does not have, was transformed");
    }

    // A multicast address: left as it is, and no reason to refresh the list
    const Locator_t multicast = udpv4_locator("239.255.0.1");
    forget_interfaces();
    if (!is_unchanged(transform(*transport, multicast), multicast))
    {
        return fail("239.255.0.1, a multicast address, was transformed");
    }
    if (interfaces_listed())
    {
        return fail("the interface list was refreshed for a multicast address");
    }

    // A stream of addresses the host does not have, each judged by an emptied list: the list is
    // refreshed no more often than the limit allows -- however many it names -- and some are declined
    int refreshes = 0;
    const auto stream_start = std::chrono::steady_clock::now();
    for (int index = 0; index < unknown_address_count; ++index)
    {
        const Locator_t unknown = udpv4_locator("203.0.113." + std::to_string(1 + (index % last_test_net_host)));
        forget_interfaces();
        if (!is_unchanged(transform(*transport, unknown), unknown))
        {
            return fail("an address this host does not have was transformed");
        }
        if (interfaces_listed())
        {
            ++refreshes;
        }
    }
    const double stream_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - stream_start).count();
    // The bucket holds at most a burst and refills at the rate; the one extra token is rounding
    const double allowed = refresh_burst + std::ceil(refreshes_per_second * stream_seconds) + 1.0;
    std::cout << "local_address: " << refreshes << " refresh(es) for " << unknown_address_count
              << " unknown address(es) in " << stream_seconds << " s (at most " << allowed << " allowed)" << '\n';
    if (static_cast<double>(refreshes) > allowed)
    {
        return fail("the interface list was refreshed more often than the rate limit allows");
    }
    if (refreshes >= unknown_address_count)
    {
        return fail("no refresh was declined, so the rate limit did not apply");
    }

    // Once the limit has refilled, the host's own address is recognised again from an emptied list
    std::this_thread::sleep_for(refill_wait);
    forget_interfaces();
    if (!is_loopback_of(transform(*transport, own), own))
    {
        return fail(own_address + " was not turned into 127.0.0.1 once the rate limit had refilled");
    }

    std::cout << "local_address: PASS (" << own_address << " turned into 127.0.0.1 from an emptied interface list)"
              << '\n';
    return 0;
}
