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
//
// Subcommand-driven tests for participant transport tuning: the MTU-sized
// default of the send-side RTPS message-size cap (the fastdds.max_message_size
// participant property, which keeps every UDP datagram within a single link
// frame so large samples travel as individually-retransmittable RTPS fragments
// on lossy networks), the PROVIZIO_DDS_MAX_MESSAGE_SIZE environment override,
// the fallback to the default on malformed input, and the loopback confinement
// of transport_mode::localhost_only. Mostly asserted by reading the configured
// QoS back off a freshly created participant; the localhost_only_round_trip case
// also sends a sample, because an interface allowlist has to leave discovery and
// delivery working and not merely look right in the QoS. Each ctest entry runs
// the same binary with a different subcommand so per-case failure stays
// isolated, mirroring the discovery_tuning test.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/subscriber.h"

#if defined(__linux__)
#include <arpa/inet.h>
#endif

// A pid is wanted on every platform, so this is deliberately NOT part of the Linux-only
// block above -- that one guards the /proc socket reading. Taking <unistd.h> in
// transitively on Linux is exactly what let the uses below compile there and nowhere else.
// Windows spells the function _getpid() in <process.h>, the unprefixed name being
// deprecated there.
#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/SocketTransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>

#include <std_msgs/msg/StringPubSubTypes.hpp>

namespace
{
    /// @brief This process' id, spelled the way the platform spells it.
    int current_process_id()
    {
#if defined(_WIN32)
        return _getpid();
#else
        return static_cast<int>(::getpid());
#endif
    }

    constexpr auto k_domain = 0;
    // The case that carries real traffic gets a domain derived from the pid rather than the
    // shared default: on a self-hosted runner domain 0 carries every other suite's
    // participants, and a loopback-confined participant is reachable by all of them because
    // they are on the same host. 100..127 mirrors the range the request/response suite
    // already spreads itself over.
    const auto k_traffic_domain = static_cast<provizio::dds::DomainId_t>(100 + (current_process_id() % 28));

    // The expected default (kept in sync with src/domain_participant.cpp and
    // python/provizio_dds.py): one ~1500-byte-MTU link frame per UDP datagram.
    constexpr std::string_view k_default_max_message_size = "1400";
    constexpr const char *k_property_name = "fastdds.max_message_size";
    constexpr const char *k_env_name = "PROVIZIO_DDS_MAX_MESSAGE_SIZE";

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

    // The fastdds.max_message_size property read back off a participant's
    // effective QoS, or "" when absent.
    std::string read_max_message_size(provizio::dds::domain_participant &participant)
    {
        const auto fdds = participant.fastdds_participant();
        for (const auto &property : fdds->get_qos().properties().properties())
        {
            if (property.name() == k_property_name)
            {
                return property.value();
            }
        }
        return {};
    }

    std::string max_message_size_of_fresh_participant()
    {
        const auto participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        return read_max_message_size(*participant);
    }

    // Case: with no env override a participant must be configured with the
    // MTU-sized default cap, NOT Fast-DDS's 65500 single-datagram default (which
    // makes large-sample delivery all-or-nothing under frame loss).
    int test_defaults()
    {
        // Hermetic: ignore any PROVIZIO_DDS_MAX_MESSAGE_SIZE override set in the ambient environment.
        unset_env(k_env_name);

        const auto value = max_message_size_of_fresh_participant();
        const bool passed = EXPECT(value == k_default_max_message_size);
        std::cout << "defaults: " << (passed ? "PASS" : "FAIL") << " (" << k_property_name << "=" << value << ")"
                  << '\n';
        return passed ? 0 : 1;
    }

    // Case: a valid PROVIZIO_DDS_MAX_MESSAGE_SIZE env value is honoured verbatim
    // (e.g. raised back to Fast-DDS's 65500 maximum by hosts publishing multi-MB
    // samples over clean links, to trade loss resilience back for CPU), down to
    // the 576-byte floor below which discovery announcements no longer fit.
    int test_env_override()
    {
        bool passed = true;

        set_env(k_env_name, "65500");
        const auto max_value = max_message_size_of_fresh_participant();
        passed &= EXPECT(max_value == "65500");

        set_env(k_env_name, "576");  // the minimum accepted value
        const auto min_value = max_message_size_of_fresh_participant();
        passed &= EXPECT(min_value == "576");

        // Above Fast-DDS's 65500 maximum the value is clamped (with a warning), not
        // honoured verbatim and not rejected to the default: the likeliest cause is a
        // typo by someone who meant "as large as possible".
        set_env(k_env_name, "70000");
        const auto clamped_value = max_message_size_of_fresh_participant();
        passed &= EXPECT(clamped_value == "65500");

        std::cout << "env_override: " << (passed ? "PASS" : "FAIL") << " (" << k_property_name << "=" << max_value
                  << ", " << min_value << ", " << clamped_value << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Whether this platform's participants get shared memory from this library — the same
    // condition src/domain_participant.cpp applies (Windows and macOS are excluded over a
    // Boost.Interprocess cleanup bug in Fast-DDS's bundled copy).
    constexpr bool k_platform_allows_shared_memory =
#if defined(_WIN32) || defined(__APPLE__)
        false;
#else
        true;
#endif

    // Case: transport_mode::localhost_only confines the socket transports to loopback while
    // KEEPING shared memory wherever the platform allows it. Both halves matter and neither
    // implies the other: without the allowlist the domain would still be announced on every
    // interface, and dropping shared memory would give up the transport that is supposed to
    // carry the samples — while dropping UDP instead would leave such a participant with no
    // transport at all once the host's shared-memory space is exhausted.
    int test_localhost_only()
    {
        bool passed = true;

        const auto participant = provizio::dds::make_domain_participant(
            k_domain, provizio::dds::network_recovery_mode::off, {}, provizio::dds::endpoint_kind::data_writer,
            provizio::dds::transport_mode::localhost_only);

        std::size_t socket_transports = 0;
        std::size_t shared_memory_transports = 0;
        {
            const auto fdds = participant->fastdds_participant();
            for (const auto &descriptor : fdds->get_qos().transport().user_transports)
            {
                if (std::dynamic_pointer_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor>(descriptor))
                {
                    ++shared_memory_transports;
                    continue;
                }
                const auto socket_descriptor =
                    std::dynamic_pointer_cast<eprosima::fastdds::rtps::SocketTransportDescriptor>(descriptor);
                if (!socket_descriptor)
                {
                    continue;
                }
                ++socket_transports;
                // Exactly loopback and nothing else: an allowlist that also held a real
                // interface would put the domain back on the wire.
                passed &= EXPECT(socket_descriptor->interface_allowlist.size() == 1);
                if (socket_descriptor->interface_allowlist.size() == 1)
                {
                    passed &= EXPECT(socket_descriptor->interface_allowlist.front().name == "127.0.0.1");
                }
                // Left at AUTO, which never filters — with one allowed interface there is
                // nothing to disambiguate. Asserted because it is a deliberate choice and
                // the VPN blocklist path would set it to ON; not because ON would break
                // anything (Fast-DDS never netmask-filters a multicast destination). Note
                // this reads the descriptor's value before Fast-DDS transforms AUTO into the
                // participant-level setting, which is what the mode intends to leave alone.
                passed &= EXPECT(socket_descriptor->netmask_filter == eprosima::fastdds::rtps::NetmaskFilterKind::AUTO);
            }
        }

        // Guards against the assertions above passing vacuously on an empty transport list.
        passed &= EXPECT(socket_transports > 0);
        passed &= EXPECT(shared_memory_transports == (k_platform_allows_shared_memory ? 1U : 0U));

        // The contrast that proves the confinement is this mode's and not everyone's: a
        // default participant must still reach the network.
        const auto unconfined =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        {
            const auto fdds = unconfined->fastdds_participant();
            for (const auto &descriptor : fdds->get_qos().transport().user_transports)
            {
                const auto socket_descriptor =
                    std::dynamic_pointer_cast<eprosima::fastdds::rtps::SocketTransportDescriptor>(descriptor);
                if (socket_descriptor)
                {
                    // Not "no allowlist at all": on a host carrying a tunnel the VPN
                    // exclusion writes one, so that every interface it leaves behind can be
                    // given the netmask filter it needs. What must never be true of a
                    // participant that did not ask for containment is that the list confines
                    // it to loopback.
                    const auto &allowlist = socket_descriptor->interface_allowlist;
                    const bool confined_to_loopback =
                        !allowlist.empty() &&
                        std::all_of(allowlist.begin(), allowlist.end(),
                                    [](const eprosima::fastdds::rtps::AllowedNetworkInterface &entry) {
                                        return entry.name.rfind("127.", 0) == 0 || entry.name == "::1";
                                    });
                    passed &= EXPECT(!confined_to_loopback);
                }
            }
        }

        std::cout << "localhost_only: " << (passed ? "PASS" : "FAIL") << " (socket=" << socket_transports
                  << ", shm=" << shared_memory_transports << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: a transport selection this library could not deliver is reported, not swallowed.
    // With FASTDDS_BUILTIN_TRANSPORTS set, Fast-DDS builds the transports and none of the
    // descriptors are ours, so localhost_only confines nothing — verified below by there
    // being no descriptor of ours at all. That matters more than a mode being ignored
    // usually would: localhost_only is a containment promise ("no remote peer can join"),
    // and a caller who thinks they hold it while the domain is on every interface is worse
    // off than one who was told.
    int test_localhost_only_not_applied_warns()
    {
        set_env("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4");

        std::vector<std::string> warnings;
        std::mutex warnings_mutex;
        auto previous = provizio::dds::set_log_callback(
            [&warnings, &warnings_mutex](const provizio::dds::log_level level, const std::string_view message) {
                if (level == provizio::dds::log_level::warning)
                {
                    const std::lock_guard<std::mutex> lock{warnings_mutex};
                    warnings.emplace_back(message);
                }
            });

        std::size_t own_socket_transports = 0;
        {
            const auto participant = provizio::dds::make_domain_participant(
                k_domain, provizio::dds::network_recovery_mode::off, {}, provizio::dds::endpoint_kind::data_writer,
                provizio::dds::transport_mode::localhost_only);

            const auto fdds = participant->fastdds_participant();
            for (const auto &descriptor : fdds->get_qos().transport().user_transports)
            {
                if (std::dynamic_pointer_cast<eprosima::fastdds::rtps::SocketTransportDescriptor>(descriptor))
                {
                    ++own_socket_transports;
                }
            }
        }

        provizio::dds::set_log_callback(std::move(previous));
        unset_env("FASTDDS_BUILTIN_TRANSPORTS");

        bool warned = false;
        {
            const std::lock_guard<std::mutex> lock{warnings_mutex};
            for (const auto &message : warnings)
            {
                warned = warned || (message.find("transport=localhost_only") != std::string::npos &&
                                    message.find("NOT confined to this host") != std::string::npos &&
                                    message.find("Fast-DDS built the transports itself") != std::string::npos);
            }
        }
        bool passed = EXPECT(warned);
        // Why the warning is not merely tidy: Fast-DDS built the transports from the
        // variable, so there is no descriptor of ours to hold an allowlist and the domain is
        // on every interface. Asserted so that a future change which DID apply the mode here
        // shows up as this case failing rather than as a stale warning.
        passed &= EXPECT(own_socket_transports == 0);

        std::cout << "localhost_only_not_applied_warns: " << (passed ? "PASS" : "FAIL") << " (" << own_socket_transports
                  << " socket descriptor(s) of ours)" << '\n';
        return passed ? 0 : 1;
    }

    // Case: transport_mode::udp_only is judged the same way -- by whether shared memory is
    // actually in the set, not by who chose the transports. FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    // hands Fast-DDS' own descriptors to the participant, none of them this library's, and
    // delivers precisely what the mode asked for: nothing to report. DEFAULT is the same
    // ownership and the opposite outcome, and that is the one worth a line.
    int test_udp_only_shared_memory_decides_warning()
    {
        std::vector<std::string> warnings;
        std::mutex warnings_mutex;
        auto previous = provizio::dds::set_log_callback(
            [&warnings, &warnings_mutex](const provizio::dds::log_level level, const std::string_view message) {
                if (level == provizio::dds::log_level::warning)
                {
                    const std::lock_guard<std::mutex> lock{warnings_mutex};
                    warnings.emplace_back(message);
                }
            });

        // One process can exercise both: unlike the Python layer, where the variable is
        // process-global and pinned by the first participant, C++ reads it per participant
        // inside setup_transports.
        const auto warned_about_the_mode = [&warnings, &warnings_mutex]() {
            const std::lock_guard<std::mutex> lock{warnings_mutex};
            for (const auto &message : warnings)
            {
                if (message.find("transport=udp_only") != std::string::npos)
                {
                    return true;
                }
            }
            return false;
        };

        // Each value below is judged on its own. Warnings accumulate across participants, so
        // a phase that follows a warning would otherwise inherit it and assert nothing at
        // all -- which is precisely how the wrong-case phase first passed against the very
        // bug it was written for.
        const auto clear_warnings = [&warnings, &warnings_mutex]() {
            const std::lock_guard<std::mutex> lock{warnings_mutex};
            warnings.clear();
        };

        const auto make_udp_only_participant = [] {
            return provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off, {},
                                                          provizio::dds::endpoint_kind::data_writer,
                                                          provizio::dds::transport_mode::udp_only);
        };

        set_env("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4");
        std::size_t shared_memory_transports = 0;
        {
            const auto participant = make_udp_only_participant();
            const auto fdds = participant->fastdds_participant();
            // Vacuity guard, and the reason the silence is right: Fast-DDS built the
            // transports from the variable, so there is no descriptor of ours here at all --
            // and no shared memory either, which is the whole of what this mode excludes.
            for (const auto &descriptor : fdds->get_qos().transport().user_transports)
            {
                if (std::dynamic_pointer_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor>(descriptor))
                {
                    ++shared_memory_transports;
                }
            }
        }
        const bool warned_without_shared_memory = warned_about_the_mode();
        bool passed = EXPECT(!warned_without_shared_memory);
        passed &= EXPECT(shared_memory_transports == 0);

        clear_warnings();
        set_env("FASTDDS_BUILTIN_TRANSPORTS", "DEFAULT");
        {
            const auto participant = make_udp_only_participant();
            (void)participant->fastdds_participant();
        }
        const bool warned_with_shared_memory = warned_about_the_mode();
        passed &= EXPECT(warned_with_shared_memory);

        // The same selection in the wrong case, which is NOT the same selection. Fast-DDS
        // compares the token with strcmp (get_element_enum_value) and does not reject what
        // fails to match: it logs "Wrong value ... Leaving as DEFAULT" and builds SHM +
        // UDPv4. So this participant has shared memory, udp_only was not honoured, and
        // saying so is the point -- a case-insensitive reading here would call it UDPv4 and
        // stay silent on the one input that needs the line.
        //
        // What Fast-DDS built cannot be counted from here, which is why the assertion is the
        // warning itself: transports built from the variable live in the RTPS attributes and
        // never reach the DomainParticipantQos, so user_transports is empty for this
        // participant exactly as it is for the UDPv4 one above.
        clear_warnings();
        set_env("FASTDDS_BUILTIN_TRANSPORTS", "udpv4");
        {
            const auto participant = make_udp_only_participant();
            (void)participant->fastdds_participant();
        }
        const bool warned_on_wrong_case = warned_about_the_mode();
        passed &= EXPECT(warned_on_wrong_case);

        unset_env("FASTDDS_BUILTIN_TRANSPORTS");
        provizio::dds::set_log_callback(std::move(previous));

        std::cout << "udp_only_shared_memory_decides_warning: " << (passed ? "PASS" : "FAIL")
                  << " (UDPv4: " << (warned_without_shared_memory ? "warned" : "silent")
                  << ", DEFAULT: " << (warned_with_shared_memory ? "warned" : "silent")
                  << ", udpv4 (wrong case): " << (warned_on_wrong_case ? "warned" : "silent") << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: transport_mode::localhost_only keeps TWO promises -- nothing leaves this host, and
    // no remote peer can join -- and the second one is the quiet half. The loopback initial peer
    // and ParticipantFilteringFlags::FILTER_DIFFERENT_HOST are applied where this library builds
    // the QoS, but a participant created from an XML profile of the caller's is created with
    // PARTICIPANT_QOS_DEFAULT, so Fast-DDS resolves everything from that profile and neither
    // reaches it. Confined transports are not enough on their own: a whitelisted UDPv4 transport
    // still opens a socket bound to the multicast group address, which the kernel does not filter
    // by joining interface, so a LAN announcement can still be parsed and held as a peer.
    //
    // The fixture is this repository's own loopback profile, which confines its transports and
    // sets no ignoreParticipantFlags -- so before this check the caller was told nothing at all.
    int test_localhost_only_discovery_filter_warns()
    {
        std::vector<std::string> warnings;
        std::mutex warnings_mutex;
        auto previous = provizio::dds::set_log_callback(
            [&warnings, &warnings_mutex](const provizio::dds::log_level level, const std::string_view message) {
                if (level == provizio::dds::log_level::warning)
                {
                    const std::lock_guard<std::mutex> lock{warnings_mutex};
                    warnings.emplace_back(message);
                }
            });

        bool passed = true;
        {
            const auto participant = provizio::dds::make_domain_participant(
                k_domain, provizio::dds::network_recovery_mode::off, {}, provizio::dds::endpoint_kind::data_writer,
                provizio::dds::transport_mode::localhost_only);
            const auto fdds = participant->fastdds_participant();

            // Vacuity guard: the point of the case is that the TRANSPORTS are confined, so a run
            // where the profile did not reach the participant would assert nothing.
            const auto &discovery = fdds->get_qos().wire_protocol().builtin.discovery_config;
            passed &= EXPECT((static_cast<std::uint32_t>(discovery.ignoreParticipantFlags) &
                              static_cast<std::uint32_t>(
                                  eprosima::fastdds::rtps::ParticipantFilteringFlags::FILTER_DIFFERENT_HOST)) == 0U);
        }

        bool warned_about_discovery = false;
        {
            const std::lock_guard<std::mutex> lock{warnings_mutex};
            for (const auto &message : warnings)
            {
                if (message.find("transport=localhost_only") != std::string::npos &&
                    message.find("discovery layer") != std::string::npos)
                {
                    warned_about_discovery = true;
                }
            }
        }
        passed &= EXPECT(warned_about_discovery);

        provizio::dds::set_log_callback(std::move(previous));

        std::cout << "localhost_only_discovery_filter_warns: " << (passed ? "PASS" : "FAIL")
                  << " (discovery filter absent, warned: " << (warned_about_discovery ? "yes" : "no") << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: the report above fires on a broken PROMISE, never on a configuration it merely
    // did not get to build itself. A caller's own descriptor that is already confined to
    // loopback delivers exactly what transport_mode::localhost_only asks for, so there is
    // nothing to tell them, while the same descriptor left binding every interface is the
    // real thing to say out loud. Both halves run here, through one code path, so a check
    // that went back to keying on who owns the transports fails the first half and one that
    // stopped looking at all fails the second.
    int test_localhost_only_caller_confinement_decides_warning()
    {
        std::vector<std::string> warnings;
        std::mutex warnings_mutex;
        auto previous = provizio::dds::set_log_callback(
            [&warnings, &warnings_mutex](const provizio::dds::log_level level, const std::string_view message) {
                if (level == provizio::dds::log_level::warning)
                {
                    const std::lock_guard<std::mutex> lock{warnings_mutex};
                    warnings.emplace_back(message);
                }
            });

        // Descriptors reaching a participant the one way nothing in the environment reveals:
        // the process-wide default QoS. Where an XML profile would do just as well, this
        // keeps the case to one process and one file.
        auto *const factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
        // How the caller spelled their interface list, since Fast-DDS accepts more than one
        // spelling and the judgement has to read every one it accepts.
        enum class caller_transport
        {
            unconfined,             ///< No interface list: binds every interface Fast-DDS finds.
            allowlist_loopback,     ///< interface_allowlist = {127.0.0.1}.
            whitelist_loopback,     ///< The deprecated interfaceWhiteList = {127.0.0.1}, which
                                    ///< Fast-DDS still honours -- and which
                                    ///< test/fast_dds_localhost_profile.xml uses.
            allowlist_device_name,  ///< interface_allowlist = {"eth0"}: a real device, and not
                                    ///< loopback, so the containment promise is NOT kept.
        };

        const auto set_caller_transport = [factory](const caller_transport spelling) {
            eprosima::fastdds::dds::DomainParticipantQos qos;
            factory->get_default_participant_qos(qos);
            qos.transport().user_transports.clear();
            auto descriptor = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
            switch (spelling)
            {
            case caller_transport::unconfined:
                break;
            case caller_transport::allowlist_loopback:
                descriptor->interface_allowlist.emplace_back("127.0.0.1");
                break;
            case caller_transport::whitelist_loopback:
                descriptor->interfaceWhiteList.emplace_back("127.0.0.1");
                break;
            case caller_transport::allowlist_device_name:
                descriptor->interface_allowlist.emplace_back("eth0");
                break;
            }
            qos.transport().user_transports.push_back(descriptor);
            factory->set_default_participant_qos(qos);
        };

        const auto warned_about_the_mode = [&warnings, &warnings_mutex]() {
            const std::lock_guard<std::mutex> lock{warnings_mutex};
            for (const auto &message : warnings)
            {
                if (message.find("transport=localhost_only") != std::string::npos)
                {
                    return true;
                }
            }
            return false;
        };

        // Each spelling below is judged on its own: warnings accumulate across participants,
        // so without this a later phase would inherit an earlier phase's verdict and every
        // "silent" assertion after the first warning would be unfalsifiable.
        const auto clear_warnings = [&warnings, &warnings_mutex]() {
            const std::lock_guard<std::mutex> lock{warnings_mutex};
            warnings.clear();
        };

        const auto make_confined_participant = [] {
            return provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off, {},
                                                          provizio::dds::endpoint_kind::data_writer,
                                                          provizio::dds::transport_mode::localhost_only);
        };

        set_caller_transport(caller_transport::allowlist_loopback);
        std::size_t caller_descriptors = 0;
        {
            const auto participant = make_confined_participant();
            // Vacuity guard: silence proves nothing unless the caller's descriptor really
            // did reach the participant, so it is counted by the property that identifies
            // it -- an allowlist this library never writes on its own is one entry long and
            // holds the loopback address, and ours are indistinguishable from it, so what is
            // asserted is simply that a socket transport exists to have been judged.
            const auto fdds = participant->fastdds_participant();
            for (const auto &descriptor : fdds->get_qos().transport().user_transports)
            {
                if (std::dynamic_pointer_cast<eprosima::fastdds::rtps::SocketTransportDescriptor>(descriptor))
                {
                    ++caller_descriptors;
                }
            }
        }
        bool passed = EXPECT(caller_descriptors >= 2);  // The caller's, plus the one of ours.
        const bool warned_when_confined = warned_about_the_mode();
        passed &= EXPECT(!warned_when_confined);

        // The same confinement, written the way Fast-DDS' older spelling writes it. Silence
        // is the whole assertion: UDPv4Transport builds its whitelist from interfaceWhiteList
        // too, so a participant confined this way IS contained, and telling its caller
        // otherwise -- which reading interface_allowlist alone does -- is a false alarm on the
        // exact shape test/fast_dds_localhost_profile.xml ships.
        clear_warnings();
        set_caller_transport(caller_transport::whitelist_loopback);
        {
            const auto participant = make_confined_participant();
            (void)participant->fastdds_participant();
        }
        const bool warned_when_confined_by_whitelist = warned_about_the_mode();
        passed &= EXPECT(!warned_when_confined_by_whitelist);

        // A device name that is not loopback. The list is not empty, so nothing here can lean
        // on "no list at all"; what makes it a warning is the name itself, which no loopback
        // device answers to. Without that judgement the mode's containment promise fails in
        // silence -- the participant announces on eth0 and its caller is never told.
        clear_warnings();
        set_caller_transport(caller_transport::allowlist_device_name);
        {
            const auto participant = make_confined_participant();
            (void)participant->fastdds_participant();
        }
        const bool warned_when_confined_elsewhere = warned_about_the_mode();
        passed &= EXPECT(warned_when_confined_elsewhere);

        clear_warnings();
        set_caller_transport(caller_transport::unconfined);
        {
            const auto participant = make_confined_participant();
            (void)participant->fastdds_participant();
        }
        const bool warned_when_unconfined = warned_about_the_mode();
        passed &= EXPECT(warned_when_unconfined);

        eprosima::fastdds::dds::DomainParticipantQos restored;
        factory->get_default_participant_qos(restored);
        restored.transport().user_transports.clear();
        factory->set_default_participant_qos(restored);
        provizio::dds::set_log_callback(std::move(previous));

        std::cout << "localhost_only_caller_confinement_decides_warning: " << (passed ? "PASS" : "FAIL")
                  << " (allowlist 127.0.0.1: " << (warned_when_confined ? "warned" : "silent")
                  << ", interfaceWhiteList 127.0.0.1: " << (warned_when_confined_by_whitelist ? "warned" : "silent")
                  << ", allowlist eth0: " << (warned_when_confined_elsewhere ? "warned" : "silent")
                  << ", no list: " << (warned_when_unconfined ? "warned" : "silent") << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Every local address this process has a UDP socket bound to, read from /proc. Linux
    // only: it is the one place the confinement can be observed from inside the process
    // without privileges, and it observes the property the mode promises rather than the QoS
    // that is supposed to produce it. Returns std::nullopt where /proc is not available, so
    // a caller can tell "nothing bound off loopback" from "could not look".
    std::optional<std::vector<std::string>> own_udp_local_addresses()
    {
#if !defined(__linux__)
        return std::nullopt;
#else
        std::vector<std::string> addresses;
        // /proc/net/udp lists every UDP socket on the host with its inode; the ones belonging
        // to this process are those whose inode appears in /proc/self/fd. Matching by inode
        // rather than by "all sockets" is what keeps a neighbouring test's participant from
        // failing this one.
        std::unordered_set<std::string> own_socket_inodes;
        std::error_code error;
        for (const auto &entry : std::filesystem::directory_iterator{"/proc/self/fd", error})
        {
            std::error_code link_error;
            const auto target = std::filesystem::read_symlink(entry.path(), link_error);
            if (link_error)
            {
                continue;
            }
            const std::string text = target.string();
            // Sockets read as "socket:[<inode>]".
            constexpr std::string_view prefix{"socket:["};
            if (text.compare(0, prefix.size(), prefix) == 0 && text.back() == ']')
            {
                own_socket_inodes.insert(text.substr(prefix.size(), text.size() - prefix.size() - 1));
            }
        }
        if (error)
        {
            return std::nullopt;
        }

        std::ifstream udp{"/proc/net/udp"};
        if (!udp)
        {
            return std::nullopt;
        }
        std::string line;
        std::getline(udp, line);  // header
        while (std::getline(udp, line))
        {
            // Columns: sl local_address rem_address st tx_queue:rx_queue tr:when retrnsmt
            //          uid timeout inode ...
            std::istringstream fields{line};
            std::string index;
            std::string local;
            std::string remote;
            std::string state;
            std::string queues;
            std::string timer;
            std::string retransmit;
            std::string uid;
            std::string timeout;
            std::string inode;
            if (!(fields >> index >> local >> remote >> state >> queues >> timer >> retransmit >> uid >> timeout >>
                  inode))
            {
                continue;
            }
            if (own_socket_inodes.count(inode) == 0)
            {
                continue;
            }
            // local is "<hex address, little-endian>:<hex port>".
            const auto colon = local.find(':');
            if (colon == std::string::npos)
            {
                continue;
            }
            const auto packed = std::stoul(local.substr(0, colon), nullptr, 16);
            std::array<char, INET_ADDRSTRLEN> text{};
            const std::uint32_t address = static_cast<std::uint32_t>(packed);
            if (::inet_ntop(AF_INET, &address, text.data(), text.size()) != nullptr)
            {
                addresses.emplace_back(text.data());
            }
        }
        return addresses;
#endif
    }

    // Case: two localhost_only participants actually talk to each other, and the process
    // opens no socket outside loopback while they do. The configuration asserted above is
    // only half the claim — an interface allowlist also has to leave discovery working, and
    // confining UDP to loopback is exactly the kind of change that can silently break the
    // announcements discovery depends on.
    //
    // What delivery alone would NOT prove, on this platform: Fast-DDS defaults to
    // INTRAPROCESS_FULL, so two participants in one process exchange data without touching a
    // transport at all, and their discovery rides shared memory where this library keeps it.
    // The case therefore passes just as happily with the allowlist absent — measured. So it
    // also reads the process' own UDP sockets and requires every one of them to be on
    // 127.0.0.1, which is the property the mode actually promises and which nothing but the
    // allowlist provides. That half is Linux-only (it reads /proc/net/udp); on Windows and
    // macOS, where this library disables shared memory, delivery itself has to cross loopback
    // UDP, so the two halves cover each other.
    int test_localhost_only_round_trip()
    {
        using namespace std::chrono_literals;
        using string_pub_sub_type = std_msgs::msg::StringPubSubType;

        // Stamped with the pid, and run on a domain of its own below: two instances of this
        // case on one self-hosted runner would otherwise satisfy each other's assertion and
        // report PASS with the confinement broken. Same reason shm_cleanup stamps its names
        // and the vpn suite leaves domain 0 alone.
        const std::string topic{"provizio_dds_localhost_only_round_trip_" + std::to_string(current_process_id())};
        // Scaled like every other completion deadline in the suite (see
        // PROVIZIO_DDS_TEST_TIMEOUT_SCALE in test/CMakeLists.txt): this waits for discovery
        // and delivery, which a sanitized build slows down as much as it slows the ctest
        // ceiling around it.
        constexpr auto k_timeout = 30s * PROVIZIO_DDS_TEST_TIMEOUT_SCALE;

        const auto make_confined_participant = [] {
            return provizio::dds::make_domain_participant(k_traffic_domain, provizio::dds::network_recovery_mode::off,
                                                          {}, provizio::dds::endpoint_kind::data_writer,
                                                          provizio::dds::transport_mode::localhost_only);
        };

        std::atomic_bool received{false};
        auto sub_participant = make_confined_participant();
        auto pub_participant = make_confined_participant();
        // An eager RELIABLE reader rather than the match-publisher default, so a failure here
        // means "loopback confinement broke delivery" and not "deferred build hadn't settled".
        auto subscriber = provizio::dds::make_subscriber<string_pub_sub_type>(
            sub_participant, topic, [&received](const std_msgs::msg::String &) { received.store(true); },
            provizio::dds::RELIABLE_RELIABILITY_QOS);
        auto publisher = provizio::dds::make_publisher<string_pub_sub_type>(
            pub_participant, topic, provizio::dds::RELIABLE_RELIABILITY_QOS, provizio::dds::use_default_history_depth);

        // What this proves, and what it does not: the confinement leaves discovery and
        // delivery working. On a platform where this library keeps shared memory (Linux)
        // the sample most likely travels over it rather than over loopback UDP, so the
        // UDP-over-loopback path is exercised where shared memory is off — Windows and
        // macOS, which CI covers. Both are worth having: this case would catch an allowlist
        // that broke discovery outright on any platform.
        std_msgs::msg::String message;
        message.data("localhost-only");
        const auto deadline = std::chrono::steady_clock::now() + k_timeout;
        while (std::chrono::steady_clock::now() < deadline && !received.load())
        {
            // Republished in a loop rather than once after a settle wait: the first samples
            // predate discovery completing, and a VOLATILE writer does not repeat them.
            std::ignore = publisher->publish(message);
            std::this_thread::sleep_for(100ms);
        }

        bool passed = EXPECT(received.load());

        // The half that only the allowlist can satisfy: nothing of ours is bound off
        // loopback. Multicast group memberships show up as a socket bound to the group
        // address (239.255.0.1), which is loopback-joined here and is not an interface
        // address, so it is allowed through explicitly.
        std::size_t checked_addresses = 0;
        if (const auto addresses = own_udp_local_addresses())
        {
            for (const auto &address : *addresses)
            {
                if (address == "239.255.0.1")
                {
                    continue;
                }
                ++checked_addresses;
                passed &= EXPECT(address == "127.0.0.1");
                if (address != "127.0.0.1")
                {
                    std::cerr << "  bound off loopback: " << address << '\n';
                }
            }
            // Guards the loop against passing because the read found nothing.
            passed &= EXPECT(checked_addresses > 0);
        }
        else
        {
            std::cout << "localhost_only_round_trip: socket check skipped (no /proc)" << '\n';
        }

        std::cout << "localhost_only_round_trip: " << (passed ? "PASS" : "FAIL") << " (" << checked_addresses
                  << " bound address(es) checked)" << '\n';
        return passed ? 0 : 1;
    }

    // Case: malformed / out-of-range env values are ignored and the default wins
    // (so a typo can never silently disable the fragmentation cap).
    int test_env_invalid()
    {
        bool passed = true;
        // "-18446744073709551615" locks the explicit minus rejection (strtoull alone would
        // wrap it around to 1 and silently accept a 1-byte cap); "100" and "575" lock the
        // 576-byte floor below which discovery announcements no longer fit one message.
        for (const char *invalid :
             {"not-a-number", "0", "-5", "1400x", "99999999999999999999", "-18446744073709551615", "100", "575"})
        {
            set_env(k_env_name, invalid);
            const auto value = max_message_size_of_fresh_participant();
            passed &= EXPECT(value == k_default_max_message_size);
            if (value != k_default_max_message_size)
            {
                std::cerr << "  " << k_env_name << "='" << invalid << "' produced " << k_property_name << "=" << value
                          << '\n';
            }
        }
        std::cout << "env_invalid: " << (passed ? "PASS" : "FAIL") << '\n';
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

    // Hermetic across the whole suite: an ambient FASTDDS_DEFAULT_PROFILES_FILE pointing at a real XML
    // profile makes the library skip its code-driven transport tuning entirely (see domain_participant.cpp),
    // so these cases would assert against settings that were never applied. Clear it up front, regardless
    // of the developer's shell. Only the participant's own QoS is read back — no pub/sub traffic — so
    // leaving loopback confinement is safe even on hosts sharing a LAN with concurrent CI runs.
    //
    // Except for the one case that is ABOUT a caller's XML profile, which asks to keep it: the variable
    // has to be set before the factory reads it, so its ctest registration supplies both, and clearing it
    // here would leave that case asserting on the very configuration path it exists to avoid.
    // NOLINTNEXTLINE(concurrency-mt-unsafe): startup-only probe, single-threaded here
    if (std::getenv("PROVIZIO_DDS_TEST_KEEP_XML_PROFILE") == nullptr)
    {
        unset_env("FASTDDS_DEFAULT_PROFILES_FILE");
    }

    if (subcommand == "defaults")
    {
        return test_defaults();
    }
    if (subcommand == "env_override")
    {
        return test_env_override();
    }
    if (subcommand == "env_invalid")
    {
        return test_env_invalid();
    }
    if (subcommand == "localhost_only")
    {
        return test_localhost_only();
    }
    if (subcommand == "udp_only_shared_memory_decides_warning")
    {
        return test_udp_only_shared_memory_decides_warning();
    }

    if (subcommand == "localhost_only_discovery_filter_warns")
    {
        return test_localhost_only_discovery_filter_warns();
    }
    if (subcommand == "localhost_only_caller_confinement_decides_warning")
    {
        return test_localhost_only_caller_confinement_decides_warning();
    }

    if (subcommand == "localhost_only_not_applied_warns")
    {
        return test_localhost_only_not_applied_warns();
    }
    if (subcommand == "localhost_only_round_trip")
    {
        return test_localhost_only_round_trip();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
