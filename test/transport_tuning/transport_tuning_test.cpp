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
// and the fallback to the default on malformed input. Asserted by reading the
// configured QoS back off a freshly created participant. Each ctest entry runs
// the same binary with a different subcommand so per-case failure stays
// isolated, mirroring the discovery_tuning test.

#include <cstdlib>
#include <iostream>
#include <iterator>
#include <string>
#include <string_view>
#include <vector>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/network_recovery.h"

#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>

namespace
{
    constexpr auto k_domain = 0;

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
    unset_env("FASTDDS_DEFAULT_PROFILES_FILE");

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
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
