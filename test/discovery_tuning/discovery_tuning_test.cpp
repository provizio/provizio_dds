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
// Subcommand-driven tests for participant auto-discovery tuning. They cover
// three things:
//   1. The de-escalated discovery defaults (a modest initial-announcement burst
//      and a relaxed periodic re-announcement) that keep the library from being
//      a primary source of UDP congestion when many participants run, while
//      still discovering robustly. Asserted by reading the configured QoS back
//      off a freshly created participant.
//   2. The PROVIZIO_DDS_DISCOVERY_* environment overrides (and their fallback to
//      the defaults on missing / malformed input).
//   3. Many-participant discovery convergence: a mesh of participants must still
//      all discover and match each other under the gentler defaults — the actual
//      congested-deployment scenario the de-escalation targets.
//
// Each ctest entry runs the same binary with a different subcommand so per-case
// failure stays isolated, mirroring match_publisher_default_test /
// discovered_endpoints_test.

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/subscriber.h"

#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <std_msgs/msg/StringPubSubTypes.hpp>

namespace
{
    using namespace std::chrono_literals;
    using string_pub_sub_type = std_msgs::msg::StringPubSubType;
    using eprosima::fastdds::dds::Duration_t;

    constexpr auto k_domain = 0;
    // The mesh case runs on a per-process domain, picked once at random within the
    // DDS-safe range and away from 0. These tests must unset FASTDDS_DEFAULT_PROFILES_FILE
    // to exercise the code-driven discovery tuning, so unlike the rest of the suite they
    // cannot be confined to loopback; with a fixed domain, concurrent ctest runs on other
    // self-hosted CI hosts sharing the LAN could cross-match into the mesh and break the
    // exact "matched == k" assertion. Randomising per process makes such a collision
    // improbable. The single-participant cases only read back their own QoS, so they are
    // unaffected by domain sharing.
    const auto k_mesh_domain = [] {
        std::random_device random_device;
        std::uniform_int_distribution<int> distribution(1, 200);  // DDS-safe range, excluding domain 0
        return distribution(random_device);
    }();

    // The expected de-escalated defaults (kept in sync with
    // src/domain_participant.cpp and python/provizio_dds.py QosDefaults).
    constexpr std::uint32_t k_default_initial_count = 15;
    constexpr std::int32_t k_default_initial_period_sec = 0;
    constexpr std::uint32_t k_default_initial_period_nsec = 100U * 1000U * 1000U;  // 100 ms
    constexpr std::int32_t k_default_announcement_period_sec = 3;
    constexpr std::uint32_t k_default_announcement_period_nsec = 0;
    constexpr std::int32_t k_default_lease_duration_sec = 30;
    constexpr std::uint32_t k_default_lease_duration_nsec = 0;

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

    // The discovery-config fields under test, read back off a participant's
    // effective QoS.
    struct discovery_cfg
    {
        std::uint32_t initial_count;
        Duration_t initial_period;
        Duration_t announcement_period;
        Duration_t lease_duration;
    };

    discovery_cfg read_cfg(provizio::dds::domain_participant &participant)
    {
        const auto fdds = participant.fastdds_participant();
        const auto &discovery = fdds->get_qos().wire_protocol().builtin.discovery_config;
        return {discovery.initial_announcements.count, discovery.initial_announcements.period,
                discovery.leaseDuration_announcementperiod, discovery.leaseDuration};
    }

    bool duration_is(const Duration_t &actual, std::int32_t seconds, std::uint32_t nanosec)
    {
        return actual.seconds == seconds && actual.nanosec == nanosec;
    }

    // Case: with no env overrides a participant must be configured with the
    // de-escalated defaults — a modest initial burst and a relaxed periodic
    // re-announcement (NOT the historical 200-shot / 1 s flood).
    int test_defaults()
    {
        // Hermetic: ignore any PROVIZIO_DDS_DISCOVERY_* overrides set in the ambient environment.
        unset_env("PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT");
        unset_env("PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS");
        unset_env("PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS");
        unset_env("PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS");

        bool passed = true;
        const auto participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        const auto cfg = read_cfg(*participant);

        passed &= EXPECT(cfg.initial_count == k_default_initial_count);
        passed &= EXPECT(duration_is(cfg.initial_period, k_default_initial_period_sec, k_default_initial_period_nsec));
        passed &= EXPECT(duration_is(cfg.announcement_period, k_default_announcement_period_sec,
                                     k_default_announcement_period_nsec));
        passed &= EXPECT(duration_is(cfg.lease_duration, k_default_lease_duration_sec, k_default_lease_duration_nsec));

        std::cout << "defaults: " << (passed ? "PASS" : "FAIL") << " (count=" << cfg.initial_count
                  << ", initial_period=" << cfg.initial_period.seconds << "s+" << cfg.initial_period.nanosec
                  << "ns, announcement_period=" << cfg.announcement_period.seconds << "s+"
                  << cfg.announcement_period.nanosec << "ns, lease=" << cfg.lease_duration.seconds << "s)" << '\n';
        return passed ? 0 : 1;
    }

    // Case: valid PROVIZIO_DDS_DISCOVERY_* env values are honoured verbatim.
    int test_env_overrides()
    {
        bool passed = true;
        set_env("PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT", "42");
        set_env("PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS", "250");
        set_env("PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS", "5000");
        set_env("PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS", "25000");

        const auto participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        const auto cfg = read_cfg(*participant);

        passed &= EXPECT(cfg.initial_count == 42U);
        passed &= EXPECT(duration_is(cfg.initial_period, 0, 250U * 1000U * 1000U));  // 250 ms
        passed &= EXPECT(duration_is(cfg.announcement_period, 5, 0U));               // 5000 ms == 5 s
        passed &= EXPECT(duration_is(cfg.lease_duration, 25, 0U));                   // 25000 ms == 25 s

        std::cout << "env_overrides: " << (passed ? "PASS" : "FAIL") << " (count=" << cfg.initial_count << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: malformed / out-of-range env values are ignored and the defaults win
    // (so a typo can never silently disable or wildly misconfigure discovery).
    int test_env_invalid()
    {
        bool passed = true;
        set_env("PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT", "not-a-number");
        set_env("PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS", "0");  // zero period is nonsensical
        set_env("PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS", "-5");
        set_env("PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS", "abc");

        const auto participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        const auto cfg = read_cfg(*participant);

        passed &= EXPECT(cfg.initial_count == k_default_initial_count);
        passed &= EXPECT(duration_is(cfg.initial_period, k_default_initial_period_sec, k_default_initial_period_nsec));
        passed &= EXPECT(duration_is(cfg.announcement_period, k_default_announcement_period_sec,
                                     k_default_announcement_period_nsec));
        passed &= EXPECT(duration_is(cfg.lease_duration, k_default_lease_duration_sec, k_default_lease_duration_nsec));

        std::cout << "env_invalid: " << (passed ? "PASS" : "FAIL") << " (count=" << cfg.initial_count << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: many participants must still all discover and match each other under
    // the de-escalated defaults. K publisher participants and K subscriber
    // participants share one topic; every subscriber must match all K publishers.
    // This is a 2K-participant discovery mesh — the congested-deployment shape the
    // de-escalation must not break.
    int test_multi_participant()
    {
        // Hermetic: ignore any PROVIZIO_DDS_DISCOVERY_* overrides set in the ambient environment so the mesh
        // converges under the de-escalated defaults, not whatever the runner exports.
        unset_env("PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT");
        unset_env("PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS");
        unset_env("PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS");
        unset_env("PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS");

        bool passed = true;
        constexpr int k = 8;
        // Generous: 2K participants doing SPDP/SEDP on a loaded runner, with the
        // gentler announcement cadence, may take a while to fully converge.
        constexpr auto convergence_timeout = 60s;
        // Suffixed with an independent per-process random draw (own random_device, NOT derived from
        // k_mesh_domain) so that even the rare case of two concurrent runs picking the same k_mesh_domain still
        // does not cross-match this mesh's topic — the two draws must vary independently of each other.
        std::random_device topic_random_device;
        std::uniform_int_distribution<std::uint32_t> topic_suffix_distribution;
        const std::string topic_name{"rt/provizio_dds_discovery_tuning_mesh_topic_" +
                                     std::to_string(topic_suffix_distribution(topic_random_device))};

        std::vector<std::shared_ptr<provizio::dds::domain_participant>> pub_participants;
        std::vector<std::shared_ptr<provizio::dds::publisher_handle<string_pub_sub_type>>> publishers;
        std::vector<std::shared_ptr<provizio::dds::domain_participant>> sub_participants;
        std::vector<std::shared_ptr<provizio::dds::subscriber_handle<string_pub_sub_type>>> subscribers;

        for (int i = 0; i < k; ++i)
        {
            auto pub_participant =
                provizio::dds::make_domain_participant(k_mesh_domain, provizio::dds::network_recovery_mode::off);
            publishers.push_back(provizio::dds::make_publisher<string_pub_sub_type>(
                pub_participant, topic_name, eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
                provizio::dds::use_default_history_depth));
            pub_participants.push_back(std::move(pub_participant));

            auto sub_participant =
                provizio::dds::make_domain_participant(k_mesh_domain, provizio::dds::network_recovery_mode::off);
            // Explicit reliability (eager reader) so this measures pure discovery
            // convergence rather than the match-publisher deferral path.
            subscribers.push_back(provizio::dds::make_subscriber<string_pub_sub_type>(
                sub_participant, topic_name, [](const std_msgs::msg::String &) {},
                eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS));
            sub_participants.push_back(std::move(sub_participant));
        }

        const auto deadline = std::chrono::steady_clock::now() + convergence_timeout;
        for (int i = 0; i < k; ++i)
        {
            int matched = 0;
            while (std::chrono::steady_clock::now() < deadline)
            {
                // settle_time 0: return the current matched count as soon as it is
                // positive, so we can poll it up toward the full mesh size.
                matched = subscribers[static_cast<std::size_t>(i)]->get_num_matched_publishers(500ms, 0ms);
                if (matched >= k)
                {
                    break;
                }
                std::this_thread::sleep_for(200ms);
            }
            passed &= EXPECT(matched == k);
            if (matched != k)
            {
                std::cerr << "  subscriber " << i << " matched " << matched << " of " << k << " publishers\n";
            }
        }

        std::cout << "multi_participant: " << (passed ? "PASS" : "FAIL") << " (k=" << k << ")" << '\n';
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
    // profile makes the library skip its code-driven discovery tuning entirely (see domain_participant.cpp),
    // so these cases would assert against settings that were never applied — or stop exercising the code
    // path under test. Clear it up front, regardless of the developer's shell.
    unset_env("FASTDDS_DEFAULT_PROFILES_FILE");

    if (subcommand == "defaults")
    {
        return test_defaults();
    }
    if (subcommand == "env_overrides")
    {
        return test_env_overrides();
    }
    if (subcommand == "env_invalid")
    {
        return test_env_invalid();
    }
    if (subcommand == "multi_participant")
    {
        return test_multi_participant();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
