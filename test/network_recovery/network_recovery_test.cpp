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
// Subcommand-driven test harness for the network-recovery / logging
// additions. Each subcommand is registered as a separate ctest entry so
// per-case environment overrides (PROVIZIO_DDS_NETWORK_RECOVERY) and
// success/failure isolation are handled by ctest itself.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "provizio/dds/detail/address_snapshot.h"
#include "provizio/dds/detail/network_recovery_coordinator.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/subscriber.h"

#include <fastdds/utils/IPFinder.hpp>

#include <std_msgs/msg/StringPubSubTypes.hpp>

// Forward-declaration of Fast-DDS's internal SystemInfo class for the
// interface-cache regression test. On Linux / macOS the data members are in
// libfastdds's default-visibility export table, so we can mutate them
// directly to simulate a stale cache. On Windows the data members are not
// exported (only update_interfaces is, via the targeted export patch), so
// the cache-mutation portion of the test is gated to non-Windows; the
// remaining cross-platform portion verifies the refresh helper is linkable
// and that the cache is non-empty after a reset.
//
// The class name must be SystemInfo verbatim (it is Fast-DDS's class) for the
// mangled symbols to resolve, so the snake_case identifier-naming convention
// can't apply — hence the NOLINT.
namespace eprosima
{
    // NOLINTBEGIN(readability-identifier-naming,cppcoreguidelines-special-member-functions)
    // — names must match Fast-DDS's internal members verbatim so the mangled
    // static-member symbols resolve at link time; the snake_case-without-
    // trailing-underscore project convention does not apply.
    class SystemInfo
    {
      public:
        static bool update_interfaces();
#if !defined(_WIN32)
        static bool cached_interfaces_;
        static std::vector<eprosima::fastdds::rtps::IPFinder::info_IP> interfaces_;
        static std::mutex interfaces_mtx_;
#endif
    };
    // NOLINTEND(readability-identifier-naming,cppcoreguidelines-special-member-functions)
}  // namespace eprosima

namespace
{
    struct captured_log
    {
        provizio::dds::log_level level;
        std::string message;
    };

    class log_capture
    {
      public:
        log_capture()
        {
            previous =
                provizio::dds::set_log_callback([this](provizio::dds::log_level level, std::string_view message) {
                    const std::lock_guard<std::mutex> lock{mutex};
                    entries.push_back({level, std::string{message}});
                });
        }
        ~log_capture()
        {
            provizio::dds::set_log_callback(std::move(previous));
        }
        log_capture(const log_capture &) = delete;
        log_capture(log_capture &&) = delete;
        log_capture &operator=(const log_capture &) = delete;
        log_capture &operator=(log_capture &&) = delete;

        std::vector<captured_log> snapshot() const
        {
            const std::lock_guard<std::mutex> lock{mutex};
            return entries;
        }

      private:
        provizio::dds::log_callback previous;
        mutable std::mutex mutex;
        std::vector<captured_log> entries;
    };

    bool expect(bool condition, const char *expression, const char *file, int line)
    {
        if (!condition)
        {
            std::cerr << "FAIL " << file << ":" << line << ": " << expression << '\n';
        }
        return condition;
    }
    // A function-like macro is the right shape here: it has to capture the
    // textual `cond` expression (#cond) and the call-site __FILE__/__LINE__,
    // none of which a constexpr template can synthesise without the caller
    // passing them explicitly — that defeats the point.
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define EXPECT(cond) expect((cond), #cond, __FILE__, __LINE__)

    int test_logging()
    {
        bool passed = true;

        {
            const log_capture capture;
            // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
            provizio::dds::log_info() << "info message " << 42;
            // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
            provizio::dds::log_warning() << "warn " << 1.5;
            provizio::dds::log_error() << "boom";

            const auto entries = capture.snapshot();
            passed &= EXPECT(entries.size() == 3);
            if (entries.size() == 3)
            {
                passed &= EXPECT(entries[0].level == provizio::dds::log_level::info);
                passed &= EXPECT(entries[0].message == "info message 42");
                passed &= EXPECT(entries[1].level == provizio::dds::log_level::warning);
                passed &= EXPECT(entries[1].message == "warn 1.5");
                passed &= EXPECT(entries[2].level == provizio::dds::log_level::error);
                passed &= EXPECT(entries[2].message == "boom");
            }
        }

        // After log_capture is destroyed the original callback is restored.
        // Emit one message — must not crash; we can't verify its destination
        // here so this is just a smoke check.
        provizio::dds::log_info() << "default-emitter smoke check";

        // Installing then restoring an empty callback should return the
        // previously installed callback intact.
        const provizio::dds::log_callback marker = [](provizio::dds::log_level, std::string_view) {};
        auto previous = provizio::dds::set_log_callback(marker);
        auto restored = provizio::dds::set_log_callback(std::move(previous));
        passed &= EXPECT(static_cast<bool>(restored));  // we just installed `marker`

        // Restore default for the rest of the process.
        provizio::dds::set_log_callback({});

        std::cout << (passed ? "logging: PASS" : "logging: FAIL") << '\n';
        return passed ? 0 : 1;
    }

    int test_env_recovery(const std::string &expected)
    {
        bool passed = true;

        // The env-var-controlled mode caches the result on first call. Hit it
        // once and trust that subsequent calls observe the same value.
        const bool env_enabled =
            provizio::dds::resolve_network_recovery_enabled(provizio::dds::network_recovery_mode::env_var_controlled);
        if (expected == "on")
        {
            passed &= EXPECT(env_enabled);
        }
        else if (expected == "off")
        {
            passed &= EXPECT(!env_enabled);
        }
        else
        {
            std::cerr << "bad <expected> value: " << expected << '\n';
            return 1;
        }

        // The explicit modes must always override the env var.
        passed &= EXPECT(provizio::dds::resolve_network_recovery_enabled(provizio::dds::network_recovery_mode::on));
        passed &= EXPECT(!provizio::dds::resolve_network_recovery_enabled(provizio::dds::network_recovery_mode::off));

        std::cout << "env_recovery " << expected << ": " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    int test_env_recovery_garbage()
    {
        // Unknown env-var value must default to enabled AND emit a warning.
        bool passed = true;
        const log_capture capture;
        const bool enabled =
            provizio::dds::resolve_network_recovery_enabled(provizio::dds::network_recovery_mode::env_var_controlled);
        passed &= EXPECT(enabled);

        const auto entries = capture.snapshot();
        bool saw_warning = false;
        for (const auto &entry : entries)
        {
            if (entry.level == provizio::dds::log_level::warning &&
                entry.message.find("not recognised") != std::string::npos)
            {
                saw_warning = true;
                break;
            }
        }
        passed &= EXPECT(saw_warning);

        std::cout << "env_recovery garbage: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    int test_snapshot()
    {
        using provizio::dds::detail::capture_address_snapshot;
        using provizio::dds::detail::interface_address;
        using provizio::dds::detail::interface_address_hash;

        bool passed = true;

        // Two back-to-back snapshots on the same host should be equal: even on
        // a busy machine, address-set changes within a few microseconds are
        // pathological. If this flakes we'd prefer to know.
        const auto first = capture_address_snapshot();
        const auto second = capture_address_snapshot();
        passed &= EXPECT(first == second);

        // The snapshot is set-typed, so equality is order-independent. Verify
        // the hash/equality plumbing by reconstructing the set element-wise
        // and comparing.
        std::unordered_set<interface_address, interface_address_hash> rebuilt;
        for (const auto &entry : first)
        {
            rebuilt.insert(entry);
        }
        passed &= EXPECT(rebuilt == first);

        // The snapshot must not contain loopback (the filter is supposed to
        // drop it). On a CI container with only loopback present the set may
        // legitimately be empty — that's fine.
        for (const auto &entry : first)
        {
            const bool is_loopback = (entry.address_text == "127.0.0.1" || entry.address_text == "::1");
            passed &= EXPECT(!is_loopback);
        }

        std::cout << "snapshot: " << (passed ? "PASS" : "FAIL") << " (" << first.size() << " interface address(es))"
                  << '\n';
        return passed ? 0 : 1;
    }

    int test_runtime_idle()
    {
        // Creating a recovery-enabled participant lazily starts the monitor /
        // coalescer. wait_for_idle() must return immediately because we have
        // not triggered any kernel events.
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        const auto start = std::chrono::steady_clock::now();
        provizio::dds::detail::network_recovery_coordinator::instance().wait_for_idle();
        const auto elapsed = std::chrono::steady_clock::now() - start;

        const bool fast =
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed) < std::chrono::milliseconds{500};
        const bool passed = EXPECT(fast);
        std::cout << "runtime_idle: " << (passed ? "PASS" : "FAIL") << " (elapsed "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms)" << '\n';
        return passed ? 0 : 1;
    }

    struct pub_sub_fixture
    {
        std::shared_ptr<provizio::dds::domain_participant> participant;
        std::shared_ptr<provizio::dds::publisher_handle<std_msgs::msg::StringPubSubType>> publisher;
        std::shared_ptr<provizio::dds::subscriber_handle<std_msgs::msg::StringPubSubType>> subscriber;
        std::mutex mutex;
        std::condition_variable cv;
        std::atomic<int> received_total{0};
        std::string last_message;
    };

    bool publish_and_wait_for(pub_sub_fixture &fixture, const std::string &payload, std::chrono::seconds timeout)
    {
        // Republish every 200 ms so the receiver gets a fresh sample even on a
        // network that briefly drops the first one — short enough to keep the
        // test responsive, long enough to avoid spinning.
        constexpr std::chrono::milliseconds publish_interval{200};

        const int baseline = fixture.received_total.load();
        std_msgs::msg::String message;
        message.data(payload);

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            fixture.publisher->publish(message);
            std::unique_lock<std::mutex> lock{fixture.mutex};
            if (fixture.cv.wait_for(lock, publish_interval, [&] {
                    return fixture.received_total.load() > baseline && fixture.last_message == payload;
                }))
            {
                return true;
            }
        }
        return false;
    }

    int test_reset_roundtrip()
    {
        bool passed = true;
        pub_sub_fixture fixture;
        const std::string topic_name{"provizio_dds_network_recovery_test_topic"};

        fixture.participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        fixture.subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
            fixture.participant, topic_name, [&fixture](const std_msgs::msg::String &message) {
                const std::lock_guard<std::mutex> lock{fixture.mutex};
                fixture.last_message = message.data();
                fixture.received_total.fetch_add(1);
                fixture.cv.notify_all();
            });
        fixture.publisher =
            provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(fixture.participant, topic_name);

        // Baseline: pub→sub round-trip works before any reset.
        passed &= EXPECT(publish_and_wait_for(fixture, "before-reset", std::chrono::seconds{10}));
        const int received_before = fixture.received_total.load();

        // Capture writer GUID before reset so we can verify a fresh writer was
        // built after.
        const auto guid_before = fixture.publisher->get_guid();

        // Trigger the reset directly — same call path the kernel-event-driven
        // coalescer takes, but synchronous and observable from the test.
        fixture.participant->trigger_network_recovery_reset();

        // Same shared_ptr handles, same callback: round-trip must resume on the
        // freshly-rebuilt Fast-DDS objects.
        passed &= EXPECT(publish_and_wait_for(fixture, "after-reset", std::chrono::seconds{15}));
        passed &= EXPECT(fixture.received_total.load() > received_before);

        const auto guid_after = fixture.publisher->get_guid();
        // guid (eprosima::fastdds::rtps::GUID_t) has no padding in practice but
        // is not formally "unique-object-representation" — memcmp is fine for
        // our equality check; bypass the cert/bugprone warning.
        // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison,cert-exp42-c,cert-flp37-c)
        const bool guid_changed = std::memcmp(&guid_before, &guid_after, sizeof(guid_before)) != 0;
        passed &= EXPECT(guid_changed);

        std::cout << "reset_roundtrip: " << (passed ? "PASS" : "FAIL") << " (received " << fixture.received_total.load()
                  << " message(s))" << '\n';
        return passed ? 0 : 1;
    }

    int test_coalescer_skips_no_change()
    {
        // Inject a burst of synthetic kernel events. Since the host's address
        // snapshot does not actually change, the coalescer should fire one
        // reset attempt and skip the participant rebuild — i.e. exactly one
        // skipped_reset_count increment, no reset_count increment.
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();

        // The snapshot is SUBSTITUTED for the duration, so that "does not actually change" is
        // true by construction rather than by the host holding still. Reading the real host
        // here makes the case assert something it does not control: a runner whose network
        // comes up mid-test produces a genuine change, a genuine rebuild, and a failure that
        // looks like the coalescer letting a no-change event through. Seen exactly so on a
        // macos-15-intel runner -- "network change detected: +1 / -0 interface address(es)
        // (0 -> 1)" landed inside the burst below, and the case reported reset_count=1.
        // The sibling cases already force the snapshot for the same reason.
        //
        // Seeded as the last known snapshot too, so the first injected event compares equal
        // rather than reading as the initial-baseline transition.
        const provizio::dds::detail::address_snapshot steady{
            {provizio::dds::detail::interface_address{"provizio_test_steady_if", "203.0.113.11"}}};
        coordinator.wait_for_idle();
        coordinator.force_snapshot_for_test(steady);
        coordinator.seed_last_known_snapshot_for_test(steady);

        const auto reset_before = coordinator.reset_count_for_test();
        const auto skipped_before = coordinator.skipped_reset_count_for_test();

        constexpr int burst_events = 5;
        constexpr std::chrono::milliseconds burst_gap{50};
        for (int iter = 0; iter < burst_events; ++iter)
        {
            coordinator.inject_kernel_event_for_test();
            // Tight burst — well under quiet_period.
            std::this_thread::sleep_for(burst_gap);
        }

        coordinator.wait_for_idle();

        const auto reset_after = coordinator.reset_count_for_test();
        const auto skipped_after = coordinator.skipped_reset_count_for_test();

        // Restored before the assertions, so a failing run still leaves the coordinator
        // reading the real host for whatever runs after it.
        coordinator.force_snapshot_for_test(std::nullopt);

        passed &= EXPECT(reset_after == reset_before);          // no participant rebuild
        passed &= EXPECT(skipped_after == skipped_before + 1);  // exactly one coalesced burst observed

        std::cout << "coalescer_skips_no_change: " << (passed ? "PASS" : "FAIL") << " (reset_count=" << reset_after
                  << " skipped=" << skipped_after << ")" << '\n';
        return passed ? 0 : 1;
    }

    int test_coalescer_resets_on_transient_flap()
    {
        // Regression: a DDS-relevant address that LEAVES and RETURNS within the
        // debounce window nets to an unchanged end-snapshot. An end-snapshot-only
        // diff coalesces it away ("snapshot unchanged, no reset") — but the
        // Fast-DDS sockets bound to that address were torn down while it was gone
        // and must be rebuilt. The coordinator now also compares the burst-START
        // snapshot; this drives that path with a synthetic start snapshot that
        // differs from the (real) end snapshot, so no host interface manipulation
        // is needed. Without the burst-start comparison this asserts FALSE (the
        // burst would be skipped), so it is a true regression guard.
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();

        // The end-state carries an address that the burst-start does NOT: that address
        // left and came back inside the window, which is the shape that needs a rebuild
        // (whatever was bound to it died while it was gone). The reverse shape — an extra
        // address at burst start, gone by the end — is an address that appeared and
        // vanished; nothing was ever bound to it, and a rebuild would achieve nothing.
        //
        // Both snapshots are substituted rather than read from the host, so the case runs
        // identically in a CI container, whose snapshot is legitimately empty (veth is
        // filtered out by design) and in which nothing could be made to return.
        auto end_state = provizio::dds::detail::capture_address_snapshot();
        end_state.insert(
            provizio::dds::detail::interface_address{"provizio_test_transient_if", "203.0.113.7"});  // TEST-NET-3
        auto simulated_start = end_state;
        simulated_start.erase(provizio::dds::detail::interface_address{"provizio_test_transient_if", "203.0.113.7"});

        coordinator.wait_for_idle();
        coordinator.force_snapshot_for_test(end_state);
        coordinator.seed_last_known_snapshot_for_test(end_state);

        const auto reset_before = coordinator.reset_count_for_test();
        const auto skipped_before = coordinator.skipped_reset_count_for_test();

        coordinator.inject_transient_for_test(simulated_start);
        coordinator.wait_for_idle();

        const auto reset_after = coordinator.reset_count_for_test();
        const auto skipped_after = coordinator.skipped_reset_count_for_test();

        coordinator.force_snapshot_for_test(std::nullopt);

        passed &= EXPECT(reset_after == reset_before + 1);  // transient → participant rebuild
        passed &= EXPECT(skipped_after == skipped_before);  // NOT coalesced away as "no change"

        std::cout << "coalescer_resets_on_transient_flap: " << (passed ? "PASS" : "FAIL")
                  << " (reset_count=" << reset_after << " skipped=" << skipped_after << ")" << '\n';
        return passed ? 0 : 1;
    }

    int test_rebuild_on_address_change()
    {
        // "Rebuild only for what was gained" is about snapshot ENTRIES, not interfaces, and
        // an entry is (interface name, address, prefix length). So re-addressing an
        // interface that never went away is a gain — the old entry leaves and a new one
        // arrives — and so is re-subnetting one without changing its address at all, which
        // changes which peers Fast-DDS considers on-link. Both must rebuild; only a change
        // that is purely subtractive must not.
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();
        coordinator.wait_for_idle();

        const provizio::dds::detail::interface_address before{"provizio_test_dhcp_if", "203.0.113.20", 24};
        const provizio::dds::detail::interface_address readdressed{"provizio_test_dhcp_if", "203.0.113.21", 24};
        const provizio::dds::detail::interface_address resubnetted{"provizio_test_dhcp_if", "203.0.113.21", 16};

        // A new address on an interface that never left.
        coordinator.seed_last_known_snapshot_for_test(provizio::dds::detail::address_snapshot{before});
        coordinator.force_snapshot_for_test(provizio::dds::detail::address_snapshot{readdressed});
        const auto reset_before_readdress = coordinator.reset_count_for_test();
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before_readdress + 1);

        // Same address, different prefix — Fast-DDS' on-link decision changes with it.
        coordinator.force_snapshot_for_test(provizio::dds::detail::address_snapshot{resubnetted});
        const auto reset_before_resubnet = coordinator.reset_count_for_test();
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before_resubnet + 1);

        coordinator.force_snapshot_for_test(std::nullopt);

        std::cout << "rebuild_on_address_change: " << (passed ? "PASS" : "FAIL")
                  << " (reset_count=" << coordinator.reset_count_for_test() << ")" << '\n';
        return passed ? 0 : 1;
    }

    int test_no_baseline_empty_list_is_not_a_rebuild()
    {
        // The other half of the rule above: with no baseline, a first readable list that is
        // EMPTY has nothing a rebuild could bind, so it must not rebuild -- the same judgement
        // the added == 0 path makes in steady state. Worth pinning separately because "no usable
        // address" is the normal reading inside a container whose only device is filtered out,
        // and rebuilding every participant on startup there would be a pure regression.
        bool passed = true;

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();
        coordinator.force_enumeration_failure_for_test(true);
        coordinator.force_snapshot_for_test(provizio::dds::detail::address_snapshot{});

        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        passed &= EXPECT(participant != nullptr);

        coordinator.wait_for_idle();
        const auto reset_before = coordinator.reset_count_for_test();
        const auto skipped_before = coordinator.skipped_reset_count_for_test();

        coordinator.force_enumeration_failure_for_test(false);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before);
        passed &= EXPECT(coordinator.skipped_reset_count_for_test() > skipped_before);

        coordinator.force_snapshot_for_test(std::nullopt);

        std::cout << "no_baseline_empty_list_is_not_a_rebuild: " << (passed ? "PASS" : "FAIL")
                  << " (reset_count=" << coordinator.reset_count_for_test() << ")" << '\n';
        return passed ? 0 : 1;
    }

    int test_unreadable_interfaces_are_not_a_change()
    {
        // Asking the OS for its interfaces can fail — getifaddrs is a sysctl(NET_RT_IFLIST)
        // pair on macOS and can lose a race with a routing-table change, GetAdaptersAddresses
        // can fail outright. Returning an empty set for that is indistinguishable from a host
        // that genuinely has no usable address, so a failed read used to present itself as
        // "every address disappeared" and rebuild every participant — then rebuild them again
        // when the next read succeeded. Two rebuilds, no network change, and any in-flight
        // request/response lost with them.
        //
        // What this pins: a failed read decides nothing and leaves the last known set alone,
        // so the next successful read of the SAME set is not a change either.
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();

        auto host_addresses = provizio::dds::detail::capture_address_snapshot();
        host_addresses.insert(provizio::dds::detail::interface_address{"provizio_test_present_if", "203.0.113.11"});

        coordinator.wait_for_idle();
        coordinator.force_snapshot_for_test(host_addresses);
        coordinator.seed_last_known_snapshot_for_test(host_addresses);

        const auto reset_before = coordinator.reset_count_for_test();
        const auto skipped_before = coordinator.skipped_reset_count_for_test();

        // The read fails.
        coordinator.force_enumeration_failure_for_test(true);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before);
        // Not even a skipped reset: nothing was decided at all.
        passed &= EXPECT(coordinator.skipped_reset_count_for_test() == skipped_before);

        // The read succeeds again with the set unchanged. Had the failure been recorded as
        // an empty snapshot, this would read as every address returning, and rebuild.
        coordinator.force_enumeration_failure_for_test(false);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before);
        passed &= EXPECT(coordinator.skipped_reset_count_for_test() == skipped_before + 1);

        coordinator.force_snapshot_for_test(std::nullopt);

        std::cout << "unreadable_interfaces_are_not_a_change: " << (passed ? "PASS" : "FAIL")
                  << " (reset_count=" << coordinator.reset_count_for_test() << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: how OFTEN a host that cannot be asked for its interfaces says so. The read is
    // attempted on every kernel event and on every safety-net tick, so a host that has
    // genuinely lost the ability to enumerate would fill the log with one identical line
    // every few seconds for the life of the process -- and the operator watching for the
    // actual fault would be reading it in a screenful of that. The other half is the
    // re-arm: a warning suppressed forever after the first streak would leave a LATER
    // outage completely silent, which is the failure this diagnostic exists to catch.
    int test_unreadable_interfaces_warn_once_per_streak()
    {
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        passed &= EXPECT(participant != nullptr);

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();
        coordinator.wait_for_idle();

        // Installed after the participant exists: its own creation reads the interfaces
        // too, and this case is about the reads it drives deliberately.
        const log_capture capture;
        constexpr std::string_view needle{"could not read this host's network interfaces"};

        const auto warnings = [&capture, needle] {
            std::size_t found = 0;
            for (const auto &entry : capture.snapshot())
            {
                if (entry.message.find(needle) != std::string::npos)
                {
                    ++found;
                }
            }
            return found;
        };

        // The streak begins.
        coordinator.force_enumeration_failure_for_test(true);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(warnings() == 1);

        // Still failing: the same line again would say nothing new.
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(warnings() == 1);

        // A successful read ends the streak. Nothing is logged for it -- recovering the
        // ability to enumerate is not news -- but the warning must be armed again.
        coordinator.force_enumeration_failure_for_test(false);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(warnings() == 1);

        // A second, separate outage, which an operator has to hear about.
        coordinator.force_enumeration_failure_for_test(true);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        const auto after_second_streak = warnings();
        passed &= EXPECT(after_second_streak == 2);

        coordinator.force_enumeration_failure_for_test(false);

        std::cout << "unreadable_interfaces_warn_once_per_streak: " << (passed ? "PASS" : "FAIL") << " ("
                  << after_second_streak << " warning(s) over two streaks)" << '\n';
        return passed ? 0 : 1;
    }

    int test_no_baseline_rebuilds_for_first_readable_list()
    {
        // The interface read at startup can fail exactly as any later one can, and the baseline
        // is seeded from it. A failure there is not "no addresses" and not "the addresses we
        // have" -- it is not knowing, and specifically not knowing what the participants
        // actually bound to.
        //
        // So the first readable list REBUILDS rather than being quietly adopted. An interface
        // can come up while the list is unreadable, and adopting it as the baseline would lose
        // that rebuild permanently: the address would no longer count as a gain against any
        // later snapshot, so nothing would ever bind it short of a process restart. An extra
        // rebuild costs one reconnect; a missed one costs the interface.
        //
        // What this pins: a failed seed leaves NO baseline, the first readable list rebuilds
        // exactly once, and a real change measured from it still rebuilds after that.
        bool passed = true;

        auto host_addresses = provizio::dds::detail::capture_address_snapshot();
        host_addresses.insert(provizio::dds::detail::interface_address{"provizio_test_seeded_if", "203.0.113.30"});

        // Both hooks are set BEFORE the participant exists, so the failure is already in force
        // when register_participant seeds the baseline, and every later read returns one known
        // set. Forcing the snapshot as well is what makes this deterministic on a developer
        // machine: a real kernel event can fire at any point below, and it must not be able to
        // adopt a DIFFERENT list than the one this test reasons about. The failure flag wins
        // over the forced snapshot in current_snapshot(), so the seed still fails.
        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();
        coordinator.force_enumeration_failure_for_test(true);
        coordinator.force_snapshot_for_test(host_addresses);

        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        passed &= EXPECT(participant != nullptr);

        coordinator.wait_for_idle();
        const auto reset_before = coordinator.reset_count_for_test();

        // Reads start succeeding. With no baseline, everything now visible counts as new, so
        // this rebuilds -- exactly once. Injected twice to pin that: the second pass finds the
        // baseline in place and must see no change, which is also what proves the first pass
        // stored it. One rebuild whichever pass performs it, so this is not timing-dependent
        // even though a real kernel event may get there first.
        coordinator.force_enumeration_failure_for_test(false);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before + 1);

        // A genuine gain measured from that baseline still rebuilds -- establishing the baseline
        // must not have left the detector inert.
        auto grown = host_addresses;
        grown.insert(provizio::dds::detail::interface_address{"provizio_test_seeded_if2", "203.0.113.31"});
        coordinator.force_snapshot_for_test(grown);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before + 2);

        coordinator.force_snapshot_for_test(std::nullopt);

        std::cout << "no_baseline_rebuilds_for_first_readable_list: " << (passed ? "PASS" : "FAIL")
                  << " (reset_count=" << coordinator.reset_count_for_test() << ")" << '\n';
        return passed ? 0 : 1;
    }

    int test_no_rebuild_on_address_loss()
    {
        // An address going away is not worth a rebuild: nothing can be bound to what is
        // gone, and tearing down endpoints that still work over the remaining interfaces
        // costs every in-flight sample for no gain. The rebuild belongs to the moment the
        // address comes BACK, which is when it can achieve something — so this drives the
        // pair and asserts exactly one rebuild across both halves.
        //
        // Losing and regaining the SAME address is the case that matters, and the one a
        // set-difference alone gets wrong: a host that ends up where it started looks
        // unchanged, yet its sockets died in between. The snapshot is substituted so both
        // halves are reachable on any host, CI containers included.
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();

        auto with_address = provizio::dds::detail::capture_address_snapshot();
        with_address.insert(provizio::dds::detail::interface_address{"provizio_test_lost_if", "203.0.113.9"});
        auto without_address = with_address;
        without_address.erase(provizio::dds::detail::interface_address{"provizio_test_lost_if", "203.0.113.9"});

        coordinator.wait_for_idle();
        coordinator.seed_last_known_snapshot_for_test(with_address);

        // Half one: the address goes away and stays away.
        coordinator.force_snapshot_for_test(without_address);
        const auto reset_before_loss = coordinator.reset_count_for_test();
        const auto skipped_before_loss = coordinator.skipped_reset_count_for_test();
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before_loss);
        passed &= EXPECT(coordinator.skipped_reset_count_for_test() == skipped_before_loss + 1);

        // Half two: the same address returns. The loss above must have been adopted as the
        // new baseline, or this would read as "no change" and never rebuild.
        coordinator.force_snapshot_for_test(with_address);
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        const auto reset_after_return = coordinator.reset_count_for_test();
        passed &= EXPECT(reset_after_return == reset_before_loss + 1);

        coordinator.force_snapshot_for_test(std::nullopt);

        std::cout << "no_rebuild_on_address_loss: " << (passed ? "PASS" : "FAIL") << " (reset_count "
                  << reset_before_loss << " -> " << reset_after_return << ")" << '\n';
        return passed ? 0 : 1;
    }

    int test_safety_net_no_rebuild_on_address_loss()
    {
        // The periodic tick decides exactly as the event path does, and for the same
        // reason: it exists to catch changes whose kernel events were never delivered (a
        // dropped ENOBUFS datagram, a monitor that died between ticks), and such a change
        // is no more deserving of a rebuild than one that arrived normally. A pure loss
        // therefore adopts the smaller set and rebuilds nothing here too. Before the
        // decision was shared, this path rebuilt every participant for a loss the event
        // path deliberately skips — a disruption no user could tell from a real recovery.
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();

        auto with_address = provizio::dds::detail::capture_address_snapshot();
        with_address.insert(provizio::dds::detail::interface_address{"provizio_test_lost_if", "203.0.113.9"});
        auto without_address = with_address;
        without_address.erase(provizio::dds::detail::interface_address{"provizio_test_lost_if", "203.0.113.9"});

        coordinator.wait_for_idle();
        coordinator.seed_last_known_snapshot_for_test(with_address);

        // Half one: the tick finds the address gone, with no event having reported it.
        coordinator.force_snapshot_for_test(without_address);
        const auto reset_before_loss = coordinator.reset_count_for_test();
        const auto skipped_before_loss = coordinator.skipped_reset_count_for_test();
        coordinator.run_safety_net_tick_for_test();
        passed &= EXPECT(coordinator.reset_count_for_test() == reset_before_loss);
        passed &= EXPECT(coordinator.skipped_reset_count_for_test() == skipped_before_loss + 1);

        // Half two: the same address returns, and THAT rebuilds — which also proves the
        // loss was adopted as the baseline rather than merely ignored.
        coordinator.force_snapshot_for_test(with_address);
        coordinator.run_safety_net_tick_for_test();
        const auto reset_after_return = coordinator.reset_count_for_test();
        passed &= EXPECT(reset_after_return == reset_before_loss + 1);

        coordinator.force_snapshot_for_test(std::nullopt);

        std::cout << "safety_net_no_rebuild_on_address_loss: " << (passed ? "PASS" : "FAIL") << " (reset_count "
                  << reset_before_loss << " -> " << reset_after_return << ")" << '\n';
        return passed ? 0 : 1;
    }

    int test_wait_for_idle_blocks_during_reset()
    {
        // wait_for_idle() must not return until the coalescer has FINISHED handling the
        // burst — not merely once the quiet period has elapsed, which the debounce alone
        // satisfies without exercising reset_in_progress tracking at all.
        //
        // Observed through the coordinator's own counters rather than through a log line.
        // This used to wait for a "snapshot unchanged" / "reset complete" message, which
        // tied the test to logging that says nothing to a user of the library and has since
        // been removed; the counters are the actual state the log was standing in for, so
        // this is both a stronger assertion and one that cannot be broken by log wording.
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();

        // Every burst ends in exactly one of the two: a reset, or a coalesced-away no-change.
        const auto handled_before = coordinator.reset_count_for_test() + coordinator.skipped_reset_count_for_test();

        const auto start = std::chrono::steady_clock::now();
        coordinator.inject_kernel_event_for_test();
        coordinator.wait_for_idle();
        const auto elapsed = std::chrono::steady_clock::now() - start;
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

        // Both counters are incremented on the coalescer thread inside run_reset, before
        // reset_in_progress is cleared and the CV is notified. wait_for_idle blocks on that
        // same CV until reset_in_progress is false, so the increment must already be visible
        // by the time it returns.
        const auto handled_after = coordinator.reset_count_for_test() + coordinator.skipped_reset_count_for_test();
        passed &= EXPECT(handled_after == handled_before + 1);

        // Sanity: also confirm we waited at least the quiet period.
        const auto quiet_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  provizio::dds::detail::network_recovery_coordinator::quiet_period)
                                  .count();
        passed &= EXPECT(elapsed_ms >= quiet_ms - 500);

        std::cout << "wait_for_idle_blocks_during_reset: " << (passed ? "PASS" : "FAIL") << " (waited " << elapsed_ms
                  << " ms, quiet_period=" << quiet_ms << " ms, bursts_handled=" << (handled_after - handled_before)
                  << ")" << '\n';
        return passed ? 0 : 1;
    }

    int test_reentrant_listener_during_reset()
    {
        // Regression test: a subscriber callback that calls back into
        // provizio APIs (taking the lifecycle lock shared) must not deadlock
        // a concurrent network-recovery reset. The reset's
        // detach_for_reset / drain phase happens BEFORE the exclusive
        // lifecycle lock is acquired, specifically so this pattern is safe.
        bool passed = true;
        pub_sub_fixture fixture;
        const std::string topic_name{"provizio_dds_network_recovery_reentrant_topic"};

        fixture.participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        // A second publisher the listener re-enters via publish() — this is
        // exactly the bridge/relay pattern that would previously deadlock.
        auto echo_publisher =
            provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(fixture.participant, topic_name + "_echo");

        fixture.subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
            fixture.participant, topic_name, [&fixture, echo_publisher](const std_msgs::msg::String &message) {
                // Re-enter: this call takes reset_mutex shared. If a reset's
                // delete_datareader on US is waiting for our callback to return
                // while the reset holds reset_mutex exclusive, this acquire
                // would deadlock. With detach_for_reset draining listeners
                // BEFORE the exclusive acquire, the deadlock is impossible.
                std_msgs::msg::String forwarded;
                forwarded.data(message.data());
                echo_publisher->publish(forwarded);

                const std::lock_guard<std::mutex> lock{fixture.mutex};
                fixture.last_message = message.data();
                fixture.received_total.fetch_add(1);
                fixture.cv.notify_all();
            });
        fixture.publisher =
            provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(fixture.participant, topic_name);

        // Establish baseline communication.
        passed &= EXPECT(publish_and_wait_for(fixture, "reentrant-before", std::chrono::seconds{10}));

        // Trigger the reset directly while the data callback is regularly
        // re-entering provizio APIs. The bug would manifest as a hang here.
        const auto reset_start = std::chrono::steady_clock::now();
        fixture.participant->trigger_network_recovery_reset();
        const auto reset_elapsed = std::chrono::steady_clock::now() - reset_start;

        // Reset must complete in bounded time (very generous bound to absorb
        // CI scheduling and Fast-DDS discovery costs).
        const auto reset_ms = std::chrono::duration_cast<std::chrono::milliseconds>(reset_elapsed).count();
        passed &= EXPECT(reset_ms < 30000);

        // After reset, traffic must resume.
        passed &= EXPECT(publish_and_wait_for(fixture, "reentrant-after", std::chrono::seconds{15}));

        std::cout << "reentrant_listener_during_reset: " << (passed ? "PASS" : "FAIL") << " (reset took " << reset_ms
                  << " ms)" << '\n';
        return passed ? 0 : 1;
    }

    int test_concurrent_make_publisher_during_reset()
    {
        // Regression test: creating publishers/subscribers concurrently with a
        // reset must not race onto a destroyed participant. The
        // registration_mutex in domain_participant serialises the two.
        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

        std::atomic<bool> stop{false};
        std::atomic<int> created_total{0};
        std::atomic<int> create_failures{0};

        // Spawn a worker that constantly creates short-lived publisher /
        // subscriber pairs against the participant.
        std::thread worker{[&] {
            int topic_seq = 0;
            while (!stop.load(std::memory_order_acquire))
            {
                try
                {
                    const std::string topic_name = "provizio_dds_concurrency_topic_" + std::to_string(topic_seq++);
                    auto pub = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(participant, topic_name);
                    auto sub = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
                        participant, topic_name, [](const std_msgs::msg::String &) {});
                    created_total.fetch_add(1, std::memory_order_acq_rel);
                }
                catch (...)
                {
                    create_failures.fetch_add(1, std::memory_order_acq_rel);
                }
            }
        }};

        // Trigger several resets while the worker is creating endpoints.
        constexpr int reset_attempts = 3;
        constexpr std::chrono::milliseconds reset_gap{200};
        for (int iter = 0; iter < reset_attempts; ++iter)
        {
            participant->trigger_network_recovery_reset();
            std::this_thread::sleep_for(reset_gap);
        }

        stop.store(true, std::memory_order_release);
        worker.join();

        passed &= EXPECT(create_failures.load() == 0);
        passed &= EXPECT(created_total.load() > 0);

        std::cout << "concurrent_make_publisher_during_reset: " << (passed ? "PASS" : "FAIL") << " (created "
                  << created_total.load() << " endpoint pairs, " << create_failures.load() << " failures)" << '\n';
        return passed ? 0 : 1;
    }

    int test_reset_disabled()
    {
        // recovery_mode::off → participant is NOT registered with the runtime
        // and trigger_network_recovery_reset() is an explicit no-op (returns
        // early). Pub/sub continues to work as it did before.
        bool passed = true;
        pub_sub_fixture fixture;
        const std::string topic_name{"provizio_dds_network_recovery_disabled_test_topic"};

        fixture.participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);

        fixture.subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
            fixture.participant, topic_name, [&fixture](const std_msgs::msg::String &message) {
                const std::lock_guard<std::mutex> lock{fixture.mutex};
                fixture.last_message = message.data();
                fixture.received_total.fetch_add(1);
                fixture.cv.notify_all();
            });
        fixture.publisher =
            provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(fixture.participant, topic_name);

        passed &= EXPECT(publish_and_wait_for(fixture, "no-recovery", std::chrono::seconds{10}));

        const auto guid_before = fixture.publisher->get_guid();
        fixture.participant->trigger_network_recovery_reset();  // must be a no-op
        const auto guid_after = fixture.publisher->get_guid();
        // See reset_roundtrip for the rationale on memcmp + this NOLINT.
        // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison,cert-exp42-c,cert-flp37-c)
        const bool guid_same = std::memcmp(&guid_before, &guid_after, sizeof(guid_before)) == 0;
        passed &= EXPECT(guid_same);

        // Confirm the (untouched) pipeline still publishes.
        passed &= EXPECT(publish_and_wait_for(fixture, "still-no-recovery", std::chrono::seconds{10}));

        std::cout << "reset_disabled: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Regression test for the APT-11792 stale-interface-cache bug.
    //
    // Without the fix, Fast-DDS's process-wide `SystemInfo` cache is populated
    // exactly once (at first participant construction) and never refreshed.
    // A participant rebuilt by the auto-recovery path therefore binds to the
    // same dead interfaces the original one did, even when the host's network
    // state has changed in the meantime. The fix wires
    // `refresh_fastdds_interface_cache()` into `create_fastdds_participant()`
    // so each new (or rebuilt) participant sees up-to-date interfaces.
    //
    // The test simulates the "cache went stale" condition by mutating the
    // cache directly. On Linux / macOS the default-visibility ELF / Mach-O
    // exports make this straightforward. On Windows the data members aren't
    // exported (CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS auto-exports functions
    // only), so the test runs a weaker variant there: it just verifies the
    // refresh helper resolves at link time and returns success, plus that
    // the cache after a reset equals a forced fresh enumeration. Both
    // variants would fail in the original buggy state — the cross-platform
    // assertion fails because `update_interfaces` is not callable at all
    // (link error or runtime missing symbol on Windows; on Linux / macOS
    // the call site exists but the cache stays stuck on stale data after
    // the cache-mutation step on the Linux / macOS path).
    int test_reset_refreshes_fastdds_interface_cache()
    {
        bool passed = true;

        // Cross-platform precondition: the refresh helper itself must
        // resolve and report success. This is the single piece of evidence
        // we have on Windows that the symbol-export setup (the targeted
        // FASTDDS_EXPORTED_API patch on SystemInfo::update_interfaces) is
        // correct — a missing export would be a link error here.
        passed &= EXPECT(provizio::dds::refresh_fastdds_interface_cache());

        pub_sub_fixture fixture;
        fixture.participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);

#if !defined(_WIN32)
        // Capture the real interface set as the ground truth. Forced fresh
        // lookup (third arg true) so we bypass the cache we're about to
        // tamper with.
        std::vector<eprosima::fastdds::rtps::IPFinder::info_IP> real_ips;
        eprosima::fastdds::rtps::IPFinder::getIPs(&real_ips, true);
        passed &= EXPECT(!real_ips.empty());

        // Mutate the SystemInfo cache to a "stale" empty state — the
        // scenario the original bug created automatically (cache locked to
        // whatever was up at first-participant time, then the host's
        // interface set changed underneath). Without the fix,
        // trigger_network_recovery_reset does NOT refresh the cache and the
        // rebuilt participant would inherit this empty set. cached_interfaces_
        // stays true so the cache-aware read path keeps using our emptied
        // vector rather than falling back to a fresh lookup.
        {
            const std::lock_guard<std::mutex> lock{eprosima::SystemInfo::interfaces_mtx_};
            eprosima::SystemInfo::interfaces_.clear();
            eprosima::SystemInfo::cached_interfaces_ = true;
        }

        // Trigger the auto-recovery reset. The fix calls
        // refresh_fastdds_interface_cache inside
        // domain_participant::trigger_network_recovery_reset before
        // recreating the participant, which repopulates the cache.
        fixture.participant->trigger_network_recovery_reset();

        // The cache must now reflect the real host interfaces again.
        std::vector<eprosima::fastdds::rtps::IPFinder::info_IP> cache_after_reset;
        {
            const std::lock_guard<std::mutex> lock{eprosima::SystemInfo::interfaces_mtx_};
            cache_after_reset = eprosima::SystemInfo::interfaces_;
        }
        passed &= EXPECT(!cache_after_reset.empty());
        passed &= EXPECT(cache_after_reset.size() == real_ips.size());
#else
        // Windows: SystemInfo's data members aren't in fastdds.dll's export
        // table (the targeted patch only exports the update_interfaces
        // function), so we can't mutate/inspect the cache directly. Settle
        // for proving the reset path runs without breaking and the refresh
        // helper still resolves and works post-reset.
        fixture.participant->trigger_network_recovery_reset();
        passed &= EXPECT(provizio::dds::refresh_fastdds_interface_cache());

        std::vector<eprosima::fastdds::rtps::IPFinder::info_IP> cache_after_reset;
        eprosima::fastdds::rtps::IPFinder::getIPs(&cache_after_reset, true);
        passed &= EXPECT(!cache_after_reset.empty());
#endif

        std::cout << "reset_refreshes_fastdds_interface_cache: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    int test_snapshot_prefix_length()
    {
        using provizio::dds::detail::address_snapshot;
        using provizio::dds::detail::capture_address_snapshot;
        using provizio::dds::detail::interface_address;

        bool passed = true;
        const auto snapshot = capture_address_snapshot();

        std::size_t with_prefix = 0;
        for (const auto &entry : snapshot)
        {
            // Hard invariant: a prefix can never exceed the address family's width.
            const bool is_ipv4 = entry.address_text.find(':') == std::string::npos;
            const unsigned int max_prefix = is_ipv4 ? 32U : 128U;
            passed &= EXPECT(entry.prefix_length <= max_prefix);
            if (entry.prefix_length > 0)
            {
                ++with_prefix;
            }
        }
        // Softer check (some exotic point-to-point devices report no mask at all, and a
        // CI container can legitimately have an empty snapshot): if anything was found,
        // at least one entry must carry a real prefix — otherwise we aren't reading
        // netmasks at all.
        if (!snapshot.empty())
        {
            passed &= EXPECT(with_prefix > 0);
        }

        // The prefix is part of the element's identity, which is what makes a
        // netmask-only change (re-subnetting an interface, 192.0.2.10/24 → /16) a
        // detected network change rather than something silently coalesced away:
        // Fast-DDS derives its netmask-based locator filtering from the prefix.
        const interface_address slash24{"provizio_test_if", "192.0.2.10", 24};
        const interface_address slash16{"provizio_test_if", "192.0.2.10", 16};
        passed &= EXPECT(!(slash24 == slash16));
        passed &= EXPECT(address_snapshot{slash24} != address_snapshot{slash16});
        // ... while identical triples stay equal, i.e. the hash and equality agree.
        passed &= EXPECT(address_snapshot{slash24} == address_snapshot{slash24});

        std::cout << "snapshot_prefix_length: " << (passed ? "PASS" : "FAIL") << " (" << snapshot.size() << " entries, "
                  << with_prefix << " with a non-zero prefix)" << '\n';
        return passed ? 0 : 1;
    }

    int test_extra_interfaces_env()
    {
        // Registered with PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES preset, in its
        // own process because the variable is parsed exactly once per process.
        bool passed = true;
        const auto &names = provizio::dds::detail::force_included_interfaces();

        passed &= EXPECT(names.size() == 3);
        passed &= EXPECT(names.count("docker0") == 1);
        passed &= EXPECT(names.count("br-test") == 1);
        passed &= EXPECT(names.count("veth9") == 1);
        // Surrounding whitespace is trimmed and empty entries ("a,,b") dropped, so no
        // unmatchable name ends up in the set.
        passed &= EXPECT(names.count(" br-test ") == 0);
        passed &= EXPECT(names.count("") == 0);

        std::cout << "extra_interfaces_env: " << (passed ? "PASS" : "FAIL") << " (" << names.size()
                  << " force-included)" << '\n';
        return passed ? 0 : 1;
    }

    int test_safety_net_env_clamped()
    {
        // Registered with PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC set to a value far
        // beyond what the duration arithmetic can hold. Left unclamped, converting it to
        // the nanoseconds that condition_variable::wait_for works in overflows int64 and
        // libstdc++ returns from wait_for IMMEDIATELY — turning "wait for the period,
        // then tick" into a busy loop that runs a getifaddrs plus a full RTM_GETLINK
        // dump per iteration, pegging a core for the process' lifetime.
        bool passed = true;

        const log_capture capture;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        auto &coordinator = provizio::dds::detail::network_recovery_coordinator::instance();
        coordinator.wait_for_idle();

        // The clamp is reported as a warning naming the maximum.
        bool warned_about_clamping = false;
        for (const auto &entry : capture.snapshot())
        {
            if (entry.level == provizio::dds::log_level::warning &&
                entry.message.find("exceeds the maximum") != std::string::npos)
            {
                warned_about_clamping = true;
            }
        }
        passed &= EXPECT(warned_about_clamping);

        // And the coalescer is genuinely waiting rather than spinning: with the period
        // clamped to a day, no tick can fire in this window, so the reset counters stay
        // put. An unclamped period would have run thousands of ticks by now.
        const auto resets_before = coordinator.reset_count_for_test();
        const auto skipped_before = coordinator.skipped_reset_count_for_test();
        // Long enough that an unclamped period (which returns from wait_for instantly)
        // would have run thousands of ticks, short enough to keep the test quick.
        constexpr std::chrono::milliseconds spin_observation_window{500};
        std::this_thread::sleep_for(spin_observation_window);
        passed &= EXPECT(coordinator.reset_count_for_test() == resets_before);
        passed &= EXPECT(coordinator.skipped_reset_count_for_test() == skipped_before);

        std::cout << "safety_net_env_clamped: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    int test_safety_net_detects_missed_change()
    {
        using provizio::dds::detail::network_recovery_coordinator;

        bool passed = true;
        // Registering a recovery-enabled participant starts the monitor and coalescer.
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        auto &coordinator = network_recovery_coordinator::instance();
        coordinator.wait_for_idle();

        const auto resets_before = coordinator.reset_count_for_test();

        // Simulate "the host's addresses changed but no kernel event told us" — a dropped
        // netlink datagram, a transition on a channel we don't subscribe to, or a change
        // that raced the monitor's startup.
        //
        // The CAPTURE is substituted, not the baseline seeded, and the difference matters:
        // what the tick has to notice is an address the host GAINED without telling us,
        // which is the only kind of change a rebuild can act on (see decide_and_apply — a
        // pure loss adopts the smaller set and rebuilds nothing, on this path exactly as on
        // the event path). Seeding a stale baseline and letting the tick read the real host
        // would produce a gain only where the host happens to have an address at all: on a
        // CI container, whose snapshot is legitimately empty because its veth is filtered
        // out, it would read as a pure loss and correctly rebuild nothing. Mirrors the
        // Python case, which substitutes its capture for the same reason.
        auto with_unreported = provizio::dds::detail::capture_address_snapshot();
        constexpr int unreported_prefix_length = 24;  // a /24 in TEST-NET-3 (RFC 5737)
        with_unreported.insert({"provizio_test_missing_if", "203.0.113.9", unreported_prefix_length});
        coordinator.force_snapshot_for_test(with_unreported);

        coordinator.run_safety_net_tick_for_test();
        passed &= EXPECT(coordinator.reset_count_for_test() == resets_before + 1);

        // The tick must store what it found, so an immediately following tick is a
        // no-op. Without this, every period would rebuild every participant.
        coordinator.run_safety_net_tick_for_test();
        passed &= EXPECT(coordinator.reset_count_for_test() == resets_before + 1);

        coordinator.force_snapshot_for_test(std::nullopt);

        std::cout << "safety_net_detects_missed_change: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: a log callback may create a participant, even from the line the very first
    // registration emits. logging.h promises exactly that, and this is the one diagnostic
    // on that path that used to be logged from inside registry_mutex + monitor_mutex: the
    // callback's make_domain_participant re-enters register_participant, which blocks on a
    // non-recursive mutex the same thread already holds. A deadlock, not a slow path --
    // so if this case ever regresses it hangs, and CTest's timeout is what reports it.
    //
    // The trigger is an interface read that fails at the first registration, which is not
    // hypothetical: it is the macOS sysctl(NET_RT_IFLIST) race this feature exists to
    // tolerate. Forced here rather than waited for.
    int test_log_callback_may_create_participant()
    {
        using provizio::dds::detail::network_recovery_coordinator;

        bool passed = true;
        auto &coordinator = network_recovery_coordinator::instance();
        coordinator.force_enumeration_failure_for_test(true);

        std::atomic_bool callback_created_participant{false};
        std::shared_ptr<provizio::dds::domain_participant> from_callback;
        std::atomic_bool reentered{false};
        auto previous =
            provizio::dds::set_log_callback([&](const provizio::dds::log_level level, const std::string_view message) {
                std::cout << "  [log] " << message << '\n' << std::flush;
                // Once, and only for the line this case is about: a callback that creates a
                // participant for every line would recurse without end.
                if (level != provizio::dds::log_level::warning ||
                    message.find("could not read this host's network interfaces") == std::string_view::npos ||
                    reentered.exchange(true))
                {
                    return;
                }
                // Recovery-enabled deliberately: an "off" participant never registers, so it
                // would not re-enter the function that holds the lock and would prove
                // nothing.
                from_callback = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
                callback_created_participant.store(from_callback != nullptr);
            });

        // The registration that starts the monitor, reads the interfaces, fails, and emits
        // the warning. Reaching the next line at all is the assertion.
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        coordinator.wait_for_idle();

        provizio::dds::set_log_callback(std::move(previous));
        coordinator.force_enumeration_failure_for_test(false);

        passed &= EXPECT(participant != nullptr);
        // Vacuity guard: silence from the callback would let this case pass without ever
        // exercising the path, e.g. if the warning stopped being emitted at all.
        passed &= EXPECT(reentered.load());
        passed &= EXPECT(callback_created_participant.load());

        from_callback.reset();

        std::cout << "log_callback_may_create_participant: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    int test_safety_net_reopens_dead_monitor()
    {
        using provizio::dds::detail::network_recovery_coordinator;

        bool passed = true;
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        auto &coordinator = network_recovery_coordinator::instance();
        coordinator.wait_for_idle();

        passed &= EXPECT(coordinator.monitor_alive_for_test());

        if (!coordinator.kill_monitor_for_test())
        {
            // Windows: the OS owns the notification thread, so there is no worker that
            // can die and nothing for the revival path to do.
            std::cout << "safety_net_reopens_dead_monitor: SKIP (no killable worker on this platform)" << '\n';
            return 0;
        }

        // The worker has to be scheduled before it observes the wake-up and clears its
        // liveness flag, so poll rather than assume.
        constexpr std::chrono::seconds worker_exit_timeout{5};
        constexpr std::chrono::milliseconds worker_exit_poll_interval{10};
        const auto deadline = std::chrono::steady_clock::now() + worker_exit_timeout;
        while (coordinator.monitor_alive_for_test() && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(worker_exit_poll_interval);
        }
        passed &= EXPECT(!coordinator.monitor_alive_for_test());

        // This is what stops a process from silently losing auto-recovery for the rest
        // of its life after one unrecoverable channel error.
        coordinator.run_safety_net_tick_for_test();
        passed &= EXPECT(coordinator.monitor_alive_for_test());

        std::cout << "safety_net_reopens_dead_monitor: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    int test_safety_net_retries_failed_rebuild()
    {
        using provizio::dds::detail::network_recovery_coordinator;

        bool passed = true;
        pub_sub_fixture fixture;
        const std::string topic_name{"provizio_dds_network_recovery_retry_topic"};

        fixture.participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        fixture.subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
            fixture.participant, topic_name, [&fixture](const std_msgs::msg::String &message) {
                const std::lock_guard<std::mutex> lock{fixture.mutex};
                fixture.last_message = message.data();
                fixture.received_total.fetch_add(1);
                fixture.cv.notify_all();
            });
        fixture.publisher =
            provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(fixture.participant, topic_name);

        constexpr std::chrono::seconds baseline_timeout{10};
        passed &= EXPECT(publish_and_wait_for(fixture, "before-failed-reset", baseline_timeout));
        passed &= EXPECT(!fixture.participant->needs_network_recovery_retry());

        // Make the rebuild fail the way a resource-starved host would: the old
        // participant and all endpoint state are already gone by then, so the
        // participant is left inert with nothing scheduled to fix it.
        provizio::dds::detail::fail_next_participant_creation_for_test();
        fixture.participant->trigger_network_recovery_reset();
        passed &= EXPECT(fixture.participant->needs_network_recovery_retry());

        auto &coordinator = network_recovery_coordinator::instance();
        coordinator.wait_for_idle();
        const auto resets_before = coordinator.reset_count_for_test();

        // The tick must notice and retry. Note the reset above went straight through the
        // participant, never through the coordinator — so the tick has to ask the
        // participants themselves rather than trust bookkeeping of its own resets.
        coordinator.run_safety_net_tick_for_test();

        passed &= EXPECT(!fixture.participant->needs_network_recovery_retry());
        passed &= EXPECT(coordinator.reset_count_for_test() == resets_before + 1);
        // The point of the retry: communication actually resumes.
        passed &= EXPECT(publish_and_wait_for(fixture, "after-retry", std::chrono::seconds{15}));

        std::cout << "safety_net_retries_failed_rebuild: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    int test_print_snapshot()
    {
        // Machine-readable dump, one "<interface> <address>/<prefix>" per line. Used by
        // carrier_recovery_test.sh to assert what the snapshot filters admit before and
        // after an interface transition.
        for (const auto &entry : provizio::dds::detail::capture_address_snapshot())
        {
            std::cout << entry.interface_name << ' ' << entry.address_text << '/' << entry.prefix_length << '\n';
        }
        return 0;
    }

    int test_await_reset(const std::string &timeout_arg)
    {
        using provizio::dds::detail::network_recovery_coordinator;

        // Driven by carrier_recovery_test.sh inside a network namespace: create a
        // recovery-enabled participant while the target interface is not yet
        // operationally up, then wait for the coordinator to rebuild it once the
        // interface comes up. That exercises the whole chain end to end — kernel
        // link/address event → IFF_RUNNING snapshot delta → participant reset — against
        // a real kernel transition rather than an injected one.
        constexpr int default_await_timeout_sec = 30;
        int timeout_sec = default_await_timeout_sec;
        try
        {
            timeout_sec = std::stoi(timeout_arg);
        }
        catch (const std::exception &)
        {
            std::cerr << "await_reset: bad timeout '" << timeout_arg << "'" << '\n';
            return 2;
        }

        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        auto &coordinator = network_recovery_coordinator::instance();
        // Flush the line so the driving script can see we are armed before it changes
        // the interface state.
        std::cout << "await_reset: armed ("
                  << provizio::dds::detail::capture_address_snapshot().size()
                  // Flushed explicitly: the driving shell script waits for this line
                  // before it changes the interface state.
                  << " interface address(es) visible)" << '\n'
                  << std::flush;

        constexpr std::chrono::milliseconds await_poll_interval{100};
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{timeout_sec};
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (coordinator.reset_count_for_test() > 0)
            {
                std::cout << "await_reset: PASS (" << coordinator.reset_count_for_test() << " reset(s), "
                          << provizio::dds::detail::capture_address_snapshot().size()
                          << " interface address(es) now visible)" << '\n';
                return 0;
            }
            std::this_thread::sleep_for(await_poll_interval);
        }

        std::cout << "await_reset: FAIL (no reset within " << timeout_sec << "s; "
                  << coordinator.skipped_reset_count_for_test() << " burst(s) judged no-change)" << '\n';
        return 1;
    }
}  // namespace

int main(int argc, char **argv)
{
    // argv is a C-style array; indexing it is pointer arithmetic under the
    // hood, which clang-tidy flags. Convert once into a vector of string_views
    // and use that for the rest of main — no further argv[N] reads needed.
    const std::vector<std::string_view> args(
        argv, std::next(argv, argc));  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    if (args.size() < 2)
    {
        std::cerr << "usage: " << args[0] << " <subcommand> [args]" << '\n';
        return 1;
    }

    const std::string subcommand{args[1]};
    if (subcommand == "logging")
    {
        return test_logging();
    }
    if (subcommand == "env_recovery")
    {
        if (args.size() < 3)
        {
            std::cerr << "usage: " << args[0] << " env_recovery <on|off>" << '\n';
            return 1;
        }
        return test_env_recovery(std::string{args[2]});
    }
    if (subcommand == "env_recovery_garbage")
    {
        return test_env_recovery_garbage();
    }
    if (subcommand == "snapshot")
    {
        return test_snapshot();
    }
    if (subcommand == "runtime_idle")
    {
        return test_runtime_idle();
    }
    if (subcommand == "reset_roundtrip")
    {
        return test_reset_roundtrip();
    }
    if (subcommand == "reset_disabled")
    {
        return test_reset_disabled();
    }
    if (subcommand == "reset_refreshes_fastdds_interface_cache")
    {
        return test_reset_refreshes_fastdds_interface_cache();
    }
    if (subcommand == "coalescer_skips_no_change")
    {
        return test_coalescer_skips_no_change();
    }
    if (subcommand == "rebuild_on_address_change")
    {
        return test_rebuild_on_address_change();
    }
    if (subcommand == "unreadable_interfaces_warn_once_per_streak")
    {
        return test_unreadable_interfaces_warn_once_per_streak();
    }
    if (subcommand == "unreadable_interfaces_are_not_a_change")
    {
        return test_unreadable_interfaces_are_not_a_change();
    }
    if (subcommand == "no_rebuild_on_address_loss")
    {
        return test_no_rebuild_on_address_loss();
    }
    if (subcommand == "no_baseline_rebuilds_for_first_readable_list")
    {
        return test_no_baseline_rebuilds_for_first_readable_list();
    }
    if (subcommand == "no_baseline_empty_list_is_not_a_rebuild")
    {
        return test_no_baseline_empty_list_is_not_a_rebuild();
    }
    if (subcommand == "coalescer_resets_on_transient_flap")
    {
        return test_coalescer_resets_on_transient_flap();
    }
    if (subcommand == "wait_for_idle_blocks_during_reset")
    {
        return test_wait_for_idle_blocks_during_reset();
    }
    if (subcommand == "reentrant_listener_during_reset")
    {
        return test_reentrant_listener_during_reset();
    }
    if (subcommand == "concurrent_make_publisher_during_reset")
    {
        return test_concurrent_make_publisher_during_reset();
    }

    if (subcommand == "snapshot_prefix_length")
    {
        return test_snapshot_prefix_length();
    }
    if (subcommand == "extra_interfaces_env")
    {
        return test_extra_interfaces_env();
    }
    if (subcommand == "safety_net_no_rebuild_on_address_loss")
    {
        return test_safety_net_no_rebuild_on_address_loss();
    }
    if (subcommand == "safety_net_env_clamped")
    {
        return test_safety_net_env_clamped();
    }
    if (subcommand == "safety_net_detects_missed_change")
    {
        return test_safety_net_detects_missed_change();
    }
    if (subcommand == "log_callback_may_create_participant")
    {
        return test_log_callback_may_create_participant();
    }

    if (subcommand == "safety_net_reopens_dead_monitor")
    {
        return test_safety_net_reopens_dead_monitor();
    }
    if (subcommand == "safety_net_retries_failed_rebuild")
    {
        return test_safety_net_retries_failed_rebuild();
    }
    if (subcommand == "print_snapshot")
    {
        return test_print_snapshot();
    }
    if (subcommand == "await_reset")
    {
        return test_await_reset(args.size() > 2 ? std::string{args[2]} : std::string{"30"});
    }

    std::cerr << "unknown subcommand: " << subcommand << '\n';
    return 1;
}
