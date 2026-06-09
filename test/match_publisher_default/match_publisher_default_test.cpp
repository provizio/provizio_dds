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
// Subcommand-driven tests for the "match publisher" reader-reliability
// default. A subscriber left at the default reliability
// (provizio::dds::match_publisher_reliability_qos) DEFERS creating its
// DataReader until the first matching DataWriter is discovered, then builds
// the reader with that writer's offered reliability. Each ctest entry runs the
// same binary with a different subcommand so per-case failure is isolated,
// mirroring discovered_endpoints_test / network_recovery_test.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/qos_defaults.h"
#include "provizio/dds/subscriber.h"

#include <std_msgs/msg/StringPubSubTypes.hpp>

namespace
{
    using namespace std::chrono_literals;

    using string_pub_sub_type = std_msgs::msg::StringPubSubType;

    // Discovery / matching is asynchronous; this is the generous upper bound for
    // a cross-participant match to settle on a busy runner. Mirrors the
    // discovered_endpoints test's k_discovery_timeout.
    constexpr auto k_match_timeout = 15s;

    // The "promptly returns 0 / stays deferred" assertions must NOT wait for a
    // match that will never come — they only need to be longer than the time it
    // takes get_num_matched_publishers to take its lock and observe the null
    // reader, and longer than a typical discovery round so a (buggy) eager
    // reader would have had time to match. Kept well under the per-test ctest
    // timeout.
    constexpr auto k_deferred_observe_window = 3s;

    // A subscriber on this domain in this process discovers in-process /
    // loopback peers quickly. Two participants are used (one for the publisher,
    // one for the subscriber) so the match is a real remote SEDP match rather
    // than an intra-participant shortcut, matching how the library is used.
    constexpr auto k_domain = 0;

    bool expect(bool condition, const char *expression, const char *file, int line)
    {
        if (!condition)
        {
            std::cerr << "FAIL " << file << ":" << line << ": " << expression << '\n';
        }
        return condition;
    }
    // Function-like macro: it captures the textual expression (#cond) plus the
    // call-site __FILE__/__LINE__, which a constexpr helper can't synthesise.
    // Same shape as the network_recovery test's EXPECT.
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define EXPECT(cond) expect((cond), #cond, __FILE__, __LINE__)

    // Shared receive-tracking state for a subscriber's on-data callback.
    struct receiver
    {
        std::mutex mutex;
        std::condition_variable cv;
        std::atomic<int> received_total{0};
        std::string last_message;
    };

    // Make a default-reliability (match-publisher) subscriber whose callback
    // records into `sink`. Left at the default reliability_kind argument on
    // purpose — that is the case under test (deferred DataReader creation).
    std::shared_ptr<provizio::dds::subscriber_handle<string_pub_sub_type>> make_matching_subscriber(
        const std::shared_ptr<provizio::dds::domain_participant> &participant, const std::string &topic_name,
        receiver &sink)
    {
        return provizio::dds::make_subscriber<string_pub_sub_type>(
            participant, topic_name, [&sink](const std_msgs::msg::String &message) {
                const std::lock_guard<std::mutex> lock{sink.mutex};
                sink.last_message = message.data();
                sink.received_total.fetch_add(1);
                sink.cv.notify_all();
            });
    }

    // Make a subscriber with an EXPLICIT reliability override (eager reader).
    std::shared_ptr<provizio::dds::subscriber_handle<string_pub_sub_type>> make_explicit_subscriber(
        const std::shared_ptr<provizio::dds::domain_participant> &participant, const std::string &topic_name,
        receiver &sink, eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability)
    {
        return provizio::dds::make_subscriber<string_pub_sub_type>(
            participant, topic_name,
            [&sink](const std_msgs::msg::String &message) {
                const std::lock_guard<std::mutex> lock{sink.mutex};
                sink.last_message = message.data();
                sink.received_total.fetch_add(1);
                sink.cv.notify_all();
            },
            reliability);
    }

    // Republish `payload` periodically until the subscriber receives that exact
    // payload at least once beyond the baseline, or the timeout elapses. The
    // republish loop (rather than a single send + sleep) keeps the test
    // deterministic across discovery jitter: a sample dropped before the match
    // settles is simply followed by another. Mirrors network_recovery's
    // publish_and_wait_for.
    bool publish_and_wait_for(provizio::dds::publisher_handle<string_pub_sub_type> &publisher, receiver &sink,
                              const std::string &payload, std::chrono::seconds timeout)
    {
        constexpr std::chrono::milliseconds publish_interval{200};

        const int baseline = sink.received_total.load();
        std_msgs::msg::String message;
        message.data(payload);

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            publisher.publish(message);
            std::unique_lock<std::mutex> lock{sink.mutex};
            if (sink.cv.wait_for(lock, publish_interval,
                                 [&] { return sink.received_total.load() > baseline && sink.last_message == payload; }))
            {
                return true;
            }
        }
        return false;
    }

    // Case 1 — Deferral: a default-reliability subscriber on a topic with NO
    // publisher must not create its DataReader, yet get_num_matched_publishers
    // must still HONOUR its timeout (rather than early-returning 0 at t=0 while
    // reliability discovery is still in progress). We assert through the public
    // API:
    //   * with no writer, get_num_matched_publishers blocks ~the full timeout and
    //     then returns 0 (it did NOT short-circuit), and get_guid() is the unknown
    //     GUID (a built reader would report a real one) — the reader was never built;
    //   * a writer that appears DURING the wait is observed within the timeout —
    //     the deferred reader is built and matched, and the call returns a positive
    //     count without running out the clock. This is the behaviour the early
    //     return used to break (a short-lived publisher in the discovery→build
    //     window was missed entirely).
    int test_deferral()
    {
        bool passed = true;
        const auto participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);

        receiver sink;
        const std::string topic_name{"provizio_dds_match_publisher_deferral_topic"};
        const auto subscriber = make_matching_subscriber(participant, topic_name, sink);

        // No writer will ever appear on this topic, so the call must wait out its
        // timeout and then report 0 — proving it honours the timeout instead of
        // early-returning 0 on the still-null deferred reader.
        const auto start = std::chrono::steady_clock::now();
        const int matched = subscriber->get_num_matched_publishers(k_deferred_observe_window, 0ms);
        const auto elapsed = std::chrono::steady_clock::now() - start;

        passed &= EXPECT(matched == 0);
        // It blocked for ~the timeout. Assert a lower bound comfortably under the
        // full window (wide margin for a fast scheduler) to prove it did NOT
        // short-circuit at t=0.
        passed &= EXPECT(elapsed >= (k_deferred_observe_window - 1s));

        // The reader was never built, so there is no DataReader GUID to report.
        passed &= EXPECT(subscriber->get_guid() == provizio::dds::guid{});

        // A writer that appears mid-wait is observed within the timeout. Create the
        // publisher from a separate thread a short moment after the (blocking) wait
        // begins, so the wait must span the discovery + deferred build and return a
        // positive count — not 0, and not by timing out.
        receiver appears_sink;
        const std::string appears_topic{"provizio_dds_match_publisher_deferral_appears_topic"};
        const auto appears_subscriber = make_matching_subscriber(participant, appears_topic, appears_sink);
        const auto pub_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        std::shared_ptr<provizio::dds::publisher_handle<string_pub_sub_type>> appears_publisher;
        std::thread writer_thread{[&] {
            std::this_thread::sleep_for(300ms);
            appears_publisher = provizio::dds::make_publisher<string_pub_sub_type>(
                pub_participant, appears_topic, eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
                provizio::dds::use_default_history_depth);
        }};
        const auto appears_start = std::chrono::steady_clock::now();
        const int appears_matched = appears_subscriber->get_num_matched_publishers(k_match_timeout, 0ms);
        const auto appears_elapsed = std::chrono::steady_clock::now() - appears_start;
        writer_thread.join();

        passed &= EXPECT(appears_matched > 0);                // saw the writer that appeared mid-wait
        passed &= EXPECT(appears_elapsed < k_match_timeout);  // returned because it matched, not on timeout

        std::cout << "deferral: " << (passed ? "PASS" : "FAIL") << " (matched=" << matched << ", waited "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                  << " ms; appears_matched=" << appears_matched << ", waited "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(appears_elapsed).count() << " ms)" << '\n';
        return passed ? 0 : 1;
    }

    // Shared body for the two "default subscriber adopts the publisher's
    // reliability" cases. The publisher is created FIRST (so the subscriber's
    // deferred build resolves against an already-discoverable writer, also
    // exercising the writer-before-subscriber cache path), then the default
    // subscriber. Success = the subscriber matches AND receives a published
    // sample.
    //
    // For writer_reliability == BEST_EFFORT this is the discriminating case: a
    // blanket-RELIABLE reader is RxO-INCOMPATIBLE with a BEST_EFFORT writer and
    // would never match, so reception here proves the subscriber adopted
    // BEST_EFFORT rather than forcing RELIABLE.
    int run_adopts(eprosima::fastdds::dds::ReliabilityQosPolicyKind writer_reliability, const std::string &topic_name,
                   const std::string &label)
    {
        bool passed = true;

        const auto pub_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        const auto sub_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);

        // Publisher first, with the reliability under test.
        auto publisher = provizio::dds::make_publisher<string_pub_sub_type>(
            pub_participant, topic_name, writer_reliability, provizio::dds::use_default_history_depth);

        receiver sink;
        const auto subscriber = make_matching_subscriber(sub_participant, topic_name, sink);

        // The default subscriber must match and RECEIVE. publish_and_wait_for
        // republishes until a sample lands, so this also implicitly proves the
        // deferred reader got built (no reader → no reception).
        passed &= EXPECT(publish_and_wait_for(*publisher, sink, "match-" + label, k_match_timeout));

        // Once it has received, it must report a matched publisher — i.e. the
        // deferred reader exists now and the listener saw the writer. This also
        // confirms get_num_matched_publishers transitions from the deferred 0 to
        // a real count.
        const int matched = subscriber->get_num_matched_publishers(k_match_timeout, 0ms);
        passed &= EXPECT(matched > 0);

        // A real DataReader was built, so a concrete GUID is now reported (it
        // was the unknown GUID while deferred — see test_deferral).
        passed &= EXPECT(subscriber->get_guid() != provizio::dds::guid{});

        std::cout << label << ": " << (passed ? "PASS" : "FAIL") << " (received=" << sink.received_total.load()
                  << ", matched=" << matched << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case 2 — default subscriber adopts a RELIABLE publisher.
    int test_adopts_reliable()
    {
        return run_adopts(eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
                          "provizio_dds_match_publisher_adopts_reliable_topic", "adopts_reliable");
    }

    // Case 3 — default subscriber adopts a BEST_EFFORT publisher (discriminating
    // case: proves the reader did NOT silently force RELIABLE).
    int test_adopts_best_effort()
    {
        return run_adopts(eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS,
                          "provizio_dds_match_publisher_adopts_best_effort_topic", "adopts_best_effort");
    }

    // Shared body for the two "explicit reliability override builds eagerly"
    // cases. With an explicit reliability the subscriber must NOT defer: its
    // DataReader is created up-front. We prove eager creation by asserting
    // get_guid() is a real GUID immediately after construction, BEFORE any
    // publisher exists — a deferred reader would still report the unknown GUID
    // here. Then a matching publisher is created and reception is confirmed,
    // i.e. the historical behaviour is unchanged.
    int run_explicit(eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability, const std::string &topic_name,
                     const std::string &label)
    {
        bool passed = true;

        const auto sub_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);

        // Subscriber first, with an explicit reliability and NO publisher yet.
        receiver sink;
        const auto subscriber = make_explicit_subscriber(sub_participant, topic_name, sink, reliability);

        // Eager build: a concrete DataReader GUID is available straight away,
        // with no publisher in sight. (The match-mode default would report the
        // unknown GUID until a writer is discovered — see test_deferral.)
        passed &= EXPECT(subscriber->get_guid() != provizio::dds::guid{});

        // Now bring up a matching publisher and confirm reception is unchanged.
        const auto pub_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        auto publisher = provizio::dds::make_publisher<string_pub_sub_type>(pub_participant, topic_name, reliability,
                                                                            provizio::dds::use_default_history_depth);

        passed &= EXPECT(publish_and_wait_for(*publisher, sink, "explicit-" + label, k_match_timeout));

        const int matched = subscriber->get_num_matched_publishers(k_match_timeout, 0ms);
        passed &= EXPECT(matched > 0);

        std::cout << label << ": " << (passed ? "PASS" : "FAIL") << " (received=" << sink.received_total.load()
                  << ", matched=" << matched << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case 4a — explicit BEST_EFFORT subscriber builds eagerly and receives.
    int test_explicit_best_effort()
    {
        return run_explicit(eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS,
                            "provizio_dds_match_publisher_explicit_best_effort_topic", "explicit_best_effort");
    }

    // Case 4b — explicit RELIABLE subscriber builds eagerly and receives.
    int test_explicit_reliable()
    {
        return run_explicit(eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
                            "provizio_dds_match_publisher_explicit_reliable_topic", "explicit_reliable");
    }

    // Case 5 — Lifetime: a match-publisher (default) subscriber must keep working
    // after the caller releases its OWN handle to the participant. The subscriber
    // holds a strong reference to the participant, so the participant — and the
    // discovery listener that drives the deferred reader build — must stay alive
    // and resolve the deferred reader once a writer appears, even though the
    // caller no longer holds the participant. This is the
    // make_subscriber(make_domain_participant(), ...) temporary-participant idiom
    // that simplest_subscriber / ros_interop rely on. The publisher is brought up
    // AFTER the drop (subscriber-first), so the subscriber is parked waiting for
    // writer discovery — exactly the path that must survive the caller dropping
    // its participant handle.
    int test_participant_dropped()
    {
        bool passed = true;

        auto sub_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);

        receiver sink;
        const std::string topic_name{"provizio_dds_match_publisher_participant_dropped_topic"};
        const auto subscriber = make_matching_subscriber(sub_participant, topic_name, sink);

        // Release the caller's participant handle: only the subscriber keeps it
        // alive from here on.
        sub_participant.reset();

        const auto pub_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        auto publisher = provizio::dds::make_publisher<string_pub_sub_type>(
            pub_participant, topic_name, eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
            provizio::dds::use_default_history_depth);

        passed &= EXPECT(publish_and_wait_for(*publisher, sink, "participant-dropped", k_match_timeout));

        const int matched = subscriber->get_num_matched_publishers(k_match_timeout, 0ms);
        passed &= EXPECT(matched > 0);
        passed &= EXPECT(subscriber->get_guid() != provizio::dds::guid{});

        std::cout << "participant_dropped: " << (passed ? "PASS" : "FAIL")
                  << " (received=" << sink.received_total.load() << ", matched=" << matched << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case 6 — Heterogeneous-writer re-derive: when the writer whose reliability the topic
    // adopted (match-first) leaves while a differently-configured writer remains, the adopted
    // reliability must be re-derived from a still-live writer — otherwise a subscriber created
    // afterwards adopts a stale reliability that matches none of the remaining writers (the
    // RELIABLE-cached / only-BEST_EFFORT-left data-loss path).
    //
    // Sequence (synchronised via the discovery callback, which fires AFTER the internal
    // resolve/remove on the same discovery thread, so observing an event means the cache is
    // already updated):
    //   1. RELIABLE writer A discovered first  -> topic adopts RELIABLE.
    //   2. BEST_EFFORT writer B discovered next -> match-first keeps RELIABLE; B is counted.
    //   3. A removed -> re-derive: only B's BEST_EFFORT remains, so adopted becomes BEST_EFFORT.
    //   4. A NEW default subscriber must now adopt BEST_EFFORT and receive from B. If the adopted
    //      value were left stale at RELIABLE, a RELIABLE reader would not match B (BEST_EFFORT
    //      writer) and would receive nothing — so reception is the discriminating assertion.
    int test_heterogeneous_rederive()
    {
        bool passed = true;

        // Declared BEFORE the participants so they outlive teardown-triggered discovery
        // callbacks (a writer going away at scope exit can still fire into this state).
        std::mutex events_mutex;
        std::condition_variable events_cv;
        struct writer_event
        {
            eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability;
            bool discovered;
        };
        std::vector<writer_event> writer_events;

        const std::string topic_name{"provizio_dds_match_publisher_heterogeneous_rederive_topic"};

        const auto sub_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        sub_participant->on_discovered_endpoint(
            [&](provizio::dds::domain_participant &, const std::string &topic, const std::string & /*type*/,
                provizio::dds::endpoint_kind kind, bool discovered,
                eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability,
                eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
                if (kind != provizio::dds::endpoint_kind::data_writer || topic != topic_name)
                {
                    return;
                }
                const std::lock_guard<std::mutex> lock{events_mutex};
                writer_events.push_back({reliability, discovered});
                events_cv.notify_all();
            },
            provizio::dds::endpoint_kind::data_writer);

        // Wait until a writer event matching (reliability, discovered) has been observed.
        const auto wait_for_writer_event = [&](eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability,
                                               bool discovered) {
            std::unique_lock<std::mutex> lock{events_mutex};
            return events_cv.wait_for(lock, k_match_timeout, [&] {
                return std::any_of(writer_events.begin(), writer_events.end(), [&](const writer_event &event) {
                    return event.reliability == reliability && event.discovered == discovered;
                });
            });
        };

        // Separate participants so each writer can be discovered / removed independently.
        const auto pub_reliable_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        const auto pub_best_effort_participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);

        // 1. RELIABLE writer discovered FIRST -> topic adopts RELIABLE.
        auto reliable_writer = provizio::dds::make_publisher<string_pub_sub_type>(
            pub_reliable_participant, topic_name, eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
            provizio::dds::use_default_history_depth);
        passed &= EXPECT(wait_for_writer_event(eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS, true));

        // 2. BEST_EFFORT writer discovered next -> match-first keeps RELIABLE adopted.
        auto best_effort_writer = provizio::dds::make_publisher<string_pub_sub_type>(
            pub_best_effort_participant, topic_name, eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS,
            provizio::dds::use_default_history_depth);
        passed &= EXPECT(wait_for_writer_event(eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS, true));

        // 3. Remove the RELIABLE writer -> re-derive adopted to the surviving BEST_EFFORT.
        reliable_writer.reset();
        passed &= EXPECT(wait_for_writer_event(eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS, false));

        // 4. A new default subscriber must adopt BEST_EFFORT and receive from the surviving writer.
        receiver sink;
        const auto subscriber = make_matching_subscriber(sub_participant, topic_name, sink);
        passed &= EXPECT(publish_and_wait_for(*best_effort_writer, sink, "rederive", k_match_timeout));

        const int matched = subscriber->get_num_matched_publishers(k_match_timeout, 0ms);
        passed &= EXPECT(matched > 0);

        std::cout << "heterogeneous_rederive: " << (passed ? "PASS" : "FAIL")
                  << " (received=" << sink.received_total.load() << ", matched=" << matched << ")" << '\n';
        return passed ? 0 : 1;
    }
}  // namespace

int main(int argc, char **argv)
{
    // argv is a C-style array; indexing it is pointer arithmetic under the hood, which
    // clang-tidy flags. Convert once into a vector of string_views and use that for the
    // rest of main — no further argv[N] reads needed. Mirrors network_recovery_test.
    const std::vector<std::string_view> args(
        argv, std::next(argv, argc));  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    if (args.size() < 2)
    {
        std::cerr << "usage: " << args[0] << " <subcommand>\n";
        return 1;
    }
    const std::string_view subcommand = args[1];
    if (subcommand == "deferral")
    {
        return test_deferral();
    }
    if (subcommand == "adopts_reliable")
    {
        return test_adopts_reliable();
    }
    if (subcommand == "adopts_best_effort")
    {
        return test_adopts_best_effort();
    }
    if (subcommand == "explicit_best_effort")
    {
        return test_explicit_best_effort();
    }
    if (subcommand == "explicit_reliable")
    {
        return test_explicit_reliable();
    }
    if (subcommand == "participant_dropped")
    {
        return test_participant_dropped();
    }
    if (subcommand == "heterogeneous_rederive")
    {
        return test_heterogeneous_rederive();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
