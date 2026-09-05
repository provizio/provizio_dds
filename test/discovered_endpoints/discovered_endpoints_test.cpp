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
// Subcommand-driven tests for the discovery-callback / type-registry
// additions to provizio::dds::domain_participant. Each ctest entry runs the
// same binary with a different subcommand so per-case failure is isolated.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "detail/test_cpu_time.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/known_types_dispatcher.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/subscriber.h"

#include <std_msgs/msg/StringPubSubTypes.hpp>

namespace
{
    using namespace std::chrono_literals;

    // Discovery is asynchronous — Fast-DDS's default initial announcement period
    // is sub-second but anything cross-participant can drift on a busy runner. Scaled
    // like the ctest ceiling and the cases' own holds: under a sanitizer everything is
    // several times slower, and a deadline that gates reaching an assertion must not be
    // the one thing that stays put.
    constexpr auto k_discovery_timeout = 15s * PROVIZIO_DDS_TEST_TIMEOUT_SCALE;

    // Recorded discovery event for the callback to deposit and the test thread
    // to inspect under a mutex.
    struct event
    {
        std::string topic;
        std::string type;
        provizio::dds::endpoint_kind kind;
        bool discovered;
    };

    // Atomically wait for `predicate` to become true on `events` (vector
    // guarded by mu/cv), up to `timeout`. Returns true on success.
    template <typename Pred>
    bool wait_for_event(std::mutex &mu, std::condition_variable &cv, const std::vector<event> &events,
                        std::chrono::steady_clock::duration timeout, Pred predicate)
    {
        std::unique_lock<std::mutex> lock{mu};
        return cv.wait_for(lock, timeout, [&] { return predicate(events); });
    }

    int test_type_registry()
    {
        const auto participant = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);

        // Pristine — no types registered yet.
        if (participant->is_known_type("std_msgs::msg::dds_::String_"))
        {
            std::cerr << "is_known_type returned true before any register_type call\n";
            return 1;
        }
        if (!participant->known_types().empty())
        {
            std::cerr << "known_types() not empty before any register_type call\n";
            return 1;
        }

        // After register_type<T>(), is_known_type / known_types should see it.
        participant->register_type<std_msgs::msg::StringPubSubType>();
        if (!participant->is_known_type("std_msgs::msg::dds_::String_"))
        {
            std::cerr << "is_known_type returned false after register_type<StringPubSubType>\n";
            return 1;
        }
        const auto names = participant->known_types();
        if (std::find(names.begin(), names.end(), "std_msgs::msg::dds_::String_") == names.end())
        {
            std::cerr << "known_types() missing String_ after registration\n";
            return 1;
        }

        // Idempotent registration.
        participant->register_type<std_msgs::msg::StringPubSubType>();
        if (participant->known_types().size() != names.size())
        {
            std::cerr << "duplicate register_type<T>() changed known_types().size()\n";
            return 1;
        }
        return 0;
    }

    // Case: destroying a participant while one of its discovery callbacks is still running
    // does not SPIN. Whether it also WAITS is reported but not asserted -- see the note
    // further down for why that half cannot be pinned portably.
    //
    // The bug this guards was live: Fast-DDS' DomainParticipantImpl::disable() detaches the
    // participant listener with set_listener()'s default timeout of
    // std::chrono::seconds::max(), which becomes a steady_clock deadline of
    // time_point::max(). libstdc++ converts a deadline on a clock other than its native one
    // by adding it to the native clock's now(), which overflows that far out, so the wait
    // returns instantly, the re-check against the caller's clock says the deadline has not
    // passed, and the predicate loop goes round again immediately -- a spin at 100% of a
    // core for as long as the callback runs. Observed on aarch64 CI runners as ~5% of runs
    // hanging for tens of seconds inside delete_participant. domain_participant detaches the
    // listener itself first, with a finite timeout, which takes the ordinary blocking path
    // and leaves Fast-DDS' own call with its predicate already satisfied.
    //
    // The CPU assertion only FAILS on a toolchain where the spin exists (libstdc++ 9 and
    // anything else whose condition_variable is not natively steady_clock) -- it is the CI
    // runners that gate this: on a toolchain without the spin, destruction consumes almost
    // no CPU whether or not it waited, so the case passes there without proving much.
    int test_teardown_does_not_spin()
    {
        std::mutex mu;
        std::condition_variable cv;
        bool callback_entered = false;

        // Long enough that a spin is unmistakable against scheduling noise, short enough to
        // keep the case quick.
        const auto callback_hold = std::chrono::milliseconds{600 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};

        auto pa = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);
        const auto pb = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);

        pa->on_discovered_endpoint([&](provizio::dds::domain_participant &, const std::string &, const std::string &,
                                       provizio::dds::endpoint_kind, bool discovered,
                                       eprosima::fastdds::dds::ReliabilityQosPolicyKind,
                                       eprosima::fastdds::dds::DurabilityQosPolicyKind) {
            if (!discovered)
            {
                return;
            }
            {
                const std::lock_guard<std::mutex> lock{mu};
                callback_entered = true;
            }
            cv.notify_all();
            // Held deliberately. Blocking in a callback is against this library's contract,
            // and that is the point: it is the state in which the participant listener
            // cannot be detached, which is what the destructor then has to wait for.
            std::this_thread::sleep_for(callback_hold);
        });

        const auto pub =
            provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, "provizio_dds_teardown_wait_topic");

        {
            std::unique_lock<std::mutex> lock{mu};
            if (!cv.wait_for(lock, k_discovery_timeout, [&] { return callback_entered; }))
            {
                std::cout << "teardown_does_not_spin: FAIL (no discovery callback arrived)" << '\n';
                return 1;
            }
        }

        // Destroy while the callback is still inside its hold.
        const auto cpu_before = provizio::dds::test::process_cpu_seconds();
        const auto started = std::chrono::steady_clock::now();
        pa.reset();
        const auto took = std::chrono::steady_clock::now() - started;
        const auto cpu_used = (provizio::dds::test::process_cpu_seconds()) - cpu_before;

        // The assertion, and the only one: destruction did not SPIN. A spin burns the wall
        // time it covers, so any CPU figure near the elapsed time is the defect; a blocking
        // wait, and an ordinary quick teardown alike, consume almost nothing.
        bool passed = true;
        const auto budget = std::chrono::duration_cast<std::chrono::duration<double>>(callback_hold).count() / 2.0;
        if (cpu_used >= budget)
        {
            std::cerr << "participant destruction SPUN: burned " << (cpu_used * 1000.0) << " ms of CPU over "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(took).count() << " ms\n";
            passed = false;
        }

        // Whether destruction WAITED is reported but not asserted, deliberately. Whether an
        // in-flight discovery dispatch of ours is inside the region Fast-DDS counts for
        // set_listener turns out to vary by platform -- on some it is, and destruction
        // blocks for the whole hold; on others the callback has already left that region by
        // the time this thread gets here, and destruction is instant. Both are correct; only
        // spinning is not. Asserting the wait would make this case fail on the platforms
        // where the overlap does not arise, which is a false alarm about a property the fix
        // never promised.
        const auto waited_for_callback = took > callback_hold / 4;

        std::cout << "teardown_does_not_spin: " << (passed ? "PASS" : "FAIL") << " (took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(took).count() << " ms, burned "
                  << (cpu_used * 1000.0) << " ms of CPU"
                  << (waited_for_callback ? "; waited for the callback"
                                          : "; callback had already left the counted "
                                            "region, no overlap to wait for")
                  << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: a network-recovery RESET does not spin either. The twin of the case above, and
    // the one that matters more in practice: a reset runs on a network event, which is
    // exactly when discovery callbacks are in flight, whereas a destructor usually runs on a
    // quiet process. reset_participant's Phase 4 destroys the old participant, so it reaches
    // the same Fast-DDS listener swap with the same infinite default, and for a while it did
    // so with no detach of ours in front of it -- the destructor had one and this path did
    // not.
    int test_reset_does_not_spin()
    {
        std::mutex mu;
        std::condition_variable cv;
        bool callback_entered = false;

        const auto callback_hold = std::chrono::milliseconds{600 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};

        // network_recovery_mode::on is LOAD-BEARING, not incidental. trigger_network_recovery_reset()
        // opens with `if (!recovery_enabled) { return; }`, and resolve_network_recovery_enabled maps
        // ::off to false -- so with ::off this case would call a function that returns at its first
        // statement, never reach Phase 4, and pass whatever the code under test did. It would be a
        // test that cannot fail. network_recovery_test's reset_disabled asserts that no-op
        // positively, which is the same fact from the other side.
        const auto pa = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        const auto pb = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);

        pa->on_discovered_endpoint([&](provizio::dds::domain_participant &, const std::string &, const std::string &,
                                       provizio::dds::endpoint_kind, bool discovered,
                                       eprosima::fastdds::dds::ReliabilityQosPolicyKind,
                                       eprosima::fastdds::dds::DurabilityQosPolicyKind) {
            if (!discovered)
            {
                return;
            }
            {
                const std::lock_guard<std::mutex> lock{mu};
                callback_entered = true;
            }
            cv.notify_all();
            // Held for the same reason as in teardown_does_not_spin: an in-flight callback is
            // the condition the infinite-timeout listener swap spins on.
            std::this_thread::sleep_for(callback_hold);
        });

        const auto pub =
            provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, "provizio_dds_reset_wait_topic");

        {
            std::unique_lock<std::mutex> lock{mu};
            if (!cv.wait_for(lock, k_discovery_timeout, [&] { return callback_entered; }))
            {
                std::cout << "reset_does_not_spin: FAIL (no discovery callback arrived)" << '\n';
                return 1;
            }
        }

        // Reset while the callback is still inside its hold. This is the same call the
        // coordinator makes on a confirmed network change; driving it directly is what makes
        // the moment observable.
        const auto generation_before = pa->participant_generation();
        const auto cpu_before = provizio::dds::test::process_cpu_seconds();
        const auto started = std::chrono::steady_clock::now();
        pa->trigger_network_recovery_reset();
        const auto took = std::chrono::steady_clock::now() - started;
        const auto cpu_used = provizio::dds::test::process_cpu_seconds() - cpu_before;

        // Same single assertion as its twin, and for the same reasons -- a spin burns the wall
        // time it covers, while a blocking wait and a quick reset alike burn almost nothing;
        // and whether the reset WAITED is platform-dependent, so it is reported not asserted.
        bool passed = true;
        const auto budget = std::chrono::duration_cast<std::chrono::duration<double>>(callback_hold).count() / 2.0;
        if (cpu_used >= budget)
        {
            std::cerr << "network-recovery reset SPUN: burned " << (cpu_used * 1000.0) << " ms of CPU over "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(took).count() << " ms\n";
            passed = false;
        }

        // The reset must also have WORKED, not merely been cheap: a rebuild that failed
        // part-way would burn no CPU and pass the assertion above on its own, leaving the
        // case green over an inert participant.
        // Asserted FIRST and separately: everything below measures a reset that has to have
        // happened. A short-circuited call burns no CPU and needs no retry, so without this the
        // case would go green over a function that returned immediately -- which is exactly what
        // it did while it was written against network_recovery_mode::off.
        if (pa->participant_generation() == generation_before)
        {
            std::cerr << "the participant was never rebuilt (generation stayed at " << generation_before
                      << "), so this case measured nothing\n";
            passed = false;
        }

        if (pa->needs_network_recovery_retry())
        {
            std::cerr << "the reset left the participant needing a retry, so the CPU figure above "
                         "describes a rebuild that did not complete\n";
            passed = false;
        }

        std::cout << "reset_does_not_spin: " << (passed ? "PASS" : "FAIL") << " (took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(took).count() << " ms, burned "
                  << (cpu_used * 1000.0) << " ms of CPU)" << '\n';
        return passed ? 0 : 1;
    }

    int test_discovers_writer()
    {
        // Declared BEFORE the participants so they outlive them: at scope exit
        // participant teardown (e.g. pb's writer going away) can still deliver a
        // REMOVED_WRITER discovery callback to pa, which touches these — they
        // must be destroyed AFTER the participants, not before.
        std::mutex mu;
        std::condition_variable cv;
        std::vector<event> events;

        // Two participants on domain 0 in the same process — Fast-DDS discovers
        // them via intra-process / loopback announcements.
        const auto pa = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);
        const auto pb = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);

        pa->on_discovered_endpoint([&](provizio::dds::domain_participant &, const std::string &topic,
                                       const std::string &type, provizio::dds::endpoint_kind kind, bool discovered,
                                       eprosima::fastdds::dds::ReliabilityQosPolicyKind /*reliability*/,
                                       eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
            const std::lock_guard<std::mutex> lock{mu};
            events.push_back({topic, type, kind, discovered});
            cv.notify_all();
        });

        // Publisher on B → A should see a writer discovery.
        const std::string topic_name = "provizio_dds_discovery_test_topic";
        const auto pub = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, topic_name);

        const bool got = wait_for_event(mu, cv, events, k_discovery_timeout, [&](const auto &ev) {
            return std::any_of(ev.begin(), ev.end(), [&](const event &e) {
                return e.discovered && e.kind == provizio::dds::endpoint_kind::data_writer && e.topic == topic_name &&
                       e.type == "std_msgs::msg::dds_::String_";
            });
        });
        if (!got)
        {
            std::cerr << "no writer discovery event observed within "
                      << std::chrono::duration_cast<std::chrono::seconds>(k_discovery_timeout).count() << "s\n";
            return 1;
        }
        return 0;
    }

    int test_kind_filter()
    {
        // Declared before the participants so they outlive teardown-triggered
        // discovery callbacks (see test_discovers_writer for the rationale).
        std::mutex mu;
        std::condition_variable cv;
        std::vector<event> events;

        // With kinds = data_reader, a writer on B must NOT fire the callback,
        // but a subscriber on B should.
        const auto pa = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);
        const auto pb = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);

        pa->on_discovered_endpoint(
            [&](provizio::dds::domain_participant &, const std::string &topic, const std::string &type,
                provizio::dds::endpoint_kind kind, bool discovered,
                eprosima::fastdds::dds::ReliabilityQosPolicyKind /*reliability*/,
                eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
                const std::lock_guard<std::mutex> lock{mu};
                events.push_back({topic, type, kind, discovered});
                cv.notify_all();
            },
            provizio::dds::endpoint_kind::data_reader);

        const std::string topic_name = "provizio_dds_discovery_test_topic_filter";

        // Publisher first — should NOT fire the data_reader-only callback.
        const auto pub = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, topic_name);

        // Subscriber — SHOULD fire.
        const auto sub = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
            pb, topic_name, [](const std_msgs::msg::String &) {});

        const bool got_reader = wait_for_event(mu, cv, events, k_discovery_timeout, [&](const auto &ev) {
            return std::any_of(ev.begin(), ev.end(), [&](const event &e) {
                return e.discovered && e.kind == provizio::dds::endpoint_kind::data_reader && e.topic == topic_name;
            });
        });
        if (!got_reader)
        {
            std::cerr << "no reader discovery event observed within timeout\n";
            return 1;
        }

        // Verify no writer event was delivered (we never asked for writers).
        const std::lock_guard<std::mutex> lock{mu};
        const auto writer_fired = std::any_of(events.begin(), events.end(), [](const event &e) {
            return e.kind == provizio::dds::endpoint_kind::data_writer;
        });
        if (writer_fired)
        {
            std::cerr << "data_writer-kind event delivered despite kinds=data_reader\n";
            return 1;
        }
        return 0;
    }

    int test_constructor_callback()
    {
        // Callback passed at make_domain_participant time must be attached
        // BEFORE Fast-DDS starts discovery. Most directly testable by having
        // a publisher already up on B, then bringing up A with the callback
        // installed via the constructor — A must observe B's writer.
        const auto pb = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);
        const std::string topic_name = "provizio_dds_discovery_test_topic_ctor";
        const auto pub = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, topic_name);

        // Give Fast-DDS a moment so B is steadily announcing before A appears.
        std::this_thread::sleep_for(500ms);

        std::mutex mu;
        std::condition_variable cv;
        std::vector<event> events;

        const auto pa = provizio::dds::make_domain_participant(
            0, provizio::dds::network_recovery_mode::off,
            [&](provizio::dds::domain_participant &, const std::string &topic, const std::string &type,
                provizio::dds::endpoint_kind kind, bool discovered,
                eprosima::fastdds::dds::ReliabilityQosPolicyKind /*reliability*/,
                eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
                const std::lock_guard<std::mutex> lock{mu};
                events.push_back({topic, type, kind, discovered});
                cv.notify_all();
            });

        const bool got = wait_for_event(mu, cv, events, k_discovery_timeout, [&](const auto &ev) {
            return std::any_of(ev.begin(), ev.end(), [&](const event &e) {
                return e.discovered && e.kind == provizio::dds::endpoint_kind::data_writer && e.topic == topic_name &&
                       e.type == "std_msgs::msg::dds_::String_";
            });
        });
        if (!got)
        {
            std::cerr << "no writer discovery event via constructor-time callback within "
                      << std::chrono::duration_cast<std::chrono::seconds>(k_discovery_timeout).count() << "s\n";
            return 1;
        }
        return 0;
    }

    // Visitor helpers used by test_visit_known_type. Local classes cannot have
    // member templates (C++ standard), so these live at the surrounding
    // namespace scope.
    struct visit_counting_visitor
    {
        int calls = 0;
        template <typename> void operator()()
        {
            ++calls;
        }
    };

    struct visit_name_recorder
    {
        std::string seen_wire_name;
        int calls = 0;
        template <typename data_pub_sub_type> void operator()()
        {
            ++calls;
            data_pub_sub_type instance;
            seen_wire_name = instance.get_name();
        }
    };

    struct visit_announce_factory
    {
        std::shared_ptr<provizio::dds::domain_participant> participant;
        std::string topic_name;
        std::shared_ptr<void> handle;
        template <typename data_pub_sub_type> void operator()()
        {
            handle = provizio::dds::make_publisher<data_pub_sub_type>(participant, topic_name);
        }
    };

    int test_visit_known_type()
    {
        // Unknown type — visitor must not be invoked, return value is false.
        {
            visit_counting_visitor v;
            if (provizio::dds::visit_known_type("not::a::known::type", v))
            {
                std::cerr << "visit_known_type returned true for an unknown type\n";
                return 1;
            }
            if (v.calls != 0)
            {
                std::cerr << "visitor invoked for an unknown type\n";
                return 1;
            }
        }

        // Known type — visitor's templated operator() is invoked exactly once
        // with the matched PubSubType. The body that records the type's wire
        // name has full compile-time access to data_pub_sub_type — that's the
        // whole point of the visitor pattern.
        {
            visit_name_recorder v;
            if (!provizio::dds::visit_known_type("std_msgs::msg::dds_::String_", v))
            {
                std::cerr << "visit_known_type returned false for std_msgs::msg::dds_::String_\n";
                return 1;
            }
            if (v.calls != 1)
            {
                std::cerr << "visitor was invoked " << v.calls << " times (expected 1)\n";
                return 1;
            }
            if (v.seen_wire_name != "std_msgs::msg::dds_::String_")
            {
                std::cerr << "visitor saw the wrong type: " << v.seen_wire_name << "\n";
                return 1;
            }
        }

        // End-to-end: a non-trivial visitor that creates a typed publisher_handle
        // for the runtime-known type. This is the recorder shape — the
        // visitor's body is written once, instantiated once per known type by
        // the dispatcher's generated code, and produces a fully typed handle.
        // Side effect: make_publisher<T> internally registers T on the
        // participant, so is_known_type answers truthfully afterwards without
        // any up-front bulk registration.
        {
            const auto participant =
                provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);
            visit_announce_factory v{participant, "visit_known_type_test_topic", {}};
            if (!provizio::dds::visit_known_type("std_msgs::msg::dds_::String_", v))
            {
                std::cerr << "visit_known_type returned false for the publisher-creation case\n";
                return 1;
            }
            if (!v.handle)
            {
                std::cerr << "visitor failed to create a publisher_handle\n";
                return 1;
            }
            if (!participant->is_known_type("std_msgs::msg::dds_::String_"))
            {
                std::cerr << "lazy registration via make_publisher inside the visitor didn't populate is_known_type\n";
                return 1;
            }
        }
        return 0;
    }

    int test_callback_receives_participant()
    {
        // The callback's first argument is the observing domain_participant.
        // Verify the reference equals the wrapper make_domain_participant
        // returns even when the very first event fires during the
        // constructor-time SEDP exchange — that's the window the constructor-
        // time callback form exists to close, and the participant ref must be
        // usable across it.
        const auto pb = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);
        const std::string topic_name = "provizio_dds_discovery_test_topic_callback_ref";
        const auto pub = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, topic_name);

        // Let B steady-state-announce before A appears so A's first discovery
        // fires close to participant-creation time.
        std::this_thread::sleep_for(500ms);

        std::mutex mu;
        std::condition_variable cv;
        provizio::dds::domain_participant *seen_participant{nullptr};

        const auto pa = provizio::dds::make_domain_participant(
            0, provizio::dds::network_recovery_mode::off,
            [&](provizio::dds::domain_participant &observer, const std::string & /*topic*/,
                const std::string & /*type*/, provizio::dds::endpoint_kind /*kind*/, bool discovered,
                eprosima::fastdds::dds::ReliabilityQosPolicyKind /*reliability*/,
                eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
                if (!discovered)
                {
                    return;
                }
                {
                    const std::lock_guard<std::mutex> lock{mu};
                    seen_participant = &observer;
                }
                cv.notify_all();
            });

        std::unique_lock<std::mutex> lock{mu};
        if (!cv.wait_for(lock, k_discovery_timeout, [&] { return seen_participant != nullptr; }))
        {
            std::cerr << "no discovery event observed within timeout\n";
            return 1;
        }
        if (seen_participant != pa.get())
        {
            std::cerr << "callback received a different domain_participant than make_domain_participant returned\n";
            return 1;
        }
        return 0;
    }

    int test_survives_reset()
    {
        // Declared before the participants so they outlive teardown-triggered
        // discovery callbacks (see test_discovers_writer for the rationale).
        std::mutex mu;
        std::condition_variable cv;
        std::vector<event> events;

        // The discovery callback must continue to fire after a network-recovery
        // reset recreates the Fast-DDS participant under us.
        const auto pa = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::on);
        const auto pb = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);

        pa->on_discovered_endpoint([&](provizio::dds::domain_participant &, const std::string &topic,
                                       const std::string &type, provizio::dds::endpoint_kind kind, bool discovered,
                                       eprosima::fastdds::dds::ReliabilityQosPolicyKind /*reliability*/,
                                       eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
            const std::lock_guard<std::mutex> lock{mu};
            events.push_back({topic, type, kind, discovered});
            cv.notify_all();
        });

        const std::string topic_name = "provizio_dds_discovery_test_topic_reset";
        const auto pub = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, topic_name);

        // Baseline: callback fires before the reset.
        const bool got_before = wait_for_event(mu, cv, events, k_discovery_timeout, [&](const auto &ev) {
            return std::any_of(ev.begin(), ev.end(), [&](const event &e) {
                return e.discovered && e.kind == provizio::dds::endpoint_kind::data_writer && e.topic == topic_name;
            });
        });
        if (!got_before)
        {
            std::cerr << "no pre-reset discovery event\n";
            return 1;
        }

        // Drain so we can clearly detect a post-reset re-discovery.
        {
            const std::lock_guard<std::mutex> lock{mu};
            events.clear();
        }

        // Recreate A's underlying participant.
        pa->trigger_network_recovery_reset();

        // The new participant re-runs discovery from scratch and should pick
        // up B's still-alive publisher; the callback must fire again.
        const bool got_after = wait_for_event(mu, cv, events, k_discovery_timeout, [&](const auto &ev) {
            return std::any_of(ev.begin(), ev.end(), [&](const event &e) {
                return e.discovered && e.kind == provizio::dds::endpoint_kind::data_writer && e.topic == topic_name;
            });
        });
        if (!got_after)
        {
            std::cerr << "no post-reset discovery event -- listener was not re-installed?\n";
            return 1;
        }
        return 0;
    }

    int test_unregister()
    {
        // Unregister path: on_discovered_endpoint({}) clears the user callback so
        // it stops firing (the listener itself stays attached to drive the internal
        // match-publisher default); a later re-register restores the callback.
        // A third participant with a live callback (pc) is a positive control —
        // once IT observes the post-unregister publisher we know discovery has
        // propagated in the domain, so asserting the unregistered participant
        // did NOT observe it is meaningful rather than a race that merely
        // checked too early.
        // Declared before the participants so they outlive teardown-triggered
        // discovery callbacks (see test_discovers_writer for the rationale).
        std::mutex mu;
        std::condition_variable cv;
        std::vector<event> events_a;  // the participant under test
        std::vector<event> events_c;  // positive-control observer

        const auto pa = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);
        const auto pb = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);
        const auto pc = provizio::dds::make_domain_participant(0, provizio::dds::network_recovery_mode::off);

        pa->on_discovered_endpoint([&](provizio::dds::domain_participant &, const std::string &topic,
                                       const std::string &type, provizio::dds::endpoint_kind kind, bool discovered,
                                       eprosima::fastdds::dds::ReliabilityQosPolicyKind /*reliability*/,
                                       eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
            const std::lock_guard<std::mutex> lock{mu};
            events_a.push_back({topic, type, kind, discovered});
            cv.notify_all();
        });
        pc->on_discovered_endpoint([&](provizio::dds::domain_participant &, const std::string &topic,
                                       const std::string &type, provizio::dds::endpoint_kind kind, bool discovered,
                                       eprosima::fastdds::dds::ReliabilityQosPolicyKind /*reliability*/,
                                       eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
            const std::lock_guard<std::mutex> lock{mu};
            events_c.push_back({topic, type, kind, discovered});
            cv.notify_all();
        });

        // Baseline — pa observes the first publisher while registered.
        const std::string topic1 = "provizio_dds_discovery_test_topic_unreg1";
        const auto pub1 = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, topic1);
        const bool got1 = wait_for_event(mu, cv, events_a, k_discovery_timeout, [&](const auto &ev) {
            return std::any_of(ev.begin(), ev.end(), [&](const event &e) {
                return e.discovered && e.kind == provizio::dds::endpoint_kind::data_writer && e.topic == topic1;
            });
        });
        if (!got1)
        {
            std::cerr << "no pre-unregister discovery event\n";
            return 1;
        }

        // Unregister pa and drain both sinks.
        pa->on_discovered_endpoint({});
        {
            const std::lock_guard<std::mutex> lock{mu};
            events_a.clear();
            events_c.clear();
        }

        // A fresh publisher: the control participant must still see it; pa,
        // having unregistered, must not.
        const std::string topic2 = "provizio_dds_discovery_test_topic_unreg2";
        const auto pub2 = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, topic2);
        const bool control_saw = wait_for_event(mu, cv, events_c, k_discovery_timeout, [&](const auto &ev) {
            return std::any_of(ev.begin(), ev.end(), [&](const event &e) { return e.discovered && e.topic == topic2; });
        });
        if (!control_saw)
        {
            std::cerr << "control participant never saw the post-unregister publisher -- test inconclusive\n";
            return 1;
        }
        {
            const std::lock_guard<std::mutex> lock{mu};
            const auto pa_saw =
                std::any_of(events_a.begin(), events_a.end(), [&](const event &e) { return e.topic == topic2; });
            if (pa_saw)
            {
                std::cerr << "callback fired after on_discovered_endpoint({}) unregistered it\n";
                return 1;
            }
        }

        // Re-register: a subsequent publisher must reach the callback again.
        pa->on_discovered_endpoint([&](provizio::dds::domain_participant &, const std::string &topic,
                                       const std::string &type, provizio::dds::endpoint_kind kind, bool discovered,
                                       eprosima::fastdds::dds::ReliabilityQosPolicyKind /*reliability*/,
                                       eprosima::fastdds::dds::DurabilityQosPolicyKind /*durability*/) {
            const std::lock_guard<std::mutex> lock{mu};
            events_a.push_back({topic, type, kind, discovered});
            cv.notify_all();
        });
        const std::string topic3 = "provizio_dds_discovery_test_topic_unreg3";
        const auto pub3 = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(pb, topic3);
        const bool got3 = wait_for_event(mu, cv, events_a, k_discovery_timeout, [&](const auto &ev) {
            return std::any_of(ev.begin(), ev.end(), [&](const event &e) {
                return e.discovered && e.kind == provizio::dds::endpoint_kind::data_writer && e.topic == topic3;
            });
        });
        if (!got3)
        {
            std::cerr << "callback did not fire after re-registration\n";
            return 1;
        }
        return 0;
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
    const std::string subcommand{args[1]};
    if (subcommand == "type_registry")
    {
        return test_type_registry();
    }
    if (subcommand == "teardown_does_not_spin")
    {
        return test_teardown_does_not_spin();
    }
    if (subcommand == "reset_does_not_spin")
    {
        return test_reset_does_not_spin();
    }
    if (subcommand == "discovers_writer")
    {
        return test_discovers_writer();
    }
    if (subcommand == "kind_filter")
    {
        return test_kind_filter();
    }
    if (subcommand == "constructor_callback")
    {
        return test_constructor_callback();
    }
    if (subcommand == "visit_known_type")
    {
        return test_visit_known_type();
    }
    if (subcommand == "callback_receives_participant")
    {
        return test_callback_receives_participant();
    }
    if (subcommand == "survives_reset")
    {
        return test_survives_reset();
    }
    if (subcommand == "unregister")
    {
        return test_unregister();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
