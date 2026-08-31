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
// Subcommand-driven tests for the per-type KEEP_LAST history depths on a KEYLESS topic
// that several emitters share -- the shape of every Provizio fleet topic, where all the
// radars publish to one topic name and a sample is attributed to its sensor only by the
// frame_id in its header.
//
// KEEP_LAST depth is a PER INSTANCE budget and a keyless topic has exactly one instance,
// so on such a topic the depth is spent by the whole fleet rather than granted to each
// emitter. Running out of it is invisible: at the writer the sample is dropped from the
// history that would have retransmitted it, at the reader it is discarded after having
// been received and acknowledged. Either way no reliability guarantee is broken, nothing
// is counted and nothing is logged -- the callback simply never fires for it.
//
// Two things are established here, and it is worth being precise about which is which.
//
//   1. The scenario works end to end on the defaults: six emitters, one keyless topic,
//      one default subscriber, every sample labelled with its emitter and sequence
//      number, a consumer that is not ready to take anything while the burst is
//      published -- and afterwards every sample is there.
//   2. The per-type WRITER depth is what it claims to be, measured directly: a
//      late-joining consumer of transient-local emitters receives exactly what their
//      histories still hold, so the backlog it collects IS the depth. Held to a single
//      slot, the same emitters can only offer their last sample each.
//
// The second is a capacity measurement, not a race: it does not depend on how fast any
// machine ran, only on how many samples a history can hold. The only wall-clock waits
// are for discovery and delivery, and those are generous and scaled for an instrumented
// build.

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <fastdds/LibrarySettings.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/qos_defaults.h"
#include "provizio/dds/subscriber.h"

#include <nav_msgs/msg/OdometryPubSubTypes.hpp>

namespace
{
    using namespace std::chrono_literals;

    // Odometry is one of the fleet-shared types: small, published by every sensor to one
    // keyless topic, and specialized to the deep reader history. Its header carries a
    // frame_id, the real system's only per-emitter discriminator, so the test can label
    // samples exactly the way the field does.
    using odometry_pub_sub_type = nav_msgs::msg::OdometryPubSubType;
    using odometry = nav_msgs::msg::Odometry;

    // Emitters sharing the one topic. Six is a typical Provizio fleet, and enough for the
    // burst below to be several times the pre-split default of four history slots for
    // everyone together.
    constexpr int k_emitters = 6;

    // Samples per emitter, chosen against both defaults at once: 4 fits inside the per-type
    // writer depth of 8 with room to spare, and 6 x 4 = 24 fits inside the fleet-shared
    // reader depth of 32. Four times over the single slot the control case allows.
    constexpr int k_samples_per_emitter = 4;

    // The whole fleet's burst.
    constexpr std::size_t k_total_samples = static_cast<std::size_t>(k_emitters) * k_samples_per_emitter;

    // Deliberately shallow WRITER history for the control case: the single slot Fast-DDS
    // gives a writer by default, which is what the per-type writer depth exists to replace.
    constexpr std::int32_t k_shallow_writer_history_depth = 1;

    // Discovery and matching are asynchronous and this waits for something that must
    // happen, so it is scaled for an instrumented build (see PROVIZIO_DDS_TEST_TIMEOUT_SCALE
    // in test/CMakeLists.txt) and set well above what a loaded CI runner needs.
    constexpr auto k_match_timeout = std::chrono::seconds{30 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};

    // How long the match count must stay unchanged to count as settled. Every emitter has
    // to be matched before the burst starts, or a sample missing because its writer was not
    // yet matched would be indistinguishable from one lost to a history.
    constexpr auto k_match_settle_time = std::chrono::milliseconds{500 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};

    // Upper bound on the wait for the samples that a history did hold to reach the
    // callback. Only ever waited out in full when something is wrong -- the wait ends as
    // soon as the expected count arrives.
    constexpr auto k_delivery_timeout = std::chrono::seconds{20 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};

    // How long to keep listening after the samples a shallow history CAN hold have all
    // arrived, before concluding that the rest are not coming. A "must not happen within T"
    // window, so it is deliberately NOT scaled by PROVIZIO_DDS_TEST_TIMEOUT_SCALE: scaling
    // it would only make the test slower, never more correct, and the samples it is waiting
    // to not see were dropped at the emitter and no longer exist anywhere.
    constexpr auto k_no_more_arrivals_window = 3s;

    bool expect(bool condition, const char *expression, const char *file, int line)
    {
        if (!condition)
        {
            std::cerr << "FAIL " << file << ":" << line << ": " << expression << '\n';
        }
        return condition;
    }
    // Function-like macro: it captures the textual expression (#cond) plus the call-site
    // __FILE__/__LINE__, which a constexpr helper can't synthesise. Same shape as the
    // match_publisher_default / network_recovery tests' EXPECT.
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define EXPECT(cond) expect((cond), #cond, __FILE__, __LINE__)

    /**
     * @brief Turns off Fast-DDS's intraprocess delivery for this process, so the emitters
     * and the consumer talk over a transport the way separate machines do.
     *
     * Fast-DDS delivers between endpoints of the same process by handing the sample
     * straight from the writing thread into the reader -- history, listener callback and
     * all -- inside the call to publish(). That shortcut makes a single-process test of
     * either endpoint's history meaningless, because the consumer can never fall behind
     * its emitters: the emitters ARE the consumer's thread. It also deadlocks a test that
     * deliberately holds the callback, since the thread being held is the publisher's.
     *
     * With it off, samples take the normal transport path and the reader's own reception
     * threads deliver them, which is the arrangement this suite is about: a fleet of
     * sensors, each in its own process on its own machine, feeding one consumer.
     *
     * Must run before any participant is created, and applies process-wide -- which is
     * exactly why each case here is its own process (one ctest entry per subcommand).
     *
     * @return True if Fast-DDS accepted the setting.
     */
    bool disable_intraprocess_delivery()
    {
        auto *factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
        eprosima::fastdds::LibrarySettings settings;
        factory->get_library_settings(settings);
        settings.intraprocess_delivery = eprosima::fastdds::INTRAPROCESS_OFF;
        return factory->set_library_settings(settings) == eprosima::fastdds::dds::RETCODE_OK;
    }

    // Highest DDS domain id this test will pick. Domains are chosen at random within
    // 1..this, which stays inside the DDS-safe span and off domain 0.
    constexpr int k_highest_test_domain = 200;

    /**
     * @brief A DDS domain chosen at random, once per process.
     *
     * CI runners are loaded and run many jobs at once, and every participant on a domain
     * hears every other one on it. A per-process domain gives each test run its own
     * discovery space, so a neighbouring job's traffic can neither be counted as one of
     * this test's samples nor crowd out its discovery. Range 1..200 stays inside the
     * DDS-safe span and off domain 0, where resident software lives.
     *
     * @return The domain id for this process.
     */
    provizio::dds::DomainId_t random_domain()
    {
        std::random_device entropy;
        std::uniform_int_distribution<int> domains{1, k_highest_test_domain};
        return static_cast<provizio::dds::DomainId_t>(domains(entropy));
    }

    /**
     * @brief The label an emitter puts in a sample's frame_id, exactly as the fleet does:
     * the sample's only clue about which sensor produced it, and here also which sample of
     * that sensor's sequence it is.
     *
     * @param emitter Zero-based emitter index.
     * @param sequence Zero-based sequence number within that emitter's stream.
     * @return The frame_id to publish, e.g. "emitter_3/2".
     */
    std::string sample_label(int emitter, int sequence)
    {
        return "emitter_" + std::to_string(emitter) + "/" + std::to_string(sequence);
    }

    /**
     * @brief Collects the labels a subscriber's callback receives, optionally holding that
     * callback on a gate so a burst can be published while the consumer is provably not
     * taking anything.
     *
     * provizio_dds invokes the user callback inside the DataReader's take loop, so a
     * callback that does not return stops the reader from taking: everything published
     * while the gate is closed has to be held by a history somewhere, or be dropped.
     */
    class label_collector final
    {
      public:
        /**
         * @brief Constructs the collector.
         * @param start_open True to let callbacks run immediately; false to hold them until
         * open_gate().
         */
        explicit label_collector(bool start_open) : open{start_open}
        {
        }

        /**
         * @brief Receives one sample: waits for the gate if it is closed, then records the
         * sample's label.
         * @param data The received sample.
         */
        void operator()(const odometry &data)
        {
            {
                std::unique_lock<std::mutex> lock{mutex};
                gate.wait(lock, [this] { return open; });
            }

            const std::lock_guard<std::mutex> lock{mutex};
            labels.insert(data.header().frame_id());
            delivered.notify_all();
        }

        /**
         * @brief Lets the held callback (and every later one) run.
         */
        void open_gate()
        {
            {
                const std::lock_guard<std::mutex> lock{mutex};
                open = true;
            }
            gate.notify_all();
        }

        /**
         * @brief Waits until at least @p expected distinct labels have been recorded, or the
         * timeout expires.
         *
         * @param expected How many distinct labels to wait for.
         * @param timeout How long to wait at most.
         * @return The number of distinct labels recorded when the wait ended.
         */
        std::size_t wait_for_labels(std::size_t expected, std::chrono::nanoseconds timeout)
        {
            std::unique_lock<std::mutex> lock{mutex};
            delivered.wait_for(lock, timeout, [this, expected] { return labels.size() >= expected; });
            return labels.size();
        }

        /**
         * @brief The distinct labels received so far.
         * @return A copy of the recorded label set.
         */
        std::set<std::string> received_labels()
        {
            const std::lock_guard<std::mutex> lock{mutex};
            return labels;
        }

      private:
        std::mutex mutex;
        std::condition_variable gate;
        std::condition_variable delivered;
        bool open;
        std::set<std::string> labels;
    };

    /**
     * @brief One emitter of the fleet: its own participant and its publisher on the shared
     * topic. Each emitter gets its own participant so every match is a real remote SEDP
     * match -- the way a fleet of separate sensors reaches one consumer -- rather than an
     * intra-participant shortcut.
     */
    struct emitter
    {
        std::shared_ptr<provizio::dds::domain_participant> participant;
        std::shared_ptr<provizio::dds::publisher_handle<odometry_pub_sub_type>> publisher;
    };

    /**
     * @brief Creates the fleet of emitters on one shared keyless topic.
     *
     * @param domain The DDS domain to publish on.
     * @param topic_name The single keyless topic every emitter shares.
     * @param writer_history_depth Each emitter's history_depth. use_default_history_depth
     * takes the per-type default this suite exists to pin.
     * @param durability_kind Durability for the emitters, or std::nullopt for the default
     * (volatile).
     * @return The emitters, which the caller must keep alive for as long as it expects
     * their samples: a writer that goes away takes its history -- and its match -- with it.
     */
    std::vector<emitter> create_the_fleet(provizio::dds::DomainId_t domain, const std::string &topic_name,
                                          const std::int32_t writer_history_depth,
                                          std::optional<provizio::dds::DurabilityQosPolicyKind> durability_kind)
    {
        std::vector<emitter> fleet;
        fleet.reserve(k_emitters);
        for (int index = 0; index < k_emitters; ++index)
        {
            auto participant =
                provizio::dds::make_domain_participant(domain, provizio::dds::network_recovery_mode::off);
            auto publisher = provizio::dds::make_publisher<odometry_pub_sub_type>(
                participant, topic_name,
                provizio::dds::qos_defaults<odometry_pub_sub_type>::datawriter_reliability_kind, writer_history_depth,
                durability_kind);
            fleet.push_back(emitter{std::move(participant), std::move(publisher)});
        }
        return fleet;
    }

    /**
     * @brief Publishes k_samples_per_emitter labelled samples from every emitter, round
     * robin, as fast as they can be written.
     *
     * @param fleet The emitters to publish from.
     * @return True if every write was accepted. NOT asserted on by the shallow-history
     * case: a reliable writer whose history is full may refuse the write outright instead
     * of dropping its oldest sample, and either outcome is the loss being demonstrated.
     */
    bool publish_the_fleet_burst(const std::vector<emitter> &fleet)
    {
        bool accepted = true;
        for (int sequence = 0; sequence < k_samples_per_emitter; ++sequence)
        {
            for (int index = 0; index < k_emitters; ++index)
            {
                odometry sample;
                sample.header().frame_id(sample_label(index, sequence));
                accepted &= fleet[static_cast<std::size_t>(index)].publisher->publish(sample);
            }
        }
        return accepted;
    }

    /**
     * @brief Reports which of the fleet's labels are missing from @p received.
     *
     * @param received The labels that reached the callback.
     * @return True if every emitter's every sample is present.
     */
    bool every_sample_arrived(const std::set<std::string> &received)
    {
        bool complete = true;
        for (int index = 0; index < k_emitters; ++index)
        {
            for (int sequence = 0; sequence < k_samples_per_emitter; ++sequence)
            {
                const std::string label = sample_label(index, sequence);
                if (received.count(label) == 0)
                {
                    std::cerr << "  missing sample " << label << '\n';
                    complete = false;
                }
            }
        }
        return complete;
    }

    /**
     * @brief Every sample from every emitter on a shared keyless topic reaches the callback
     * when both endpoints are left on the library's defaults, even though the consumer is
     * not taking anything while the burst is published.
     *
     * The failing scenario in miniature, end to end: one keyless topic, six emitters told
     * apart only by frame_id, a default (match-publisher) subscriber, and a consumer that
     * is busy for the whole burst. Nothing may be missing afterwards.
     */
    int test_every_emitters_samples_survive_the_defaults()
    {
        bool passed = EXPECT(disable_intraprocess_delivery());

        const provizio::dds::DomainId_t domain = random_domain();
        const std::string topic_name = "rt/keyless_topic_history_test";

        auto participant = provizio::dds::make_domain_participant(domain, provizio::dds::network_recovery_mode::off);
        label_collector collector{false};
        auto subscriber = provizio::dds::make_subscriber<odometry_pub_sub_type>(
            participant, topic_name, [&collector](const odometry &data) { collector(data); });

        // Declared after the subscriber so the emitters are still alive while the gate is
        // opened and the surviving samples are collected below.
        const std::vector<emitter> fleet =
            create_the_fleet(domain, topic_name, provizio::dds::use_default_history_depth, std::nullopt);

        // Progress is reported as it happens: when this test fails on CI it fails inside a
        // wait, and knowing which wait it was is the difference between a diagnosis and a
        // guess.
        std::cerr << "  created " << k_emitters << " emitters, waiting for them to match\n";
        // Every emitter must be matched before the burst starts. A sample missing because
        // its writer was not yet matched is not a history loss, and this test must not be
        // able to confuse the two.
        //
        // Asked repeatedly until the EXPECTED count is reached, rather than once.
        // get_num_matched_publishers returns as soon as the count has been stable for the
        // settle window, which is a heuristic for "they have all arrived" and not a promise:
        // on a loaded runner one straggler's discovery outlasts that window, the count
        // settles at five of six, and the call returns having spent a fraction of its
        // budget. Observed exactly so on a jetson Debug runner -- it returned 5 after 106 s
        // of a 150 s allowance and the case then reported 20 of 24 samples, which reads as
        // history loss and is nothing of the kind. Re-asking until the deadline is what
        // makes the timeout mean what it says.
        const auto match_deadline = std::chrono::steady_clock::now() + k_match_timeout;
        int matched = 0;
        for (;;)
        {
            const auto remaining = match_deadline - std::chrono::steady_clock::now();
            if (remaining <= std::chrono::steady_clock::duration::zero())
            {
                break;
            }
            matched = subscriber->get_num_matched_publishers(
                std::chrono::duration_cast<std::chrono::milliseconds>(remaining),
                std::chrono::duration_cast<std::chrono::milliseconds>(k_match_settle_time));
            if (matched == k_emitters)
            {
                break;
            }
        }
        passed &= EXPECT(matched == k_emitters);

        // Bailing out here rather than carrying on: with an emitter unmatched, every
        // assertion below is about samples that were never written, so continuing would
        // bury the one fact that explains the run under two failures that do not.
        if (matched != k_emitters)
        {
            std::cout << "every_emitters_samples_survive_the_defaults: FAIL (only " << matched << " of " << k_emitters
                      << " emitters matched; no burst was published)\n";
            return 1;
        }

        std::cerr << "  matched " << matched << " emitters, publishing the burst\n";
        passed &= EXPECT(publish_the_fleet_burst(fleet));

        // Release the callback only after the whole burst has been written.
        std::cerr << "  burst published, opening the gate\n";
        collector.open_gate();
        collector.wait_for_labels(k_total_samples, k_delivery_timeout);
        const std::set<std::string> received = collector.received_labels();

        passed &= EXPECT(every_sample_arrived(received));
        passed &= EXPECT(received.size() == k_total_samples);

        std::cout << "every_emitters_samples_survive_the_defaults: " << (passed ? "PASS" : "FAIL") << " ("
                  << received.size() << " of " << k_total_samples << " samples)\n";
        return passed ? 0 : 1;
    }

    /**
     * @brief Measures what an emitter's history actually holds, by letting a consumer join
     * after the burst is over and collecting the backlog the emitters can still offer it.
     *
     * A transient-local writer hands a late-joining reader whatever its history still
     * contains -- no more, because the rest has been dropped to make room, and no less,
     * because the samples are reliable. So the size of the backlog IS the depth, measured
     * rather than asserted about, with no dependence on how fast anything ran.
     *
     * @param writer_history_depth Each emitter's history_depth.
     * @param expected_labels How many labels to wait for before the settling window.
     * @param[out] received The labels that reached the callback.
     * @return True if the fleet was set up and matched.
     */
    bool measure_the_backlog_a_late_joiner_gets(const std::int32_t writer_history_depth,
                                                const std::size_t expected_labels, std::set<std::string> &received)
    {
        bool passed = EXPECT(disable_intraprocess_delivery());

        const provizio::dds::DomainId_t domain = random_domain();
        const std::string topic_name = "rt/keyless_topic_history_late_join_test";

        const std::vector<emitter> fleet =
            create_the_fleet(domain, topic_name, writer_history_depth, provizio::dds::TRANSIENT_LOCAL_DURABILITY_QOS);
        std::cerr << "  created " << k_emitters << " emitters, publishing the burst with no consumer present\n";
        // Asserted only where every write CAN be accepted. A RELIABLE writer whose history has
        // no free slot may refuse the write outright rather than evict its oldest sample, and
        // either outcome is the loss the shallow case exists to demonstrate -- so asserting it
        // there would fail the case for observing exactly what it was written to observe. That
        // is what publish_the_fleet_burst's @return documents, and this is the caller it was
        // documented for.
        //
        // The condition is "explicitly shallower than one emitter's share of the burst", not
        // "not the shallow case": a non-positive depth is the use-default sentinel, which
        // resolves to a per-type default deep enough to take the whole burst, so it is asserted
        // rather than skipped along with it.
        const bool every_write_accepted = publish_the_fleet_burst(fleet);
        if (writer_history_depth <= 0 || writer_history_depth >= k_samples_per_emitter)
        {
            passed &= EXPECT(every_write_accepted);
        }

        // Only now does the consumer appear. Everything it receives from here on came out
        // of an emitter's history, because there is nothing live left to send.
        auto participant = provizio::dds::make_domain_participant(domain, provizio::dds::network_recovery_mode::off);
        label_collector collector{true};
        auto subscriber = provizio::dds::make_subscriber<odometry_pub_sub_type>(
            participant, topic_name, [&collector](const odometry &data) { collector(data); },
            provizio::dds::qos_defaults<odometry_pub_sub_type>::datareader_reliability_kind,
            provizio::dds::use_default_history_depth, provizio::dds::TRANSIENT_LOCAL_DURABILITY_QOS);

        std::cerr << "  consumer joined, waiting for the backlog\n";
        collector.wait_for_labels(expected_labels, k_delivery_timeout);
        // Then keep listening for a while, so a case expecting FEWER samples than were
        // published cannot pass merely by looking too early.
        std::this_thread::sleep_for(k_no_more_arrivals_window);
        received = collector.received_labels();
        return passed;
    }

    /**
     * @brief On the per-type writer default, an emitter's history still holds every sample
     * of the burst, so a consumer that joins afterwards gets all of them.
     *
     * This is the retransmission head-room the writer depth buys, measured directly. Four
     * samples per emitter against a default depth of eight: nothing has been dropped, so a
     * reader that asks for the backlog gets the whole fleet's burst.
     */
    int test_the_default_writer_history_holds_the_whole_burst()
    {
        bool passed = true;

        std::set<std::string> received;
        passed &= EXPECT(measure_the_backlog_a_late_joiner_gets(provizio::dds::use_default_history_depth,
                                                                k_total_samples, received));
        passed &= EXPECT(every_sample_arrived(received));
        passed &= EXPECT(received.size() == k_total_samples);

        std::cout << "the_default_writer_history_holds_the_whole_burst: " << (passed ? "PASS" : "FAIL") << " ("
                  << received.size() << " of " << k_total_samples << " samples)\n";
        return passed ? 0 : 1;
    }

    /**
     * @brief Held to the single history slot Fast-DDS gives a writer by default, the same
     * emitters can offer only their last sample each -- the rest are gone, silently.
     *
     * The control for the case above, and the reason the writer default stopped being
     * "whatever Fast-DDS does". Every write here succeeded and every sample was reliable;
     * three quarters of them simply no longer exist, with nothing counted and nothing
     * logged. The claim is about capacity, not speed: one slot cannot hold four samples
     * however fast or slow the machine is.
     */
    int test_a_one_slot_writer_history_drops_all_but_the_newest()
    {
        bool passed = true;

        std::set<std::string> received;
        // One slot per emitter, so one label per emitter is all there can be to wait for.
        passed &= EXPECT(measure_the_backlog_a_late_joiner_gets(k_shallow_writer_history_depth,
                                                                static_cast<std::size_t>(k_emitters), received));
        passed &= EXPECT(received.size() == static_cast<std::size_t>(k_emitters));
        passed &= EXPECT(received.size() < k_total_samples);
        // And what survived is each emitter's NEWEST sample, which is what KEEP_LAST means.
        for (int index = 0; index < k_emitters; ++index)
        {
            passed &= EXPECT(received.count(sample_label(index, k_samples_per_emitter - 1)) == 1);
        }

        std::cout << "a_one_slot_writer_history_drops_all_but_the_newest: " << (passed ? "PASS" : "FAIL") << " ("
                  << received.size() << " of " << k_total_samples << " samples survived)\n";
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

    if (subcommand == "every_emitters_samples_survive_the_defaults")
    {
        return test_every_emitters_samples_survive_the_defaults();
    }
    if (subcommand == "the_default_writer_history_holds_the_whole_burst")
    {
        return test_the_default_writer_history_holds_the_whole_burst();
    }
    if (subcommand == "a_one_slot_writer_history_drops_all_but_the_newest")
    {
        return test_a_one_slot_writer_history_drops_all_but_the_newest();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
