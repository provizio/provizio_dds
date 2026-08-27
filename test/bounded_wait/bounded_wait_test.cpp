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

// The deadline arithmetic behind every caller-supplied timeout in the public API. What it
// guards is not hypothetical: a "wait as good as forever" value overflows a plain
// now() + timeout into the PAST, and a steady_clock deadline at time_point::max() makes
// libstdc++'s condition_variable spin at 100% of a core rather than wait -- which is how a
// participant destruction was found burning a CPU for tens of seconds on an aarch64 runner.

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>
#include <vector>

#include "detail/test_cpu_time.h"
#include "provizio/dds/detail/bounded_wait.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/request_response.h"
#include "provizio/dds/subscriber.h"

#include <std_msgs/msg/Int32PubSubTypes.hpp>
#include <std_msgs/msg/Int64PubSubTypes.hpp>
#include <std_msgs/msg/StringPubSubTypes.hpp>

namespace
{
    bool expect(bool condition, const char *expression, const char *file, int line)
    {
        if (!condition)
        {
            std::cerr << "FAIL " << file << ":" << line << ": " << expression << '\n';
        }
        return condition;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define EXPECT(cond) expect((cond), #cond, __FILE__, __LINE__)

    // Case: a timeout so large that adding it to now() overflows must still produce a
    // deadline in the FUTURE. A plain addition yields one in the past, so the caller who
    // asked to wait essentially forever is told at once that the wait expired.
    int test_saturating_deadline_does_not_overflow()
    {
        using namespace provizio::dds::detail;

        const auto before = std::chrono::steady_clock::now();
        const auto saturated = saturating_deadline<std::chrono::steady_clock>(std::chrono::milliseconds::max());
        bool passed = EXPECT(saturated > before);
        // Saturated, not merely large: max() is what "no deadline" has to mean.
        passed &= EXPECT(saturated == std::chrono::steady_clock::time_point::max());

        // The overflowing addition this replaces is NOT performed here to demonstrate it:
        // signed overflow is undefined behaviour, and a sanitizer build is right to abort on
        // it. The assertions above are the guard -- against the plain addition they fail,
        // because it yields a deadline in the past rather than one at max().

        // An ordinary timeout is left exactly as it was.
        const auto ordinary = saturating_deadline<std::chrono::steady_clock>(std::chrono::seconds{10});
        passed &= EXPECT(ordinary > before + std::chrono::seconds{9});
        passed &= EXPECT(ordinary < before + std::chrono::seconds{11});

        // The NEGATIVE end, which the upper guard alone does not cover: a large-magnitude
        // negative duration overflows the cast and wraps to the far future, so a timeout meaning
        // "already expired" becomes a wait of roughly 290 years. Measured before the fix as ~9.15e9
        // seconds ahead; the property asserted here is simply that it is not in the future at all.
        //
        // Reachable from the public API -- future_response::wait_for takes any duration, and the
        // matched-count timeouts are unvalidated signed milliseconds -- so this is a caller's
        // mistake turning into a hang rather than an immediate return.
        const auto after_negative = std::chrono::steady_clock::now();
        const auto huge_negative =
            saturating_deadline<std::chrono::steady_clock>(std::chrono::milliseconds{-9300000000000LL});
        passed &= EXPECT(huge_negative <= after_negative + std::chrono::seconds{1});

        // The same at the extreme of a coarser unit, where the multiplication into the clock's
        // ticks is what overflows.
        const auto after_min = std::chrono::steady_clock::now();
        passed &= EXPECT(saturating_deadline<std::chrono::steady_clock>(std::chrono::hours::min()) <=
                         after_min + std::chrono::seconds{1});

        // An ordinary negative stays expired too -- the clamp must not turn a small negative into
        // a wait either.
        const auto after_small = std::chrono::steady_clock::now();
        passed &= EXPECT(saturating_deadline<std::chrono::steady_clock>(std::chrono::milliseconds{-5}) <=
                         after_small + std::chrono::seconds{1});

        std::cout << "saturating_deadline_does_not_overflow: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: reserving a settle time out of a saturated deadline does not undo the saturation.
    //
    // The hazard sits one step past saturating_deadline and is easy to reintroduce, because
    // the natural spelling looks harmless: `deadline - settle_time` converts the settle time
    // into the clock's own ticks before subtracting, which for a near-max value expressed in
    // coarser units is a multiplication that overflows. The deadline then lands arbitrarily
    // far in the PAST, the wait expires at once, and a caller who asked to wait as long as it
    // takes is told immediately that nothing matched -- the exact behaviour the saturation
    // above it exists to prevent, undone on the next line.
    int test_deadline_reserving_does_not_overflow()
    {
        using namespace provizio::dds::detail;

        const auto floor = std::chrono::steady_clock::now() + std::chrono::milliseconds{50};

        // A settle time too large to express in the clock's ticks consumes the whole budget,
        // so the floor is the answer -- never something in the past.
        const auto saturated = std::chrono::steady_clock::time_point::max();
        const auto starved =
            deadline_reserving<std::chrono::steady_clock>(saturated, std::chrono::milliseconds::max(), floor);
        bool passed = EXPECT(starved == floor);
        // Stated separately, and against the clock rather than against the floor, because
        // "not in the past" is the property that actually matters. Comparing with the floor
        // cannot express it: were the floor itself ever computed as a past instant, a
        // starved >= floor would pass exactly as the equality above does, so it would restate
        // the first assertion instead of guarding it. Reading the clock keeps holding whatever
        // the floor becomes.
        passed &= EXPECT(starved > std::chrono::steady_clock::now());

        // An ordinary reserve is subtracted exactly as the plain expression would.
        const auto ordinary_deadline = std::chrono::steady_clock::now() + std::chrono::seconds{10};
        const auto reserved =
            deadline_reserving<std::chrono::steady_clock>(ordinary_deadline, std::chrono::seconds{2}, floor);
        // Exactly, not within a tolerance: deadline_reserving reads no clock, so nothing
        // between the two operands can drift and a window would only hide an arithmetic
        // error the size of its own slack.
        passed &= EXPECT(reserved == ordinary_deadline - std::chrono::seconds{2});

        // A deadline already at or behind the floor yields the floor, which is what the
        // std::max this helper replaced did.
        const auto behind = deadline_reserving<std::chrono::steady_clock>(
            std::chrono::steady_clock::now() - std::chrono::seconds{1}, std::chrono::milliseconds{10}, floor);
        passed &= EXPECT(behind == floor);

        // A NEGATIVE reserve reaches the same overflow from the other side, and it takes only
        // one nonsense operand to get there: the near-max timeout is the supported way to say
        // "wait as long as it takes", while settle_time is a public signed milliseconds with
        // no precondition and no validation. Unclamped, the comparison passes a negative
        // reserve through (it is smaller than any positive difference) and the subtraction
        // becomes deadline + |reserve| on a deadline saturating_deadline has just put at
        // time_point::max() -- signed overflow, which under the sanitizers Debug builds enable
        // by default aborts the caller outright.
        //
        // Reserving a negative amount can only mean reserving nothing, so the deadline is
        // expected back unchanged, which is also what the Python mirror's
        // max(0.0, settle_time_sec) produces.
        const auto negative_reserve =
            deadline_reserving<std::chrono::steady_clock>(saturated, std::chrono::milliseconds{-1000}, floor);
        passed &= EXPECT(negative_reserve == saturated);

        // The same clamp on an ordinary deadline, where "unchanged" is distinguishable from
        // "saturated": a negative reserve must not push the deadline OUT either.
        passed &= EXPECT(deadline_reserving<std::chrono::steady_clock>(
                             ordinary_deadline, std::chrono::milliseconds{-1000}, floor) == ordinary_deadline);

        std::cout << "deadline_reserving_does_not_overflow: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: waiting with no deadline at all -- steady_clock::time_point::max(), the value a
    // caller passes to mean "however long it takes" -- blocks instead of spinning, and still
    // returns as soon as the predicate holds.
    int test_unbounded_deadline_waits_without_spinning()
    {
        using namespace provizio::dds::detail;

        std::mutex mutex;
        std::condition_variable condition;
        bool ready = false;

        constexpr auto hold = std::chrono::milliseconds{400 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};
        std::thread setter{[&] {
            std::this_thread::sleep_for(hold);
            {
                const std::lock_guard<std::mutex> lock{mutex};
                ready = true;
            }
            condition.notify_all();
        }};

        const auto cpu_before = provizio::dds::test::process_cpu_seconds();
        const auto started = std::chrono::steady_clock::now();
        bool satisfied = false;
        {
            std::unique_lock<std::mutex> lock{mutex};
            satisfied = wait_until_bounded(condition, lock, std::chrono::steady_clock::time_point::max(),
                                           [&ready] { return ready; });
        }
        const auto waited = std::chrono::steady_clock::now() - started;
        const auto cpu_used = provizio::dds::test::process_cpu_seconds() - cpu_before;
        setter.join();

        bool passed = EXPECT(satisfied);
        // It really did wait for the predicate rather than returning early.
        passed &= EXPECT(waited > hold / 2);
        // And it waited rather than spun. A spin consumes the wall time it covers; a
        // blocking wait consumes almost nothing. Half the hold is far above the former and
        // far below the latter, so the threshold needs no tuning.
        const auto budget = std::chrono::duration_cast<std::chrono::duration<double>>(hold).count() / 2.0;
        passed &= EXPECT(cpu_used < budget);

        std::cout << "unbounded_deadline_waits_without_spinning: " << (passed ? "PASS" : "FAIL") << " (waited "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(waited).count() << " ms, burned "
                  << (cpu_used * 1000.0) << " ms of CPU)" << '\n';
        return passed ? 0 : 1;
    }

    // Case: the same "no deadline" value, but on a time_point whose duration is COARSER than
    // the clock's own -- a different hazard, not the same one twice.
    //
    // future_response::wait_until is a public API templated on both the clock AND its
    // duration, so a caller may hand it time_point<steady_clock, seconds>::max(). Weighing
    // that against steady_clock::now() converts it to nanoseconds, and the multiplication
    // overflows: measured, the comparison "now >= deadline" then answers TRUE for a deadline
    // 292 years out, so an unbounded wait returns a timeout immediately. The predicate here
    // is satisfied only after a delay, so a wait that gave up early returns false.
    int test_coarse_deadline_is_not_already_passed()
    {
        using namespace provizio::dds::detail;
        using coarse_time_point = std::chrono::time_point<std::chrono::steady_clock, std::chrono::seconds>;

        std::mutex mutex;
        std::condition_variable condition;
        bool ready = false;

        constexpr auto hold = std::chrono::milliseconds{200 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};
        std::thread setter{[&] {
            std::this_thread::sleep_for(hold);
            {
                const std::lock_guard<std::mutex> lock{mutex};
                ready = true;
            }
            condition.notify_all();
        }};

        const auto started = std::chrono::steady_clock::now();
        bool satisfied = false;
        {
            std::unique_lock<std::mutex> lock{mutex};
            satisfied = wait_until_bounded(condition, lock, coarse_time_point::max(), [&ready] { return ready; });
        }
        const auto waited = std::chrono::steady_clock::now() - started;
        setter.join();

        bool passed = EXPECT(satisfied);
        // Waited for the predicate rather than mistaking a far-future deadline for one
        // already behind it.
        passed &= EXPECT(waited > hold / 2);

        std::cout << "coarse_deadline_is_not_already_passed: " << (passed ? "PASS" : "FAIL") << " (waited "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(waited).count() << " ms)" << '\n';
        return passed ? 0 : 1;
    }

    // Case: a deadline that passes with the predicate still false reports a timeout, and
    // does so at the deadline rather than at the end of a slice.
    int test_deadline_expires()
    {
        using namespace provizio::dds::detail;

        std::mutex mutex;
        std::condition_variable condition;

        constexpr auto deadline_in = std::chrono::milliseconds{200 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};
        const auto started = std::chrono::steady_clock::now();
        bool satisfied = true;
        {
            std::unique_lock<std::mutex> lock{mutex};
            satisfied = wait_until_bounded(condition, lock, started + deadline_in, [] { return false; });
        }
        const auto waited = std::chrono::steady_clock::now() - started;

        bool passed = EXPECT(!satisfied);
        passed &= EXPECT(waited >= deadline_in);
        // Not rounded up to a whole slice: max_wait_slice is an hour, so a case that waited
        // one would never finish.
        passed &= EXPECT(waited < deadline_in * 10);

        std::cout << "deadline_expires: " << (passed ? "PASS" : "FAIL") << " (waited "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(waited).count() << " ms)" << '\n';
        return passed ? 0 : 1;
    }

    // The cases above test the helpers in isolation. These four test the WIRING -- the
    // public entry points that now route a caller's timeout through them. A mistake made
    // only at a call site (the wrong clock handed to saturating_deadline, a predicate
    // miswired into wait_until_bounded) is invisible to a helper test, and every one of
    // these entry points takes its timeout from the caller with no default, so the value a
    // caller reaches for to mean "however long it takes" is exactly the dangerous one.

    // Not the default domain: Provizio's self-hosted runners share a LAN where domain 0
    // carries every other suite's participants.
    constexpr int k_domain = 46;

    std::shared_ptr<provizio::dds::domain_participant> make_participant()
    {
        return provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
    }

    // Case: the matched-count APIs accept a timeout so large it cannot be added to now().
    // A settle time is essential here: with none, the overflowed deadline is never read, so
    // the case would pass against the plain addition and prove nothing. With one, the settle
    // loop runs "while (now < timeout_point)" against a deadline the overflow put in the
    // PAST, gives up on the first pass, and answers -1 ("never settled") instead of a count.
    int test_matched_counts_accept_an_unbounded_timeout()
    {
        const std::string topic_name{"provizio_dds_bounded_wait_matched_topic"};
        const auto participant = make_participant();
        const auto publisher = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(participant, topic_name);
        const auto subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
            participant, topic_name, [](const std_msgs::msg::String &) {});

        constexpr auto settle = std::chrono::milliseconds{100 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};
        const int subscribers = publisher->get_num_matched_subscribers(std::chrono::milliseconds::max(), settle);
        const int publishers = subscriber->get_num_matched_publishers(std::chrono::milliseconds::max(), settle);

        // Counted, not -1: -1 is what the overflowed deadline produces, and 0 would mean the
        // two never matched at all, which would make the case vacuous.
        bool passed = EXPECT(subscribers > 0);
        passed &= EXPECT(publishers > 0);

        std::cout << "matched_counts_accept_an_unbounded_timeout: " << (passed ? "PASS" : "FAIL") << " (subscribers "
                  << subscribers << ", publishers " << publishers << ")" << '\n';
        return passed ? 0 : 1;
    }

    // Case: future_response's two waits accept the values a caller uses to mean "no
    // deadline" -- a near-max duration through wait_for, and time_point::max() through
    // wait_until -- and return the response rather than an instant timeout.
    int test_future_response_accepts_an_unbounded_timeout()
    {
        const std::string service_name{"provizio_dds_bounded_wait_service"};
        const auto service_participant = make_participant();
        const auto client_participant = make_participant();
        const auto service =
            provizio::dds::make_service<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
                service_participant, service_name, [](const std_msgs::msg::Int32 &request) {
                    std_msgs::msg::Int64 response;
                    response.data(request.data() + 1);
                    return response;
                });

        std_msgs::msg::Int32 request_data;
        request_data.data(41);

        bool passed = true;
        {
            auto future = provizio::dds::request<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
                client_participant, service_name, request_data);
            // The overflowing addition this replaces lands in the past, so wait_for would
            // report a timeout at once instead of waiting for the response.
            passed &= EXPECT(future.wait_for(std::chrono::milliseconds::max()) == std::future_status::ready);
        }
        {
            auto future = provizio::dds::request<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int64PubSubType>(
                client_participant, service_name, request_data);
            // And the deadline form: steady_clock::time_point::max() is the value that makes
            // an unguarded condition_variable spin rather than wait.
            passed &=
                EXPECT(future.wait_until(std::chrono::steady_clock::time_point::max()) == std::future_status::ready);
            passed &= EXPECT(future.get().data() == 42);
        }

        std::cout << "future_response_accepts_an_unbounded_timeout: " << (passed ? "PASS" : "FAIL") << '\n';
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

    if (subcommand == "saturating_deadline_does_not_overflow")
    {
        return test_saturating_deadline_does_not_overflow();
    }
    if (subcommand == "deadline_reserving_does_not_overflow")
    {
        return test_deadline_reserving_does_not_overflow();
    }
    if (subcommand == "unbounded_deadline_waits_without_spinning")
    {
        return test_unbounded_deadline_waits_without_spinning();
    }
    if (subcommand == "coarse_deadline_is_not_already_passed")
    {
        return test_coarse_deadline_is_not_already_passed();
    }
    if (subcommand == "matched_counts_accept_an_unbounded_timeout")
    {
        return test_matched_counts_accept_an_unbounded_timeout();
    }
    if (subcommand == "future_response_accepts_an_unbounded_timeout")
    {
        return test_future_response_accepts_an_unbounded_timeout();
    }
    if (subcommand == "deadline_expires")
    {
        return test_deadline_expires();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
