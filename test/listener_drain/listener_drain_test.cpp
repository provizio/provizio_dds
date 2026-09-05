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

// The detach + drain primitive every provizio-owned Fast-DDS listener holds, tested
// directly rather than through a participant reset. What it promises -- that
// detach_and_drain returns only once every in-flight callback has returned, and that a
// callback which never returns says so in the log until it does -- is reachable through a
// reset only by wedging a real subscriber callback for longer than the default reporting
// slice, which would trade a deterministic assertion for a multi-second timing race.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "provizio/dds/detail/listener_drain.h"
#include "provizio/dds/logging.h"

namespace
{
    using namespace std::chrono_literals;

    // Short enough to keep every case under a second, long enough that a loaded runner
    // cannot mistake one slice for two. Scaled with the environment for the same reason
    // the completion deadlines of the DDS tests are: a sanitizer build wakes threads
    // several times slower, and a slice that no longer outlasts the scheduling jitter
    // would report stalls that are not there.
    constexpr auto stall_slice = 50ms * PROVIZIO_DDS_TEST_TIMEOUT_SCALE;

    // How long a wedged callback is held. Many slices, so "the warning repeats" is not a
    // question of catching a single boundary.
    constexpr auto wedged_callback_duration = stall_slice * 8;

    bool expect(bool condition, const char *expression, const char *file, int line)
    {
        if (!condition)
        {
            std::cerr << "FAIL " << file << ":" << line << ": " << expression << '\n';
        }
        return condition;
    }
    // A function-like macro is the right shape here: it has to capture the textual
    // expression and the call-site __FILE__/__LINE__, which no constexpr template can
    // synthesise without the caller passing them explicitly.
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define EXPECT(cond) expect((cond), #cond, __FILE__, __LINE__)

    class log_capture
    {
      public:
        log_capture()
        {
            previous = provizio::dds::set_log_callback([this](provizio::dds::log_level, std::string_view message) {
                const std::lock_guard<std::mutex> lock{mutex};
                messages.emplace_back(message);
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

        std::size_t count_containing(const std::string_view needle) const
        {
            const std::lock_guard<std::mutex> lock{mutex};
            std::size_t found = 0;
            for (const auto &message : messages)
            {
                if (message.find(needle) != std::string::npos)
                {
                    ++found;
                }
            }
            return found;
        }

      private:
        provizio::dds::log_callback previous;
        mutable std::mutex mutex;
        std::vector<std::string> messages;
    };

    constexpr std::string_view stall_needle{"listener drain has been waiting"};
    constexpr std::string_view completion_needle{"listener drain completed after"};

    // Case: the drain BLOCKS. A callback that is still running when the reset begins must
    // keep detach_and_drain waiting, because what follows the drain deletes the Fast-DDS
    // entity the callback is inside. Asserted on the callback's own bookkeeping rather
    // than on elapsed time: a drain that returned early would be a use-after-free in
    // production and must not be reported as a slow machine here.
    int test_blocks_until_callbacks_return()
    {
        provizio::dds::detail::listener_drain drain{stall_slice};

        std::atomic<bool> callback_entered{false};
        std::atomic<bool> callback_returned{false};
        std::thread callback{[&] {
            const provizio::dds::detail::listener_drain::scoped_call call{drain};
            callback_entered.store(true);
            std::this_thread::sleep_for(wedged_callback_duration);
            callback_returned.store(true);
        }};

        // The drain has nothing to wait for until the callback is actually in flight, so
        // starting before that would test nothing at all.
        while (!callback_entered.load())
        {
            std::this_thread::yield();
        }

        drain.detach_and_drain();
        // Read after the drain returned: the callback sets it before leaving its
        // scoped_call, and the drain may not return until that scope has been left.
        const bool passed = EXPECT(callback_returned.load()) && EXPECT(drain.is_detached());

        callback.join();
        std::cout << "blocks_until_callbacks_return: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: the stall reports itself repeatedly, and says when it ended. One line cannot
    // say whether a stall is over -- the wait is unbounded and the participant's
    // registration lock is held throughout -- so an operator seeing a single warning
    // could not tell a stall that cleared from one still going hours later.
    int test_reports_a_stall_and_its_end()
    {
        provizio::dds::detail::listener_drain drain{stall_slice};
        const log_capture capture;

        std::atomic<bool> callback_entered{false};
        std::thread callback{[&] {
            const provizio::dds::detail::listener_drain::scoped_call call{drain};
            callback_entered.store(true);
            std::this_thread::sleep_for(wedged_callback_duration);
        }};
        while (!callback_entered.load())
        {
            std::this_thread::yield();
        }

        drain.detach_and_drain();
        callback.join();

        bool passed = true;
        // At least two, not exactly N: the count is what the scheduler grants in the time
        // the callback is held, and pinning it would make the case a timing assertion. Two
        // is what distinguishes a heartbeat from a one-shot report.
        const auto stalls = capture.count_containing(stall_needle);
        passed &= EXPECT(stalls >= 2);
        // Exactly one, and only because a stall was reported: this line exists to
        // supersede the warnings above it.
        passed &= EXPECT(capture.count_containing(completion_needle) == 1);

        std::cout << "reports_a_stall_and_its_end: " << (passed ? "PASS" : "FAIL") << " (" << stalls
                  << " stall report(s))" << '\n';
        return passed ? 0 : 1;
    }

    // Case: the ordinary drain -- nothing in flight, or a callback that returns at once --
    // says nothing at all. The completion line is worth emitting only where a warning
    // preceded it; unconditional, it would put a warning into the log of every network
    // recovery reset on a healthy system.
    int test_quiet_when_nothing_stalls()
    {
        provizio::dds::detail::listener_drain drain{stall_slice};
        const log_capture capture;

        {
            const provizio::dds::detail::listener_drain::scoped_call call{drain};
            // Entered and left before the drain begins, which is the ordinary case.
        }
        drain.detach_and_drain();

        bool passed = EXPECT(capture.count_containing(stall_needle) == 0);
        passed &= EXPECT(capture.count_containing(completion_needle) == 0);

        std::cout << "quiet_when_nothing_stalls: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: a callback that starts after the detach is told to do nothing, but is still
    // waited for. Both halves matter -- the first is what stops user state being touched
    // during a reset, and the second is what stops the entity being deleted underneath a
    // callback that entered a moment too late.
    int test_detached_callbacks_are_still_awaited()
    {
        provizio::dds::detail::listener_drain drain{stall_slice};
        drain.detach_and_drain();

        std::atomic<bool> callback_entered{false};
        std::atomic<bool> callback_returned{false};
        std::atomic<bool> body_ran{false};
        std::thread callback{[&] {
            const provizio::dds::detail::listener_drain::scoped_call call{drain};
            callback_entered.store(true);
            if (call.should_run())
            {
                body_ran.store(true);
            }
            std::this_thread::sleep_for(wedged_callback_duration);
            callback_returned.store(true);
        }};
        while (!callback_entered.load())
        {
            std::this_thread::yield();
        }

        // A second drain, as a second reset would do.
        drain.detach_and_drain();
        bool passed = EXPECT(callback_returned.load());
        passed &= EXPECT(!body_ran.load());

        callback.join();
        // And after reattach a callback runs again, or every endpoint would stay mute for
        // the rest of the process' life.
        drain.reattach();
        {
            const provizio::dds::detail::listener_drain::scoped_call call{drain};
            passed &= EXPECT(call.should_run());
        }

        std::cout << "detached_callbacks_are_still_awaited: " << (passed ? "PASS" : "FAIL") << '\n';
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

    if (subcommand == "blocks_until_callbacks_return")
    {
        return test_blocks_until_callbacks_return();
    }
    if (subcommand == "reports_a_stall_and_its_end")
    {
        return test_reports_a_stall_and_its_end();
    }
    if (subcommand == "quiet_when_nothing_stalls")
    {
        return test_quiet_when_nothing_stalls();
    }
    if (subcommand == "detached_callbacks_are_still_awaited")
    {
        return test_detached_callbacks_are_still_awaited();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
