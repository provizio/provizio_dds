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
// Crash-safety of user callbacks: a user-supplied callback that THROWS must not
// abort the process (it previously escaped into Fast-DDS' listener / a worker
// std::thread -> std::terminate) and must be reported through the configurable
// provizio_dds logging facility (set_log_callback), not stdout. Each subcommand
// is its own ctest entry so per-case failure stays isolated, mirroring
// match_publisher_default_test.

#include <atomic>
#include <chrono>
#include <iostream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/ignore_request.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/request_response.h"
#include "provizio/dds/subscriber.h"

#include <std_msgs/msg/Int32PubSubTypes.hpp>
#include <std_msgs/msg/Int64PubSubTypes.hpp>
#include <std_msgs/msg/StringPubSubTypes.hpp>

namespace
{
    using namespace std::chrono_literals;
    using StringPubSubType = std_msgs::msg::StringPubSubType;
    using Int32PubSubType = std_msgs::msg::Int32PubSubType;
    using Int64PubSubType = std_msgs::msg::Int64PubSubType;

    constexpr auto k_domain = 0;
    constexpr auto k_timeout = 15s;

    // Counts ERROR-level messages routed through the provizio_dds logging
    // facility. Installed via set_log_callback so the test also proves the
    // exception is reported through the customer-replaceable logger rather than
    // printed to stdout/stderr directly. Invoked from Fast-DDS / worker threads,
    // hence the mutex.
    struct log_recorder
    {
        std::mutex mutex;
        int error_count{0};
        std::string last_error;

        int errors()
        {
            const std::lock_guard<std::mutex> lock{mutex};
            return error_count;
        }

        std::string last()
        {
            const std::lock_guard<std::mutex> lock{mutex};
            return last_error;
        }
    };

    log_recorder g_recorder;

    void install_recorder()
    {
        // Clear any prior state so the assertions can't be satisfied by a stale or unrelated
        // ERROR — each case must observe the throw raised by its own callback.
        {
            const std::lock_guard<std::mutex> lock{g_recorder.mutex};
            g_recorder.error_count = 0;
            g_recorder.last_error.clear();
        }
        provizio::dds::set_log_callback([](provizio::dds::log_level level, std::string_view message) {
            if (level == provizio::dds::log_level::error)
            {
                const std::lock_guard<std::mutex> lock{g_recorder.mutex};
                ++g_recorder.error_count;
                g_recorder.last_error.assign(message);
            }
        });
    }

    void restore_recorder()
    {
        provizio::dds::set_log_callback({});
    }

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

    std::shared_ptr<provizio::dds::domain_participant> make_participant()
    {
        return provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
    }

    // Case: a throwing on-data callback. The throw fires on the Fast-DDS
    // reception thread; unguarded it terminates the process. Guarded, it is
    // logged and the subscriber keeps working.
    int test_on_data()
    {
        install_recorder();
        bool passed = true;
        const std::string topic{"provizio_dds_cbexc_on_data_topic"};

        auto pub_participant = make_participant();
        auto sub_participant = make_participant();
        // Force an eager RELIABLE reader (not the match-publisher default) so this case stays focused
        // on callback-exception safety and doesn't depend on deferred-build discovery/settling timing.
        auto subscriber = provizio::dds::make_subscriber<StringPubSubType>(
            sub_participant, topic,
            [](const std_msgs::msg::String &) { throw std::runtime_error("on_data callback boom"); },
            provizio::dds::RELIABLE_RELIABILITY_QOS);
        auto publisher = provizio::dds::make_publisher<StringPubSubType>(
            pub_participant, topic, provizio::dds::RELIABLE_RELIABILITY_QOS, provizio::dds::use_default_history_depth);

        std_msgs::msg::String message;
        message.data("x");
        const auto deadline = std::chrono::steady_clock::now() + k_timeout;
        while (std::chrono::steady_clock::now() < deadline && g_recorder.errors() == 0)
        {
            publisher->publish(message);
            std::this_thread::sleep_for(100ms);
        }

        // Reaching here at all proves the throwing callback did not terminate the
        // process; the error count proves it fired AND was reported via the logger.
        passed &= EXPECT(g_recorder.errors() > 0);
        // ...and the logged error is the one the throwing callback raised, not an unrelated ERROR.
        passed &= EXPECT(g_recorder.last().find("on_data callback boom") != std::string::npos);
        // The subscriber is still alive and matched after the throw.
        passed &= EXPECT(subscriber->get_num_matched_publishers(k_timeout, 0ms) > 0);

        std::cout << "on_data: " << (passed ? "PASS" : "FAIL") << " (errors=" << g_recorder.errors() << ")" << '\n';
        restore_recorder();
        return passed ? 0 : 1;
    }

    // NOTE: the publisher on_matched callback is NOT exercised here. Its first
    // parameter is publisher_handle<T, F>& where F is the callback's own type, so a
    // plain lambda cannot name it (a pre-existing API quirk — no test/example uses
    // it). The publisher-side match callback's crash-safety is covered cross-language
    // by the Python suite (on_has_subscriber_changed), and the C++ wrap mirrors the
    // subscriber on_has_publisher_changed wrap exercised below.

    // Case: a throwing subscriber on-has-publisher-changed callback.
    int test_on_has_publisher_changed()
    {
        install_recorder();
        bool passed = true;
        const std::string topic{"provizio_dds_cbexc_on_pub_changed_topic"};

        auto pub_participant = make_participant();
        auto sub_participant = make_participant();
        auto subscriber = provizio::dds::make_subscriber<StringPubSubType>(
            sub_participant, topic, [](const std_msgs::msg::String &) {},
            [](bool) { throw std::runtime_error("on_has_publisher_changed callback boom"); },
            provizio::dds::RELIABLE_RELIABILITY_QOS);
        auto publisher = provizio::dds::make_publisher<StringPubSubType>(
            pub_participant, topic, provizio::dds::RELIABLE_RELIABILITY_QOS, provizio::dds::use_default_history_depth);

        const auto deadline = std::chrono::steady_clock::now() + k_timeout;
        while (std::chrono::steady_clock::now() < deadline && g_recorder.errors() == 0)
        {
            std::this_thread::sleep_for(100ms);
        }

        passed &= EXPECT(g_recorder.errors() > 0);
        passed &= EXPECT(g_recorder.last().find("on_has_publisher_changed callback boom") != std::string::npos);
        passed &= EXPECT(subscriber->get_num_matched_publishers(k_timeout, 0ms) > 0);

        std::cout << "on_has_publisher_changed: " << (passed ? "PASS" : "FAIL") << " (errors=" << g_recorder.errors()
                  << ")" << '\n';
        restore_recorder();
        return passed ? 0 : 1;
    }

    // Case: a throwing service request handler. Unguarded it escapes the handler
    // std::thread -> std::terminate. Guarded, each throw is logged and the
    // handler thread survives to process the next request (so >= 2 errors prove
    // the thread is still alive after the first throw).
    int test_service_handler()
    {
        install_recorder();
        bool passed = true;
        const std::string service_name{"provizio_dds_cbexc_service"};

        auto service_participant = make_participant();
        auto client_participant = make_participant();
        auto service = provizio::dds::make_service<Int32PubSubType, Int64PubSubType>(
            service_participant, service_name, [](const std_msgs::msg::Int32 &) -> std_msgs::msg::Int64 {
                throw std::runtime_error("service handler boom");
            });

        std_msgs::msg::Int32 request_data;
        request_data.data(1);
        // Wait for >= 2 throws (two request cycles), so allow twice the single-error
        // budget: the second cycle is fast once this participant has discovered the
        // service, but the first can be slow in the ROS2-compat Debug containers.
        const auto deadline = std::chrono::steady_clock::now() + 2 * k_timeout;
        while (std::chrono::steady_clock::now() < deadline && g_recorder.errors() < 2)
        {
            // Keep the future alive and wait on it so the request is actually
            // delivered to the service (and the handler runs). The handler throws,
            // so the response never arrives — the wait times out, which is fine;
            // we only need the handler to fire. Dropping the future immediately
            // would tear the client down before the request is sent.
            auto future = provizio::dds::request<Int32PubSubType, Int64PubSubType>(client_participant, service_name,
                                                                                   request_data);
            future.wait_for(2s);
        }

        // Require >= 2 throws: one proves the handler fired without terminating the
        // process, and the second proves the handler thread SURVIVED the first throw
        // and went on to process another request — the regression this test guards
        // against (a handler that logs once and then dies).
        passed &= EXPECT(g_recorder.errors() >= 2);
        passed &= EXPECT(g_recorder.last().find("service handler boom") != std::string::npos);

        std::cout << "service_handler: " << (passed ? "PASS" : "FAIL") << " (errors=" << g_recorder.errors() << ")"
                  << '\n';
        restore_recorder();
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
    const std::string subcommand{args[1]};
    if (subcommand == "on_data")
    {
        return test_on_data();
    }
    if (subcommand == "on_has_publisher_changed")
    {
        return test_on_has_publisher_changed();
    }
    if (subcommand == "service_handler")
    {
        return test_service_handler();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
