// Copyright 2025 Provizio Ltd.
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

#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <random>
#include <string>

#include "provizio/dds/request_response.h"

#include <std_msgs/msg/Int32PubSubTypes.hpp>

namespace
{
    constexpr const char *log_prefix = "request_response_reliability_client: ";

    // Captured at process start (static initialisation), so the timestamps below are relative
    // to the process start rather than the first timestamp() call. This keeps waits that precede
    // the first log line (random startup delay, participant creation, discovery) visible in the
    // deltas. steady_clock::now() is noexcept.
    const auto process_start_time = std::chrono::steady_clock::now();

    std::string timestamp()
    {
        constexpr double ms_in_s = 1000.0;

        return "[" +
               std::to_string(static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                     std::chrono::steady_clock::now() - process_start_time)
                                                     .count()) /
                              ms_in_s) +
               "] ";
    }
}  // namespace

// Arguments: test_name_postfix value [domain_id]
int main(int argc, char *argv[])
try
{
    // Scaled like every other completion deadline in this suite (see
    // PROVIZIO_DDS_TEST_TIMEOUT_SCALE in test/CMakeLists.txt): this waits for a response,
    // and a sanitized build or a loaded runner needs the same slack the outer ctest TIMEOUT
    // already gets. It was the one deadline here left unscaled, so on the runners where
    // discovery for a fresh participant occasionally stalls it gave up 5x sooner than its
    // siblings would have.
    constexpr std::chrono::seconds timeout{30 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};
    constexpr int max_wait_rnd = 1999;
    constexpr int half_wait_rnd = 1000;

    if (argc < 3 || argc > 4)
    {
        std::cerr << "Wrong number of arguments!" << '\n';
        return 1;
    }

    const std::string test_name_postfix = argv[1];  // NOLINT: OK in a unit test
    auto value = std::atoi(argv[2]);                // NOLINT: OK in a unit test
    // Use per-iteration domain IDs (100-127) to avoid DDS multicast discovery state accumulation across rapid
    // participant create/destroy cycles (root cause of flaky timeouts on Windows CI).
    // High range avoids conflicts with other tests; wraps at 127 (max Fast-DDS domain ID).
    constexpr provizio::dds::DomainId_t base_domain_id = 100;
    constexpr provizio::dds::DomainId_t domain_range = 28;  // 100..127
    const provizio::dds::DomainId_t domain_id =
        (argc > 3)
            ? static_cast<provizio::dds::DomainId_t>(base_domain_id + (std::atoi(argv[3]) % domain_range))  // NOLINT
            : 14;  // NOLINT: OK in a unit test

    const std::string test_log_prefix = "request_response_reliability_client" + test_name_postfix + ": ";
    const std::string service_name{"provizio_dds_test_request_response_reliability" + test_name_postfix};

    // Sleep some random time, to test different combinations of client/service startup times
    std::mt19937 engine(value);
    const auto wait = std::uniform_int_distribution<int>{0, max_wait_rnd}(engine);
    if (wait < half_wait_rnd)
    {
        std::cout << test_log_prefix << timestamp() << "Waiting " << wait << "ms..." << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds{wait});
    }

    std_msgs::msg::Int32 request;
    request.data(value);

    std::cout << test_log_prefix << timestamp() << "Requesting " << value << "..." << '\n';
    auto future_response = provizio::dds::request<std_msgs::msg::Int32PubSubType, std_msgs::msg::Int32PubSubType>(
        provizio::dds::make_domain_participant(domain_id), service_name, request);

    const auto wait_status = future_response.wait_for(timeout);
    if (wait_status != std::future_status::ready)
    {
        std::cerr << test_log_prefix << timestamp() << "Timeout waiting for response" << '\n';
        return 1;
    }

    const auto received_value = future_response.get().data();
    if (received_value != value)
    {
        std::cerr << test_log_prefix << timestamp() << "Unexpected response " << received_value << ", expected "
                  << value << '\n';
        return 1;
    }

    std::cout << test_log_prefix << timestamp() << "Successfully received expected response = " << received_value
              << '\n';
    return 0;
}
catch (const std::exception &exception)
{
    std::cerr << log_prefix << timestamp() << "Exception during the test: " << exception.what() << '\n';
    return 1;
}
