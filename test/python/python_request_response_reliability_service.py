#!/usr/bin/env python3

# Copyright 2025 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import random
import sys
import threading
import time

import provizio_dds

_START_TIME = time.monotonic()


def _timestamp() -> str:
    seconds = time.monotonic() - _START_TIME
    return f"[{seconds:0.3f}] "


def main() -> int:
    if len(sys.argv) not in (4, 5):
        print(
            "Usage: python_request_response_reliability_service.py "
            "<test_name_postfix> <first_iteration_value> <num_iterations> [<domain_id>]"
        )
        return 1

    test_name_postfix = sys.argv[1]
    expected_value = int(sys.argv[2])
    num_iterations = int(sys.argv[3])
    # Remap to domain IDs 100-127 to avoid exceeding Fast-DDS max (127) and
    # to prevent multicast discovery state accumulation across rapid
    # participant create/destroy cycles.
    _BASE_DOMAIN_ID = 100
    _DOMAIN_RANGE = 28  # 100..127
    domain_id = (
        (_BASE_DOMAIN_ID + int(sys.argv[4]) % _DOMAIN_RANGE)
        if len(sys.argv) > 4
        else 14
    )
    initial_iterations = max(1, num_iterations)
    wait_timeout = num_iterations * 3.0 + 5

    log_prefix = f"python_request_response_reliability_service{test_name_postfix}: "
    service_name = f"provizio_dds_test_request_response_reliability{test_name_postfix}"

    max_wait_both_sides_ms = 1999
    wait_in_server_over_ms = 1000  # In every iteration we either postpone the client or the server, never both
    ms_in_s = 1000.0

    random.seed(expected_value)
    wait_ms = random.randint(0, max_wait_both_sides_ms)
    if wait_ms >= wait_in_server_over_ms:
        print(f"{log_prefix}{_timestamp()}Waiting {wait_ms}ms...")
        time.sleep(wait_ms / ms_in_s)

    condition_variable = threading.Condition()
    requests_processed = False
    received_expected_values = True

    domain_participant = provizio_dds.make_domain_participant(domain_id)

    def handle_request(request):
        nonlocal expected_value, num_iterations, requests_processed
        nonlocal received_expected_values

        value = request.data()
        response = provizio_dds.Int32()
        response.data(expected_value)

        print(f"{log_prefix}{_timestamp()}Got request: {value}")

        with condition_variable:
            requests_processed = num_iterations <= 1
            if value != expected_value:
                print(
                    f"{log_prefix}{_timestamp()}Incorrect request received: "
                    f"{value} when {expected_value} was expected!"
                )
                received_expected_values = False
            expected_value += 1
            num_iterations -= 1
            if requests_processed:
                condition_variable.notify_all()

        print(f"{log_prefix}{_timestamp()}Responding value: {response.data()}")
        return response

    service = provizio_dds.Service(
        domain_participant=domain_participant,
        service_name=service_name,
        request_pub_sub_type=provizio_dds.Int32PubSubType,
        request_data_type=provizio_dds.Int32,
        response_pub_sub_type=provizio_dds.Int32PubSubType,
        handle_request_function=handle_request,
        max_history_depth=initial_iterations,
    )

    simulated_request_processing_time_sec=0.25
    try:
        with condition_variable:
            finished = condition_variable.wait_for(
                lambda: requests_processed, timeout=wait_timeout
            )
        if not finished:
            print(f"{log_prefix}{_timestamp()}Timeout waiting for request")
            return 1

        if not received_expected_values:
            return 1

        time.sleep(simulated_request_processing_time_sec)
        print(
            f"{log_prefix}{_timestamp()}Successfully processed value "
            f"{expected_value}"
        )
        return 0
    finally:
        service.stop()


if __name__ == "__main__":
    sys.exit(main())
