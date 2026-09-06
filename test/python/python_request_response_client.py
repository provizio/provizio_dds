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

import asyncio
import os
import sys
import provizio_dds

# Test budgets are scaled by the factor provizio_dds_finalize_tests exports (5 under a
# sanitizer build), exactly as the C++ mirrors scale theirs by PROVIZIO_DDS_TEST_TIMEOUT_SCALE.
_TIMEOUT_SCALE = float(os.environ.get("PROVIZIO_DDS_TEST_TIMEOUT_SCALE", "1") or "1")

log_prefix = "python_request_response_client: "


class Timeout:
    pass


async def main():
    service_name = "provizio_dds_test_request_response"
    domain_id = 14
    # Bounded by the service's own 20 s wait (python_request_response_service.py), so the
    # C++ mirror's 30 s would never be reached; scaled like every C++ budget is.
    timeout = 15 * _TIMEOUT_SCALE

    expected_request_response_pairs = [
        (50, 0),
        (10, 100),
        (20, 400),
        (-15, 225),
        (75, 0),
    ]

    domain_participant = provizio_dds.make_domain_participant(domain_id)

    for request_val, expected_response in expected_request_response_pairs:
        request_msg = provizio_dds.Int32()
        request_msg.data(request_val)

        print(f"{log_prefix}Requesting {request_val}...")

        try:
            response = await asyncio.wait_for(
                provizio_dds.request(
                    domain_participant,
                    provizio_dds.Int32PubSubType,
                    provizio_dds.Int64PubSubType,
                    provizio_dds.Int64,
                    request_msg,
                    service_name=service_name,
                ),
                timeout,
            )
        except asyncio.TimeoutError:
            response = Timeout()

        if expected_response != 0:
            if isinstance(response, Timeout):
                print(
                    f"{log_prefix}Timeout waiting when {expected_response} was expected!"
                )
                return 1

            received = response.data()
            if received != expected_response:
                print(
                    f"{log_prefix}Unexpected value received from test_service! {expected_response} expected, {received} received."
                )
                return 1

            print(f"{log_prefix}Correctly got expected response = {received}")

    print(f"{log_prefix}Successfully completed")
    return 0

sys.exit(asyncio.run(main()))
