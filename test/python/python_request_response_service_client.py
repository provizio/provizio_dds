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

# Validates ServiceClient issuing CONSECUTIVE requests from a single, created-in-advance client, after an
# explicit wait_for_service() readiness check. Reuses the existing x^2 request/response service.

import asyncio
import sys
import provizio_dds

log_prefix = "python_request_response_service_client: "


async def main():
    service_name = "provizio_dds_test_request_response"
    domain_id = 14
    timeout = 30.0

    # x and x^2 pairs; expected value 0 means "fire the request but don't validate the response".
    expected_request_response_pairs = [
        (50, 0),
        (10, 100),
        (20, 400),
        (-15, 225),
        (75, 0),
    ]

    domain_participant = provizio_dds.make_domain_participant(domain_id)
    client = provizio_dds.ServiceClient(
        domain_participant,
        provizio_dds.Int32PubSubType,
        provizio_dds.Int64PubSubType,
        provizio_dds.Int64,
        service_name=service_name,
    )

    # Created in advance; block until the service(s) are ready (includes the settling window).
    if not await client.wait_for_service(timeout_sec=timeout):
        print(f"{log_prefix}Service did not become ready!")
        return 1
    print(f"{log_prefix}Service ready; sending requests...")

    for request_val, expected_response in expected_request_response_pairs:
        request_msg = provizio_dds.Int32()
        request_msg.data(request_val)
        try:
            response = await asyncio.wait_for(client.request(request_msg), timeout)
        except asyncio.TimeoutError:
            if expected_response != 0:
                print(f"{log_prefix}Timeout waiting when {expected_response} was expected!")
                return 1
            continue

        if expected_response != 0:
            received = response.data()
            if received != expected_response:
                print(
                    f"{log_prefix}Unexpected value received from test_service! "
                    f"{expected_response} expected, {received} received."
                )
                return 1
            print(f"{log_prefix}Correctly got expected response = {received}")

    print(f"{log_prefix}Successfully completed")
    return 0


sys.exit(asyncio.run(main()))
