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
import provizio_dds

log_prefix = "python_request_response_concurrent_client: "


class Timeout:
    pass


async def execute_with_timeout(task, timeout: float):
    try:
        # Use asyncio.wait_for to apply the timeout
        result = await asyncio.wait_for(task, timeout=timeout)
        return result
    except asyncio.TimeoutError:
        return Timeout()


async def main():
    service_name = "provizio_dds_test_request_response"
    domain_id = 14
    time_to_match = 3
    timeout = 20

    expected_request_response_pairs = [
        (50, 0),
        (10, 100),
        (20, 400),
        (-15, 225),
        (75, 0),
    ]

    domain_participant = provizio_dds.make_domain_participant(domain_id)

    request_tasks = []
    for request_val, response_val in expected_request_response_pairs:
        request_msg = provizio_dds.Int32()
        request_msg.data(request_val)

        print(
            f"{log_prefix}Requesting {request_val} with expected response of {response_val}..."
        )

        request_tasks.append(
            execute_with_timeout(
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
        )

    # Some time for matching
    await asyncio.sleep(time_to_match)
    print(f"{log_prefix}Requests sent, expecting responses now...")

    responses = await asyncio.gather(*request_tasks)

    # Wait for all responses concurrently
    for response, expected_response in zip(
        responses,
        [pair[1] for pair in expected_request_response_pairs],
    ):
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

    print(f"{log_prefix}Successfully complete")
    return 0


exit(asyncio.run(main()))
