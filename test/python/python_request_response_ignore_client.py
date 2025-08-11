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

log_prefix = "python_request_response_ignore_client: "


class Timeout:
    pass


async def main():
    service_name = "provizio_dds_test_request_response_ignore"
    domain_id = 14
    timeout = 5

    domain_participant = provizio_dds.make_domain_participant(domain_id)

    # Requests 1..5
    values = [1, 2, 3, 4, 5]
    received = 0

    # Allow some time for matching
    await asyncio.sleep(2)

    for value in values:
        req = provizio_dds.Int32()
        req.data(value)
        try:
            resp = await asyncio.wait_for(
                provizio_dds.request(
                    domain_participant,
                    provizio_dds.Int32PubSubType,
                    provizio_dds.Int64PubSubType,
                    provizio_dds.Int64,
                    req,
                    service_name=service_name,
                ),
                timeout,
            )
        except asyncio.TimeoutError:
            resp = Timeout()

        if value % 2 == 0:
            if not isinstance(resp, Timeout):
                print(f"{log_prefix}Expected timeout for ignored request {value}")
                return 1
        else:
            if isinstance(resp, Timeout):
                print(f"{log_prefix}Timeout! Expected response for request {value}")
                return 1
            expected = value * value
            if resp.data() != expected:
                print(
                    f"{log_prefix}Unexpected response for {value}: got {resp.data()}, expected {expected}"
                )
                return 1
            received += 1

    if received != 3:
        print(f"{log_prefix}Expected 3 responses, got {received}")
        return 1

    print(f"{log_prefix}Successfully validated ignore behavior")
    return 0


exit(asyncio.run(main()))
