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
import random
import sys
import time

import provizio_dds

_START_TIME = time.monotonic()


def _timestamp() -> str:
    seconds = time.monotonic() - _START_TIME
    return f"[{seconds:0.3f}] "


async def main() -> int:
    if len(sys.argv) != 3:
        print(
            "Usage: python_request_response_reliability_client.py "
            "<test_name_postfix> <value>"
        )
        return 1

    test_name_postfix = sys.argv[1]
    value = int(sys.argv[2])

    log_prefix = f"python_request_response_reliability_client{test_name_postfix}: "
    service_name = (
        f"provizio_dds_test_request_response_reliability{test_name_postfix}"
    )
    domain_id = 14
    timeout_sec = 5.0

    random.seed(value)
    wait_ms = random.randint(0, 1999)
    if wait_ms < 1000:
        print(f"{log_prefix}{_timestamp()}Waiting {wait_ms}ms...")
        await asyncio.sleep(wait_ms / 1000.0)

    domain_participant = provizio_dds.make_domain_participant(domain_id)

    request = provizio_dds.Int32()
    request.data(value)

    print(f"{log_prefix}{_timestamp()}Requesting {value}...")
    try:
        response = await asyncio.wait_for(
            provizio_dds.request(
                domain_participant,
                provizio_dds.Int32PubSubType,
                provizio_dds.Int32PubSubType,
                provizio_dds.Int32,
                request,
                service_name=service_name,
            ),
            timeout_sec,
        )
    except asyncio.TimeoutError:
        print(f"{log_prefix}{_timestamp()}Timeout waiting for response")
        return 1

    received_value = response.data()
    if received_value != value:
        print(
            f"{log_prefix}{_timestamp()}Unexpected response {received_value}, "
            f"expected {value}"
        )
        return 1

    print(
        f"{log_prefix}{_timestamp()}Successfully received expected "
        f"response = {received_value}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

