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

log_prefix = "python_request_response_ignore_client: "

# Test budgets are scaled by the factor provizio_dds_finalize_tests exports (5 under a
# sanitizer build), exactly as the C++ mirrors scale theirs by PROVIZIO_DDS_TEST_TIMEOUT_SCALE.
_TIMEOUT_SCALE = float(os.environ.get("PROVIZIO_DDS_TEST_TIMEOUT_SCALE", "1") or "1")


class Timeout:
    pass


async def main():
    service_name = "provizio_dds_test_request_response_ignore"
    domain_id = 14
    # The budgets of the C++ mirror (test/request_response/request_response_client/
    # request_response_ignore_client.cpp): a request that must be answered waits as long as
    # matching a fresh pair of endpoints can legitimately take, and one that must be ignored
    # waits only long enough to be sure no answer is coming. This client used to give both 8 s,
    # which is under the mirror's 15 s AND under the 10 s the service itself may hold a response
    # for while the client's reader is still matching (max_time_to_keep_ready_responses in
    # request_response.h, mirrored in provizio_dds.py) -- so it could report a failure while the
    # library was still inside its own contract, which is what one macos-15-intel run did.
    timeout = 15 * _TIMEOUT_SCALE
    ignore_timeout = 5 * _TIMEOUT_SCALE
    # Endpoint matching gets its own deadline just inside the request's, so a request that fails
    # says WHICH half failed: a ServiceMatchingTimeoutError means the client and the service never
    # matched (nothing was ever sent), an asyncio timeout means the request went out and no
    # response came back. The distinction is the first question a CI failure here raises.
    match_timeout = max(timeout - 1.0, 1.0)

    domain_participant = provizio_dds.make_domain_participant(domain_id)

    # Requests 1..5
    values = [1, 2, 3, 4, 5]
    received = 0

    # Allow some time for DDS discovery matching (Windows CI can be slow)
    await asyncio.sleep(3)

    for value in values:
        req = provizio_dds.Int32()
        req.data(value)
        ignored = value % 2 == 0
        try:
            resp = await asyncio.wait_for(
                provizio_dds.request(
                    domain_participant,
                    provizio_dds.Int32PubSubType,
                    provizio_dds.Int64PubSubType,
                    provizio_dds.Int64,
                    req,
                    service_name=service_name,
                    service_match_timeout_sec=match_timeout,
                ),
                ignore_timeout if ignored else timeout,
            )
        except asyncio.TimeoutError:
            resp = Timeout()
        except provizio_dds.ServiceMatchingTimeoutError as matching_error:
            # Never matched: nothing was sent, so this is not the "ignored" outcome even for an
            # even value -- say so instead of passing on a silence that means something else.
            print(f"{log_prefix}Endpoints never matched for request {value}: {matching_error}")
            return 1

        if ignored:
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


sys.exit(asyncio.run(main()))
