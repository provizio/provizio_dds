#!/usr/bin/env python3
# Copyright 2026 Provizio Ltd.
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
#
# Cross-version compat request client. Mirrors
# test/python/python_request_response_client.py but on a dedicated DDS
# domain + service name so the test can safely run in parallel with the
# same-version test suite (which uses domain 14 for request/response).

import asyncio
import sys
import provizio_dds

log_prefix = "cross_compat_request_client: "

CROSS_COMPAT_DOMAIN_ID = 42
CROSS_COMPAT_SERVICE_NAME = "provizio_dds_cross_compat_service"


class Timeout:
    pass


async def main():
    timeout = 15
    # The service computes value*value. Rows whose `expected_response == 0`
    # exercise true fire-and-forget — the caller publishes the request and
    # immediately moves on without awaiting (or correlating) the response.
    # We schedule provizio_dds.request as a background task and give the
    # event loop a brief tick to emit the wire send before moving on; the
    # task is reaped at the end so asyncio doesn't log a "Task was
    # destroyed but it is pending!" warning at loop shutdown. The
    # service-side process independently verifies the request count it
    # actually received, so a silent drop of the un-awaited request still
    # fails the pair even though the client doesn't inspect the response
    # here.
    expected_request_response_pairs = [
        (50, 0),   # fire-and-forget: client does not await the response
        (10, 100),
        (20, 400),
        (-15, 225),
        (75, 0),   # fire-and-forget: client does not await the response
    ]

    domain_participant = provizio_dds.make_domain_participant(CROSS_COMPAT_DOMAIN_ID)
    fire_and_forget_tasks: list = []

    for request_val, expected_response in expected_request_response_pairs:
        request_msg = provizio_dds.Int32()
        request_msg.data(request_val)
        print(f"{log_prefix}Requesting {request_val}...")

        if expected_response == 0:
            task = asyncio.create_task(
                provizio_dds.request(
                    domain_participant,
                    provizio_dds.Int32PubSubType,
                    provizio_dds.Int64PubSubType,
                    provizio_dds.Int64,
                    request_msg,
                    service_name=CROSS_COMPAT_SERVICE_NAME,
                )
            )
            fire_and_forget_tasks.append(task)
            # Yield to the event loop so the new task can begin its
            # discovery + publish; the actual publish typically lands
            # after the 1 s stable-match settling window inside
            # provizio_dds.request, so we don't try to guarantee
            # publish here — the end-of-main grace period below is
            # what makes sure the wire send for the *last* fire-and-
            # forget row actually fires before the loop closes.
            await asyncio.sleep(0)
            print(f"{log_prefix}Sent {request_val} without awaiting the response")
            continue

        try:
            response = await asyncio.wait_for(
                provizio_dds.request(
                    domain_participant,
                    provizio_dds.Int32PubSubType,
                    provizio_dds.Int64PubSubType,
                    provizio_dds.Int64,
                    request_msg,
                    service_name=CROSS_COMPAT_SERVICE_NAME,
                ),
                timeout,
            )
        except asyncio.TimeoutError:
            response = Timeout()

        if isinstance(response, Timeout):
            print(f"{log_prefix}Timeout when {expected_response} was expected!")
            return 1
        received = response.data()
        if received != expected_response:
            print(f"{log_prefix}Unexpected value: {expected_response} expected, {received} received")
            return 1
        print(f"{log_prefix}Correctly got expected response = {received}")

    # Keep the asyncio loop alive long enough for the un-awaited tasks
    # to complete their internal endpoint matching + publish before we
    # tear the loop down. provizio_dds.request first awaits a
    # stable-match settling window (~1 s by default) and only then
    # publishes — cancelling the task too early would drop the wire send
    # entirely and the service-side count check would (correctly) fail
    # the pair. 3 s comfortably covers the matching wait + publish even
    # under CI contention. Once the publish has happened, cancelling the
    # response-await is what makes this truly fire-and-forget on the
    # caller's side.
    if fire_and_forget_tasks:
        await asyncio.sleep(3.0)
        for task in fire_and_forget_tasks:
            if not task.done():
                task.cancel()
        await asyncio.gather(*fire_and_forget_tasks, return_exceptions=True)

    print(f"{log_prefix}Successfully completed")
    return 0


sys.exit(asyncio.run(main()))
