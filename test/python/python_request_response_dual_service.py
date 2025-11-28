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

log_prefix = "python_request_response_dual_service: "


async def main():
    domain_id = 14
    service_name = "provizio_dds_test_request_response_dual"

    dp = provizio_dds.make_domain_participant(domain_id)

    # Service 1: odd only
    def handle_odd(req: provizio_dds.Int32) -> provizio_dds.Int64:
        if req.data() % 2 == 0:
            raise provizio_dds.Service.IgnoreRequest()
        resp = provizio_dds.Int64()
        resp.data(req.data() * req.data())
        return resp

    # Service 2: even only
    def handle_even(req: provizio_dds.Int32) -> provizio_dds.Int64:
        if req.data() % 2 != 0:
            raise provizio_dds.Service.IgnoreRequest()
        resp = provizio_dds.Int64()
        resp.data(req.data() * req.data())
        return resp

    s1 = provizio_dds.Service(
        dp,
        provizio_dds.Int32PubSubType,
        provizio_dds.Int32,
        provizio_dds.Int64PubSubType,
        handle_odd,
        service_name=service_name,
    )
    s2 = provizio_dds.Service(
        dp,
        provizio_dds.Int32PubSubType,
        provizio_dds.Int32,
        provizio_dds.Int64PubSubType,
        handle_even,
        service_name=service_name,
    )

    await asyncio.sleep(0.1)

    # Client issuing requests 1..5 with extra 0.5s delay post-match
    async def send_one(v: int):
        msg = provizio_dds.Int32()
        msg.data(v)
        resp = await provizio_dds.request(
            dp,
            provizio_dds.Int32PubSubType,
            provizio_dds.Int64PubSubType,
            provizio_dds.Int64,
            msg,
            service_name=service_name,
        )
        expected = v * v
        if resp.data() != expected:
            print(f"{log_prefix}Unexpected response for {v}: got {resp.data()}, expected {expected}")
            return 1
        return 0

    tasks = [send_one(v) for v in [1, 2, 3, 4, 5]]
    results = await asyncio.gather(*tasks)
    rc = 0 if sum(results) == 0 else 1

    s1.stop()
    s2.stop()
    return rc


exit(asyncio.run(main()))


