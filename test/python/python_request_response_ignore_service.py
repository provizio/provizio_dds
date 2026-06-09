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

import time
import threading
import sys
import provizio_dds

log_prefix = "python_request_response_ignore_service: "


def main():
    service_name = "provizio_dds_test_request_response_ignore"
    domain_id = 14
    requests_expected = 5
    total_timeout = 25
    end_sleep = 4

    domain_participant = provizio_dds.make_domain_participant(domain_id)

    print(f"{log_prefix}Starting...")

    condition_variable = threading.Condition()
    got_requests = 0

    def handle_request(request: provizio_dds.Int32) -> provizio_dds.Int64:
        print(f"{log_prefix}Received request {request.data()}")
        
        nonlocal got_requests
        value = request.data()

        # Count the request before filtering, to mirror C++ test behavior
        with condition_variable:
            got_requests += 1
            if got_requests >= requests_expected:
                condition_variable.notify_all()

        if value % 2 == 0:
            # Drop even requests silently
            raise provizio_dds.Service.IgnoreRequest()

        response = provizio_dds.Int64()
        response.data(value * value)
        return response

    service = provizio_dds.Service(
        domain_participant=domain_participant,
        service_name=service_name,
        request_pub_sub_type=provizio_dds.Int32PubSubType,
        request_data_type=provizio_dds.Int32,
        response_pub_sub_type=provizio_dds.Int64PubSubType,
        handle_request_function=handle_request,
    )

    # Wait until all 5 requests were observed
    with condition_variable:
        if not condition_variable.wait_for(lambda: got_requests >= requests_expected, timeout=total_timeout):
            print(f"{log_prefix}Timeout! Got {got_requests} requests so far")
            service.stop()
            return 1

    # Give some time to deliver the last response
    time.sleep(end_sleep)

    service.stop()
    return 0


sys.exit(main())
