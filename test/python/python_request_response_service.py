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
import provizio_dds

log_prefix = "python_request_response_service: "

def main():
    service_name = "provizio_dds_test_request_response"
    domain_id = 14
    requests_expected = 5
    total_timeout = 20
    end_sleep = 4

    condition_variable = threading.Condition()
    got_requests = 0

    domain_participant = provizio_dds.make_domain_participant(domain_id)

    def handle_request(request):
        nonlocal got_requests
        response = provizio_dds.Int64()
        value = request.data() * request.data()

        print(f"{log_prefix}Processing request {request.data()} => {value}")
        time.sleep(value / 1000.0)

        response.data(int(value))

        with condition_variable:
            got_requests += 1
            if got_requests >= requests_expected:
                condition_variable.notify_all()

        print(f"{log_prefix}Response sent ({request.data()} => {value})")
        return response

    print(f"{log_prefix}Waiting for requests...")
    service = provizio_dds.Service(
        domain_participant=domain_participant,
        service_name=service_name,
        request_pub_sub_type=provizio_dds.Int32PubSubType,
        request_data_type=provizio_dds.Int32,
        response_pub_sub_type=provizio_dds.Int64PubSubType,
        handle_request_function=handle_request,
        max_history_depth=requests_expected,
    )

    with condition_variable:
        if not condition_variable.wait_for(lambda: got_requests >= requests_expected, timeout=total_timeout):
            print(f"{log_prefix}Timeout! Got {got_requests} requests so far")
            service.stop()
            return 1

    time.sleep(end_sleep)

    print(f"{log_prefix}Successfully sent all the responses")

    service.stop()
    return 0

exit(main())
