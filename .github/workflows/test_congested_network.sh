#!/bin/bash

# Copyright 2022 Provizio Ltd.
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

# Respects the following arguments from env variables:
# NETWORK_DELAY (f.e. 300ms)
# PACKETS_LOSS (f.e. 50%)
# NETWORK_RATE (f.e. 128kbit)
# DISCOVERY_SERVER (ON/OFF)
# NUM_ITERATIONS (f.e. 5)

set -eu

cd "$(cd "$(dirname "$0")" && pwd -P)"/../..

NUM_ITERATIONS=${NUM_ITERATIONS:-5}

if [[ "${DISCOVERY_SERVER:-}" == "ON" ]]; then
    echo "Participants matching will use a discovery server"
    export XML_PROFILE_PUBLISHER="/opt/provizio_dds/test/congested_network_test/fast_dds_server_config.xml"
    export XML_PROFILE_SUBSCRIBER="/opt/provizio_dds/test/congested_network_test/fast_dds_client_config.xml"
else
    echo "Participants matching will use the simple discovery protocol"
    export XML_PROFILE_PUBLISHER=""
    export XML_PROFILE_SUBSCRIBER=""
fi

(docker container rm provizio_dds_congested_publisher || true) >/dev/null 2>&1
(docker container rm provizio_dds_congested_subscriber || true) >/dev/null 2>&1
(docker network rm provizio_dds_congested_network || true) >/dev/null 2>&1

for (( i = 0; i <= NUM_ITERATIONS; i++ )); do
    echo "congested_network_test #${i}..."
    if ! COMPOSE_FILE=congested_network_compose.yml docker compose --progress plain up --force-recreate --build --exit-code-from subscriber; then
        echo "Run #${i} failed!"
        exit 1
    fi
    echo "congested_network_test #${i}: Success!"
    (docker container rm provizio_dds_congested_publisher || true) >/dev/null 2>&1
    (docker container rm provizio_dds_congested_subscriber || true) >/dev/null 2>&1
    (docker network rm provizio_dds_congested_network || true) >/dev/null 2>&1
done

echo "Successfully communicated in all ${NUM_ITERATIONS} runs!"
