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
# NETWORK_LOSS (f.e. 50%)
# NETWORK_RATE (f.e. 128kbit)

set -eu

cd "$(cd "$(dirname "$0")" && pwd -P)"/../..

for i in {1..5}; do
    echo "congested_network_test #${i}..."
    docker container rm /provizio_dds_congested_publisher || true >/dev/null
    docker container rm /provizio_dds_congested_subscriber || true >/dev/null
    if ! COMPOSE_FILE=congested_network_compose.yml docker compose --progress plain up --build --exit-code-from subscriber; then
        echo "Run #${i} failed!"
        exit 1
    fi
done

echo "Successfully communicated in all 5 runs!"
