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

# Building the images is INFRASTRUCTURE, and it fails for reasons that have nothing to do with
# provizio_dds: a Docker Hub pull that 5xx's or hits a rate limit, an apt mirror hiccup inside
# install_dependencies.sh, a PyPI timeout. Running the containers is the TEST, and its exit code
# is the result -- retrying that would hide exactly the regressions this suite exists to catch.
# So the two are split: the build retries, the run never does.
build_images() {
    local attempt=1
    local delay=15
    local log
    log="$(mktemp)"
    local status
    while true; do
        # Streamed through tee so the build stays visible live AND can be inspected below.
        # PIPESTATUS[0] rather than the pipeline's status, which is tee's and always 0 -- reading
        # that would treat every failed build as a success. Declared before the pipeline because
        # any intervening command resets PIPESTATUS.
        COMPOSE_FILE=congested_network_compose.yml docker compose --progress plain build 2>&1 | tee "${log}" || true
        status="${PIPESTATUS[0]}"
        if [[ "${status}" -eq 0 ]]; then
            rm -f "${log}"
            return 0
        fi
        # A registry rate limit is NOT a transient fault. Docker Hub answers a pull past its
        # unauthenticated per-IP limit with an immediate HTTP 429, and the limit resets on an
        # hourly window — no backoff this loop could apply will outlast it. Retrying only burns
        # minutes and buries the real reason, so say what happened and stop. (The base image
        # defaults to a mirror that is not subject to those limits precisely so this should not
        # arise; see BASE_IMAGE in the dockerfile.)
        if grep -qiE "toomanyrequests|rate limit|pull rate limit|unauthorized: authentication required" "${log}"; then
            echo "docker compose build hit a registry rate limit or an authentication error --" \
                 "NOT retrying, because neither clears within a build. Authenticate the runner" \
                 "to the registry, or set BASE_IMAGE to one it can pull."
            rm -f "${log}"
            return 1
        fi
        if [[ "${attempt}" -ge "${DOCKER_BUILD_MAX_ATTEMPTS}" ]]; then
            echo "docker compose build failed after ${DOCKER_BUILD_MAX_ATTEMPTS} attempts"
            rm -f "${log}"
            return 1
        fi
        echo "docker compose build failed (attempt ${attempt}/${DOCKER_BUILD_MAX_ATTEMPTS}); retrying in ${delay}s..."
        sleep "${delay}"
        delay=$((delay * 2))
        attempt=$((attempt + 1))
    done
}

DOCKER_BUILD_MAX_ATTEMPTS=${DOCKER_BUILD_MAX_ATTEMPTS:-3}
if ! build_images; then
    exit 1
fi

for (( i = 0; i <= NUM_ITERATIONS; i++ )); do
    echo "congested_network_test #${i}..."
    # --no-build: the images were built (with retries) above, so a failure here is the test's.
    if ! COMPOSE_FILE=congested_network_compose.yml docker compose --progress plain up --force-recreate --no-build --exit-code-from subscriber; then
        echo "Run #${i} failed!"
        exit 1
    fi
    echo "congested_network_test #${i}: Success!"
    (docker container rm provizio_dds_congested_publisher || true) >/dev/null 2>&1
    (docker container rm provizio_dds_congested_subscriber || true) >/dev/null 2>&1
    (docker network rm provizio_dds_congested_network || true) >/dev/null 2>&1
done

echo "Successfully communicated in all ${NUM_ITERATIONS} runs!"
