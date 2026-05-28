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

set -e

# Resolve to an absolute path before any `cd` so later references stay
# valid regardless of how this script was invoked.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd -P)"
cd "${SCRIPT_DIR}"

source ./python_venv.sh

# Source ROS, if present and not yet sourced
if [[ -z "${ROS_DISTRO:-}" ]]; then
    for ROS_DIR in /opt/ros/* ; do
        if [[ -f "${ROS_DIR}/setup.bash" ]]; then
            # shellcheck disable=SC1091
            source "${ROS_DIR}/setup.bash"
            break
        fi
    done
fi
if [[ -n "${ROS_DISTRO:-}" ]]; then
    echo "ROS 2 ${ROS_DISTRO} sourced"
else
    echo "No ROS 2 sourced"
fi

cd ../../build

# Cross-version wire-interop test against the deployed-fleet baseline
# (provizio_dds 1.10.1, installed into a venv). Only relevant when the
# current build includes Python bindings — the test exercises the python
# pub/sub + request/response paths across the version boundary, with the
# 1.10.1 side providing the canonical wire format radars and pipelines
# already speak.
#
# In CI we *require* the legacy venv to install successfully: a setup
# failure means the run isn't actually verifying wire-compat, which is
# the whole point of the matrix on this branch. Outside CI (developer
# machines without 1.10.1 already cached or building) the setup failure
# is non-fatal — cross_version_compat_test simply reports Skipped via
# its own SKIP_REGULAR_EXPRESSION.
if grep -q '^PYTHON_BINDINGS:BOOL=ON' CMakeCache.txt; then
    SETUP_SCRIPT="${SCRIPT_DIR}/../../test/python/setup_legacy_provizio_dds_venv.sh"
    if PROVIZIO_DDS_LEGACY_PYTHON="$("${SETUP_SCRIPT}")"; then
        export PROVIZIO_DDS_LEGACY_PYTHON
        echo "Cross-version compat enabled: legacy python at ${PROVIZIO_DDS_LEGACY_PYTHON}"
    elif [[ "${CI:-}" == "true" ]]; then
        echo "::error::Failed to set up legacy provizio_dds 1.10.1 venv in CI — cross-version compat would silently skip, which defeats the matrix"
        exit 1
    else
        echo "Warning: Failed to set up legacy provizio_dds 1.10.1 venv — cross_version_compat_test will report Skipped, not run"
    fi
fi

ctest --output-on-failure
