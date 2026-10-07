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

set -eo pipefail

cd "$(cd "$(dirname "$0")" && pwd -P)"

# Make (or re-make) and activate a test Python virtual environment
VENV=/tmp/provizio_dds_test_pip_package.venv
rm -rf ${VENV}
python3 -m venv ${VENV}
# shellcheck source=/dev/null
source ${VENV}/bin/activate

# Manually install some dependencies in older versions of Python to avoid known incompatibilities in numpy and Cython
python3 -m pip install wheel setuptools
python_version=$(python3 -c 'import sys; print("".join(map(str, sys.version_info[:2])))')
if [[ "${python_version}" -lt "39" ]]; then
    python3 -m pip install "Cython<3" "numpy>=1.16"
fi

export CC=${CC:-"gcc"}
if [ -z "${CXX:-}" ]; then
    case "${CC}" in
        gcc)
            export CXX=g++
            ;;
        clang)
            export CXX=clang++
            ;;
        *)
            ;;
    esac
fi

cd ../../

# Sweep pip's leftovers from earlier jobs before adding to them. pip copies the whole
# source directory into a temporary build directory before building it, and deletes that
# copy when it exits -- but a job that is cancelled or killed never lets it, and nothing
# else ever removes it. On the self-hosted jetson pool two such trees, 5.2 and 6.2 GB,
# dated three and four days earlier, were most of what had brought a runner to 98% full;
# the install that finally hit the wall failed with "[Errno 28] No space left on device"
# partway through its own copy. Each run leaves the next one less room until a human
# intervenes, which is what this loop is for.
#
# Two guards, not one. The age guard alone was justified as "a self-hosted runner takes one
# job at a time", which is not a safe assumption for this workflow's self-hosted jetson pool:
# its runners have shared a physical machine and its /tmp before, and the ARM pip matrix alone
# schedules more than a dozen jobs onto it. A from-source build for a new Python on ARM can
# legitimately exceed an hour, so the age guard by itself would delete a SIBLING JOB's tree
# mid-build, and the failure would surface somewhere else entirely.
#
#   -user: only our own temporaries. install_dependencies.sh runs pip as root, so a cancelled
#          job leaves root-owned pip-* trees the runner user cannot remove -- and under
#          `set -eo pipefail` that failing rm aborted this job with a message pointing nowhere
#          near the cause. Restricting the search is better than ignoring the error, because
#          those trees were never ours to delete.
#   -mmin: and of those, only ones old enough to belong to a job that is gone.
#
# -exec rather than a newline-delimited read loop: find's output is not newline-safe, and a
# path containing one split into a second, relative, iteration -- `rm -rf build` run from the
# repo root. `|| true` because a tree vanishing between the find and the rm is not an error.
find "${TMPDIR:-/tmp}" -maxdepth 1 -user "$(id -u)" -mmin +60 \
    \( -name 'pip-req-build-*' -o -name 'pip-install-*' -o -name 'pip-build-env-*' \
       -o -name 'pip-wheel-*' -o -name 'pip-ephem-wheel-cache-*' -o -name 'pip-unpack-*' \) \
    -exec sh -c 'echo "Removing a stale pip temporary left by an earlier job: $1"; rm -rf "$1"' _ {} \; \
    2>/dev/null || true

# Free space, reported before the install rather than guessed at afterwards: an ENOSPC
# during a pip build surfaces as a shutil stack trace deep in pip's output, which is a poor
# way to learn that a runner simply had no room. The warning names the machine while there
# is still time to prune it, and stays a warning rather than a refusal because the install
# may well still fit -- a job that will not run is worse than one that says why it might not.
# Both df calls cannot be allowed to fail the script: this runs under `set -eo pipefail`, and
# the whole point of the block is to WARN rather than refuse. A df that exits non-zero -- a
# TMPDIR that has gone away, a stale mount on a self-hosted runner -- would otherwise abort
# here, before the pip install this file exists to test has run at all. The 2>/dev/null on the
# second one hides df's message but not its status, so pipefail still needs the `|| true`.
echo "Free space before the pip install:"
df -h "${TMPDIR:-/tmp}" || true
free_kb=$(df -Pk "${TMPDIR:-/tmp}" 2>/dev/null | awk 'NR == 2 {print $4}' || true)
if [ -n "${free_kb}" ] && [ "${free_kb}" -lt 10485760 ]; then
    echo "::warning::$(hostname) has only $((free_kb / 1024 / 1024)) GiB free on the volume \
holding ${TMPDIR:-/tmp}; pip copies the whole source tree there before building it. Prune \
stale job workspaces and pip temporaries on this runner."
fi

# Build and install the package, capturing output to verify binary cache usage. A name of its own
# rather than a fixed one in /tmp, which another job on a shared self-hosted runner would write too.
PIP_LOG="$(mktemp)"
trap 'rm -f "${PIP_LOG}"' EXIT
python3 -m pip install -v . 2>&1 | tee "${PIP_LOG}"

# Verify the package was built the way this job is here to test. PIP_PACKAGE_BUILD=source is the
# build from source that a user's pip install falls back to whenever no prebuilt binaries fit their
# host or configuration -- which the job forces as a user would, through CMAKE_ARGUMENTS, since the
# prebuilt binaries would otherwise be taken wherever they exist. Anywhere else, on Linux, the
# prebuilt binaries must have been used: every other pip job runs on the commit CI publishes them in.
# macOS has none to use.
case "${PIP_PACKAGE_BUILD:-}" in
source)
    if ! grep -q "Building C++ libraries from source" "${PIP_LOG}"; then
        echo "::error::The pip install did NOT build from source, which is what this job is here to test"
        exit 1
    fi
    echo "Verified: built from source"
    ;;
*)
    if [ "$(uname -s)" != "Darwin" ]; then
        if ! grep -q "Bin cache located and will be used" "${PIP_LOG}"; then
            echo "::error::Binary cache was NOT used during pip install - check cache artifacts and CMake config"
            exit 1
        fi
        echo "Verified: binary cache was used"
    fi
    ;;
esac
rm -f "${PIP_LOG}"

# Test it works fine by executing Python tests directly (without copying provizio_dds.py and other beside the tests)
python3 test/python/python_publisher.py & python3 test/python/python_subscriber.py
python3 test/python/point_cloud2_test.py

# Deactivate and delete the virtual environment
deactivate
rm -rf ${VENV}
