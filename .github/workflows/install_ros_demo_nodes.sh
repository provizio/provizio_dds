#!/bin/bash

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

# Installs ros-<distro>-demo-nodes-cpp, whose talker and listener the ROS 2 interop tests run, and
# upgrades the ROS 2 packages the image already has along with it.
#
# The image is pinned (see resolve_ros_base_image.sh), while the demo nodes come from the ROS apt
# repository as it is on the day, built against that day's sync of the rest of ROS 2. ROS packages
# depend on one another without versions, so installing them alone leaves the image's older ones in
# place, and the two syncs do not link together: the talker then fails to start on a symbol only a
# newer message package defines (has_buffer_fields_service_msgs__msg__ServiceEventInfo, for one).
# Installing the image's ROS packages again in the same transaction takes every one from one sync.
#
#   install_ros_demo_nodes.sh <distro>

set -eu

DISTRO="${1:?usage: install_ros_demo_nodes.sh <distro>}"
export DEBIAN_FRONTEND=noninteractive

# The same two layers of defence against apt's transient failures on CI hosts as install_dependencies.sh's
# apt_get, which says why: apt's own retry of a failed download, and an outer loop refreshing the
# package lists between attempts, for a package version gone 404 once the ROS repository has synced
# again, which this many packages from it make likelier
APT_MAX_ATTEMPTS=${APT_MAX_ATTEMPTS:-5}
APT_OPTIONS=(-o "Acquire::Retries=3" -o "DPkg::Lock::Timeout=180")
apt_get() {
    local attempt=1
    local delay=5
    until apt-get "${APT_OPTIONS[@]}" "$@"; do
        if [[ "${attempt}" -ge "${APT_MAX_ATTEMPTS}" ]]; then
            echo "apt-get $* failed after ${APT_MAX_ATTEMPTS} attempts" >&2
            return 1
        fi
        echo "apt-get $* failed (attempt ${attempt}/${APT_MAX_ATTEMPTS}); retrying in ${delay}s..." >&2
        sleep "${delay}"
        delay=$((delay * 2))
        apt-get "${APT_OPTIONS[@]}" update || true
        attempt=$((attempt + 1))
    done
}

apt_get update
mapfile -t INSTALLED < <(dpkg-query -W -f='${db:Status-Abbrev} ${Package}\n' "ros-${DISTRO}-*" 2>/dev/null |
    awk '$1 == "ii" { print $2 }')
# Without them the demo nodes would be installed alone again, and fail as above
if [[ "${#INSTALLED[@]}" -eq 0 ]]; then
    echo "No ros-${DISTRO}-* package is installed: not the ROS 2 ${DISTRO} image expected" >&2
    exit 1
fi
apt_get install -y "ros-${DISTRO}-demo-nodes-cpp" "${INSTALLED[@]}"
