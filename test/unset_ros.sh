#!/bin/bash

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

# This script unsets ROS environment variables.
# It checks if AMENT_PREFIX_PATH is set. If so, it filters
# PYTHONPATH, LD_LIBRARY_PATH, and PATH to remove any entries
# that are subpaths of AMENT_PREFIX_PATH, then unsets AMENT_PREFIX_PATH.

if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
    # Function to filter a colon-separated path variable
    filter_path() {
        local var_name=$1
        local prefix_to_remove=$2
        local old_path="${!var_name}"
        local new_path=""

        if [ -z "${old_path}" ]; then
            return
        fi

        local OLD_IFS="${IFS:-}"
        IFS=':'
        for path_component in ${old_path}; do
            # Check if the segment starts with the prefix
            if [[ "${path_component}" != "${prefix_to_remove}"* ]]; then
                if [ -n "${new_path}" ]; then
                    new_path="${new_path}:"
                fi
                new_path="${new_path}${path_component}"
            fi
        done
        IFS="${OLD_IFS}"

        export "${var_name}"="${new_path}"
    }

    # Iterate over the variables and filter them
    for var in "PYTHONPATH" "LD_LIBRARY_PATH" "PATH"; do
        filter_path "${var}" "${AMENT_PREFIX_PATH}"
    done

    unset AMENT_PREFIX_PATH
fi
