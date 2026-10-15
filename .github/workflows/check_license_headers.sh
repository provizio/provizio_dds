#!/bin/bash

# Copyright 2023 Provizio Ltd.
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

# List of all excluded files
EXCLUDED=("./python/gps_utils.py")

cd "$(cd "$(dirname "$0")" && pwd -P)"
cd ../..

# Use as check_license_header file_to_check comment_mark
check_license_header() {
    OUTPUT_PREFIX="Checking license header:"
    FAILURE_PREFIX="License header is missing or invalid in"
    FILE=$1
    COMMENT_MARK=$2

    for EX in "${EXCLUDED[@]}"; do
        if [[ "${EX}" == "${FILE}" ]]; then
            # Ignore the check for this file
            return
        fi
    done

    echo "${OUTPUT_PREFIX} ${FILE}"
    grep -q "${COMMENT_MARK} Copyright 2[0-9][0-9][0-9] Provizio Ltd." "${FILE}" || (echo "${FAILURE_PREFIX} ${FILE}"; exit 1)
    grep -q "${COMMENT_MARK} Licensed under the Apache License, Version 2.0 (the \"License\");" "${FILE}" || (echo "${FAILURE_PREFIX} ${FILE}"; exit 1)
}

# check_license_headers <comment mark> <name pattern>...: checks every file matching one of the
# patterns, outside ./build. NUL-separated, so that no path is split at a space or read as a glob.
check_license_headers() {
    local comment_mark="$1"
    shift
    local names=(-name "$1")
    shift
    local pattern
    for pattern in "$@"; do
        names+=(-o -name "${pattern}")
    done
    local file
    while IFS= read -r -d '' file; do
        check_license_header "${file}" "${comment_mark}"
    done < <(find . -path ./build -prune -o -type f \( "${names[@]}" \) -print0)
}

check_license_headers "//" '*.c' '*.cpp' '*.h' '*.hpp'
check_license_headers "#" '*.sh' '*.py'
check_license_headers "#" 'CMakeLists.txt' '*.cmake' '*.cmake.in'
check_license_headers "::" '*.bat'
check_license_headers "#" '*.ps1'

echo "Licence headers OK"
