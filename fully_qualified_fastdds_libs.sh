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

# Use as:
# fully_qualified_fastdds_libs.sh LIB_DIR_TO_PATCH [revert]

set -eu
set -o pipefail

if [ "${OSTYPE}" != "linux-gnu" ]; then
    echo "Only Linux is supported"
    exit 1
fi

LIB_DIR_TO_PATCH="$1"
REVERT="${2:-}"

if [[ "${REVERT}" == "revert" ]]; then
    echo "Adding softlinks for Fast-DDS libs in ${LIB_DIR_TO_PATCH}..."
    for file in "${LIB_DIR_TO_PATCH}"/libfast*.so.*.*.*; do
        if [[ ! -L "${file}" ]]; then
            file_basename="$(basename "${file}")"
            ln -fs "${file_basename}" "${LIB_DIR_TO_PATCH}/${file_basename%.*}" #.so.x.y
            ln -fs "${file_basename}" "${LIB_DIR_TO_PATCH}/${file_basename%.*.*}" #.so.x
            ln -fs "${file_basename}" "${LIB_DIR_TO_PATCH}/${file_basename%.*.*.*}" #.so
            echo "Softlinks added for ${file_basename}"
        fi
    done
else
    echo "Patching all libraries in ${LIB_DIR_TO_PATCH} to link Fast-DDS libs by fully qualified names..."

    for file in "${LIB_DIR_TO_PATCH}"/*; do
        # Skip if the file is a soft link or not an executable/shared object
        if [ ! -L "${file}" ] && [ -f "${file}" ] && { [[ -x "${file}" ]] || [[ "${file}" == *.so* ]]; }; then
            LD_LIBRARY_PATH="${LIB_DIR_TO_PATCH}:${LD_LIBRARY_PATH:-}" ldd "${file}" | awk '/=>/ { print $(NF-1) }' | while read -r lib; do
                lib_basename="$(basename "${lib}")"
                if [[ "${lib_basename}" == libfast*.so* && -L "${lib}" ]]; then
                    lib_full_path="$(realpath "${lib}")"
                    lib_full_name="$(basename "${lib_full_path}")"

                    echo "Patching ${file} so it links against ${lib_full_name} instead of ${lib_basename}"
                    patchelf --replace-needed "${lib_basename}" "${lib_full_name}" "${file}"
                fi
            done
        fi
    done

    # Now remove the extra softlinks
    for file in "${LIB_DIR_TO_PATCH}"/libfast*.so*; do
        if [[ -L "${file}" ]]; then
            rm "${file}"
            echo "Extra softlink ${file} removed"
        fi
    done
fi
