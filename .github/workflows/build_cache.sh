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
# build_cache.sh [BUILD_TYPE=Release] [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON]

set -eu
set -o pipefail

BUILD_TYPE=${1:-"Release"}
PYTHON=${2:-"ON"}
STATIC_ANALYSIS=${3:-"OFF"}

cd "$(cd "$(dirname "$0")" && pwd -P)"

source ./python_venv.sh

cd ../..

BIN_CACHE_CONFIG_NAME="$(./bin_cache_config_name.sh "${BUILD_TYPE}")"
BIN_CACHE_PATH="$(realpath ./cache)"
TARGET_PATH="${BIN_CACHE_PATH}/${BIN_CACHE_CONFIG_NAME}"
PYTHON_TARGET_PATH="${TARGET_PATH}/python"

PROVIZIO_DDS_CHECK_FILE="${TARGET_PATH}/lib/libprovizio_dds.so"
CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO="${PYTHON_TARGET_PATH}/provizio_dds_python_types/_provizio_dds_python_types.so"

# Check if it's already built
ALREADY_BUILT="FALSE"
if [ -f "${TARGET_PATH}.zip" ]; then
    ALREADY_BUILT="TRUE"
    unzip -q "${TARGET_PATH}.zip" -d "${BIN_CACHE_PATH}"

    if [ ! -f "${PROVIZIO_DDS_CHECK_FILE}" ]; then
        ALREADY_BUILT="FALSE"
    fi
    if [ "${PYTHON}" == "ON" ] && [ ! -f "${CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO}" ]; then
        ALREADY_BUILT="FALSE"
    fi

    rm -rf "${TARGET_PATH}"
fi

if [ "${ALREADY_BUILT}" == "TRUE" ]; then
    echo "The bin cache is already built for ${BIN_CACHE_CONFIG_NAME}"
else
    echo "Building bin cache for ${BIN_CACHE_CONFIG_NAME}..."

    # But first, let's delete any obsolete version there may be
    # shellcheck disable=SC2046
    rm -rf "${BIN_CACHE_PATH:?}"/$(./bin_cache_config_name.sh "${BUILD_TYPE}" WILDCARD)*

    IGNORE_BIN_CACHE=TRUE .github/workflows/build.sh -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DSTATIC_ANALYSIS="${STATIC_ANALYSIS}" -DPYTHON_BINDINGS="${PYTHON}" -DENABLE_TESTS="OFF" -DENABLE_CHECK_FORMAT="OFF" -DCMAKE_INSTALL_PREFIX="${TARGET_PATH}" -DPYTHON_PACKAGES_INSTALL_DIR="${PYTHON_TARGET_PATH}"
    cd ./build
    cmake --install .

    # Delete extra copy of python-specific libs produced by Fast-DDS Python wrapper (as it's already included in dedicated Python subfolder)
    rm -rf "${TARGET_PATH}"/lib/python*
    # Delete extra cmake files
    rm -rf "${TARGET_PATH}/lib/cmake"

    # Collect all the dependencies as well
    collect_libs() {
        local binary="$1"
        local output_dir="$2"
        local lib_basename

        # Update RPATH to make it look for its dependencies in the same directory
        # shellcheck disable=SC2016
        patchelf --set-rpath '$ORIGIN' "${binary}"

        # Use ldd to find shared libraries the binary depends on
        ldd "${binary}" | awk '/=>/ { print $(NF-1) }' | while read -r lib; do
            if [ -n "${lib}" ] && [ -f "${lib}" ]; then
                # Exclude system libraries
                lib_basename="$(basename "${lib}")"
                if [[ "${lib_basename}" != libc.so* && "${lib_basename}" != libm.so* && "${lib_basename}" != librt.so* && "${lib_basename}" != libpthread.so* && "${lib_basename}" != libdl.so* ]]; then
                    # Copy the library if it hasn't been copied yet
                    if [ ! -f "${output_dir}/${lib_basename}" ]; then
                        if [ -L "${lib}" ]; then
                            cp -a "$(realpath "${lib}")" "${output_dir}/"
                        fi
                        cp -a "${lib}" "${output_dir}/"

                        # Recursively collect dependencies of the copied library
                        collect_libs "${output_dir}/${lib_basename}" "${output_dir}"
                    fi
                fi
            fi
        done
    }
    collect_all_libs() {
        local dir_to_process="$1"
        for file in "${dir_to_process}"/*; do
            # Skip if the file is a soft link or not an executable/shared object
            if [ ! -L "${file}" ] && [ -f "${file}" ] && { [[ -x "${file}" ]] || [[ "${file}" == *.so* ]]; }; then
                collect_libs "${file}" "${dir_to_process}"
            fi
        done
    }

    collect_all_libs "${TARGET_PATH}/lib"
    collect_all_libs "${TARGET_PATH}/python/fastdds"
    collect_all_libs "${TARGET_PATH}/python/provizio_dds"
    collect_all_libs "${TARGET_PATH}/python/provizio_dds_python_types"

    # zip it now!
    cd "${BIN_CACHE_PATH}"
    zip -r -y "${BIN_CACHE_CONFIG_NAME}.zip" "${BIN_CACHE_CONFIG_NAME}"
    rm -rf "${TARGET_PATH}"
fi
