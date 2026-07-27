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

# In aarch64, make sure libstdc++.so.6.0.28 is used, to be compatible with both Orin and TX2
if [[ "$(uname -i)" == "aarch64" && "$(realpath /usr/lib/aarch64-linux-gnu/libstdc++.so.6)" != "/usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.28" ]]; then
  echo "/usr/lib/aarch64-linux-gnu/libstdc++.so.6 is $(realpath /usr/lib/aarch64-linux-gnu/libstdc++.so.6) while /usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.28 is required for compatibility!"
  exit 1
fi

source ./python_venv.sh

cd ../..

BIN_CACHE_CONFIG_NAME="$(./bin_cache_config_name.sh "${BUILD_TYPE}")"
BIN_CACHE_PATH="$(realpath ./cache)"
TARGET_PATH="${BIN_CACHE_PATH}/${BIN_CACHE_CONFIG_NAME}"
PYTHON_TARGET_PATH="${TARGET_PATH}/python"

# Detect Python ABI group tag when building with Python.
# Python 3.8-3.13 share ABI compatibility (tag "3"), while 3.14+ broke ABI (tag "3_14").
PYTHON_VERSION_TAG=""
if [ "${PYTHON}" == "ON" ]; then
    PYTHON_VERSION_TAG="$(python3 -c 'import sys; print("3_14" if sys.version_info >= (3, 14) else "3")')"
fi

PYTHON_CACHE_CONFIG_NAME=""
if [ -n "${PYTHON_VERSION_TAG}" ]; then
    PYTHON_CACHE_CONFIG_NAME="$(./bin_cache_config_name.sh "${BUILD_TYPE}" "" "${PYTHON_VERSION_TAG}")"
fi

PROVIZIO_DDS_CHECK_FILE="${TARGET_PATH}/lib/libprovizio_dds.so"
CACHED_PROVIZIO_DDS_PYTHON_TYPES_SO="${PYTHON_TARGET_PATH}/provizio_dds_python_types/_provizio_dds_python_types.so"

# Check if it's already built
ALREADY_BUILT="FALSE"
if [ "${PYTHON}" == "ON" ]; then
    # When building with Python, both general and python caches must exist
    if [ -f "${BIN_CACHE_PATH}/${BIN_CACHE_CONFIG_NAME}.zip" ] && [ -f "${BIN_CACHE_PATH}/${PYTHON_CACHE_CONFIG_NAME}.zip" ]; then
        ALREADY_BUILT="TRUE"

        # Verify the general cache contains expected files
        unzip -q "${BIN_CACHE_PATH}/${BIN_CACHE_CONFIG_NAME}.zip" -d "${BIN_CACHE_PATH}"
        if [ ! -f "${PROVIZIO_DDS_CHECK_FILE}" ]; then
            ALREADY_BUILT="FALSE"
        fi
        rm -rf "${TARGET_PATH}"

        # Verify the python cache contains expected files
        if [ "${ALREADY_BUILT}" == "TRUE" ]; then
            unzip -q "${BIN_CACHE_PATH}/${PYTHON_CACHE_CONFIG_NAME}.zip" -d "${BIN_CACHE_PATH}"
            PYTHON_CACHE_EXTRACTED="${BIN_CACHE_PATH}/${PYTHON_CACHE_CONFIG_NAME}"
            if [ ! -f "${PYTHON_CACHE_EXTRACTED}/python/provizio_dds_python_types/_provizio_dds_python_types.so" ]; then
                ALREADY_BUILT="FALSE"
            fi
            rm -rf "${PYTHON_CACHE_EXTRACTED}"
        fi
    fi
else
    # When building without Python, only the general cache must exist
    if [ -f "${BIN_CACHE_PATH}/${BIN_CACHE_CONFIG_NAME}.zip" ]; then
        ALREADY_BUILT="TRUE"
        unzip -q "${BIN_CACHE_PATH}/${BIN_CACHE_CONFIG_NAME}.zip" -d "${BIN_CACHE_PATH}"
        if [ ! -f "${PROVIZIO_DDS_CHECK_FILE}" ]; then
            ALREADY_BUILT="FALSE"
        fi
        rm -rf "${TARGET_PATH}"
    fi
fi

if [ "${ALREADY_BUILT}" == "TRUE" ]; then
    echo "The bin cache is already built for ${BIN_CACHE_CONFIG_NAME}"
else
    echo "Building bin cache for ${BIN_CACHE_CONFIG_NAME}..."

    # But first, let's delete any obsolete version there may be (targeted to avoid removing
    # Python caches for other Python versions that share the same base cache name)
    WILDCARD_NAME="$(./bin_cache_config_name.sh "${BUILD_TYPE}" WILDCARD)"
    # shellcheck disable=SC2086
    rm -rf "${BIN_CACHE_PATH:?}"/${WILDCARD_NAME} "${BIN_CACHE_PATH:?}"/${WILDCARD_NAME}.zip
    if [ -n "${PYTHON_VERSION_TAG}" ]; then
        WILDCARD_PYTHON_NAME="$(./bin_cache_config_name.sh "${BUILD_TYPE}" WILDCARD "${PYTHON_VERSION_TAG}")"
        # shellcheck disable=SC2086
        rm -rf "${BIN_CACHE_PATH:?}"/${WILDCARD_PYTHON_NAME} "${BIN_CACHE_PATH:?}"/${WILDCARD_PYTHON_NAME}.zip
    fi

    IGNORE_BIN_CACHE=TRUE .github/workflows/build.sh -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DSTATIC_ANALYSIS="${STATIC_ANALYSIS}" -DPYTHON_BINDINGS="${PYTHON}" -DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS="ON" -DENABLE_TESTS="OFF" -DENABLE_CHECK_FORMAT="OFF" -DCMAKE_INSTALL_PREFIX="${TARGET_PATH}" -DPYTHON_PACKAGES_INSTALL_DIR="${PYTHON_TARGET_PATH}"
    cd ./build
    cmake --install .

    # Delete extra copy of python-specific libs produced by Fast-DDS Python wrapper (as it's already included in dedicated Python subfolder)
    rm -rf "${TARGET_PATH}"/lib/python*

    # Collect all the dependencies as well
    collect_libs() {
        local binary="$1"
        local output_dir="$2"
        local lib_basename
        local lib_realpath

        # Update RUNPATH to make it look for its dependencies in the same directory or ../lib/
        if [[ "${binary}" != *libprovizio*.so* ]]; then # Provizio libs already have correct RUNPATHS
            # shellcheck disable=SC2016
            patchelf --set-rpath '$ORIGIN:$ORIGIN/../lib' "${binary}"
        fi

        # Use ldd to find shared libraries the binary depends on
        ldd "${binary}" | awk '/=>/ { print $(NF-1) }' | while read -r lib; do
            if [ -n "${lib}" ] && [ -f "${lib}" ]; then
                # Exclude system libraries
                lib_basename="$(basename "${lib}")"
                if [[ "${lib_basename}" != libc.so* && "${lib_basename}" != libm.so* && "${lib_basename}" != librt.so* && "${lib_basename}" != libpthread.so* && "${lib_basename}" != libdl.so* ]]; then
                    # Copy the library if it hasn't been copied yet
                    if [ ! -f "${output_dir}/${lib_basename}" ]; then
                        if [ -L "${lib}" ]; then
                            lib_realpath="$(realpath "${lib}")"
                            cp -a "${lib_realpath}" "${output_dir}/"
                            lib_basename="$(basename "${lib_realpath}")"
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
    if [ "${PYTHON}" == "ON" ]; then
        collect_all_libs "${TARGET_PATH}/python/fastdds"
        collect_all_libs "${TARGET_PATH}/python/provizio_dds"
        collect_all_libs "${TARGET_PATH}/python/provizio_dds_python_types"
    fi

    # Store the build machine's kernel version in the cache, as build provenance (it says nothing
    # about which hosts the binaries can run on — abi_requirements below is what decides that)
    uname -r > "${TARGET_PATH}/kernel_version"

    # Store the ABI level the binaries require of a host, so consumers can tell whether they can
    # use them without needing binutils of their own (see the bin cache section of ../../CMakeLists.txt
    # and ../../cmake/bin_cache/host_abi_compatibility.cmake).
    #
    # The requirement is the highest versioned-symbol tag ("Version needs", i.e. references such as
    # pthread_kill@GLIBC_2.34, NOT the versions the libraries define themselves) across every ELF
    # object built here - the general and the Python payloads alike, so both zips can carry the same
    # numbers (they are built by one toolchain on one machine, so they don't differ in practice).
    # The bundled libstdc++ / libgcc_s are included in the scan on purpose: they aren't installed by
    # default (DONT_INSTALL_STDCPP_LIBS), but they are the ones loaded when the cache is consumed
    # straight from a build tree, so their own glibc requirement is just as real as that of the
    # Provizio libraries.
    abi_version_needs() {
        local root="$1"
        local binary

        while IFS= read -r binary; do
            # "|| true": readelf exits non-zero on a non-ELF file (e.g. a GNU ld linker
            # script named *.so), and under set -e + pipefail that would abort the whole
            # cache build with the diagnostic suppressed by 2>/dev/null. Skipping the file
            # is safe — the empty-result checks below are the real gate.
            readelf --version-info "${binary}" 2>/dev/null | awk '
                /Version needs section/ { needs = 1; next }
                /Version (symbols|definition) section/ { needs = 0 }
                needs && match($0, /Name: [A-Za-z_]+_[0-9.]+/) { print substr($0, RSTART + 6, RLENGTH - 6) }' ||
                true
        done < <(find "${root}" -type f \( -name "*.so" -o -name "*.so.*" \))
    }

    highest_abi_version() {
        local tag="$1"
        local needs_file="$2"

        grep -E "^${tag}_[0-9]" "${needs_file}" | sed "s/^${tag}_//" | sort -V | tail -n 1
    }

    if ! command -v readelf > /dev/null; then
        echo "readelf (binutils) is required to record the ABI requirements of the bin cache!"
        exit 1
    fi

    ABI_VERSION_NEEDS_FILE="$(mktemp)"
    abi_version_needs "${TARGET_PATH}" > "${ABI_VERSION_NEEDS_FILE}"
    REQUIRED_GLIBC="$(highest_abi_version GLIBC "${ABI_VERSION_NEEDS_FILE}" || true)"
    REQUIRED_GLIBCXX="$(highest_abi_version GLIBCXX "${ABI_VERSION_NEEDS_FILE}" || true)"
    REQUIRED_CXXABI="$(highest_abi_version CXXABI "${ABI_VERSION_NEEDS_FILE}" || true)"
    rm -f "${ABI_VERSION_NEEDS_FILE}"

    if [ -z "${REQUIRED_GLIBC}" ]; then
        # Every shipped binary references glibc, so an empty result means the scan itself failed
        echo "Failed to determine the glibc version required by the binaries in ${TARGET_PATH}!"
        exit 1
    fi
    if [ -z "${REQUIRED_GLIBCXX}" ] || [ -z "${REQUIRED_CXXABI}" ]; then
        # The C++ payload always references GLIBCXX_/CXXABI_ versioned symbols, so an empty
        # result likewise means the scan failed. Recording an empty value would silently
        # disable the consumer-side libstdc++ gate: the cache would be accepted on a host
        # whose libstdc++ is too old and fail at load time instead of at configure time.
        echo "Failed to determine the libstdc++ (GLIBCXX/CXXABI) versions required by the binaries in ${TARGET_PATH}!"
        exit 1
    fi

    {
        echo "# Highest versioned-symbol levels the binaries in this cache require of a host."
        echo "glibc=${REQUIRED_GLIBC}"
        echo "glibcxx=${REQUIRED_GLIBCXX}"
        echo "cxxabi=${REQUIRED_CXXABI}"
    } > "${TARGET_PATH}/abi_requirements"
    echo "Bin cache ABI requirements: glibc ${REQUIRED_GLIBC}, GLIBCXX_${REQUIRED_GLIBCXX}, CXXABI_${REQUIRED_CXXABI}"

    cd "${BIN_CACHE_PATH}"

    if [ "${PYTHON}" == "ON" ]; then
        # Create python-versioned cache zip (python/ + metadata only)
        PYTHON_CACHE_DIR="${BIN_CACHE_PATH}/${PYTHON_CACHE_CONFIG_NAME}"
        mkdir -p "${PYTHON_CACHE_DIR}"
        mv "${TARGET_PATH}/python" "${PYTHON_CACHE_DIR}/python"
        cp "${TARGET_PATH}/kernel_version" "${PYTHON_CACHE_DIR}/kernel_version"
        cp "${TARGET_PATH}/abi_requirements" "${PYTHON_CACHE_DIR}/abi_requirements"
        zip -r -y "${PYTHON_CACHE_CONFIG_NAME}.zip" "${PYTHON_CACHE_CONFIG_NAME}"
        rm -rf "${PYTHON_CACHE_DIR}"
    fi

    # Create general cache zip (include/, lib/, metadata — no python/)
    # Skip if it already exists (e.g., when only rebuilding for a new Python version)
    if [ ! -f "${BIN_CACHE_CONFIG_NAME}.zip" ]; then
        zip -r -y "${BIN_CACHE_CONFIG_NAME}.zip" "${BIN_CACHE_CONFIG_NAME}"
    fi
    rm -rf "${TARGET_PATH}"
fi
