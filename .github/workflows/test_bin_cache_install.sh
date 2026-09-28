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

# A C++ consumer's install from the prebuilt binaries: the plain Release configure of an unmodified
# checkout, on the commit CI publishes the binaries in, then an install into a prefix of its own.
# Checks that the configure took the prebuilt binaries, that the install kept the OpenSSL runtime
# they carry out of lib/ -- in lib/provizio_dds, where no other program looks (see
# PROVIZIO_DDS_PRIVATE_LIB_DIR in the top-level CMakeLists.txt) -- and that the Fast-DDS installed
# loads everything it needs, OpenSSL from there.
#
# Use as: test_bin_cache_install.sh (from anywhere; builds in <repository>/build)

set -eu
set -o pipefail

cd "$(cd "$(dirname "$0")" && pwd -P)/../.."

fail() {
    echo "::error::$*"
    exit 1
}

CONFIGURE_LOG="$(mktemp)"
PREFIX="$(mktemp -d)/prefix"
trap 'rm -rf "${CONFIGURE_LOG}" "$(dirname "${PREFIX}")"' EXIT

cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON 2>&1 |
    tee "${CONFIGURE_LOG}"
grep -q "Prebuilt binaries are located and will be used" "${CONFIGURE_LOG}" ||
    fail "The configure did not take the prebuilt binaries, so their install cannot be checked"

cmake --install build --prefix "${PREFIX}"

PRIVATE="${PREFIX}/lib/provizio_dds"
shopt -s nullglob
private_openssl=("${PRIVATE}"/libssl.so* "${PRIVATE}"/libcrypto.so*)
public_openssl=("${PREFIX}"/lib/libssl* "${PREFIX}"/lib/libcrypto*)
fastdds=("${PREFIX}"/lib/libfastdds.so*)
shopt -u nullglob

[ "${#private_openssl[@]}" -ge 2 ] ||
    fail "The install holds no OpenSSL runtime in lib/provizio_dds, though the prebuilt binaries carry one"
[ "${#public_openssl[@]}" -eq 0 ] ||
    fail "The install put OpenSSL in lib/ itself, where every other program would load it: ${public_openssl[*]}"
[ "${#fastdds[@]}" -ge 1 ] || fail "The install holds no libfastdds"
echo "OpenSSL runtime installed apart from lib/: ${private_openssl[*]}"

for library in "${fastdds[@]}"; do
    if [ -L "${library}" ]; then
        continue
    fi
    # No LD_LIBRARY_PATH: what the library finds by itself, through its RUNPATH
    resolved="$(env -u LD_LIBRARY_PATH ldd "${library}")"
    echo "${resolved}"
    if grep -q "not found" <<<"${resolved}"; then
        fail "$(basename "${library}") does not find everything it loads"
    fi
    for openssl in libssl libcrypto; do
        grep -qE "^[[:space:]]*${openssl}\.so[^ ]* => ${PRIVATE}/${openssl}\.so" <<<"${resolved}" ||
            fail "$(basename "${library}") does not load ${openssl} from lib/provizio_dds"
    done
done
echo "Verified: the installed Fast-DDS loads the OpenSSL runtime from lib/provizio_dds"
