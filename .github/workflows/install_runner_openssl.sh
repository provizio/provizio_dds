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


# Installs the OpenSSL 3 the aarch64 prebuilt binaries are built against into /opt/openssl-3 of a
# self-hosted jetson-20.04 runner. Run as root, once per runner, and again to move a runner to the
# version pinned below (the CI jobs fail on a runner that holds another one):
#   sudo .github/workflows/install_runner_openssl.sh
#
# Why a private prefix: Ubuntu 20.04 ships OpenSSL 1.1.1, which the prebuilt binaries used to carry,
# and there is no supported package of a newer one. Replacing the system's would break everything on
# the runner linked against libssl.so.1.1 (apt, curl, git, Python's ssl module, the runner agent),
# as 3.x has another SONAME and ABI. Under /opt it sits beside the system's: the CI jobs point
# OPENSSL_ROOT_DIR at it, and the build copies its shared libraries into the binaries' lib/provizio_dds/.
#
# 3.5 is the LTS series (supported until April 2030): the prebuilt binaries ship this OpenSSL to
# customers, so one that keeps receiving fixes outweighs the newest series. 3.0 is past its end of life.

set -eu
set -o pipefail

OPENSSL_VERSION="3.5.9"
# From the release's own openssl-${OPENSSL_VERSION}.tar.gz.sha256 asset
OPENSSL_SHA256="603f5602e2eef00d77fbd429d34dcd5822bb301757a1bc9cdb24c670f1eb859a"
PREFIX="/opt/openssl-3"

if [ "$(id -u)" -ne 0 ]; then
  echo "Run as root: the OpenSSL goes to ${PREFIX}"
  exit 1
fi

# opensslv.h, not "openssl version": the libraries carry no RUNPATH (see below), so the binary does not run
installed_version() {
  awk -F'"' '/^# *define +OPENSSL_FULL_VERSION_STR/ { print $2 }' "${PREFIX}/include/openssl/opensslv.h" 2>/dev/null || true
}

if [ "$(installed_version)" == "${OPENSSL_VERSION}" ] && [ -f "${PREFIX}/lib/libssl.so.3" ] && [ -f "${PREFIX}/lib/libcrypto.so.3" ]; then
  echo "OpenSSL ${OPENSSL_VERSION} is already installed in ${PREFIX}"
  exit 0
fi

WORK_DIR="$(mktemp -d)"
trap 'rm -rf "${WORK_DIR}"' EXIT
cd "${WORK_DIR}"

curl --fail --silent --show-error --location --retry 5 \
  --output openssl.tar.gz \
  "https://github.com/openssl/openssl/releases/download/openssl-${OPENSSL_VERSION}/openssl-${OPENSSL_VERSION}.tar.gz"
echo "${OPENSSL_SHA256}  openssl.tar.gz" | sha256sum --check --strict -
tar -xzf openssl.tar.gz
cd "openssl-${OPENSSL_VERSION}"

# --openssldir is what Ubuntu's own OpenSSL uses, so the CA store and configuration are looked up
# where a customer's Ubuntu host has them. Only "install_sw" is run, so nothing is written there.
# --libdir=lib: OpenSSL 3 defaults to lib64 on 64-bit Linux.
# No -rpath: these libraries are copied into the prebuilt binaries, whose RUNPATH is set by
# build_cache.sh; a baked-in /opt/openssl-3/lib would ship with them, to hosts that have no such directory.
./Configure linux-aarch64 shared \
  --prefix="${PREFIX}" \
  --openssldir=/usr/lib/ssl \
  --libdir=lib
nice make -j"$(nproc)"

# Replace the previous installation as a whole, so no file of an earlier version is left behind
STAGING="${PREFIX}.new"
rm -rf "${STAGING}"
make install_sw DESTDIR="${WORK_DIR}/stage"
mv "${WORK_DIR}/stage${PREFIX}" "${STAGING}"
rm -rf "${PREFIX}.old"
if [ -e "${PREFIX}" ]; then
  mv "${PREFIX}" "${PREFIX}.old"
fi
mv "${STAGING}" "${PREFIX}"
rm -rf "${PREFIX}.old"

echo "Installed OpenSSL $(installed_version) in ${PREFIX}"
