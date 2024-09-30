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
# bin_cache_config_name.sh [<BUILD_TYPE>] [<PROVIZIO_DDS_IDLS_VERSION>/WILDCARD]

set -eu
set -o pipefail

if [ "${OSTYPE}" != "linux-gnu" ]; then
  echo "Only Linux is currently supported for bin cache"
  exit 1
fi

cd "$(cd "$(dirname "$0")" && pwd -P)"

BUILD_TYPE="${1:-Release}"
PROVIZIO_DDS_IDLS_VERSION="${2:-"$(grep "set(PROVIZIO_DDS_IDLS_VERSION" ./CMakeLists.txt | awk '{print $2}' | tr -d '"')"}"

CPU_ARCH="$(uname -i)"

if [ "${PROVIZIO_DDS_IDLS_VERSION}" == "WILDCARD" ]; then
  # Used when cleaning up obsolete bin cache
  PROVIZIO_DDS_CONTENTS_HASH="*"
  IDLS_COMMIT_HASH="*"
else
  IDLS_COMMIT_HASH="$(git ls-remote https://github.com/provizio/provizio_dds_idls.git | grep -w "${PROVIZIO_DDS_IDLS_VERSION}" | awk '{print $1}')"

  # sha256 has of all non-ignored files in this repo except "media" and "cache" directories 
  PROVIZIO_DDS_CONTENTS_HASH="$(git ls-files | grep -wv media | grep -wv cache | xargs -d '\n' cat | sha256sum | awk '{print $1}')"
fi

echo "linux_${CPU_ARCH}.${PROVIZIO_DDS_CONTENTS_HASH}.idls_${IDLS_COMMIT_HASH}.${BUILD_TYPE}"
