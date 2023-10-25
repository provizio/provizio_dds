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

set -e

cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)

source ./python_venv.sh

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

# Build and install the package
pip3 install -v .

# Test it works fine by executing Python tests directly (without copying provizio_dds.py and other beside the tests)
python3 test/python/python_publisher.py & python3 test/python/python_subscriber.py
python3 test/python/pointcloud2_test.py
