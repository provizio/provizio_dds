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

# What the python/ directory of a Python bin cache must hold for the install of it to produce a
# package that imports: the modules, the extension modules, and the libraries they load, Fast-DDS's and
# the OpenSSL it loads bundled under their versioned names among them (a glob pattern, see
# missing_file.cmake). A cache
# truncated or malformed extracts all the same, and the install copies whatever python/ holds, so
# without this a package that cannot import is built where a build from source would serve.
#
# The same list setup.py checks for a pip package (BIN_CACHE_PYTHON_REQUIRED there, one entry per
# file with its spelling on each platform); test/source_text/bin_cache_python_required_parity.py
# keeps the two in step, so a file added to one cannot be forgotten in the other.

# Linux (the extension modules of a bin cache are shared objects)
set(PROVIZIO_DDS_BIN_CACHE_PYTHON_REQUIRED_LINUX
    version.txt
    fastdds/fastdds.py
    fastdds/_fastdds_python.so
    provizio_dds/provizio_dds.py
    provizio_dds/libprovizio_dds.so
    "provizio_dds/libfastdds.so.*"
    "provizio_dds/libfastcdr.so.*"
    "provizio_dds/libssl.so.*"
    "provizio_dds/libcrypto.so.*"
    provizio_dds_python_types/provizio_dds_python_types.py
    provizio_dds_python_types/_provizio_dds_python_types.so
    provizio_dds_python_types/libprovizio_dds_types.so)

# Windows (extension modules are .pyd, libraries .dll)
set(PROVIZIO_DDS_BIN_CACHE_PYTHON_REQUIRED_WINDOWS
    version.txt
    fastdds/fastdds.py
    fastdds/_fastdds_python.pyd
    provizio_dds/provizio_dds.py
    provizio_dds/provizio_dds.dll
    "provizio_dds/fastdds*.dll"
    "provizio_dds/fastcdr*.dll"
    "provizio_dds/libssl-3*.dll"
    "provizio_dds/libcrypto-3*.dll"
    provizio_dds_python_types/provizio_dds_python_types.py
    provizio_dds_python_types/_provizio_dds_python_types.pyd
    provizio_dds_python_types/provizio_dds_types.dll)
