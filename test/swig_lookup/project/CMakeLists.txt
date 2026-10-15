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

# Stands in for provizio_dds in the swig_lookup test: looks SWIG up as provizio_dds does, and writes
# what it found to OUT_FILE, as <executable>|<version>|<library directory>. Configured by
# swig_lookup_test.cmake, never built.

cmake_minimum_required(VERSION 3.15)
project(swig_lookup_stand_in LANGUAGES NONE)

include("${REPOSITORY}/cmake/python_bindings/swig_lookup.cmake")
provizio_dds_find_swig()
file(WRITE "${OUT_FILE}" "${SWIG_EXECUTABLE}|${SWIG_VERSION}|${SWIG_DIR}")
