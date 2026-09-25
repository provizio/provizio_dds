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

# Stands in for dumpbin, readelf and otool in the fast_dds_openssl_runtime test: prints what
# FAKE_IMPORTS_DIR holds for the file its last argument names, <file name>.txt, being what the real
# tool prints for that file. The options before it are the real tool's, and ignored.
#
#   cmake -DFAKE_IMPORTS_DIR=<directory> -P openssl_runtime_fake_tool.cmake -- <options> <file>
#
# The -- keeps CMake from taking the tool's options for its own (it would otherwise take otool's -L).

math(EXPR _last "${CMAKE_ARGC} - 1")
get_filename_component(_name "${CMAKE_ARGV${_last}}" NAME)
if(NOT EXISTS "${FAKE_IMPORTS_DIR}/${_name}.txt")
    message(FATAL_ERROR "openssl_runtime_fake_tool.cmake: no output given for ${_name}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E cat "${FAKE_IMPORTS_DIR}/${_name}.txt" RESULT_VARIABLE _result)
if(NOT _result EQUAL 0)
    message(FATAL_ERROR "openssl_runtime_fake_tool.cmake: could not print ${FAKE_IMPORTS_DIR}/${_name}.txt")
endif()
