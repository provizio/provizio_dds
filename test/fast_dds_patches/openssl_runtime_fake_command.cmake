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

# Stands in for install_name_tool and codesign in the fast_dds_openssl_runtime test: appends to
# FAKE_LOG one line, FAKE_NAME and then the arguments it was given, each after a space. With
# FAKE_VERIFY_ERROR, a --verify fails, saying that.
#
#   cmake -DFAKE_LOG=<file> -DFAKE_NAME=<name> [-DFAKE_VERIFY_ERROR=<text>]
#         -P openssl_runtime_fake_command.cmake -- <arguments>
#
# The -- keeps CMake from taking the arguments for its own options.

set(_line "${FAKE_NAME}")
set(_after_separator FALSE)
set(_first)
math(EXPR _last "${CMAKE_ARGC} - 1")
foreach(_index RANGE 1 ${_last})
    if(_after_separator)
        string(APPEND _line " ${CMAKE_ARGV${_index}}")
        if(NOT DEFINED _first)
            set(_first "${CMAKE_ARGV${_index}}")
        endif()
    elseif(CMAKE_ARGV${_index} STREQUAL "--")
        set(_after_separator TRUE)
    endif()
endforeach()
file(APPEND "${FAKE_LOG}" "${_line}\n")
if(FAKE_VERIFY_ERROR AND _first STREQUAL "--verify")
    message(FATAL_ERROR "${FAKE_VERIFY_ERROR}")
endif()
