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

# Line-ending coverage for the Fast-DDS patch scripts in cmake/fast_dds/.
#
# Each of them edits Fast-DDS sources that git may have checked out with either line ending, and is
# itself a file git may have checked out either way -- see cmake/fast_dds/patch_io.cmake for why
# both are to be expected and how the scripts cope. A host sees whichever combination its own git
# configuration produces; this test puts every script through LF and CRLF sources, patched by the
# script as checked out and by a CRLF copy of it. Every combination must
#
#   - apply to the pristine sources, as a fresh build does;
#   - produce the same text as every other combination: a patch does not depend on the line
#     endings it met;
#   - leave each file it wrote in one line ending, the host's own (CRLF on Windows, LF
#     elsewhere) -- never doubled, never mixed;
#   - and change nothing when applied a second time.
#
# The pristine sources come from the Fast-DDS checkout's own history: HEAD is the upstream release
# the patches apply to, while the working tree is already patched by the time tests run.
# `git cat-file --filters` renders a file with whichever line ending it is asked for, byte for byte,
# on any host. Everything happens on copies under WORK_DIR; the build tree is never written to.
#
# Invoked as:
#   cmake -DGIT_EXECUTABLE=<git> -DFAST_DDS_SOURCE_DIR=<path> -DPATCH_SCRIPTS=<script|script|...>
#         -DWORK_DIR=<scratch dir> -P line_endings_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS GIT_EXECUTABLE FAST_DDS_SOURCE_DIR PATCH_SCRIPTS WORK_DIR)
    if(NOT DEFINED ${_var} OR "${${_var}}" STREQUAL "")
        message(FATAL_ERROR "line_endings_test.cmake: ${_var} must be defined")
    endif()
endforeach()

# Every patch script with the files it patches: the variable it takes each one in, and that file's
# path in the Fast-DDS sources -- as the PATCH_COMMAND in the top-level CMakeLists.txt passes them.
# The check below fails when a script applied there is missing here, so a new one cannot go
# uncovered; a path or a variable out of step with the PATCH_COMMAND fails the case itself.
set(_patches
    "export_system_info.cmake|SYSTEMINFO_HPP=src/cpp/utils/SystemInfo.hpp"
    "host_id_without_interfaces.cmake|HOST_HPP=src/cpp/utils/Host.hpp"
    "resource_event_per_timer_wait.cmake|RESOURCE_EVENT_H=src/cpp/rtps/resources/ResourceEvent.h|RESOURCE_EVENT_CPP=src/cpp/rtps/resources/ResourceEvent.cpp|WRITER_PROXY_CPP=src/cpp/rtps/reader/WriterProxy.cpp"
    "topic_payload_pool_registry_lock_first.cmake|REGISTRY_HPP=src/cpp/rtps/history/TopicPayloadPoolRegistry_impl/TopicPayloadPoolRegistry.hpp"
    "local_reader_under_writer_mutex.cmake|STATEFUL_WRITER_CPP=src/cpp/rtps/writer/StatefulWriter.cpp|STATELESS_WRITER_CPP=src/cpp/rtps/writer/StatelessWriter.cpp")

string(REPLACE "|" ";" _applied_scripts "${PATCH_SCRIPTS}")
list(GET _applied_scripts 0 _first_script)
get_filename_component(_patch_dir "${_first_script}" DIRECTORY)

# The same reading every patch script uses (it is also what classifies line endings below).
include("${_patch_dir}/patch_io.cmake")

set(_covered)
foreach(_entry IN LISTS _patches)
    string(REPLACE "|" ";" _fields "${_entry}")
    list(GET _fields 0 _script)
    list(APPEND _covered "${_script}")
endforeach()
set(_applied)
foreach(_script_path IN LISTS _applied_scripts)
    get_filename_component(_script "${_script_path}" NAME)
    list(APPEND _applied "${_script}")
    if(NOT _script IN_LIST _covered)
        message(FATAL_ERROR
            "line_endings_test: ${_script} is applied to Fast-DDS but not covered here. Add it to "
            "_patches with the files it patches, as its PATCH_COMMAND entry passes them.")
    endif()
endforeach()
foreach(_script IN LISTS _covered)
    if(NOT _script IN_LIST _applied)
        message(FATAL_ERROR
            "line_endings_test: _patches lists ${_script}, which PROVIZIO_DDS_FAST_DDS_PATCH_SCRIPTS "
            "does not: remove it here, or list it there if the PATCH_COMMAND still runs it.")
    endif()
endforeach()

if(CMAKE_HOST_WIN32)
    set(_host_line_ending "crlf")
else()
    set(_host_line_ending "lf")
endif()

# Set <out_var> to "lf" or "crlf" when every line of <path> ends that way, or to "mixed": a doubled
# CR, or some lines of each. Decided by size against the LF text: equal when no line ends with a CR,
# one byte per line more when every line ends with exactly one.
function(_line_ending_of path out_var)
    provizio_dds_patch_read("${path}" _text)
    file(SIZE "${path}" _size)
    string(LENGTH "${_text}" _length)
    string(REPLACE "\n" "" _without_lf "${_text}")
    string(LENGTH "${_without_lf}" _length_without_lf)
    math(EXPR _crlf_size "2 * ${_length} - ${_length_without_lf}")
    if(_size EQUAL _length)
        set(${out_var} "lf" PARENT_SCOPE)
    elseif(_size EQUAL _crlf_size)
        set(${out_var} "crlf" PARENT_SCOPE)
    else()
        set(${out_var} "mixed" PARENT_SCOPE)
    endif()
endfunction()

function(_expect_line_ending path expected what)
    _line_ending_of("${path}" _actual)
    if(NOT _actual STREQUAL expected)
        message(FATAL_ERROR "line_endings_test: ${what}: ${path} has ${_actual} line endings, not ${expected}")
    endif()
endfunction()

# Render <relative> as the checkout's HEAD has it, with <line_ending> line endings, to <destination>.
function(_render_pristine relative line_ending destination)
    if(line_ending STREQUAL "crlf")
        set(_config -c core.autocrlf=true)
    else()
        set(_config -c core.autocrlf=false -c core.eol=lf)
    endif()
    get_filename_component(_directory "${destination}" DIRECTORY)
    file(MAKE_DIRECTORY "${_directory}")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -c "safe.directory=*" ${_config} -C "${FAST_DDS_SOURCE_DIR}"
                cat-file --filters "HEAD:${relative}"
        OUTPUT_FILE "${destination}"
        RESULT_VARIABLE _result
        ERROR_VARIABLE _error)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR
            "line_endings_test: could not read the pristine ${relative} from the Fast-DDS checkout at "
            "${FAST_DDS_SOURCE_DIR}: ${_error}")
    endif()
    # Otherwise every comparison below would compare one line ending with itself.
    _expect_line_ending("${destination}" "${line_ending}" "rendering the pristine source")
endfunction()

# Write <text> to <destination> with CRLF line endings, on any host: file(WRITE) already produces
# them on Windows, and writes CRLF text as given everywhere else.
function(_write_crlf text destination)
    if(CMAKE_HOST_WIN32)
        file(WRITE "${destination}" "${text}")
    else()
        string(REPLACE "\n" "\r\n" _crlf_text "${text}")
        file(WRITE "${destination}" "${_crlf_text}")
    endif()
    _expect_line_ending("${destination}" "crlf" "writing a CRLF copy")
endfunction()

file(REMOVE_RECURSE "${WORK_DIR}")

# The pristine sources, once per line ending. Laid out flat, by file name -- no two patched files
# share one -- rather than at their paths in the Fast-DDS tree, which under a deep enough build
# directory would exceed the 260 characters Windows allows a path.
foreach(_line_ending IN ITEMS lf crlf)
    foreach(_entry IN LISTS _patches)
        string(REPLACE "|" ";" _fields "${_entry}")
        list(REMOVE_AT _fields 0)
        foreach(_file IN LISTS _fields)
            string(REGEX REPLACE "^[A-Z_]+=" "" _relative "${_file}")
            get_filename_component(_leaf "${_relative}" NAME)
            _render_pristine("${_relative}" "${_line_ending}" "${WORK_DIR}/pristine/${_line_ending}/${_leaf}")
        endforeach()
    endforeach()
endforeach()

# CRLF copies of the scripts and of the helper they include, side by side as in cmake/fast_dds.
set(_crlf_script_dir "${WORK_DIR}/crlf_scripts")
foreach(_script IN LISTS _covered ITEMS patch_io.cmake)
    provizio_dds_patch_read("${_patch_dir}/${_script}" _script_text)
    _write_crlf("${_script_text}" "${_crlf_script_dir}/${_script}")
endforeach()

set(_combinations 0)
foreach(_entry IN LISTS _patches)
    string(REPLACE "|" ";" _fields "${_entry}")
    list(GET _fields 0 _script)
    list(REMOVE_AT _fields 0)

    set(_have_reference FALSE)
    foreach(_script_form IN ITEMS as_checked_out crlf)
        if(_script_form STREQUAL "crlf")
            set(_script_path "${_crlf_script_dir}/${_script}")
        else()
            set(_script_path "${_patch_dir}/${_script}")
        endif()
        foreach(_line_ending IN ITEMS lf crlf)
            set(_case "${_script} (${_script_form} script, ${_line_ending} sources)")
            # Numbered rather than named after the case, for the same path-length reason as above.
            set(_run_dir "${WORK_DIR}/runs/${_combinations}")
            set(_args)
            set(_outputs)
            foreach(_file IN LISTS _fields)
                string(REGEX MATCH "^[A-Z_]+" _variable "${_file}")
                string(REGEX REPLACE "^[A-Z_]+=" "" _relative "${_file}")
                get_filename_component(_leaf "${_relative}" NAME)
                configure_file("${WORK_DIR}/pristine/${_line_ending}/${_leaf}" "${_run_dir}/${_leaf}" COPYONLY)
                list(APPEND _args "-D${_variable}=${_run_dir}/${_leaf}")
                list(APPEND _outputs "${_leaf}")
            endforeach()

            execute_process(COMMAND "${CMAKE_COMMAND}" ${_args} -P "${_script_path}"
                            RESULT_VARIABLE _result OUTPUT_VARIABLE _stdout ERROR_VARIABLE _stderr)
            if(NOT _result EQUAL 0)
                message(FATAL_ERROR "line_endings_test: ${_case} failed:\n${_stdout}${_stderr}")
            endif()

            foreach(_leaf IN LISTS _outputs)
                set(_output "${_run_dir}/${_leaf}")

                _expect_line_ending("${_output}" "${_host_line_ending}" "${_case} wrote")
                provizio_dds_patch_read("${_output}" _patched_text)
                provizio_dds_patch_read("${WORK_DIR}/pristine/${_line_ending}/${_leaf}" _pristine_text)
                string(COMPARE EQUAL "${_patched_text}" "${_pristine_text}" _unchanged)
                if(_unchanged)
                    message(FATAL_ERROR "line_endings_test: ${_case} left ${_leaf} unpatched")
                endif()
                if(_have_reference)
                    string(COMPARE EQUAL "${_patched_text}" "${_reference_${_script}_${_leaf}}" _same)
                    if(NOT _same)
                        message(FATAL_ERROR
                            "line_endings_test: ${_case} patched ${_leaf} differently from the same script "
                            "on LF sources: a patch must not depend on the line endings it meets.")
                    endif()
                else()
                    set(_reference_${_script}_${_leaf} "${_patched_text}")
                endif()
                file(SHA256 "${_output}" _hash_${_leaf})
            endforeach()
            set(_have_reference TRUE)

            # A second application finds everything patched and must leave every byte alone.
            execute_process(COMMAND "${CMAKE_COMMAND}" ${_args} -P "${_script_path}"
                            RESULT_VARIABLE _result OUTPUT_VARIABLE _stdout ERROR_VARIABLE _stderr)
            if(NOT _result EQUAL 0)
                message(FATAL_ERROR "line_endings_test: applying ${_case} a second time failed:\n${_stdout}${_stderr}")
            endif()
            foreach(_leaf IN LISTS _outputs)
                file(SHA256 "${_run_dir}/${_leaf}" _hash_again)
                if(NOT _hash_again STREQUAL _hash_${_leaf})
                    message(FATAL_ERROR
                        "line_endings_test: applying ${_case} a second time rewrote ${_leaf}; an already "
                        "patched file must be left as it is.")
                endif()
            endforeach()
            math(EXPR _combinations "${_combinations} + 1")
        endforeach()
    endforeach()
endforeach()

file(REMOVE_RECURSE "${WORK_DIR}")
list(LENGTH _patches _script_count)
message(STATUS "line_endings: PASS (${_script_count} patch scripts, ${_combinations} combinations of script and "
               "source line endings, each written with ${_host_line_ending} line endings and idempotent)")
