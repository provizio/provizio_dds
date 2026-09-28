# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

# Tests for the build-time Fast-DDS patches and steps in cmake/fast_dds/. They exercise Fast-DDS'
# own internals, so they only make sense where this project BUILT Fast-DDS and therefore
# applied those patches: LOOK_FOR_FAST_DDS=TRUE (a system Fast-DDS) and the prebuilt bin
# cache both skip the source build, and FAST_DDS_SOURCE_DIR is unset in exactly those cases.
#
# That variable is the whole condition, deliberately. The sources it names do not exist yet when
# a fresh tree is configured -- ExternalProject downloads them during the build, before anything
# here compiles against them -- so asking whether a file in them exists would skip every test
# below on the single configure a CI job runs.
if(NOT FAST_DDS_SOURCE_DIR)
    message(STATUS "Fast-DDS was not built from source here; skipping its patch tests")
    return()
endif()

# Migration coverage for the timer patch script itself: pure CMake script mode, so unlike the
# runtime test below it needs no target and runs everywhere Fast-DDS was built from source --
# Windows included, where the incremental-build failure it guards against is just as reachable.
add_test(NAME fast_dds_patch_timer_migration
         COMMAND "${CMAKE_COMMAND}"
                 "-DPATCH_SCRIPT=${PROJECT_SOURCE_DIR}/cmake/fast_dds/resource_event_per_timer_wait.cmake"
                 "-DFAST_DDS_SOURCE_DIR=${FAST_DDS_SOURCE_DIR}"
                 "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/timer_patch_migration"
                 -P "${CMAKE_CURRENT_LIST_DIR}/timer_patch_migration_test.cmake")
# Seconds. It copies three files a few times and runs CMake in script mode; the ceiling only
# has to outlast a loaded CI runner.
set_tests_properties(fast_dds_patch_timer_migration PROPERTIES TIMEOUT 60)

# Coverage for the step placing the OpenSSL runtime Fast-DDS loads next to it
# (cmake/fast_dds/openssl_runtime.cmake): pure CMake script mode as well, with stand-ins for the
# tool reading a library's imports and for those changing and signing one, so every platform's
# case runs on any of them.
add_test(NAME fast_dds_openssl_runtime
         COMMAND "${CMAKE_COMMAND}"
                 "-DRUNTIME_SCRIPT=${PROJECT_SOURCE_DIR}/cmake/fast_dds/openssl_runtime.cmake"
                 "-DFAKE_TOOL=${CMAKE_CURRENT_LIST_DIR}/openssl_runtime_fake_tool.cmake"
                 "-DFAKE_COMMAND=${CMAKE_CURRENT_LIST_DIR}/openssl_runtime_fake_command.cmake"
                 "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/openssl_runtime"
                 -P "${CMAKE_CURRENT_LIST_DIR}/openssl_runtime_test.cmake")
# Seconds. It writes a few small files and runs CMake in script mode a few dozen times.
set_tests_properties(fast_dds_openssl_runtime PROPERTIES TIMEOUT 60)

# Line-ending coverage for every patch script (see cmake/fast_dds/patch_io.cmake): pure CMake
# script mode as well, applying each to LF and CRLF renderings of the pristine sources, with the
# script as checked out and as a CRLF copy. It runs everywhere Fast-DDS is built from source --
# Windows above all, where file(WRITE) writes CRLF and a CRLF checkout of either repository is the
# likeliest. It takes the patches from PROVIZIO_DDS_FAST_DDS_PATCHES (top-level CMakeLists.txt),
# the one list the PATCH_COMMAND is made from, so every script the build applies is covered. Handed
# over as a file, one entry per line: an entry holds the separators a command line would split it at.
find_package(Git REQUIRED)
set(_fast_dds_patches_file "${CMAKE_CURRENT_BINARY_DIR}/fast_dds_patches.txt")
string(REPLACE ";" "\n" _fast_dds_patches "${PROVIZIO_DDS_FAST_DDS_PATCHES}")
file(WRITE "${_fast_dds_patches_file}" "${_fast_dds_patches}\n")
add_test(NAME fast_dds_patch_line_endings
         COMMAND "${CMAKE_COMMAND}"
                 "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
                 "-DFAST_DDS_SOURCE_DIR=${FAST_DDS_SOURCE_DIR}"
                 "-DPATCH_DIR=${PROJECT_SOURCE_DIR}/cmake/fast_dds"
                 "-DPATCHES_FILE=${_fast_dds_patches_file}"
                 "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/line_endings"
                 -P "${CMAKE_CURRENT_LIST_DIR}/line_endings_test.cmake")
# Seconds. It renders eighteen small files with git and runs CMake in script mode forty-eight times.
set_tests_properties(fast_dds_patch_line_endings PROPERTIES TIMEOUT 120)

# Not on Windows, where these tests CANNOT be linked. ResourceEvent and TimedEventImpl, like
# StatefulWriter and RTPSDomainImpl, are Fast-DDS internals with no export annotations, and
# Fast-DDS does not auto-export every symbol -- verified against the shipped fastdds-3.6.lib,
# whose only ResourceEvent-related export is RTPSParticipant::get_resource_event();
# register_timer, unregister_timer, init_thread, go_ready and update_interval_millisec appear
# nowhere in it. A private include directory supplies declarations, not DLL exports, so the
# targets would fail to link with unresolved externals in every tests-enabled Windows build.
# Auto-exporting all of Fast-DDS is not the answer either -- it exceeds Windows' 65535-symbol
# limit per DLL, which is precisely why export_system_info.cmake exports one function by hand.
#
# The patched code itself is platform-independent, and the sanitizers that turn its regression
# into a hard failure rather than silent corruption only run on Linux, so this costs no real
# coverage.
if(WIN32)
    message(STATUS "Fast-DDS patch tests need its unexported internals; skipping on Windows")
    return()
endif()

add_executable(timer_unregister_test timer_unregister_test.cpp)
find_package(Threads REQUIRED)
target_link_libraries(timer_unregister_test PUBLIC provizio_dds Threads::Threads)
# Fast-DDS' private sources, which is where the patched ResourceEvent lives. Deliberate --
# see the test's own header comment.
target_include_directories(timer_unregister_test SYSTEM PRIVATE "${FAST_DDS_SOURCE_DIR}/src/cpp")

# Seconds. The case is deterministic and sub-second in the good path; the ceiling only has to
# outlast a loaded CI runner, and a regression shows up as an immediate sanitizer abort or as
# the deadlock the patch exists to prevent (which the case fails explicitly rather than
# hanging out its whole budget).
add_test(NAME fast_dds_patch_timer_unregister
         COMMAND "$<TARGET_FILE:timer_unregister_test>"
         WORKING_DIRECTORY "$<TARGET_FILE_DIR:timer_unregister_test>")
set_tests_properties(fast_dds_patch_timer_unregister PROPERTIES TIMEOUT 60)

# Guards local_reader_under_writer_mutex.cmake: forces the order in which an intraprocess
# heartbeat that read its reader without the writer mutex delivers to a reader removed meanwhile.
add_executable(local_reader_lock_test local_reader_lock_test.cpp)
target_link_libraries(local_reader_lock_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)
# Fast-DDS' private sources, which is where the patched StatefulWriter lives. Deliberate -- see
# the test's own header comment.
target_include_directories(local_reader_lock_test SYSTEM PRIVATE "${FAST_DDS_SOURCE_DIR}/src/cpp")

# Seconds. The case takes about half a second on Linux, where it waits for the heartbeat to wait for
# the writer mutex, and about a second elsewhere, most of it the half second it lets the heartbeat
# reach that mutex; the ceiling only has to outlast a loaded CI runner, and a heartbeat that never
# returns fails the case explicitly rather than hanging out its whole budget.
add_test(NAME fast_dds_patch_local_reader_lock
         COMMAND "$<TARGET_FILE:local_reader_lock_test>"
         WORKING_DIRECTORY "$<TARGET_FILE_DIR:local_reader_lock_test>")
set_tests_properties(fast_dds_patch_local_reader_lock PROPERTIES TIMEOUT 60)

provizio_dds_finalize_tests()
