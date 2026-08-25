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

# Subcommand-driven test exe for the dead-owner shared-memory sweep: what it
# reclaims, what it must never touch, its age guard, its kill switch, its rate
# limit, and that creating a participant triggers it. Each subcommand is its own
# ctest entry so per-case failure stays isolated — same pattern as the
# transport_tuning and discovery_tuning tests.
#
# Linux only. Shared memory is provizio_dds's default transport there and nowhere
# else (Windows and macOS force UDP over a Boost.Interprocess cleanup bug), so no
# other platform creates the files this reclaims — and the test drives POSIX
# flock / /dev/shm directly.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "shm_cleanup tests are Linux-only; skipping on ${CMAKE_SYSTEM_NAME}")
    return()
endif()

add_executable(shm_cleanup_test shm_cleanup_test.cpp)
# Threads::Threads because the spares_the_living case creates a subscriber, whose deferred
# match-publisher build is a header-only std::async instantiated into this translation unit —
# so the test object itself references pthread_create. Needed explicitly on the glibc versions
# that still ship libpthread separately (jetson-20.04); newer ones merged it into libc, which
# is why this links without it on a modern host. Same as every other test target that spawns
# a thread (discovery_tuning, accumulation, the request_response services).
find_package(Threads REQUIRED)
target_link_libraries(shm_cleanup_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)
# The directory-taking entry point the hermetic cases drive is deliberately NOT part of the
# installed headers (it unlinks files under a caller-chosen directory), so reach it through
# the library's own private header.
target_include_directories(shm_cleanup_test PRIVATE "${CMAKE_SOURCE_DIR}/src")

# Cases that only lay out files and sweep them return near-instantly; the two that
# fork a real participant (and, for spares_the_living, wait for samples to flow over
# shared memory across a sweep) need discovery time. Both ceilings just tolerate
# loaded CI runners.
set(_SC_FAST_TIMEOUT 60)
set(_SC_PARTICIPANT_TIMEOUT 120)

# Cases that lay their fixtures out in a directory of their own are hermetic: no other
# process on the host sweeps a subdirectory of /dev/shm, so they can run alongside anything.
foreach(_case reclaims_synthetic age_guard kill_switch leaves_other_files_alone rejects_planted_lock_files)
    add_test(NAME shm_cleanup_${_case}
             COMMAND "$<TARGET_FILE:shm_cleanup_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:shm_cleanup_test>")
    set_tests_properties(shm_cleanup_${_case} PROPERTIES TIMEOUT ${_SC_FAST_TIMEOUT})
endforeach()

# The rest must work in the real /dev/shm — either because they drive a real participant, or
# because they drive the production entry point, which always sweeps the platform directory.
# They lower the age guard to reach their own fixtures and would therefore happily reclaim
# each other's; other suites cannot interfere, since their sweeps use the default guard and
# these fixtures are deliberately kept younger than it. PROVIZIO_DDS_SHM_CLEANUP is the
# shared resource, and these run in seconds, so serialising them costs nothing. The Python
# mirror (test/python/CMakeLists.txt) takes the same "provizio_dds_shm_cleanup" lock, so the
# two suites never overlap either.
add_test(NAME shm_cleanup_rate_limited
         COMMAND "$<TARGET_FILE:shm_cleanup_test>" rate_limited
         WORKING_DIRECTORY "$<TARGET_FILE_DIR:shm_cleanup_test>")
set_tests_properties(shm_cleanup_rate_limited PROPERTIES TIMEOUT ${_SC_FAST_TIMEOUT}
                     RESOURCE_LOCK provizio_dds_shm_cleanup)

foreach(_case reclaims_dead_participant spares_the_living)
    add_test(NAME shm_cleanup_${_case}
             COMMAND "$<TARGET_FILE:shm_cleanup_test>" ${_case}
             WORKING_DIRECTORY "$<TARGET_FILE_DIR:shm_cleanup_test>")
    set_tests_properties(shm_cleanup_${_case} PROPERTIES TIMEOUT ${_SC_PARTICIPANT_TIMEOUT}
                         RESOURCE_LOCK provizio_dds_shm_cleanup)
endforeach()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
