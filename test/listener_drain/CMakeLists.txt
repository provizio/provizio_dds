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

# Subcommand-driven test exe for the listener detach + drain primitive: that the drain
# waits for an in-flight callback, that a callback which does not return reports itself
# for as long as it does not, and that an ordinary drain stays silent. Each subcommand is
# its own ctest entry so per-case failure stays isolated.
add_executable(listener_drain_test listener_drain_test.cpp)
# Threads::Threads because every case drives the drain from a second thread -- there is no
# other way to have a callback in flight while the drain waits for it.
find_package(Threads REQUIRED)
target_link_libraries(listener_drain_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)

# No DDS entity is created and no traffic flows: every case is threads, a condition
# variable and the log callback, so the ceiling only tolerates a loaded runner.
if(WIN32)
    set(_DRAIN_TIMEOUT 90)
else()
    set(_DRAIN_TIMEOUT 60)
endif()

foreach(_case blocks_until_callbacks_return reports_a_stall_and_its_end quiet_when_nothing_stalls
        detached_callbacks_are_still_awaited)
    provizio_dds_add_subcommand_test(listener_drain_${_case}
        TARGET listener_drain_test
        ARGS ${_case}
        TIMEOUT ${_DRAIN_TIMEOUT}
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:listener_drain_test>")
endforeach()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
