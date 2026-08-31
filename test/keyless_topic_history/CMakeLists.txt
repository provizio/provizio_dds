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

# Subcommand-driven test exe for the per-type KEEP_LAST history depths on a keyless topic
# several emitters share -- the shape of every Provizio fleet topic. One case runs the
# failing scenario end to end on the defaults and asserts nothing is lost; the other two
# measure what an emitter's history actually holds, by letting a consumer join after the
# burst and collecting the backlog -- the whole burst on the per-type default, one sample
# per emitter when held to a single slot. Each subcommand is its own ctest entry so
# per-case failure stays isolated, and because each turns off Fast-DDS's process-wide
# intraprocess delivery.
add_executable(keyless_topic_history_test keyless_topic_history_test.cpp)
# Threads::Threads because the cases run their emitters on threads of their own. Needed
# EXPLICITLY even though the library already uses threads: on glibc 2.34+ the pthread
# symbols were folded into libc, so a host toolchain links this without complaint, while
# aarch64 Ubuntu 20.04 (glibc 2.31 -- the jetson devices and their CI runners) still keeps
# them in libpthread and fails with "undefined reference to pthread_create" / "DSO missing
# from command line". The siblings here all declare it for the same reason.
find_package(Threads REQUIRED)
target_link_libraries(keyless_topic_history_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)

# Six publishing participants plus a subscribing one all have to discover each other before
# anything is published, so the ceiling is sized for discovery on a heavily loaded runner
# rather than for the burst itself, which is twenty-four small samples and takes no time.
# Sanitizer builds scale this further, in provizio_dds_finalize_tests below.
if(WIN32)
    set(_KEYLESS_TOPIC_HISTORY_TIMEOUT 240)
else()
    set(_KEYLESS_TOPIC_HISTORY_TIMEOUT 180)
endif()

foreach(_case every_emitters_samples_survive_the_defaults
        the_default_writer_history_holds_the_whole_burst
        a_one_slot_writer_history_drops_all_but_the_newest)
    provizio_dds_add_subcommand_test(keyless_topic_history_${_case}
        TARGET keyless_topic_history_test
        ARGS ${_case}
        TIMEOUT ${_KEYLESS_TOPIC_HISTORY_TIMEOUT}
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:keyless_topic_history_test>")
endforeach()

# The two late-join cases both publish to rt/keyless_topic_history_late_join_test, on a
# domain each process draws at random from 1..200. Run concurrently under `ctest -j`, a
# domain collision puts the depth-8 fleet's TRANSIENT_LOCAL backlog in front of the
# one-slot case's late joiner, which then counts twenty-four labels where it required six
# -- a 1-in-200 failure that reads as a real history regression rather than as cross-talk.
# A shared lock serialises exactly those two; every_emitters_samples_survive_the_defaults
# uses a different topic name and is deliberately left free to run alongside them.
set_tests_properties(keyless_topic_history_the_default_writer_history_holds_the_whole_burst
                     keyless_topic_history_a_one_slot_writer_history_drops_all_but_the_newest
                     PROPERTIES RESOURCE_LOCK provizio_dds_test_keyless_topic_history_late_join)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
