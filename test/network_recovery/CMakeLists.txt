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

# Subcommand-driven test executable. Each ctest entry runs the same binary
# with a different subcommand (and, for env_recovery_*, a per-test
# PROVIZIO_DDS_NETWORK_RECOVERY override) so cached state from a previous
# subcommand can't leak between assertions.
add_executable(network_recovery_test network_recovery_test.cpp)
find_package(Threads REQUIRED)
target_link_libraries(network_recovery_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)

# Per-OS short timeouts: every subcommand either completes in milliseconds
# (logging, snapshot, runtime_idle, env_recovery) or has its own internal
# bounded retry loop (reset_*).
if(WIN32)
    set(_NR_TIMEOUT_FAST 30)
    set(_NR_TIMEOUT_DDS 60)
    set(_NR_ENV_BASE "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set(_NR_TIMEOUT_FAST 10)
    set(_NR_TIMEOUT_DDS 45)
    set(_NR_ENV_BASE "")
endif()

# Helper: register a subcommand as a ctest entry with an optional extra
# environment variable. Concatenating to PATH is needed on Windows.
function(_nr_add_test name)
    set(options)
    set(one_value_args TIMEOUT ENV)
    set(multi_value_args ARGS)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    add_test(NAME ${name} COMMAND network_recovery_test ${ARG_ARGS})
    set(_env "${_NR_ENV_BASE}")
    if(ARG_ENV)
        if(_env)
            list(APPEND _env "${ARG_ENV}")
        else()
            set(_env "${ARG_ENV}")
        endif()
    endif()
    if(_env)
        set_tests_properties(${name} PROPERTIES TIMEOUT ${ARG_TIMEOUT} ENVIRONMENT "${_env}")
    else()
        set_tests_properties(${name} PROPERTIES TIMEOUT ${ARG_TIMEOUT})
    endif()
endfunction()

_nr_add_test(network_recovery_logging        ARGS logging          TIMEOUT ${_NR_TIMEOUT_FAST})
_nr_add_test(network_recovery_snapshot       ARGS snapshot         TIMEOUT ${_NR_TIMEOUT_FAST})
_nr_add_test(network_recovery_runtime_idle   ARGS runtime_idle     TIMEOUT ${_NR_TIMEOUT_FAST})

# env_recovery: each entry forces a fresh process so the function-local
# `static const bool cached` in resolve_network_recovery_enabled() observes
# the configured env value on its first (and only) call.
_nr_add_test(network_recovery_env_default ARGS env_recovery on TIMEOUT ${_NR_TIMEOUT_FAST}
    ENV "PROVIZIO_DDS_NETWORK_RECOVERY=")  # unset / empty: default-on
_nr_add_test(network_recovery_env_on      ARGS env_recovery on TIMEOUT ${_NR_TIMEOUT_FAST}
    ENV "PROVIZIO_DDS_NETWORK_RECOVERY=on")
_nr_add_test(network_recovery_env_off     ARGS env_recovery off TIMEOUT ${_NR_TIMEOUT_FAST}
    ENV "PROVIZIO_DDS_NETWORK_RECOVERY=off")
_nr_add_test(network_recovery_env_truthy  ARGS env_recovery on TIMEOUT ${_NR_TIMEOUT_FAST}
    ENV "PROVIZIO_DDS_NETWORK_RECOVERY=yes")
_nr_add_test(network_recovery_env_falsy   ARGS env_recovery off TIMEOUT ${_NR_TIMEOUT_FAST}
    ENV "PROVIZIO_DDS_NETWORK_RECOVERY=NO")
_nr_add_test(network_recovery_env_garbage ARGS env_recovery_garbage TIMEOUT ${_NR_TIMEOUT_FAST}
    ENV "PROVIZIO_DDS_NETWORK_RECOVERY=banana")

_nr_add_test(network_recovery_reset_roundtrip ARGS reset_roundtrip TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_reset_disabled  ARGS reset_disabled  TIMEOUT ${_NR_TIMEOUT_DDS})
# APT-11792 regression test: verifies the auto-recovery reset actually
# refreshes Fast-DDS's process-wide interface cache. Without the fix the
# rebuilt participant binds to the same dead interfaces as the original
# (see provizio::dds::refresh_fastdds_interface_cache for the full story).
_nr_add_test(network_recovery_reset_refreshes_fastdds_interface_cache
    ARGS reset_refreshes_fastdds_interface_cache TIMEOUT ${_NR_TIMEOUT_DDS})

# Coalescer + concurrency tests using the test-only injection hooks.
_nr_add_test(network_recovery_coalescer_skips_no_change
    ARGS coalescer_skips_no_change TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_wait_for_idle_blocks_during_reset
    ARGS wait_for_idle_blocks_during_reset TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_reentrant_listener_during_reset
    ARGS reentrant_listener_during_reset TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_concurrent_make_publisher_during_reset
    ARGS concurrent_make_publisher_during_reset TIMEOUT ${_NR_TIMEOUT_DDS})

# Scale every test TIMEOUT in this directory when a sanitizer is active (no-op otherwise).
provizio_dds_scale_test_timeouts()
