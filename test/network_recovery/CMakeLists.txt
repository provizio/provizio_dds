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
else()
    set(_NR_TIMEOUT_FAST 10)
    set(_NR_TIMEOUT_DDS 45)
endif()

# Helper: register a subcommand as a ctest entry with an optional extra environment
# variable. A thin adapter over provizio_dds_add_subcommand_test (test/CMakeLists.txt),
# which owns the registration boilerplate — the Windows DLL PATH above all — that this
# suite and test/vpn_interfaces would otherwise each carry a copy of. Kept as a local name
# because 29 call sites read better for it.
function(_nr_add_test name)
    set(options)
    set(one_value_args TIMEOUT ENV)
    set(multi_value_args ARGS)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    provizio_dds_add_subcommand_test(${name}
        TARGET network_recovery_test
        ARGS ${ARG_ARGS}
        TIMEOUT ${ARG_TIMEOUT}
        ENV ${ARG_ENV})
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
# Regression: a transient flap (a DDS-relevant address that leaves and returns
# within the debounce window) nets to an unchanged end-snapshot but must still
# rebuild — the coalescer compares the burst-start snapshot too.
_nr_add_test(network_recovery_coalescer_resets_on_transient_flap
    ARGS coalescer_resets_on_transient_flap TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_no_rebuild_on_address_loss
    ARGS no_rebuild_on_address_loss TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_unreadable_interfaces_are_not_a_change
    ARGS unreadable_interfaces_are_not_a_change TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_unreadable_interfaces_warn_once_per_streak
    ARGS unreadable_interfaces_warn_once_per_streak TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_no_baseline_rebuilds_for_first_readable_list
    ARGS no_baseline_rebuilds_for_first_readable_list TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_no_baseline_empty_list_is_not_a_rebuild
    ARGS no_baseline_empty_list_is_not_a_rebuild TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_rebuild_on_address_change
    ARGS rebuild_on_address_change TIMEOUT ${_NR_TIMEOUT_DDS})
# transport_mode::localhost_only declines auto-recovery whatever the recovery mode says:
# a locator set that holds nothing but 127.0.0.1 has nothing a network change can affect.
_nr_add_test(network_recovery_localhost_only_not_monitored
    ARGS localhost_only_not_monitored TIMEOUT ${_NR_TIMEOUT_DDS})
# A log callback is documented as free to create a participant. This pins the one
# diagnostic that used to be emitted while registry_mutex was held, where doing so
# deadlocked; a regression hangs, so the TIMEOUT is what reports it.
_nr_add_test(network_recovery_log_callback_may_create_participant
    ARGS log_callback_may_create_participant TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_wait_for_idle_blocks_during_reset
    ARGS wait_for_idle_blocks_during_reset TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_reentrant_listener_during_reset
    ARGS reentrant_listener_during_reset TIMEOUT ${_NR_TIMEOUT_DDS})
_nr_add_test(network_recovery_concurrent_make_publisher_during_reset
    ARGS concurrent_make_publisher_during_reset TIMEOUT ${_NR_TIMEOUT_DDS})

# The prefix length is part of the snapshot identity, so re-subnetting an interface
# without changing its address counts as a network change (Fast-DDS derives its
# netmask-based locator filtering from it).
_nr_add_test(network_recovery_snapshot_prefix_length
    ARGS snapshot_prefix_length TIMEOUT ${_NR_TIMEOUT_FAST})

# Own process: PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES is parsed exactly once.
# The value deliberately contains stray whitespace and an empty entry.
_nr_add_test(network_recovery_extra_interfaces_env ARGS extra_interfaces_env TIMEOUT ${_NR_TIMEOUT_FAST}
    ENV "PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES=docker0, br-test ,,veth9")

# Safety-net tick: the backstop for everything the kernel event channel cannot
# report. Each of these used to be a permanent, process-lifetime outage.
#
# The four tick tests below set SAFETY_NET_SEC=0 to stop the COALESCER running its own
# tick concurrently: they drive run_safety_net_tick_for_test() from the test thread,
# which is only safe while the coordinator is idle, and two threads mutating
# last_known_snapshot would be a data race (and could move reset_count by two).
# A pathological safety-net period must be clamped, not turned into a busy loop: the
# duration→nanoseconds conversion wait_for performs overflows int64 beyond ~9.2e9 s and
# then returns immediately, every time.
_nr_add_test(network_recovery_safety_net_env_clamped ARGS safety_net_env_clamped
    TIMEOUT ${_NR_TIMEOUT_DDS} ENV "PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=99999999999")

_nr_add_test(network_recovery_safety_net_detects_missed_change
    ARGS safety_net_detects_missed_change TIMEOUT ${_NR_TIMEOUT_DDS} ENV "PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0")
_nr_add_test(network_recovery_safety_net_reopens_dead_monitor
    ARGS safety_net_reopens_dead_monitor TIMEOUT ${_NR_TIMEOUT_DDS} ENV "PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0")
_nr_add_test(network_recovery_safety_net_retries_failed_rebuild
    ARGS safety_net_retries_failed_rebuild TIMEOUT ${_NR_TIMEOUT_DDS} ENV "PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0")
# The tick decides what a change means exactly as the event path does — including not
# rebuilding for an address loss alone, which it used to ignore.
_nr_add_test(network_recovery_safety_net_no_rebuild_on_address_loss
    ARGS safety_net_no_rebuild_on_address_loss TIMEOUT ${_NR_TIMEOUT_DDS}
    ENV "PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0")

# End-to-end against a REAL kernel interface transition inside a throw-away network
# namespace: a link coming up with its address already assigned (emitting only
# RTM_NEWLINK), and carrier appearing on an interface whose address and
# administrative state never change. Both used to go undetected. Needs root or
# passwordless sudo plus iproute2, and reports ctest SKIP otherwise.
if(UNIX AND NOT APPLE)
    add_test(NAME network_recovery_carrier
        COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/carrier_recovery_test.sh" "$<TARGET_FILE:network_recovery_test>")
    set_tests_properties(network_recovery_carrier PROPERTIES TIMEOUT 180 SKIP_RETURN_CODE 77)
endif()

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
