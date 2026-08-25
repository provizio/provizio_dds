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

# Subcommand-driven test exe for keeping VPN / tunnel interfaces out of the DDS
# transports: the interface classifier, the interface_blocklist a fresh
# participant is configured with, the PROVIZIO_DDS_ALLOW_VPN_INTERFACES
# override, and the matching exclusion from network-recovery change detection.
# Each subcommand is its own ctest entry so per-case failure stays isolated, and
# so each case gets a fresh process — the override is parsed once per process.
add_executable(vpn_interfaces_test vpn_interfaces_test.cpp)
# Threads::Threads for the std::mutex guarding the warnings this collects from a log
# callback: on a glibc older than 2.34 the pthread symbols behind it live outside libc
# (nm reports pthread_rwlock_rdlock and pthread_mutex_lock undefined here), so linking
# provizio_dds alone fails there and succeeds on every newer host. Every other threading
# test target in test/ does the same.
find_package(Threads REQUIRED)
target_link_libraries(vpn_interfaces_test PUBLIC provizio_dds provizio_dds_types Threads::Threads)
# The blocklist-shaping helper the POSIX backends share is a template in a library-private
# header, so the case that pins which entries it keeps has to include it directly.
target_include_directories(vpn_interfaces_test PRIVATE "${CMAKE_SOURCE_DIR}/src")

# Per-OS timeouts. Every case enumerates interfaces and at most creates a couple
# of participants whose QoS it reads back — no pub/sub traffic — so it returns
# near-instantly; the ceiling just tolerates loaded CI runners. Registration itself
# (including the Windows DLL PATH, without which the executable cannot start) is
# provizio_dds_add_subcommand_test's job — see test/CMakeLists.txt.
if(WIN32)
    set(_VPN_TIMEOUT 90)
else()
    set(_VPN_TIMEOUT 60)
endif()

set(_VPN_CASES classifier blocklist blocklist_forced blocklist_override_all blocklist_override_named
    blocklist_override_unmatched blocklist_override_all_matched blocklist_participant_netmask_off
    blocklist_allowed_enumeration_failure_changes_nothing
    no_op_with_builtin_transports_env leaves_caller_transports_alone
    snapshot_excludes_vpn snapshot_honours_override
    snapshot_policy_excludes_tunnel snapshot_policy_follows_transports classifier_table)
if(NOT WIN32)
    # POSIX-only: the helper it covers is the getifaddrs backends' own, and does not
    # compile (or exist) on Windows, whose enumeration is GetAdaptersAddresses.
    list(APPEND _VPN_CASES blocklist_entries_from_walk)
endif()

foreach(_case ${_VPN_CASES})
    provizio_dds_add_subcommand_test(vpn_interfaces_${_case}
        TARGET vpn_interfaces_test
        ARGS ${_case}
        TIMEOUT ${_VPN_TIMEOUT}
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:vpn_interfaces_test>")
endforeach()

# Its own entry, because it needs PROVIZIO_DDS_ALLOW_VPN_INTERFACES set before the process
# starts: the variable is parsed once, on first use, which is the very call the case asserts
# on.
provizio_dds_add_subcommand_test(vpn_interfaces_snapshot_policy_honours_override
    TARGET vpn_interfaces_test
    ARGS snapshot_policy_honours_override
    TIMEOUT ${_VPN_TIMEOUT}
    ENV "PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1"
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:vpn_interfaces_test>")

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
