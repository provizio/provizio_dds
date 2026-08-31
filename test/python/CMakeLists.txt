set(TEST_PATH "${CMAKE_CURRENT_BINARY_DIR}")

# Install dependencies, copy tests and all dependencies to the same test directory.
# Suppressing errors as packages can be installed manually, as the case in CI.
# On Windows, pywin32 is also required at runtime: Fast-DDS-Python 2.x's generated
# `fastdds.py` calls `win32api.LoadLibrary('fastdds-X.Y.dll')` at import time on
# Windows, and without it every Python test fails with ModuleNotFoundError.
if(WIN32)
    set(PYTHON_TEST_DEPS "'numpy>=1.16', 'transforms3d>=0.4.1', 'pywin32'")
else()
    set(PYTHON_TEST_DEPS "'numpy>=1.16', 'transforms3d>=0.4.1'")
endif()
add_custom_target(python_tests ALL
    COMMAND ${PYTHON_EXECUTABLE} -c "import subprocess, sys$<SEMICOLON> subprocess.call([sys.executable, '-m', 'pip', 'install', ${PYTHON_TEST_DEPS}, '--quiet'])"
    VERBATIM
)
file(GLOB TEST_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/*.py")
foreach(FILE ${PROVIZIO_DDS_ALL_PYTHON_LIBS} ${TEST_SCRIPTS})
    add_custom_command(TARGET python_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${FILE}" "${TEST_PATH}"
    )
endforeach()
if("${CMAKE_SHARED_LIBRARY_SUFFIX}" STREQUAL ".dylib")
    # Python expects .so and .so only in import
    add_custom_command(TARGET python_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink "${TEST_PATH}/_provizio_dds_python_types${CMAKE_SHARED_LIBRARY_SUFFIX}" "${TEST_PATH}/_provizio_dds_python_types.so")
    add_custom_command(TARGET python_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink "${TEST_PATH}/_fastdds_python${CMAKE_SHARED_LIBRARY_SUFFIX}" "${TEST_PATH}/_fastdds_python.so")
endif("${CMAKE_SHARED_LIBRARY_SUFFIX}" STREQUAL ".dylib")
if(WIN32 AND PROVIZIO_DDS_PYTHON_EXTRA_DLLS)
    foreach(TGT ${PROVIZIO_DDS_PYTHON_EXTRA_DLLS})
        add_custom_command(TARGET python_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${TGT}>" "${TEST_PATH}"
        )
    endforeach()
endif()
# On every platform we also copy libprovizio_dds alongside the Python tests.
# The Python wrapper's network_recovery module loads libprovizio_dds via
# ctypes to call the C ABI shim `provizio_dds_refresh_fastdds_interface_cache`
# — this matches the pip-wheel layout where libprovizio_dds is shipped
# alongside provizio_dds.py inside the package. Without this copy the
# in-tree test layout can't resolve the library on Linux / macOS (only the
# SWIG module's RUNPATH points there, and ctypes does not walk other libs'
# RUNPATH entries).
add_custom_command(TARGET python_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:provizio_dds>" "${TEST_PATH}"
)
add_dependencies(python_tests provizio_dds_python_types provizio_dds)
if(FAST_DDS_DEPENDENCIES)
    add_dependencies(python_tests ${FAST_DDS_DEPENDENCIES})
endif(FAST_DDS_DEPENDENCIES)

add_test(NAME python_pub_python_sub COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
add_test(NAME python_pub_python_sub_type_reuse COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher_type_reuse.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber_type_reuse.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_pub_cpp_sub COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher.py"
    "\"$<TARGET_FILE:simplest_subscriber>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_pub_cpp_sub PROPERTIES TIMEOUT 20)

add_test(NAME cpp_pub_python_sub COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:simplest_publisher>\""
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(cpp_pub_python_sub PROPERTIES TIMEOUT 20)

add_test(NAME python_request_response COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response PROPERTIES TIMEOUT 30)

add_test(NAME python_request_response_async COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service_concurrent.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_async PROPERTIES TIMEOUT 30)

# ServiceClient (Python): created in advance, many requests each returning its own awaitable. Reuse the
# existing x^2 services. Consecutive requests after an explicit wait_for_service():
add_test(NAME python_request_response_service_client COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_service_client PROPERTIES TIMEOUT 90)

# Many parallel requests (asyncio.gather) against the async service:
add_test(NAME python_request_response_concurrent_service_client COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service_concurrent.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_concurrent_service_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_concurrent_service_client PROPERTIES TIMEOUT 90)

# Cross-compat: Python ServiceClient against the C++ service.
add_test(NAME python_service_client_cpp_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\" serial_requests"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_service_client_cpp_service PROPERTIES TIMEOUT 90)

# Cross-compat: C++ service_client against the Python service.
add_test(NAME cpp_service_client_python_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service.py"
    "\"$<TARGET_FILE:request_response_service_client>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(cpp_service_client_python_service PROPERTIES TIMEOUT 90)

# Add inter-iteration delay to let the OS release DDS resources between rapid
# participant create/destroy cycles.
set(PYTHON_RELIABILITY_DELAY_ARGS "--delay" "0.5")

# Per-iteration harness timeout: kept above the client's own 30s request timeout
# (python_request_response_reliability_client.py) so a stalled iteration reaches the
# client's self-diagnostic ("Timeout waiting for response") and clean exit first, rather
# than being SIGKILLed mid-request with no usable log. Purely a backstop for a fully hung
# process -- healthy iterations complete in ~1-3s.
add_test(NAME python_request_response_reliability COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_times.py" ${PYTHON_RELIABILITY_DELAY_ARGS} "--timeout" "40" "128"
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_reliability_service.py \"\" {i} 1 {i}"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_reliability_client.py \"\" {i} {i}"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_reliability PROPERTIES TIMEOUT 2000) # Takes long by design

add_test(NAME python_request_response_reliability_single_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_reliability_service.py _single_service 1 128 200"
    "\"${PYTHON_EXECUTABLE}\" \"${CMAKE_SOURCE_DIR}/test/run_times.py\" 128 \"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_reliability_client.py _single_service {i} 200"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_response_reliability_single_service PROPERTIES TIMEOUT 3500) # Takes long by design

add_test(NAME python_request_response_ignore COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_ignore_service.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_ignore_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
add_test(NAME python_request_response_ignore_async COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_ignore_service_async.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_ignore_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_request_response_dual_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_dual_service.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
# The test's deterministic floor is ~3 s (2 services + 5 concurrent requests
# with a 1 s stable-match settle window per request). Everything above that
# is platform variance: aarch64 Debug clang builds are observed at ~30 s on
# the jetson runner due to per-op _registration_mutex / _lifecycle_lock /
# ListenerDrain overhead on top of slow SWIG-binding import. 60 s gives 20x
# headroom over the floor without masking a real regression (a true hang
# still times out).
set_tests_properties(python_request_response_dual_service PROPERTIES TIMEOUT 60)

add_test(NAME python_request_cpp_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\""
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_service PROPERTIES TIMEOUT 30)

add_test(NAME python_request_cpp_response_concurrent_requests_serial_service COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_service>\""
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_concurrent_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_response_concurrent_requests_serial_service PROPERTIES TIMEOUT 30)

add_test(NAME python_request_cpp_service_concurrent COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"$<TARGET_FILE:request_response_concurrent_service>\" 5"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_concurrent_client.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_request_cpp_service_concurrent PROPERTIES TIMEOUT 30)

add_test(NAME python_service_cpp_client COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service.py"
    "\"$<TARGET_FILE:request_response_client>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_service_cpp_client PROPERTIES TIMEOUT 30)

add_test(NAME python_serial_service_cpp_concurrent_requests COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service.py"
    "\"$<TARGET_FILE:request_response_concurrent_requests>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_serial_service_cpp_concurrent_requests PROPERTIES TIMEOUT 30)

add_test(NAME python_service_cpp_requests_concurrent COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_request_response_service_concurrent.py"
    "\"$<TARGET_FILE:request_response_concurrent_requests>\""
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_service_cpp_requests_concurrent PROPERTIES TIMEOUT 30)

add_test(NAME python_point_cloud2_test COMMAND
    "${PYTHON_EXECUTABLE}" -q -X faulthandler ./point_cloud2_test.py
    WORKING_DIRECTORY "${TEST_PATH}"
)
set_tests_properties(python_point_cloud2_test PROPERTIES TIMEOUT 30)

add_test(NAME python_publisher_no_leaks_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_publisher_no_leaks_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_subscriber_no_leaks_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_subscriber_no_leaks_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME python_stop_subscriber_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./python_stop_subscriber_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

add_test(NAME accumulation_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./accumulation_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

# Python network auto-recovery tests — mirror the C++
# network_recovery_test entries. Each subcommand is a separate ctest
# entry so the env-var resolution cache (one-shot per process) stays
# clean between cases.
# Tests with no env-var override — they get only the default
# Windows-DLL PATH applied via PYTHON_TEST_NAMES at the bottom of
# this file.
set(_PYTHON_NETWORK_RECOVERY_TEST_NAMES_NO_ENV)
foreach(_nrt_case logging env_recovery snapshot reset_roundtrip reset_disabled
        reset_refreshes_fastdds_interface_cache teardown_deferred
        coalescer_resets_on_transient_flap no_rebuild_on_address_loss
        unreadable_interfaces_are_not_a_change no_baseline_rebuilds_for_first_readable_list
        rebuild_on_address_change snapshot_prefix_length
        netmask_read_is_bounded linked_list_walk_advances
        netlink_binds_before_snapshot safety_net_env
        snapshot_policy_excludes_tunnel snapshot_policy_follows_transports
        netlink_kinds_match_the_kernel listener_drain_reports_a_stall_until_it_ends
        safety_net_retries_while_interfaces_are_unreadable)
    add_test(NAME python_network_recovery_${_nrt_case} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_network_recovery_test.py ${_nrt_case}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    list(APPEND _PYTHON_NETWORK_RECOVERY_TEST_NAMES_NO_ENV python_network_recovery_${_nrt_case})
    # The reset_roundtrip case does Fast-DDS discovery so it gets a bit
    # more time; the others are sub-second. Compare against the variable
    # name (not its value via ${...} expansion) so CMake matches the
    # variable's current contents against the literal — `${_nrt_case}`
    # would expand to e.g. `logging`, then `if(logging STREQUAL ...)`
    # would look for a CMake variable named `logging` and compare against
    # its (empty) contents instead.
    if(_nrt_case STREQUAL "reset_roundtrip" OR _nrt_case STREQUAL "reset_refreshes_fastdds_interface_cache" OR _nrt_case STREQUAL "teardown_deferred")
        set_tests_properties(python_network_recovery_${_nrt_case} PROPERTIES TIMEOUT 45)
    else()
        set_tests_properties(python_network_recovery_${_nrt_case} PROPERTIES TIMEOUT 30)
    endif()
endforeach()

# The safety-net tick cases drive run_safety_net_tick_for_test() from the test thread,
# which is only safe while the coordinator is idle — so they disable the monitor's own
# periodic tick rather than race it. Mirrors the C++ entries in
# test/network_recovery/CMakeLists.txt.
foreach(_nrt_tick_case safety_net_detects_missed_change safety_net_reopens_dead_monitor
        safety_net_retries_failed_rebuild safety_net_retry_gives_up_after_bound)
    add_test(NAME python_network_recovery_${_nrt_tick_case} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_network_recovery_test.py ${_nrt_tick_case}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    # SAFETY_NET_SEC=0 silences the netlink backend's periodic tick (Linux). The polling
    # backend (macOS / Windows, or Linux without netlink) deliberately ignores it — its
    # poll interval IS the tick — so ALSO pin the poll interval past any test duration:
    # otherwise a background poller tick races the test-driven run_safety_net_tick_for_test()
    # calls (it would consume bounded retry passes and fire resets inside monkeypatched
    # snapshot windows).
    set(_tick_env "PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0")
    list(APPEND _tick_env "PROVIZIO_DDS_NETWORK_RECOVERY_POLL_INTERVAL_SEC=3600")
    if(WIN32)
        list(APPEND _tick_env "PATH=${PROVIZIO_DDS_TEST_PATH}")
    endif()
    # The give-up-bound case runs a participant rebuild per retry pass (7+ of them),
    # so it gets more headroom than the sub-second tick cases.
    if(_nrt_tick_case STREQUAL "safety_net_retry_gives_up_after_bound")
        set(_tick_timeout 90)
    else()
        set(_tick_timeout 45)
    endif()
    set_tests_properties(python_network_recovery_${_nrt_tick_case}
        PROPERTIES TIMEOUT ${_tick_timeout} ENVIRONMENT "${_tick_env}")
endforeach()

# Discovered-endpoints / type-registry suite — Python mirror of the C++
# test/discovered_endpoints/ subcommands.
set(_PYTHON_DISCOVERED_ENDPOINTS_TEST_NAMES)
foreach(_de_case type_registry discovers_writer kind_filter constructor_callback callback_receives_participant survives_reset unregister teardown_does_not_spin)
    add_test(NAME python_discovered_endpoints_${_de_case} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_discovered_endpoints_test.py ${_de_case}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    list(APPEND _PYTHON_DISCOVERED_ENDPOINTS_TEST_NAMES python_discovered_endpoints_${_de_case})
    # Deterministic subcommand (type_registry) finishes in milliseconds; the
    # DDS-discovery ones have an internal 15s wait. `unregister` chains three
    # such waits sequentially (pre-unregister, the control participant observing
    # the post-unregister publisher, then post-re-register), so it gets double
    # the ceiling to keep overhead margin under that 3x worst case on loaded
    # runners.
    if(_de_case STREQUAL "type_registry")
        set_tests_properties(python_discovered_endpoints_${_de_case} PROPERTIES TIMEOUT 15)
    elseif(_de_case STREQUAL "unregister")
        set_tests_properties(python_discovered_endpoints_${_de_case} PROPERTIES TIMEOUT 90)
    else()
        set_tests_properties(python_discovered_endpoints_${_de_case} PROPERTIES TIMEOUT 45)
    endif()
endforeach()

# Match-publisher reader-reliability default — Python mirror of the C++
# test/match_publisher_default/ subcommands. A default-reliability subscriber
# defers creating its DataReader until a matching writer is discovered, then
# adopts that writer's reliability; an explicit reliability builds eagerly. Each
# subcommand is its own ctest entry so per-case failure stays isolated — same
# pattern as python_discovered_endpoints_*. Every case either resolves a real
# cross-participant match (internal 15s wait) or, for the deferral case, returns
# near-instantly; the DDS ceiling covers all of them.
set(_PYTHON_MATCH_PUBLISHER_TEST_NAMES)
foreach(_mp_case deferral adopts_reliable adopts_best_effort explicit_best_effort explicit_reliable participant_dropped heterogeneous_rederive)
    add_test(NAME python_match_publisher_default_${_mp_case} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_match_publisher_default_test.py ${_mp_case}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    list(APPEND _PYTHON_MATCH_PUBLISHER_TEST_NAMES python_match_publisher_default_${_mp_case})
    set_tests_properties(python_match_publisher_default_${_mp_case} PROPERTIES TIMEOUT 45)
endforeach()

# Participant auto-discovery tuning — Python mirror of the C++ test/discovery_tuning/
# subcommands: the de-escalated discovery defaults, the PROVIZIO_DDS_DISCOVERY_* env overrides,
# and many-participant discovery convergence. Each subcommand is its own ctest entry so per-case
# failure stays isolated. The multi_participant case spins up 2*K participants (its own 60s
# convergence budget); the rest create a single participant and return near-instantly.
set(_PYTHON_DISCOVERY_TUNING_TEST_NAMES)
foreach(_dt_case defaults env_overrides env_invalid multi_participant)
    add_test(NAME python_discovery_tuning_${_dt_case} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_discovery_tuning_test.py ${_dt_case}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    list(APPEND _PYTHON_DISCOVERY_TUNING_TEST_NAMES python_discovery_tuning_${_dt_case})
    if(_dt_case STREQUAL "multi_participant")
        set_tests_properties(python_discovery_tuning_${_dt_case} PROPERTIES TIMEOUT 90)
    else()
        set_tests_properties(python_discovery_tuning_${_dt_case} PROPERTIES TIMEOUT 30)
    endif()
endforeach()

# Participant transport tuning — Python mirror of the C++ test/transport_tuning/
# subcommands: the MTU-sized default of the send-side RTPS message-size cap
# (fastdds.max_message_size), the PROVIZIO_DDS_MAX_MESSAGE_SIZE env override, and the
# fallback to the default on malformed input. Each subcommand is its own ctest entry so
# per-case failure stays isolated. Every case creates one (or a handful of) participants
# and only reads its QoS back, so it returns near-instantly.
set(_PYTHON_TRANSPORT_TUNING_TEST_NAMES)
foreach(_tt_case defaults env_override env_invalid)
    add_test(NAME python_transport_tuning_${_tt_case} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_transport_tuning_test.py ${_tt_case}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    list(APPEND _PYTHON_TRANSPORT_TUNING_TEST_NAMES python_transport_tuning_${_tt_case})
    set_tests_properties(python_transport_tuning_${_tt_case} PROPERTIES TIMEOUT 30)
endforeach()

# Keeping VPN / tunnel interfaces out of the DDS transports — Python mirror of the
# C++ test/vpn_interfaces/ subcommands: the interface classifier, the generated
# transport profile and its once-per-configuration loading, that a participant
# configured with an interface blocklist still communicates, the
# PROVIZIO_DDS_ALLOW_VPN_INTERFACES override, and the matching exclusion from
# network-recovery change detection. Each subcommand is its own ctest entry so
# per-case failure stays isolated — and because the override is parsed once per
# process, so a case can set it before anything reads it.
set(_PYTHON_VPN_INTERFACES_TEST_NAMES)
foreach(_vpn_case classifier transports_profile profile_loaded_once profile_limit
        profile_route_honours_process_transports leaves_caller_profile_alone
        participant_communicates blocklist_read_failure_keeps_exclusion
        blocklist_read_failure_reports_to_change_detection
        interface_kind_lookup_failure_reported
        allowed_read_failure_keeps_exclusion
        allowed_read_failure_reports_to_change_detection
        profile_forgotten_when_tunnel_gone read_failure_flags_are_per_thread
        allowed_interfaces_include_loopback_on_windows
        exclusion_report_names_netmask_filtering caller_takeover_forgets_profile
        profile_cache_follows_the_interfaces
        override_all override_named override_unmatched_names_reported
        override_all_matched_is_quiet unmatched_name_survives_a_failed_profile_readback
        no_op_with_builtin_transports_env
        caller_builtin_transports_matching_ours env_xml_profile_honoured_per_participant
        snapshot_excludes_vpn
        snapshot_excludes_vpn_ipv6 unreadable_interfaces_still_create_participant)
    add_test(NAME python_vpn_interfaces_${_vpn_case} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_vpn_interfaces_test.py ${_vpn_case}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    list(APPEND _PYTHON_VPN_INTERFACES_TEST_NAMES python_vpn_interfaces_${_vpn_case})
    # participant_communicates waits for a match and a delivered sample; the rest
    # only enumerate interfaces or build a profile string and return near-instantly.
    if(_vpn_case STREQUAL "participant_communicates")
        set_tests_properties(python_vpn_interfaces_${_vpn_case} PROPERTIES TIMEOUT 90)
    else()
        set_tests_properties(python_vpn_interfaces_${_vpn_case} PROPERTIES TIMEOUT 30)
    endif()
endforeach()

# Dead-owner shared-memory cleanup — Python mirror of the C++ test/shm_cleanup/
# subcommands: what the sweep reclaims, what it must never touch, its age guard, its kill
# switch, its rate limit, and that creating a participant triggers it. Each subcommand is
# its own ctest entry so per-case failure stays isolated.
#
# Linux only, like the C++ suite: shared memory is provizio_dds's default transport there
# and nowhere else (Windows and macOS force UDP over a Boost.Interprocess cleanup bug), so
# no other platform creates the files this reclaims. The script skips itself on any other
# platform too, so the guard here is a convenience rather than the only safeguard.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    foreach(_sc_case reclaims_synthetic age_guard kill_switch leaves_other_files_alone
                     rejects_planted_lock_files rate_limited reclaims_dead_participant spares_the_living)
        add_test(NAME python_shm_cleanup_${_sc_case} COMMAND
            "${PYTHON_EXECUTABLE}" -q -X faulthandler
            ./python_shm_cleanup_test.py ${_sc_case}
            WORKING_DIRECTORY "${TEST_PATH}"
        )
    endforeach()
    # The cases that only lay out files in /dev/shm and sweep them return near-instantly;
    # the two that fork a real participant (and, for spares_the_living, wait for samples to
    # flow over shared memory across a sweep) need discovery time.
    #
    # Cases that lay their fixtures out in a directory of their own are hermetic — no other
    # process on the host sweeps a subdirectory of /dev/shm — so they need no lock.
    set_tests_properties(python_shm_cleanup_reclaims_synthetic python_shm_cleanup_age_guard
                         python_shm_cleanup_kill_switch python_shm_cleanup_leaves_other_files_alone
                         python_shm_cleanup_rejects_planted_lock_files PROPERTIES TIMEOUT 60)
    # The rest must work in the real /dev/shm and lower the age guard to reach their own
    # fixtures, so they would happily reclaim each other's — and the C++ mirror's, hence the
    # shared lock name it also uses. Other suites cannot interfere: their sweeps use the
    # default guard, and these fixtures are deliberately kept younger than it.
    set_tests_properties(python_shm_cleanup_rate_limited PROPERTIES TIMEOUT 60
                         RESOURCE_LOCK provizio_dds_shm_cleanup)
    set_tests_properties(python_shm_cleanup_reclaims_dead_participant
                         python_shm_cleanup_spares_the_living PROPERTIES TIMEOUT 120
                         RESOURCE_LOCK provizio_dds_shm_cleanup)
endif()

# Cross-language parity of the per-type QoS default overrides: the Python registration
# must cover the C++ qos_defaults<T> specialization list exactly AND register the same
# values for every type. Both source files are passed from the source tree: the header
# for the list of specializations, and qos_defaults_checks.cpp for the values it already
# pins in static_asserts — so the C++ compiler, not a regex, is what keeps those values
# in step with the header. Import-only — no DDS participants — so it returns in
# milliseconds.
add_test(NAME python_per_type_qos_parity COMMAND
    "${PYTHON_EXECUTABLE}" -q -X faulthandler
    ./python_qos_parity_test.py "${CMAKE_SOURCE_DIR}/include/provizio/dds/qos_defaults.h"
    "${CMAKE_SOURCE_DIR}/src/qos_defaults_checks.cpp"
    WORKING_DIRECTORY "${TEST_PATH}"
)
if(WIN32)
    set_tests_properties(python_per_type_qos_parity PROPERTIES TIMEOUT 30
        ENVIRONMENT "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set_tests_properties(python_per_type_qos_parity PROPERTIES TIMEOUT 30)
endif()

# Cross-language parity of the VPN / tunnel classifier: the same table of interface
# names is put to both languages' classifiers and the verdicts must match, so a vendor
# needle or prefix rule added on one side only is caught here rather than by two hosts
# disagreeing about one tunnel in the field. Import-only plus one short subprocess.
add_test(NAME python_vpn_classifier_parity COMMAND
    "${PYTHON_EXECUTABLE}" -q -X faulthandler
    ./vpn_classifier_parity_test.py "$<TARGET_FILE:vpn_interfaces_test>"
    WORKING_DIRECTORY "${TEST_PATH}"
)
if(WIN32)
    set_tests_properties(python_vpn_classifier_parity PROPERTIES TIMEOUT 60
        ENVIRONMENT "PATH=${PROVIZIO_DDS_TEST_PATH}")
else()
    set_tests_properties(python_vpn_classifier_parity PROPERTIES TIMEOUT 60)
endif()

# Crash-safety of throwing user callbacks (Python mirror of the C++
# test/callback_exceptions/ suite): a raising user callback must be reported
# through the configurable logger and must NOT abort the process / silently stop a
# service. Each subcommand is its own ctest entry.
foreach(_cbe_case on_data on_has_publisher_changed on_has_subscriber_changed service_sync service_async)
    add_test(NAME python_callback_exceptions_${_cbe_case} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_callback_exceptions_test.py ${_cbe_case}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    list(APPEND _PYTHON_CALLBACK_EXCEPTIONS_TEST_NAMES python_callback_exceptions_${_cbe_case})
    set_tests_properties(python_callback_exceptions_${_cbe_case} PROPERTIES TIMEOUT 45)
endforeach()

# Env-var resolution cases — each forces a fresh process so the
# function-local cache in _resolve_env_once observes the configured
# env value on its first (and only) call. Mirrors the C++ env_recovery_*
# entries in test/network_recovery/CMakeLists.txt.
#
# These set ENVIRONMENT directly (PROVIZIO_DDS_NETWORK_RECOVERY plus the
# Windows-DLL PATH when on Windows). They MUST NOT be appended to
# PYTHON_TEST_NAMES at the bottom of this file — that block overwrites
# ENVIRONMENT with PATH alone, which would erase our env-var override.
function(_python_nr_env_test name case_name env_value)
    add_test(NAME ${name} COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler
        ./python_network_recovery_test.py ${case_name}
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    set(_env "PROVIZIO_DDS_NETWORK_RECOVERY=${env_value}")
    if(WIN32)
        # PATH must be on its own list entry — ctest ENVIRONMENT is a
        # semicolon-separated list of NAME=VALUE pairs.
        list(APPEND _env "PATH=${PROVIZIO_DDS_TEST_PATH}")
    endif()
    set_tests_properties(${name} PROPERTIES TIMEOUT 30 ENVIRONMENT "${_env}")
endfunction()

_python_nr_env_test(python_network_recovery_env_default env_default "")
_python_nr_env_test(python_network_recovery_env_on      env_on      "on")
_python_nr_env_test(python_network_recovery_env_off     env_off     "off")
_python_nr_env_test(python_network_recovery_env_truthy  env_truthy  "yes")
_python_nr_env_test(python_network_recovery_env_falsy   env_falsy   "NO")
_python_nr_env_test(python_network_recovery_env_garbage env_garbage "banana")

# Force-included interfaces — same one-shot-per-process caching, so it needs its own
# process too. The value deliberately carries stray whitespace and an empty entry.
# Written out rather than routed through _python_nr_env_test because it overrides a
# different variable.
add_test(NAME python_network_recovery_extra_interfaces_env COMMAND
    "${PYTHON_EXECUTABLE}" -q -X faulthandler
    ./python_network_recovery_test.py extra_interfaces_env
    WORKING_DIRECTORY "${TEST_PATH}"
)
set(_extra_if_env "PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES=docker0, br-test ,,veth9")
if(WIN32)
    list(APPEND _extra_if_env "PATH=${PROVIZIO_DDS_TEST_PATH}")
endif()
set_tests_properties(python_network_recovery_extra_interfaces_env
    PROPERTIES TIMEOUT 30 ENVIRONMENT "${_extra_if_env}")

# The override case needs PROVIZIO_DDS_ALLOW_VPN_INTERFACES set before the process starts:
# it is parsed once per process, on first use, which is inside the very call the case
# asserts on. Its no-override twin runs in the plain list above.
add_test(NAME python_network_recovery_snapshot_policy_honours_override COMMAND
    "${PYTHON_EXECUTABLE}" -q -X faulthandler
    ./python_network_recovery_test.py snapshot_policy_honours_override
    WORKING_DIRECTORY "${TEST_PATH}"
)
set(_allow_vpn_env "PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1")
if(WIN32)
    list(APPEND _allow_vpn_env "PATH=${PROVIZIO_DDS_TEST_PATH}")
endif()
set_tests_properties(python_network_recovery_snapshot_policy_honours_override
    PROPERTIES TIMEOUT 30 ENVIRONMENT "${_allow_vpn_env}")

# 1.10.x Python API source-compat: ensures `ReturnCode_t.RETCODE_OK`,
# `pub_sub_type.getName()`, and other camelCase / enum-class spellings
# carried over from Fast-DDS-python 1.x still resolve after the 2.x
# migration. Pure import-level checks, no DDS network activity.
add_test(NAME legacy_api_compat_test COMMAND
    "${PYTHON_EXECUTABLE}" -q -X faulthandler ./legacy_api_compat_test.py
    WORKING_DIRECTORY "${TEST_PATH}"
)

# Cross-version wire-interop: spin up the current (2.x) build and the
# pip-installed 1.10.1 reference side-by-side and exchange traffic in
# every direction (pub<->sub, req<->svc). Skips itself when the legacy
# venv isn't installed, so dev machines without it stay green; CI sets
# PROVIZIO_DDS_LEGACY_PYTHON in its provizio_dds.yml workflow after
# running setup_legacy_provizio_dds_venv.sh. See the runner script's
# docstring for the full matrix.
#
# Probe the runtime Python's `(major, minor)` so we can skip registering
# this test on platform/version combos where the legacy 1.10.1 venv setup
# is known not to work. Right now that's Python 3.8 on Windows: the
# `pip install git+...@1.10.1` source build of Fast-DDS 2.14.x against
# the EOL'd Python 3.8 toolchain fails before any output is even emitted
# (WinError 2 from the venv bootstrap), with no clean workaround. Python
# 3.9+ on Windows works fine and stays in the matrix.
execute_process(COMMAND "${PYTHON_EXECUTABLE}" -c
    "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
    OUTPUT_VARIABLE PYTHON_RUNTIME_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CROSS_VERSION_COMPAT_TEST_ENABLED TRUE)
if(WIN32 AND PYTHON_RUNTIME_VERSION VERSION_LESS "3.9")
    message(STATUS
        "Skipping cross_version_compat_test registration: Python "
        "${PYTHON_RUNTIME_VERSION} on Windows can't build the legacy "
        "provizio_dds 1.10.1 source distribution.")
    set(CROSS_VERSION_COMPAT_TEST_ENABLED FALSE)
endif()

if(CROSS_VERSION_COMPAT_TEST_ENABLED)
    add_test(NAME cross_version_compat_test COMMAND
        "${PYTHON_EXECUTABLE}" -q -X faulthandler ./cross_version_compat_test.py
        WORKING_DIRECTORY "${TEST_PATH}"
    )
    # `SKIPPED:` is the marker the test prints when `PROVIZIO_DDS_LEGACY_PYTHON`
    # is unset or the legacy interpreter can't import provizio_dds. With this
    # regex set, ctest reports the test as "Skipped" rather than "Passed" so a
    # missing setup step on a CI runner shows up loudly in the output.
    set_tests_properties(cross_version_compat_test PROPERTIES
        TIMEOUT 240
        SKIP_REGULAR_EXPRESSION "^SKIPPED:")
endif()

# Platform-specific timeouts: tighter on Linux/macOS to detect regressions,
# wider on Windows where DDS discovery and process startup are slower.
# "Takes long by design" tests keep the same timeout on all platforms.
if(WIN32)
    set_tests_properties(python_pub_python_sub PROPERTIES TIMEOUT 45)
    set_tests_properties(python_pub_python_sub_type_reuse PROPERTIES TIMEOUT 30)
    set_tests_properties(
        python_request_response_ignore
        python_request_response_ignore_async
        PROPERTIES TIMEOUT 60)
else()
    # These pub/sub tests spend ~10s in a fixed publish loop and create several
    # network-recovery-enabled participants. The ROS2-compatibility CI containers
    # (especially the Debug/clang configs) run roughly 2x slower, so the previous
    # 20s budget was too tight and flaked with spurious ***Timeout. 40s still
    # detects a genuine hang/regression while tolerating the slower containers.
    set_tests_properties(python_pub_python_sub PROPERTIES TIMEOUT 40)
    set_tests_properties(python_pub_python_sub_type_reuse PROPERTIES TIMEOUT 40)
    # 60, as on Windows, and for the same reason the pub/sub budgets above were raised:
    # these two spend ~25 s by design (the service waits out its request budget and then
    # sleeps), so 30 left about five seconds of margin -- which a jetson runner duly
    # consumed, failing a test whose assertions had already passed.
    set_tests_properties(
        python_request_response_ignore
        python_request_response_ignore_async
        PROPERTIES TIMEOUT 60)
endif()

if(WIN32)
    set(PYTHON_TEST_NAMES
        ${_PYTHON_NETWORK_RECOVERY_TEST_NAMES_NO_ENV}
        ${_PYTHON_DISCOVERED_ENDPOINTS_TEST_NAMES}
        ${_PYTHON_MATCH_PUBLISHER_TEST_NAMES}
        ${_PYTHON_DISCOVERY_TUNING_TEST_NAMES}
        ${_PYTHON_TRANSPORT_TUNING_TEST_NAMES}
        ${_PYTHON_VPN_INTERFACES_TEST_NAMES}
        ${_PYTHON_CALLBACK_EXCEPTIONS_TEST_NAMES}
        python_pub_python_sub python_pub_python_sub_type_reuse
        python_pub_cpp_sub cpp_pub_python_sub
        python_request_response python_request_response_async
        python_request_response_reliability python_request_response_reliability_single_service
        python_request_response_ignore python_request_response_ignore_async
        python_request_response_dual_service
        python_request_cpp_service python_request_cpp_response_concurrent_requests_serial_service
        python_request_cpp_service_concurrent python_service_cpp_client
        python_serial_service_cpp_concurrent_requests python_service_cpp_requests_concurrent
        python_request_response_service_client python_request_response_concurrent_service_client
        python_service_client_cpp_service cpp_service_client_python_service
        python_point_cloud2_test python_publisher_no_leaks_test python_subscriber_no_leaks_test
        python_stop_subscriber_test accumulation_test
        legacy_api_compat_test
    )
    if(CROSS_VERSION_COMPAT_TEST_ENABLED)
        list(APPEND PYTHON_TEST_NAMES cross_version_compat_test)
    endif()
    # PATH: DLLs must be locatable at runtime.
    set_tests_properties(${PYTHON_TEST_NAMES} PROPERTIES ENVIRONMENT
        "PATH=${PROVIZIO_DDS_TEST_PATH}")
endif(WIN32)


# ---------------------------------------------------------------------------------------
# Test isolation, mirroring test/request_response/CMakeLists.txt: these entries reuse the
# same DDS service name on the same domain (14) as each other AND as the C++ entries, so
# they take the same locks. Under `ctest -j` without them, one test's service answers (and
# fills its request quota from) another test's client. See the long note in
# test/request_response/CMakeLists.txt for the measurements.
# ---------------------------------------------------------------------------------------
set_tests_properties(
    cpp_service_client_python_service
    python_request_cpp_response_concurrent_requests_serial_service
    python_request_cpp_service
    python_request_cpp_service_concurrent
    python_request_response
    python_request_response_async
    python_request_response_concurrent_service_client
    python_request_response_service_client
    python_serial_service_cpp_concurrent_requests
    python_service_client_cpp_service
    python_service_cpp_client
    python_service_cpp_requests_concurrent
    PROPERTIES RESOURCE_LOCK provizio_dds_test_request_response)
set_tests_properties(
    python_request_response_ignore
    python_request_response_ignore_async
    PROPERTIES RESOURCE_LOCK provizio_dds_test_request_response_ignore)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
