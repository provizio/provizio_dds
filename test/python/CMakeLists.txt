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

add_test(NAME pointcloud2_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/test/run_parallel.py"
    "\"${PYTHON_EXECUTABLE}\" -q -X faulthandler ./pointcloud2_test.py"
    WORKING_DIRECTORY "${TEST_PATH}"
)

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
        reset_refreshes_fastdds_interface_cache teardown_deferred)
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

# Discovered-endpoints / type-registry suite — Python mirror of the C++
# test/discovered_endpoints/ subcommands.
set(_PYTHON_DISCOVERED_ENDPOINTS_TEST_NAMES)
foreach(_de_case type_registry discovers_writer kind_filter constructor_callback callback_receives_participant survives_reset unregister)
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
    set_tests_properties(
        python_request_response_ignore
        python_request_response_ignore_async
        PROPERTIES TIMEOUT 30)
endif()

if(WIN32)
    set(PYTHON_TEST_NAMES
        ${_PYTHON_NETWORK_RECOVERY_TEST_NAMES_NO_ENV}
        ${_PYTHON_DISCOVERED_ENDPOINTS_TEST_NAMES}
        ${_PYTHON_MATCH_PUBLISHER_TEST_NAMES}
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
        pointcloud2_test python_publisher_no_leaks_test python_subscriber_no_leaks_test
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
