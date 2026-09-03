# AGENTS.md

This file provides guidance to Claude Code and other AI agents when working with code in this repository.

## Project Overview

provizio_dds is a C++17/Python DDS communication library built on eProsima Fast-DDS (v3.6.x). It provides RAII-based pub/sub and request/response abstractions compatible with ROS 2 (Humble+ for pub/sub, Jazzy+ for request/response). Licensed under Apache 2.0.

## Quality Standards

This repository implements a **public API** consumed by Provizio customers and integrated into ROS 2 ecosystems. All contributions must meet the highest standards of code and documentation quality:

- **API surface**: Every public header, function signature, and parameter name is part of the customer-facing contract. Changes must be intentional, backward-compatible where possible, and clearly documented.
- **Documentation**: All public C++ functions, classes, and parameters must have Doxygen-style `@brief`/`@param`/`@return` comments. Python docstrings are required for all public functions and classes. README examples must stay accurate and runnable.
- **Code clarity**: Favor readability and explicitness over brevity. Template-heavy code must include comments explaining the intent. Error messages exposed to users must be actionable.
- **Testing**: New functionality requires corresponding tests. Existing tests must continue to pass across all supported platforms (Linux, macOS, Windows) and compilers (gcc, clang, MSVC).
- **ABI/API stability**: Avoid breaking changes to public headers. When adding `PROVIZIO_DDS_API`-exported symbols, ensure they are correctly decorated for DLL boundaries on Windows. Template-only code does not need the macro; non-template functions and data symbols do.

## Build Commands

### C++ Build (from repository root)

```bash
# Basic build (builds Fast-DDS from source if not found)
mkdir -p build && cd build
cmake .. -G Ninja -DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON
cmake --build . -- -j 16

# Build with tests
cmake .. -G Ninja -DENABLE_TESTS=ON -DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON
cmake --build . -- -j 16

# Build with Python bindings
cmake .. -G Ninja -DPYTHON_BINDINGS=ON -DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON
cmake --build . -- -j 16

# Build with static analysis (not supported on macOS+clang)
cmake .. -G Ninja -DSTATIC_ANALYSIS=ON
cmake --build . -- -j 16
```

### Running Tests

```bash
# Run all tests (from build directory)
ctest --output-on-failure

# Run a single test
ctest --output-on-failure -R simplest_pub_sub
```

Tests are defined in `test/CMakeLists.txt`. Each test launches paired publisher/subscriber processes via bash. Test names include: `simplest_pub_sub`, `reliable_pub_sub`, `pub_sub_type_reuse`, `request_response`, `request_response_concurrent`, `ros_interop`, `legacy_api_compat`, `network_recovery`, `discovered_endpoints`, `match_publisher_default`, `discovery_tuning`, `transport_tuning`, `shm_cleanup`, `callback_exceptions`, `point_cloud2`, `accumulation`, `vpn_interfaces`, `listener_drain`, `bounded_wait`, `keyless_topic_history`.

### CI Build Scripts

The `.github/workflows/build.sh` and `.github/workflows/test.sh` scripts are the canonical way CI builds and tests. They use Ninja, default to gcc/g++, and build in a `build/` directory at the repo root.

### Install Dependencies (Linux/macOS)

```bash
sudo ./install_dependencies.sh [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON]
```

## Architecture

### Library Structure

Two shared libraries are produced:

- **provizio_dds_types** — Generated DDS type support code from IDLs (`provizio_dds_idls` repo, fetched via CMake FetchContent). Links to `fastdds` and `fastcdr`.
- **provizio_dds** — The main library with RAII wrappers. Links to `provizio_dds_types`.

### Core C++ API (include/provizio/dds/)

- **`domain_participant.h`** — `make_domain_participant()` creates a shared DomainParticipant. Manages thread-safe type/topic registration with internal mutex.
- **`publisher.h`** — Template `make_publisher<PubSubType>(participant, topic, ...)`. Configurable QoS (reliability, durability, history depth). Match/unmatch callbacks. Header-only.
- **`subscriber.h`** — Template `make_subscriber<PubSubType>(participant, topic, callback)`. Callback takes `(const Data&)` or `(const Data&, const SampleInfo&)` — dispatched via `function_traits.h`. Header-only.
- **`request_response.h`** — `make_service<ReqType, ResType>(...)` and `request<ReqType, ResType>(...)`. Uses correlation tracking via `SampleIdentity`. Dual topics with `_request`/`_response` suffixes.
- **`accumulation.h`** — Point clouds accumulation & multi-radar fusion: `rigid_transform`, core `point_clouds_accumulator` and the DDS-fed `dds_point_clouds_accumulator` (Odometry/NavSatFix/no localization). Mirrors `python/accumulation.py`. Non-template logic is compiled into the library; the maths path (`get_points_*` → `detail/accumulation_math.h`) is header-only and auto-detects Eigen at the consumer's compile time (`PROVIZIO_DDS_DISABLE_EIGEN` opts out; `DISABLE_EIGEN` CMake option for provizio_dds's own builds).
- **`common.h`** — `PROVIZIO_DDS_API` macro for DLL export/import, namespace aliases.
- **`point_cloud2.h`** — Generic + Provizio-radar-specific PointCloud2 reading/writing: `cloud_view` field-driven reading, tiered `create_cloud` writing, `radar_point`/`read_radar_points`/`make_radar_point_cloud`, entity cloud makers + unified read_entities/get_entities_kind. Mirrors `python/point_cloud2.py`. Templates header-only; non-template functions compiled into the library.
- **`qos_defaults.h`** — The per-type QoS defaults template `qos_defaults<PubSubType>` (reliability, publish mode, memory policy and the two KEEP_LAST history depths — reader and writer are configured separately). Specializations for the Provizio fleet-shared types and the large-sample types live here; `src/qos_defaults_checks.cpp` pins every one of them against the real generated types at compile time, and `test/python/python_qos_parity_test.py` checks the Python registration agrees. Documented for consumers under "QoS Defaults per Type" in DETAILS.md.
- **`topic.h`** — RAII `make_topic()` with deduplication (reuses existing topic if same name/type).
- **`detail/vpn_interfaces.h`** / **`src/vpn_interfaces.cpp`** — Identifies VPN / overlay-tunnel interfaces, which are kept out of the DDS transports (and out of network-recovery change detection) by default; `PROVIZIO_DDS_ALLOW_VPN_INTERFACES` opts out. Mirrors the classifier in `python/network_recovery.py`. Exists because Fast-DDS announces an address on every bindable interface and a writer sends every sample to all of a peer's announced locators, so two hosts sharing a LAN that are both on a VPN duplicate all traffic through the tunnel.

### Key Design Patterns

- **Template-based type safety**: Publisher/subscriber are fully templated on `PubSubType`, avoiding runtime type errors. Non-template functions that cross DLL boundaries are marked `PROVIZIO_DDS_API`.
- **Shared ownership**: `make_domain_participant()` returns `shared_ptr<DomainParticipant>`. Publishers/subscribers capture it, ensuring participant lifetime.
- **Function traits dispatch**: `function_traits.h` introspects callback arity at compile time to support both 1-arg `(data)` and 2-arg `(data, sample_info)` handlers.
- **Request/response correlation**: `request_response_details.h` maintains a pending-requests map keyed by `SampleIdentity`. Services echo the request identity back in the response's `related_sample_identity`.

### Python Layer (python/)

- `provizio_dds.py` — Main API wrapping C++ via SWIG-generated `fastdds` and `provizio_dds_python_types` modules.
- `point_cloud2.py` — Point cloud parsing utilities. (Has a C++ counterpart: include/provizio/dds/point_cloud2.h; keep behavior in sync.)
- `accumulation.py` — Multi-radar point cloud accumulation/fusion with odometry. (Has a C++ counterpart: include/provizio/dds/accumulation.h; keep behavior in sync.)
- `gps_utils.py` — GPS/GNSS coordinate utilities.
- `network_recovery.py` — Network-interface monitoring and participant recreation. (C++ counterpart: `src/network_recovery*.cpp`.)
- `shm_cleanup.py` — Reclaims the shared-memory files of participants that died without cleaning up. (C++ counterpart: `src/shm_cleanup.cpp`; keep behavior in sync.)

Python bindings require SWIG 4.0+ and are generated from Fast-DDS-python + provizio_dds_idls `.i` files.

### Windows Support (feature/windows-support branch)

- `PROVIZIO_DDS_API` macro in `common.h`: `__declspec(dllexport)` when building, `__declspec(dllimport)` when consuming. Must be applied to all non-template public symbols.
- `PROVIZIO_DDS_EXPORTS` / `PROVIZIO_DDS_TYPES_EXPORTS` are set via CMake `DEFINE_SYMBOL` property per target.
- `EPROSIMA_ALL_DYN_LINK` enables `__declspec(dllimport)` for eProsima symbols; `EPROSIMA_ALL_NO_LIB` disables MSVC auto-linking `#pragma comment(lib, ...)`.
- MSVC uses versioned library names (e.g., `fastdds-3.6.lib`).
- Python extension modules use `.pyd` on Windows (vs `.so` on Linux/macOS). Each `.pyd` links against a specific `pythonXY.dll`, so every minor Python version needs its own build.

### Dependency Management

Dependencies are auto-downloaded and built by CMake when not found:
- **Fast-DDS** (ExternalProject from `provizio/Fast-DDS` fork)
- **foonathan_memory_vendor** (built in a subprocess during configure)
- **provizio_dds_idls** (FetchContent)

Fast-DDS is patched at build time by the scripts in `cmake/fast_dds/`, run as the ExternalProject `PATCH_COMMAND`: `export_system_info.cmake` (Windows DLL export of `SystemInfo::update_interfaces`, which network recovery calls) and `host_id_without_interfaces.cmake` (a machine-id-derived host id for a process that creates its first participant before any interface has carrier — see "Starting before the network is up" under Network Auto-Recovery in DETAILS.md). Each is idempotent and fails the configure loudly when its anchor in the Fast-DDS sources has moved, so a `FAST_DDS_VERSION` bump must re-check both, and re-check `test/sanitizers/tsan.supp`, whose one `race:` entry covers a Fast-DDS 3.6.2 race fixed upstream in 3.6.3. `LOOK_FOR_FAST_DDS=TRUE` builds against an unpatched system Fast-DDS and gets neither.

A prebuilt binary cache system exists for Linux (x86_64, aarch64) in `cache/`. It is bypassed when `ENABLE_TESTS=ON`, `STATIC_ANALYSIS=ON`, or `IGNORE_BIN_CACHE=ON`. On Linux a cache is only used when the host provides the glibc / libstdc++ ABI level its binaries require, which each cache records in its `abi_requirements` file — see `cmake/bin_cache/host_abi_compatibility.cmake`.

### CMake Options Reference

| Option | Default | Description |
|--------|---------|-------------|
| `ENABLE_TESTS` | OFF | Build and enable CTest tests |
| `PYTHON_BINDINGS` | OFF | Generate SWIG Python bindings |
| `PYTHON_PACKAGES_INSTALL_DIR` | "" | Install directory for Python artifacts (empty uses default sysconfig path) |
| `LOOK_FOR_FAST_DDS` | FALSE | Try system Fast-DDS before building from source |
| `IGNORE_BIN_CACHE` | OFF | Force build from source (skip prebuilt cache) |
| `DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS` | OFF | Disable Provizio coding standards (clang-tidy, formatting) **and the sanitizers Debug builds otherwise enable** — see Conventions |
| `STATIC_ANALYSIS` | OFF | Enable clang-tidy static analysis (requires coding standards checks enabled; not supported on macOS+clang) |
| `INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS` | OFF | Linux: use versioned .so names to avoid runtime conflicts |
| `FAST_DDS_VERSION` | "v3.6.2.0" | Fast-DDS Git tag to build from source |
| `FAST_CDR_VERSION` | "2.3" | Fast-CDR major.minor version for Windows versioned library naming (must match FAST_DDS_VERSION bundle) |
| `DONT_INSTALL_STDCPP_LIBS` | ON | When installing from prebuilt binaries, skip standard C++ libraries |
| `DISABLE_EIGEN` | OFF | Force plain-CPU linear algebra in point clouds accumulation for provizio_dds's own builds/tests even when Eigen3 is installed (consumers choose at their own compile time via Eigen visibility / `PROVIZIO_DDS_DISABLE_EIGEN`) |

## Git Workflow

- **Binary cache push conflicts**: When pushing to a feature branch and the push is rejected because the remote has newer binary cache commits (from CI `commit-cache` jobs), **force-push** (`git push --force`). The cache contains prebuilt binaries for unreleased code and has no value worth preserving — it will be rebuilt by CI on the next run.

## Conventions

- License: Apache 2.0 header required on all source files.
- Coding standards enforced by `provizio/coding_standards` (downloaded at configure time). Disable with `DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON` for faster local iteration.
- **Formatting and static analysis**: All C/C++ code must conform to the repository `.clang-format` (Microsoft style) and `.clang-tidy` configurations. CI enforces these checks and will reject non-conforming code. **MANDATORY: After modifying any C/C++ file, always run `clang-format -i <file>` on it before committing.** Do not rely on manual formatting — the tool must be run to ensure compliance.
- **Sanitizers are ON by default in Debug builds.** The coding standards enable ASan + LSan + UBSan for any `CMAKE_BUILD_TYPE=Debug` build (see `StandardConfig.cmake`); TSan is opt-in via `-DENABLE_TSAN=TRUE` and has its own CI job. `DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON` turns them off along with clang-tidy and the formatting checks. Consequences worth knowing:
  - A Debug `libprovizio_dds.so` is instrumented, so linking it into a **non-instrumented** application produces `ASan runtime does not come first in initial library list` and degrades ASan's own accuracy. Build with `DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON` (as the README's integration snippet does) for a Debug library meant to be consumed by other code.
  - Debug test timeouts are multiplied by `PROVIZIO_DDS_TEST_TIMEOUT_SCALE` (5) because instrumented runs are several times slower.
  - MSan is not used: it needs an instrumented libc++/libstdc++, which this project does not build.
- ROS 2 topic names use the `rt/` prefix convention.
- **Emitted text is ASCII.** Any string literal that can reach a stream at runtime -- log messages, exception text, assertion and test-failure messages -- must contain only ASCII. A Python interpreter on Windows encodes `stdout` with the ANSI code page (cp1252 in CI), so `print` raises `UnicodeEncodeError` on an em dash or an arrow, and the same text routed through `logging` is dropped silently instead. Comments and docstrings are exempt and use the full character set freely; nothing prints them. Where a non-ASCII character IS the payload (test data for a rejected input, say), write it as an escape (`"\u00a0"`) so the source itself stays ASCII. The `runtime_text_is_ascii` test enforces this over `include/`, `src/`, `python/`, `test/` and `cmake/`.
