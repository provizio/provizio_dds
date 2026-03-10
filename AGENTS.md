# AGENTS.md

This file provides guidance to Claude Code and other AI agents when working with code in this repository.

## Project Overview

provizio_dds is a C++17/Python DDS communication library built on eProsima Fast-DDS (v2.14.2). It provides RAII-based pub/sub and request/response abstractions compatible with ROS 2 (Humble+ for pub/sub, Jazzy+ for request/response). Licensed under Apache 2.0.

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

Tests are defined in `test/CMakeLists.txt`. Each test launches paired publisher/subscriber processes via bash. Test names: `simplest_pub_sub`, `reliable_pub_sub`, `pub_sub_type_reuse`, `request_response`, `request_response_concurrent`, `ros_interop`.

### CI Build Scripts

The `.github/workflows/build.sh` and `.github/workflows/test.sh` scripts are the canonical way CI builds and tests. They use Ninja, default to gcc/g++, and build in a `build/` directory at the repo root.

### Install Dependencies (Linux/macOS)

```bash
sudo ./install_dependencies.sh [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON]
```

## Architecture

### Library Structure

Two shared libraries are produced:

- **provizio_dds_types** — Generated DDS type support code from IDLs (`provizio_dds_idls` repo, fetched via CMake FetchContent). Links to `fastrtps` and `fastcdr`.
- **provizio_dds** — The main library with RAII wrappers. Links to `provizio_dds_types`.

### Core C++ API (include/provizio/dds/)

- **`domain_participant.h`** — `make_domain_participant()` creates a shared DomainParticipant. Manages thread-safe type/topic registration with internal mutex.
- **`publisher.h`** — Template `make_publisher<PubSubType>(participant, topic, ...)`. Configurable QoS (reliability, durability, history depth). Match/unmatch callbacks. Header-only.
- **`subscriber.h`** — Template `make_subscriber<PubSubType>(participant, topic, callback)`. Callback takes `(const Data&)` or `(const Data&, const SampleInfo&)` — dispatched via `function_traits.h`. Header-only.
- **`request_response.h`** — `make_service<ReqType, ResType>(...)` and `request<ReqType, ResType>(...)`. Uses correlation tracking via `SampleIdentity`. Dual topics with `_request`/`_response` suffixes.
- **`common.h`** — `PROVIZIO_DDS_API` macro for DLL export/import, namespace aliases.
- **`qos_defaults.h`** — `apply_qos_defaults()` configures QoS policies (reliability, durability, history, memory).
- **`topic.h`** — RAII `make_topic()` with deduplication (reuses existing topic if same name/type).

### Key Design Patterns

- **Template-based type safety**: Publisher/subscriber are fully templated on `PubSubType`, avoiding runtime type errors. Non-template functions that cross DLL boundaries are marked `PROVIZIO_DDS_API`.
- **Shared ownership**: `make_domain_participant()` returns `shared_ptr<DomainParticipant>`. Publishers/subscribers capture it, ensuring participant lifetime.
- **Function traits dispatch**: `function_traits.h` introspects callback arity at compile time to support both 1-arg `(data)` and 2-arg `(data, sample_info)` handlers.
- **Request/response correlation**: `request_response_details.h` maintains a pending-requests map keyed by `SampleIdentity`. Services echo the request identity back in the response's `related_sample_identity`.

### Python Layer (python/)

- `provizio_dds.py` — Main API wrapping C++ via SWIG-generated `fastdds` and `provizio_dds_python_types` modules.
- `point_cloud2.py` — Point cloud parsing utilities.
- `accumulation.py` — Multi-radar point cloud accumulation/fusion with odometry.
- `gps_utils.py` — GPS/GNSS coordinate utilities.

Python bindings require SWIG 4.0+ and are generated from Fast-DDS-python + provizio_dds_idls `.i` files.

### Windows Support (feature/windows-support branch)

- `PROVIZIO_DDS_API` macro in `common.h`: `__declspec(dllexport)` when building, `__declspec(dllimport)` when consuming. Must be applied to all non-template public symbols.
- `PROVIZIO_DDS_EXPORTS` / `PROVIZIO_DDS_TYPES_EXPORTS` are set via CMake `DEFINE_SYMBOL` property per target.
- `EPROSIMA_ALL_DYN_LINK` enables `__declspec(dllimport)` for eProsima symbols; `EPROSIMA_ALL_NO_LIB` disables MSVC auto-linking `#pragma comment(lib, ...)`.
- MSVC uses versioned library names (e.g., `fastrtps-2.14.lib`).
- Python extension modules use `.pyd` on Windows (vs `.so` on Linux/macOS). Each `.pyd` links against a specific `pythonXY.dll`, so every minor Python version needs its own build.

### Dependency Management

Dependencies are auto-downloaded and built by CMake when not found:
- **Fast-DDS** (ExternalProject from `provizio/Fast-DDS` fork)
- **foonathan_memory_vendor** (built in a subprocess during configure)
- **provizio_dds_idls** (FetchContent)

A prebuilt binary cache system exists for Linux (x86_64, aarch64) in `cache/`. It is bypassed when `ENABLE_TESTS=ON`, `STATIC_ANALYSIS=ON`, or `IGNORE_BIN_CACHE=ON`.

### CMake Options Reference

| Option | Default | Description |
|--------|---------|-------------|
| `ENABLE_TESTS` | OFF | Build and enable CTest tests |
| `PYTHON_BINDINGS` | OFF | Generate SWIG Python bindings |
| `PYTHON_PACKAGES_INSTALL_DIR` | "" | Install directory for Python artifacts (empty uses default sysconfig path) |
| `LOOK_FOR_FAST_DDS` | FALSE | Try system Fast-DDS before building from source |
| `IGNORE_BIN_CACHE` | OFF | Force build from source (skip prebuilt cache) |
| `DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS` | OFF | Disable Provizio coding standards (clang-tidy, formatting) |
| `STATIC_ANALYSIS` | OFF | Enable clang-tidy static analysis (requires coding standards checks enabled; not supported on macOS+clang) |
| `INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS` | OFF | Linux: use versioned .so names to avoid runtime conflicts |
| `FAST_DDS_VERSION` | "v2.14.2" | Fast-DDS Git tag to build from source |
| `FAST_CDR_VERSION` | "2.2" | Fast-CDR major.minor version for Windows versioned library naming (must match FAST_DDS_VERSION bundle) |
| `DONT_INSTALL_STDCPP_LIBS` | ON | When installing from prebuilt binaries, skip standard C++ libraries |

## Git Workflow

- **Binary cache push conflicts**: When pushing to a feature branch and the push is rejected because the remote has newer binary cache commits (from CI `commit-cache` jobs), **force-push** (`git push --force`). The cache contains prebuilt binaries for unreleased code and has no value worth preserving — it will be rebuilt by CI on the next run.

## Conventions

- License: Apache 2.0 header required on all source files.
- Coding standards enforced by `provizio/coding_standards` (downloaded at configure time). Disable with `DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON` for faster local iteration.
- **Formatting and static analysis**: All C/C++ code must conform to the repository `.clang-format` (Microsoft style) and `.clang-tidy` configurations. CI enforces these checks and will reject non-conforming code. **MANDATORY: After modifying any C/C++ file, always run `clang-format -i <file>` on it before committing.** Do not rely on manual formatting — the tool must be run to ensure compliance.
- Runtime sanitizers (ASan/TSan/MSan) are explicitly disabled due to known Fast-DDS issues.
- ROS 2 topic names use the `rt/` prefix convention.
