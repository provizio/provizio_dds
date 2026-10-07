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

Tests are defined in `test/CMakeLists.txt`. Each test launches paired publisher/subscriber processes via bash. Test names include: `simplest_pub_sub`, `reliable_pub_sub`, `pub_sub_type_reuse`, `request_response`, `request_response_concurrent`, `ros_interop`, `legacy_api_compat`, `network_recovery`, `discovered_endpoints`, `match_publisher_default`, `discovery_tuning`, `transport_tuning`, `shm_cleanup`, `callback_exceptions`, `point_cloud2`, `accumulation`, `vpn_interfaces`, `listener_drain`, `bounded_wait`, `keyless_topic_history`, `bin_cache_config_name`, `resolve_ros_base_image` (the CI script choosing the ROS 2 base images, run against stand-ins for `docker`), `install_root` and `fully_qualified_fastdds_libs` (the install steps of provizio_dds's own, which honour `DESTDIR` and the prefix of the install being run), and `swig_lookup` (which SWIG the Python bindings take: the first on the search path, or the one under `SWIG_ROOT`, rather than a distribution's `swig4.0` that older FindSWIG prefers by name). Two further network-recovery cases (`network_recovery_carrier`, `network_recovery_cold_start_hosts`) are registered only with `-DENABLE_PRIVILEGED_TESTS=ON` — see that option.

### CI Build Scripts

The `.github/workflows/build.sh` and `.github/workflows/test.sh` scripts are the canonical way CI builds and tests. They use Ninja, default to gcc/g++, and build in a `build/` directory at the repo root.

The ROS 2 compatibility matrix runs in `ros:<distro>` containers taken from this organisation's GHCR mirror (`ghcr.io/provizio/ros`, kept current by the "Mirror ROS base images" workflow of `provizio_radar_api_ros2`), because Docker Hub rate-limits unauthenticated pulls per source IP and a hosted runner's IP is shared. A job's container is pulled before any of its steps run, so the choice cannot be made inside that job: `.github/workflows/resolve_ros_base_image.sh` runs in the `resolve-ros-base-image` job ahead of it and outputs a distro → image map the container line indexes, plus the distro list the matrix itself is built from (so the list lives in one place). Mirror images are pinned to the digest it resolved, so every job of a run pulls the identical image. Anything the mirror cannot serve — no such tag, or no `linux/amd64` in the manifest — falls back to `mirror.gcr.io/library` and raises a workflow warning; never silently, or a broken mirror would go unnoticed until Docker Hub throttled the matrix again. Setting the `CONTAINER_REGISTRY_PREFIX` variable redirects the mirror **and clears the fallback**, so an air-gapped or policy-restricted setup cannot be quietly redirected back at Docker Hub; a distro that registry cannot serve is then handed to its jobs as the registry's own tag, unpinned, with a warning, so that it fails its own four jobs rather than the resolver job every one of the twenty needs. A registry read is retried unless its answer is one (not found, denied), so that one 5xx does not send a distro to the fallback, and a prefix no image reference can be made from — or one naming no registry, whose first component Docker would read as a Docker Hub namespace (Docker Hub itself is `docker.io/<namespace>`) — fails the resolver outright, every distro being affected alike. The script validates every distro and repository name it emits, because those reach a container reference, a `GITHUB_OUTPUT` line and `run:` blocks that interpolate `matrix.ros` into shell. No `credentials:` block is needed on the container: the runner fills them in from `github.actor` / `GITHUB_TOKEN` itself for a `ghcr.io` image when none are given, which is why the job only needs `packages: read` — and why a static block would be wrong, as it would be tried against the fallback's registry too. The image being pinned, the demo nodes the interop tests run are installed by `.github/workflows/install_ros_demo_nodes.sh`, which upgrades the image's own ROS packages in the same transaction: the ROS apt repository serves the day's sync, whose packages depend on one another without versions and do not link against an older image's (the talker then fails to start on an undefined `has_buffer_fields_*` symbol). So the ROS 2 packages under test are the repository's current sync, not the image's: the digest pins the base system, and a re-run can meet newer ROS packages than the last one did.

### Install Dependencies (Linux/macOS)

```bash
sudo ./install_dependencies.sh [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON]
```

Ubuntu 20.04 or newer: 18.04 is no longer supported, and the script refuses it.

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

**OpenSSL** is not built: `find_package(OpenSSL REQUIRED)` finds it for the Fast-DDS built here (its TLS transport, and its security plugin when on), which is made to link that same OpenSSL, and to require it, rather than look for one itself: left to that, it takes whichever it comes across (on Windows an installer's, whose DLLs a machine the binaries are deployed to does not have), or builds without TLS. The Fast-DDS sub-build does not get this project's toolchain, so it is handed what was found: when OpenSSL came as a package (`OpenSSL_CONFIG` set — Conan's, say), a generated `FindOpenSSL.cmake` (`cmake/fast_dds/FindOpenSSL.cmake.in`) loading that package's configuration, whose dependencies (a static OpenSSL's zlib) nothing else knows of, with the package search settings and the configuration (build type) provizio_dds consumes it in applying to that lookup alone (a Fast-DDS built as Release for the release CRT takes the package's Release configuration); otherwise FindOpenSSL's root and results, the library files among them (with MSVC, its `LIB_EAY_*` / `SSL_EAY_*` entries). An `OPENSSL_USE_STATIC_LIBS` given to provizio_dds is given to Fast-DDS too. The shared OpenSSL libraries Fast-DDS then loads go next to its own (`cmake/fast_dds/openssl_runtime.cmake`, a step after its install, covered by the `fast_dds_openssl_runtime` test), from where the Python package and a consumer's build tree take them along as they are, and an install on Linux or macOS into `lib/provizio_dds` (`PROVIZIO_DDS_PRIVATE_LIB_DIR`), never `lib/` itself: an install prefix's `lib/` is often on every program's library search path (`/usr/local/lib` is, on most distributions), and an OpenSSL of ours there would be loaded in place of the system's by every program without an rpath. Every install of the Fast-DDS built here or prebuilt also removes from there any OpenSSL an earlier install into the same prefix left that it does not install itself -- of a build given another OpenSSL since, or the system's -- as Fast-DDS would load that one from there ahead of the one now chosen (`cmake/install_openssl_runtime.cmake`, covered by the `openssl_runtime_install` test). Fast-DDS finds it there through the `$ORIGIN/provizio_dds` (`@loader_path/provizio_dds`) entry of the install rpath it is built with -- its first, ahead of `$ORIGIN`, as a `lib/` shared with other software can hold another OpenSSL of the same name -- which Fast-DDS keeps on macOS only thanks to `install_rpath_as_given.cmake`, checked on the library built by `fast_dds_install_rpath` -- and the prebuilt binaries, whose Linux caches carry the build machine's OpenSSL, get the same entry from `build_cache.sh`, so the two must name the same directory; `test-nix-bin-cache-install` installs them and checks where OpenSSL went and that Fast-DDS loads it from there. The step's inputs, the found OpenSSL's libraries, are globbed with `CONFIGURE_DEPENDS`, so an OpenSSL upgraded or removed since the configure makes the build configure again instead of failing on a missing input. Which ones is read from what its installed libraries import: the OpenSSL DLLs on Windows; on Linux the SONAMEs they need, but those found in the system's directories, where the loader looks anyway; on macOS those they name by an install name relative to their rpath (`@rpath/`) or to themselves (`@loader_path/`, which the step makes `@rpath/` in Fast-DDS's libraries with `install_name_tool`, as an install puts the two apart, signing again ad hoc any whose signature then no longer holds), not one named by an absolute path (Homebrew's), which dyld loads from there. They are copied from the found OpenSSL's directories. Anywhere else the loader could come across another OpenSSL of the same name first — the distribution's, of another version. A consumer chooses the OpenSSL, and whether it is shared, with its own toolchain, `OPENSSL_ROOT_DIR` or `OPENSSL_USE_STATIC_LIBS`. The `FindOpenSSL.cmake` generated for an OpenSSL found as a package (`cmake/fast_dds/openssl_package.cmake`) runs the package's own lookups of its dependencies with every package location provizio_dds was given or found (a `<Package>_DIR` holding a package configuration, a `<Package>_ROOT`), its `CMAKE_MODULE_PATH` as well as its `CMAKE_PREFIX_PATH`, inside a function so that none of it leaks into the rest of the Fast-DDS configure; with every lookup result provizio_dds holds and every value given on its command line that nothing declared (its `FILEPATH`, `PATH` and `UNINITIALIZED` cache entries, `NOTFOUND` and empty ones included) written into the Fast-DDS build's cache for that lookup and taken out or put back as that build had them after it; and removes what the lookup cached -- so that a dependency cached by an earlier configure cannot win over the one provizio_dds finds now. The names that cache held, and the entries it overwrites, are journalled in that cache first (`cmake/fast_dds/openssl_lookup_journal.cmake`): a configure stopping inside the lookup leaves them there, and every configure of the Fast-DDS build that provizio_dds runs is given `cmake/fast_dds/openssl_lookup_recovery.cmake` first (`-C`, `PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY`), whatever OpenSSL it is given, which puts the cache back from them ahead of its command line and of any of Fast-DDS's code (one run otherwise has the generated `FindOpenSSL.cmake` do it, later). All of those are in the module's text, so one first cached later in provizio_dds's configure makes the second configure of a new build tree configure Fast-DDS once more. `fast_dds_openssl_package` covers it with stand-in packages.

Fast-DDS is patched at build time by the scripts in `cmake/fast_dds/`, run as the ExternalProject `PATCH_COMMAND`: `export_system_info.cmake` (Windows DLL export of `SystemInfo::update_interfaces`, which network recovery calls), `host_id_without_interfaces.cmake` (a machine-id-derived host id for a process that creates its first participant before any interface has carrier — see "Starting before the network is up" under Network Auto-Recovery in DETAILS.md), `resource_event_per_timer_wait.cmake` (fixes a Fast-DDS deadlock between destroying/recreating a `TimedEvent` while holding an endpoint mutex and the timer thread reaping an expired participant lease — eProsima/Fast-DDS#6502; it made tests hang for their whole ctest TIMEOUT), `topic_payload_pool_registry_lock_first.cmake` (fixes a null payload pool handed to `create_datareader` when an endpoint on the same topic is destroyed concurrently — a SegFault), `local_reader_under_writer_mutex.cmake` (upstream's own eProsima/Fast-DDS#6422, merged for v3.6.3.0: a writer's intraprocess heartbeat read its same-process reader on the timer thread without the writer mutex while deleting that reader or the writer reset it — a SegFault; guarded by the `fast_dds_patch_local_reader_lock` test) and `install_rpath_as_given.cmake` (Fast-DDS replaces the install rpath it is given with the build tree's absolute install directory on macOS, so libfastdds found its `@rpath` dependencies, the OpenSSL placed next to it among them, in that build tree only; guarded by `fast_dds_install_rpath`). They are listed once, each with the files it edits, in `PROVIZIO_DDS_FAST_DDS_PATCHES` of the top-level CMakeLists.txt: the `PATCH_COMMAND`, the patch step's re-run on a script change and the line-endings test below are all made from that list. Each is idempotent and fails the configure loudly when its anchor in the Fast-DDS sources has moved, so a `FAST_DDS_VERSION` bump must re-check all six — and drop `local_reader_under_writer_mutex.cmake` once the version carries #6422, which its failure message then says outright. **Changing a patch script's own replacement text is a separate hazard from a version bump**: an existing build tree carries a marker saying "patched" but no record of by which revision, so the old text keeps shipping while the script reports a no-op. `resource_event_per_timer_wait.cmake` carries a `_revision` / `_revision_marker` pair and a migration for that reason, covered by the `fast_dds_patch_timer_migration` test; the other five do not, because their text has never changed, and each says at its marker what to add before changing it. Such a migration must decide from what the file *contains*, never from which marker it carries. Every script reads and writes the sources through `cmake/fast_dds/patch_io.cmake`, never with a bare `file(READ)` / `file(WRITE)`: a checkout may have either line ending (Fast-DDS marks its sources `text`, and provizio_dds's own `.gitattributes` sets no `text` or `eol`), and `file(WRITE)` on Windows turns a CRLF into CR CR LF. Its read yields LF text and its write leaves the file in the host's own line ending throughout, verified before it replaces the original; `fast_dds_patch_line_endings` applies every script to LF and CRLF sources, from LF and CRLF copies of itself, and covers every script listed in `PROVIZIO_DDS_FAST_DDS_PATCHES`, with the files the build gives it. The patch tests are registered wherever Fast-DDS is built from source here and nowhere else, and the configure fails if that tree registers none. `LOOK_FOR_FAST_DDS=TRUE` builds against an unpatched system Fast-DDS and gets none of them — which is why CI's `test-preinstalled-fastdds`, the one job that builds that way (and checks that it did), runs a smoke test only (`PROVIZIO_DDS_CTEST_INCLUDE` in `test.sh`): the full suite would keep meeting the defects those patches fix. A pip package (`PYTHON_PIP_PACKAGE`, which `setup.py` sets, as do the cache builders for the Python binaries it unpacks) refuses the option outright, ahead of `project()`, and forgets the refused value, so that configuring again without it is not refused as well: the package always bundles provizio_dds's own Fast-DDS and loads it from there, so a system one would be neither patched nor packaged. Its Fast-DDS takes foonathan_memory from the vendored build only, too: a system or ROS copy found first is a shared library the wheel would depend on without containing. The Python bindings installed as packages against a system Fast-DDS without `PYTHON_PIP_PACKAGE`, as a distribution's recipe may, are left alone (`pip_package_refuses_system_fast_dds`, `pip_package_vendored_foonathan_memory`). `test-nix-pip-package-from-source` is the one Linux pip job building from source, forced as a user forces it, through `CMAKE_ARGUMENTS`; every other one takes the prebuilt binaries and fails unless it did.

A prebuilt binary cache system exists for Linux (x86_64, aarch64) in `cache/`, built on Ubuntu 22.04 and on the `jetson-20.04` runners (Ubuntu 20.04, with its libstdc++ 6.0.28, which `build_cache.sh` insists on) respectively. The binaries carry the OpenSSL their build found, so the jetson-20.04 runners (ghr-001..005) each hold an OpenSSL 3 of their own in `/opt/openssl-3`, beside Ubuntu 20.04's 1.1.1 -- installed by `.github/workflows/install_runner_openssl.sh`, which pins its version (the 3.5 LTS series) and is run again on every runner to move to another -- and the aarch64 jobs of CI set `OPENSSL_ROOT_DIR` to it; `build_cache.sh` refuses to build on aarch64 without an OpenSSL 3 there, or with 1.1 in what it built. It is bypassed when `ENABLE_TESTS=ON`, `STATIC_ANALYSIS=ON`, `IGNORE_BIN_CACHE=ON`, or `PYTHON_BINDINGS=ON` without a `PYTHON_PACKAGES_INSTALL_DIR`. On Linux a cache is only used when the host provides the glibc / libstdc++ ABI level its binaries require, which each cache records in its `abi_requirements` file — see `cmake/bin_cache/host_abi_compatibility.cmake`. The key itself comes from `bin_cache_config_name.sh` (`bin_cache_config_name.ps1` on Windows) and names the architecture, a hash of every tracked file, the IDLs revision and the build type — so a miss is normal for a Debug build or a modified checkout, and a configure that would have used a cache says which key it looked for whenever the archive is absent. The bypassed configurations above stay silent by design — they build from source whatever they find, so naming a key would only send the reader after a cache that was never going to be used — which is why three of the four build commands above print no such line, and the plain one does. The architecture must come from `uname -m`: Linux populates no hardware-platform or processor field, so `uname -i` / `-p` answer `unknown` under uutils coreutils (Ubuntu 26.04's default) and silently turn every key into `linux_unknown.*`. The `bin_cache_config_name` test pins both halves of that. A configure reports why it took no cache: the archive absent -- but not when an earlier configure extracted the one of this key, which is then used, as a checkout of the commit before CI's cache commit does -- the key not worked out, an archive that could not be extracted (the tool missing or failing, what it left behind removed), or one that lacks a file the build hands on -- the two libraries, the Fast-DDS stack they load (by its versioned names) and the Python extension module (`cmake/bin_cache/missing_file.cmake`, covered by `bin_cache_missing_file`), and for a pip package everything of its Python packages it needs to import, which `setup.py` checks itself.

### CMake Options Reference

| Option | Default | Description |
|--------|---------|-------------|
| `ENABLE_TESTS` | OFF | Build and enable CTest tests |
| `ENABLE_PRIVILEGED_TESTS` | OFF | Also register the two tests that need root / passwordless sudo (`network_recovery_carrier`, `network_recovery_cold_start_hosts`): they create network namespaces. Off by default so that selecting tests cannot reach them — a test chosen by regex cannot be excluded by name, so not registering it is the only reliable answer. It does not make `ctest` itself safe: `ctest -S script.cmake` and `--build-and-test … --test-command` run arbitrary commands whatever is registered. CI turns it on explicitly in the `test-nix` matrix, which is the only place these two run |
| `PYTHON_BINDINGS` | OFF | Generate SWIG Python bindings |
| `PYTHON_PACKAGES_INSTALL_DIR` | "" | Install directory for Python artifacts (empty uses default sysconfig path) |
| `PYTHON_PIP_PACKAGE` | OFF | Build a pip package, as `setup.py` does: bundles provizio_dds's own Fast-DDS with everything it loads (foonathan_memory from the vendored build only) and refuses `LOOK_FOR_FAST_DDS`. Needs `PYTHON_BINDINGS` and `PYTHON_PACKAGES_INSTALL_DIR` |
| `LOOK_FOR_FAST_DDS` | FALSE | Try system Fast-DDS before building from source (it gets none of the `cmake/fast_dds/` patches; refused for a pip package, `PYTHON_PIP_PACKAGE`) |
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
- **Paths in globs are escaped.** A checkout, build or install directory may hold `[`, `*` or `?` in its name, which a glob reads as part of its pattern and then matches nothing, in silence. A CMake `file(GLOB)` takes such a path through `provizio_dds_glob_escape` (`cmake/glob_escape.cmake`, covered by `glob_escape`), and Python through `glob.escape`.
- **Emitted text is ASCII.** Any string literal that can reach a stream at runtime -- log messages, exception text, assertion and test-failure messages -- must contain only ASCII. A Python interpreter on Windows encodes `stdout` with the ANSI code page (cp1252 in CI), so `print` raises `UnicodeEncodeError` on an em dash or an arrow, and the same text routed through `logging` is dropped silently instead. Comments and docstrings are exempt and use the full character set freely; nothing prints them. Where a non-ASCII character IS the payload (test data for a rejected input, say), write it as an escape (`"\u00a0"`) so the source itself stays ASCII. The `runtime_text_is_ascii` test enforces this over `include/`, `src/`, `python/`, `test/` and `cmake/`.
