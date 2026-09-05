# provizio_dds

C++ and Python library for [DDS communication](https://www.dds-foundation.org/what-is-dds-3/) in Provizio customer-facing APIs and internal Provizio software components. Built using
[eProsima Fast-DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds) DDS implementation (Apache
License 2.0).

Although based directly on DDS, it's compatible with [ROS2](https://docs.ros.org/en/rolling/) and provides all ROS2
built-in data types.

This README is the guide to using the library. Behaviour and configuration that most callers never
need to touch — network auto-recovery, transports, discovery tuning, logging, XML profiles — is in
[DETAILS.md](DETAILS.md).

| CI Status: master | CI Status: develop |
| ----------------- | ----------------- |
| ![CI Status: master](https://github.com/provizio/provizio_dds/actions/workflows/ci.yml/badge.svg?branch=master) | ![CI Status: develop](https://github.com/provizio/provizio_dds/actions/workflows/ci.yml/badge.svg?branch=develop) |

## Provizio DDS API

[Table of DDS topics and their respective data types used in Provizio API](https://github.com/provizio/provizio_dds_idls/blob/master/TOPICS.md)

## Build dependencies

**C++ (Linux / macOS):**

- CMake (>= 3.15)
- Git
- C++ 17 compiler (gcc, clang, or MSVC)
- libssl-dev (OpenSSL development headers)
- eProsima Fast-DDS 3.x (provizio_dds builds against the Fast-DDS 3.x API): when a Fast-DDS installation is present it will be used, otherwise it's downloaded and built automatically. Note that the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable was renamed to `FASTDDS_DEFAULT_PROFILES_FILE` in Fast-DDS 3.x (see [XML Profiles](DETAILS.md#xml-profiles)).

**C++ (Windows):**

- Visual Studio 2019+ (MSVC) with C++ 17 support
- CMake (>= 3.15)
- Git
- Ninja build system
- OpenSSL (install via `install_dependencies.ps1` or manually provide headers/libs)
- SWIG 4.0+ (SWIG 4.4+ required when using Python 3.14 or newer; only if building Python bindings)

**Python (Linux / macOS):**

- Git
- Python 3
- Pip 3
- unzip

**Python (Windows):**

- Git
- Python 3
- Pip 3
- All C++ (Windows) dependencies above (pip install builds from source)

When using non-binary-prebuilt configurations (i.e. any macOS, Linux Debug, non-x64/non-aarch64, non-default provizio_dds_idls) all the C++ dependencies will also be required.

There is a convenience Bash script to install all dependencies in *apt*-featuring Linux and macOS. In Linux it's to be executed with root privileges, f.e. using `sudo`.

**Windows:** Use the PowerShell script from an elevated (Administrator) prompt:

```PowerShell
.\install_dependencies.ps1 [-WithPython ON|OFF]
```

This installs OpenSSL (if not already available), SWIG (when Python is enabled), and other required tools. When static analysis is enabled, it also installs LLVM and cppcheck.

Use as:

```Bash
<PATH_TO_PROVIZIO_DDS>/install_dependencies.sh [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON] [INSTALL_ROS=OFF|ON] [FAST_DDS_INSTALL=OFF|ON|install_path]
```

Or download and execute (assuming `curl` is present):

```Bash
curl -s https://raw.githubusercontent.com/provizio/provizio_dds/<BRANCH_OR_TAG>/install_dependencies.sh | [sudo] bash -s [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON] [INSTALL_ROS=OFF|ON] [FAST_DDS_INSTALL=OFF|ON|install_path]
```

For example, for Python-enabled builds from non-root Linux user:

```Bash
curl -s https://raw.githubusercontent.com/provizio/provizio_dds/master/install_dependencies.sh | sudo bash -s ON
```

## Importing

**C++ (CMake):**

```CMake
# Resolve provizio_dds (https://github.com/provizio/provizio_dds)
set(PROVIZIO_DDS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds_build")
set(PROVIZIO_DDS_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds")
set(PROVIZIO_DDS_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
set(PROVIZIO_DDS_GITHUB_PROJECT "provizio/provizio_dds")
set(PROVIZIO_DDS_GITHUB_BRANCH "master") # Or a specific tag or branch you prefer
set(PROVIZIO_DDS_INSTALL_DIR "${PROVIZIO_DDS_BINARY_DIR}/install")
find_package(Git REQUIRED)
ExternalProject_Add(libprovizio_dds
    # Cloned by hand rather than with GIT_REPOSITORY/GIT_SHALLOW, which would also fetch the
    # tip of every other branch and tag — see "Cloning only what you need" in DETAILS.md. The
    # remove_directory keeps the step re-runnable: git refuses a non-empty destination, which
    # a second run (after changing the branch above, say) would otherwise hit.
    DOWNLOAD_COMMAND "${CMAKE_COMMAND}" -E remove_directory "${PROVIZIO_DDS_SOURCE_DIR}"
             COMMAND "${GIT_EXECUTABLE}" clone --depth 1 --single-branch
                     --branch "${PROVIZIO_DDS_GITHUB_BRANCH}"
                     "https://github.com/${PROVIZIO_DDS_GITHUB_PROJECT}.git"
                     "${PROVIZIO_DDS_SOURCE_DIR}"
    UPDATE_COMMAND ""
    PREFIX "${PROVIZIO_DDS_PREFIX}"
    SOURCE_DIR "${PROVIZIO_DDS_SOURCE_DIR}"
    BINARY_DIR "${PROVIZIO_DDS_BINARY_DIR}"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${PROVIZIO_DDS_INSTALL_DIR}" "-DENABLE_TESTS=OFF" "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON"
)
add_dependencies(<YOUR_CMAKE_TARGET> libprovizio_dds)
target_include_directories(<YOUR_CMAKE_TARGET> SYSTEM PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/include")
target_link_directories(<YOUR_CMAKE_TARGET> PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/lib")
if(WIN32)
    # On Windows the installed import libraries are versioned (e.g.
    # fastdds-3.6.lib, fastcdr-2.3.lib). The ExternalProject above is a
    # standalone CMake build, so its internal unversioned imported targets
    # don't propagate here — link the versioned names directly. Replace
    # the trailing numbers if you pinned a different Fast-DDS version.
    target_link_libraries(<YOUR_CMAKE_TARGET> PUBLIC provizio_dds provizio_dds_types fastdds-3.6 fastcdr-2.3)
else()
    target_link_libraries(<YOUR_CMAKE_TARGET> PUBLIC provizio_dds provizio_dds_types fastdds fastcdr)
endif()
```

`DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON` above is not only about skipping clang-tidy: provizio_dds's coding standards also enable **ASan, LSan and UBSan for any `Debug` build**. That is right for working on provizio_dds itself, but an instrumented `libprovizio_dds` linked into your own non-instrumented application warns `ASan runtime does not come first in initial library list` and makes ASan's own findings unreliable. Keep the flag on for a library you intend to consume; drop it only when you deliberately want the sanitizers.

**Python (pip):**

```Bash
python3 -m pip install -v git+https://github.com/provizio/provizio_dds.git
```

or

```Bash
python3 -m pip install -v git+https://github.com/provizio/provizio_dds.git@TAG_or_BRANCH
```

On Windows, use `python` instead of `python3`:

```PowerShell
python -m pip install -v git+https://github.com/provizio/provizio_dds.git
```

## Publishing Data

**C++ Example:**

```C++
#include "provizio/dds/publisher.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>

int main()
{
    // Make a DDS Publisher
    auto publisher = provizio::dds::make_publisher<
        std_msgs::msg::StringPubSubType>(           // DDS Pub/Sub Type
        provizio::dds::make_domain_participant(),   // DDS Domain Participant
        "rt/chatter"                                // DDS Topic Name
    );

    // Create a message
    std_msgs::msg::String str;
    str.data("Hello World!");

    // Publish the message
    publisher->publish(str);

    return 0;
}
```

For more details see [provizio/dds/publisher.h](include/provizio/dds/publisher.h).

**Python Example:**

```Python
import provizio_dds

# Make a DDS Publisher
publisher = provizio_dds.Publisher(
    provizio_dds.make_domain_participant(), # DDS Domain Participant
    "rt/chatter",                           # DDS Topic Name
    provizio_dds.StringPubSubType)          # DDS Pub/Sub Type

# Create a message
message = provizio_dds.String()
message.data("Hello World!")

# Publish the message
publisher.publish(message)
```

For more details see [python/provizio_dds.py](python/provizio_dds.py) and [test/python/python_publisher.py](test/python/python_publisher.py).

**Notes:**

- Publishers support configurable reliability QoS (BEST_EFFORT or RELIABLE; RELIABLE by default). See `provizio::dds::make_publisher` (C++) or `provizio_dds.Publisher` (Python) for parameters.
- Durability and history depth are now configured **independently** (they used to be a single combined parameter):
  - `durability_kind` selects the DDS durability QoS — pass `TRANSIENT_LOCAL_DURABILITY_QOS` for late-joiner delivery or `VOLATILE_DURABILITY_QOS` to force volatile. Leave it unset (`std::nullopt` in C++, omit / `None` in Python) to keep the Fast-DDS / XML-profile default.
  - `history_depth` controls the KEEP_LAST history depth only. Pass `use_default_history_depth` (`-1`; `USE_DEFAULT_HISTORY_DEPTH` in Python), or any non-positive value, to keep the default depth (the per-type *writer* default where one exists — see the note below — otherwise Fast-DDS's default); pass a positive value for an explicit KEEP_LAST depth.
- **Per-type QoS defaults.** Publish mode, writer reliability and the KEEP_LAST history depths are defaulted per message type, with the **writer and reader depths configured separately** (they do different jobs, usually on different machines). Large-sample types publish ASYNCHRONOUSly so a multi-megabyte write hands off to the participant's async sender thread; the types every sensor publishes to one shared, keyless topic get a deep reader history, because on such a topic the depth is spent by the whole fleet rather than granted to each sensor; and raw `sensor_msgs::msg::Image` defaults to a **BEST_EFFORT** writer, a deliberate breaking change for ROS 2 subscribers. The full table and the reasoning behind every number are in [DETAILS.md](DETAILS.md#qos-defaults-per-type) — read the raw-image note there before upgrading if you consume `rt/provizio_camera` or `rt/provizio_freespace` from ROS 2.
- You can optionally receive subscriber match/unmatch notifications. See `provizio::dds::make_publisher` (C++) or `provizio_dds.Publisher` (Python) for parameters.

## Receiving Data

**C++ Example:**

```C++
#include "provizio/dds/subscriber.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>
#include <iostream>

int main()
{
    // Make a DDS Subscriber
    const auto subscriber = provizio::dds::make_subscriber<
        std_msgs::msg::StringPubSubType>(           // DDS Pub/Sub Type
        provizio::dds::make_domain_participant(),   // DDS Domain Participant
        "rt/chatter",                               // DDS Topic Name
        [&](const std_msgs::msg::String &message) { // Message handler (takes DDS Data Type as a const reference)
            // Print the received message
            std::cout << message.data() << std::endl;
        });
    std::cin.get(); // Wait for any user input

    return 0;
}
```

For more details see [provizio/dds/subscriber.h](include/provizio/dds/subscriber.h).

**Python Example:**

```Python
import provizio_dds

subscriber = provizio_dds.Subscriber(
    provizio_dds.make_domain_participant(), # DDS Domain Participant
    "rt/chatter",                           # DDS Topic Name
    provizio_dds.StringPubSubType,          # DDS Pub/Sub Type
    provizio_dds.String,                    # DDS Data Type
    lambda message: print(message.data()))  # Message handler (takes a DDS Data Type object), prints the received message
input("Press Enter to continue...") # Wait for any user input
```

For more details see [python/provizio_dds.py](python/provizio_dds.py) and [test/python/python_subscriber.py](test/python/python_subscriber.py).

**Notes:**

- The data callback can take either one argument (the data) or two (data and `SampleInfo`). Both are supported in C++ and Python bindings.
- **By default a subscriber adopts the discovered publisher's reliability.** Rather than creating its DataReader eagerly, it defers creation until a matching remote DataWriter is discovered on its topic, then builds the reader with that writer's offered reliability (the first-discovered writer's reliability is adopted while it stays live; if it later leaves and only a differently-configured writer remains, a newly-created default subscriber re-derives the reliability from a still-live writer so it still matches — while an already-built reader keeps its own reliability for its lifetime). A default subscriber is therefore automatically reliable against a RELIABLE publisher and best-effort against a best-effort one, with no configuration and without the reliability mismatch that would otherwise prevent matching. Pass an explicit `reliability_kind` (`BEST_EFFORT_RELIABILITY_QOS` / `RELIABLE_RELIABILITY_QOS`) to opt out and get an eagerly-created reader with that fixed reliability (the previous behaviour).
- Durability and history depth are configured **independently** (as for publishers above): `durability_kind` selects the durability QoS (unset keeps the default), and `max_history_depth` controls only the KEEP_LAST history depth (`-1` / non-positive keeps the per-type *reader* default; a positive value sets an explicit depth). That reader default is deep for the types a whole fleet of sensors shares a topic for, so a consumer that must not lose samples needs no configuration — but a **latency-sensitive** consumer (a live display) should pass a small explicit depth, or a momentarily-behind reader works through a backlog of stale frames instead of skipping to the newest. See [DETAILS.md](DETAILS.md#qos-defaults-per-type).
- You can optionally receive publisher match/unmatch notifications. See `provizio::dds::make_subscriber` (C++) or `provizio_dds.Subscriber` (Python) for parameters.

## Behaviour and configuration

The defaults are the recommended settings — a participant from `make_domain_participant()` needs
none of the following. Each topic is summarised here and covered in full in
[DETAILS.md](DETAILS.md).

- **[Network auto-recovery](DETAILS.md#network-auto-recovery)** is **on by default**. Participants
  survive an interface going down, a DHCP lease changing the host's address, or a cable moving to
  another NIC: the change is detected, endpoints are rebuilt and publishers and subscribers carry
  on without the application restarting.
- **[VPN and tunnel interfaces](DETAILS.md#vpn-and-tunnel-interfaces)** are **kept out of the DDS
  transports by default** — new in this release. A participant neither binds nor announces a
  tunnel address, so traffic is no longer duplicated over a VPN shared with a LAN peer. A
  deployment that carried DDS *through* a tunnel (unicast discovery over it) sets
  `PROVIZIO_DDS_ALLOW_VPN_INTERFACES=1` to keep doing so.
- **[Transport selection](DETAILS.md#transport-selection)** defaults to shared memory plus UDP on
  Linux and UDP-only on Windows and macOS, with enlarged socket buffers and an MTU-sized message
  cap suited to camera frames and point clouds. Pass `transport_mode` to override it — including
  `localhost_only`, which confines a same-host domain to shared memory and loopback UDP.
- **[Discovery tuning](DETAILS.md#discovery-tuning)** controls the announcement burst and lease
  durations. The defaults suit a mixed fleet; large deployments may want them longer.
- **[Logging](DETAILS.md#logging)** goes to stdout and stderr unless you install a callback with
  `set_log_callback`, which routes every message into your own logger.
- **[XML profiles](DETAILS.md#xml-profiles)** let you configure Fast-DDS directly. A participant
  configured that way keeps exactly what your profile asks for.
- **[Discovering endpoints and known types](DETAILS.md#discovering-endpoints-and-known-types)**
  reports remote publishers, subscribers and services as they appear, for tooling that has to
  learn the network rather than be told about it.

## Request/Response

The library provides a lightweight request/response API built on top of DDS topics. A service subscribes to a request topic and publishes replies to a response topic with correlation tracking; clients send requests and await replies.

### Sending a Request

**C++ Example:**

```C++
#include "provizio/dds/request_response.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>
#include <chrono>
#include <iostream>

int main()
{
    auto participant = provizio::dds::make_domain_participant();
    std_msgs::msg::String request;
    request.data("hello");
    auto future = provizio::dds::request<std_msgs::msg::StringPubSubType,
                                    std_msgs::msg::StringPubSubType>(
        participant, "echo_service", request);

    if (future.wait_for(std::chrono::seconds{2}) == std::future_status::ready) {
        const auto &response = future.get();
        // Print the received message
        std::cout << response.data() << std::endl;
    }

    return 0;
}
```

**Python Example:**

```Python
import asyncio
import provizio_dds

async def main():
    participant = provizio_dds.make_domain_participant()
    request = provizio_dds.String()
    request.data("hello")
    response = await provizio_dds.request(
        participant,
        provizio_dds.StringPubSubType,
        provizio_dds.StringPubSubType,
        provizio_dds.String,
        request,
        service_name="echo_service",
    )
    print(response.data())

asyncio.run(main())
```

### Reusable Client (`service_client`)

`request(...)` is a one-shot helper: it creates a fresh client, waits for matching, sends a single request, and tears the client down. To issue **many requests** — especially concurrently — create a `service_client` once, up front. It matches the service(s) in the background (so it is ready by the time you need it) and supports many in-flight requests at once, each returning its own future. Optionally call `wait_for_service(...)` to block until the service(s) are ready before sending; like `request(...)`, that readiness wait includes a settling window, so all services sharing the topic (e.g. multiple sensors keyed by `frame_id`) are discovered first.

**C++ Example:**

```C++
#include "provizio/dds/request_response.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>
#include <chrono>
#include <iostream>
#include <vector>

int main()
{
    auto participant = provizio::dds::make_domain_participant();
    auto client = provizio::dds::make_service_client<std_msgs::msg::StringPubSubType,
                                                      std_msgs::msg::StringPubSubType>(
        participant, "echo_service");

    // Optional: block until the service(s) are ready (includes the settling window).
    client->wait_for_service(std::chrono::seconds{5});

    // Many requests, each returning its own future_response; all may be in flight at once.
    std::vector<provizio::dds::future_response<std_msgs::msg::StringPubSubType,
                                               std_msgs::msg::StringPubSubType>> futures;
    for (int i = 0; i < 10; ++i) {
        std_msgs::msg::String request;
        request.data("hello " + std::to_string(i));
        futures.push_back(client->request(request));
    }
    for (auto &future : futures) {
        if (future.wait_for(std::chrono::seconds{2}) == std::future_status::ready) {
            std::cout << future.get().data() << std::endl;
        }
    }

    return 0;
}
```

**Python Example:**

```Python
import asyncio
import provizio_dds

async def main():
    participant = provizio_dds.make_domain_participant()
    client = provizio_dds.ServiceClient(
        participant,
        provizio_dds.StringPubSubType,  # request PubSub type
        provizio_dds.StringPubSubType,  # response PubSub type
        provizio_dds.String,            # response data type
        service_name="echo_service",
    )

    # Optional: block until the service(s) are ready (includes the settling window).
    await client.wait_for_service(timeout_sec=5.0)

    # Many requests in flight at once; each request() is awaitable.
    requests = []
    for i in range(10):
        request = provizio_dds.String()
        request.data(f"hello {i}")
        requests.append(client.request(request))
    for response in await asyncio.gather(*requests):
        print(response.data())

asyncio.run(main())
```

### Creating a Service

**C++ Example:**

```C++
#include "provizio/dds/request_response.h"
#include <std_msgs/msg/StringPubSubTypes.hpp>
#include <iostream>

int main() {
    auto participant = provizio::dds::make_domain_participant();

    // Create an "echo" service using a single service name for both topics
    auto service = provizio::dds::make_service<std_msgs::msg::StringPubSubType,
                                               std_msgs::msg::StringPubSubType>(
        participant,
        "echo_service",
        [](const std_msgs::msg::String &request) {
            std_msgs::msg::String response;
            response.data(request.data());
            return response; // synchronous handler, return std::future for async requests handling
        }
    );

    std::cin.get(); // Wait for any user input

    return 0;
}
```

**Python Example:**

```Python
import provizio_dds

participant = provizio_dds.make_domain_participant()

def handle_request(request: provizio_dds.String) -> provizio_dds.String:
    response = provizio_dds.String()
    response.data(request.data())
    return response

service = provizio_dds.Service(
    participant,
    provizio_dds.StringPubSubType,  # request PubSub type
    provizio_dds.String,            # request data type
    provizio_dds.StringPubSubType,  # response PubSub type
    handle_request,                 # can be sync or async
    service_name="echo_service",
)

input("Press any key to finish...")

# Ensure you call service.stop() on exit to stop internal threads, otherwise the application will keep running
service.stop()
```

**Notes:**

- Both synchronous and asynchronous service handlers are supported. Asynchronous handlers return `std::future` (C++) or are `async def` coroutines (Python).
- Reliability is set to RELIABLE for response readers/writers by default. The client’s request Publisher uses default (volatile) durability, while the service’s response Publisher uses TRANSIENT_LOCAL durability with a small history (depth 10) for robust delivery (compatible with ROS 2).
- Per-endpoint durability and history are configurable on both sides. `make_service(...)` / `provizio_dds.Service(...)` and `request(...)` accept an optional `durability_kind` and an `endpoint_history_depth` (the KEEP_LAST history depth of the underlying request/response DataWriters/DataReaders); leaving them at the defaults preserves the behaviour described above. Note that `max_history_depth` is a separate knob — it bounds the request **queue size**, not the endpoint history (`minimal_request_queue` / `MINIMAL_REQUEST_QUEUE`, i.e. `0`, selects a minimal bounded queue).
- Before publishing the first request, a short graph-based “readiness” wait is performed to ensure endpoints are matched and stable, avoiding races immediately after discovery.
- Optionally, the client request API lets you enforce a post-match settling window and (if desired) a finite matching timeout before the first request publish. See C++ `provizio::dds::request(..., std::chrono::milliseconds stable_matches_period = 1000ms, std::chrono::milliseconds service_match_timeout = 0ms)`, and Python `provizio_dds.request(..., stable_matches_period_sec=1.0, service_match_timeout_sec=0.0)` for the settling window control (use 0 to skip the extra wait).
- For issuing many requests (especially concurrently), reuse a single client created up front: `provizio::dds::make_service_client(...)` (C++) or `provizio_dds.ServiceClient(...)` (Python) — see [Reusable Client](#reusable-client-service_client) above. Its `request(...)` returns a `future_response` (C++) or an awaitable (Python), supports many requests in flight at once, and `wait_for_service(timeout[, stable_matches_period])` blocks until ready (settling window included). Its endpoints default to a bounded KEEP_LAST history (depth 10, matching the service's default response history), so a burst of up to that many concurrent requests/responses is retained and the reliable writer applies back-pressure beyond it; raise `endpoint_history_depth` for higher concurrency.
- To drop a request silently from a service handler, throw `provizio::dds::ignore_request` (C++) or raise `provizio_dds.Service.IgnoreRequest` (Python). Such requests are discarded without warnings.

### ROS 2 compatibility

- Publish/subscribe is compatible with ROS 2 starting from Humble.
- Request/response is compatible with ROS 2 starting from Jazzy.
- For both publish/subscribe and request/response to interoperate with ROS 2, either:
  - Keep the default `max_history_depth` on subscribers and services/requests, or
  - Configure ROS 2 QoS to use TRANSIENT_LOCAL durability on the ROS 2 publishers with appropriate history depth for better reliability.
- **Raw camera images need `SensorDataQoS` on the ROS 2 side.** provizio_dds publishes `sensor_msgs::msg::Image` (`rt/provizio_camera` and `rt/provizio_freespace`) with a **BEST_EFFORT** writer by default. Reliability is an RxO policy, so a ROS 2 subscriber on the default (RELIABLE) QoS will not match such a publisher at all, and an RxO mismatch is a silent non-match rather than a logged error. Request `rclcpp::SensorDataQoS()` / `rclpy.qos.qos_profile_sensor_data` — the conventional ROS 2 profile for camera streams — or have the publisher pass an explicit `RELIABLE_RELIABILITY_QOS`. This is deliberate: raw images are being superseded by compressed images and by polygonal freespace. Every other type keeps a RELIABLE writer and is unaffected. See [DETAILS.md](DETAILS.md#ros-2-interoperability-raw-images-need-sensordataqos).
- When running provizio_dds binaries in a shell with a **sourced ROS 2 environment** (`setup.bash`), note that ROS 2 distros bundle their own eProsima Fast-DDS. If its major.minor version matches the Fast-DDS bundled by provizio_dds (e.g. ROS 2 Lyrical ships Fast-DDS v3.6.1 while provizio_dds bundles v3.6.2 — both named `libfastdds.so.3.6`), the sourced `LD_LIBRARY_PATH` makes provizio_dds binaries load the ROS-bundled library. Fast-DDS doesn't guarantee ABI stability across patch releases, so this can break discovery or crash. To make an installed provizio_dds immune to it, build with `-DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=ON` (Linux): installed binaries then link Fast-DDS by its fully-qualified name (e.g. `libfastdds.so.3.6.2.0`), which never collides with the ROS-bundled copy. Linking both provizio_dds and ROS 2 libraries into the *same process* is a different problem — see [provizio_radar_api_ros2](https://github.com/provizio/provizio_radar_api_ros2) for the namespace-isolation approach it requires.

## Reading and Creating PointCloud2 Messages

Utilities for reading and creating `PointCloud2` messages — both generically (any field layout) and in the standard Provizio radar format (see [TOPICS.md of provizio_dds_idls](https://github.com/provizio/provizio_dds_idls/blob/master/TOPICS.md)):

### Python Example

```Python
import provizio_dds

# Reading Provizio radar point clouds: a structured numpy array, one record per point with named fields
# x, y, z, radar_relative_radial_velocity, signal_to_noise_ratio, ground_relative_radial_velocity
# (access by name, e.g. points["x"]). Use read_points_numpy(...) for a plain (N, 6) float matrix.
points = provizio_dds.point_cloud2.read_points(point_cloud2_message)

# Or as named tuples (slower, but self-descriptive: point.x, point.signal_to_noise_ratio, ...):
points = provizio_dds.point_cloud2.read_points_list(point_cloud2_message)

# Reading ANY PointCloud2 generically, selecting fields by name:
xyz = provizio_dds.point_cloud2.read_points(point_cloud2_message, field_names=["x", "y", "z"])

# Creating radar point clouds:
message = provizio_dds.point_cloud2.make_radar_point_cloud(
    provizio_dds.point_cloud2.make_header(timestamp_sec, timestamp_nanosec, "provizio_radar_front_center"),
    [[1.0, 2.0, 3.0, 0.5, 7.0, 0.0]],  # any iterable of per-point field values
)

# Creating and reading Provizio entity clouds (radar / camera / fused):
from types import SimpleNamespace
entity = SimpleNamespace(
    entity_id=42, entity_class=3, x=1.5, y=-2.5, z=0.5,
    radar_relative_radial_velocity=10.5, ground_relative_radial_velocity=9.5,
    orientation=(0.1, 0.2, 0.3, 0.4), size=(4.5, 1.8, 1.5),
    camera_bbox=(float("nan"),) * 4, entity_confidence=90, entity_class_confidence=80,
    camera_entity_id=provizio_dds.point_cloud2.NO_ENTITY_ID,
)
entities_message = provizio_dds.point_cloud2.make_entities_from(
    provizio_dds.point_cloud2.make_header(timestamp_sec, timestamp_nanosec, "provizio_radar_front_center"),
    "radar",
    [entity],
)
entities = provizio_dds.point_cloud2.read_entities(entities_message)
entity_id = entities[0].entity_id
orientation = entities[0].orientation  # (x, y, z, w); fields absent from the cloud kind read as NaN
                                       # (floats) or NO_ENTITY_ID (ids)
entity_kind = entities[0].kind()  # per-entity: "radar"/"camera"/"fused" (see also entities_kind for the whole cloud)
kind = provizio_dds.point_cloud2.entities_kind(entities_message)  # "radar", "camera", "fused" or None
```

Arbitrary field layouts can be created with `provizio_dds.point_cloud2.create_cloud`; the flat-row makers
`make_radar_entities` / `make_camera_entities` / `make_fused_entities` also exist for pre-flattened data.

For more details see [python/point_cloud2.py](python/point_cloud2.py) and [test/python/accumulation_test.py](test/python/accumulation_test.py).

### C++ Example

```C++
#include <provizio/dds/point_cloud2.h>
namespace pc2 = provizio::dds::point_cloud2;

// Reading Provizio radar point clouds:
const std::vector<pc2::radar_point> radar_points = pc2::read_radar_points(point_cloud2_message);

// Reading ANY PointCloud2 generically, field by field:
const pc2::cloud_view view{point_cloud2_message};
const auto x_field = view.field("x");
for (const auto point : view)
{
    const auto x = x_field ? point.get<float>(*x_field) : std::numeric_limits<float>::quiet_NaN();
    // ...
}

// Creating radar point clouds (any iterable range of radar_point; contiguous ranges are bulk-copied):
const auto message = pc2::make_radar_point_cloud(
    pc2::make_header(timestamp_sec, timestamp_nanosec, "provizio_radar_front_center"), radar_points);

// Creating arbitrary clouds (rows in field order; values are converted to each field's datatype):
const auto fields = pc2::make_fields({{"x", sensor_msgs::msg::PointField_Constants::FLOAT32},
                                      {"y", sensor_msgs::msg::PointField_Constants::FLOAT32},
                                      {"intensity", sensor_msgs::msg::PointField_Constants::UINT16}});
const auto custom_cloud = pc2::create_cloud(pc2::make_header(0, 0, "frame"), fields,
                                            std::vector<std::array<float, 3>>{{1.0F, 2.0F, 3.0F}});

// Creating and reading Provizio entity clouds (radar / camera / fused):
pc2::entity an_entity;
an_entity.entity_id = 42;
an_entity.entity_class = 3;
an_entity.x = 1.5F; an_entity.y = -2.5F; an_entity.z = 0.5F;
an_entity.orientation = {0.1F, 0.2F, 0.3F, 0.4F};
an_entity.size = {4.5F, 1.8F, 1.5F};
an_entity.radar_relative_radial_velocity = 10.5F;
an_entity.ground_relative_radial_velocity = 9.5F;
an_entity.entity_confidence = 90; an_entity.entity_class_confidence = 80;
const auto entities_message =
    pc2::make_entities(pc2::make_header(timestamp_sec, timestamp_nanosec, "provizio_radar_front_center"),
                       pc2::entities_kind::radar, std::vector<pc2::entity>{an_entity});
const std::vector<pc2::entity> entities = pc2::read_entities(entities_message);
const auto entity_id = entities[0].entity_id;  // fields absent from the cloud kind read as NaN (floats) or
                                               // pc2::no_entity_id (ids)
const auto entity_kind = entities[0].kind();  // per-entity: radar/camera/fused (see also get_entities_kind for the whole cloud)
const auto kind = pc2::get_entities_kind(entities_message);  // entities_kind::radar/camera/fused or nullopt
```

The flat-row makers `make_radar_entities` / `make_camera_entities` / `make_fused_entities` also exist for pre-flattened data.

For more details see [include/provizio/dds/point_cloud2.h](include/provizio/dds/point_cloud2.h) and [test/point_cloud2/point_cloud2_test.cpp](test/point_cloud2/point_cloud2_test.cpp).

## Points Accumulation and Multi-Radar Fusion

Point clouds accumulation keeps some of the reflected points (normally ones from static objects) "visible" for a number of frames after they were originally received. It makes point clouds much denser and features of objects much clearer - similar to long exposure of dark scenes in photography. Usually, accumulation requires at least 2 types of inputs: point clouds and localization (both odometry and GNSS are supported). In case of static ego or single frame multi-radar fusion, accumulation can be used without odometry/localization inputs. When Provizio radar odometry is used it effectively turns into a SLAM (Simultaneous Localization and Mapping) solution, but external localization/odometry source is also supported.
Accumulation is also used for fusion of point clouds from multiple radars in a vehicle. In this case the sensors' extrinsics calibration data is required.

### Example of Point Clouds Accumulation

![Point Clouds Accumulation](media/point_clouds_accumulation.png)

### Python Example

```Python
import provizio_dds

max_accumulate_frames_per_radar = 30
# By default, relies on rt/provizio_radar_odometry as a localization data source and rt/provizio_extrinsics as the sensors' extrinsics data source
points_accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(max_accumulate_frames_per_radar)

# Then some time later, when data was received
print(f"Accumulated, relative to the local frame:")
for point in points_accumulator.get_points_local_frame_relative():
    print(f"x = {point.position[0]}, y = {point.position[1]}, z = {point.position[2]}, ground_relative_velocity = {point.ground_relative_velocity}, snr = {point.snr}")
print(f"Accumulated, relative to the current ego position:")
for point in points_accumulator.get_points_ego_relative():
    print(f"x = {point.position[0]}, y = {point.position[1]}, z = {point.position[2]}, ground_relative_velocity = {point.ground_relative_velocity}, snr = {point.snr}")
```

Instead of polling, an `on_point_cloud` callback can be invoked on receiving and accumulating every radar point cloud. It's fired outside the accumulator's internal lock, so `get_points_local_frame_relative` / `get_points_ego_relative` may be called right from the callback:

```Python
def on_point_cloud(accumulator):
    print(f"{len(accumulator.get_points_ego_relative())} points accumulated")

points_accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(
    max_accumulate_frames_per_radar, on_point_cloud=on_point_cloud
)
```

For more details see [python/accumulation.py](python/accumulation.py) and [test/python/accumulation_test.py](test/python/accumulation_test.py).

### C++ Example

```C++
#include <provizio/dds/accumulation.h>

const std::size_t max_accumulate_frames_per_radar = 30;
// By default, relies on rt/provizio_radar_odometry as a localization data source and rt/provizio_extrinsics as the sensors' extrinsics data source
provizio::dds::accumulation::dds_point_clouds_accumulator points_accumulator{max_accumulate_frames_per_radar};

// Then some time later, when data was received
std::cout << "Accumulated, relative to the local frame:" << std::endl;
for (const auto &point : points_accumulator.get_points_local_frame_relative())
{
    std::cout << "x = " << point.position[0] << ", y = " << point.position[1] << ", z = " << point.position[2]
              << ", ground_relative_radial_velocity = " << point.ground_relative_radial_velocity
              << ", signal_to_noise_ratio = " << point.signal_to_noise_ratio << std::endl;
}
std::cout << "Accumulated, relative to the current ego position:" << std::endl;
for (const auto &point : points_accumulator.get_points_ego_relative())
{
    std::cout << "x = " << point.position[0] << ", y = " << point.position[1] << ", z = " << point.position[2]
              << ", ground_relative_radial_velocity = " << point.ground_relative_radial_velocity
              << ", signal_to_noise_ratio = " << point.signal_to_noise_ratio << std::endl;
}
```

Instead of polling, an `on_point_cloud` callback can be invoked on receiving and accumulating every radar point cloud. It's fired outside the accumulator's internal lock, so the `get_points_*` getters may be called right from the callback. It runs on a DDS thread though: never block in it, and never create or destroy DDS endpoints (publishers, subscribers, participants) from it:

```C++
provizio::dds::accumulation::dds_accumulation_options options;
options.on_point_cloud = [](provizio::dds::accumulation::dds_point_clouds_accumulator &accumulator) {
    std::cout << accumulator.get_points_ego_relative().size() << " points accumulated" << std::endl;
};
provizio::dds::accumulation::dds_point_clouds_accumulator points_accumulator{max_accumulate_frames_per_radar,
                                                                             options};
```

The C++ accumulation maths uses [Eigen](https://eigen.tuxfamily.org/) when YOUR compiler can see it (auto-detected via `__has_include(<Eigen/Dense>)` at the point your code compiles the accumulation headers — e.g. `find_package(Eigen3)` + linking `Eigen3::Eigen` in your CMake; `install_dependencies.sh` installs it). Without Eigen it falls back to less efficient plain-CPU processing (provizio_dds's CMake warns about it at configure time). Define `PROVIZIO_DDS_DISABLE_EIGEN` (or configure provizio_dds with `DISABLE_EIGEN=ON`) to force the fallback deliberately. Prebuilt provizio_dds binaries are identical either way — the maths-path choice is made when YOUR code compiles, never baked into the library.

#### Localization frame and extrinsics

The localization source has its own extrinsics (sensor → ego frame), read by default from the same `rt/provizio_extrinsics` topic as the radars. For **odometry** the localization frame defaults to `provizio_radar_front_center`: radar-based odometry is published in the front-center radar's frame, so it shares that radar's calibration — the same `rt/provizio_extrinsics` transform is applied to both the odometry and that radar's point clouds. The localization extrinsics is **assumed identity until that frame's transform is received**; when it arrives (or later changes) any already-accumulated frames are **retroactively re-placed**, so none are lost. For **GNSS** (`NavSatFix`) the localization frame is instead learned from the first fix and, having no radar extrinsics, stays ego-relative (identity). When multiple localization sources share the topic, only the one whose `child_frame_id` (odometry) / `header.frame_id` (GNSS) matches `localization_frame_id` is used — the rest are dropped. Set `localization_frame_id` / `localization_extrinsics_topic` to override these defaults.

For more details see [include/provizio/dds/accumulation.h](include/provizio/dds/accumulation.h) and [test/accumulation/accumulation_test.cpp](test/accumulation/accumulation_test.cpp).
