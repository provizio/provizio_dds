# provizio_dds

C++ and Python library for [DDS communication](https://www.dds-foundation.org/what-is-dds-3/) in Provizio customer
facing APIs and internal Provizio software components. Built using
[eProsima Fast-DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds) DDS implementation (Apache
License 2.0).

Although based directly on a DDS, it's compatible with [ROS2](https://docs.ros.org/en/rolling/) and provides all ROS2
built-in data types.

| CI Status: master | CI Status: develop |
| ----------------- | ----------------- |
| ![CI Status: master](https://github.com/provizio/provizio_dds/actions/workflows/ci.yml/badge.svg?branch=master) | ![CI Status: develop](https://github.com/provizio/provizio_dds/actions/workflows/ci.yml/badge.svg?branch=develop) |

## Provizio DDS API

[Table of DDS topics and their respective data types used in Provizio API](https://github.com/provizio/provizio_dds_idls/blob/master/TOPICS.md)

## Build dependencies

**C++:**

- Linux or macOS
- CMake
- Git
- C++ 14 compiler
- libssl-dev
- When Fast-DDS installation is present it will be used, otherwise downloaded and built automatically

**Python:**

- Linux or macOS
- Git
- Python 3
- Pip 3
- unzip

When using non-binary-prebuilt configurations (i.e. any macOS, Linux Debug, non-x64/non-aarch64, non-master provizio_dds_idls) all the C++ dependencies will also be required.

There is a convenience Bash script to install all dependencies in *apt*-featuring Linux and macOS. In Linux it's to be executed with root privileges, f.e. using `sudo`.

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
ExternalProject_Add(libprovizio_dds
    GIT_REPOSITORY "https://github.com/${PROVIZIO_DDS_GITHUB_PROJECT}.git"
    GIT_TAG "${PROVIZIO_DDS_GITHUB_BRANCH}"
    UPDATE_COMMAND ""
    PREFIX "${PROVIZIO_DDS_PREFIX}"
    SOURCE_DIR "${PROVIZIO_DDS_SOURCE_DIR}"
    BINARY_DIR "${PROVIZIO_DDS_BINARY_DIR}"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${PROVIZIO_DDS_INSTALL_DIR}" "-DENABLE_CHECK_FORMAT=OFF" "-DENABLE_TESTS=OFF" "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON"
)
add_dependencies(<YOUR_CMAKE_TARGET> libprovizio_dds)
target_include_directories(<YOUR_CMAKE_TARGET> SYSTEM PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/include")
target_link_directories(<YOUR_CMAKE_TARGET> PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/lib")
target_link_libraries(<YOUR_CMAKE_TARGET> PUBLIC provizio_dds provizio_dds_types fastrtps fastcdr)
```

**Python (pip):**

```Bash
python3 -m pip install -v git+https://github.com/provizio/provizio_dds.git
```

or

```Bash
python3 -m pip install -v git+https://github.com/provizio/provizio_dds.git@TAG_or_BRANCH
```

## Publishing Data

**C++ Example:**

```C++
#include "provizio/dds/publisher.h"
#include <std_msgs/msg/StringPubSubTypes.h>

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

- Publishers support configurable reliability QoS (BEST_EFFORT or RELIABLE). See `provizio::dds::make_publisher` (C++) or `provizio_dds.Publisher` (Python) for parameters.
- Publishers support configurable durability QoS and optional history depth: keep default durability by passing `-1` (no changes), pass `0` to force VOLATILE durability (no history), or pass a positive value to enable TRANSIENT_LOCAL durability (KEEP_LAST with the given depth). See headers/Python docs for details.
- You can optionally receive subscriber match/unmatch notifications. See `provizio::dds::make_publisher` (C++) or `provizio_dds.Publisher` (Python) for parameters.

## Receiving Data

**C++ Example:**

```C++
#include "provizio/dds/subscriber.h"
#include <std_msgs/msg/StringPubSubTypes.h>
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
- Subscribers support configurable reliability QoS (BEST_EFFORT or RELIABLE). See `provizio::dds::make_subscriber` (C++) or `provizio_dds.Subscriber` (Python) for parameters.
- Subscribers support configurable durability QoS and optional history depth: keep default durability by passing `-1` (no changes), pass `0` to force VOLATILE durability (no history), or pass a positive value to enable TRANSIENT_LOCAL durability (KEEP_LAST with the given depth). See headers/Python docs for details.
- You can optionally receive publisher match/unmatch notifications. See `provizio::dds::make_subscriber` (C++) or `provizio_dds.Subscriber` (Python) for parameters.

## Request/Response

The library provides a lightweight request/response API built on top of DDS topics. A service subscribes to a request topic and publishes replies to a response topic with correlation tracking; clients send requests and await replies.

### Sending a Request

**C++ Example:**

```C++
#include "provizio/dds/request_response.h"
#include <std_msgs/msg/StringPubSubTypes.h>
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

### Creating a Service

**C++ Example:**

```C++
#include "provizio/dds/request_response.h"
#include <std_msgs/msg/StringPubSubTypes.h>
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
- Before publishing the first request, a short graph-based “readiness” wait is performed to ensure endpoints are matched and stable, avoiding races immediately after discovery.
- Optionally, the client request API lets you enforce a post-match settling window and (if desired) a finite matching timeout before the first request publish. See C++ `provizio::dds::request(..., std::chrono::milliseconds stable_matches_period = 0ms, std::chrono::milliseconds service_match_timeout = 0ms)`, and Python `provizio_dds.request(..., stable_matches_period_sec=0.0, service_match_timeout_sec=0.0)` for the settling window control.
- To drop a request silently from a service handler, throw `provizio::dds::ignore_request` (C++) or raise `provizio_dds.Service.IgnoreRequest` (Python). Such requests are discarded without warnings.

### ROS 2 compatibility

- Publish/subscribe is compatible with ROS 2 starting from Humble.
- Request/response is compatible with ROS 2 starting from Jazzy.
- For both publish/subscribe and request/response to interoperate with ROS 2, either:
  - Keep the default `max_history_depth` on subscribers and services/requests, or
  - Configure ROS 2 QoS to use TRANSIENT_LOCAL durability on the ROS 2 publishers with appropriate history depth for better reliability.

## Points Accumulation and Multi-Radar Fusion

Point clouds accumulation keeps some of reflected points (normally ones from static objects) "visible" for a number of frames after they were originally received. It makes point clouds much denser and features of objects much clearer - similar to long exposure of dark scenes in photography. Usually, accumulation requires at least 2 types of inputs: point clouds and localization (both odometry and GNSS are supported). In case of static ego or single frame multi-radar fusion, accumulation can be used without odometry/localization inputs. When Provizio radar odometry is used it effectively turns into a SLAM (Simultaneous Localization and Mapping) solution, but external localization/odometry source is also supported.
Accumulation is also used for fusion of point clouds from multiple radars in a vehicle. In this case sensors extrinsics calibration data is required.

### Example of Point Clouds Accumulation

![Point Clouds Accumulation](media/point_clouds_accumulation.png)

### Python Example

```Python
import provizio_dds

max_accumulate_frames_per_radar = 30
# By default, relies on rt/provizio_radar_odometry as a localization data source and rt/provizio_extrinsics as sensors extrinsics data source
points_accumulator = provizio_dds.accumulation.DDSPointCloudsAccumulator(max_accumulate_frames_per_radar)

# Then some time later, when data was received
print(f"Accumulated, relative to the local frame:")
for point in points_accumulator.get_points_local_frame_relative():
    print(f"x = {point.position[0]}, y = {point.position[1]}, z = {point.position[2]}, ground_relative_velocity = {point.ground_relative_velocity}, snr = {point.snr}")
print(f"Accumulated, relative to the current ego position:")
for point in points_accumulator.get_points_ego_relative():
    print(f"x = {point.position[0]}, y = {point.position[1]}, z = {point.position[2]}, ground_relative_velocity = {point.ground_relative_velocity}, snr = {point.snr}")
```

For more details see [python/accumulation.py](python/accumulation.py) and [test/python/accumulation_test.py](test/python/accumulation_test.py).

## XML Profiles

As DDS allows for configuring many of its parameters, you can optionally define a custom XML profile and enable it via setting its path to environment variable `FASTRTPS_DEFAULT_PROFILES_FILE`. Make sure to set the `participant` tag's attribute `is_default_profile` to `true` to activate the profile automatically, f.e.:

```XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="my_custom_profile" is_default_profile="true">
        <rtps>
        ...
        </rtps>
    </participant>
</profiles>
```

For more details, please refer to [Fast-DDS documentation](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/xml_configuration/xml_configuration.html).
Please note that some of che changes may break compatibility between publishers and subscribers, unless applied on both sides.

You may see how this functionality is used to activate Discovery-Server-based participants discovery in provizio_dds tests:

- [test/congested_network_test/fast_dds_server_config.xml](test/congested_network_test/fast_dds_server_config.xml)
- [test/congested_network_test/fast_dds_client_config.xml](test/congested_network_test/fast_dds_client_config.xml)
