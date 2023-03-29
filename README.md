# provizio_dds

C++ and Python library for [DDS communication](https://www.dds-foundation.org/what-is-dds-3/) in Provizio customer
facing APIs and internal Provizio software components. Built using
[eProsima Fast-DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds) DDS implementation (Apache
License 2.0).

Although based directly on a DDS, it's compatible with [ROS2](https://docs.ros.org/en/rolling/) and provides all ROS2
built-in data types.

## Build dependencies

- cmake, git, C++ 14 compiler, libssl-dev
- For Python bindings, also Python3, SWIG 4, libpython3-dev

## Publishing Data

```C++
#include "provizio/dds/publisher.h"
#include <std_msgs/msg/StringPubSubTypes.h>

// In a function
auto publisher = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(
    provizio::dds::make_domain_participant(), "rt/chatter");

std_msgs::msg::String str;
str.data("Hello World!");

publisher->publish(str);
```

For more details see [provizio/dds/publisher.h](include/provizio/dds/publisher.h).

## Receiving Data

```C++
#include "provizio/dds/subscriber.h"
#include <std_msgs/msg/StringPubSubTypes.h>
#include <iostream>

// In a function
const auto subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
    provizio::dds::make_domain_participant(), "rt/chatter", [&](const std_msgs::msg::String &message) {
        std::cout << message.data() << std::endl;
    });
```

For more details see [provizio/dds/subscriber.h](include/provizio/dds/subscriber.h).

## Built-In Data Types

See <https://github.com/provizio/provizio_dds_idls>
