#!/usr/bin/env python3
# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Cross-version compat publisher for provizio::msg::radar_info — the key
# wire-compat case for the APT-11626 Fast-DDS 3.x migration. The
# `current_range` / `supported_ranges` fields changed from an IDL
# `enum radar_range` (in 1.10.x) to a plain `uint32` (in 3.x). Both
# encodings are a 32-bit unsigned int on the CDR wire and the topic-type
# name is preserved across the version boundary, so a 1.10.1 subscriber
# should still decode messages produced by this script verbatim — and
# vice versa. This publisher exercises both directions when paired with
# cross_compat_radar_info_subscriber.py via cross_version_compat_test.py.

import sys
import time
import provizio_dds

CROSS_COMPAT_DOMAIN_ID = 42
CROSS_COMPAT_RADAR_INFO_TOPIC_NAME = "provizio_dds_cross_compat_radar_info_topic"

SERIAL_NUMBER = "cross_compat_radar_42"
# The full enumerator set — both versions assign identical integer values
# (short=0, medium=1, long=2, ultra=3, hyper=4), which is the byte-level
# invariant we're checking. UNKNOWN_RANGE (0xffff) is intentionally
# omitted: we want the receiver to assert *exact* equality of the
# advertised set, so picking values that round-trip cleanly through both
# uint32 and enum representations keeps the failure signal focused on
# the schema change.
SUPPORTED_RANGES = [0, 1, 2, 3, 4]
CURRENT_RANGE = 2  # long_range / LONG_RANGE

WAIT_TIME = 0.2
PUBLISH_TIMES = 50


def _assign_supported_ranges(message, values):
    """Populate `radar_info.supported_ranges` with `values` (a list of
    ints), picking the SWIG-wrapped vector class that matches *this*
    build's C++ signature.

    Both 1.10.1 and 3.x expose `uint32_t_vector` in `provizio_dds` (it
    comes from various unrelated sequence<uint32> fields), so attribute
    presence alone is not a reliable discriminator — what actually
    differs is the C++ type `radar_info::supported_ranges()` expects:
    `std::vector<uint32_t>` in 3.x (post enum->uint32 migration) vs
    `std::vector<radar_range>` in 1.10.1. We try the uint32 form first
    and fall back to the legacy enum-vector form on the TypeError SWIG
    raises when the wrong wrapped type is passed."""
    candidates = ("uint32_t_vector", "provizio_msg_radar_range_vector")
    last_error = None
    for name in candidates:
        cls = getattr(provizio_dds, name, None)
        if cls is None:
            continue
        vec = cls()
        for v in values:
            vec.append(v)
        try:
            message.supported_ranges(vec)
            return
        except TypeError as e:
            last_error = e
            continue
    raise RuntimeError(
        "cross_compat_radar_info_publisher: no SWIG-wrapped vector type "
        f"accepted by radar_info.supported_ranges (tried {candidates}; "
        f"last error: {last_error})")


publisher = provizio_dds.Publisher(
    provizio_dds.make_domain_participant(CROSS_COMPAT_DOMAIN_ID),
    CROSS_COMPAT_RADAR_INFO_TOPIC_NAME,
    provizio_dds.radar_infoPubSubType,
    lambda _, has_subscriber: print(
        "cross_compat_radar_info_publisher: " +
        ("first subscriber matched" if has_subscriber else "all subscribers unmatched")))

message = provizio_dds.radar_info()
message.serial_number(SERIAL_NUMBER)
_assign_supported_ranges(message, SUPPORTED_RANGES)
message.current_range(CURRENT_RANGE)

successful_times = 0
for _ in range(PUBLISH_TIMES):
    successful_times += 1 if publisher.publish(message) else 0
    time.sleep(WAIT_TIME)

print(f"cross_compat_radar_info_publisher: Successfully published {successful_times} times")
sys.exit(0 if successful_times > 0 else 1)
