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
# Cross-version compat subscriber for sensor_msgs::msg::PointCloud2. See
# cross_compat_pointcloud2_publisher.py for the test rationale. The
# checks here cover the parts of the message most likely to surface a
# version-skew regression: header timestamp + frame_id, geometry
# (height/width, point_step/row_step), every PointField name + offset +
# datatype, and the raw `data` byte blob compared verbatim. The last
# check is the actual CDR wire-fingerprint — anything that re-encodes
# differently in 3.x vs 1.10.x will produce a mismatched byte sequence
# even if all the structured fields look right.

import struct
import sys
import threading
import provizio_dds

CROSS_COMPAT_DOMAIN_ID = 42
CROSS_COMPAT_POINTCLOUD2_TOPIC_NAME = "provizio_dds_cross_compat_pointcloud2_topic"

EXPECTED_HEADER_SEC = 1717000000
EXPECTED_HEADER_NANOSEC = 123456789
EXPECTED_FRAME_ID = "cross_compat_radar_frame"
EXPECTED_WIDTH = 2
EXPECTED_HEIGHT = 1
EXPECTED_POINT_STEP = 24   # 6 fields * float32
EXPECTED_ROW_STEP = 48     # point_step * width
EXPECTED_FIELD_NAMES = (
    "x", "y", "z",
    "radar_relative_radial_velocity",
    "signal_to_noise_ratio",
    "ground_relative_radial_velocity",
)
WAIT_TIME = 10


# Build the *exact* binary blob make_radar_point_cloud(...) produces from
# the publisher's two-point input. The helper packs little-endian float32
# in row-major order; we replicate that here without going through the
# helper so the validation is independent of any future helper change
# (and so it would catch a regression where the helper's serialization
# silently drifted).
def _expected_data_bytes():
    points = [
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        [1.5, 2.5, 3.5, 4.5, 5.5, 6.5],
    ]
    buf = bytearray()
    for p in points:
        for v in p:
            buf += struct.pack("<f", v)
    return bytes(buf)


EXPECTED_DATA = _expected_data_bytes()


def _data_to_bytes(data):
    """Normalize PointCloud2.data() to a plain `bytes` object. SWIG can
    expose this as a wrapped std::vector<uint8_t> or as a bytes-like
    sequence depending on the binding flavour; iterating yields plain
    ints on every variant we've seen, so materialising via bytearray
    works uniformly."""
    return bytes(bytearray(int(b) & 0xff for b in data))


received = None
had_publishers = False
cv = threading.Condition()


def on_message(message):
    with cv:
        global received
        # Capture *all* the metadata up front so any later equality
        # assertion can produce a coherent diff if it fails.
        fields = message.fields()
        received = {
            "sec": int(message.header().stamp().sec()),
            "nanosec": int(message.header().stamp().nanosec()),
            "frame_id": message.header().frame_id(),
            "height": int(message.height()),
            "width": int(message.width()),
            "point_step": int(message.point_step()),
            "row_step": int(message.row_step()),
            "field_names": tuple(f.name() for f in fields),
            "field_offsets": tuple(int(f.offset()) for f in fields),
            "field_datatypes": tuple(int(f.datatype()) for f in fields),
            "data_bytes": _data_to_bytes(message.data()),
        }
        cv.notify()


def on_has_publisher_changed_function(has):
    if has:
        with cv:
            global had_publishers
            had_publishers = True
            cv.notify()


subscriber = provizio_dds.Subscriber(
    provizio_dds.make_domain_participant(CROSS_COMPAT_DOMAIN_ID),
    CROSS_COMPAT_POINTCLOUD2_TOPIC_NAME,
    provizio_dds.PointCloud2PubSubType,
    provizio_dds.PointCloud2,
    on_message, on_has_publisher_changed_function)


def _validation_errors(msg):
    errs = []
    if msg["sec"] != EXPECTED_HEADER_SEC:
        errs.append(f"sec={msg['sec']!r} (want {EXPECTED_HEADER_SEC})")
    if msg["nanosec"] != EXPECTED_HEADER_NANOSEC:
        errs.append(f"nanosec={msg['nanosec']!r} (want {EXPECTED_HEADER_NANOSEC})")
    if msg["frame_id"] != EXPECTED_FRAME_ID:
        errs.append(f"frame_id={msg['frame_id']!r} (want {EXPECTED_FRAME_ID!r})")
    if msg["height"] != EXPECTED_HEIGHT:
        errs.append(f"height={msg['height']} (want {EXPECTED_HEIGHT})")
    if msg["width"] != EXPECTED_WIDTH:
        errs.append(f"width={msg['width']} (want {EXPECTED_WIDTH})")
    if msg["point_step"] != EXPECTED_POINT_STEP:
        errs.append(f"point_step={msg['point_step']} (want {EXPECTED_POINT_STEP})")
    if msg["row_step"] != EXPECTED_ROW_STEP:
        errs.append(f"row_step={msg['row_step']} (want {EXPECTED_ROW_STEP})")
    if msg["field_names"] != EXPECTED_FIELD_NAMES:
        errs.append(f"field_names={msg['field_names']} (want {EXPECTED_FIELD_NAMES})")
    expected_offsets = tuple(i * 4 for i in range(6))
    if msg["field_offsets"] != expected_offsets:
        errs.append(f"field_offsets={msg['field_offsets']} (want {expected_offsets})")
    expected_datatype = provizio_dds.FLOAT32
    if any(dt != expected_datatype for dt in msg["field_datatypes"]):
        errs.append(
            f"field_datatypes={msg['field_datatypes']} "
            f"(want all {expected_datatype}/FLOAT32)")
    if msg["data_bytes"] != EXPECTED_DATA:
        errs.append(
            f"data_bytes mismatch ({len(msg['data_bytes'])} B got, "
            f"{len(EXPECTED_DATA)} B expected); "
            f"first diff at offset {next((i for i, (a, b) in enumerate(zip(msg['data_bytes'], EXPECTED_DATA)) if a != b), 'EOF')}")
    return errs


with cv:
    cv.wait_for(lambda: had_publishers and received is not None
                and not _validation_errors(received),
                WAIT_TIME)
    del subscriber

    if not had_publishers:
        print("cross_compat_pointcloud2_subscriber: never matched a publisher")
        sys.exit(1)
    if received is None:
        print("cross_compat_pointcloud2_subscriber: matched publisher but received no message")
        sys.exit(1)
    errors = _validation_errors(received)
    if errors:
        print("cross_compat_pointcloud2_subscriber: validation failures:")
        for e in errors:
            print(f"  - {e}")
        sys.exit(1)
    print("cross_compat_pointcloud2_subscriber: Success!")
    sys.exit(0)
