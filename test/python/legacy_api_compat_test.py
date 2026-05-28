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

"""Source-compat test: exercises the 1.10.x Python API spellings against the
current provizio_dds build. The intent is to catch any future migration that
breaks consumer code written against the camelCase / ReturnCode_t-class
flavour of Fast-DDS-python 1.x.

This is a no-DDS, no-network test — it only checks that names resolve and
return the expected types/values."""

import sys
import provizio_dds


def expect(name: str, cond: bool) -> None:
    if not cond:
        print(f"FAIL: {name}", file=sys.stderr)
        sys.exit(1)
    print(f"OK: {name}")


# ReturnCode_t was a SWIG-wrapped enum class in Fast-DDS-python 1.x. In 2.x
# the bindings flattened it to module-level `RETCODE_*` constants. The
# provizio_dds wrapper re-exposes the old class so user code keeps working.
expect("provizio_dds.ReturnCode_t resolves",
       hasattr(provizio_dds, "ReturnCode_t"))

expect("provizio_dds.ReturnCode_t.RETCODE_OK resolves",
       hasattr(provizio_dds.ReturnCode_t, "RETCODE_OK"))

expect("ReturnCode_t.RETCODE_OK equals fastdds.RETCODE_OK",
       provizio_dds.ReturnCode_t.RETCODE_OK == provizio_dds.RETCODE_OK)

# A handful of representative return codes are present.
for code in ("RETCODE_OK", "RETCODE_NO_DATA", "RETCODE_TIMEOUT",
             "RETCODE_BAD_PARAMETER", "RETCODE_ERROR"):
    expect(f"ReturnCode_t.{code}", hasattr(provizio_dds.ReturnCode_t, code))


# TopicDataType.getName() → get_name() in 2.x; wrapper aliases it back.
# Verify through a concrete generated PubSubType so we exercise the alias
# inherited by every Provizio type.
string_pub_sub = provizio_dds.StringPubSubType()
expect("StringPubSubType().getName() resolves",
       hasattr(string_pub_sub, "getName"))
expect("StringPubSubType().get_name() resolves",
       hasattr(string_pub_sub, "get_name"))
expect("getName() and get_name() return the same string",
       string_pub_sub.getName() == string_pub_sub.get_name())


# Wire-name fingerprint test: every PubSubType must advertise the canonical
# `<pkg>::msg::dds_::<Type>_` name that pre-3.x provizio_dds and stock ROS 2
# subscribers expect on the wire. Catches accidental name regressions —
# e.g. the historic double-underscore (`Bool__`) bug from the old trailing-
# underscore keyword escape — by failing here before the binary ever
# publishes a wrongly-named topic.
_WIRE_NAMES = {
    # std_msgs types that hit the IDL keyword escape (per OMG IDL §7.4.4.1
    # the leading-underscore form is stripped at parse time, so the
    # generated wire name has no leading underscore).
    "BoolPubSubType":        "std_msgs::msg::dds_::Bool_",
    "BytePubSubType":        "std_msgs::msg::dds_::Byte_",
    "CharPubSubType":        "std_msgs::msg::dds_::Char_",
    "Int8PubSubType":        "std_msgs::msg::dds_::Int8_",
    "Int16PubSubType":       "std_msgs::msg::dds_::Int16_",
    "Int32PubSubType":       "std_msgs::msg::dds_::Int32_",
    "Int64PubSubType":       "std_msgs::msg::dds_::Int64_",
    "UInt8PubSubType":       "std_msgs::msg::dds_::UInt8_",
    "UInt16PubSubType":      "std_msgs::msg::dds_::UInt16_",
    "UInt32PubSubType":      "std_msgs::msg::dds_::UInt32_",
    "UInt64PubSubType":      "std_msgs::msg::dds_::UInt64_",
    "StringPubSubType":      "std_msgs::msg::dds_::String_",
    # A representative non-escaped std_msgs type — same generation rules,
    # different code path.
    "HeaderPubSubType":      "std_msgs::msg::dds_::Header_",
    # Provizio types that don't collide with IDL keywords — the snake_case
    # source name maps to the snake_case wire name plus one trailing
    # underscore from `-typeros2`.
    "radar_infoPubSubType":          "provizio::msg::dds_::radar_info_",
    "camera_intrinsicsPubSubType":   "provizio::msg::dds_::camera_intrinsics_",
}
for pub_sub_name, expected_wire_name in _WIRE_NAMES.items():
    cls = getattr(provizio_dds, pub_sub_name, None)
    if cls is None:
        # Not every build configuration exposes every type; just skip the
        # missing ones rather than fail the whole compat test.
        print(f"SKIP: {pub_sub_name} (not exported)")
        continue
    actual = cls().getName()
    expect(f"{pub_sub_name}().getName() == {expected_wire_name!r}",
           actual == expected_wire_name)


# The wrapper's high-level entry points should still be importable by their
# 1.10.x names too.
for name in ("Publisher", "Subscriber", "make_domain_participant",
             "RELIABLE_RELIABILITY_QOS", "BEST_EFFORT_RELIABILITY_QOS",
             "TopicDataType", "DataReaderListener", "DataWriterListener"):
    expect(f"provizio_dds.{name} resolves", hasattr(provizio_dds, name))


print("All legacy-API compat checks passed.")
