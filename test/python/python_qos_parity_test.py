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
"""Cross-language parity of the large-sample QoS default overrides.

The C++ side declares them as ``qos_defaults<T>`` specializations
(include/provizio/dds/qos_defaults.h); the Python side registers the same type
set in ``_register_large_sample_qos_defaults()``. Nothing else ties the two
lists together, so a type added on one side only would silently ship with
mismatched defaults — a Python publisher of that type would keep SYNCHRONOUS
publish and KEEP_LAST(1), the retransmit-starvation scenario the overrides
exist to prevent (this exact drift happened with the freespace polygon types).

The C++ header is the source of truth: this test parses its specialization list
and asserts the Python registration covers it exactly — both directions, so a
Python-only extra is flagged as drift too. Takes the header path as its
argument (provided by the CMake registration from the source tree)."""

import re
import sys

import provizio_dds

_failures = []


def expect(condition, description):
    if not condition:
        print(f"FAIL: {description}")
        _failures.append(description)
    return condition


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <path/to/qos_defaults.h>", file=sys.stderr)
        return 1

    with open(sys.argv[1], encoding="utf-8") as header_file:
        header = header_file.read()

    # Matches e.g. "struct qos_defaults<::sensor_msgs::msg::ImagePubSubType> final".
    specialized = set(re.findall(r"struct qos_defaults<::[A-Za-z_]+::msg::(\w+)>", header))
    expect(specialized, "no qos_defaults specializations parsed -- header layout or this regex drifted")

    registered = {
        cls.__name__
        for cls in provizio_dds.QosDefaults.datawriter_publish_mode_per_type
        if cls is not None
    }
    # Types whose generated binding is absent from this build register nothing — the
    # lookup in _register_large_sample_qos_defaults is guarded. Only compare the types
    # this build actually ships, so a trimmed-types build can't produce a false drift.
    shipped_specialized = {name for name in specialized if hasattr(provizio_dds, name)}

    expect(
        shipped_specialized <= registered,
        f"C++ large-sample specializations missing from the Python registration: "
        f"{sorted(shipped_specialized - registered)}",
    )
    expect(
        registered <= specialized,
        f"Python-registered large-sample types with no C++ qos_defaults specialization: "
        f"{sorted(registered - specialized)}",
    )

    # The per-type override values must also agree with detail::large_sample_qos_defaults.
    for name in sorted(shipped_specialized & registered):
        cls = getattr(provizio_dds, name)
        expect(
            provizio_dds.QosDefaults.datawriter_publish_mode_per_type[cls]
            == provizio_dds.ASYNCHRONOUS_PUBLISH_MODE,
            f"{name} must default to ASYNCHRONOUS_PUBLISH_MODE",
        )
        expect(
            provizio_dds.QosDefaults.keep_last_history_depth_per_type[cls] == 4,
            f"{name} must default to KEEP_LAST(4)",
        )

    print(
        f"large_sample_qos_parity: {'PASS' if not _failures else 'FAIL'} "
        f"({len(shipped_specialized)} shipped of {len(specialized)} specialized, "
        f"{len(registered)} registered)"
    )
    return 0 if not _failures else 1


if __name__ == "__main__":
    sys.exit(main())
