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
"""Cross-language parity of the per-type QoS default overrides.

The C++ side declares them as ``qos_defaults<T>`` specializations
(include/provizio/dds/qos_defaults.h); the Python side registers the same type set
with the same values in ``_register_per_type_qos_defaults()``. Nothing else ties
the two together, so a type or a value changed on one side only would silently
ship mismatched defaults -- and these particular defaults fail invisibly. A
publisher left on the primary template's synchronous publish and KEEP_LAST(1) is
the retransmit-starvation scenario the overrides exist to prevent (this exact
drift happened once with the freespace polygon types), and a subscriber left on
Fast-DDS's KEEP_LAST(1) silently loses samples on a keyless topic the whole fleet
shares, with no RTPS loss recorded and nothing logged.

C++ is the source of truth, read from two files:

* the header supplies the LIST of specialized types, which is what catches a type
  added on one side only; and
* src/qos_defaults_checks.cpp supplies the VALUES, because it already pins every
  one of them in a static_assert against the real generated types. Reading the
  values from there rather than re-parsing the header's inheritance tiers means
  the C++ compiler, not a regex in this file, is what guarantees they match the
  header -- the checks translation unit does not build otherwise.

Both paths are arguments, provided by the CMake registration from the source
tree. The checks file's deliberately-unspecialized control type (std_msgs/String)
doubles as the parity check for the primary (None) defaults."""

import re
import sys

import provizio_dds

_failures = []


def expect(condition, description):
    """Records a failure (and prints it) unless the condition holds.

    :param condition: The condition that must hold.
    :param str description: What was expected, printed and collected on failure.
    :return: The condition, so a caller can skip dependent checks.
    """
    if not condition:
        print(f"FAIL: {description}")
        _failures.append(description)
    return condition


def read(path):
    """Reads a source file into a string.

    :param str path: Path to the file.
    :return str: Its contents.
    """
    with open(path, encoding="utf-8") as source_file:
        return source_file.read()


def parse_specialized_type_names(header):
    """Extracts the set of types the C++ header specializes qos_defaults for.

    :param str header: Contents of include/provizio/dds/qos_defaults.h.
    :return set: Leaf pub/sub type names, e.g. {"ImagePubSubType", ...}.
    """
    # Matches e.g. "struct qos_defaults<::sensor_msgs::msg::ImagePubSubType> final".
    return set(re.findall(r"struct qos_defaults<::[A-Za-z_]+::msg::(\w+)>", header))


def parse_checked_values(checks):
    """Extracts, per type, the QoS values src/qos_defaults_checks.cpp pins for it.

    The checks file states each expectation through a small predicate taking the
    expected value, and names its depths with local ``expected_*`` constants, so
    parsing is a matter of resolving those constants and then reading one call per
    predicate. A type is included only once every predicate has been seen for it,
    which is also how a half-written assertion gets noticed.

    :param str checks: Contents of src/qos_defaults_checks.cpp.
    :return dict: Leaf type name -> dict with "publish_mode", "reliability",
        "reader_depth" and "writer_depth" (modes and kinds as their C++ token
        names, depths as ints).
    """
    depth_constants = {
        name: int(value)
        for name, value in re.findall(
            r"constexpr\s+std::int32_t\s+(expected_\w+)\s*=\s*(\d+);", checks
        )
    }

    def leaf(qualified_name):
        return qualified_name.rsplit("::", 1)[-1]

    values = {}
    for qualified_name, mode in re.findall(
        r"publishes<([\w:]+)>\(\s*(\w+)\s*\)", checks
    ):
        values.setdefault(leaf(qualified_name), {})["publish_mode"] = mode
    for qualified_name, kind in re.findall(
        r"writes_with_reliability<([\w:]+)>\(\s*(\w+)\s*\)", checks
    ):
        values.setdefault(leaf(qualified_name), {})["reliability"] = kind
    for qualified_name, reader, writer in re.findall(
        r"keeps_history<([\w:]+)>\(\s*(\w+)\s*,\s*(\w+)\s*\)", checks, re.S
    ):
        entry = values.setdefault(leaf(qualified_name), {})
        entry["reader_depth"] = depth_constants.get(reader)
        entry["writer_depth"] = depth_constants.get(writer)

    complete = {}
    for type_name, entry in values.items():
        missing = {
            "publish_mode",
            "reliability",
            "reader_depth",
            "writer_depth",
        } - {key for key, value in entry.items() if value is not None}
        if expect(
            not missing,
            f"src/qos_defaults_checks.cpp pins {type_name} incompletely, missing "
            f"{sorted(missing)} -- either the assertion is half written or this "
            f"test's parsing has drifted from the file's shape",
        ):
            complete[type_name] = entry
    return complete


def expect_python_matches(type_name, checked, publish_mode, reliability, reader_depth, writer_depth):
    """Asserts one type's Python per-type registrations equal the C++ expectations.

    :param str type_name: Leaf pub/sub type name, for failure messages.
    :param dict checked: The C++ expectations for this type, as parsed.
    :param publish_mode: The Python publish mode registered for the type.
    :param reliability: The Python datawriter reliability registered for the type.
    :param int reader_depth: The Python datareader history depth for the type.
    :param int writer_depth: The Python datawriter history depth for the type.
    """
    expect(
        publish_mode == getattr(provizio_dds, checked["publish_mode"]),
        f"{type_name} must default to {checked['publish_mode']} in Python too",
    )
    expect(
        reliability == getattr(provizio_dds, checked["reliability"]),
        f"{type_name} datawriter must default to {checked['reliability']} in Python too",
    )
    expect(
        reader_depth == checked["reader_depth"],
        f"{type_name} datareader must default to KEEP_LAST({checked['reader_depth']}) in "
        f"Python too, not {reader_depth}",
    )
    expect(
        writer_depth == checked["writer_depth"],
        f"{type_name} datawriter must default to KEEP_LAST({checked['writer_depth']}) in "
        f"Python too, not {writer_depth}",
    )


# The type the C++ checks deliberately leave unspecialized, to pin what an ordinary type gets.
# Its expectations are the parity check for Python's primary (None) dict entries, so it must
# NOT appear among the specialized types on either side.
UNSPECIALIZED_CONTROL_TYPE = "StringPubSubType"


def main():
    """Runs the parity checks.

    :return int: 0 if every check passed, 1 otherwise.
    """
    if len(sys.argv) < 3:
        print(
            f"usage: {sys.argv[0]} <path/to/qos_defaults.h> <path/to/qos_defaults_checks.cpp>",
            file=sys.stderr,
        )
        return 1

    specialized = parse_specialized_type_names(read(sys.argv[1]))
    expect(
        specialized,
        "no qos_defaults specializations parsed -- header layout or this regex drifted",
    )
    checked = parse_checked_values(read(sys.argv[2]))
    expect(
        checked,
        "no pinned QoS values parsed -- qos_defaults_checks.cpp layout or these regexes drifted",
    )

    # Every C++ specialization must be pinned by the checks file, or its values reach nobody:
    # this test would compare Python against nothing and pass.
    expect(
        specialized <= set(checked),
        f"C++ qos_defaults specializations that src/qos_defaults_checks.cpp does not pin: "
        f"{sorted(specialized - set(checked))}",
    )
    expect(
        UNSPECIALIZED_CONTROL_TYPE in checked,
        f"src/qos_defaults_checks.cpp no longer pins {UNSPECIALIZED_CONTROL_TYPE} as the "
        f"unspecialized control -- the primary (None) defaults are then unchecked here",
    )
    expect(
        UNSPECIALIZED_CONTROL_TYPE not in specialized,
        f"{UNSPECIALIZED_CONTROL_TYPE} is now specialized in C++, so it can no longer serve "
        f"as the unspecialized control -- pick another type for that role",
    )

    registered = {
        cls.__name__
        for cls in provizio_dds.QosDefaults.datawriter_publish_mode_per_type
        if cls is not None
    }
    # Types whose generated binding is absent from this build register nothing -- the
    # lookup in _register_per_type_qos_defaults is guarded. Only compare the types this
    # build actually ships, so a trimmed-types build can't produce a false drift.
    shipped_specialized = {name for name in specialized if hasattr(provizio_dds, name)}

    expect(
        shipped_specialized <= registered,
        f"C++ qos_defaults specializations missing from the Python registration: "
        f"{sorted(shipped_specialized - registered)}",
    )
    expect(
        registered <= specialized,
        f"Python-registered types with no C++ qos_defaults specialization: "
        f"{sorted(registered - specialized)}",
    )

    for type_name in sorted(shipped_specialized & registered & set(checked)):
        cls = getattr(provizio_dds, type_name)
        expect_python_matches(
            type_name,
            checked[type_name],
            provizio_dds.QosDefaults.datawriter_publish_mode_per_type[cls],
            provizio_dds.QosDefaults.datawriter_reliability_kind_per_type[cls],
            provizio_dds.QosDefaults.datareader_keep_last_history_depth_per_type[cls],
            provizio_dds.QosDefaults.datawriter_keep_last_history_depth_per_type[cls],
        )
        # Every subscriber is meant to keep adopting its publisher's reliability, including
        # for the best-effort Image writer: only the writer side of a reliability decision
        # is ever stated per type.
        expect(
            provizio_dds.QosDefaults.datareader_reliability_kind_per_type[cls]
            == provizio_dds.MATCH_PUBLISHER_RELIABILITY_QOS,
            f"{type_name} datareader must keep the match-publisher reliability default",
        )

    # And the primary (None) defaults, against the C++ unspecialized control type.
    if UNSPECIALIZED_CONTROL_TYPE in checked:
        expect_python_matches(
            f"an unspecialized type (via {UNSPECIALIZED_CONTROL_TYPE})",
            checked[UNSPECIALIZED_CONTROL_TYPE],
            provizio_dds.QosDefaults.datawriter_publish_mode_per_type[None],
            provizio_dds.QosDefaults.datawriter_reliability_kind_per_type[None],
            provizio_dds.QosDefaults.datareader_keep_last_history_depth_per_type[None],
            provizio_dds.QosDefaults.datawriter_keep_last_history_depth_per_type[None],
        )

    # The pre-split dict must be gone rather than merely unread: an entry left in it would
    # be silently ignored, which is the whole failure mode the split had to avoid.
    try:
        provizio_dds.QosDefaults.keep_last_history_depth_per_type[None] = 1
    except AttributeError:
        pass
    else:
        expect(
            False,
            "QosDefaults.keep_last_history_depth_per_type still accepts entries -- a stale "
            "per-type depth would be registered there and never read",
        )

    print(
        f"per_type_qos_parity: {'PASS' if not _failures else 'FAIL'} "
        f"({len(shipped_specialized)} shipped of {len(specialized)} specialized, "
        f"{len(registered)} registered, {len(checked)} pinned in C++)"
    )
    return 0 if not _failures else 1


if __name__ == "__main__":
    sys.exit(main())
