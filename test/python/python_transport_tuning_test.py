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
"""Subcommand-driven tests for participant transport tuning — Python mirror of
the C++ test/transport_tuning/ suite.

They cover the MTU-sized default of the send-side RTPS message-size cap (the
fastdds.max_message_size participant property, which keeps every UDP datagram
within a single link frame so large samples travel as individually-
retransmittable RTPS fragments on lossy networks), the
PROVIZIO_DDS_MAX_MESSAGE_SIZE environment override, and the fallback to the
default on malformed input — asserted by reading the configured QoS back off a
freshly created participant. Each subcommand is its own ctest entry so
per-case failure stays isolated, mirroring python_discovery_tuning_test.py."""

import os
import sys
import traceback

import provizio_dds

DOMAIN = 0

# The expected default (kept in sync with the C++ test/transport_tuning/ suite,
# src/domain_participant.cpp and _resolve_max_message_size in provizio_dds.py):
# one ~1500-byte-MTU link frame per UDP datagram.
DEFAULT_MAX_MESSAGE_SIZE = "1400"
PROPERTY_NAME = "fastdds.max_message_size"
ENV_NAME = "PROVIZIO_DDS_MAX_MESSAGE_SIZE"

_failures = []


def expect(condition, description):
    if not condition:
        print(f"FAIL: {description}")
        _failures.append(description)
    return condition


def _max_message_size_of_fresh_participant():
    """The fastdds.max_message_size property read back off a freshly created
    participant's EFFECTIVE QoS (as the created Fast-DDS participant reports it,
    not the cached request — mirroring the C++ test's fastdds_participant()
    read-back), or None when absent."""
    # network_recovery OFF keeps each case self-contained / fast. Mirrors the C++
    # make_domain_participant(..., network_recovery_mode::off).
    participant = provizio_dds.make_domain_participant(
        DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
    )
    effective_qos = provizio_dds.DomainParticipantQos()
    participant._participant.get_qos(effective_qos)
    properties = effective_qos.properties().properties()
    for i in range(properties.size()):
        if properties[i].name() == PROPERTY_NAME:
            return properties[i].value()
    return None


def test_defaults():
    """With no env override a participant is configured with the MTU-sized
    default cap — NOT Fast-DDS's 65500 single-datagram default (which makes
    large-sample delivery all-or-nothing under frame loss)."""
    os.environ.pop(ENV_NAME, None)  # hermetic: ignore any ambient override
    value = _max_message_size_of_fresh_participant()
    expect(value == DEFAULT_MAX_MESSAGE_SIZE, f"{PROPERTY_NAME} {value} == {DEFAULT_MAX_MESSAGE_SIZE}")
    print(f"defaults: {'PASS' if not _failures else 'FAIL'} ({PROPERTY_NAME}={value})")


def test_env_override():
    """A valid PROVIZIO_DDS_MAX_MESSAGE_SIZE env value is honoured verbatim
    (e.g. raised back to Fast-DDS's 65500 maximum by hosts publishing multi-MB
    samples over clean links, to trade loss resilience back for CPU), down to
    the 576-byte floor below which discovery announcements no longer fit."""
    os.environ[ENV_NAME] = "65500"
    max_value = _max_message_size_of_fresh_participant()
    expect(max_value == "65500", f"{PROPERTY_NAME} {max_value} == 65500")
    os.environ[ENV_NAME] = "576"  # the minimum accepted value
    min_value = _max_message_size_of_fresh_participant()
    expect(min_value == "576", f"{PROPERTY_NAME} {min_value} == 576")
    # Above Fast-DDS's 65500 maximum the value is clamped (with a warning), not honoured
    # verbatim and not rejected to the default: the likeliest cause is a typo by someone
    # who meant "as large as possible".
    os.environ[ENV_NAME] = "70000"
    clamped_value = _max_message_size_of_fresh_participant()
    expect(clamped_value == "65500", f"{PROPERTY_NAME} {clamped_value} == 65500 (clamped)")
    print(
        f"env_override: {'PASS' if not _failures else 'FAIL'} "
        f"({PROPERTY_NAME}={max_value}, {min_value}, {clamped_value})"
    )


def test_env_invalid():
    """Malformed / out-of-range env values are ignored and the default wins, so
    a typo can never silently disable the fragmentation cap."""
    # "-18446744073709551615" locks the explicit minus rejection (C++ strtoull alone would
    # wrap it around to 1); the 5000-digit string locks the fallback on input beyond
    # int()'s conversion-length limit (Python 3.11+), which raises instead of parsing;
    # "100" and "575" lock the 576-byte floor below which discovery announcements no
    # longer fit one message.
    for invalid in (
        "not-a-number",
        "0",
        "-5",
        "1400x",
        "99999999999999999999",
        "-18446744073709551615",
        "1" * 5000,
        "100",
        "575",
    ):
        os.environ[ENV_NAME] = invalid
        value = _max_message_size_of_fresh_participant()
        expect(
            value == DEFAULT_MAX_MESSAGE_SIZE,
            f"{ENV_NAME}='{invalid}' produced {PROPERTY_NAME}={value}, expected {DEFAULT_MAX_MESSAGE_SIZE}",
        )
    print(f"env_invalid: {'PASS' if not _failures else 'FAIL'}")


_SUBCOMMANDS = {
    "defaults": test_defaults,
    "env_override": test_env_override,
    "env_invalid": test_env_invalid,
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in _SUBCOMMANDS:
        print(f"usage: {sys.argv[0]} <{'|'.join(_SUBCOMMANDS)}>", file=sys.stderr)
        return 1
    # Hermetic across the whole suite: an ambient FASTDDS_DEFAULT_PROFILES_FILE pointing at a real XML
    # profile makes the library skip its code-driven transport tuning entirely, which would break every
    # subcommand here (defaults and env override alike). Clear it so the tests exercise the code path.
    # Only the participant's own QoS is read back — no pub/sub traffic — so leaving loopback confinement
    # is safe even on hosts sharing a LAN with concurrent CI runs.
    os.environ.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
    try:
        _SUBCOMMANDS[sys.argv[1]]()
    except Exception:  # noqa: BLE001 — surface any unexpected error as a failure
        traceback.print_exc()
        return 1
    return 0 if not _failures else 1


if __name__ == "__main__":
    sys.exit(main())
