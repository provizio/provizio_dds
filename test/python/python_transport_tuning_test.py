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
import threading
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


def test_dynamic_port_range_domain_warns():
    """A domain whose UDP ports fall in the OS's dynamic port range is a discovery hazard the
    caller cannot see -- a taken port moves the participant to another one silently, where
    peers probing by unicast initial peers never look (see "Choosing a domain id" in
    DETAILS.md) -- so creating a participant on one says so, and creating one below the range
    does not. The boundary is per OS: 100/101 on Linux, 166/167 on Windows and macOS. Mirrors
    the C++ transport_tuning dynamic_port_range_domain_warns case."""
    warnings = []
    lock = threading.Lock()

    def recorder(level, message):
        if level == provizio_dds.LogLevel.WARNING:
            with lock:
                warnings.append(message)

    def warned_for(domain):
        with lock:
            warnings.clear()
        participant = provizio_dds.make_domain_participant(domain, provizio_dds.NetworkRecoveryMode.OFF)
        del participant
        prefix = f"DDS domain {domain} maps to UDP ports "
        with lock:
            return any(m.startswith(prefix) and "dynamic port range" in m for m in warnings)

    highest_port = 65535
    rtps_port_base = 7400
    rtps_domain_gain = 250
    if sys.platform in ("win32", "darwin"):
        last_safe = 166
        # The IANA range, which these two genuinely run to the top of -- so no domain sits
        # above it. Both bounds are set in BOTH branches: mid_range_domain below needs the
        # start as well as the end, and setting it only in the else branch is a NameError on
        # the platform that skips it rather than anything a Linux run can see.
        range_start = 49152
        range_end = highest_port
    else:
        # The library reads net.ipv4.ip_local_port_range; judge it by the same numbers.
        range_start = 32768
        range_end = highest_port
        try:
            with open("/proc/sys/net/ipv4/ip_local_port_range", encoding="ascii") as sysctl:
                first, last = (int(field) for field in sysctl.read().split()[:2])
            if 0 < first <= last <= highest_port:
                range_start, range_end = first, last
        except (OSError, ValueError, IndexError):
            pass
        if range_start < rtps_port_base + rtps_domain_gain:
            # The library returns -1 for any range starting below rtps_port_base +
            # rtps_domain_gain (not merely at or below the base): domain 0's own ports already
            # reach into it. This arithmetic goes negative across that whole window. Judge it
            # the same way rather than creating a participant on domain -1.
            print(
                "dynamic_port_range_domain_warns: SKIP (this host's dynamic range starts at "
                f"{range_start}, at or below the RTPS port base -- no domain choice helps)"
            )
            return
        last_safe = (range_start - rtps_port_base) // rtps_domain_gain - 1

    # A second in-range domain, well past the boundary -- derived from the range rather than
    # hard-coded. 200 was hard-coded here and is only in-range on a host whose ephemeral range
    # happens to span its ports (7400 + 250*200 = 57400-57649): with 32768-50000, or
    # 60000-65535, domain 200 correctly does not warn and the assertion failed on correct
    # behaviour.
    mid_range_domain = last_safe + 1 + ((range_end - range_start) // rtps_domain_gain) // 2

    # The OTHER edge, and the one nothing covered: the range has a TOP. Linux's default stops
    # at 60999, so domains from 215 up map entirely above it and are the safest a 16-bit port
    # can be -- yet "at or above range_start" warned about every one of them. Skipped where the
    # range really does run to 65535 (Windows, macOS, or a host that tuned it that way): there
    # is no such domain to test, and 200 above already covers the in-range case there.
    first_above = (range_end - rtps_port_base) // rtps_domain_gain + 1
    range_has_a_top = (
        range_end < highest_port
        and rtps_port_base + rtps_domain_gain * (first_above + 1) - 1 <= highest_port
    )

    provizio_dds.set_log_callback(recorder)
    try:
        quiet_on_default = not warned_for(DOMAIN)
        quiet_on_last_safe = not warned_for(last_safe)
        warns_on_first_unsafe = warned_for(last_safe + 1)
        warns_high = warned_for(mid_range_domain)
        quiet_above_range = not range_has_a_top or not warned_for(first_above)
    finally:
        provizio_dds.set_log_callback(None)
    expect(quiet_on_default, f"no warning on domain {DOMAIN}")
    expect(quiet_on_last_safe, f"no warning on domain {last_safe}")
    expect(warns_on_first_unsafe, f"warning on domain {last_safe + 1}")
    expect(warns_high, f"warning on domain {mid_range_domain}")
    expect(
        quiet_above_range,
        f"no warning on domain {first_above} (entirely above the range)"
        if range_has_a_top
        else "no domain above the range on this host (n/a)",
    )
    above = (
        f"quiet above the range (domain {first_above})"
        if range_has_a_top
        else "quiet above the range (n/a, range runs to 65535)"
    )
    print(
        f"dynamic_port_range_domain_warns: {'PASS' if not _failures else 'FAIL'} "
        f"(quiet on {DOMAIN} and {last_safe}, warns on {last_safe + 1} and {mid_range_domain}, {above})"
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
    "dynamic_port_range_domain_warns": test_dynamic_port_range_domain_warns,
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
