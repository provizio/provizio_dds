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
"""Subcommand-driven tests for PROVIZIO_DDS_LOCALHOST_ONLY in the Python API.

The C++ API turns the variable into transport_mode::localhost_only for every participant that
chose no mode. Python has no such mode, so it must not pretend: a participant created while the
variable is set to an on value logs one warning, once per process, saying the variable does not apply --
and says nothing for an explicit transport, an off value, or an unset variable. Each case is its
own process (ctest registers the environment), because the notice is once per process."""

import os
import sys
import traceback

import provizio_dds
import provizio_test_domain

DOMAIN = provizio_test_domain.random_test_domain()
_failures = []


def expect(condition, description):
    if not condition:
        print(f"FAIL: {description}")
        _failures.append(description)
    return condition


def _capture_warnings(create):
    """Run ``create`` with a log callback installed; return the warnings that name the variable."""
    captured = []
    previous = provizio_dds.set_log_callback(lambda level, message: captured.append((level, message)))
    try:
        create()
    finally:
        provizio_dds.set_log_callback(previous)
    return [
        message
        for level, message in captured
        if level == provizio_dds.LogLevel.WARNING and "PROVIZIO_DDS_LOCALHOST_ONLY" in message
    ]


def test_warns_once():
    """Two default participants, one notice."""

    def create():
        first = provizio_dds.make_domain_participant(DOMAIN)
        second = provizio_dds.make_domain_participant(DOMAIN)
        del first, second

    warnings = _capture_warnings(create)
    expect(len(warnings) == 1, f"exactly one notice expected, got {len(warnings)}: {warnings}")
    expect(
        all("does not apply to the Python API" in message for message in warnings),
        "the notice must say the variable does not apply in Python",
    )


def test_silent():
    """Off values, an empty value and an unset variable say nothing."""
    warnings = _capture_warnings(lambda: provizio_dds.make_domain_participant(DOMAIN))
    expect(not warnings, f"no notice expected, got {warnings}")


def test_explicit_transport_silent():
    """A transport chosen in code is not what the variable is about: no notice."""
    warnings = _capture_warnings(
        lambda: provizio_dds.make_domain_participant(DOMAIN, transport=provizio_dds.TransportMode.UDP_ONLY)
    )
    expect(not warnings, f"no notice expected for an explicit transport, got {warnings}")


_SUBCOMMANDS = {
    "warns_once": test_warns_once,
    "silent": test_silent,
    "explicit_transport_silent": test_explicit_transport_silent,
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in _SUBCOMMANDS:
        print(f"usage: {sys.argv[0]} <{'|'.join(_SUBCOMMANDS)}>", file=sys.stderr)
        return 1
    # An ambient profile would make the library skip its own transport handling, and with it nothing
    # here would be reached the way a deployed application reaches it.
    os.environ.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
    try:
        _SUBCOMMANDS[sys.argv[1]]()
    except Exception:  # noqa: BLE001 - surface any unexpected error as a failure
        traceback.print_exc()
        return 1
    print(f"{sys.argv[1]}: {'PASS' if not _failures else 'FAIL'}")
    return 0 if not _failures else 1


if __name__ == "__main__":
    sys.exit(main())
