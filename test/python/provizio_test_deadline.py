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
"""Make a Python test that hangs say what it was doing, before CTest kills it.

CTest kills a timed-out test with SIGKILL, which no handler can catch, so a stuck test
reports nothing at all -- and because a test's stdout is a pipe rather than a terminal,
Python's own buffer is lost with it, so even the lines it already printed never arrive.
A hang therefore shows up as a bare ``***Timeout`` with an empty output block, which is
exactly what it looked like when ``python_vpn_interfaces_caller_builtin_transports_matching_ours``
hung on a jetson runner: no traceback, no partial output, nothing to distinguish a
deadlock from a slow machine.

``test/CMakeLists.txt``'s ``provizio_dds_finalize_tests`` already puts
``PROVIZIO_DDS_TEST_DEADLINE_SEC`` in every test's environment, a few seconds inside its
own TIMEOUT; ``run_parallel.py`` reads it and dumps its children's thread states. This is
the equivalent for a Python test process, and ``test/python/CMakeLists.txt`` starts every
one of them through it::

    python -q -X faulthandler -m provizio_test_deadline ./the_test.py [args...]

which arms the dump and then runs the script as ``__main__``. The launcher form exists so
that no test has to remember to call :func:`arm` -- ``python_callback_exceptions_on_data``
had not, and its one hang in CI (45 s for a 0.6 s test) reported nothing at all. A test
that installs its own ``faulthandler`` watchdog replaces this dump (``faulthandler`` keeps
one timer) and must re-arm it afterwards, as ``python_network_recovery_test`` does.

Reports and nothing else. The verdict stays CTest's, because a test whose own budget runs
close to its TIMEOUT is slow rather than stuck, and killing it here would turn a slow
runner into a failure. ``exit=False`` is what keeps that true.
"""

import faulthandler
import os
import sys
import time

# When this process started, near enough. The deadline below is a budget measured from
# process start (CMake derives it as the ctest TIMEOUT less a margin), so arming it needs to
# know how much of that budget is already spent. Module import is the earliest moment this
# file can observe, and every harness imports it at the top, so the difference from the real
# process start is the interpreter's own startup.
#
# That gap makes the dump slightly LATER than the true deadline, not earlier: with P the real
# process start, this constant is P + e, and a dump armed at any A fires at
# A + (deadline - (A - _IMPORTED_AT)) = P + e + deadline, while the budget's own zero is P. So
# the measured elapsed time is short by e, the remaining budget is long by e, and e is spent
# out of the margin rather than saved from it. What makes that safe is the size of the margin
# (PROVIZIO_DDS_TEST_DEADLINE_MARGIN_SEC, 5 s in test/CMakeLists.txt) against the size of e --
# interpreter startup plus three stdlib imports, since both harnesses import this module
# before provizio_dds and so keep the SWIG load outside it -- and not the direction of e.
_IMPORTED_AT = time.monotonic()


def arm() -> bool:
    """Dump every thread's stack shortly before CTest's TIMEOUT would kill this process.

    Safe to call from anywhere, including after the case body has run: the delay is measured
    from process start rather than from this call, so a late arming still fires before ctest's
    TIMEOUT instead of after it. With one limit -- the one-second floor below means an arming
    later than a second before the deadline fires a second after it, so the guarantee is
    "arm at any point up to a second before the budget runs out", not literally any point.

    Idempotent, as a consequence of counting from process start: calling twice re-schedules
    the dump to the same absolute instant rather than extending it, which the relative form
    this replaces did not give.

    Returns whether a dump was armed: ``False`` when the variable is absent (a test run by
    hand, or one registered with no TIMEOUT) or unusable, which is not an error.
    """
    raw = os.environ.get("PROVIZIO_DDS_TEST_DEADLINE_SEC", "").strip()
    if not raw:
        return False
    try:
        deadline = float(raw)
    except ValueError:
        return False
    if deadline <= 0:
        return False

    # Counted from PROCESS START, not from this call. dump_traceback_later takes a DELAY,
    # while PROVIZIO_DDS_TEST_DEADLINE_SEC is a budget CMake derives from the ctest TIMEOUT --
    # so passing it straight through schedules the dump at `time_already_spent + deadline`,
    # which overruns ctest's SIGKILL by the time spent before arming LESS the margin CMake
    # left -- so it lands after the kill only once more than that margin has been spent, and
    # by that excess rather than by the whole of it. That is not
    # hypothetical: python_network_recovery_test arms this from its `finally`, once the case
    # body has already run, and several of its cases sleep for a second or more -- so the
    # fallback would have been scheduled past the kill for most of them and never fired,
    # silently reinstating the very "hang with no output" this module exists to prevent.
    #
    # Subtracting the elapsed time makes the call site irrelevant, which is what the callers
    # actually need: one arms at the top of main(), the other hands over after the body.
    remaining = deadline - (time.monotonic() - _IMPORTED_AT)
    # A floor rather than a refusal: arming late is exactly when a dump is most wanted, and a
    # non-positive delay would either throw or fire so late it races the kill. One second is
    # enough for the dump to be written and still beat the margin CMake left.
    remaining = max(remaining, 1.0)

    # stderr, not stdout: ctest interleaves both, and stderr is unbuffered, so the dump
    # survives even though whatever the test printed to stdout does not.
    #
    # exit=False: report, never decide. See the module docstring.
    faulthandler.dump_traceback_later(remaining, repeat=False, exit=False, file=sys.stderr)
    return True


def _run_as_launcher() -> None:
    """``python -m provizio_test_deadline <script.py> [args...]``: arm, then run the script.

    The script runs as ``__main__`` with ``sys.argv`` rebased onto it, so it cannot tell it
    was not started directly; its ``sys.exit`` propagates as the process exit code.
    """
    if len(sys.argv) < 2:
        sys.stderr.write("usage: python -m provizio_test_deadline <script.py> [args...]\n")
        sys.exit(2)
    import runpy

    sys.argv = sys.argv[1:]
    arm()
    runpy.run_path(sys.argv[0], run_name="__main__")


if __name__ == "__main__":
    _run_as_launcher()
