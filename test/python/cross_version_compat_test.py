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

"""End-to-end wire-interop test against provizio_dds 1.10.1.

Boots one process with the *current* (2.x) provizio_dds python wrapper and a
sibling process with a pip-installed 1.10.1 build in a venv, then exercises
every wire-level interaction across the version boundary:

  1. pub(2.x)              → sub(1.10.1)              [std_msgs/String]
  2. pub(1.10.1)           → sub(2.x)                 [std_msgs/String]
  3. pub(2.x)              → sub(1.10.1)              [provizio/radar_info]
  4. pub(1.10.1)           → sub(2.x)                 [provizio/radar_info]
  5. pub(2.x)              → sub(1.10.1)              [sensor_msgs/PointCloud2]
  6. pub(1.10.1)           → sub(2.x)                 [sensor_msgs/PointCloud2]
  7. req(2.x)              → service(1.10.1)
  8. req(1.10.1)           → service(2.x)

The dedicated radar_info pair (#3, #4) is the load-bearing check for the
APT-11626 enum -> uint32 migration on `radar_range`: discovery must still
match across the version boundary (the .msg comment claims the topic-type
name carries through despite XTypes type-identity drift) and the CDR
payload must decode field-for-field on both sides. The PointCloud2 pair
(#5, #6) covers Provizio's dominant on-wire payload — a regression there
would silently lose entire radar frames across mixed-version fleets, so
the subscriber asserts not just metadata equality but byte-for-byte
agreement on the raw `data` blob.

Pass/fail is determined by the child processes — each side exits 0 only after
it has done what it expected (received the right value, served the expected
request count, etc.), and the harness here keys solely on the exit codes.
The 1.10.x install is the canonical wire-compat reference: any failure here
means a topic name, schema layout, or CDR encoding has drifted since the
deployed-fleet baseline.

This test is also runnable as a standalone script outside ctest. Both sides
use thin cross_compat/*.py entry-points sibling to this file — they import
provizio_dds from whichever site-packages each interpreter resolves (the
local build for the current-version process, the venv install for the
legacy one).

Required environment:
  * PROVIZIO_DDS_LEGACY_PYTHON — absolute path to the python3 interpreter
    of a venv with provizio_dds 1.10.1 installed. The venv is set up by
    `test/python/setup_legacy_provizio_dds_venv.sh` (see that script for
    install steps). If unset, the test prints a SKIP marker and exits 0
    so CI environments without the legacy install don't block on it.

Parallelism note: the cross_compat/*.py scripts run on DDS domain 42 and
use their own topic + service names so this test can execute concurrently
with the same-version suite (which uses domain 0 for pub/sub and domain
14 for request/response). That keeps the heavy request_response_reliability
tests from gating the cross-version verification.

The current-build python interpreter is `sys.executable`. The cross_compat
scripts are sibling to this file, so they import provizio_dds from
whichever site-packages each interpreter resolves — the local build for
the current-version process, the venv install for the legacy one.
"""

import os
import subprocess
import sys
import threading
import time
from pathlib import Path


LEGACY_ENV_VAR = "PROVIZIO_DDS_LEGACY_PYTHON"
HERE = Path(__file__).resolve().parent
CURRENT_PYTHON = sys.executable


def skip(reason: str) -> None:
    # The leading-anchor "SKIPPED:" marker is what the matching CMake
    # `set_tests_properties(... SKIP_REGULAR_EXPRESSION ...)` looks for so
    # ctest reports the test as `Skipped` (not silent green) — important
    # because a missing legacy venv on a CI runner is a setup gap worth
    # noticing, not pretending we ran wire-interop checks we actually
    # didn't.
    print(f"SKIPPED: cross_version_compat — {reason}")
    sys.exit(0)


def fail(stage: str, *outputs: str) -> None:
    print(f"FAIL {stage}")
    for chunk in outputs:
        if chunk:
            print(chunk.rstrip())
    sys.exit(1)


def _drain_into(proc: subprocess.Popen, sink: list) -> None:
    """Read proc.stdout into `sink` (one chunk per read) until EOF. Runs in
    a worker thread per child so neither side's kernel pipe buffer (Linux
    default ~64 KB) can fill while the main thread is waiting on the
    sibling and deadlock both."""
    try:
        while True:
            chunk = proc.stdout.read(4096)
            if not chunk:
                break
            sink.append(chunk)
    except (ValueError, OSError):
        # Pipe closed mid-read (e.g. after .kill()) — fine, we just stop.
        pass


def _kill_quietly(proc: subprocess.Popen) -> None:
    """SIGKILL `proc` if it is still alive and wait briefly for it to exit
    so its drain thread can see EOF and unblock. Used to reap the sibling
    of a failed/timed-out child so it cannot keep speaking on the wire
    after we have already decided the pair failed."""
    if proc.poll() is None:
        proc.kill()
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        # Child ignored SIGKILL somehow; leave it — better than blocking
        # the whole suite. The drain thread is daemon and will be torn
        # down at interpreter exit.
        pass


def run_pair(stage: str, cmd_a, cmd_b, timeout: float = 25.0) -> None:
    """Run two child processes in parallel; both must exit cleanly within
    `timeout` for the pair to count as a pass. This mirrors what
    `test/run_parallel.py` already does for the same-version tests so the
    behaviour is consistent across the suite."""
    print(f"--- {stage} ---")
    print(f"  A: {' '.join(cmd_a)}")
    print(f"  B: {' '.join(cmd_b)}")
    proc_a = subprocess.Popen(cmd_a, cwd=str(HERE), stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True)
    # Tiny stagger: bringing both endpoints up at the exact same instant
    # occasionally misses the first match on slow CI runners. 100 ms is
    # plenty and doesn't measurably extend the test.
    time.sleep(0.1)
    proc_b = subprocess.Popen(cmd_b, cwd=str(HERE), stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True)

    # Drain both children's stdout concurrently — sequential communicate()
    # calls would let the not-yet-waited child block on its pipe buffer
    # filling, hanging the whole pair.
    buf_a: list = []
    buf_b: list = []
    t_a = threading.Thread(target=_drain_into, args=(proc_a, buf_a), daemon=True)
    t_b = threading.Thread(target=_drain_into, args=(proc_b, buf_b), daemon=True)
    t_a.start()
    t_b.start()

    deadline = time.monotonic() + timeout

    def join_drains() -> None:
        # 5s is enough for the kernel to flush a closed pipe; if a thread
        # is still stuck after that the child is hung and we have already
        # killed it, so further waiting buys nothing.
        t_a.join(timeout=5.0)
        t_b.join(timeout=5.0)

    # On any failure path we tear down the sibling before calling fail() so
    # no orphaned DDS endpoint keeps publishing on the test domain (which
    # would race subsequent ctest runs of the same fixtures).
    for proc, other, slot in ((proc_a, proc_b, "A"), (proc_b, proc_a, "B")):
        remaining = max(0.0, deadline - time.monotonic())
        try:
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            _kill_quietly(proc)
            _kill_quietly(other)
            join_drains()
            fail(f"{stage}: {slot} timed out", "".join(buf_a), "".join(buf_b))
        if proc.returncode != 0:
            _kill_quietly(other)
            join_drains()
            fail(f"{stage}: {slot} exited {proc.returncode}",
                 "".join(buf_a), "".join(buf_b))

    join_drains()
    print(f"OK {stage}")
    print("".join(buf_a).rstrip())
    print("".join(buf_b).rstrip())


def main() -> int:
    legacy_python = os.environ.get(LEGACY_ENV_VAR)
    if not legacy_python:
        skip(f"{LEGACY_ENV_VAR} unset")
    if not Path(legacy_python).is_file():
        skip(f"{LEGACY_ENV_VAR} points at non-existent file: {legacy_python}")

    # Sanity-check that the legacy interpreter can actually import the
    # 1.10.x wrapper from its venv. `-P` strips the script-directory entry
    # (or cwd for `-c`) from sys.path[0] — without it, the current build's
    # `provizio_dds.py` copied into this test directory would shadow the
    # venv-installed `provizio_dds/` package and the probe (plus every
    # cross_compat_*.py run below) would silently exercise current-vs-
    # current. Verify the resolved path lives under the venv to catch any
    # future Python version where -P semantics change.
    probe = subprocess.run(
        [legacy_python, "-P", "-c",
         "import provizio_dds; print(provizio_dds.__file__)"],
        capture_output=True, text=True)
    if probe.returncode != 0:
        skip(f"legacy interpreter cannot import provizio_dds:\n{probe.stderr}")
    resolved = probe.stdout.strip()
    if str(HERE) in resolved or not resolved:
        # Defensive: -P should have prevented this, but assert anyway so a
        # silent regression in shadowing logic can't masquerade as a pass.
        skip(f"legacy provizio_dds resolved to test dir (shadowing): {resolved!r}")
    print(f"legacy provizio_dds: {resolved}")

    pub = ["./cross_compat_publisher.py"]
    sub = ["./cross_compat_subscriber.py"]
    radar_pub = ["./cross_compat_radar_info_publisher.py"]
    radar_sub = ["./cross_compat_radar_info_subscriber.py"]
    pc2_pub = ["./cross_compat_pointcloud2_publisher.py"]
    pc2_sub = ["./cross_compat_pointcloud2_subscriber.py"]
    req = ["./cross_compat_request_client.py"]
    svc = ["./cross_compat_request_service.py"]

    def py_cur(script):
        return [CURRENT_PYTHON, "-q", "-X", "faulthandler", *script]

    def py_legacy(script):
        # `-P` is mandatory: this test directory contains the *current*
        # build's `provizio_dds.py` (copied by the python_tests CMake
        # target), and Python's default behaviour of prepending the
        # script directory to sys.path[0] would shadow the venv-installed
        # 1.10.1 package — turning the cross-version test into a silent
        # current-vs-current test. -P (Python 3.11+) suppresses that
        # prepend so the venv's site-packages provizio_dds wins.
        return [legacy_python, "-P", "-q", "-X", "faulthandler", *script]

    run_pair("pub(2.x) -> sub(1.10.1)", py_cur(pub), py_legacy(sub))
    run_pair("pub(1.10.1) -> sub(2.x)", py_legacy(pub), py_cur(sub))
    run_pair("radar_info pub(2.x) -> sub(1.10.1)",
             py_cur(radar_pub), py_legacy(radar_sub))
    run_pair("radar_info pub(1.10.1) -> sub(2.x)",
             py_legacy(radar_pub), py_cur(radar_sub))
    run_pair("PointCloud2 pub(2.x) -> sub(1.10.1)",
             py_cur(pc2_pub), py_legacy(pc2_sub))
    run_pair("PointCloud2 pub(1.10.1) -> sub(2.x)",
             py_legacy(pc2_pub), py_cur(pc2_sub))
    run_pair("req(2.x) -> svc(1.10.1)", py_cur(req), py_legacy(svc), timeout=40.0)
    run_pair("req(1.10.1) -> svc(2.x)", py_legacy(req), py_cur(svc), timeout=40.0)

    print("All cross-version compat checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
