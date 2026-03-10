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

# Runs command N times, replacing {i} with iteration number
# Arguments are preserved as separate entries to maintain proper quoting
# when nesting with run_parallel.py
# Also handles ROS environment cleanup
#
# Usage: run_times.py [--delay D] [--timeout T] N command...
#   --delay D:   sleep D seconds between iterations to let OS release DDS resources (default 0)
#   --timeout T: kill iteration if it takes longer than T seconds (default: no timeout)

import os
import signal
import sys
import subprocess
import time

# Unset ROS environment variables to avoid interference (cross-platform equivalent of unset_ros.sh)
ament = os.environ.get("AMENT_PREFIX_PATH", "")
if ament:
    # AMENT_PREFIX_PATH can be a pathsep-separated list of prefixes
    ament_prefixes = tuple(ament.split(os.pathsep))
    for var in ("PYTHONPATH", "LD_LIBRARY_PATH", "PATH"):
        val = os.environ.get(var, "")
        if val:
            os.environ[var] = os.pathsep.join(
                p for p in val.split(os.pathsep) if not p.startswith(ament_prefixes)
            )
    os.environ.pop("AMENT_PREFIX_PATH", None)

args = sys.argv[1:]
delay = 0
timeout = None
while args:
    if args[0] == "--delay":
        if len(args) < 2:
            print("Error: --delay requires a value", file=sys.stderr)
            sys.exit(2)
        delay = float(args[1])
        args = args[2:]
    elif args[0] == "--timeout":
        if len(args) < 2:
            print("Error: --timeout requires a value", file=sys.stderr)
            sys.exit(2)
        timeout = float(args[1])
        args = args[2:]
    else:
        break

if len(args) < 2:
    print("Usage: run_times.py [--delay D] [--timeout T] N command...", file=sys.stderr)
    sys.exit(2)

try:
    n = int(args[0])
except ValueError:
    print(f"Error: N must be an integer, got '{args[0]}'", file=sys.stderr)
    sys.exit(2)
if n < 1:
    print(f"Error: N must be >= 1, got {n}", file=sys.stderr)
    sys.exit(2)
cmd_parts = args[1:]
for i in range(1, n + 1):
    if i > 1 and delay > 0:
        time.sleep(delay)
    parts = [part.replace("{i}", str(i)) for part in cmd_parts]
    print(f"Iteration #{i}: {subprocess.list2cmdline(parts)}...")
    if timeout is not None:
        popen_kwargs = {}
        if sys.platform != "win32":
            popen_kwargs["preexec_fn"] = os.setsid
        proc = subprocess.Popen(parts, **popen_kwargs)
        try:
            rc = proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            print(f"Iteration #{i} timed out after {timeout}s, killing...", file=sys.stderr)
            if sys.platform == "win32":
                subprocess.call(
                    ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            else:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except (ProcessLookupError, OSError):
                    pass
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except (ProcessLookupError, OSError):
                        pass
            proc.wait(timeout=10)
            sys.exit(1)
    else:
        rc = subprocess.call(parts)
    if rc != 0:
        sys.exit(rc)
print("All iterations finished successfully!")
