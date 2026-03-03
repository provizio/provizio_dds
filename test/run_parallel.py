#!/usr/bin/env python3

# Copyright 2025 Provizio Ltd.
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

# Cross-platform replacement for run_multiple.sh
# Launches all arguments as parallel subprocesses, waits for all, exits non-zero if any fails
# Also handles ROS environment cleanup
#
# Per-command ROS environment:
#   By default, ROS environment variables are cleaned to avoid conflicts with
#   provizio_dds's bundled Fast-DDS. Commands that need the full ROS environment
#   (e.g. ros2 CLI, ROS Python nodes) should be prefixed with "--ros:" so they
#   inherit the original ROS-sourced environment.

import os
import signal
import sys
import subprocess

ROS_CMD_PREFIX = "--ros:"

# Save the original environment before any modifications
original_env = os.environ.copy()

# Build a clean environment with ROS entries removed (avoids conflicts with
# ROS-bundled Fast-DDS, which is non-backwards-compatible with provizio_dds's version)
ament = os.environ.get("AMENT_PREFIX_PATH", "")
if ament:
    clean_env = os.environ.copy()
    for var in ("PYTHONPATH", "LD_LIBRARY_PATH", "PATH"):
        val = clean_env.get(var, "")
        if val:
            clean_env[var] = os.pathsep.join(
                p for p in val.split(os.pathsep) if not p.startswith(ament)
            )
    clean_env.pop("AMENT_PREFIX_PATH", None)
else:
    clean_env = original_env


def kill_all(procs):
    """Kill all processes that are still alive."""
    for p in procs:
        if p.poll() is None:
            try:
                if sys.platform == "win32":
                    # On Windows, shell=True spawns cmd.exe; killing just the
                    # shell may orphan the child. Use taskkill /T to kill the
                    # entire process tree.
                    subprocess.call(
                        ["taskkill", "/F", "/T", "/PID", str(p.pid)],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                else:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except (ProcessLookupError, OSError):
                pass


# Launch each command with the appropriate environment
procs = []
popen_kwargs = {}
if sys.platform != "win32":
    popen_kwargs["preexec_fn"] = os.setsid  # New process group for clean killpg

for cmd in sys.argv[1:]:
    if cmd.startswith(ROS_CMD_PREFIX):
        procs.append(subprocess.Popen(cmd[len(ROS_CMD_PREFIX):], shell=True, env=original_env, **popen_kwargs))
    else:
        procs.append(subprocess.Popen(cmd, shell=True, env=clean_env, **popen_kwargs))

import time

try:
    # Poll instead of sequential wait: if any process fails, kill the rest
    # immediately rather than blocking on p.wait() for a process that may
    # never exit (e.g. a publisher in an infinite publish loop).
    while True:
        for p in procs:
            ret = p.poll()
            if ret is not None and ret != 0:
                kill_all(procs)
                sys.exit(1)
        if all(p.poll() is not None for p in procs):
            break
        time.sleep(0.1)
except KeyboardInterrupt:
    kill_all(procs)
    sys.exit(1)
