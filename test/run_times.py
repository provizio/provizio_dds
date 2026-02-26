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

# Cross-platform replacement for run_times.sh
# Runs command N times, replacing {i} with iteration number
# Arguments are preserved as separate entries to maintain proper quoting
# when nesting with run_parallel.py
# Also handles ROS environment cleanup (equivalent of unset_ros.sh)
#
# Usage: run_times.py [--retries R] N command...
#   --retries R: retry each failed iteration up to R times (default 0)

import os
import sys
import subprocess
import time

# Unset ROS environment variables to avoid interference (cross-platform equivalent of unset_ros.sh)
ament = os.environ.get("AMENT_PREFIX_PATH", "")
if ament:
    for var in ("PYTHONPATH", "LD_LIBRARY_PATH", "PATH"):
        val = os.environ.get(var, "")
        if val:
            os.environ[var] = os.pathsep.join(
                p for p in val.split(os.pathsep) if not p.startswith(ament)
            )
    os.environ.pop("AMENT_PREFIX_PATH", None)

args = sys.argv[1:]
retries = 0
if args and args[0] == "--retries":
    retries = int(args[1])
    args = args[2:]

n = int(args[0])
cmd_parts = args[1:]
for i in range(1, n + 1):
    parts = [part.replace("{i}", str(i)) for part in cmd_parts]
    for attempt in range(1 + retries):
        if attempt > 0:
            print(f"Iteration #{i}: retry {attempt}/{retries}...")
            time.sleep(2)
        else:
            print(f"Iteration #{i}: {subprocess.list2cmdline(parts)}...")
        rc = subprocess.call(parts)
        if rc == 0:
            break
    if rc != 0:
        sys.exit(rc)
print("All iterations finished successfully!")
