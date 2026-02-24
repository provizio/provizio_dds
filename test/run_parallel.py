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
# Also handles ROS environment cleanup (equivalent of unset_ros.sh)

import os
import sys
import subprocess

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

procs = [subprocess.Popen(cmd, shell=True) for cmd in sys.argv[1:]]
codes = [p.wait() for p in procs]
sys.exit(0 if all(c == 0 for c in codes) else 1)
