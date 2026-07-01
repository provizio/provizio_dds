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
import time

ROS_CMD_PREFIX = "--ros:"

# Real-time, kill-safe logging (mirrors run_times.py): export PYTHONUNBUFFERED *before*
# the environment snapshots below so it is captured into both original_env and clean_env
# and thus inherited by every launched child -- their output then survives even a SIGTERM/
# SIGKILL. Also line-buffer our own streams so this wrapper's messages interleave with the
# children's in real time rather than being flushed only at exit.
os.environ["PYTHONUNBUFFERED"] = "1"
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass

# Save the original environment before any modifications
original_env = os.environ.copy()

# Build a clean environment with ROS entries removed (avoids conflicts with
# ROS-bundled Fast-DDS, which is non-backwards-compatible with provizio_dds's version)
ament = os.environ.get("AMENT_PREFIX_PATH", "")
if ament:
    # AMENT_PREFIX_PATH can be a pathsep-separated list of prefixes
    ament_prefixes = tuple(ament.split(os.pathsep))
    clean_env = os.environ.copy()
    for var in ("PYTHONPATH", "LD_LIBRARY_PATH", "PATH"):
        val = clean_env.get(var, "")
        if val:
            clean_env[var] = os.pathsep.join(
                p for p in val.split(os.pathsep) if not p.startswith(ament_prefixes)
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


# Platform-specific child process cleanup setup
procs = []
popen_kwargs = {}
if sys.platform == "win32":
    # Windows child-process cleanup has three layers:
    #
    # 1. Job Object (JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE): When CTest times out
    #    and calls TerminateProcess on this runner, the Job handle is closed and
    #    all child processes are killed automatically. This is the only way to
    #    handle TerminateProcess, which cannot be caught by any signal handler.
    #
    # 2. Console control handler (SetConsoleCtrlHandler): Catches Ctrl+C,
    #    Ctrl+Break, console close, logoff, and shutdown events, calling
    #    kill_all(procs) before the process exits.
    #
    # 3. SIGBREAK signal: Fallback for Ctrl+Break if the ctypes setup fails.
    try:
        import ctypes
        from ctypes import wintypes

        _k32 = ctypes.windll.kernel32

        # Define Win32 API signatures to avoid HANDLE truncation on 64-bit
        _k32.CreateJobObjectW.restype = wintypes.HANDLE
        _k32.CreateJobObjectW.argtypes = [wintypes.LPVOID, wintypes.LPCWSTR]
        _k32.SetInformationJobObject.restype = wintypes.BOOL
        _k32.SetInformationJobObject.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPVOID, wintypes.DWORD]
        _k32.AssignProcessToJobObject.restype = wintypes.BOOL
        _k32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
        _k32.GetCurrentProcess.restype = wintypes.HANDLE
        _k32.GetCurrentProcess.argtypes = []
        _HANDLER_ROUTINE = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.DWORD)
        _k32.SetConsoleCtrlHandler.restype = wintypes.BOOL
        _k32.SetConsoleCtrlHandler.argtypes = [_HANDLER_ROUTINE, wintypes.BOOL]

        # Layer 1: Job Object
        _job = _k32.CreateJobObjectW(None, None)
        if _job:
            class _BASIC(ctypes.Structure):
                _fields_ = [("PerProcessUserTimeLimit", wintypes.LARGE_INTEGER),
                            ("PerJobUserTimeLimit", wintypes.LARGE_INTEGER),
                            ("LimitFlags", wintypes.DWORD),
                            ("MinimumWorkingSetSize", ctypes.c_size_t),
                            ("MaximumWorkingSetSize", ctypes.c_size_t),
                            ("ActiveProcessLimit", wintypes.DWORD),
                            ("Affinity", ctypes.c_size_t),
                            ("PriorityClass", wintypes.DWORD),
                            ("SchedulingClass", wintypes.DWORD)]

            class _IO(ctypes.Structure):
                _fields_ = [("ReadOperationCount", ctypes.c_uint64),
                            ("WriteOperationCount", ctypes.c_uint64),
                            ("OtherOperationCount", ctypes.c_uint64),
                            ("ReadTransferCount", ctypes.c_uint64),
                            ("WriteTransferCount", ctypes.c_uint64),
                            ("OtherTransferCount", ctypes.c_uint64)]

            class _EXT(ctypes.Structure):
                _fields_ = [("BasicLimitInformation", _BASIC),
                            ("IoInfo", _IO),
                            ("ProcessMemoryLimit", ctypes.c_size_t),
                            ("JobMemoryLimit", ctypes.c_size_t),
                            ("PeakProcessMemoryUsed", ctypes.c_size_t),
                            ("PeakJobMemoryUsed", ctypes.c_size_t)]

            ext = _EXT()
            ext.BasicLimitInformation.LimitFlags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            _k32.SetInformationJobObject(_job, 9, ctypes.byref(ext), ctypes.sizeof(ext))
            _k32.AssignProcessToJobObject(_job, _k32.GetCurrentProcess())

        # Layer 2: Console control handler for graceful cleanup on Ctrl+C,
        # Ctrl+Break, console close, logoff, and shutdown events.
        def _ctrl_handler(event):
            kill_all(procs)
            return False  # Let the default handler terminate the process

        # prevent garbage collection of the callback
        _ctrl_handler_cb = _HANDLER_ROUTINE(_ctrl_handler)
        _k32.SetConsoleCtrlHandler(_ctrl_handler_cb, True)
    except Exception as e:
        print(f"Warning: Win32 Job Object setup failed ({e}), falling back to taskkill", file=sys.stderr)

    # Layer 3: SIGBREAK as a simpler fallback for Ctrl+Break. Works even if
    # the ctypes-based SetConsoleCtrlHandler setup above failed.
    signal.signal(signal.SIGBREAK, lambda _sig, _frame: (kill_all(procs), sys.exit(1)))
else:
    popen_kwargs["preexec_fn"] = os.setsid  # New process group for clean killpg
    # Handle SIGTERM (sent by CTest on timeout) to clean up child processes
    signal.signal(signal.SIGTERM, lambda _sig, _frame: (kill_all(procs), sys.exit(1)))

# shell=True is required: commands come from CTest add_test() which passes them as
# single strings with arguments (e.g. "python test.py arg1"). This script is only
# invoked from CTest, never from untrusted user input.
for cmd in sys.argv[1:]:
    if cmd.startswith(ROS_CMD_PREFIX):
        procs.append(subprocess.Popen(cmd[len(ROS_CMD_PREFIX):], shell=True, env=original_env, **popen_kwargs))
    else:
        procs.append(subprocess.Popen(cmd, shell=True, env=clean_env, **popen_kwargs))

try:
    # Poll instead of sequential wait: if any process fails, kill the rest
    # immediately rather than blocking on p.wait() for a process that may
    # never exit (e.g. a publisher in an infinite publish loop).
    while True:
        for idx, p in enumerate(procs):
            ret = p.poll()
            if ret is not None and ret != 0:
                print(f"Command failed (exit code {ret}): {sys.argv[1 + idx]}", file=sys.stderr)
                kill_all(procs)
                sys.exit(ret)
        if all(p.poll() is not None for p in procs):
            break
        time.sleep(0.1)
except KeyboardInterrupt:
    kill_all(procs)
    sys.exit(1)
