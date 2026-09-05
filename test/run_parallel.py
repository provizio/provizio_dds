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
import shutil
import signal
import sys
import subprocess
import threading
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

# Environment for "--ros:" children: the original ROS-sourced environment,
# minus the source-built Fast-DDS directory test.sh may have prepended to
# LD_LIBRARY_PATH (exported as PROVIZIO_DDS_TEST_PREPENDED_LIB_PATH). That
# prepend makes directly-launched provizio_dds tests resolve the bundled
# Fast-DDS; native ROS nodes must NOT see it — when the ROS-bundled Fast-DDS
# shares the SONAME with the bundled one (e.g. ROS 2 Lyrical's v3.6.1 vs the
# bundled v3.6.2, both libfastdds.so.3.6), it would bind ROS-built code to
# the bundled library: the same patch-release ABI mismatch in reverse.
ros_env = original_env.copy()
_prepended_lib_path = ros_env.pop("PROVIZIO_DDS_TEST_PREPENDED_LIB_PATH", "")
if _prepended_lib_path:
    _ld_library_path = ros_env.get("LD_LIBRARY_PATH", "")
    if _ld_library_path:
        ros_env["LD_LIBRARY_PATH"] = os.pathsep.join(
            p for p in _ld_library_path.split(os.pathsep) if p != _prepended_lib_path
        )

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


def _process_group_pids(pgid):
    """Every pid in ``pgid``, read straight out of /proc.

    The children are launched through a shell, so a child's own pid may be the shell rather
    than the test binary. Walking the process group finds the binary either way, without
    needing pgrep or ps.
    """
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/stat", encoding="utf-8", errors="replace") as handle:
                fields = handle.read().rsplit(") ", 1)[-1].split()
            # /proc/pid/stat after the comm field: state, ppid, pgrp, ...
            if len(fields) > 2 and int(fields[2]) == pgid:
                pids.append(int(entry))
        except (OSError, ValueError):
            continue
    return sorted(pids)


def _read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            return handle.read().strip()
    except OSError:
        return "?"


# Ticks per second for the utime / stime fields of /proc/<pid>/stat. Read once; a host
# that cannot answer disables the CPU column rather than reporting nonsense.
try:
    _CLOCK_TICKS = os.sysconf("SC_CLK_TCK") or 0
except (AttributeError, ValueError, OSError):
    _CLOCK_TICKS = 0


def _thread_cpu_seconds(base):
    """CPU seconds one thread has consumed (user + system), or None if unreadable.

    The comm field of /proc/<pid>/stat is parenthesised and may itself contain spaces, so
    every field is counted from after the closing parenthesis: state first, then utime
    twelfth and stime thirteenth.
    """
    if not _CLOCK_TICKS:
        return None
    raw = _read(base + "/stat")
    fields = raw.rsplit(") ", 1)[-1].split(" ")
    try:
        return (int(fields[11]) + int(fields[12])) / _CLOCK_TICKS
    except (IndexError, ValueError):
        return None


def _sample_thread_cpu(children):
    """{(pid, tid): cpu_seconds} for every thread of every listed process, best-effort."""
    sample = {}
    for _proc, _pgid, pids in children:
        for pid in pids:
            try:
                tids = os.listdir(f"/proc/{pid}/task")
            except OSError:
                continue
            for tid in tids:
                seconds = _thread_cpu_seconds(f"/proc/{pid}/task/{tid}")
                if seconds is not None:
                    sample[(pid, tid)] = seconds
    return sample


def _run_tool(argv, unavailable=None, timeout=5):
    """Run a diagnostic helper, printing its output. Never raises, never blocks for long.

    The timeout is short by default and deliberately so: the callers run with CTest
    already counting down to SIGKILL, and a diagnostic that eats that window costs the very
    report it is trying to produce -- once per stuck child. Never called from the signal
    handler, which may not fork at all (see _sigterm_handler).
    """
    try:
        finished = subprocess.run(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout, check=False
        )
    except (OSError, subprocess.SubprocessError):
        print(f"  ({unavailable or argv[0] + ' unavailable'})", file=sys.stderr)
        return
    text = finished.stdout.decode("utf-8", "replace")
    if finished.returncode != 0 or "Could not attach" in text or "ptrace:" in text:
        print(f"  ({unavailable or argv[0] + ' produced nothing usable'})", file=sys.stderr)
        return
    print(text, file=sys.stderr)


def dump_stuck_processes(procs, allow_subprocess=True, heading=None, cpu_sample_gap=None):
    """Report what every still-running child is doing, before it is killed. Never raises.

    ``allow_subprocess`` gates the helpers that have to be forked (``ps``, ``gdb``): false
    from a signal handler, where forking is what must not happen -- see _sigterm_handler.

    ``heading`` is the one line printed above the report, and every caller passes its own:
    a report that names the wrong moment is worse than one that names none, and the two
    callers here observe very different moments (a deadline a few seconds BEFORE the
    timeout, and the timeout itself). ``cpu_sample_gap``, when set, spends that many
    seconds measuring how much CPU each thread consumes, which is what separates a thread
    that is wedged from one that is merely slow -- see _dump_stuck_processes.

    A thin wrapper around :func:`_dump_stuck_processes`, whose whole body is guarded: it
    runs from a signal handler, where even ``print`` can raise -- CPython refuses a
    reentrant write if the signal landed while the main thread held the stderr buffer lock
    (RuntimeError: reentrant call) -- and an exception escaping here would replace the
    child cleanup that follows it with a traceback.
    """
    try:
        _dump_stuck_processes(procs, allow_subprocess, heading, cpu_sample_gap)
    except BaseException as ex:  # noqa: B036 - a diagnostic must never break the cleanup
        # Reported through os.write rather than print: if print is what failed, using it
        # again to say so would fail the same way.
        try:
            os.write(2, f"(run_parallel: timeout report unavailable: {ex!r})\n".encode())
        except OSError:
            pass


def _dump_stuck_processes(procs, allow_subprocess=True, heading=None, cpu_sample_gap=None):
    """Report what every still-running child is doing, before it is killed.

    A CTest timeout otherwise yields one word -- "Timeout" -- which cannot distinguish a test
    that is genuinely slow from one wedged in a lock, and the kill destroys the only moment
    the answer was available. Each thread's name plus its kernel wait channel is normally
    enough to tell those apart: a teardown deadlock shows every thread parked in a futex,
    whereas a slow-but-healthy run shows threads in poll/epoll doing work.

    Except for the one case a single sample cannot answer: a thread reported RUNNABLE. That
    is one instant of one thread, and it is equally the picture of a process spinning and of
    a process doing ordinary work that happened to be on-CPU when /proc was read. So where
    the caller can afford the wait (cpu_sample_gap -- the deadline path, which runs seconds
    before the timeout rather than during it), each thread's CPU time is measured across a
    short window and reported as a delta: a runnable thread that consumed a window's worth
    of CPU is spinning, one that consumed none was merely sampled at a runnable moment.

    Best-effort by design, and deliberately cheap: it runs with CTest already winding up to
    SIGKILL, so the /proc reads come first and always, and a real backtrace is attempted only
    where forking is allowed at all (allow_subprocess -- not from the signal handler) and gdb
    happens to be installed. Nothing here may raise or block -- a diagnostic that breaks the
    cleanup it is attached to is worse than none.
    """
    if sys.platform == "win32":
        return
    alive = [p for p in procs if p.poll() is None]
    if not alive:
        return
    print(
        heading or "\n=== run_parallel: timed out with process(es) still running; state at kill time ===",
        file=sys.stderr,
    )

    # Resolved once, before anything is printed: the CPU pre-sample below has to read the
    # same set of processes the report then describes, and re-listing /proc per pass would
    # compare two different sets.
    on_linux = sys.platform.startswith("linux")
    children = []
    for p in alive:
        try:
            pgid = os.getpgid(p.pid)
        except OSError:
            pgid = p.pid
        children.append((p, pgid, _process_group_pids(pgid) if on_linux else []))

    before = {}
    if cpu_sample_gap and on_linux:
        before = _sample_thread_cpu(children)
        time.sleep(cpu_sample_gap)
        print(
            f"(cpu= is CPU seconds consumed over the last {cpu_sample_gap:g} s: a runnable "
            f"thread with cpu=0 is waiting, not spinning)",
            file=sys.stderr,
        )

    for p, pgid, pids in children:
        if not on_linux:
            # No /proc to read. ps -M lists a process's threads on macOS, which is at least
            # the thread count and each one's state.
            if allow_subprocess:
                _run_tool(["ps", "-M", str(p.pid)])
            continue
        print(f"--- child pid {p.pid} (process group {pgid}): pids {pids}", file=sys.stderr)
        for pid in pids:
            name = _read(f"/proc/{pid}/comm")
            print(f"  pid {pid} ({name})", file=sys.stderr)
            try:
                tids = sorted(os.listdir(f"/proc/{pid}/task"))
            except OSError:
                continue
            for tid in tids:
                base = f"/proc/{pid}/task/{tid}"
                cpu = ""
                if before:
                    now = _thread_cpu_seconds(base)
                    was = before.get((pid, tid))
                    # A thread that appeared inside the window has no "before" to subtract,
                    # and saying so beats reporting its whole lifetime as a delta.
                    cpu = f" cpu={now - was:.2f}s" if now is not None and was is not None else " cpu=?"
                # wchan is the kernel function the thread is blocked in, readable without
                # privileges; "0" or empty means runnable.
                print(
                    f"    thread {tid} [{_read(base + '/comm')}] "
                    f"state={_read(base + '/stat').rsplit(') ', 1)[-1].split(' ')[0] or '?'} "
                    f"wchan={_read(base + '/wchan') or '-'}{cpu}",
                    file=sys.stderr,
                )
        # Full backtraces when the host allows it. Yama's ptrace_scope=1 restricts attaching
        # to a tracer's own descendants, and gdb is a sibling of the target rather than its
        # ancestor, so this is refused on a hardened host -- reported as one line, because the
        # /proc summary above is the part that always works.
        if allow_subprocess and shutil.which("gdb") and pids:
            _run_tool(
                ["gdb", "-p", str(pids[-1]), "-batch", "-nx", "-ex", "thread apply all bt"],
                unavailable="gdb could not attach (ptrace_scope?); see the /proc summary above",
                # Shorter than the default: attaching to a wedged process is the one
                # diagnostic here that can hang, and the /proc summary above has already
                # captured the part that always works.
                timeout=5,
            )
    print("=== end of run_parallel state report ===\n", file=sys.stderr)
    sys.stderr.flush()


def arm_deadline(procs):
    """Report what the children are doing shortly before CTest's TIMEOUT expires.

    Reports and nothing else: no kill, no exit, no change to what passes. CTest owns the
    verdict, and it has to -- a test whose own internal budget runs close to its TIMEOUT
    (python_request_response_ignore waits up to 25 s for its requests and then sleeps 4 s,
    against a TIMEOUT of 30) is slow, not stuck, and killing it a few seconds early would
    turn a passing run into a failure on exactly the loaded runners this exists to
    diagnose. So the deadline only opens the window: if the children finish afterwards the
    test still passes and nobody reads the report, and if they do not, CTest kills them and
    --output-on-failure shows it.

    The report cannot wait for a signal. CTest kills a timed-out test with SIGKILL, which
    no handler can catch -- verified: the SIGTERM handler installed below never runs on a
    CTest timeout, so for as long as the report depended on it, it could not fire at all.
    provizio_dds_finalize_tests (test/CMakeLists.txt) therefore passes
    PROVIZIO_DDS_TEST_DEADLINE_SEC, a few seconds inside each test's own TIMEOUT; without
    it (a direct run, an older configure) nothing is armed.

    A timer thread rather than SIGALRM: the main thread is in a poll loop and a signal
    would only interrupt its sleep, and this keeps the Windows path identical.

    This is also the only path that can afford to MEASURE anything, which is why it asks for
    a CPU sampling window: it runs seconds ahead of the timeout on an ordinary thread, where
    a short sleep costs nothing but the margin it was given, whereas the signal handler runs
    with the kill already due.
    """
    raw = os.environ.get("PROVIZIO_DDS_TEST_DEADLINE_SEC", "").strip()
    if not raw.isdigit() or int(raw) <= 0:
        return None

    def on_deadline():
        # The heading goes to the report rather than being printed here, so that a child
        # exiting between this check and the report cannot leave a header with nothing under
        # it -- and so that the report names the moment it actually describes. This one is
        # NOT the kill: the children may still finish and the test still pass.
        dump_stuck_processes(
            procs,
            heading=(
                f"\n=== run_parallel: {raw}s elapsed and children are still running; CTest's "
                f"TIMEOUT is close behind, so here is their state while it can still be read ==="
            ),
            # Comfortably inside the margin this deadline is set back by (see
            # PROVIZIO_DDS_TEST_DEADLINE_MARGIN_SEC in test/CMakeLists.txt), and long enough
            # that a spinning thread accumulates a clearly non-zero share of it.
            cpu_sample_gap=1.5,
        )

    timer = threading.Timer(int(raw), on_deadline)
    timer.daemon = True
    timer.start()
    return timer


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

    # How long the report gets before the cleanup takes the window back. Short: CTest is
    # already counting down to SIGKILL, and the children outliving this process is a worse
    # outcome than an incomplete report -- they keep their DDS ports and shared-memory
    # segments and pollute every test that follows.
    _REPORT_ALARM_SEC = 3

    def _sigterm_handler(_sig, _frame):
        """CTest's timeout: say what the children were doing, then kill them.

        Two rules, both learned from what can go wrong in a signal handler. Nothing here
        forks: subprocess.run from a handler can block on the malloc or import lock the
        interrupted main thread was holding, and a stall there would skip the cleanup
        entirely -- so the report is /proc reads only (dump_stuck_processes'
        allow_subprocess=False) and the backtrace is left to the deadline timer, which runs
        on an ordinary thread where forking is safe. And the cleanup is guaranteed by an
        alarm rather than by trusting the report to return: SIGALRM kills the children and
        exits immediately, from os._exit, because a SystemExit raised into a wedged C call
        may never unwind.
        """
        signal.signal(signal.SIGALRM, lambda _s, _f: (kill_all(procs), os._exit(1)))
        signal.alarm(_REPORT_ALARM_SEC)
        try:
            dump_stuck_processes(
                procs,
                allow_subprocess=False,
                heading=(
                    "\n=== run_parallel: timed out with process(es) still running; "
                    "state at kill time ==="
                ),
                # No sampling window: the kill is already due, and a report that arrives
                # after the children have been SIGKILLed describes nothing.
            )
        finally:
            signal.alarm(0)
        kill_all(procs)
        sys.exit(1)

    # Handle SIGTERM (sent by CTest on timeout) to clean up child processes
    signal.signal(signal.SIGTERM, _sigterm_handler)

# shell=True is required: commands come from CTest add_test() which passes them as
# single strings with arguments (e.g. "python test.py arg1"). This script is only
# invoked from CTest, never from untrusted user input.
for cmd in sys.argv[1:]:
    if cmd.startswith(ROS_CMD_PREFIX):
        procs.append(subprocess.Popen(cmd[len(ROS_CMD_PREFIX):], shell=True, env=ros_env, **popen_kwargs))
    else:
        procs.append(subprocess.Popen(cmd, shell=True, env=clean_env, **popen_kwargs))

arm_deadline(procs)

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
