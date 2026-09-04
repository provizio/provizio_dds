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

"""External stall sampler for the ctest suite.

CTest kills a timed-out test with SIGKILL and reports one word - "Timeout". Whatever the
process was doing, and whatever it had already written to a buffered stdout, is gone. This
watchdog runs alongside ctest, learns every test's own TIMEOUT from
``ctest --show-only=json-v1``, and, a few seconds BEFORE a running test reaches that
TIMEOUT, captures what it is doing: per-thread states, full user-space backtraces
(``sample`` / ``lldb`` on macOS, ``gdb`` / ``/proc`` on Linux) and its open sockets.

It changes no verdict: it only reads. A test that finishes after being sampled still
passes, and its report is simply never read.

Usage:
    ci_stall_watchdog.py --build-dir DIR [--out DIR] [--margin SEC] [--poll SEC]
"""

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time

# How long any diagnostic subprocess may run before it is abandoned. Attaching a debugger
# to a wedged process is the one step here that can itself hang, and a watchdog that hangs
# reports nothing.
TOOL_TIMEOUT_SEC = 25

# A process is sampled again this long after its first sample, so a stack that is moving
# (slow) can be told from one that is not (wedged).
RESAMPLE_AFTER_SEC = 6.0


def _now():
    return time.strftime("%H:%M:%S", time.localtime()) + f".{int((time.time() % 1) * 1000):03d}"


def log(message, out_file):
    line = f"[stall-watchdog {_now()}] {message}"
    print(line, file=sys.stderr, flush=True)
    if out_file is not None:
        out_file.write(line + "\n")
        out_file.flush()


def run_tool(argv, out_file, heading=None, use_sudo=False):
    """Run one diagnostic command, echoing its output. Never raises."""
    if use_sudo:
        argv = ["sudo", "-n"] + argv
    if shutil.which(argv[0]) is None:
        log(f"({argv[0]} not installed)", out_file)
        return False
    if heading:
        log(heading, out_file)
    try:
        completed = subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=TOOL_TIMEOUT_SEC,
            check=False,
        )
    except subprocess.TimeoutExpired:
        log(f"({' '.join(argv)} timed out after {TOOL_TIMEOUT_SEC}s)", out_file)
        return False
    except OSError as ex:
        log(f"({' '.join(argv)} failed: {ex!r})", out_file)
        return False
    text = completed.stdout.decode("utf-8", "replace")
    sys.stderr.write(text)
    sys.stderr.flush()
    if out_file is not None:
        out_file.write(text)
        out_file.flush()
    return completed.returncode == 0 and bool(text.strip())


def read_ctest_tests(build_dir):
    """Map each registered test's full command line to its TIMEOUT, via ctest's JSON dump."""
    try:
        completed = subprocess.run(
            ["ctest", "--show-only=json-v1"],
            cwd=build_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=120,
            check=False,
        )
        data = json.loads(completed.stdout.decode("utf-8", "replace"))
    except (OSError, ValueError, subprocess.TimeoutExpired) as ex:
        print(f"[stall-watchdog] could not read ctest test list: {ex!r}", file=sys.stderr, flush=True)
        return [], {}

    tests = []
    # Executable path -> the largest TIMEOUT of any test that mentions it. A test launched
    # through run_parallel.py names its real workers as arguments, and those children are
    # the processes worth sampling, so every path-looking token counts. The LARGEST timeout
    # is deliberate where one path serves several tests (the Python interpreter serves all
    # of them): sampling late costs one missed report, sampling a healthy long-running test
    # every poll costs the machine the stalled one is on.
    by_executable = {}
    for test in data.get("tests", []):
        command = test.get("command") or []
        if not command:
            continue
        timeout = None
        for prop in test.get("properties", []) or []:
            if prop.get("name") == "TIMEOUT":
                try:
                    timeout = float(prop.get("value"))
                except (TypeError, ValueError):
                    timeout = None
        if timeout is None:
            continue
        name = test.get("name", "?")
        tests.append({"name": name, "cmdline": " ".join(command), "timeout": timeout})
        for token in command:
            # ctest renders a grouped argument with its quotes inside the argv element.
            path = token.strip('"').split(" ")[0]
            if "/" not in path:
                continue
            previous = by_executable.get(path)
            if previous is None or timeout > previous["timeout"]:
                by_executable[path] = {"name": f"{name} (or a sibling using {os.path.basename(path)})",
                                       "cmdline": path, "timeout": timeout}
    # Longest command line first: a run_parallel.py test and a bare binary can share a
    # prefix, and the more specific match is the right one.
    tests.sort(key=lambda t: len(t["cmdline"]), reverse=True)
    return tests, by_executable


_ETIME_RE = re.compile(r"^(?:(?:(\d+)-)?(\d+):)?(\d+):(\d+)$")


def parse_etime(text):
    """Parse ps(1) elapsed time ([[dd-]hh:]mm:ss) into seconds. None when unparseable."""
    match = _ETIME_RE.match(text.strip())
    if not match:
        return None
    days, hours, minutes, seconds = match.groups()
    return (
        int(days or 0) * 86400 + int(hours or 0) * 3600 + int(minutes) * 60 + int(seconds)
    )


def list_processes():
    """[(pid, elapsed_seconds, command_line)] for every process this user can see."""
    try:
        completed = subprocess.run(
            ["ps", "-axww", "-o", "pid=,etime=,command="],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=20,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    processes = []
    for line in completed.stdout.decode("utf-8", "replace").splitlines():
        parts = line.split(None, 2)
        if len(parts) < 3:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        elapsed = parse_etime(parts[1])
        if elapsed is None:
            continue
        processes.append((pid, elapsed, parts[2]))
    return processes


def match_test(cmdline, tests, by_executable):
    """The registered test this running process belongs to, if any.

    Exact command lines first (a directly-launched C++ test binary matches verbatim), then
    the executable alone -- which is how a worker launched by run_parallel.py, whose own
    argv appears only as an argument of the ctest command, is recognised.
    """
    for test in tests:
        if cmdline.startswith(test["cmdline"]):
            return test
    executable = cmdline.split(" ", 1)[0]
    return by_executable.get(executable)


def _emit_file(path, out_file, heading):
    """Echo a diagnostic tool's output file, then remove it. Never raises."""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            text = handle.read()
    except OSError as ex:
        log(f"({heading}: no output file: {ex!r})", out_file)
        return False
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass
    if not text.strip():
        return False
    log(heading, out_file)
    sys.stderr.write(text)
    sys.stderr.flush()
    if out_file is not None:
        out_file.write(text)
        out_file.flush()
    return True


def _run_sample(pid, out_file, tag, scratch, use_sudo):
    """One `sample` run. It writes its report to a file, so ask for one we can echo."""
    path = os.path.join(scratch, f"sample.{pid}.{tag}.{'sudo' if use_sudo else 'user'}.txt")
    argv = ["sample", str(pid), "2", "-mayDie", "-fullPaths", "-file", path]
    if use_sudo:
        argv = ["sudo", "-n"] + argv
    try:
        subprocess.run(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=TOOL_TIMEOUT_SEC, check=False
        )
    except (OSError, subprocess.TimeoutExpired) as ex:
        log(f"(sample {pid} {'via sudo ' if use_sudo else ''}failed: {ex!r})", out_file)
        return False
    return _emit_file(path, out_file, f"--- sample {pid} ({tag}{', sudo' if use_sudo else ''})")


def sample_macos(pid, out_file, tag, scratch):
    """Full user-space backtraces of every thread of `pid`, macOS.

    `sample` needs task_for_pid, which an unprivileged same-user attach gets only where the
    host allows it, so the unprivileged attempt comes first and sudo is the fallback; lldb
    is the last resort because attaching a debugger is the step most likely to hang.
    """
    run_tool(["ps", "-M", str(pid)], out_file, heading=f"--- ps -M {pid} ({tag})")
    got = _run_sample(pid, out_file, tag, scratch, use_sudo=False)
    if not got:
        got = _run_sample(pid, out_file, tag, scratch, use_sudo=True)
    if not got:
        run_tool(
            [
                "lldb",
                "-p",
                str(pid),
                "--batch",
                "-o",
                "thread backtrace all",
                "-o",
                "process detach",
                "-o",
                "quit",
            ],
            out_file,
            heading=f"--- sudo lldb {pid} ({tag})",
            use_sudo=True,
        )
    # spindump additionally reports each thread's KERNEL wait reason, which is what
    # separates "blocked on a mutex" from "blocked in kevent waiting for a packet".
    spindump_path = os.path.join(scratch, f"spindump.{pid}.{tag}.txt")
    try:
        subprocess.run(
            ["sudo", "-n", "spindump", str(pid), "2", "-noProcessingWhileSampling", "-o", spindump_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=TOOL_TIMEOUT_SEC,
            check=False,
        )
        _emit_file(spindump_path, out_file, f"--- spindump {pid} ({tag})")
    except (OSError, subprocess.TimeoutExpired):
        pass
    run_tool(["lsof", "-p", str(pid), "-nP"], out_file, heading=f"--- lsof {pid} ({tag})")


def sample_linux(pid, out_file, tag, scratch):  # noqa: ARG001 - signature shared with macOS
    """Per-thread kernel wait channels plus, where allowed, full backtraces. Linux."""
    task_dir = f"/proc/{pid}/task"
    heading = f"--- /proc thread states {pid} ({tag})"
    log(heading, out_file)
    try:
        for tid in sorted(os.listdir(task_dir)):
            base = f"{task_dir}/{tid}"
            fields = {}
            for name in ("comm", "wchan", "stat"):
                try:
                    with open(f"{base}/{name}", "rb") as handle:
                        fields[name] = handle.read().decode("utf-8", "replace").strip()
                except OSError:
                    fields[name] = "?"
            state = fields["stat"].rsplit(") ", 1)[-1].split(" ")[0] if fields["stat"] != "?" else "?"
            log(f"  thread {tid} [{fields['comm']}] state={state} wchan={fields['wchan'] or '-'}", out_file)
    except OSError as ex:
        log(f"  (unreadable: {ex!r})", out_file)
    run_tool(
        ["gdb", "-p", str(pid), "-batch", "-nx", "-ex", "thread apply all bt"],
        out_file,
        heading=f"--- gdb {pid} ({tag})",
    )
    run_tool(["lsof", "-p", str(pid), "-nP"], out_file, heading=f"--- lsof {pid} ({tag})")


def capture(pid, cmdline, elapsed, test, out_file, tag, scratch):
    log(
        f"STALL {tag}: pid {pid} has run {elapsed:.0f}s of its "
        f"{test['timeout']:.0f}s TIMEOUT -- test '{test['name']}'",
        out_file,
    )
    log(f"  cmdline: {cmdline}", out_file)
    if sys.platform == "darwin":
        sample_macos(pid, out_file, tag, scratch)
    elif sys.platform.startswith("linux"):
        sample_linux(pid, out_file, tag, scratch)
    log(f"END STALL {tag}: pid {pid}", out_file)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True)
    parser.add_argument("--out", default=None, help="directory for the report file")
    parser.add_argument(
        "--margin",
        type=float,
        default=8.0,
        help="seconds before a test's TIMEOUT at which to capture its state",
    )
    parser.add_argument("--poll", type=float, default=1.0)
    args = parser.parse_args()

    scratch = args.out or os.environ.get("TMPDIR", "/tmp")
    out_file = None
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        out_file = open(os.path.join(args.out, f"stall_report.{os.getpid()}.txt"), "w", encoding="utf-8")

    tests, by_executable = read_ctest_tests(args.build_dir)
    log(
        f"armed for {len(tests)} tests with a TIMEOUT and {len(by_executable)} executables "
        f"(margin {args.margin:g}s)",
        out_file,
    )
    if not tests:
        return 0

    # pid -> (first_capture_monotonic, captures_done)
    captured = {}
    stop = {"requested": False}

    def _stop(_signum, _frame):
        stop["requested"] = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    while not stop["requested"]:
        for pid, elapsed, cmdline in list_processes():
            if pid == os.getpid():
                continue
            state = captured.get(pid)
            if state is not None and state[1] >= 2:
                continue
            test = match_test(cmdline, tests, by_executable)
            if test is None:
                continue
            if elapsed < test["timeout"] - args.margin:
                continue
            now = time.monotonic()
            if state is None:
                captured[pid] = (now, 1)
                capture(pid, cmdline, elapsed, test, out_file, "first", scratch)
            elif now - state[0] >= RESAMPLE_AFTER_SEC:
                captured[pid] = (state[0], 2)
                capture(pid, cmdline, elapsed, test, out_file, "second", scratch)
        # Drop bookkeeping for pids that are gone, so a recycled pid is not silently skipped.
        if len(captured) > 256:
            live = {pid for pid, _, _ in list_processes()}
            captured = {pid: state for pid, state in captured.items() if pid in live}
        time.sleep(args.poll)

    log("stopping", out_file)
    if out_file is not None:
        out_file.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
