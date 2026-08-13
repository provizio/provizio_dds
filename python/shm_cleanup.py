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

"""
Reclaims the shared-memory files of participants that died without destroying
themselves. The Python counterpart of ``src/shm_cleanup.cpp`` — the two layers
share no code (the Python bindings sit on eProsima's SWIG module and never enter
the C++ ``provizio::dds`` core), so both implement it and must stay in sync.

Fast-DDS never garbage-collects those files: every unclean process death
(SIGKILL, a bare ``exit()``, an uncaught exception) leaks that participant's data
segment (tens of MiB at the default transport configuration) plus its lock and
port files, forever. A service that is restarted in a loop therefore fills the
shared-memory filesystem, and once it is full EVERY new participant on the host
silently loses shared-memory transport and falls back to UDP. Upstream's answer
is a manual ``fastdds shm clean`` CLI run; this is the same algorithm, run
automatically.

Safety rests entirely on the lock file Fast-DDS keeps beside each object and
keeps ``flock()``ed for its owner's whole lifetime: the kernel releases flocks on
process death (SIGKILL included), so a lock file that can be locked provably has
no live owner. No PID scanning, no root, no heuristics — and an age guard on top,
so a participant caught mid-creation (its object created, its lock not yet taken)
is never mistaken for a corpse.
"""

import os
import re
import stat
import sys
import threading
import time
from typing import Optional

if __package__ or "." in __name__:
    from . import network_recovery as _network_recovery
else:
    import network_recovery as _network_recovery

if os.name == "posix":
    import fcntl


# Kill switch, default on. An unrecognised value keeps the sweep enabled (and warns): a
# typo must not silently reintroduce the leak this exists to stop. Same grammar as
# PROVIZIO_DDS_NETWORK_RECOVERY. Mirrors src/shm_cleanup.cpp.
_ENABLED_ENV = "PROVIZIO_DDS_SHM_CLEANUP"

# How old (seconds since its last modification) a lock file must be before it may be
# reclaimed, even when nothing holds its lock.
_MIN_AGE_ENV = "PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC"

# A participant creating a segment writes the object first and takes its lock a few
# microseconds later; a sweep landing in between would find an unlocked lock file with a very
# much alive owner, delete its segment, and leave it unreachable over shared memory for the
# rest of its life without either side noticing. Fast-DDS re-creates a lock file that
# disappeared under it (its RobustExclusiveLock retries on ENOENT precisely because its own
# clean-up CLI can do this), but nothing re-creates the DATA SEGMENT, so the window is real.
#
# Five seconds is ~three orders of magnitude wider than that window even when a large
# segment's creation is descheduled on a loaded target, and short enough that a service
# exiting and being restarted every 25-65 s (the case this was written for) has its corpse
# reclaimed by its very next incarnation — one dead generation at a time rather than the two
# or three a minute-long guard would leave coexisting. 0 disables the guard entirely, which
# re-opens the window above.
_DEFAULT_MIN_AGE_SEC = 5

# Upper bound on the age guard. A day is far past any sane value, and clamping keeps the C++
# side's arithmetic honest (time_t is 32-bit signed on some targets, where a multi-billion-
# second guard would wrap negative and turn "too young to judge" into "reclaim everything"),
# so both languages reject the same values.
_MAX_MIN_AGE_SEC = 24 * 60 * 60

# Minimum spacing between sweeps. Only the pressure-triggered path can repeat, and a host
# that is genuinely out of shared memory will not become reclaimable again within seconds,
# so this only caps the cost of a process that creates participants in a loop.
_MIN_SWEEP_INTERVAL_SEC = 30

# Free space below which the shared-memory filesystem cannot fit a handful more segments
# of the size a participant allocates at the default transport configuration (~33.5 MiB
# each). Mirrors src/domain_participant.cpp.
_BYTES_PER_MEBIBYTE = 1024 * 1024
_LOW_WATER_BYTES = 256 * _BYTES_PER_MEBIBYTE

# The domain-name prefixes Fast-DDS gives its shared-memory objects: "fastdds" since 3.x
# and "fastrtps" in 2.x. Both leak identically and both are still found on hosts running a
# mixed fleet (a 2.x-based application, or provizio_dds 1.x), and the lock files work the
# same way in both, so both are swept.
_PREFIXES = ("fastdds_", "fastrtps_")

# "_el" (exclusive lock, one owner) / "_sl" (shared lock, several attachers).
_LOCK_SUFFIXES = ("_el", "_sl")
_LOCK_SUFFIX_LENGTH = 3
# Segment names are a 16-character random id; port names carry a port number, at most 5
# digits.
_SEGMENT_ID_LENGTH = 16
_MAX_PORT_DIGITS = 5

_SEGMENT = "segment"
_PORT = "port"

# Sweep bookkeeping shared by both entry points, so a pressure-triggered sweep is also
# rate-limited against the once-per-process one.
_sweep_lock = threading.Lock()
# Whether the once-per-process sweep has run. Separate from the rate-limit bookkeeping
# below: a pressure-triggered sweep must not disarm the one that runs before the first
# participant, which is what the C++ side's dedicated std::once_flag guarantees.
_swept_once = False
_has_swept = False
_last_sweep = 0.0


class ShmCleanupStats:
    """What one sweep reclaimed."""

    # `size_bytes` rather than the C++ side's `bytes`, which is a builtin type name here.
    def __init__(self, segments: int = 0, ports: int = 0, size_bytes: int = 0):
        #: Data segments removed (each with its lock file).
        self.segments = segments
        #: Ports removed (each with its lock file and named semaphore).
        self.ports = ports
        #: Total size of every removed file.
        self.size_bytes = size_bytes

    def anything_reclaimed(self) -> bool:
        """Whether the sweep removed anything at all."""
        return self.segments != 0 or self.ports != 0

    def __repr__(self) -> str:
        return (
            f"ShmCleanupStats(segments={self.segments}, ports={self.ports}, "
            f"size_bytes={self.size_bytes})"
        )


def shared_memory_dir():
    """The directory Boost.Interprocess — and therefore Fast-DDS — keeps its
    shared-memory objects in, or ``None`` on platforms this sweep does not support.

    Mirrors eProsima's own ``fastdds shm clean`` directory resolution. Windows uses
    neither these paths nor flock semantics, and is also the one platform where the
    Python bindings disable shared memory outright, so there is nothing of ours to
    reclaim there."""

    if os.name != "posix":
        return None
    if sys.platform == "darwin":
        return "/private/tmp/boost_interprocess"
    return "/dev/shm"


def _parse_u32(raw: str) -> int:
    """``raw`` as a uint32 (zero accepted), else ``-1``.

    Mirrors the C++ ``try_parse_u32`` (src/detail/env_utils.h): leading whitespace and a
    single leading '+', then ASCII digits to end-of-string, so both languages treat the
    same env values as valid — Python's permissive ``int()`` would additionally accept
    underscores, trailing whitespace and other input the C++ side rejects."""

    # re.ASCII keeps \s to ASCII whitespace — C++ isspace on raw bytes rejects e.g. a
    # leading NBSP, and Python's int() would otherwise accept it.
    if not re.fullmatch(r"\s*\+?[0-9]+", raw, re.ASCII):
        return -1
    try:
        value = int(raw)
    except ValueError:
        # int() refuses digit strings beyond its conversion-length limit (4300 digits by
        # default on Python 3.11+); such input is invalid here anyway, never an error.
        return -1
    return value if value <= 0xFFFFFFFF else -1


def _cleanup_enabled() -> bool:
    """Whether dead-owner cleanup is enabled (``PROVIZIO_DDS_SHM_CLEANUP``, default on)."""

    raw = os.environ.get(_ENABLED_ENV)
    if not raw:
        return True

    value = raw.lower()
    if value in ("off", "0", "false", "no"):
        return False
    if value in ("on", "1", "true", "yes"):
        return True

    _network_recovery._emit_log(
        _network_recovery.LogLevel.WARNING,
        f"{_ENABLED_ENV}='{_network_recovery._sanitise_env_value_for_log(raw)}' is not "
        f"recognised (use on/off); dead-owner shared-memory cleanup stays enabled",
    )
    return True


def _min_age_seconds() -> int:
    """The age guard in seconds (``PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC``, default 5)."""

    raw = os.environ.get(_MIN_AGE_ENV)
    if not raw:
        return _DEFAULT_MIN_AGE_SEC

    value = _parse_u32(raw)
    if value >= 0:
        if value > _MAX_MIN_AGE_SEC:
            _network_recovery._emit_log(
                _network_recovery.LogLevel.WARNING,
                f"{_MIN_AGE_ENV}={value} exceeds the maximum of {_MAX_MIN_AGE_SEC} seconds; "
                f"clamping to it",
            )
            return _MAX_MIN_AGE_SEC
        return value

    _network_recovery._emit_log(
        _network_recovery.LogLevel.WARNING,
        f"ignoring invalid {_MIN_AGE_ENV}="
        f"'{_network_recovery._sanitise_env_value_for_log(raw)}'; "
        f"using default {_DEFAULT_MIN_AGE_SEC}",
    )
    return _DEFAULT_MIN_AGE_SEC


def _classify(name: str):
    """Classify a directory entry by name alone: ``_SEGMENT``, ``_PORT`` or ``None``.

    Fully anchored at both ends: ``^<prefix>[0-9a-z]{16}_el$`` for a segment and
    ``^<prefix>port[0-9]{1,5}(_el|_sl)$`` for a port. Anything else — including the
    ``fast_datasharing_*`` segments, whose liveness has entirely different semantics (a live
    reader may map a dead writer's segment) — is ``None`` and is left alone.

    Narrower than eProsima's ``fastdds shm clean``, which accepts ``_sl`` for segments too.
    Fast-DDS only ever gives a SEGMENT an exclusive lock (its SharedMemManager builds
    ``segment_name + "_el"``, never ``"_sl"``; only ports take a shared one), so accepting
    ``_sl`` there reclaims no real corpse and leaves a name that is free for the taking while
    the segment is alive — exactly what someone would need to aim the sweep at a live
    segment. Both suffixes stay valid for ports, where Fast-DDS uses both."""

    if len(name) <= _LOCK_SUFFIX_LENGTH or not name.endswith(_LOCK_SUFFIXES):
        return None

    exclusive_lock = name.endswith("_el")
    body = name[:-_LOCK_SUFFIX_LENGTH]
    for prefix in _PREFIXES:
        if not body.startswith(prefix):
            continue

        tail = body[len(prefix):]
        if exclusive_lock and len(tail) == _SEGMENT_ID_LENGTH and all(
            "0" <= character <= "9" or "a" <= character <= "z" for character in tail
        ):
            return _SEGMENT
        if tail.startswith(_PORT) and len(tail) > len(_PORT):
            digits = tail[len(_PORT):]
            if len(digits) <= _MAX_PORT_DIGITS and all(
                "0" <= character <= "9" for character in digits
            ):
                return _PORT

    return None


def _remove_companion_file(dir_fd: int, name: str, owner: int):
    """Unlink one of a dead owner's companion files — the object a lock file names, or a
    port's named semaphore — and return ``(removed, size)``.

    Removal and size are reported separately because Fast-DDS's files include zero-byte
    ones, so a size of 0 says nothing about whether anything went.

    ``owner`` is the uid of the lock file whose flock proved the owner dead, and the file
    must belong to the same user to be removed. That check is what stops the whole sweep
    from being steered by whoever wrote the lock file's NAME: only the lock file's own
    liveness is proven by the flock, and a name like ``<live segment id>_sl`` is one
    Fast-DDS never creates for a segment (its SharedMemManager uses ``_el`` exclusively), so
    anyone able to write to this world-writable directory could otherwise plant one and have
    a sweep delete a live participant's segment out from under it. A regular file owned by
    the same user as its lock file is the only thing Fast-DDS ever creates here.

    Every failure is a silent skip: ENOENT means a concurrent sweeper (in any process) got
    there first, and EACCES / EPERM means the sticky bit refused — both are normal, neither
    is worth a log line."""

    try:
        info = os.lstat(name, dir_fd=dir_fd)
    except OSError:
        return False, 0
    if not stat.S_ISREG(info.st_mode) or info.st_uid != owner:
        return False, 0
    try:
        os.unlink(name, dir_fd=dir_fd)
    except OSError:
        return False, 0
    return True, info.st_size


def _is_lock_file(info) -> bool:
    """Whether the file this ``os.stat_result`` describes is a Fast-DDS lock file rather
    than something that merely took its name.

    Fast-DDS creates them empty and never writes to them (its RobustExclusiveLock /
    RobustSharedLock only open and flock), so a non-empty one was written by something else;
    and a link count above one means the name was hardlinked onto another file — the way an
    attacker would make a lock file whose ownership matches a victim's object. Taken from a
    stat of the DESCRIPTOR, so it describes the file that was actually locked rather than
    whatever the name resolves to now."""

    return stat.S_ISREG(info.st_mode) and info.st_size == 0 and info.st_nlink == 1


def sweep_dead_shared_memory(directory: Optional[str] = None) -> ShmCleanupStats:
    """Remove every Fast-DDS shared-memory segment and port in the platform's shared-memory
    directory whose owner process is gone, and which is older than
    ``PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC``.

    Files whose owner is alive, files that are not what Fast-DDS creates, data-sharing
    segments and anything else in the directory are left untouched. Does nothing when
    ``PROVIZIO_DDS_SHM_CLEANUP`` is off or on platforms without a POSIX shared-memory
    directory (Windows). Safe to run concurrently with itself in any number of processes,
    and with participants starting up. Logs nothing — the callers below own that.

    :param directory: sweep this directory instead of the platform's. Internal, for the test
        suite: it lets a case lay its fixtures out in a directory of its own, where no other
        process on the host can reclaim them and nothing of the host's can be counted as the
        case's. Nothing else has a reason to pass it.
    :return: a :class:`ShmCleanupStats` describing what was reclaimed
    """

    stats = ShmCleanupStats()
    if directory is None:
        directory = shared_memory_dir()
    if directory is None or os.name != "posix" or not _cleanup_enabled():
        return stats

    # O_DIRECTORY | O_NOFOLLOW, and every lookup below made RELATIVE to the resulting
    # descriptor: the sweep is pinned to the one directory it opened for its whole run, so
    # nothing it unlinks can be redirected by swapping that directory for a symlink
    # underneath it. (/dev/shm is a root-owned mount point, but the macOS location lives under
    # a world-writable /private/tmp.) It also removes path assembly from the delete path
    # entirely — the only names used are the ones the directory scan returned.
    try:
        dir_fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    except OSError:
        # No shared-memory directory, or no permission to open it: nothing this process could
        # clean up even in principle.
        return stats

    min_age = _min_age_seconds()
    now = time.time()
    try:
        # scandir rather than listdir: it yields entries lazily, so a directory someone has
        # filled with millions of names costs no proportional memory here.
        with os.scandir(dir_fd) as entries:
            for entry in entries:
                name = entry.name
                kind = _classify(name)
                if kind is None:
                    continue

                # No O_CREAT, so a lock file that vanished between the scan and here is a
                # skip rather than a resurrection; O_NOFOLLOW so a symlink dropped in this
                # world-writable directory under a Fast-DDS-looking name is never opened; and
                # O_NONBLOCK so the open itself cannot wait on something that is not a regular
                # file, which is only established afterwards, from the descriptor.
                try:
                    lock_fd = os.open(
                        name,
                        os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK,
                        dir_fd=dir_fd,
                    )
                except OSError:
                    # Vanished, a symlink, or not writable by this user — skip either way.
                    # Note this is not what keeps another user's files safe: Fast-DDS creates
                    # them 0666 before umask, and root ignores the mode entirely. The ownership
                    # check on each companion below is the control that matters.
                    continue

                try:
                    # Everything from here on is judged on the DESCRIPTOR, never on the name:
                    # the name could be renamed onto a different file between any two lookups,
                    # and this is the file whose lock is about to decide another file's fate.
                    lock_info = os.fstat(lock_fd)
                    if not _is_lock_file(lock_info) or now - lock_info.st_mtime < min_age:
                        # Not a Fast-DDS lock file (non-empty, hardlinked, or not a regular
                        # file), or too young to judge — either a participant starting up right
                        # now, or one that died moments ago and will be reclaimed by the next
                        # sweep.
                        continue

                    try:
                        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except OSError:
                        # The owner holds the lock, so the owner is alive. The kernel would
                        # have dropped it otherwise, SIGKILL included — this is the whole
                        # safety argument.
                        continue

                    # Dead owner. Unlink while still holding the lock, so a participant that
                    # starts up during this sweep either waits behind it or creates its files
                    # afresh afterwards. The companions are removed only if they belong to the
                    # same user as the lock file (see _remove_companion_file); the lock file
                    # itself is removed regardless, since its own liveness is what was just
                    # proven.
                    object_name = name[:-_LOCK_SUFFIX_LENGTH]
                    reclaimed, freed = _remove_companion_file(
                        dir_fd, object_name, lock_info.st_uid
                    )
                    if kind == _PORT:
                        # Ports also have a POSIX named semaphore, which lives in the same
                        # directory as "sem.<port object name>_mutex" and leaks with the rest.
                        _, semaphore_freed = _remove_companion_file(
                            dir_fd, f"sem.{object_name}_mutex", lock_info.st_uid
                        )
                        freed += semaphore_freed
                    try:
                        os.unlink(name, dir_fd=dir_fd)
                        freed += lock_info.st_size
                    except OSError:
                        pass

                    stats.size_bytes += freed
                    # Counted on the OBJECT, not on the lock file: a port can have both an
                    # "_el" and an "_sl" lock, and counting per lock file would report one
                    # reclaimed port as two. A lock file with no object left is still removed,
                    # just not counted — there is no object to have reclaimed.
                    if reclaimed:
                        if kind == _PORT:
                            stats.ports += 1
                        else:
                            stats.segments += 1
                finally:
                    # Releases the lock; the file it belonged to is gone by now.
                    os.close(lock_fd)
    finally:
        os.close(dir_fd)

    return stats


def cleanup_shared_memory_once() -> None:
    """Sweep once per process, on the first call. Subsequent calls do nothing.

    Called immediately before the process creates its first Fast-DDS participant, so that
    a service restarted in a loop buries its own predecessor's corpse: the steady state
    becomes at most one dead generation per service rather than unbounded growth."""

    global _swept_once, _has_swept, _last_sweep
    with _sweep_lock:
        if _swept_once:
            return
        _swept_once = True
        _has_swept = True
        _last_sweep = time.monotonic()

    # Nothing is logged, however much is reclaimed: this is housekeeping the caller neither
    # asked for nor can act on. What it reclaims is, by definition, memory no live process can
    # reach.
    sweep_dead_shared_memory()


def cleanup_shared_memory_if_due() -> bool:
    """Sweep unless a sweep already ran recently (30 s). Silent, as above.

    Called when a participant finds the shared-memory filesystem nearly full, so a
    long-running process heals its host instead of only complaining about it. The rate
    limit is shared with :func:`cleanup_shared_memory_once`, whose sweep therefore also
    suppresses an immediately-following one.

    :return: whether a sweep actually ran (``False`` when rate-limited), so the caller can
        re-check the free space it complained about
    """

    global _has_swept, _last_sweep
    with _sweep_lock:
        now = time.monotonic()
        if _has_swept and now - _last_sweep < _MIN_SWEEP_INTERVAL_SEC:
            return False
        # Stamped before the sweep rather than after it, so a second thread arriving while
        # this one scans the directory is turned away immediately instead of duplicating
        # the work.
        _has_swept = True
        _last_sweep = now

    sweep_dead_shared_memory()
    return True


def _shared_memory_space():
    """``(capacity, available)`` of the shared-memory filesystem in bytes, or
    ``(0, 0)`` when it cannot be determined."""

    directory = shared_memory_dir()
    if directory is None:
        return 0, 0
    try:
        info = os.statvfs(directory)
    except OSError:
        return 0, 0
    return info.f_blocks * info.f_frsize, info.f_bavail * info.f_frsize


# Set once the near-full warning has been emitted, so a process creating participants in a
# loop says it once rather than on every one.
_warned_nearly_full = False


def manage_shared_memory_space() -> None:
    """Reclaim the shared-memory files of participants that died without cleaning up, and
    report a shared-memory filesystem that still cannot fit a handful more segments
    afterwards.

    Called before creating each Fast-DDS participant that may use shared memory.
    Exhaustion otherwise shows up only as an obscure "Failed to create segment" from
    Fast-DDS followed by a silent, host-wide fallback to UDP. Mirrors
    ``manage_shared_memory_space`` in src/domain_participant.cpp."""

    global _warned_nearly_full

    # Before anything else, and exactly once per process: bury the corpses. A service that
    # exits and is restarted in a loop then reclaims its own predecessor's segment on
    # every incarnation, so the steady state is one dead generation rather than unbounded
    # growth — whatever else on the host is or isn't fixed.
    cleanup_shared_memory_once()

    capacity, available = _shared_memory_space()
    if capacity == 0 or available >= _LOW_WATER_BYTES:
        return

    # Still short. Sweep again (rate-limited, and suppressed entirely right after the
    # once-per-process sweep above) so a long-running process heals the host over its
    # lifetime instead of only complaining about it, and re-check before saying anything.
    if cleanup_shared_memory_if_due():
        capacity, available = _shared_memory_space()
        if capacity == 0 or available >= _LOW_WATER_BYTES:
            return

    with _sweep_lock:
        if _warned_nearly_full:
            return
        _warned_nearly_full = True
    _network_recovery._emit_log(
        _network_recovery.LogLevel.WARNING,
        f"{shared_memory_dir()} has only {available // _BYTES_PER_MEBIBYTE} MiB free of "
        f"{capacity // _BYTES_PER_MEBIBYTE} MiB; shared-memory transport registration can "
        f"fail and fall back to UDP. What is left is in use, or held by files this process "
        f"may not remove — segments of participants that did not exit cleanly are "
        f"automatically reclaimed (see {_ENABLED_ENV})",
    )
