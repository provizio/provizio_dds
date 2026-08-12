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
"""Subcommand-driven tests for the dead-owner shared-memory sweep — Python
mirror of the C++ test/shm_cleanup/ suite. The two layers share no code (the
Python bindings sit on eProsima's SWIG module and never enter the C++
provizio::dds core), so both are tested the same way.

They cover what the sweep reclaims (segments, ports and their named semaphores,
under both the 3.x "fastdds_" and the 2.x "fastrtps_" naming), what it must
never touch (files whose owner still holds their lock — including a live
participant's, and this very process's own — files younger than the age guard,
and everything in the directory that is not a Fast-DDS lock file), that the kill
switch really switches it off, that repeat sweeps are rate-limited, and that
creating a participant runs it. Each subcommand is its own ctest entry so
per-case failure stays isolated.

Linux-only: shared memory is the default transport there and nowhere else in
provizio_dds (Windows and macOS force UDP over a Boost.Interprocess cleanup
bug), so no other platform has anything to reclaim."""

import fcntl
import multiprocessing
import os
import signal
import stat
import sys
import threading
import time
import traceback

import provizio_dds

# Domains of their own, so the live-participant case can neither feed nor be fed by the
# rest of the suite, which lives on domain 0. The killed-participant case puts its
# sweeping participant on a SEPARATE domain from the child it killed: Fast-DDS derives
# shared-memory port numbers from the domain, so a same-domain participant would
# immediately recreate the very port files the sweep had just reclaimed.
DOMAIN = 71
SWEEPER_DOMAIN = 72

# Where Fast-DDS keeps its shared-memory objects on Linux.
PLATFORM_SHM_DIR = "/dev/shm"
ENABLED_ENV = "PROVIZIO_DDS_SHM_CLEANUP"
MIN_AGE_ENV = "PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC"

# Most cases sweep a directory of their own (see PrivateShmDir) rather than the host's live
# one: nothing else on the machine can then reclaim their fixtures, nothing of the host's is
# counted as theirs, and no age guard is needed in either direction.
NO_AGE_GUARD = "0"

# The age-guard case is the exception that needs the guard itself under test. Its fixtures are
# still private, so the numbers only have to be far apart: a guard well above the time the case
# takes to run, and a corpse backdated past it.
AGE_GUARD_MIN_AGE = "30"
AGE_GUARD_BACKDATE_SECONDS = 60

# The cases that must work in the REAL shared-memory directory, alongside live participants,
# instead lower the guard just enough to reach their own fixtures while staying under the 5 s
# default every other provizio_dds process on the host uses — so a participant starting up
# elsewhere in a parallel ctest run can never reclaim them mid-case. (Those cases are also kept
# apart from each other, and from the C++ mirror's, by a ctest RESOURCE_LOCK.)
SHARED_DIR_MIN_AGE = "1"
SHARED_DIR_BACKDATE_SECONDS = 2

# The directory the current case works in — the platform one unless it claimed a private
# directory of its own.
_shm_dir = PLATFORM_SHM_DIR

LOCK_SUFFIX_LENGTH = 3
SEGMENT_ID_LENGTH = 16

MAX_WAIT_TIME = 20
PUBLISH_EVERY_SEC = 0.1

_failures = []


def expect(condition, description):
    if not condition:
        print(f"FAIL: {description}")
        _failures.append(description)
    return condition


def shm_path(name):
    return os.path.join(_shm_dir, name)


def exists(name):
    return os.path.lexists(shm_path(name))


def inode_of(name):
    """The file's inode number, or 0 when it does not exist.

    Identity rather than presence is what "was it reclaimed?" needs: a participant
    starting up right after a sweep can legitimately recreate a name the sweep just freed
    (port names are derived from the domain, so they repeat), and tmpfs hands out inode
    numbers from a monotonic counter, so a different inode under the same name is provably
    a different file."""
    try:
        return os.lstat(shm_path(name)).st_ino
    except OSError:
        return 0


def reclaimed(name, previous_inode):
    """Whether the file called ``name`` is gone or has since been replaced by a different
    one. An inode of 0 means it was not there to begin with, which counts as reclaimed."""
    return previous_inode == 0 or inode_of(name) != previous_inode


def backdate(name, seconds_ago):
    """Set a file's modification time into the past, so a synthetic lock file can pose as
    a corpse the age guard is willing to reclaim."""
    when = time.time() - seconds_ago
    os.utime(shm_path(name), (when, when))


def hardlink(existing, link_name):
    """Hardlink ``existing`` to ``link_name``, giving both a link count of 2."""
    try:
        os.link(shm_path(existing), shm_path(link_name))
        return True
    except OSError:
        return False


class FixtureFiles:
    """Every synthetic file a case created, removed on the way out however the case ends."""

    def __init__(self):
        self._names = []

    def add(self, name, size=0):
        """Create ``name`` and remember it for teardown, failing (rather than truncating)
        when the name is already taken — a synthetic fixture must never land on a name a
        real participant is using. Returns whether it was created."""
        try:
            fd = os.open(shm_path(name), os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o644)
        except OSError:
            return False
        self._names.append(name)
        try:
            if size:
                os.ftruncate(fd, size)
        finally:
            os.close(fd)
        return True

    def cleanup(self):
        for name in self._names:
            try:
                os.unlink(shm_path(name))
            except OSError:
                pass
        self._names = []

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.cleanup()
        return False


class PrivateShmDir:
    """Claims a directory of this case's own, inside the shared-memory filesystem so the
    fixtures keep tmpfs semantics (flock, unlink, monotonic inode numbers), and points every
    helper above at it for the case's lifetime.

    This is what makes the synthetic cases hermetic: no other provizio_dds process on the host
    sweeps a subdirectory (the sweep does not recurse, and the directory's own name matches no
    lock-file shape), so a fixture cannot be reclaimed by someone else's participant starting
    up — and equally, a case can assert exact counts because nothing of the host's own leaked
    shared memory is in scope. Enter it BEFORE the case's FixtureFiles so the files are removed
    before the directory is."""

    def __init__(self, name):
        self._path = os.path.join(
            PLATFORM_SHM_DIR, f"provizio_dds_shm_cleanup_{name}_{unique_tag()}"
        )
        self._claimed = False

    def path(self):
        return self._path

    def claimed(self):
        return self._claimed

    def __enter__(self):
        global _shm_dir
        try:
            os.mkdir(self._path, 0o700)
        except OSError:
            return self
        self._claimed = True
        _shm_dir = self._path
        return self

    def __exit__(self, *_):
        global _shm_dir
        if self._claimed:
            _shm_dir = PLATFORM_SHM_DIR
            try:
                os.rmdir(self._path)
            except OSError:
                pass
        return False


def unique_tag():
    """A per-process discriminator, so several test processes (and the rest of a parallel
    ctest run) can never pick the same synthetic names. 12 hex digits of the pid, which
    leaves room for a 4-character marker within a 16-character segment id and is
    unmistakably not a real (random) Fast-DDS id."""
    return f"{os.getpid():012x}"[-12:]


def segment_name(prefix, marker="dead"):
    """``<prefix><4-character marker><12 hex of the pid>`` — a syntactically valid segment
    name. The marker distinguishes several fixtures within one case and says what each is
    for."""
    return prefix + marker + unique_tag()


def create_dead_segment(files, prefix, backdate_seconds=0, marker="dead"):
    """Create a segment corpse: the object plus its exclusive lock file, locked by nobody.

    ``backdate_seconds`` ages it when the case needs it past a guard; 0 leaves the mtime where
    it is, which is what a case sweeping with no guard at all wants. Returns the object name,
    or None on failure."""
    name = segment_name(prefix, marker)
    if not files.add(name, 4096) or not files.add(name + "_el"):
        return None
    if backdate_seconds:
        backdate(name, backdate_seconds)
        backdate(name + "_el", backdate_seconds)
    return name


def candidate_port(offset):
    """A synthetic port number in the "...5" tail Fast-DDS never assigns (its own ports end
    in 0, 1 or 3), spread across processes by pid. ``offset`` walks further candidates."""
    return 60005 + ((os.getpid() % 90) + offset) * 10


def create_dead_port(files, prefix, lock_suffix="_el"):
    """Create a port corpse: the object, its exclusive lock file and its named semaphore, all
    unlocked. Names are claimed with O_EXCL, so a live port cannot be shadowed even if the
    candidate arithmetic ever collides. Returns the object name, or None on failure."""
    for attempt in range(16):
        name = f"{prefix}port{candidate_port(attempt)}"
        if not files.add(name, 52400):
            continue  # Taken — try the next candidate.
        semaphore = f"sem.{name}_mutex"
        if not files.add(name + lock_suffix) or not files.add(semaphore):
            return None
        return name
    return None


class ScopedFlock:
    """Takes an exclusive flock on a file and holds it until released — exactly what a live
    Fast-DDS participant does with its lock files, and what tells the sweep to keep its
    hands off. Also used the other way round, to ask whether anyone else holds one."""

    def __init__(self, name):
        self._fd = None
        try:
            fd = os.open(shm_path(name), os.O_RDWR | os.O_CLOEXEC)
        except OSError:
            return
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            return
        self._fd = fd

    def acquired(self):
        """Whether the lock was taken — which also means nobody else was holding it."""
        return self._fd is not None

    def release(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.release()
        return False


def is_lock_file_name(name):
    """An independent re-implementation of the sweep's name matcher, deliberately not
    shared with it: a case that asks "what was reclaimable before the sweep?" must not
    inherit the very classification it is checking. Returns ``None`` when ``name`` is not a
    lock file, else whether it is a port's (as opposed to a segment's)."""
    if len(name) <= LOCK_SUFFIX_LENGTH or not name.endswith(("_el", "_sl")):
        return None
    exclusive_lock = name.endswith("_el")
    body = name[:-LOCK_SUFFIX_LENGTH]
    for prefix in ("fastdds_", "fastrtps_"):
        if not body.startswith(prefix) or len(body) <= len(prefix):
            continue
        tail = body[len(prefix):]
        if (
            exclusive_lock
            and len(tail) == SEGMENT_ID_LENGTH
            and all("0" <= character <= "9" or "a" <= character <= "z" for character in tail)
        ):
            return False
        if tail.startswith("port") and 0 < len(tail) - 4 <= 5 and tail[4:].isascii() and tail[4:].isdigit():
            return True
    return None


def reclaimable_objects(min_age_seconds):
    """Every Fast-DDS shared-memory object in the directory this process could reclaim
    right now: nothing holds its lock, and it is old enough for the given age guard. The
    sweep's contract is that ALL of these go and nothing else does — which is what makes
    the killed-participant case immune to whatever else happens to be running on the host,
    rather than depending on a remembered list of one child's files.

    Returns a list of ``(lock name, lock inode, object inode, is port)`` tuples."""
    result = []
    now = time.time()
    # Deliberately one second stricter than the sweep's own guard, mirroring the C++ suite:
    # everything this returns must be something the sweep will certainly take, so it rounds
    # the wrong way on purpose.
    strict_min_age = min_age_seconds + 1
    for name in os.listdir(_shm_dir):
        is_port = is_lock_file_name(name)
        if is_port is None:
            continue
        try:
            info = os.lstat(shm_path(name))
        except OSError:
            continue
        # Same predicate the sweep applies to a lock file, plus the stricter age.
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_size != 0
            or info.st_nlink != 1
            or now - info.st_mtime < strict_min_age
        ):
            continue
        with ScopedFlock(name) as probe:
            if probe.acquired():
                result.append(
                    (name, info.st_ino, inode_of(name[:-LOCK_SUFFIX_LENGTH]), is_port)
                )
    return result


# ---- Child process ---------------------------------------------------------------


def participant_child(ready, publish_topic):
    """Runs in a forked child: creates a participant — whose shared-memory files are what
    the parent is really testing against — and signals ``ready``. Never returns; the parent
    SIGKILLs it, which is the whole point (the kernel then drops its flocks and Fast-DDS
    gets no chance to remove anything)."""
    # A child must never sweep: the parent owns every assertion about what disappeared.
    os.environ[ENABLED_ENV] = "0"

    participant = provizio_dds.make_domain_participant(
        DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
    )
    publisher = None
    if publish_topic:
        publisher = provizio_dds.Publisher(
            participant, publish_topic, provizio_dds.StringPubSubType
        )

    ready.set()

    message = provizio_dds.String()
    message.data("alive")
    while True:
        if publisher is not None:
            publisher.publish(message)
        time.sleep(PUBLISH_EVERY_SEC)


class ParticipantChild:
    """A forked child running a participant, SIGKILLed on the way out."""

    def __init__(self, publish_topic=""):
        self._ready = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=participant_child, args=(self._ready, publish_topic), daemon=True
        )
        self._process.start()

    def ready(self):
        return self._ready.wait(MAX_WAIT_TIME)

    def alive(self):
        return self._process.is_alive()

    def kill_and_reap(self):
        if self._process.pid is not None and self._process.is_alive():
            os.kill(self._process.pid, signal.SIGKILL)
        self._process.join(MAX_WAIT_TIME)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.kill_and_reap()
        return False


# ---- Cases -----------------------------------------------------------------------


def test_reclaims_synthetic():
    """A segment corpse and a port corpse — under both the 3.x and the 2.x naming — are
    reclaimed with every file that belongs to them, the port's named semaphore included."""
    os.environ[MIN_AGE_ENV] = NO_AGE_GUARD
    with PrivateShmDir("synthetic") as directory, FixtureFiles() as files:
        if not expect(directory.claimed(), "reclaims_synthetic: claimed a private directory"):
            return
        fastdds_segment = create_dead_segment(files, "fastdds_")
        fastrtps_segment = create_dead_segment(files, "fastrtps_")
        fastdds_port = create_dead_port(files, "fastdds_")
        fastrtps_port = create_dead_port(files, "fastrtps_")
        if not expect(
            fastdds_segment and fastrtps_segment and fastdds_port and fastrtps_port,
            "reclaims_synthetic: could not lay out the fixture files",
        ):
            return

        stats = provizio_dds.shm_cleanup.sweep_dead_shared_memory(directory.path())

        for segment in (fastdds_segment, fastrtps_segment):
            expect(not exists(segment), f"{segment} removed")
            expect(not exists(segment + "_el"), f"{segment}_el removed")
        for port in (fastdds_port, fastrtps_port):
            expect(not exists(port), f"{port} removed")
            expect(not exists(port + "_el"), f"{port}_el removed")
            expect(not exists(f"sem.{port}_mutex"), f"sem.{port}_mutex removed")
        # The directory holds nothing but this case's fixtures, so the counts are exact.
        expect(stats.segments == 2, f"{stats.segments} segments reclaimed == 2")
        expect(stats.ports == 2, f"{stats.ports} ports reclaimed == 2")
        expect(stats.anything_reclaimed(), "the sweep reported something reclaimed")

    print(f"reclaims_synthetic: {'PASS' if not _failures else 'FAIL'} ({stats})")


def test_reclaims_dead_participant():
    """The real thing — a participant is SIGKILLed, and the next participant created in
    this process reclaims what it left behind, before it creates its own."""
    # Lowered just enough to reach the child's files, which are older than that by the time
    # the sweep runs; a participant starting up elsewhere in a parallel ctest run is still
    # shielded by it. See SHARED_DIR_MIN_AGE.
    min_age = int(SHARED_DIR_MIN_AGE)
    os.environ[MIN_AGE_ENV] = SHARED_DIR_MIN_AGE

    with ParticipantChild() as child:
        if not expect(child.ready(), "reclaims_dead_participant: the child came up"):
            return
    # SIGKILLed here — no cleanup of any kind runs in it.

    # Past the age guard — and past the stricter one reclaimable_objects scans with, or the
    # child's freshly-abandoned files would not make its list.
    time.sleep(min_age + 1.5)

    before = reclaimable_objects(min_age)
    if not expect(before, "the killed child left reclaimable files behind") or not expect(
        any(not is_port for _, _, _, is_port in before),
        "the killed child left a reclaimable segment behind",
    ):
        return

    # The trigger under test: creating a participant, nothing else. On a domain of its
    # own, so its own port files cannot be confused with the reclaimed ones.
    participant = provizio_dds.make_domain_participant(
        SWEEPER_DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
    )
    assert participant is not None

    for lock_name, lock_inode, object_inode, _ in before:
        expect(reclaimed(lock_name, lock_inode), f"{lock_name} reclaimed")
        expect(
            reclaimed(lock_name[:-LOCK_SUFFIX_LENGTH], object_inode),
            f"{lock_name[:-LOCK_SUFFIX_LENGTH]} reclaimed",
        )

    print(
        f"reclaims_dead_participant: {'PASS' if not _failures else 'FAIL'} "
        f"({len(before)} reclaimable file(s) before the sweep)"
    )


def test_spares_the_living():
    """A sweep must not disturb anything alive. Three live owners are exercised at once —
    a synthetic lock file this process holds, the participant of a separate live process,
    and this process's own participant — and the shared-memory data path between the last
    two must keep carrying samples across the sweep."""
    # Low enough that everything here is old enough to be reclaimed on age by the time the
    # sweep runs, so only the locks can be what saves it. See SHARED_DIR_MIN_AGE.
    min_age = int(SHARED_DIR_MIN_AGE)
    os.environ[MIN_AGE_ENV] = SHARED_DIR_MIN_AGE

    with FixtureFiles() as files:
        held = segment_name("fastdds_", "live")
        if not expect(
            files.add(held, 4096) and files.add(held + "_el"),
            "spares_the_living: could not lay out the fixture files",
        ):
            return
        backdate(held, SHARED_DIR_BACKDATE_SECONDS)
        backdate(held + "_el", SHARED_DIR_BACKDATE_SECONDS)

        with ScopedFlock(held + "_el") as lock:
            if not expect(lock.acquired(), "the fixture lock file was locked"):
                return

            # Stamped with THIS process's pid and handed to the child before it forks, so
            # two hosts running this case at the same time cannot feed each other's
            # subscriber.
            topic = "provizio_dds_python_test_shm_cleanup_" + unique_tag()
            with ParticipantChild(topic) as child:
                expect(child.ready(), "the publishing child came up")

                condition = threading.Condition()
                counter = {"count": 0}

                def on_message(_):
                    with condition:
                        counter["count"] += 1
                        condition.notify_all()

                participants_created = time.monotonic()
                participant = provizio_dds.make_domain_participant(
                    DOMAIN, provizio_dds.NetworkRecoveryMode.OFF
                )
                subscriber = provizio_dds.Subscriber(
                    participant,
                    topic,
                    provizio_dds.StringPubSubType,
                    provizio_dds.String,
                    on_message,
                )
                assert subscriber is not None

                def wait_for(target):
                    with condition:
                        return condition.wait_for(
                            lambda: counter["count"] >= target, MAX_WAIT_TIME
                        )

                expect(wait_for(3), "samples arrived before the sweep")

                # A corpse laid out right next to the living — and only now, after the
                # participant creation above has done its own once-per-process sweep — so
                # the sweep below provably did its rounds rather than bailing out early
                # and "sparing" everything by doing nothing at all.
                corpse = create_dead_segment(files, "fastdds_", SHARED_DIR_BACKDATE_SECONDS)
                expect(corpse is not None, "the corpse fixture was laid out")

                # Wait out the age guard, so both participants' files are past it and
                # nothing but their locks stands between them and the sweep. Without this
                # the case could pass for the wrong reason on a fast host.
                time.sleep(max(0.0, min_age + 1 - (time.monotonic() - participants_created)))

                stats = provizio_dds.shm_cleanup.sweep_dead_shared_memory()

                expect(exists(held), f"{held} survived the sweep")
                expect(exists(held + "_el"), f"{held}_el survived the sweep")
                expect(not exists(corpse), f"{corpse} was reclaimed by the same sweep")
                expect(stats.anything_reclaimed(), "the sweep reported something reclaimed")
                expect(child.alive(), "the publishing child survived the sweep")

                with condition:
                    count_at_sweep = counter["count"]
                # The data path survives: samples keep arriving after the sweep.
                expect(wait_for(count_at_sweep + 5), "samples kept arriving after the sweep")

    print(
        f"spares_the_living: {'PASS' if not _failures else 'FAIL'} "
        f"({count_at_sweep} samples before the sweep)"
    )


def test_age_guard():
    """A lock file younger than the threshold is left alone however unlocked it is — that
    is the window in which a participant has created its segment but not yet taken its
    lock — while an older one is reclaimed."""
    os.environ[MIN_AGE_ENV] = AGE_GUARD_MIN_AGE

    with PrivateShmDir("age") as directory, FixtureFiles() as files:
        fresh = segment_name("fastdds_", "fres")
        old = segment_name("fastdds_", "dead")
        if not expect(
            directory.claimed()
            and files.add(fresh, 4096)
            and files.add(fresh + "_el")
            and files.add(old, 4096)
            and files.add(old + "_el"),
            "age_guard: could not lay out the fixture files",
        ):
            return
        backdate(old, AGE_GUARD_BACKDATE_SECONDS)
        backdate(old + "_el", AGE_GUARD_BACKDATE_SECONDS)

        provizio_dds.shm_cleanup.sweep_dead_shared_memory(directory.path())

        expect(exists(fresh), f"{fresh} (fresh) survived")
        expect(exists(fresh + "_el"), f"{fresh}_el (fresh) survived")
        expect(not exists(old), f"{old} (old) was reclaimed")
        expect(not exists(old + "_el"), f"{old}_el (old) was reclaimed")

    print(f"age_guard: {'PASS' if not _failures else 'FAIL'}")


def test_kill_switch():
    """PROVIZIO_DDS_SHM_CLEANUP=0 removes nothing at all.
    """
    os.environ[MIN_AGE_ENV] = NO_AGE_GUARD

    for value in ("0", "off", "false", "no"):
        os.environ[ENABLED_ENV] = value
        with PrivateShmDir("kill") as directory, FixtureFiles() as files:
            segment = create_dead_segment(files, "fastdds_")
            port = create_dead_port(files, "fastdds_")
            if not expect(
                directory.claimed() and segment and port,
                "kill_switch: could not lay out the fixture files",
            ):
                return

            stats = provizio_dds.shm_cleanup.sweep_dead_shared_memory(directory.path())
            expect(not stats.anything_reclaimed(), f"{ENABLED_ENV}='{value}' reclaimed nothing")
            expect(stats.size_bytes == 0, f"{ENABLED_ENV}='{value}' freed no bytes")
            expect(exists(segment) and exists(segment + "_el"), f"the segment survived {value}")
            expect(
                exists(port) and exists(port + "_el") and exists(f"sem.{port}_mutex"),
                f"the port survived {value}",
            )
        if _failures:
            return

    # An unrecognised value must leave the sweep ENABLED — a typo must never silently
    # reintroduce the leak this exists to stop.
    os.environ[ENABLED_ENV] = "maybe"
    with PrivateShmDir("kill_unknown") as directory, FixtureFiles() as files:
        corpse = create_dead_segment(files, "fastdds_")
        if expect(
            directory.claimed() and corpse, "the unrecognised-value fixture was laid out"
        ):
            provizio_dds.shm_cleanup.sweep_dead_shared_memory(directory.path())
            expect(not exists(corpse), f"{corpse} reclaimed despite an unrecognised value")
            expect(not exists(corpse + "_el"), f"{corpse}_el reclaimed")

    # Nothing here creates a participant: that would sweep the host's real shared-memory
    # directory, where a corpse of this case's cannot be laid out without another test's
    # participant being free to reclaim it. The end-to-end property is covered by composition
    # instead — reclaims_dead_participant shows that creating a participant runs the sweep, and
    # the four spellings above show the sweep obeys the switch.

    print(f"kill_switch: {'PASS' if not _failures else 'FAIL'}")


def test_leaves_other_files_alone():
    """Everything in the shared-memory directory that is not one of the two recognised
    lock-file shapes is left strictly alone — however old, however unlocked. Data-sharing
    segments matter most (a live reader may map a dead writer's, so liveness there means
    something else entirely), but so does a stranger's file that merely starts with
    "fastdds_", and so does an object whose lock file is already gone: the algorithm is
    driven by lock files and deliberately has nothing to say about an object without one
    (a port opened for writing never gets one, and may well be in use)."""
    os.environ[MIN_AGE_ENV] = NO_AGE_GUARD
    tag = unique_tag()

    decoys = [
        "provizio_dds_shm_cleanup_decoy_" + tag,  # Nothing to do with Fast-DDS.
        "fast_datasharing_" + tag + "_1.2.3",     # Data-sharing: never ours to judge.
        "fastdds_odd" + tag + "_el",              # 15-character id — not a segment name.
        "fastdds_port1234567_el",                 # 7 digits — not a port name.
        "fastdds_dead" + tag + "_ex",             # Neither "_el" nor "_sl".
        "fastdds_dead" + tag,                     # A segment object with no lock file.
        f"sem.fastdds_port{candidate_port(0)}_mutex",  # A semaphore whose port is long gone.
    ]
    with PrivateShmDir("decoys") as directory, FixtureFiles() as files:
        if not expect(directory.claimed(), "claimed a private directory"):
            return
        for decoy in decoys:
            if not expect(files.add(decoy, 128), f"could not lay out the decoy {decoy}"):
                return

        provizio_dds.shm_cleanup.sweep_dead_shared_memory(directory.path())

        for decoy in decoys:
            expect(exists(decoy), f"{decoy} was left alone")

    print(f"leaves_other_files_alone: {'PASS' if not _failures else 'FAIL'}")


def test_rejects_planted_lock_files():
    """A lock file is trusted only when it is one Fast-DDS could actually have written.

    Whoever can write to this world-writable directory could otherwise plant one beside a
    LIVE object and have the sweep delete it — the flock proves only that the LOCK FILE's
    owner is gone, and the object is then reached from its name. Three plants are refused
    here, with a genuine corpse alongside them to prove the sweep still ran."""
    os.environ[MIN_AGE_ENV] = NO_AGE_GUARD

    with PrivateShmDir("planted") as directory, FixtureFiles() as files:
        ok = expect(directory.claimed(), "claimed a private directory")

        # 1. A segment with a SHARED lock file. Fast-DDS gives a segment an exclusive lock and
        #    nothing else, so this name stays free for the taking while the segment is alive —
        #    the cheapest way to aim a sweep at a live participant.
        shared_suffix = segment_name("fastdds_", "shrd")
        ok = ok and files.add(shared_suffix, 4096) and files.add(shared_suffix + "_sl")

        # 2. A lock file with content. Fast-DDS opens its lock files and never writes them, so
        #    anything non-empty was written by something else.
        written = segment_name("fastdds_", "wrtn")
        ok = ok and files.add(written, 4096) and files.add(written + "_el", 1)

        # 3. A lock file hardlinked onto the object itself — the way to make a planted lock
        #    file whose ownership matches its victim's.
        linked = segment_name("fastdds_", "hlnk")
        ok = ok and files.add(linked, 4096) and hardlink(linked, linked + "_el")

        # 4. The control: a real corpse, and specifically a port with a SHARED lock, which
        #    Fast-DDS does create and which nothing else in the suite covers.
        corpse = create_dead_port(files, "fastdds_", "_sl")
        if not expect(ok and corpse, "rejects_planted_lock_files: laid out the fixtures"):
            return

        stats = provizio_dds.shm_cleanup.sweep_dead_shared_memory(directory.path())

        expect(exists(shared_suffix), f"{shared_suffix} (shared-lock plant) survived")
        expect(exists(shared_suffix + "_sl"), f"{shared_suffix}_sl survived")
        expect(exists(written), f"{written} (written-lock plant) survived")
        expect(exists(written + "_el"), f"{written}_el survived")
        expect(exists(linked), f"{linked} (hardlink plant) survived")
        expect(exists(linked + "_el"), f"{linked}_el survived")
        # The control was taken, so the sweep did run over all of the above.
        expect(not exists(corpse), f"{corpse} (the control corpse) was reclaimed")
        expect(not exists(corpse + "_sl"), f"{corpse}_sl was reclaimed")
        expect(not exists(f"sem.{corpse}_mutex"), f"sem.{corpse}_mutex was reclaimed")
        expect(stats.segments == 0, f"{stats.segments} segments reclaimed == 0")
        expect(stats.ports == 1, f"{stats.ports} ports reclaimed == 1")

    print(f"rejects_planted_lock_files: {'PASS' if not _failures else 'FAIL'}")


def test_rate_limited():
    """The pressure-triggered sweep is rate-limited, so a process creating participants in
    a loop on a full host rescans the directory occasionally rather than constantly.

    No private directory here: cleanup_shared_memory_if_due is the production entry point and
    always sweeps the platform's directory, so the fixtures live there under the
    shared-directory guard choreography."""
    os.environ[MIN_AGE_ENV] = SHARED_DIR_MIN_AGE

    with FixtureFiles() as files:
        first = create_dead_segment(files, "fastdds_", SHARED_DIR_BACKDATE_SECONDS)
        if not expect(first is not None, "rate_limited: could not lay out the fixture files"):
            return

        expect(provizio_dds.shm_cleanup.cleanup_shared_memory_if_due(), "the first sweep ran")
        expect(not exists(first), f"{first} was reclaimed")

        # A second corpse appearing immediately afterwards is NOT swept: the rate limit
        # holds for 30 s, far longer than this case runs.
        second = create_dead_segment(files, "fastrtps_", SHARED_DIR_BACKDATE_SECONDS)
        expect(second is not None, "the second corpse fixture was laid out")
        expect(
            not provizio_dds.shm_cleanup.cleanup_shared_memory_if_due(),
            "the immediately following sweep was rate-limited away",
        )
        expect(exists(second), f"{second} survived the rate-limited sweep")
        expect(exists(second + "_el"), f"{second}_el survived the rate-limited sweep")

    print(f"rate_limited: {'PASS' if not _failures else 'FAIL'}")


_SUBCOMMANDS = {
    "reclaims_synthetic": test_reclaims_synthetic,
    "reclaims_dead_participant": test_reclaims_dead_participant,
    "spares_the_living": test_spares_the_living,
    "age_guard": test_age_guard,
    "kill_switch": test_kill_switch,
    "leaves_other_files_alone": test_leaves_other_files_alone,
    "rejects_planted_lock_files": test_rejects_planted_lock_files,
    "rate_limited": test_rate_limited,
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in _SUBCOMMANDS:
        print(f"usage: {sys.argv[0]} <{'|'.join(_SUBCOMMANDS)}>", file=sys.stderr)
        return 1
    if sys.platform != "linux":
        print(f"{sys.argv[1]}: SKIPPED (shared memory is disabled on {sys.platform})")
        return 0

    # Hermetic: the suite's loopback XML profile replaces the built-in transports with a
    # single UDPv4 one, so a participant created under it never touches shared memory at
    # all — and every case here is about shared-memory files. Clear it, as the
    # discovery_tuning and transport_tuning cases do. Cross-host discovery is harmless
    # here: the one case that exchanges samples uses a domain and a pid-stamped topic of
    # its own.
    os.environ.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
    # The default, made explicit so the ambient environment cannot switch the sweep off
    # under a case that asserts it happened (kill_switch sets it itself).
    os.environ[ENABLED_ENV] = "1"

    try:
        _SUBCOMMANDS[sys.argv[1]]()
    except Exception:  # noqa: BLE001 — surface any unexpected error as a failure
        traceback.print_exc()
        return 1
    return 0 if not _failures else 1


if __name__ == "__main__":
    sys.exit(main())
