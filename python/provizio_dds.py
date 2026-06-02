# Copyright 2023 Provizio Ltd.
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
Python library for DDS communication in Provizio customer facing APIs and
internal Provizio software components. Built using eProsima Fast-DDS DDS
implementation (Apache License 2.0).
"""
import asyncio
import atexit
import inspect
import os
import sys
import threading
import weakref
from typing import Any, Callable, Optional
import time
from queue import Queue

# On Windows, Python 3.8+ restricts DLL search paths (https://bugs.python.org/issue46276).
# Add the directory containing this module so dependent DLLs (provizio_dds, Fast-DDS,
# OpenSSL, etc.) can be found.  All required DLLs must be co-located with this module
# (pip install does this automatically; ctest copies them in test/python/CMakeLists.txt).
# Scanning PATH is intentionally avoided: there can be ABI-incompatible copies of
# common libraries (e.g. MinGW/MySQL/PHP OpenSSL) that cause access violations when
# loaded instead of the MSVC-built copies we ship.
if os.name == "nt":
    # Keep the handle alive: closing it removes the directory from the search path.
    _dll_directory_handle = os.add_dll_directory(os.path.dirname(os.path.abspath(__file__)))

    # Preload fastdds-X.Y.dll (and its fastcdr-X.Y.dll dependency) via ctypes
    # before triggering `from fastdds import *` below. The upstream
    # Fast-DDS-Python 2.x package's generated fastdds.fastdds module calls
    # `win32api.LoadLibrary('fastdds-X.Y.dll')` at import time, and
    # win32api uses the legacy Win32 LoadLibrary which does *not* respect
    # `os.add_dll_directory` (that flag only feeds the newer
    # LoadLibraryEx LOAD_LIBRARY_SEARCH_USER_DIRS path). ctypes.WinDLL on
    # Python 3.8+ uses the LoadLibraryEx path and *does* respect
    # add_dll_directory, so preloading here brings the DLL into the
    # process and the later win32api.LoadLibrary call returns the same
    # handle without consulting the search path.
    #
    # The `fastdds*-*.dll` / `fastcdr*-*.dll` patterns intentionally match
    # both the Release naming (`fastdds-3.6.dll`) and the Debug naming
    # with the `d` suffix Fast-DDS adds via FAST_DDS_DEBUG_SUFFIX
    # (`fastddsd-3.6.dll`). The pip wheel always builds Release, but in-
    # tree Debug builds reuse this same import path and would otherwise
    # silently no-op the preload.
    import ctypes as _ctypes
    import glob as _glob
    _module_dir = os.path.dirname(os.path.abspath(__file__))
    for _dll in sorted(_glob.glob(os.path.join(_module_dir, "fastcdr*-*.dll"))):
        _ctypes.WinDLL(_dll)
    for _dll in sorted(_glob.glob(os.path.join(_module_dir, "fastdds*-*.dll"))):
        _ctypes.WinDLL(_dll)

# Import fastdds first: on Windows, _provizio_dds_python_types.pyd has a DLL-level
# dependency on _fastdds_python.pyd.  If provizio_dds_python_types is imported first,
# Windows loads _fastdds_python.pyd as a regular DLL (DllMain only), without calling
# PyInit__fastdds_python, leaving SWIG type tables uninitialised.  Importing fastdds
# first ensures the module is properly initialised before anything depends on it.
from fastdds import *
from provizio_dds_python_types import *

if __package__ or "." in __name__:
    from . import point_cloud2
    from . import accumulation
    from . import network_recovery as _network_recovery
else:
    import point_cloud2
    import accumulation
    import network_recovery as _network_recovery

# Re-export the network-recovery public surface so user code can import it
# directly from `provizio_dds`. Mirrors the C++ side where the symbols live
# under `provizio::dds`. Also expose the underlying module as the
# attribute `network_recovery` so test code (and any caller that wants the
# private hooks) can do `from provizio_dds import network_recovery` — this
# is a module file, not a package, so a plain `import` would otherwise
# fail to resolve the submodule path.
network_recovery = _network_recovery
NetworkRecoveryMode = _network_recovery.NetworkRecoveryMode
LogLevel = _network_recovery.LogLevel
set_log_callback = _network_recovery.set_log_callback
resolve_network_recovery_enabled = _network_recovery.resolve_network_recovery_enabled


# Tracks, per thread, whether we are executing inside a Fast-DDS listener
# callback. Endpoint teardown (delete_data{writer,reader}) joins the Fast-DDS
# listener / event thread, so it must never run on one — an endpoint __del__
# fired on a callback thread (e.g. a dropped reference) consults this and
# defers its Fast-DDS teardown to the reaper thread instead of self-joining.
_listener_callback_tls = threading.local()


def _on_fastdds_callback_thread() -> bool:
    return getattr(_listener_callback_tls, "depth", 0) > 0


class _fastdds_callback_scope:  # noqa: N801 — lower_snake to read like a CM
    """Marks the current thread as running a Fast-DDS listener callback for
    the duration of the ``with`` block (see :data:`_listener_callback_tls`)."""

    def __enter__(self):
        _listener_callback_tls.depth = getattr(_listener_callback_tls, "depth", 0) + 1
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        _listener_callback_tls.depth -= 1
        return False


# --------------------------------------------------------------------------- #
# Backwards-compat shims for the Fast-DDS-python 1.x → 2.x API renames        #
# --------------------------------------------------------------------------- #
#
# Fast-DDS 3.x (and the matching Fast-DDS-python 2.x bindings) renamed
# several camelCase APIs to snake_case and flattened the `ReturnCode_t`
# enum class to module-level `RETCODE_*` constants. provizio_dds is
# expected to be source-compatible with 1.10.x consumer code, so we
# re-expose the old spellings here. The wrapper's own code uses the new
# snake_case form — these shims exist purely so user code written
# against 1.10.x keeps importing and running unchanged.
import fastdds as _fastdds  # noqa: E402

if not hasattr(_fastdds, "ReturnCode_t"):
    class _ReturnCode_t_shim:  # pylint: disable=invalid-name
        """Stand-in for the Fast-DDS-python 1.x `fastdds.ReturnCode_t`
        enum class. The 2.x bindings dropped the enclosing class and
        exposed each return code as a module-level integer constant
        (`fastdds.RETCODE_OK`); this shim re-mounts them under their
        legacy `ReturnCode_t.RETCODE_*` path."""

    for _attr_name in dir(_fastdds):
        if _attr_name.startswith("RETCODE_"):
            setattr(_ReturnCode_t_shim, _attr_name, getattr(_fastdds, _attr_name))
    _fastdds.ReturnCode_t = _ReturnCode_t_shim
    # The `from fastdds import *` above already executed by the time we
    # add this attribute to `_fastdds`, so that import does not pull the
    # shim into the provizio_dds namespace. Bind it explicitly here so
    # consumer code spelling `provizio_dds.ReturnCode_t.RETCODE_OK`
    # resolves to the legacy 1.x path.
    ReturnCode_t = _ReturnCode_t_shim

# `TopicDataType::getName()` was renamed to `get_name()` in Fast-DDS 3.x.
# Add a method alias on the base class — it propagates to every SWIG-
# generated `*PubSubType` subclass, so user code that called
# `MyTypePubSubType().getName()` continues to resolve.
if hasattr(_fastdds, "TopicDataType"):
    _topic_data_type = _fastdds.TopicDataType
    if hasattr(_topic_data_type, "get_name") and not hasattr(_topic_data_type, "getName"):
        _topic_data_type.getName = _topic_data_type.get_name


class QosDefaults:
    """Defines default QOS policies. They can be overriden for specific types"""

    """Per type defaults for datawriter_reliability_kind. RELIABLE_RELIABILITY_QOS by default in Fast DDS"""
    datawriter_reliability_kind_per_type = {None: RELIABLE_RELIABILITY_QOS}

    """Per type defaults for datareader_reliability_kind. BEST_EFFORT_RELIABILITY_QOS by default in Fast DDS"""
    datareader_reliability_kind_per_type = {None: BEST_EFFORT_RELIABILITY_QOS}

    """Per type defaults for memory policies, both datawriter and datareader. PREALLOCATED_WITH_REALLOC_MEMORY_MODE in Fast-DDS 2.9+"""
    memory_policy_per_type = {None: PREALLOCATED_WITH_REALLOC_MEMORY_MODE}

    """Number of initial participants discovery messages to be broadcast on period of initial_announcements_period"""
    num_initial_discovery_announcements = 200

    """Period of broadcasting initial participants discovery messages"""
    initial_announcements_period = Duration_t(0, 50000000)  # As (sec, nanosec)

    """Period of broadcasting participants discovery messages after the initial announcements"""
    lease_duration_announcement_period = Duration_t(1, 0)  # As (sec, nanosec)

    def __init__(self, pub_sub_type: TopicDataType):
        """Constructs an instance of QosDefaults for the DDS Pub/Sub type.

        :param pub_sub_type: The DDS PubSub Type, f.e. provizio_dds.StringPubSubType
        """
        try:
            self.datawriter_reliability_kind = (
                QosDefaults.datawriter_reliability_kind_per_type[pub_sub_type]
            )
        except KeyError:
            self.datawriter_reliability_kind = (
                QosDefaults.datawriter_reliability_kind_per_type[None]
            )
        try:
            self.datareader_reliability_kind = (
                QosDefaults.datareader_reliability_kind_per_type[pub_sub_type]
            )
        except KeyError:
            self.datareader_reliability_kind = (
                QosDefaults.datareader_reliability_kind_per_type[None]
            )
        try:
            self.memory_policy = QosDefaults.memory_policy_per_type[pub_sub_type]
        except KeyError:
            self.memory_policy = QosDefaults.memory_policy_per_type[None]


USE_DEFAULT_QOS_DURABILITY = -1
NO_HISTORY = 0

_DEFAULT_STABLE_MATCH_WINDOW_SEC = 1.0
_MIN_MATCH_WAIT_SEC = 0.05


def _get_stable_match_count(
    condition: threading.Condition,
    accessor: Callable[[], int],
    timeout_sec: float,
    settle_time_sec: float,
) -> int:
    settle_time_sec = max(0.0, settle_time_sec)
    timeout_sec = max(timeout_sec, 0.0) if timeout_sec is not None else 0.0
    start_time = time.monotonic()
    timeout_point = start_time + timeout_sec
    wait_for_match = max(timeout_sec - settle_time_sec, _MIN_MATCH_WAIT_SEC)

    with condition:
        if not condition.wait_for(lambda: accessor() > 0, timeout=wait_for_match):
            return 0

        if settle_time_sec <= 0.0:
            return accessor()

        while True:
            previous = accessor()

            def _count_changed(prev=previous):
                return accessor() != prev

            if not condition.wait_for(_count_changed, timeout=settle_time_sec):
                return previous

            if timeout_point - time.monotonic() <= 0.0:
                return -1


class ServiceMatchingTimeoutError(RuntimeError):
    """Raised when DDS endpoints do not match within the requested timeout."""


class RequestPublishError(RuntimeError):
    """Raised when the request DataWriter fails to publish."""


# ---------------------------------------------------------------------------
# DDS participant lifecycle
#
# During Python interpreter shutdown, the garbage collector may destroy SWIG
# wrapper objects in an unpredictable order.  If a DomainParticipant is
# collected before its child entities (Publisher, Subscriber, …) the C++
# destructors access already-freed memory → access violation.
#
# To prevent this we:
#   1. Track every live _DomainParticipant via weak refs.
#   2. Register an atexit handler that calls _cleanup() on each participant
#      *before* the interpreter starts tearing down modules.
#   3. _cleanup() calls delete_contained_entities() + delete_participant()
#      and sets a flag so that child __del__ methods become safe no-ops.
# ---------------------------------------------------------------------------
_live_participants = []
_live_participants_lock = threading.Lock()


@atexit.register
def _cleanup_all_participants():
    """Explicitly destroy every DomainParticipant before interpreter shutdown
    so that SWIG C++ destructors run in the correct order."""
    with _live_participants_lock:
        for ref in reversed(_live_participants):
            p = ref()
            if p is not None:
                p._cleanup()
        _live_participants.clear()


def make_domain_participant(domain_id: int = 0,
                            recovery_mode: "NetworkRecoveryMode" = None):
    """Creates a new DDS Domain Participant that automatically cleans up internal objects on deletion

    :param domain_id: DDS domain_id, 0 by default
    :param recovery_mode: Network auto-recovery participation
        (see :class:`NetworkRecoveryMode`). Defaults to
        :attr:`NetworkRecoveryMode.ENV_VAR_CONTROLLED`, which honours the
        ``PROVIZIO_DDS_NETWORK_RECOVERY`` env variable (auto-recovery is on
        by default). When enabled, the underlying Fast-DDS participant is
        torn down and rebuilt on a confirmed network-interface address
        change; existing Publisher / Subscriber / Service handles created
        against this participant continue to work — their internal
        Fast-DDS objects are swapped under the user-held Python object.
        See ``network_recovery.py`` for details.
    :return: A wrapped DDS Domain Participant
    """

    if recovery_mode is None:
        recovery_mode = _network_recovery.NetworkRecoveryMode.ENV_VAR_CONTROLLED

    class _DomainParticipant:
        # Hardcoded — must match the default_fastdds_env_variable constant in
        # src/domain_participant.cpp. Fast-DDS 3.x renamed the variable from
        # FASTRTPS_DEFAULT_PROFILES_FILE to FASTDDS_DEFAULT_PROFILES_FILE and
        # removed the <fastrtps/...> extern that previously backed a runtime
        # sanity check, so both sides hardcode the 3.x value.
        xml_profiles_env_variable = "FASTDDS_DEFAULT_PROFILES_FILE"

        # Disable shared memory transport on Windows and macOS via environment
        # variable (the SWIG bindings do not expose setup_transports()).
        # Fast-DDS's bundled Boost.Interprocess has a known bug where shared
        # memory segments and named semaphores from a previous DDS participant
        # are not cleaned up promptly.  On Windows this causes assertion
        # failures; on macOS the default system-wide shared memory limits
        # (kern.sysv.shmmni=32) are quickly exhausted when creating
        # participants across multiple domain IDs, causing participant
        # creation to hang indefinitely.
        _disable_shm = sys.platform in ("win32", "darwin")

        def __init__(self, domain_id):
            self._cleaned_up = False
            self._domain_id = domain_id

            # Set before create_participant so Fast-DDS picks it up
            if _DomainParticipant._disable_shm:
                os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

            factory = DomainParticipantFactory.get_instance()
            # It's required so consequent get_default_participant_qos() respects XML profiles
            factory.load_profiles()

            self._participant_qos = DomainParticipantQos()
            factory.get_default_participant_qos(self._participant_qos)

            # Unless defined in the XML Profile, enable more reliable participants matching
            if (
                _DomainParticipant.xml_profiles_env_variable not in os.environ
                or not os.path.isfile(
                    os.environ[_DomainParticipant.xml_profiles_env_variable]
                )
            ):
                self._participant_qos.wire_protocol().builtin.discovery_config.initial_announcements.count = (
                    QosDefaults.num_initial_discovery_announcements
                )
                self._participant_qos.wire_protocol().builtin.discovery_config.initial_announcements.period = (
                    QosDefaults.initial_announcements_period
                )
                self._participant_qos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod = (
                    QosDefaults.lease_duration_announcement_period
                )

            self._participant = factory.create_participant(
                domain_id, self._participant_qos
            )
            if self._participant is None:
                # Fast-DDS create_participant returned None — typically a
                # malformed XML profile (FASTDDS_DEFAULT_PROFILES_FILE),
                # RLIMIT_NOFILE exhaustion, or memory pressure. Surface
                # the failure immediately rather than letting the half-
                # constructed _DomainParticipant escape and produce
                # opaque AttributeError / None-deref later when the
                # caller starts creating Publishers / Subscribers.
                # Mirrors the C++ ctor's runtime_error.
                raise RuntimeError(
                    "domain_participant: Fast-DDS create_participant returned None "
                    "(check FASTDDS_DEFAULT_PROFILES_FILE / system limits / logs)"
                )

            # Lifecycle lock: held by every operation that touches
            # self._participant. The reset path takes it for the entire
            # duration of teardown+rebuild; publish() / get_guid() etc.
            # take it briefly. The Python threading model uses a single
            # exclusive lock (no shared/exclusive RW lock in stdlib) — the
            # reset is rare and bounded, so the cost of fully serialising
            # publishes during reset is acceptable.
            self._lifecycle_lock = threading.RLock()
            # Monotonically increasing on every successful recreate; used
            # by endpoints to detect "I was built against a participant
            # that has since been destroyed" and skip stale operations.
            # Matches the C++ participant_generation semantics. Starts at 1
            # (0 reserved for 'never built').
            self._generation = 1

            # Registration mutex: held across both endpoint registration
            # AND the full reset sequence (mirrors the C++
            # domain_participant::registration_mutex). With this in place,
            # an endpoint cannot be created mid-reset (and end up built
            # against the about-to-be-destroyed participant); endpoints
            # registered before the reset are guaranteed to be in Phase 1's
            # live snapshot.
            self._registration_mutex = threading.Lock()

            # Endpoint registry for reset. Weak refs so natural endpoint
            # destruction ends registration; the participant doesn't keep
            # them alive.
            self._endpoints_lock = threading.Lock()
            self._endpoints = []

            self._register_type_mutex = threading.Lock()
            self._registered_types = dict()
            self._register_topic_mutex = threading.Lock()
            self._registered_topics = dict()

            # Network-recovery participation, resolved on construction (the
            # C++ side resolves the env var the same way — once per
            # process, cached).
            self._recovery_enabled = _network_recovery.resolve_network_recovery_enabled(recovery_mode)

            def _remove_ref(r):
                with _live_participants_lock:
                    try:
                        _live_participants.remove(r)
                    except ValueError:
                        pass  # Already removed by _cleanup_all_participants

            ref = weakref.ref(self, _remove_ref)
            with _live_participants_lock:
                _live_participants.append(ref)

        def _cleanup(self):
            """Deterministic cleanup: delete all C++ entities then the
            participant. Takes the lifecycle lock so it is mutually exclusive
            with any in-flight network-recovery reset — without this, a reset
            running Phase 2 (drain) without the lock would race with cleanup
            and double-free the Fast-DDS participant."""
            with self._lifecycle_lock:
                if self._cleaned_up:
                    return
                self._cleaned_up = True
                factory = DomainParticipantFactory.get_instance()
                if self._participant is not None:
                    self._participant.delete_contained_entities()
                    factory.delete_participant(self._participant)
                    self._participant = None
                    # Bump the generation so any in-flight endpoint teardown —
                    # an inline _teardown_state_for_reset racing this cleanup,
                    # or a deferred reaper thunk — observes the mismatch and
                    # skips re-deleting the entities delete_contained_entities
                    # just freed, rather than double-freeing them. Symmetric
                    # with the generation bump in _reset_hook_locked. (The C++
                    # side needs no equivalent: shared_ptr ownership guarantees
                    # the participant outlives every endpoint, so this runs
                    # only after each has torn itself down.)
                    self._generation += 1

        def __del__(self):
            self._cleanup()

        def fastdds_participant(self):
            return self._participant

        def register_type(self, pub_sub_type_instance):
            # Lifecycle lock keeps self._participant stable across the
            # Fast-DDS register_type call below — without it a concurrent
            # reset could free self._participant mid-call. The lock is an
            # RLock so internal callers (build_state during register or
            # reset) that already hold it can re-acquire safely.
            with self._lifecycle_lock:
                # After a failed network-recovery recreate, _participant is
                # None — surface a clear RuntimeError rather than an opaque
                # AttributeError on .register_type. Mirrors the C++ side's
                # null guards in register_type_locked / register_topic_locked.
                if self._participant is None:
                    raise RuntimeError(
                        "domain_participant: Fast-DDS participant is not available "
                        "(most likely a network-recovery recreate failed); see logs"
                    )
                with self._register_type_mutex:
                    type_support = self._registered_types.get(
                        pub_sub_type_instance.get_name(), None
                    )
                    if type_support is None:
                        type_support = TypeSupport(pub_sub_type_instance)
                        self._participant.register_type(type_support)
                        self._registered_types[pub_sub_type_instance.get_name()] = (
                            type_support
                        )
                    return type_support

        def register_topic(self, topic_name, pub_sub_type):
            # See register_type for the lifecycle_lock rationale.
            with self._lifecycle_lock, self._register_topic_mutex:
                # Same dead-participant guard as register_type — see comment
                # there. Surface a clean RuntimeError instead of an
                # AttributeError on self._participant.create_topic.
                if self._participant is None:
                    raise RuntimeError(
                        f"domain_participant: cannot register topic '{topic_name}' — "
                        f"the Fast-DDS participant is not available "
                        f"(most likely a network-recovery recreate failed); see logs"
                    )
                topic_info = self._registered_topics.get(topic_name)

                if topic_info:
                    # Ensure the type matches the already registered topic's type
                    existing_type_name = topic_info["type_support"].get_type_name()
                    requested_type_name = pub_sub_type().get_name()
                    if existing_type_name != requested_type_name:
                        raise RuntimeError(
                            f"Topic {topic_name} has been already registered, but with a different type (existing: '{existing_type_name}', requested: '{requested_type_name}')!"
                        )
                    topic_info["ref_count"] += 1
                    return topic_info["topic"]
                else:
                    # Register the type first
                    pub_sub_type_instance = pub_sub_type()
                    type_support = self.register_type(pub_sub_type_instance)

                    # Create the topic
                    topic_qos = TopicQos()
                    self.fastdds_participant().get_default_topic_qos(topic_qos)
                    new_topic = self.fastdds_participant().create_topic(
                        topic_name, pub_sub_type_instance.get_name(), topic_qos
                    )

                    self._registered_topics[topic_name] = {
                        "topic": new_topic,
                        "ref_count": 1,
                        "type_support": type_support,
                    }
                    return new_topic

        def unregister_topic(self, topic_name):
            # See register_type for the lifecycle_lock rationale.
            with self._lifecycle_lock, self._register_topic_mutex:
                topic_info = self._registered_topics.get(topic_name)
                if topic_info:
                    topic_info["ref_count"] -= 1
                    if topic_info["ref_count"] <= 0:
                        if not self._cleaned_up and self._participant is not None:
                            self.fastdds_participant().delete_topic(topic_info["topic"])
                        del self._registered_topics[topic_name]

        # --- Network auto-recovery support ---------------------------------

        def participant_generation(self):
            """Returns the current participant generation. Increments by 1
            on every successful reset-and-rebuild."""
            return self._generation

        def lifecycle_lock(self):
            """Internal accessor — returns the RLock that serializes
            publish() / get_guid() / reset against each other. Endpoint
            classes acquire it briefly; the reset path acquires it for the
            full reset duration."""
            return self._lifecycle_lock

        def _register_endpoint(self, endpoint, build_state_fn):
            """Build the endpoint's Fast-DDS state AND add it to the
            recovery registry atomically, under :attr:`_registration_mutex`.

            Mirrors the C++ ``domain_participant::register_endpoint`` —
            both steps must happen under the same lock instance so a
            concurrent reset cannot observe a half-built endpoint or miss
            a freshly-built one. The endpoint passes a callable
            ``build_state_fn(fastdds_participant)`` so the build runs
            against the *current* Fast-DDS participant (which may have
            been replaced if the reset finished while we were waiting on
            the registration mutex).

            On success, the endpoint is appended to the registry. On
            build failure (exception), the exception propagates and the
            registry is not modified.
            """
            with self._registration_mutex:
                with self._lifecycle_lock:
                    # Same dead-participant guard as register_type — see
                    # comment there. The build_state callback would
                    # otherwise dereference a None participant and raise
                    # an opaque AttributeError mid-build.
                    if self._participant is None:
                        raise RuntimeError(
                            "domain_participant: cannot register endpoint — "
                            "the Fast-DDS participant is not available "
                            "(most likely a network-recovery recreate failed); see logs"
                        )
                    build_state_fn(self._participant)
                with self._endpoints_lock:
                    self._endpoints = [r for r in self._endpoints if r() is not None]
                    self._endpoints.append(weakref.ref(endpoint))

        def _deregister_endpoint(self, endpoint):
            with self._endpoints_lock:
                self._endpoints = [
                    r for r in self._endpoints if r() is not None and r() is not endpoint
                ]

        def _reset_hook(self, old_snapshot, new_snapshot):
            """Invoked by the network-recovery coordinator when a confirmed
            network-interface address change is observed. Mirrors the C++
            ``domain_participant::trigger_network_recovery_reset``:

              Phase 0: take :attr:`_registration_mutex` for the whole reset
                so no new endpoint can be registered while we tear down +
                rebuild. (Same constraint as C++: callbacks must NOT call
                ``Publisher`` / ``Subscriber`` / ``Service`` constructors —
                that would deadlock against this serialization.)
              Phase 1: snapshot endpoints under the registry lock.
              Phase 2: detach listener drains WITHOUT the lifecycle lock —
                lets in-flight callbacks complete and bail out.
              Phase 3: take the lifecycle lock exclusively.
              Phase 4: tear down each endpoint's Fast-DDS state, then the
                participant itself.
              Phase 5: create a new participant, re-register types, rebuild
                each endpoint.
            """
            # Phase 0: serialise against new registrations.
            with self._registration_mutex:
                self._reset_hook_locked(old_snapshot, new_snapshot)

        def _reset_hook_locked(self, old_snapshot, new_snapshot):
            # Phase 1: snapshot.
            with self._endpoints_lock:
                live = [(r, r()) for r in self._endpoints]
                live = [(r, e) for (r, e) in live if e is not None]

            # Phase 2: detach listeners (no lock held — callbacks can
            # complete by re-entering provizio APIs that need the lifecycle
            # lock shared).
            for (_, endpoint) in live:
                try:
                    endpoint._detach_for_reset()
                except Exception as ex:
                    _network_recovery._emit_log(
                        _network_recovery.LogLevel.ERROR,
                        f"endpoint detach failed during reset: {ex}",
                    )

            # Phase 3: take the lifecycle lock for the whole reset.
            with self._lifecycle_lock:
                if self._cleaned_up:
                    return

                # Phase 4: tear down each endpoint's Fast-DDS state.
                for (_, endpoint) in live:
                    try:
                        endpoint._teardown_state_for_reset()
                    except Exception as ex:
                        _network_recovery._emit_log(
                            _network_recovery.LogLevel.ERROR,
                            f"endpoint teardown failed during reset: {ex}",
                        )

                # Drop topic handles — they reference the OLD participant.
                # User-held _TopicHandle weakrefs will get fresh topics on
                # rebuild via the existing register_topic path.
                with self._register_topic_mutex:
                    self._registered_topics.clear()

                # Destroy the old participant.
                factory = DomainParticipantFactory.get_instance()
                if self._participant is not None:
                    self._participant.delete_contained_entities()
                    factory.delete_participant(self._participant)
                    self._participant = None

                # Refresh Fast-DDS's interface cache BEFORE recreating —
                # this is the whole point of the auto-recovery reset
                # (without the refresh, the new participant binds to the
                # same stale interfaces as the old one). Mirrors the C++
                # domain_participant::trigger_network_recovery_reset path.
                _network_recovery.refresh_fastdds_interface_cache()

                # Recreate with the same QoS.
                self._participant = factory.create_participant(self._domain_id, self._participant_qos)
                if self._participant is None:
                    _network_recovery._emit_log(
                        _network_recovery.LogLevel.ERROR,
                        f"failed to recreate participant on domain {self._domain_id}; "
                        f"endpoints left in torn-down state",
                    )
                    # Bump generation anyway so any racing teardown observes the mismatch.
                    self._generation += 1
                    return

                self._generation += 1

                # Re-register all known types against the new participant.
                with self._register_type_mutex:
                    for type_name, type_support in self._registered_types.items():
                        self._participant.register_type(type_support)

                # Phase 5: rebuild each endpoint.
                for (_, endpoint) in live:
                    try:
                        endpoint._rebuild_state_after_reset(self._participant)
                    except Exception as ex:
                        _network_recovery._emit_log(
                            _network_recovery.LogLevel.ERROR,
                            f"endpoint rebuild failed during reset: {ex}",
                        )

    participant = _DomainParticipant(domain_id)
    if participant._recovery_enabled:
        _network_recovery._NetworkRecoveryCoordinator.instance().register_participant(
            participant, participant._reset_hook
        )
    return participant


class _TopicHandle:
    """Holds a topic registration against the parent participant.

    On a network-recovery reset the underlying topic is destroyed together
    with the participant, so the Publisher / Subscriber subclasses fetch a
    fresh handle from the new participant via :meth:`_acquire_topic`. The
    caller is responsible for releasing the previous handle (via
    ``unregister_topic``) before re-acquiring.
    """

    def __init__(self, domain_participant, topic_name, pub_sub_type):
        self._participant = domain_participant
        self._topic_name = topic_name
        self._pub_sub_type = pub_sub_type
        self._topic = None  # Set by _acquire_topic on first build.

    def _acquire_topic(self):
        """Register / look up the topic on the current participant. Call
        from _build_state under the lifecycle lock."""
        self._topic = self._participant.register_topic(self._topic_name, self._pub_sub_type)

    def _release_topic(self):
        """Drop the topic registration. Idempotent."""
        if self._topic is not None:
            self._participant.unregister_topic(self._topic_name)
            self._topic = None

    def __del__(self):
        try:
            self._release_topic()
        except Exception:
            # Destruction order during interpreter shutdown can leave the
            # participant or its registry already torn down; swallow.
            pass


class Publisher(_TopicHandle):
    """Provides publishing functionality for a DDS data type and topic name specified when constructing.

    Participates in network auto-recovery: when the parent participant's
    Fast-DDS participant is recreated on a confirmed network change, this
    Publisher's underlying DataWriter is rebuilt automatically and the
    user-held Python handle remains valid. The :attr:`on_has_subscriber_changed_function`
    callback persists across the reset.

    Note: the on_has_subscriber_changed callback must NOT block indefinitely
    nor create new endpoints (``Publisher`` / ``Subscriber`` / ``Service``).
    Creating new endpoints from inside a callback during a network-recovery
    reset would deadlock — same restriction as the C++ side; see
    :file:`include/provizio/dds/publisher.h`.
    """

    class _WriterListener(DataWriterListener):
        # The listener OWNS the drain and matched-count state — see
        # Subscriber._ReaderListener for the full rationale. In short, this
        # keeps the Fast-DDS callback thread from holding a strong reference
        # to the Publisher, so the Publisher can't be destroyed from inside a
        # callback (which would self-join in delete_datawriter →
        # DataWriterImpl::stop()). The Publisher is still resolved (briefly,
        # via weakref) when there is a user on_has_subscriber_changed
        # callback, because that callback's API passes the Publisher to user
        # code; callers are expected to hold their own reference to the
        # Publisher across its lifetime (the C++ side has the same
        # contract).
        def __init__(self, publisher, on_has_subscriber_changed_function):
            super().__init__()
            self._publisher = weakref.ref(publisher)
            self._on_has_subscriber_changed_function = (
                on_has_subscriber_changed_function
            )
            self._drain = _network_recovery.ListenerDrain()
            self._num_matched_cv = threading.Condition()
            self._num_matched_subscribers = 0
            try:
                sig = inspect.signature(self._on_has_subscriber_changed_function)
                self._on_has_subscriber_changed_takes_guid = len(sig.parameters) == 3
            except (ValueError, TypeError):
                self._on_has_subscriber_changed_takes_guid = False

        def __del__(self):
            del self._publisher
            del self._on_has_subscriber_changed_function

        def _update_num_matched_subscribers(self, count):
            with self._num_matched_cv:
                self._num_matched_subscribers = count
                self._num_matched_cv.notify_all()

        def on_publication_matched(self, _, info):
            # Drain-aware: if a reset is in progress, bail out without
            # running user code (which may re-enter provizio APIs that need
            # the lifecycle lock). The drain counter increments on enter()
            # and decrements on leave() — the reset's detach_and_drain()
            # waits for the counter to reach 0 before taking the lifecycle
            # lock for teardown.
            #
            # The whole callback runs inside _fastdds_callback_scope: if the
            # transient strong Publisher reference resolved below (for the
            # user callback) turns out to be the last one, the resulting
            # Publisher.__del__ defers its Fast-DDS teardown off this thread
            # instead of self-joining in delete_datawriter.
            publisher = None
            with _fastdds_callback_scope():
                try:
                    with self._drain.scope() as should_run:
                        if not should_run:
                            return

                        self._update_num_matched_subscribers(info.current_count)

                        if not self._on_has_subscriber_changed_function:
                            return

                        # Resolve the Publisher only when there is a user
                        # callback whose API hands it the Publisher.
                        publisher = self._publisher()
                        if publisher is None:
                            return

                        if self._on_has_subscriber_changed_takes_guid:
                            # Copy the GUID before handing it to user code.
                            # InstanceHandle_t::get_guid() is SWIG-aliased to
                            # ``operator const GUID_t&`` (see fast_dds_python's
                            # InstanceHandle.i), so the return is a Python
                            # wrapper holding a raw pointer into the
                            # InstanceHandle_t bytes inside ``info``. ``info``
                            # only lives until this listener callback returns,
                            # at which point a user callback that stored the
                            # GUID would be reading freed memory. Constructing
                            # a fresh GUID_t invokes the C++ copy constructor
                            # so the value the user gets is Python-owned.
                            matched_guid = GUID_t(info.last_subscription_handle.get_guid())
                            if info.current_count_change > 0:
                                self._on_has_subscriber_changed_function(publisher, True, matched_guid)
                            elif info.current_count_change < 0:
                                self._on_has_subscriber_changed_function(publisher, False, matched_guid)
                        else:
                            if (
                                info.current_count > 0
                                and info.current_count_change == info.current_count
                            ):
                                # Just matched the first subscriber
                                self._on_has_subscriber_changed_function(publisher, True)
                            elif info.current_count == 0 and info.current_count_change < 0:
                                # Just unmatched the last subscriber
                                self._on_has_subscriber_changed_function(publisher, False)
                finally:
                    # Drop the transient reference now — after the drain scope
                    # has exited (so a __del__ triggered here will not dead-wait
                    # in detach_and_drain on our own still-in-flight callback)
                    # but while still inside the callback scope (so that
                    # __del__ defers its Fast-DDS teardown off this thread).
                    publisher = None

    def __init__(
        self,
        domain_participant: object,
        topic_name: str,
        pub_sub_type: TopicDataType,
        on_has_subscriber_changed_function: Optional[
            Callable[[Publisher, bool], Any]
        ] = None,
        reliability_kind: Optional[Any] = None,
        history_depth: int = USE_DEFAULT_QOS_DURABILITY,
    ):
        """Constructs a DDS Publisher

        :param domain_participant: A DDS Domain Participant wrapper object, as created by provizio_dds.make_domain_participant
        :param str topic_name: A string DDS Topic name
        :param pub_sub_type: The DDS PubSub Type to be published, f.e. provizio_dds.StringPubSubType
        :param on_has_subscriber_changed_function: Optional, a function to be invoked on matching first / unmatching last subscriber, takes two arguments: a Publisher and a bool: True when the first subscriber is matched, False when the last subscriber is unmatched; Note: called from a background Thread
        :param reliability_kind: Optional, a DDS data writer reliability kind to be used: either BEST_EFFORT_RELIABILITY_QOS or RELIABLE_RELIABILITY_QOS; if not specified, QosDefaults for pub_sub_type will be used
        :param int history_depth: Controls durability QoS: -1 keeps defaults, 0 forces VOLATILE (no history), positive values enable TRANSIENT_LOCAL with KEEP_LAST of the given depth
        """

        super().__init__(domain_participant, topic_name, pub_sub_type)

        qos_defaults = QosDefaults(pub_sub_type)
        if reliability_kind is None:
            reliability_kind = qos_defaults.datawriter_reliability_kind

        # Captured parameters for replay on reset.
        self._captured_reliability_kind = reliability_kind
        self._captured_history_depth = history_depth
        self._captured_qos_defaults = qos_defaults

        # Build state once. Survives reset via _rebuild_state_after_reset.
        # The participant's _register_endpoint does the build AND adds us
        # to the registry under _registration_mutex atomically, so a
        # concurrent reset cannot observe a half-built or stale endpoint.
        self._publisher = None
        self._writer = None
        # The listener owns the drain + matched-count state (see
        # _WriterListener) — the Publisher reaches them through
        # self._listener rather than holding its own copies.
        self._listener = Publisher._WriterListener(self, on_has_subscriber_changed_function)
        self._built_against_generation = 0
        self._participant._register_endpoint(
            self,
            lambda fastdds_participant: self._build_state(fastdds_participant),
        )

    def _build_state(self, fastdds_participant):
        """(Re)create the Fast-DDS Publisher + DataWriter against
        @c fastdds_participant. Caller holds the lifecycle lock."""
        # Topic must be (re)registered against the new participant.
        self._release_topic()
        self._acquire_topic()

        publisher_qos = PublisherQos()
        fastdds_participant.get_default_publisher_qos(publisher_qos)
        self._publisher = fastdds_participant.create_publisher(publisher_qos)
        if self._publisher is None:
            raise RuntimeError(
                f"Publisher: create_publisher returned None for topic {self._topic_name}"
            )

        writer_qos = DataWriterQos()
        self._publisher.get_default_datawriter_qos(writer_qos)
        writer_qos.reliability().kind = self._captured_reliability_kind
        writer_qos.endpoint().history_memory_policy = self._captured_qos_defaults.memory_policy
        if sys.platform in ("win32", "darwin"):
            writer_qos.data_sharing().off()
        if self._captured_history_depth == USE_DEFAULT_QOS_DURABILITY:
            pass
        elif self._captured_history_depth == NO_HISTORY:
            writer_qos.durability().kind = VOLATILE_DURABILITY_QOS
        elif self._captured_history_depth > 0:
            writer_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS
            writer_qos.history().kind = KEEP_LAST_HISTORY_QOS
            writer_qos.history().depth = self._captured_history_depth

        # Prime the listener BEFORE create_datawriter attaches it: Fast-DDS
        # can fire on_publication_matched on an internal thread before
        # create_datawriter returns, and we must observe those callbacks
        # rather than clobber them. Zeroing the count afterwards (the
        # previous order) raced the first match callback to zero, which
        # Fast-DDS would never re-fire because the underlying matched
        # state never changed again — leaving get_num_matched_subscribers
        # stuck at 0 forever and request/response clients timing out.
        with self._listener._num_matched_cv:
            self._listener._num_matched_subscribers = 0
            self._listener._num_matched_cv.notify_all()
        self._listener._drain.reattach()

        self._writer = self._publisher.create_datawriter(self._topic, writer_qos, self._listener)
        if self._writer is None:
            fastdds_participant.delete_publisher(self._publisher)
            self._publisher = None
            self._release_topic()
            raise RuntimeError(
                f"Publisher: create_datawriter returned None for topic {self._topic_name}"
            )

        self._built_against_generation = self._participant.participant_generation()

    def _teardown_state_for_reset(self):
        """Tear down Fast-DDS state without releasing user-visible
        attributes. Caller holds the lifecycle lock. Generation check
        protects against use-after-free in the rare 'we missed the reset
        snapshot' path."""
        if self._built_against_generation == 0:
            return
        if self._built_against_generation != self._participant.participant_generation():
            # Stale — the participant we built against is already gone.
            # Just clear our handles; delete_contained_entities already
            # freed the underlying Fast-DDS objects.
            self._writer = None
            self._publisher = None
            self._topic = None
            self._built_against_generation = 0
            return
        if self._writer is not None and self._publisher is not None:
            self._publisher.delete_datawriter(self._writer)
        self._writer = None
        if self._publisher is not None:
            self._participant.fastdds_participant().delete_publisher(self._publisher)
        self._publisher = None
        # Drop the topic registration properly: decrements the participant's
        # _registered_topics ref_count (and delete_topic when the count hits
        # zero) instead of just nulling our reference. Without this, the
        # destruction path leaks the topic because _TopicHandle.__del__ only
        # unregisters when self._topic is still non-None.
        self._release_topic()
        self._built_against_generation = 0

    def _defer_teardown_off_thread(self):
        """Tear down Fast-DDS state on the reaper thread instead of inline.

        Used when __del__ fires on a Fast-DDS listener thread, where
        delete_datawriter would self-join. Captures the handles (never
        ``self`` — that would resurrect this object mid-__del__) and deletes
        them on the reaper thread, which re-checks the participant generation
        under the lifecycle lock first: if a reset has recreated the
        participant in the meantime, delete_contained_entities already freed
        these entities and the reaper skips them."""
        if self._built_against_generation == 0:
            return
        writer = self._writer
        publisher = self._publisher
        had_topic = self._topic is not None
        topic_name = self._topic_name
        gen = self._built_against_generation
        drain = self._listener._drain
        participant_ref = weakref.ref(self._participant)
        # Release our handles now so this object is reclaimed immediately; the
        # reaper owns the captured handles until the deletes run.
        self._writer = None
        self._publisher = None
        self._topic = None
        self._built_against_generation = 0

        def _delete_off_thread():
            participant = participant_ref()
            if participant is None:
                return
            # Quiesce in-flight callbacks here, not on the Fast-DDS thread —
            # the callback that triggered this __del__ has already left the
            # drain scope, so this does not wait on ourselves.
            drain.detach_and_drain()
            with participant.lifecycle_lock():
                if participant.participant_generation() != gen:
                    return
                if writer is not None and publisher is not None:
                    publisher.delete_datawriter(writer)
                if publisher is not None:
                    participant.fastdds_participant().delete_publisher(publisher)
                if had_topic:
                    participant.unregister_topic(topic_name)

        _network_recovery.submit_off_thread(_delete_off_thread)

    def _rebuild_state_after_reset(self, new_fastdds_participant):
        self._build_state(new_fastdds_participant)

    def _detach_for_reset(self):
        """Drain in-flight listener callbacks. Called BEFORE the lifecycle
        lock is acquired for teardown so re-entrant callbacks don't
        deadlock."""
        self._listener._drain.detach_and_drain()

    def __del__(self):
        # Best-effort cleanup; the participant's atexit hook handles the
        # case where it was already torn down.
        try:
            participant = self._participant
        except AttributeError:
            return
        if participant is None:
            return
        if not getattr(participant, "_cleaned_up", True):
            try:
                participant._deregister_endpoint(self)
            except Exception:
                pass
            if _on_fastdds_callback_thread():
                # On a Fast-DDS listener thread (a reference dropped from
                # inside on_publication_matched): detach_and_drain would
                # dead-wait on our own in-flight callback and delete_datawriter
                # would self-join this thread, so defer the whole teardown to
                # the reaper thread.
                try:
                    self._defer_teardown_off_thread()
                except Exception:
                    pass
            else:
                # Detach the listener drain BEFORE taking the lifecycle lock
                # and calling delete_datawriter, so a reset / re-entrant
                # callback can't deadlock against the lifecycle lock or
                # Fast-DDS's internal callback-drain.
                try:
                    self._listener._drain.detach_and_drain()
                except Exception:
                    pass
                try:
                    with participant.lifecycle_lock():
                        self._teardown_state_for_reset()
                except Exception:
                    pass
        super().__del__()

    def get_guid(self):
        """Return a stable Python-owned copy of the underlying DataWriter's GUID.

        Fast-DDS' ``DataWriter::guid()`` returns by const-reference. Through SWIG that
        becomes a Python wrapper holding a raw pointer into the DataWriter — if the
        DataWriter is later destroyed (e.g. by a network-recovery reset), reading
        that wrapper is use-after-free. Constructing a new ``GUID_t`` from the
        reference invokes the C++ copy constructor so the returned object lives
        independently of the writer.
        """
        with self._participant.lifecycle_lock():
            if (
                self._writer is None
                or self._built_against_generation != self._participant.participant_generation()
            ):
                return GUID_t.unknown()
            return GUID_t(self._writer.guid())

    def publish(self, data: object, params: WriteParams = None):
        """Publishes DDS data

        :param data: actual data (not Pub Sub Type), f.e. provizio_dds.String
        :param params: optional WriteParams to control the write operation
        :return: True if published successfully, and False otherwise
        """
        with self._participant.lifecycle_lock():
            if (
                self._writer is None
                or self._built_against_generation != self._participant.participant_generation()
            ):
                # Either never built, or a network-recovery reset rebuilt
                # the participant without rebuilding us. Surface a clean
                # failure rather than use-after-free.
                return False
            # Fast-DDS 3.x: DataWriter.write() returns a ReturnCode_t integer
            # (RETCODE_OK == 0) rather than the bool that Fast-DDS 2.x produced
            # via the Python bindings. Compare explicitly so a successful write
            # maps to True and any other return code to False — without this,
            # `RETCODE_OK == 0` falsifies as Python bool and every publish would
            # be reported as a failure.
            if params:
                return self._writer.write(data, params) == RETCODE_OK
            return self._writer.write(data) == RETCODE_OK

    def get_num_matched_subscribers(
        self, timeout_sec: float, settle_time_sec: float
    ) -> int:
        """Return the stable number of matched subscribers or -1 if unstable until timeout."""

        listener = self._listener
        return _get_stable_match_count(
            listener._num_matched_cv,
            lambda: listener._num_matched_subscribers,
            timeout_sec,
            settle_time_sec,
        )

class Subscriber(_TopicHandle):
    """Provides subscription functionality for a DDS data type and topic name specified when constructing.

    Participates in network auto-recovery: see :class:`Publisher` for the
    full set of restrictions on callbacks. In short: the on_data /
    on_has_publisher_changed callbacks MUST NOT block indefinitely and MUST
    NOT create new endpoints (``Publisher`` / ``Subscriber`` / ``Service``)
    — doing the latter during a network-recovery reset would deadlock.
    """

    class _ReaderListener(DataReaderListener):
        # The listener OWNS the drain and the matched-count state (cv +
        # count), rather than reaching back into the Subscriber for them.
        # This is critical for deadlock safety: Fast-DDS invokes these
        # callbacks on its own reception thread, and if the callback held a
        # strong reference to the Subscriber, a concurrent `del subscriber`
        # on another thread could leave the reception thread as the last
        # owner — so the Subscriber's __del__ (and its delete_datareader →
        # DataReaderImpl::stop()) would run INLINE on the reception thread
        # and self-join (stop() waits for the very thread executing the
        # callback). Keeping only the drain + match state here (small
        # objects that don't reference the Subscriber back) means the
        # callbacks never own the Subscriber, so its destruction can never
        # be triggered from within a callback. Mirrors the C++ side, where
        # this state lives on data_reader_listener, not the handle.
        def __init__(self, data_type, on_data_function, on_has_publisher_changed_function):
            super().__init__()
            self._data_type = data_type
            self._on_data_function = on_data_function
            self._on_has_publisher_changed_function = on_has_publisher_changed_function
            self._drain = _network_recovery.ListenerDrain()
            self._num_matched_cv = threading.Condition()
            self._num_matched_publishers = 0
            try:
                sig = inspect.signature(self._on_data_function)
                self._on_data_takes_info = len(sig.parameters) == 2
            except (ValueError, TypeError):
                self._on_data_takes_info = False

        def __del__(self):
            del self._data_type
            del self._on_data_function
            del self._on_has_publisher_changed_function

        def _update_num_matched_publishers(self, count):
            with self._num_matched_cv:
                self._num_matched_publishers = count
                self._num_matched_cv.notify_all()

        def on_data_available(self, reader):
            # Drain-aware: a reset's detach_and_drain waits for the
            # in-flight counter to reach 0 before taking the lifecycle
            # lock, so a callback re-entering provizio APIs can finish. The
            # callback scope marks this Fast-DDS thread so any endpoint
            # __del__ triggered here (e.g. GC of an unreferenced endpoint)
            # defers its Fast-DDS teardown off this thread.
            with _fastdds_callback_scope(), self._drain.scope() as should_run:
                if not should_run:
                    return
                while True:
                    info = SampleInfo()
                    data = self._data_type()
                    if reader.take_next_sample(data, info) != RETCODE_OK:
                        break
                    if not info.valid_data:
                        continue
                    if self._on_data_takes_info:
                        self._on_data_function(data, info)
                    else:
                        self._on_data_function(data)

        def on_subscription_matched(self, _, info):
            with _fastdds_callback_scope(), self._drain.scope() as should_run:
                if not should_run:
                    return
                self._update_num_matched_publishers(info.current_count)

                if self._on_has_publisher_changed_function:
                    if (
                        info.current_count > 0
                        and info.current_count_change == info.current_count
                    ):
                        # Just matched the first publisher
                        self._on_has_publisher_changed_function(True)
                    elif info.current_count == 0 and info.current_count_change < 0:
                        # Just unmatched the last publisher
                        self._on_has_publisher_changed_function(False)

    def __init__(
        self,
        domain_participant: object,
        topic_name: str,
        pub_sub_type: TopicDataType,
        data_type: object,
        on_data_function: Callable,
        on_has_publisher_changed_function: Optional[Callable[[bool], Any]] = None,
        reliability_kind: Optional[Any] = None,
        max_history_depth: int = USE_DEFAULT_QOS_DURABILITY,
    ):
        """Constructs a DDS Subscriber

        :param domain_participant: A DDS Domain Participant wrapper object, as created by provizio_dds.make_domain_participant
        :param str topic_name: A string DDS Topic name
        :param pub_sub_type: The DDS PubSub Type to be received, f.e. provizio_dds.StringPubSubType
        :param data_type: The DDS Data Type to be received, f.e. provizio_dds.String
        :param on_data_function: A function to be invoked on receiving published data. It can take one argument (the data) or two arguments (data and a SampleInfo object). Note: called from a background Thread
        :param on_has_publisher_changed_function: Optional, a function to be invoked on matching first / unmatching last publisher, takes a single bool argument: True when the first publisher is matched, False when the last publisher is unmatched; Note: called from a background Thread
        :param reliability_kind: Optional, a DDS data reader reliability kind to be used: either BEST_EFFORT_RELIABILITY_QOS or RELIABLE_RELIABILITY_QOS; if not specified, QosDefaults for pub_sub_type will be used
        :param int max_history_depth: Controls durability QoS: -1 keeps defaults, 0 forces VOLATILE (no history), positive values enable TRANSIENT_LOCAL with KEEP_LAST of the given depth
        """
        super().__init__(domain_participant, topic_name, pub_sub_type)

        qos_defaults = QosDefaults(pub_sub_type)
        if reliability_kind is None:
            reliability_kind = qos_defaults.datareader_reliability_kind

        self._captured_data_type = data_type
        self._captured_reliability_kind = reliability_kind
        self._captured_max_history_depth = max_history_depth
        self._captured_qos_defaults = qos_defaults

        self._subscriber = None
        self._reader = None
        # The listener owns the drain and matched-count state (see
        # _ReaderListener for why) — the Subscriber reaches them through
        # self._listener rather than holding its own copies.
        self._listener = Subscriber._ReaderListener(
            data_type, on_data_function, on_has_publisher_changed_function
        )
        self._built_against_generation = 0
        # Atomic build + register — see Publisher.__init__ for rationale.
        self._participant._register_endpoint(
            self,
            lambda fastdds_participant: self._build_state(fastdds_participant),
        )

    def _build_state(self, fastdds_participant):
        """(Re)create the Fast-DDS Subscriber + DataReader against
        @c fastdds_participant. Caller holds the lifecycle lock."""
        self._release_topic()
        self._acquire_topic()

        subscriber_qos = SubscriberQos()
        fastdds_participant.get_default_subscriber_qos(subscriber_qos)
        self._subscriber = fastdds_participant.create_subscriber(subscriber_qos)
        if self._subscriber is None:
            raise RuntimeError(
                f"Subscriber: create_subscriber returned None for topic {self._topic_name}"
            )

        reader_qos = DataReaderQos()
        self._subscriber.get_default_datareader_qos(reader_qos)
        reader_qos.reliability().kind = self._captured_reliability_kind
        reader_qos.endpoint().history_memory_policy = self._captured_qos_defaults.memory_policy
        if sys.platform in ("win32", "darwin"):
            reader_qos.data_sharing().off()
        if self._captured_max_history_depth == USE_DEFAULT_QOS_DURABILITY:
            pass
        elif self._captured_max_history_depth == NO_HISTORY:
            reader_qos.durability().kind = VOLATILE_DURABILITY_QOS
        elif self._captured_max_history_depth > 0:
            reader_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS
            reader_qos.history().kind = KEEP_LAST_HISTORY_QOS
            reader_qos.history().depth = self._captured_max_history_depth

        # Prime the listener BEFORE create_datareader attaches it: Fast-DDS
        # can fire on_subscription_matched on an internal thread before
        # create_datareader returns, and we must observe those callbacks
        # rather than clobber them. Zeroing the count afterwards (the
        # previous order) raced the first match callback to zero, which
        # Fast-DDS would never re-fire because the underlying matched
        # state never changed again — leaving get_num_matched_publishers
        # stuck at 0 forever and request/response clients timing out.
        with self._listener._num_matched_cv:
            self._listener._num_matched_publishers = 0
            self._listener._num_matched_cv.notify_all()
        self._listener._drain.reattach()

        self._reader = self._subscriber.create_datareader(self._topic, reader_qos, self._listener)
        if self._reader is None:
            fastdds_participant.delete_subscriber(self._subscriber)
            self._subscriber = None
            self._release_topic()
            raise RuntimeError(
                f"Subscriber: create_datareader returned None for topic {self._topic_name}"
            )

        self._built_against_generation = self._participant.participant_generation()

    def _teardown_state_for_reset(self):
        """Tear down Fast-DDS state. See Publisher._teardown_state_for_reset."""
        if self._built_against_generation == 0:
            return
        if self._built_against_generation != self._participant.participant_generation():
            self._reader = None
            self._subscriber = None
            self._topic = None
            self._built_against_generation = 0
            return
        if self._reader is not None and self._subscriber is not None:
            self._subscriber.delete_datareader(self._reader)
        self._reader = None
        if self._subscriber is not None:
            self._participant.fastdds_participant().delete_subscriber(self._subscriber)
        self._subscriber = None
        # See Publisher._teardown_state_for_reset — properly drop the topic
        # registration rather than just nulling our reference.
        self._release_topic()
        self._built_against_generation = 0

    def _defer_teardown_off_thread(self):
        """Tear down Fast-DDS state on the reaper thread. See
        Publisher._defer_teardown_off_thread for the full rationale."""
        if self._built_against_generation == 0:
            return
        reader = self._reader
        subscriber = self._subscriber
        had_topic = self._topic is not None
        topic_name = self._topic_name
        gen = self._built_against_generation
        drain = self._listener._drain
        participant_ref = weakref.ref(self._participant)
        self._reader = None
        self._subscriber = None
        self._topic = None
        self._built_against_generation = 0

        def _delete_off_thread():
            participant = participant_ref()
            if participant is None:
                return
            drain.detach_and_drain()
            with participant.lifecycle_lock():
                if participant.participant_generation() != gen:
                    return
                if reader is not None and subscriber is not None:
                    subscriber.delete_datareader(reader)
                if subscriber is not None:
                    participant.fastdds_participant().delete_subscriber(subscriber)
                if had_topic:
                    participant.unregister_topic(topic_name)

        _network_recovery.submit_off_thread(_delete_off_thread)

    def _rebuild_state_after_reset(self, new_fastdds_participant):
        self._build_state(new_fastdds_participant)

    def _detach_for_reset(self):
        self._listener._drain.detach_and_drain()

    def __del__(self):
        try:
            participant = self._participant
        except AttributeError:
            return
        if participant is None:
            return
        if not getattr(participant, "_cleaned_up", True):
            try:
                participant._deregister_endpoint(self)
            except Exception:
                pass
            if _on_fastdds_callback_thread():
                # On a Fast-DDS listener thread — defer the teardown off this
                # thread (delete_datareader would self-join it). The listener
                # no longer holds a strong Subscriber ref, so this normally
                # cannot happen on the reception thread; the guard also covers
                # a Subscriber GC'd from inside any Fast-DDS callback.
                try:
                    self._defer_teardown_off_thread()
                except Exception:
                    pass
            else:
                # Detach the listener drain before delete_datareader so a reset
                # / re-entrant callback can't deadlock against the lifecycle
                # lock.
                try:
                    self._listener._drain.detach_and_drain()
                except Exception:
                    pass
                try:
                    with participant.lifecycle_lock():
                        self._teardown_state_for_reset()
                except Exception:
                    pass
        super().__del__()

    def get_guid(self):
        """Return a stable Python-owned copy of the underlying DataReader's GUID.

        Fast-DDS' ``DataReader::guid()`` returns by const-reference. Through SWIG
        that becomes a Python wrapper holding a raw pointer into the DataReader — if
        the DataReader is later destroyed (e.g. by a network-recovery reset),
        reading that wrapper is use-after-free. Constructing a new ``GUID_t`` from
        the reference invokes the C++ copy constructor so the returned object lives
        independently of the reader.
        """
        with self._participant.lifecycle_lock():
            if (
                self._reader is None
                or self._built_against_generation != self._participant.participant_generation()
            ):
                return GUID_t.unknown()
            return GUID_t(self._reader.guid())

    def get_num_matched_publishers(
        self, timeout_sec: float, settle_time_sec: float
    ) -> int:
        """Return the stable number of matched publishers or -1 if unstable until timeout."""

        listener = self._listener
        return _get_stable_match_count(
            listener._num_matched_cv,
            lambda: listener._num_matched_publishers,
            timeout_sec,
            settle_time_sec,
        )


async def request(
    domain_participant: object,
    request_pub_sub_type,
    response_pub_sub_type,
    response_data_type,
    request_data,
    request_topic_name: str = None,
    response_topic_name: str = None,
    service_name: str = None,
    stable_matches_period_sec: float = _DEFAULT_STABLE_MATCH_WINDOW_SEC,
    service_match_timeout_sec: float = 0.0,
):
    """Send a request and await the response.

    One of (request_topic_name, response_topic_name) or service_name must be provided.
    The client uses a volatile (default durability) request Publisher and a reliable response Subscriber.
    The service uses a transient-local response Publisher (depth 10) by default for robust delivery.
    Before publishing the first request, discovery readiness is awaited using a graph-based stable match check
    to avoid races right after endpoint matching.

    :param domain_participant: Domain participant wrapper created by ``make_domain_participant``
    :param request_pub_sub_type: PubSub type for the request
    :param response_pub_sub_type: PubSub type for the response
    :param response_data_type: Concrete data type of the response
    :param request_data: Concrete data instance to publish as the request
    :param request_topic_name: Optional explicit request topic name
    :param response_topic_name: Optional explicit response topic name
    :param service_name: Optional base name to derive request/response topics
    :param float stable_matches_period_sec: Settling window (seconds) that match counts must remain stable before sending the first request (defaults to 1.0). Set to 0 to skip the extra wait
    :param float service_match_timeout_sec: Deadline (seconds) to complete endpoint matching. Set to 0 to wait indefinitely
    :returns: The response data instance
    :raises ServiceMatchingTimeoutError: If endpoints fail to match within the timeout
    :raises RequestPublishError: If publishing the request fails
    """
    if request_topic_name is None:
        assert (
            service_name is not None
        ), "Either of request_topic_name or service_name are required"
        request_topic_name = _request_prefix + service_name + _request_suffix

    if response_topic_name is None:
        assert (
            service_name is not None
        ), "Either of response_topic_name or service_name are required"
        response_topic_name = _response_prefix + service_name + _response_suffix

    loop = asyncio.get_running_loop()
    future = loop.create_future()
    request_identity = SampleIdentity()
    # Response matching runs entirely on the event-loop thread: on_response (a
    # Fast-DDS reception-thread callback) only hands the sample to the loop via
    # call_soon_threadsafe, and _try_match — which runs on the loop thread, the
    # same thread that sets request_identity below — does the comparison. This
    # deliberately uses NO cross-thread lock around request_identity: a lock
    # there would AB-BA. request_publisher.publish() (below) acquires the
    # participant's lifecycle RLock, and a response published by a service
    # (which already holds that lifecycle RLock) is delivered intra-process,
    # synchronously re-entering on_response — so a lock held across publish()
    # and also taken by on_response would deadlock.
    identity_ready = False
    pending_responses = []  # responses that arrived before request_identity was set

    def _deliver(data):
        if not future.done():
            future.set_result(data)

    def _try_match(data, related):
        # Loop thread only — touches future / identity_ready / request_identity
        # / pending_responses, all of which are otherwise touched only by the
        # coroutine body below (also the loop thread), so no lock is needed.
        if future.done():
            return
        if not identity_ready:
            pending_responses.append((data, related))
        elif related == request_identity:
            _deliver(data)

    def on_response(data, info):
        # Fast-DDS reception thread: copy the related identity (info is reused
        # between calls) and hand off to the loop thread — no lock, no AB-BA.
        related = SampleIdentity(info.related_sample_identity)
        loop.call_soon_threadsafe(_try_match, data, related)

    response_subscriber = Subscriber(
        domain_participant,
        response_topic_name,
        response_pub_sub_type,
        response_data_type,
        on_response,
        reliability_kind=RELIABLE_RELIABILITY_QOS,
    )

    request_publisher = Publisher(
        domain_participant,
        request_topic_name,
        request_pub_sub_type,
        reliability_kind=RELIABLE_RELIABILITY_QOS,
    )

    async def _wait_for_matching():
        check_period = (
            stable_matches_period_sec
            if stable_matches_period_sec is not None and stable_matches_period_sec > 0.0
            else _MIN_MATCH_WAIT_SEC
        )
        match_deadline = (
            time.monotonic() + service_match_timeout_sec
            if service_match_timeout_sec and service_match_timeout_sec > 0.0
            else None
        )

        while True:
            if match_deadline is not None:
                remaining = match_deadline - time.monotonic()
                if remaining <= 0.0:
                    raise ServiceMatchingTimeoutError(
                        f"Request endpoints were not matched within {service_match_timeout_sec} seconds"
                    )
            else:
                remaining = 0.0

            pub_task = loop.run_in_executor(
                None,
                lambda: request_publisher.get_num_matched_subscribers(remaining, check_period),
            )
            sub_task = loop.run_in_executor(
                None,
                lambda: response_subscriber.get_num_matched_publishers(remaining, check_period),
            )
            num_matched_subscribers, num_matched_publishers = await asyncio.gather(pub_task, sub_task)

            if (
                num_matched_subscribers > 0
                and num_matched_subscribers == num_matched_publishers
            ):
                return

            # To avoid too heavy CPU load
            await asyncio.sleep(_MIN_MATCH_WAIT_SEC)

    try:
        await _wait_for_matching()

        # get_guid() and publish() both acquire the participant's lifecycle
        # RLock; neither runs under a response-matching lock (matching is
        # lock-free, on the loop thread — see above) so the intra-process
        # inline-delivery AB-BA they would otherwise form cannot occur. Reading
        # the GUID once here also avoids reading it twice (for `params` and for
        # `request_identity`).
        response_guid = response_subscriber.get_guid()

        params = WriteParams()
        params.related_sample_identity().writer_guid(response_guid)

        if not request_publisher.publish(request_data, params):
            raise RequestPublishError("Failed to publish the DDS request")
        # Set the identity on the loop thread (no await between here and the
        # buffer drain, so _try_match callbacks cannot observe a half-set
        # identity). A response can race ahead of this via intra-process
        # delivery during publish(); match any that _try_match buffered.
        request_identity.writer_guid(response_guid)
        request_identity.sequence_number(params.sample_identity().sequence_number())
        identity_ready = True
        for buffered_data, related in pending_responses:
            if related == request_identity:
                _deliver(buffered_data)
                break
        pending_responses = []

        return await future
    except Exception:
        if not future.done():
            future.cancel()
        raise
    finally:
        del request_publisher
        del response_subscriber


class Service:
    """Request/response service.

    Consumes requests from a request topic and publishes responses to the corresponding response topic.
    Supports both synchronous and async request handlers, back-pressure via max_history_depth, and delayed dispatch
    of responses until the originating client is matched. To drop a request silently from a handler, raise
    Service.IgnoreRequest.
    """

    class IgnoreRequest(Exception):
        """Raise from a request handler to drop the request silently."""
        pass

    class _RequestHandler:
        def __init__(
            self, handle_request_function, on_response_function, max_queue_size
        ):
            self._handle_request_function = handle_request_function
            self._on_response_function = on_response_function
            self._max_queue_size = max_queue_size
            self._requests_queue = Queue()
            self._stop = False
            self._cv = threading.Condition()
            # Daemon: stop() joins this thread for clean shutdown during normal
            # operation, but if a Service is dropped without stop() being
            # called (e.g. relied on GC) it must not survive to interpreter
            # shutdown — threading._shutdown joins non-daemon threads BEFORE
            # late __del__s run, so a non-daemon worker waiting here would
            # deadlock the join forever.
            self._thread = threading.Thread(
                target=self._process_requests, daemon=True
            )
            self._thread.start()

        def __del__(self):
            self.stop()

        def stop(self):
            join = False
            with self._cv:
                if not self._stop:
                    self._stop = True
                    self._cv.notify()
                    join = True
            if join:
                self._thread.join()

        def handle_request(self, request, identity):
            with self._cv:
                if self._requests_queue.qsize() < self._max_queue_size:
                    self._requests_queue.put((request, identity))
                    self._cv.notify()
                else:
                    print(f"Service requests queue is full, dropping request")

        def _process_requests(self):
            while True:
                with self._cv:
                    self._cv.wait_for(
                        lambda: self._stop or not self._requests_queue.empty()
                    )
                    if self._stop:
                        return
                    request, identity = self._requests_queue.get()

                try:
                    response = self._handle_request_function(request)
                    self._on_response_function(response, identity)
                except Service.IgnoreRequest:
                    # Silently drop
                    pass

    class _AsyncRequestHandler:
        def __init__(
            self, handle_request_function, on_response_function, max_queue_size
        ):
            self._handle_request_function = handle_request_function
            self._on_response_function = on_response_function
            self._max_queue_size = max_queue_size
            self._loop = asyncio.new_event_loop()
            # Daemon for the same reason as _RequestHandler._thread — must not
            # block interpreter shutdown if the Service was never stop()ped.
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()

        def __del__(self):
            self.stop()

        def stop(self):
            join = False
            if self._loop.is_running():
                self._loop.call_soon_threadsafe(self._loop.stop)
                join = True
            if join:
                self._thread.join()

        def _run_loop(self):
            asyncio.set_event_loop(self._loop)
            self._loop.run_forever()

        def handle_request(self, request, identity):
            if len(asyncio.all_tasks(self._loop)) < self._max_queue_size:
                asyncio.run_coroutine_threadsafe(
                    self._process_request(request, identity), self._loop
                )
            else:
                print(
                    f"provizio_dds: The service requests queue is full! A request will be dropped."
                )

        async def _process_request(self, request, identity):
            try:
                response = await self._handle_request_function(request)
                self._on_response_function(response, identity)
            except Service.IgnoreRequest:
                # Silently drop
                pass

    def __init__(
        self,
        domain_participant: object,
        request_pub_sub_type: TopicDataType,
        request_data_type: object,
        response_pub_sub_type: TopicDataType,
        handle_request_function: Callable,
        request_topic_name: str = None,
        response_topic_name: str = None,
        service_name: str = None,
        max_history_depth: int = USE_DEFAULT_QOS_DURABILITY,
    ):
        """Construct a request/response service.

        :param domain_participant: Domain participant wrapper created by ``make_domain_participant``
        :param request_pub_sub_type: PubSub type for requests
        :param request_data_type: Concrete request data type
        :param response_pub_sub_type: PubSub type for responses
        :param handle_request_function: Callable or coroutine to process a request and return response data
        :param request_topic_name: Optional explicit request topic name, or use ``service_name``
        :param response_topic_name: Optional explicit response topic name, or use ``service_name``
        :param service_name: If provided, request topic is rq/<service_name>Request and response is rr/<service_name>Reply
        :param int max_history_depth: Reader history depth for transient local durability; 0 for no history (volatile durability), USE_DEFAULT_QOS_DURABILITY for default
        """
        default_max_queue_size = 10
        minimal_max_queue_size = 1

        if request_topic_name is None:
            assert (
                service_name is not None
            ), "Either of request_topic_name or service_name are required"
            request_topic_name = _request_prefix + service_name + _request_suffix

        if response_topic_name is None:
            assert (
                service_name is not None
            ), "Either of response_topic_name or service_name are required"
            response_topic_name = _response_prefix + service_name + _response_suffix

        self._stop = False
        self._ready_responses = []
        self._matched_subscriptions = set()
        self._service_cv = threading.Condition()
        self._request_data_type = request_data_type

        max_queue_size = (
            max_history_depth
            if max_history_depth > 0
            else (
                minimal_max_queue_size
                if max_history_depth == NO_HISTORY
                else default_max_queue_size
            )
        )
        if inspect.iscoroutinefunction(handle_request_function):
            self._request_handler = self._AsyncRequestHandler(
                handle_request_function, self._on_response, max_queue_size
            )
        else:
            self._request_handler = self._RequestHandler(
                handle_request_function, self._on_response, max_queue_size
            )

        self._publisher = Publisher(
            domain_participant,
            response_topic_name,
            response_pub_sub_type,
            on_has_subscriber_changed_function=self._on_matched,
            reliability_kind=RELIABLE_RELIABILITY_QOS,
            history_depth=max_queue_size,
        )

        self._subscriber = Subscriber(
            domain_participant,
            request_topic_name,
            request_pub_sub_type,
            request_data_type,
            on_data_function=self._on_data,
            reliability_kind=RELIABLE_RELIABILITY_QOS,
            max_history_depth=max_history_depth,
        )

        # Daemon for the same reason as _RequestHandler._thread — must not
        # block interpreter shutdown if the Service was never stop()ped.
        self._dispatch_responses_thread = threading.Thread(
            target=self._dispatch_responses, daemon=True
        )
        self._dispatch_responses_thread.start()

    def __del__(self):
        self.stop()

    def stop(self):
        """Stops the service and joins internal threads."""
        join = False
        with self._service_cv:
            if not self._stop:
                self._stop = True
                self._service_cv.notify_all()
                join = True
        if join:
            # All shutdown work happens with _service_cv RELEASED. The
            # Publisher's on_publication_matched listener callback (wired
            # to Service._on_matched) tries to acquire _service_cv; a
            # client disconnecting during teardown fires an unmatch
            # callback that races endpoint destruction. If we held
            # _service_cv across delattr(_publisher), the listener thread
            # would block on _service_cv while inside the publisher's
            # _drain scope — and Publisher.__del__'s drain.detach_and_drain
            # (and Fast-DDS's own delete_datawriter callback drain) would
            # wait forever for that callback to leave the scope. AB-BA
            # between _service_cv and the drain.
            #
            # Order matters:
            #  1. _request_handler.stop() joins the request-handler thread;
            #     that thread is the only one calling _on_response, which
            #     uses _publisher.publish. Must finish before _publisher
            #     is destroyed.
            #  2. _dispatch_responses_thread.join() joins the dispatch
            #     thread; it also calls _publisher.publish via
            #     _dispatch_matched. Must finish before _publisher is
            #     destroyed.
            #  3. Only then is it safe to destroy the endpoints — by this
            #     point no Service-managed thread can call into them.
            if hasattr(self, '_request_handler'):
                self._request_handler.stop()
            self._dispatch_responses_thread.join()
            # Delete in child-before-parent order; their __del__ methods
            # are guarded against participant-already-cleaned-up.
            for attr in ('_subscriber', '_publisher', '_request_handler'):
                if hasattr(self, attr):
                    delattr(self, attr)

    def _on_data(self, data, info):
        identity = info.sample_identity
        if info.related_sample_identity.writer_guid() != GUID_t.unknown():
            identity.writer_guid(info.related_sample_identity.writer_guid())

        # info and data are reused internally between on_data calls despite
        # _on_data itself is not called concurrently, so copies are required to
        # avoid race conditions
        self._request_handler.handle_request(
            self._request_data_type(data), SampleIdentity(identity)
        )

    def _on_matched(self, _, matched, subscriber_guid):
        with self._service_cv:
            subscriber_guid_str = str(subscriber_guid)
            if matched:
                self._matched_subscriptions.add(subscriber_guid_str)
            else:
                self._matched_subscriptions.discard(subscriber_guid_str)
            self._service_cv.notify_all()

    def _on_response(self, data, identity):
        # Decide under the lock, publish outside it. Holding _service_cv
        # across self._publisher.publish risks the following AB-BA when
        # another thread is destroying a response Subscriber:
        #   destroying thread:  holds participant.lifecycle_lock (in
        #                       Subscriber.__del__'s teardown), waits on
        #                       _service_cv (delete_datareader fires
        #                       on_publication_matched on the SERVICE
        #                       publisher's listener, which calls
        #                       Service._on_matched).
        #   this thread:        holds _service_cv (this `with` block),
        #                       waits on lifecycle_lock (inside
        #                       self._publisher.publish).
        # Decoupling the check from the publish lets the destroying
        # thread's on-publication-matched callback acquire _service_cv
        # freely and complete, releasing lifecycle_lock back to us. The
        # match state can change between check and publish, but a
        # send-to-unmatched is a benign no-op and a now-matched response
        # that we instead queued will be picked up by
        # _dispatch_responses + _dispatch_matched on the next wake-up.
        publish_now = False
        with self._service_cv:
            if self._is_subscriber_matched_mutex_prelocked(identity.writer_guid()):
                publish_now = True
            else:
                self._ready_responses.append(
                    {"data": data, "identity": identity, "time_ready": time.time()}
                )
                self._service_cv.notify_all()
        if publish_now:
            params = WriteParams()
            params.related_sample_identity(identity)
            self._publisher.publish(data, params)

    def _is_subscriber_matched_mutex_prelocked(self, subscriber_guid):
        return str(subscriber_guid) in self._matched_subscriptions

    def _has_dispatchable_matched(self):
        """Return True if any queued response now has a matched subscriber.
        Caller holds _service_cv. Pure check, no side effects (does NOT
        publish or pop) — the dispatch happens outside the lock in
        _dispatch_responses to avoid AB-BA with Subscriber teardown."""
        for response in self._ready_responses:
            if self._is_subscriber_matched_mutex_prelocked(
                response["identity"].writer_guid()
            ):
                return True
        return False

    def _cleanup_timed_out(self):
        cleaned = False
        max_time_to_match = 10
        current_time = time.time()
        for i in range(len(self._ready_responses) - 1, -1, -1):
            response = self._ready_responses[i]
            if current_time - response["time_ready"] > max_time_to_match:
                self._ready_responses.pop(i)
                cleaned = True

        return cleaned

    def _dispatch_responses(self):
        while True:
            to_publish = []
            with self._service_cv:
                self._service_cv.wait_for(
                    lambda: self._stop
                    or self._has_dispatchable_matched()
                    or self._cleanup_timed_out(),
                    timeout=2,
                )
                if self._stop:
                    return
                # Pop newly-matched responses out of the queue under the
                # lock; publish below outside the lock so a Subscriber
                # destructor's delete_datareader (which fires
                # on_publication_matched on our publisher → Service._on_matched
                # → wants _service_cv) can complete without blocking on a
                # held _service_cv. Same AB-BA rationale as _on_response.
                for i in range(len(self._ready_responses) - 1, -1, -1):
                    response = self._ready_responses[i]
                    if self._is_subscriber_matched_mutex_prelocked(
                        response["identity"].writer_guid()
                    ):
                        to_publish.append(response)
                        self._ready_responses.pop(i)
            # Publish outside the lock.
            for response in to_publish:
                params = WriteParams()
                params.related_sample_identity(response["identity"])
                self._publisher.publish(response["data"], params)


_request_prefix = "rq/"
_response_prefix = "rr/"
_request_suffix = "Request"
_response_suffix = "Reply"
