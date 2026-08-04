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
import re
import sys
import threading
import weakref
from collections import deque
from enum import Enum, IntFlag
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


# Sentinel reliability kind selecting "match the discovered publisher". A
# subscriber left at this reliability DEFERS creating its Fast-DDS DataReader
# until the first matching DataWriter is discovered on its topic, then builds the
# reader with that writer's offered reliability (reliability only — durability is
# untouched). eProsima's ReliabilityQosPolicyKind uses 1=BEST_EFFORT, 2=RELIABLE,
# so 0 is unused and free to repurpose as this sentinel — mirrors the C++
# provizio::dds::match_publisher_reliability_qos (qos_defaults.h) and the
# use_default_history_depth=-1 idiom. Compared against the integer reliability
# kinds (reader_qos.reliability().kind / the discovered info.reliability.kind),
# never written into a DataReaderQos.
MATCH_PUBLISHER_RELIABILITY_QOS = 0

# Fail fast at import if a Fast-DDS upgrade ever assigns 0 to a real reliability kind:
# the sentinel above would silently collide with it and every default (match-publisher)
# subscriber would misbehave. Mirrors the C++ static_assert in qos_defaults.h. (Not an
# `assert` so `python -O` can't strip the check.)
if MATCH_PUBLISHER_RELIABILITY_QOS in (
    BEST_EFFORT_RELIABILITY_QOS,
    RELIABLE_RELIABILITY_QOS,
):
    raise RuntimeError(
        "MATCH_PUBLISHER_RELIABILITY_QOS sentinel collides with a real "
        "ReliabilityQosPolicyKind — pick an unused value for the sentinel"
    )


class QosDefaults:
    """Defines default QOS policies. They can be overriden for specific types"""

    """Per type defaults for datawriter_reliability_kind. RELIABLE_RELIABILITY_QOS by default in Fast DDS"""
    datawriter_reliability_kind_per_type = {None: RELIABLE_RELIABILITY_QOS}

    """Per type defaults for datareader_reliability_kind. MATCH_PUBLISHER_RELIABILITY_QOS — the subscriber adopts
    the discovered publisher's reliability (deferred DataReader creation). Override per type, or per subscriber via
    the reliability_kind argument, with an explicit BEST_EFFORT_RELIABILITY_QOS / RELIABLE_RELIABILITY_QOS to build
    the reader eagerly. Mirrors the C++ qos_defaults<T>::datareader_reliability_kind default."""
    datareader_reliability_kind_per_type = {None: MATCH_PUBLISHER_RELIABILITY_QOS}

    """Per type defaults for memory policies, both datawriter and datareader. PREALLOCATED_WITH_REALLOC_MEMORY_MODE in Fast-DDS 2.9+"""
    memory_policy_per_type = {None: PREALLOCATED_WITH_REALLOC_MEMORY_MODE}

    """Per type defaults for the datawriter publish mode. SYNCHRONOUS_PUBLISH_MODE by default in Fast-DDS;
    large sample types (images, point clouds) override this to ASYNCHRONOUS_PUBLISH_MODE so a multi-MB write
    hands off to the participant's async sender thread instead of blocking the publishing thread. Writer-local —
    NOT an RxO QoS, so it never affects reader/writer matching or ROS 2 interop. Registered lazily for the large
    sample types by _register_large_sample_qos_defaults() below."""
    datawriter_publish_mode_per_type = {None: SYNCHRONOUS_PUBLISH_MODE}

    """Per type default KEEP_LAST history depth, applied to both datawriter and datareader only when the caller
    doesn't request an explicit positive history_depth / max_history_depth. 0 means "no per-type override" (keep
    the Fast-DDS default). Large sample types override this to a small depth (4) so a momentarily slow consumer
    doesn't drop big frames. Sets history depth only — durability is controlled separately, so it is NOT an RxO
    QoS and doesn't affect matching or ROS 2 interop. Registered lazily by _register_large_sample_qos_defaults()."""
    keep_last_history_depth_per_type = {None: 0}

    """Default count of the INITIAL discovery-announcement burst sent once at participant creation.
    De-escalated from a former 200 to a modest burst that still beats best-effort multicast loss on
    a lossy link (well above Fast-DDS's own default of 5) without flooding the network. Overridable
    via PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT. Mirrors src/domain_participant.cpp."""
    num_initial_discovery_announcements = 15

    """Default spacing of the initial discovery announcements. Overridable (in milliseconds) via
    PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS."""
    initial_announcements_period = Duration_t(0, 100000000)  # 100 ms, as (sec, nanosec)

    """Default period of the PERIODIC participant re-announcement after the initial burst
    (leaseDuration_announcementperiod). Paid by every participant forever, so its multicast rate
    scales with the participant count; de-escalated from a former 1 s to the Fast-DDS default of 3 s
    (well under the 20 s lease) so the library is not itself a primary source of UDP congestion when
    many participants run. Overridable (in milliseconds) via PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS."""
    lease_duration_announcement_period = Duration_t(3, 0)  # 3 s, as (sec, nanosec)

    """Default participant lease duration — how long a peer is considered alive without a fresh
    announcement. Raised from Fast-DDS's 20 s default to 30 s so the relaxed announcement cadence has
    more margin before a peer is wrongly declared lost when announcements are dropped on a congested
    or lossy link. Must stay longer than lease_duration_announcement_period. Overridable (in
    milliseconds) via PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS. Mirrors src/domain_participant.cpp."""
    lease_duration = Duration_t(30, 0)  # 30 s, as (sec, nanosec)

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
        try:
            self.datawriter_publish_mode = (
                QosDefaults.datawriter_publish_mode_per_type[pub_sub_type]
            )
        except KeyError:
            self.datawriter_publish_mode = (
                QosDefaults.datawriter_publish_mode_per_type[None]
            )
        try:
            self.keep_last_history_depth = (
                QosDefaults.keep_last_history_depth_per_type[pub_sub_type]
            )
        except KeyError:
            self.keep_last_history_depth = (
                QosDefaults.keep_last_history_depth_per_type[None]
            )


def _register_large_sample_qos_defaults():
    """Register per-type QoS defaults for the "large sample" message types
    (camera frames, point clouds, occupancy grids): ASYNCHRONOUS datawriter
    publish mode + a modest KEEP_LAST(4) history so a multi-MB write hands off
    to the participant's async sender thread and a momentarily slow consumer
    doesn't drop big frames. Mirrors the C++ ``qos_defaults<T>`` specializations
    (``detail::large_sample_qos_defaults`` in include/provizio/dds/qos_defaults.h).

    The publish mode is writer-local and the history depth is set without
    touching durability, so neither is an RxO QoS — reader/writer matching and
    ROS 2 interop are unaffected.

    Registration is lazy and defensive: the generated message bindings come from
    ``provizio_dds_python_types`` (wildcard-imported above), but not every build
    necessarily ships every type. Each lookup is guarded so a missing type is
    skipped silently rather than breaking import of the whole module.
    """
    # Bare names resolve against this module's globals, populated by the
    # `from provizio_dds_python_types import *` at the top of the file (the same
    # mechanism by which user code reaches e.g. provizio_dds.PointCloud2PubSubType).
    large_sample_pub_sub_type_names = (
        "ImagePubSubType",            # sensor_msgs/msg/Image
        "CompressedImagePubSubType",  # sensor_msgs/msg/CompressedImage
        "MultiEchoLaserScanPubSubType",  # sensor_msgs/msg/MultiEchoLaserScan
        "PointCloud2PubSubType",      # sensor_msgs/msg/PointCloud2
        "OccupancyGridPubSubType",    # nav_msgs/msg/OccupancyGrid
        # A freespace polygon is a sequence of vertices, so a dense one runs to several
        # KB and fragments over UDP; without the async + KEEP_LAST(4) override a
        # reliable writer's single history slot is overwritten before a momentarily
        # slow reader has acknowledged the previous sample's fragments.
        "PolygonStampedPubSubType",   # geometry_msgs/msg/PolygonStamped
        "PolygonInstanceStampedPubSubType",  # geometry_msgs/msg/PolygonInstanceStamped
    )
    for type_name in large_sample_pub_sub_type_names:
        try:
            cls = globals()[type_name]
        except KeyError:
            # The generated binding for this type isn't available in this build;
            # leave it on the (synchronous, no-override) primary defaults.
            continue
        QosDefaults.datawriter_publish_mode_per_type[cls] = ASYNCHRONOUS_PUBLISH_MODE
        QosDefaults.keep_last_history_depth_per_type[cls] = 4
        # Reader reliability default = match-publisher, same as the primary
        # (None) default. The large-sample types inherit it via the None
        # fallback anyway, but the C++ large_sample_qos_defaults sets it
        # explicitly, so mirror that here so a future change to the primary
        # default can't silently flip these types to a different reliability.
        QosDefaults.datareader_reliability_kind_per_type[cls] = MATCH_PUBLISHER_RELIABILITY_QOS


_register_large_sample_qos_defaults()


USE_DEFAULT_HISTORY_DEPTH = -1
# Default KEEP_LAST history depth for a ServiceClient's request/response endpoints: bounds how many requests
# may be concurrently in flight before the reliable endpoints apply back-pressure. Matches the service's
# default response history and the C++ service_client_default_history_depth. Raise it via the ServiceClient
# endpoint_history_depth argument when you need more than this many requests outstanding at once.
SERVICE_CLIENT_DEFAULT_HISTORY_DEPTH = 10
# Sentinel 0 for request/response max_history_depth: select the minimal bounded request queue.
# (For make_publisher / make_subscriber any non-positive history_depth simply keeps the default depth.)
MINIMAL_REQUEST_QUEUE = 0

_DEFAULT_STABLE_MATCH_WINDOW_SEC = 1.0
_MIN_MATCH_WAIT_SEC = 0.05
# Cap the match-settling at this multiple of the settling window after the first
# match, so endpoints matching/unmatching under churn can't delay the first
# request forever.
_SETTLE_CAP_MULTIPLIER = 4


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
# Topology discovery: optional callback fired when remote DataWriters /
# DataReaders appear or disappear in the DDS domain. Mirrors the C++
# `provizio::dds::domain_participant::on_discovered_endpoint` /
# `endpoint_kind` API. Zero cost for participants that don't opt in.
# ---------------------------------------------------------------------------


class EndpointKind(IntFlag):
    """Which remote endpoint kinds a :meth:`_DomainParticipant.on_discovered_endpoint`
    callback fires for. Bit flags — combine with ``|`` to receive both."""

    NONE = 0  # Empty mask: no endpoint kinds (mirrors C++ endpoint_kind::none).
    DATA_WRITER = 1  # Remote DataWriters (someone publishing on a topic).
    DATA_READER = 2  # Remote DataReaders (someone subscribing to a topic).


class _DiscoveryListener(DomainParticipantListener):
    """SWIG director that forwards Fast-DDS DomainParticipantListener discovery
    events to a Python callback. Owned by :class:`_DomainParticipant` and installed
    EAGERLY (always, via ``set_listener``) — it is no longer opt-in, because it now
    drives the match-publisher subscriber default (a discovered DataWriter resolves
    any subscriber parked on its topic) in addition to the optional user
    ``on_discovered_endpoint`` callback. With no user callback registered it still
    runs the internal resolver and is otherwise a near-zero-cost observer. Must be
    kept alive by a strong reference on the participant (Fast-DDS holds it by raw
    pointer).

    The listener carries a weak reference to its owning :class:`_DomainParticipant`
    so it can pass the wrapper into the user callback as the first argument,
    mirroring the C++ ``on_discovered_endpoint_callback`` signature. The owner
    is set by :class:`_DomainParticipant` immediately after constructing the
    listener; the same wrapper instance is preserved across network-recovery
    resets (only the underlying Fast-DDS participant is swapped), so the owner
    reference doesn't need re-setting on reset. Weak to avoid a strong cycle
    between the wrapper (holds a strong listener ref) and the listener (would
    otherwise hold a strong wrapper ref)."""

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._callback = None
        self._kinds = EndpointKind.DATA_WRITER
        self._owner_ref = None

    def set_owner(self, owner):
        # Stored as a weakref so the listener doesn't keep the wrapper alive.
        # The wrapper outlives the listener (the listener is a member), so the
        # ref dereferences successfully for every callback invocation.
        with self._lock:
            self._owner_ref = weakref.ref(owner) if owner is not None else None

    def set_callback(self, callback, kinds):
        with self._lock:
            self._callback = callback
            self._kinds = kinds

    def _invoke(self, info, kind, discovered):
        # Snapshot under the lock so a concurrent set_callback can't swap
        # the function out mid-call.
        with self._lock:
            cb = self._callback
            kinds = self._kinds
            owner = self._owner_ref() if self._owner_ref is not None else None

        # Convert the topic name at most once per call: it's needed by the internal deferred
        # resolution (writers) and again by the user callback, and to_string() allocates on the
        # discovery thread for every event. Lazy so a data_reader event with no user callback
        # performs zero conversions. Mirrors the C++ invoke().
        _topic_name = None

        def topic_name():
            nonlocal _topic_name
            if _topic_name is None:
                _topic_name = info.topic_name.to_string()
            return _topic_name

        # Internal match-publisher resolution runs FIRST and UNCONDITIONALLY of the
        # user callback: a discovered DataWriter must resolve any subscriber parked
        # on its topic regardless of whether the user opted into on_discovered_endpoint
        # (the listener is now installed eagerly precisely so no SEDP writer event is
        # missed). resolve_for_writer only records the reliability and submits the build
        # to the process-wide reaper — it never builds an endpoint on this discovery
        # thread. Mirrors the C++ invoke(): owner.resolve_deferred_for_writer
        # before the user callback. Wrapped so a (defensive) failure can't escape the
        # Fast-DDS director.
        if owner is not None and (kind & EndpointKind.DATA_WRITER):
            try:
                with _fastdds_callback_scope():
                    if discovered:
                        owner._resolve_deferred_for_writer(
                            topic_name(), info.reliability.kind
                        )
                    else:
                        # REMOVED writer: maintain the per-reliability live-writer count so the
                        # adopted reliability is re-derived / dropped as writers leave. The removed
                        # writer's offered reliability is carried on the same discovery info.
                        owner._on_writer_removed(topic_name(), info.reliability.kind)
            except Exception as ex:  # noqa: BLE001
                _network_recovery._emit_log(
                    _network_recovery.LogLevel.ERROR,
                    f"deferred-subscriber writer resolution threw: {ex}",
                )

        if cb is None or not (kinds & kind) or owner is None:
            # owner is None either because set_owner hasn't run yet (extremely
            # tight window between listener construction and the wrapper
            # assigning self as owner) or because the wrapper has been garbage-
            # collected. Drop the event — there's nothing meaningful the user
            # callback could do with a dead participant.
            return
        # Mark this thread as running a Fast-DDS listener callback so any
        # endpoint __del__ fired from inside the user callback (e.g. via a
        # dropped strong reference) defers its Fast-DDS-side teardown to the
        # reaper thread instead of self-joining the discovery thread. Matches
        # the convention used by the publisher / subscriber listener
        # callbacks elsewhere in this module.
        with _fastdds_callback_scope():
            try:
                # Convert type_name only now that the callback is known to fire (topic_name()
                # reuses the at-most-once conversion above), inside the guard: info.*.to_string()
                # can raise (MemoryError, or a SWIG-translated C++ exception), and that must not
                # escape the Fast-DDS director callback either.
                #
                # The reliability / durability kinds are plain enum reads off
                # the discovery info — Publication/SubscriptionBuiltinTopicData
                # expose them as the public ``reliability`` / ``durability`` QoS
                # policy members (offered for a discovered DataWriter, requested
                # for a discovered DataReader). They are SWIG attributes (not
                # methods — no parentheses), each carrying a ``.kind`` directly
                # comparable to the module-level ``*_RELIABILITY_QOS`` /
                # ``*_DURABILITY_QOS`` constants. Forwarded as the two trailing
                # callback arguments so a recording bridge can match QoS per
                # topic. Mirrors the C++ on_discovered_endpoint invoke().
                cb(
                    owner,
                    topic_name(),
                    info.type_name.to_string(),
                    kind,
                    discovered,
                    info.reliability.kind,
                    info.durability.kind,
                )
            except Exception as ex:
                # A throwing user callback (or a failed name conversion) must
                # not propagate out of the Fast-DDS discovery thread (mirrors
                # the C++ logging convention). Log so it's debuggable rather
                # than silently dropped.
                _network_recovery._emit_log(
                    _network_recovery.LogLevel.ERROR,
                    f"on_discovered_endpoint dispatch threw: {ex}",
                )

    def on_data_writer_discovery(self, participant, reason, info, should_be_ignored):
        # should_be_ignored is intentionally not assigned: SWIG passes it as an
        # opaque bool* proxy (SWIGTYPE_p_bool), so `should_be_ignored = False`
        # here would only rebind the local name and never reach the C++ variable.
        # Fast-DDS already initialises it to false at the call site
        # (DomainParticipantImpl::on_data_writer_discovery) before invoking the
        # listener, so a discovered writer is never ignored — exactly what this
        # purely observational listener wants.
        if reason == WriterDiscoveryStatus_DISCOVERED_WRITER:
            discovered = True
        elif reason == WriterDiscoveryStatus_REMOVED_WRITER:
            discovered = False
        else:
            # CHANGED_QOS_WRITER / IGNORED_WRITER — not relevant to a
            # "is data flowing on this topic?" subscriber.
            return
        self._invoke(info, EndpointKind.DATA_WRITER, discovered)

    def on_data_reader_discovery(self, participant, reason, info, should_be_ignored):
        # should_be_ignored intentionally left untouched — see the explanation in
        # on_data_writer_discovery (SWIG bool* proxy + Fast-DDS pre-init to false).
        if reason == ReaderDiscoveryStatus_DISCOVERED_READER:
            discovered = True
        elif reason == ReaderDiscoveryStatus_REMOVED_READER:
            discovered = False
        else:
            return
        self._invoke(info, EndpointKind.DATA_READER, discovered)


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


_DEFAULT_UDP_SOCKET_BUFFER_SIZE = 16 * 1024 * 1024


def _resolve_udp_socket_buffer_size():
    """UDP socket send/recv buffer ceiling in bytes (default 16 MiB).

    Override via the ``PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE`` env variable. It's a
    ceiling — the OS clamps it to ``net.core.rmem_max``/``wmem_max``. Mirrors
    ``src/domain_participant.cpp``.
    """
    raw = os.environ.get("PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE")
    if raw:
        try:
            value = int(raw)
        except ValueError:
            value = 0
        if 0 < value <= 0xFFFFFFFF:
            return value
        _network_recovery._emit_log(
            _network_recovery.LogLevel.ERROR,
            f"ignoring invalid PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE="
            f"'{raw}'; using default {_DEFAULT_UDP_SOCKET_BUFFER_SIZE}",
        )
    return _DEFAULT_UDP_SOCKET_BUFFER_SIZE


# ---- Auto-discovery (SPDP) tuning ------------------------------------------------------------
#
# Participant discovery announcements are multicast best-effort, so some are lost on a busy or
# lossy network. Fast-DDS counters this with an INITIAL burst of announcements at participant
# creation plus a PERIODIC re-announcement thereafter. Set too high, both make the library itself
# a primary source of UDP congestion once many participants (sensors + clients) run at once. The
# defaults (in QosDefaults) are de-escalated to a modest burst and a relaxed cadence that still
# discover robustly; all four are overridable at runtime via these env variables (counts as plain
# integers, periods in milliseconds) without recompiling. Keep these names and the resolver
# behaviour in sync with src/domain_participant.cpp.
_INITIAL_ANNOUNCEMENT_COUNT_ENV = "PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT"
_INITIAL_ANNOUNCEMENT_PERIOD_MS_ENV = "PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS"
_ANNOUNCEMENT_PERIOD_MS_ENV = "PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS"
_LEASE_DURATION_MS_ENV = "PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS"


def _resolve_discovery_positive_int_env(name, fallback):
    """Strictly-positive int from env var ``name``, else ``fallback`` (logged at warning).

    Rejects unset/empty/non-numeric/zero/overflowing input so a typo can never silently disable or
    wildly misconfigure discovery. Mirrors ``resolve_positive_u32_env`` in src/domain_participant.cpp.
    """
    raw = os.environ.get(name)
    if not raw:
        return fallback
    # Mirror the C++ resolve_positive_u32_env (strtoull + full-consume): allow leading whitespace and a single
    # leading '+', then ASCII digits to end-of-string. This rejects trailing characters, underscores and other
    # input that Python's permissive int() would accept, so both languages treat the same env values as valid.
    value = int(raw) if re.fullmatch(r"\s*\+?[0-9]+", raw) else 0
    if 0 < value <= 0xFFFFFFFF:
        return value
    _network_recovery._emit_log(
        _network_recovery.LogLevel.WARNING,
        f"ignoring invalid {name}='{raw}'; using default {fallback}",
    )
    return fallback


def _milliseconds_to_duration(milliseconds):
    """Milliseconds -> Fast-DDS ``Duration_t(sec, nanosec)``."""
    return Duration_t(milliseconds // 1000, (milliseconds % 1000) * 1000000)


def _resolve_discovery_initial_announcement_count():
    """Initial discovery-announcement count, env-overridable (default
    ``QosDefaults.num_initial_discovery_announcements``)."""
    return _resolve_discovery_positive_int_env(
        _INITIAL_ANNOUNCEMENT_COUNT_ENV, QosDefaults.num_initial_discovery_announcements
    )


def _resolve_discovery_initial_announcement_period():
    """Spacing of the initial discovery announcements as a ``Duration_t``, env-overridable in
    milliseconds (default ``QosDefaults.initial_announcements_period``)."""
    fallback_ms = QosDefaults.initial_announcements_period.to_ns() // 1000000
    return _milliseconds_to_duration(
        _resolve_discovery_positive_int_env(_INITIAL_ANNOUNCEMENT_PERIOD_MS_ENV, fallback_ms)
    )


def _resolve_discovery_announcement_period():
    """Periodic participant re-announcement period as a ``Duration_t``, env-overridable in
    milliseconds (default ``QosDefaults.lease_duration_announcement_period``)."""
    fallback_ms = QosDefaults.lease_duration_announcement_period.to_ns() // 1000000
    return _milliseconds_to_duration(
        _resolve_discovery_positive_int_env(_ANNOUNCEMENT_PERIOD_MS_ENV, fallback_ms)
    )


def _resolve_discovery_lease_duration():
    """Participant lease duration as a ``Duration_t``, env-overridable in milliseconds (default
    ``QosDefaults.lease_duration``)."""
    fallback_ms = QosDefaults.lease_duration.to_ns() // 1000000
    return _milliseconds_to_duration(
        _resolve_discovery_positive_int_env(_LEASE_DURATION_MS_ENV, fallback_ms)
    )


def _build_deferred_subscriber(subscriber, reliability):
    """Run a deferred (match-publisher) DataReader build for ``subscriber`` with the
    adopted ``reliability``, OFF the Fast-DDS discovery thread.

    Submitted to the process-wide reaper by
    :meth:`_DomainParticipant._dispatch_deferred_build` when a matching writer is
    discovered (or was already cached). It must NOT run inline on the discovery
    thread for two reasons:

    * Building an endpoint there is a Fast-DDS reentrancy hazard and a lock-order
      deadlock against a concurrent network-recovery reset (the documented
      ``on_discovered_endpoint`` contract).
    * The reaper is the one thread on which it is always safe to drop the LAST
      reference to a ``Subscriber``: a Python ``__del__`` / Fast-DDS teardown landing
      here cannot self-join a Fast-DDS callback/event thread (see
      :class:`network_recovery.DeferredReaper`). The reaper task holds the only strong
      reference the build needs; when it returns and drops it, any cascading
      ``Subscriber.__del__`` / participant ``_cleanup`` also runs on the reaper —
      never on a Fast-DDS thread.

    Takes the participant's registration mutex then lifecycle lock — the SAME order as
    ``_register_endpoint``'s initial build — so there is no lock-order inversion;
    ``_resolve_match_and_build`` -> ``_build_state`` therefore must NOT re-acquire them.

    The Python analog of C++ ``domain_participant::run_deferred_build``. Replaces the
    per-participant ``_DeferredBuildWorker`` thread: the participant now only routes
    discovery to parked subscribers (a non-owning weakref registry) and hops the build
    onto the reaper that already exists for off-thread teardown."""
    try:
        participant = subscriber._participant
        if participant is None:
            return
        # Same lock order as _register_endpoint's initial build: registration mutex
        # (serialize against resets) then lifecycle lock (keep the inner participant
        # stable). _resolve_match_and_build -> _build_state runs under both, so it
        # must NOT re-acquire them.
        with participant._registration_mutex:
            with participant.lifecycle_lock():
                fastdds_participant = participant._participant
                if fastdds_participant is not None:
                    subscriber._resolve_match_and_build(reliability, fastdds_participant)
                # else: a network-recovery recreate failed and the participant is dead.
                # Skip — the eventual reset rebuilds every endpoint (re-deferring this
                # one, which re-registers and is resolved again from the cache).
    except Exception as ex:  # noqa: BLE001
        _network_recovery._emit_log(
            _network_recovery.LogLevel.ERROR,
            f"deferred match-publisher subscriber build failed: {ex}; this subscriber "
            f"stays inactive (get_num_matched_publishers returns 0) until the next discovered "
            f"writer on its topic retries the build (or a network-recovery reset rebuilds it)",
        )


# Serialises the brief, process-global factory-autoenable toggle performed by
# _create_participant_with_listener across concurrent participant creation.
_participant_create_lock = threading.Lock()


def _create_participant_with_listener(factory, domain_id, participant_qos, listener):
    """Create a DomainParticipant with ``listener`` attached BEFORE it starts
    discovery, then enable it. Returns the enabled participant, or ``None`` if
    Fast-DDS could not create it (the caller decides how to surface that).

    Why not the obvious ``create_participant`` then ``set_listener``: a participant
    is auto-enabled at creation and immediately begins discovery, so any endpoint
    it discovers between ``create_participant`` returning and ``set_listener`` runs
    fires the discovery callback with no listener attached and is lost forever
    (Fast-DDS never replays already-discovered endpoints to a later listener). That
    gap is normally microseconds, but a scheduling stall on a loaded host widens it
    enough to drop the event — reproducibly the post-reset re-discovery in the
    discovered-endpoints ``survives_reset`` test, where the peer's writer already
    exists and is discoverable the instant the recreated participant enables.

    The C++ side avoids this by passing the listener into ``create_participant``;
    the Python binding can't (handing a SWIG director to ``create_participant``
    disowns it and breaks reuse across resets). So instead create the participant
    disabled — Fast-DDS only auto-enables when the factory's entity-factory QoS says
    to, and there is no per-call flag — attach the listener while it is quiescent,
    then enable it. The factory toggle is process-global, hence the lock; it is
    always restored (``finally``)."""
    with _participant_create_lock:
        factory_qos = DomainParticipantFactoryQos()
        factory.get_qos(factory_qos)
        previous_autoenable = factory_qos.entity_factory().autoenable_created_entities
        factory_qos.entity_factory().autoenable_created_entities = False
        factory.set_qos(factory_qos)
        try:
            participant = factory.create_participant(domain_id, participant_qos)
        finally:
            factory_qos.entity_factory().autoenable_created_entities = previous_autoenable
            factory.set_qos(factory_qos)
    if participant is None:
        return None
    # Attach BEFORE enable so the participant is quiescent (no discovery yet) while
    # the listener goes on; enable() then starts discovery with it already in place.
    participant.set_listener(listener, StatusMask.none())
    participant.enable()
    return participant


class TransportMode(Enum):
    """Network transport selection for :func:`make_domain_participant`, mirroring the
    C++ ``provizio::dds::transport_mode``. Controls only whether shared memory is used
    in addition to UDP; the enlarged UDP socket buffers (for reliable large-sample
    delivery) apply in both modes.

    NOTE: in Python the choice is applied via the process-global
    ``FASTDDS_BUILTIN_TRANSPORTS`` env variable (the SWIG bindings expose no
    per-participant transport API), so the first participant created in a process fixes
    the transport for the whole process, and an externally-set value is always honoured.
    """

    #: Platform default: SHM + UDPv4 on Linux; UDPv4-only on Windows/macOS (where shared
    #: memory is disabled to dodge a Boost.Interprocess cleanup bug).
    AUTOMATIC = 0
    #: UDPv4-only (shared memory disabled) on every platform. For participants bridging
    #: mismatched Fast-DDS major versions (e.g. a recorder relaying 2.x publishers).
    UDP_ONLY = 1


def make_domain_participant(domain_id: int = 0,
                            recovery_mode: "NetworkRecoveryMode" = None,
                            initial_discovery_callback=None,
                            initial_discovery_kinds=None,
                            transport: "TransportMode" = None):
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
    :param initial_discovery_callback: Optional endpoint-discovery callback
        registered on the (always-installed) discovery listener at construction,
        before the participant starts discovering — so it cannot miss an endpoint,
        unlike calling :meth:`on_discovered_endpoint` after construction (which can
        race a discovery already in flight). The participant is created disabled,
        the listener attached, then the participant enabled (see
        ``_create_participant_with_listener``); this closes the
        create-then-``set_listener`` gap that would otherwise drop an endpoint
        discovered in between, matching the C++ path (which passes the listener
        into ``create_participant``). Defaults to ``None`` (no user callback; the
        listener still runs its internal match-publisher resolver).
    :param initial_discovery_kinds: Which :class:`EndpointKind` values the
        initial callback fires for. Ignored when
        ``initial_discovery_callback`` is ``None``. Defaults to
        :attr:`EndpointKind.DATA_WRITER` (the right choice for a recorder).
    :param transport: Network transport selection (see :class:`TransportMode`).
        Defaults to :attr:`TransportMode.AUTOMATIC` (platform default: SHM+UDP on
        Linux, UDP-only on Windows/macOS). Pass :attr:`TransportMode.UDP_ONLY` to
        disable shared memory on all platforms (e.g. when bridging mismatched Fast-DDS
        major versions). Applied via the process-global ``FASTDDS_BUILTIN_TRANSPORTS``
        env variable (see :class:`TransportMode`).

        LIMITATION: unlike C++ (which applies transports per-participant), this is
        **process-global** — the first participant created in the process (or an
        externally-set ``FASTDDS_BUILTIN_TRANSPORTS``) fixes the transport for every
        later participant. A non-AUTOMATIC ``transport`` that conflicts with an
        already-set value is ignored and logged at WARNING; it does NOT take effect.
        To give a participant a specific transport, ensure it is the first one created,
        or set ``FASTDDS_BUILTIN_TRANSPORTS`` yourself before creating any participant.
    :return: A wrapped DDS Domain Participant
    """

    if recovery_mode is None:
        recovery_mode = _network_recovery.NetworkRecoveryMode.ENV_VAR_CONTROLLED
    if initial_discovery_kinds is None:
        initial_discovery_kinds = EndpointKind.DATA_WRITER
    if transport is None:
        transport = TransportMode.AUTOMATIC

    class _DomainParticipant:
        # Hardcoded — must match the default_fastdds_env_variable constant in
        # src/domain_participant.cpp. Fast-DDS 3.x renamed the variable from
        # FASTRTPS_DEFAULT_PROFILES_FILE to FASTDDS_DEFAULT_PROFILES_FILE and
        # removed the <fastrtps/...> extern that previously backed a runtime
        # sanity check, so both sides hardcode the 3.x value.
        xml_profiles_env_variable = "FASTDDS_DEFAULT_PROFILES_FILE"

        # Transport tuning via FASTDDS_BUILTIN_TRANSPORTS — the SWIG bindings expose
        # neither setup_transports()'s argument types nor the transport descriptors, so
        # the env variable is the only lever. Mirrors src/domain_participant.cpp:
        #   - Enlarge the UDP socket buffers (sockets_size) so large samples (camera
        #     frames, point clouds) survive reliable delivery without the receive buffer
        #     overflowing. A ceiling, clamped by net.core.rmem_max/wmem_max; applied on
        #     every platform and in both transport modes (the UDP path carries any
        #     cross-host peer regardless of shared memory).
        #   - Shared memory: on by default on Linux; off on Windows/macOS (Fast-DDS's
        #     bundled Boost.Interprocess leaks SHM segments/semaphores — assertion
        #     failures on Windows, kern.sysv.shmmni exhaustion hangs on macOS); off on
        #     any platform when TransportMode.UDP_ONLY is requested.
        # setdefault: an externally-set FASTDDS_BUILTIN_TRANSPORTS always wins, and the
        # first participant created in the process fixes it for the rest (env is global).
        @staticmethod
        def _builtin_transports_value(transport):
            disable_shm = (transport == TransportMode.UDP_ONLY) or sys.platform in ("win32", "darwin")
            kind = "UDPv4" if disable_shm else "DEFAULT"
            return f"{kind}?sockets_size={_resolve_udp_socket_buffer_size()}"

        def __init__(self, domain_id, initial_discovery_callback=None,
                     initial_discovery_kinds=None, transport=TransportMode.AUTOMATIC):
            self._cleaned_up = False
            self._domain_id = domain_id

            # Set before create_participant so Fast-DDS picks it up. FASTDDS_BUILTIN_TRANSPORTS
            # is process-global, so the FIRST participant (or an external setting) fixes it for
            # every later one — unlike C++, which applies transports per-participant via
            # setup_transports(). When the caller explicitly asked for a non-AUTOMATIC transport
            # but the env var is already set to a different value, that request will be silently
            # ignored (e.g. a UDP_ONLY participant created to dodge SHM still inherits SHM), so
            # warn rather than mislead. AUTOMATIC means "no preference" — never warn for it.
            desired_transports = _DomainParticipant._builtin_transports_value(transport)
            existing_transports = os.environ.get("FASTDDS_BUILTIN_TRANSPORTS")
            if (
                transport != TransportMode.AUTOMATIC
                and existing_transports is not None
                and existing_transports != desired_transports
            ):
                _network_recovery._emit_log(
                    _network_recovery.LogLevel.WARNING,
                    f"transport={transport} requested but FASTDDS_BUILTIN_TRANSPORTS is already "
                    f"set to '{existing_transports}' (it is process-global — fixed by the first "
                    f"participant created in this process or by an external setting). This "
                    f"participant keeps the existing transport, NOT the requested "
                    f"'{desired_transports}'.",
                )
            os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", desired_transports)

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
                discovery_config = self._participant_qos.wire_protocol().builtin.discovery_config
                discovery_config.initial_announcements.count = (
                    _resolve_discovery_initial_announcement_count()
                )
                discovery_config.initial_announcements.period = (
                    _resolve_discovery_initial_announcement_period()
                )
                discovery_config.leaseDuration = _resolve_discovery_lease_duration()
                announcement_period = _resolve_discovery_announcement_period()
                discovery_config.leaseDuration_announcementperiod = announcement_period
                # Fast-DDS guidance: the periodic re-announcement must stay shorter than the lease
                # duration, or a peer can be declared lost in the gap between announcements. The
                # default 3 s is well clear of the 30 s default lease; warn only if an env override
                # crossed it. Mirrors src/domain_participant.cpp.
                if announcement_period.to_ns() >= discovery_config.leaseDuration.to_ns():
                    _network_recovery._emit_log(
                        _network_recovery.LogLevel.WARNING,
                        f"{_ANNOUNCEMENT_PERIOD_MS_ENV} ({announcement_period.to_ns() // 1000000} ms) is >= "
                        f"the participant lease duration "
                        f"({discovery_config.leaseDuration.to_ns() // 1000000} ms); peers may be declared "
                        f"lost between announcements",
                    )

            # Type / topic registries. Initialised BEFORE create_participant
            # below: an initial_discovery_callback can fire as soon as the
            # participant is created (even during the tail of __init__), and such
            # a callback may call is_known_type() / known_types(), which read
            # these. Defining them afterwards would race into AttributeError.
            self._register_type_mutex = threading.Lock()
            self._registered_types = dict()
            self._register_topic_mutex = threading.Lock()
            self._registered_topics = dict()

            # Deferred-subscriber registry (match-publisher default). A subscriber
            # left at the match sentinel parks a NON-owning weakref here, keyed by
            # topic, until a matching writer is discovered; the build is then hopped
            # onto the process-wide reaper (see _build_deferred_subscriber /
            # _dispatch_deferred_build). The participant owns no build thread of its
            # own — it only routes discovery — mirroring the C++ participant's
            # non-owning registry. _deferred_lock is a leaf (never nested under the
            # lifecycle / registration locks).
            self._deferred_lock = threading.Lock()
            # topic -> list[weakref[Subscriber]] parked awaiting the first writer.
            self._parked_subscribers = {}
            # Per-topic discovered-writer reliability state for match-publisher resolution.
            # Consolidates the former parallel _writer_reliability / _writer_counts dicts into one
            # entry per topic, and tracks live writers PER reliability kind so the adopted value
            # can be re-derived if the writer that fixed it leaves while differently-configured
            # writers remain. topic -> {"adopted": kind, "live_counts": {kind: count}}:
            #  * "adopted": the reliability a match-mode subscriber on this topic adopts. Match-
            #    first — the first writer on an otherwise-writer-less topic fixes it and it stays
            #    while that writer (and its kind) remain; re-derived only once no live writer
            #    offers it anymore (see _on_writer_removed). A subscriber that registers AFTER a
            #    writer was discovered resolves immediately from this value.
            #  * "live_counts": currently-discovered writers keyed by offered reliability kind
            #    (incremented on a discovered DataWriter, decremented on a removed one). A kind is
            #    dropped at count 0 and the whole topic entry is dropped when it empties, so the
            #    dict can't grow without bound as topics churn and an empty/absent entry cleanly
            #    means "no writer (still) on this topic".
            self._writer_reliability = {}

            # Discovery listener — installed EAGERLY (always), unlike before.
            # It now drives the internal match-publisher resolution (a discovered
            # DataWriter resolves any subscriber parked on its topic) in addition
            # to the optional user on_discovered_endpoint callback. Strong
            # reference, because Fast-DDS holds the listener by raw pointer; the
            # same wrapper survives network-recovery reset (re-attached in
            # _reset_hook_locked). The user callback (if any) is layered on top via
            # set_callback; with no user callback the listener still runs the
            # internal resolver and is otherwise a near-zero-cost observer.
            self._discovery_listener = _DiscoveryListener()
            # set_owner BEFORE the listener is attached so the very first discovery
            # callback the new Fast-DDS participant fires can resolve the owner
            # (both the internal resolver and the user callback need it).
            self._discovery_listener.set_owner(self)
            if initial_discovery_callback is not None:
                self._discovery_listener.set_callback(initial_discovery_callback,
                                                     initial_discovery_kinds)

            # Create the participant with the discovery listener attached BEFORE it
            # starts discovery (see _create_participant_with_listener). Attaching the
            # listener microseconds later via set_listener leaves a gap in which an
            # endpoint discovered by the freshly-enabled participant fires the
            # discovery callback with no listener and is lost forever — the listener
            # drives the match-publisher default, so it must never miss a discovered
            # writer. This matches the C++ ctor, which passes the listener into
            # create_fastdds_participant. (The SWIG director can't be handed to
            # create_participant directly — that disowns it and breaks re-attach
            # across resets — so the helper creates the participant disabled, attaches
            # the listener, then enables.)
            self._participant = _create_participant_with_listener(
                factory, domain_id, self._participant_qos, self._discovery_listener
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

            # Network-recovery participation, resolved on construction (the
            # C++ side resolves the env var the same way — once per
            # process, cached).
            self._recovery_enabled = _network_recovery.resolve_network_recovery_enabled(recovery_mode)

            # Set by _reset_hook_locked when a reset left this participant unusable —
            # its Fast-DDS participant could not be recreated, or an endpoint failed to
            # rebuild. The coordinator polls it after every reset and retries the
            # affected participants on its periodic safety-net tick, because nothing in
            # the event-driven path would ever come back to them otherwise. Mirrors the
            # C++ domain_participant::needs_network_recovery_retry().
            self._recovery_retry_needed = False

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
            # No deferred-build worker to stop: deferred builds run on the shared
            # reaper, and a pending/in-flight build holds a strong reference to its
            # Subscriber (which holds a strong reference to this participant), so a
            # build can never be in flight while this participant is torn down by its
            # own refcount reaching zero. The atexit force-cleanup path (references may
            # still exist) is covered by the lifecycle lock plus the generation / None
            # guards here and in _build_deferred_subscriber: a late build simply sees
            # _participant is None (or a bumped generation) and skips. Parked weakrefs
            # expire on their own.
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

        def is_known_type(self, type_name):
            """Returns ``True`` if a type with ``type_name`` has been registered
            on this participant (via :meth:`register_type` directly, or via
            the :class:`Publisher` / :class:`Subscriber` / :class:`Service`
            constructors that call it).

            Intended companion to :meth:`on_discovered_endpoint`: register the
            types you can deserialise up front, then use ``is_known_type`` to
            filter the stream of discovered (topic, type) pairs.

            :param type_name: Wire-format type name to query (e.g.
                ``"std_msgs::msg::dds_::String_"``)."""
            with self._register_type_mutex:
                return type_name in self._registered_types

        def known_types(self):
            """Returns the names of every type currently registered on this
            participant. See :meth:`is_known_type` for usage."""
            with self._register_type_mutex:
                return list(self._registered_types.keys())

        def on_discovered_endpoint(self, callback, kinds=None):
            """Register a callback invoked when a remote DDS endpoint of one
            of the requested ``kinds`` is discovered or removed in the DDS
            domain. Used by callers (e.g. a recorder) that want to dynamically
            subscribe to every topic carrying a known type, rather than
            hard-coding a topic list.

            :param callback: ``callback(participant, topic_name: str,
                type_name: str, kind: EndpointKind, discovered: bool,
                reliability, durability) -> None`` — or ``None`` to unregister
                and uninstall the underlying Fast-DDS listener. The first
                argument is this :class:`_DomainParticipant` wrapper (same
                object the callback was registered on; survives
                network-recovery resets), so the callback can call
                :meth:`is_known_type` / :meth:`known_types` without having to
                capture the participant externally. ``reliability`` and
                ``durability`` are the discovered endpoint's reliability /
                durability QoS kinds (offered for a discovered DataWriter,
                requested for a discovered DataReader) — integer kinds directly
                comparable to the module-level ``RELIABLE_RELIABILITY_QOS`` /
                ``BEST_EFFORT_RELIABILITY_QOS`` and ``VOLATILE_DURABILITY_QOS``
                / ``TRANSIENT_LOCAL_DURABILITY_QOS`` constants — so a recording
                bridge can match QoS per topic.
            :param kinds: An :class:`EndpointKind` (or bitwise-OR of them)
                deciding which kinds the callback fires for. Defaults to
                :attr:`EndpointKind.DATA_WRITER` — the right choice for a
                recorder.

            The Fast-DDS ``DomainParticipantListener`` is installed eagerly at
            construction and stays attached for the participant's whole life — it
            drives the match-publisher subscriber default (a discovered writer
            must resolve any subscriber parked on its topic), so it can no longer
            be lazy. This method only sets or clears the optional user callback
            that rides on that listener; passing ``callback=None`` clears the
            user callback but leaves the listener attached as a near-zero-cost
            internal observer. On a network-recovery reset the listener is
            re-installed on the recreated participant automatically. Because the
            participant holds that single ``DomainParticipantListener`` slot
            unconditionally, you cannot attach a separate listener of your own on
            the same participant.

            The callback runs on the Fast-DDS discovery thread — keep it
            brief; do not create endpoints from inside it (the network-recovery
            reset path is a lock-order hazard), and do not call
            :meth:`on_discovered_endpoint` from within it (re-registering on the
            discovery thread re-enters Fast-DDS' ``set_listener``, which is
            forbidden and can deadlock)."""
            if kinds is None:
                kinds = EndpointKind.DATA_WRITER
            with self._lifecycle_lock:
                if callback is None:
                    # Unregister: clear only the stored USER callback so any
                    # in-flight (or about-to-fire) user dispatch becomes a no-op.
                    # The Fast-DDS listener itself stays attached — it is installed
                    # eagerly to drive the internal match-publisher resolution,
                    # which must keep running after a user unregister. _invoke
                    # snapshots the (now-None) callback and skips the user path,
                    # so no further user events are delivered; the listener
                    # remains a near-zero-cost internal observer. (This differs
                    # from the previous lazy design, which detached the listener
                    # entirely on unregister.)
                    self._discovery_listener.set_callback(None, kinds)
                    return
                self._discovery_listener.set_callback(callback, kinds)
                # The listener is installed at construction and re-attached on
                # reset, so there is nothing to (re-)attach here.

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

        # --- Deferred (match-publisher) subscriber subsystem ----------------
        # Deferred-subscriber routing (match-publisher default). The participant only
        # ROUTES discovery to parked subscribers via a non-owning weakref registry and
        # hops the actual build onto the process-wide reaper — it owns no build thread.
        # Mirrors the C++ domain_participant::register_deferred_subscriber /
        # resolve_deferred_for_writer (run_deferred_build = the reaper task in
        # _build_deferred_subscriber).

        def _register_deferred_subscriber(self, topic_name, subscriber):
            """Park a match-mode subscriber until a matching writer is discovered, or —
            if a writer was already discovered on its topic — dispatch the build
            immediately. Called from Subscriber._build_state under the lifecycle lock;
            takes only _deferred_lock (a leaf), so the order is lifecycle -> deferred."""
            with self._deferred_lock:
                # Park first (idempotent per (topic, subscriber)) even when a writer was already
                # discovered: parking is what lets a FAILED build be retried by the next writer
                # event, and a reset re-dispatch find us — a successful build deregisters itself
                # (_resolve_match_and_build). Idempotent because a network-recovery reset re-runs
                # _build_state for a still-unresolved subscriber; skip an already-parked one rather
                # than accumulate duplicate weakrefs. Mirrors C++ register_deferred_subscriber.
                parked = self._parked_subscribers.setdefault(topic_name, [])
                already_parked = any(weak_sub() is subscriber for weak_sub in parked)
                if not already_parked:
                    parked.append(weakref.ref(subscriber))
                # If a writer was already discovered on this topic, dispatch the build immediately
                # (closes the writer-before-subscriber race) in addition to parking.
                cached = self._writer_reliability.get(topic_name)
                if cached is not None:
                    self._dispatch_deferred_build(subscriber, cached["adopted"])

        def _deregister_deferred_subscriber(self, topic_name, subscriber):
            """Remove a match-mode subscriber's parked weakref. Called from
            ``Subscriber.__del__``. Without this, a subscriber that is destroyed before
            any matching writer is discovered leaks a dead weakref in
            ``_parked_subscribers[topic_name]`` — that entry is only ever pruned by a
            writer event for the exact topic (``_resolve_deferred_for_writer``), which
            for a never-published topic never comes. Takes only ``_deferred_lock`` (a
            leaf), prunes already-dead refs in the same pass, and drops the topic entry
            once empty. Mirrors C++
            ``domain_participant::deregister_deferred_subscriber``. A subscriber that was
            already resolved (and thus already removed) is simply not found — harmless."""
            with self._deferred_lock:
                parked = self._parked_subscribers.get(topic_name)
                if parked is None:
                    return
                remaining = []
                for weak_sub in parked:
                    other = weak_sub()
                    if other is not None and other is not subscriber:
                        remaining.append(weak_sub)
                if remaining:
                    self._parked_subscribers[topic_name] = remaining
                else:
                    self._parked_subscribers.pop(topic_name, None)

        def _resolve_deferred_for_writer(self, topic_name, reliability):
            """Record a discovered writer's reliability (match-first) and dispatch every
            parked subscriber on its topic. Called from the discovery thread (via
            _DiscoveryListener._invoke), before the user callback — so it only touches
            _deferred_lock-guarded state and submits build tasks to the reaper; it never
            builds an endpoint on this thread. Mirrors C++ resolve_deferred_for_writer."""
            with self._deferred_lock:
                # Match-first: the first writer on an otherwise-writer-less topic fixes the
                # adopted reliability; while it (and its kind) remain a later heterogeneous
                # writer does not change it. A fresh entry has empty live_counts (emptied
                # entries are erased), so "was the topic writer-less?" is exactly that.
                state = self._writer_reliability.get(topic_name)
                if state is None:
                    state = {"adopted": reliability, "live_counts": {}}
                    self._writer_reliability[topic_name] = state
                # Track this writer toward the topic's per-reliability live-writer count, so the
                # adopted value can be re-derived / dropped as writers are removed (see
                # _on_writer_removed).
                live_counts = state["live_counts"]
                live_counts[reliability] = live_counts.get(reliability, 0) + 1
                effective = state["adopted"]
                parked = self._parked_subscribers.get(topic_name)
                if not parked:
                    return
                # Dispatch each parked subscriber but KEEP it parked: a build that FAILS (e.g.
                # create_datareader returns None under resource exhaustion) stays parked so the
                # next discovered writer retries it, instead of staying inactive until the next
                # reset. A successful build deregisters itself (_resolve_match_and_build). Prune
                # expired weakrefs in the same pass; drop the topic entry if all refs were dead.
                live = []
                for weak_sub in parked:
                    subscriber = weak_sub()
                    if subscriber is None:
                        continue
                    live.append(weak_sub)
                    self._dispatch_deferred_build(subscriber, effective)
                if live:
                    self._parked_subscribers[topic_name] = live
                else:
                    self._parked_subscribers.pop(topic_name, None)

        def _on_writer_removed(self, topic_name, reliability):
            """Decrement the topic's per-reliability live-writer count. Once the last writer of
            any kind is gone, drop the cached entry so a match-mode subscriber created afterwards
            defers for a fresh writer instead of adopting a stale value (and so the dict can't
            grow without bound as topics churn). If writers remain but no live writer still offers
            the adopted reliability, re-derive it from a still-live kind — otherwise a subscriber
            created now would adopt a reliability matching none of the remaining writers (e.g.
            RELIABLE adopted while only a BEST_EFFORT writer is left, which a RELIABLE reader
            silently fails to match). Called from the discovery thread; takes only _deferred_lock.
            Mirrors C++ domain_participant::on_writer_removed."""
            with self._deferred_lock:
                state = self._writer_reliability.get(topic_name)
                if state is None:
                    return  # a removed-without-counted-discovered event — nothing to do
                live_counts = state["live_counts"]
                count = live_counts.get(reliability)
                if count is None:
                    return  # a removed reliability kind we never counted — nothing to do
                if count > 1:
                    live_counts[reliability] = count - 1
                else:
                    live_counts.pop(reliability, None)
                if not live_counts:
                    # Last writer on this topic (of any kind) is gone — drop the entry.
                    self._writer_reliability.pop(topic_name, None)
                    return
                # Writers remain: re-derive the adopted value if its kind no longer has a live
                # writer. Match-first is preserved while the writer that fixed it is alive; this
                # only fires once it is gone. Any remaining kind is valid — pick one deterministically.
                if state["adopted"] not in live_counts:
                    state["adopted"] = next(iter(live_counts))

        def _dispatch_deferred_build(self, subscriber, reliability):
            """Hop a deferred reader build onto the process-wide reaper. Caller holds
            _deferred_lock; submit_off_thread takes only the reaper's own lock (never a
            participant lock), so there is no inversion. The submitted task holds the
            only strong reference the build needs and drops it on the reaper, so any
            cascading teardown stays off the discovery / Fast-DDS threads (see
            _build_deferred_subscriber)."""
            _network_recovery.submit_off_thread(
                lambda s=subscriber, r=reliability: _build_deferred_subscriber(s, r)
            )

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

                # Drop the match-publisher discovery cache: it tracked writers seen by the
                # now-destroyed participant. The new participant re-discovers currently-present
                # writers and repopulates it, so clearing here prevents the per-reliability writer
                # counts from inflating across resets (the old participant's writers never fire a
                # removal), which would otherwise pin the adopted reliability and grow it without bound.
                with self._deferred_lock:
                    self._writer_reliability.clear()

                # Refresh Fast-DDS's interface cache BEFORE recreating —
                # this is the whole point of the auto-recovery reset
                # (without the refresh, the new participant binds to the
                # same stale interfaces as the old one). Mirrors the C++
                # domain_participant::trigger_network_recovery_reset path.
                _network_recovery.refresh_fastdds_interface_cache()

                # Recreate with the same QoS, attaching the discovery listener
                # BEFORE the new participant starts discovery (see
                # _create_participant_with_listener). This is the load-bearing part
                # of the fix for survives_reset: the peer's writer already exists, so
                # the recreated participant can rediscover it the instant it enables.
                # If the listener were attached a step later (the old
                # create_participant-then-set_listener sequence), a rediscovery event
                # in that gap fires with no listener and is lost forever — stranding
                # the match-publisher resolver and any user callback until the next
                # reset. The listener is always present (it drives the match-publisher
                # default even with no user callback). An unresolved match-mode
                # subscriber re-defers on rebuild and resolves again once a writer is
                # rediscovered against the new participant.
                self._participant = _create_participant_with_listener(
                    factory, self._domain_id, self._participant_qos, self._discovery_listener
                )
                if self._participant is None:
                    _network_recovery._emit_log(
                        _network_recovery.LogLevel.ERROR,
                        f"failed to recreate participant on domain {self._domain_id}; "
                        f"endpoints left in torn-down state, will be retried by the "
                        f"network-recovery safety-net check",
                    )
                    # Bump generation anyway so any racing teardown observes the mismatch.
                    self._generation += 1
                    # Flag for the coordinator: this participant is inert until a later
                    # attempt succeeds, and no further network event is guaranteed.
                    self._recovery_retry_needed = True
                    return

                self._generation += 1

                # Re-register all known types against the new participant.
                with self._register_type_mutex:
                    for type_name, type_support in self._registered_types.items():
                        self._participant.register_type(type_support)

                # Phase 5: rebuild each endpoint.
                any_endpoint_failed = False
                for (_, endpoint) in live:
                    try:
                        endpoint._rebuild_state_after_reset(self._participant)
                    except Exception as ex:
                        any_endpoint_failed = True
                        _network_recovery._emit_log(
                            _network_recovery.LogLevel.ERROR,
                            f"endpoint rebuild failed during reset: {ex}; this endpoint stays "
                            f"inactive until the network-recovery safety-net check retries the reset",
                        )

                # The participant itself is healthy either way; an endpoint that failed
                # to come back still needs another attempt, which the coordinator's
                # safety-net tick will make (a later network event alone cannot be
                # relied on). See _NetworkRecoveryCoordinator._on_safety_net_tick.
                self._recovery_retry_needed = any_endpoint_failed

    participant = _DomainParticipant(domain_id, initial_discovery_callback,
                                     initial_discovery_kinds, transport)
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

                        # A raising user callback must not escape into the Fast-DDS
                        # listener thread (a SWIG DirectorMethodException ->
                        # std::terminate). Report it through the configurable logger and
                        # continue; the finally below still drops the transient Publisher
                        # reference.
                        try:
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
                        except Exception as exception:  # noqa: BLE001
                            _network_recovery._emit_log(
                                _network_recovery.LogLevel.ERROR,
                                f"publisher on_has_subscriber_changed callback threw: {exception}",
                            )
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
        history_depth: int = USE_DEFAULT_HISTORY_DEPTH,
        durability_kind: Optional[Any] = None,
    ):
        """Constructs a DDS Publisher

        :param domain_participant: A DDS Domain Participant wrapper object, as created by provizio_dds.make_domain_participant
        :param str topic_name: A string DDS Topic name
        :param pub_sub_type: The DDS PubSub Type to be published, f.e. provizio_dds.StringPubSubType
        :param on_has_subscriber_changed_function: Optional, a function to be invoked on matching first / unmatching last subscriber, takes two arguments: a Publisher and a bool: True when the first subscriber is matched, False when the last subscriber is unmatched; Note: called from a background Thread
        :param reliability_kind: Optional, a DDS data writer reliability kind to be used: either BEST_EFFORT_RELIABILITY_QOS or RELIABLE_RELIABILITY_QOS; if not specified, QosDefaults for pub_sub_type will be used
        :param int history_depth: Controls the KEEP_LAST history depth only (no longer tied to durability): USE_DEFAULT_HISTORY_DEPTH (-1) or any non-positive value uses the default depth (the per-type QosDefaults.keep_last_history_depth if specialized for pub_sub_type, otherwise the Fast-DDS default); a positive value sets KEEP_LAST history with that depth. Configure durability separately via durability_kind.
        :param durability_kind: Optional, a DDS durability kind to be used (e.g. VOLATILE_DURABILITY_QOS or TRANSIENT_LOCAL_DURABILITY_QOS); if not specified, the Fast-DDS / XML default durability is kept. Mirrors reliability_kind — independent of history_depth.
        """

        super().__init__(domain_participant, topic_name, pub_sub_type)

        qos_defaults = QosDefaults(pub_sub_type)
        if reliability_kind is None:
            reliability_kind = qos_defaults.datawriter_reliability_kind

        # Captured parameters for replay on reset.
        self._captured_reliability_kind = reliability_kind
        self._captured_history_depth = history_depth
        self._captured_durability_kind = durability_kind
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
        # History (untied from durability): an explicit positive depth wins, else fall
        # back to the per-type default (0 = leave the Fast-DDS default). KEEP_LAST only —
        # durability is configured independently below, so this is not an RxO QoS (ROS 2
        # interop unaffected). Mirrors the C++ make_publisher.
        effective_history_depth = (
            self._captured_history_depth
            if self._captured_history_depth > 0
            else self._captured_qos_defaults.keep_last_history_depth
        )
        if effective_history_depth > 0:
            writer_qos.history().kind = KEEP_LAST_HISTORY_QOS
            writer_qos.history().depth = effective_history_depth
        # Durability is independent of history: applied only if the caller requested it,
        # otherwise the Fast-DDS / XML default is preserved. Mirrors reliability_kind.
        if self._captured_durability_kind is not None:
            writer_qos.durability().kind = self._captured_durability_kind
        # Publish mode is writer-local (per-type default; ASYNCHRONOUS for large sample
        # types). Not an RxO QoS, so it never affects matching or ROS 2 interop.
        writer_qos.publish_mode().kind = self._captured_qos_defaults.datawriter_publish_mode

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
                    # A raising user callback must not escape into the Fast-DDS reception
                    # thread (a SWIG DirectorMethodException -> std::terminate). Report it
                    # through the configurable logger and carry on with the next sample.
                    try:
                        if self._on_data_takes_info:
                            self._on_data_function(data, info)
                        else:
                            self._on_data_function(data)
                    except Exception as exception:  # noqa: BLE001
                        _network_recovery._emit_log(
                            _network_recovery.LogLevel.ERROR,
                            f"subscriber on_data callback threw: {exception}",
                        )

        def on_subscription_matched(self, _, info):
            with _fastdds_callback_scope(), self._drain.scope() as should_run:
                if not should_run:
                    return
                self._update_num_matched_publishers(info.current_count)

                if self._on_has_publisher_changed_function:
                    # A raising user callback must not escape into the Fast-DDS listener
                    # thread (a SWIG DirectorMethodException -> std::terminate). Report it
                    # through the configurable logger and continue.
                    try:
                        if (
                            info.current_count > 0
                            and info.current_count_change == info.current_count
                        ):
                            # Just matched the first publisher
                            self._on_has_publisher_changed_function(True)
                        elif info.current_count == 0 and info.current_count_change < 0:
                            # Just unmatched the last publisher
                            self._on_has_publisher_changed_function(False)
                    except Exception as exception:  # noqa: BLE001
                        _network_recovery._emit_log(
                            _network_recovery.LogLevel.ERROR,
                            f"subscriber on_has_publisher_changed callback threw: {exception}",
                        )

    def __init__(
        self,
        domain_participant: object,
        topic_name: str,
        pub_sub_type: TopicDataType,
        data_type: object,
        on_data_function: Callable,
        on_has_publisher_changed_function: Optional[Callable[[bool], Any]] = None,
        reliability_kind: Optional[Any] = None,
        max_history_depth: int = USE_DEFAULT_HISTORY_DEPTH,
        durability_kind: Optional[Any] = None,
    ):
        """Constructs a DDS Subscriber

        :param domain_participant: A DDS Domain Participant wrapper object, as created by provizio_dds.make_domain_participant
        :param str topic_name: A string DDS Topic name
        :param pub_sub_type: The DDS PubSub Type to be received, f.e. provizio_dds.StringPubSubType
        :param data_type: The DDS Data Type to be received, f.e. provizio_dds.String
        :param on_data_function: A function to be invoked on receiving published data. It can take one argument (the data) or two arguments (data and a SampleInfo object). Note: called from a background Thread
        :param on_has_publisher_changed_function: Optional, a function to be invoked on matching first / unmatching last publisher, takes a single bool argument: True when the first publisher is matched, False when the last publisher is unmatched; Note: called from a background Thread
        :param reliability_kind: Optional, a DDS data reader reliability kind to be used: either BEST_EFFORT_RELIABILITY_QOS or RELIABLE_RELIABILITY_QOS; if not specified, QosDefaults for pub_sub_type will be used
        :param int max_history_depth: Controls the KEEP_LAST history depth only (no longer tied to durability): USE_DEFAULT_HISTORY_DEPTH (-1) or any non-positive value uses the default depth (the per-type QosDefaults.keep_last_history_depth if specialized for pub_sub_type, otherwise the Fast-DDS default); a positive value sets KEEP_LAST history with that depth. Configure durability separately via durability_kind.
        :param durability_kind: Optional, a DDS durability kind to be used (e.g. VOLATILE_DURABILITY_QOS or TRANSIENT_LOCAL_DURABILITY_QOS); if not specified, the Fast-DDS / XML default durability is kept. Mirrors reliability_kind — independent of max_history_depth.
        """
        super().__init__(domain_participant, topic_name, pub_sub_type)

        qos_defaults = QosDefaults(pub_sub_type)
        if reliability_kind is None:
            reliability_kind = qos_defaults.datareader_reliability_kind

        self._captured_data_type = data_type
        self._captured_reliability_kind = reliability_kind
        self._captured_max_history_depth = max_history_depth
        self._captured_durability_kind = durability_kind
        self._captured_qos_defaults = qos_defaults

        # Match-publisher mode: when reliability_kind == MATCH_PUBLISHER_RELIABILITY_QOS the
        # DataReader is NOT created until a matching writer is discovered; this holds the
        # reliability adopted from that writer once resolved. None while still deferred /
        # unresolved. Reader QoS reliability is immutable after enable, so the reader can't
        # be created speculatively and patched later — it must wait for the offered value.
        # Mirrors the C++ subscriber_handle::resolved_match_reliability.
        self._resolved_match_reliability = None

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

    def _effective_reliability(self):
        """The reliability the reader will be built with: the one adopted from a
        discovered writer if resolved, otherwise the captured kind. While the
        captured kind is MATCH_PUBLISHER_RELIABILITY_QOS and nothing is resolved
        yet, this is the match sentinel — meaning "still deferred". Mirrors the
        C++ subscriber_handle::effective_reliability()."""
        if self._resolved_match_reliability is not None:
            return self._resolved_match_reliability
        return self._captured_reliability_kind

    def _build_state(self, fastdds_participant):
        """(Re)create the Fast-DDS Subscriber + DataReader against
        @c fastdds_participant. Caller holds the lifecycle lock."""
        # Match-publisher mode: when the effective reliability is still the match sentinel
        # (no matching writer discovered yet), DO NOT create the Subscriber / DataReader.
        # Reader QoS reliability is immutable after enable, so we can't create now and fix it
        # later — we must wait for the writer's offered reliability. Register with the
        # participant's deferred-subscriber registry; once a writer is seen it submits our
        # build to the process-wide reaper (OFF the Fast-DDS discovery thread), which calls
        # _resolve_match_and_build -> _build_state again with _resolved_match_reliability set.
        # Re-entrant on a network-recovery reset: an unresolved match-mode subscriber simply
        # re-defers here; a resolved one falls through and rebuilds with the adopted
        # reliability. Mirrors the C++ subscriber_handle::build_state.
        effective_reliability_kind = self._effective_reliability()
        if effective_reliability_kind == MATCH_PUBLISHER_RELIABILITY_QOS:
            self._participant._register_deferred_subscriber(self._topic_name, self)
            return

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
        reader_qos.reliability().kind = effective_reliability_kind
        reader_qos.endpoint().history_memory_policy = self._captured_qos_defaults.memory_policy
        if sys.platform in ("win32", "darwin"):
            reader_qos.data_sharing().off()
        # History (untied from durability): an explicit positive depth wins, else fall
        # back to the per-type default (0 = leave the Fast-DDS default). KEEP_LAST only —
        # durability is configured independently below, so this is not an RxO QoS (ROS 2
        # interop unaffected). Mirrors the C++ make_subscriber.
        effective_max_history_depth = (
            self._captured_max_history_depth
            if self._captured_max_history_depth > 0
            else self._captured_qos_defaults.keep_last_history_depth
        )
        if effective_max_history_depth > 0:
            reader_qos.history().kind = KEEP_LAST_HISTORY_QOS
            reader_qos.history().depth = effective_max_history_depth
        # Durability is independent of history: applied only if the caller requested it,
        # otherwise the Fast-DDS / XML default is preserved. Mirrors reliability_kind.
        if self._captured_durability_kind is not None:
            reader_qos.durability().kind = self._captured_durability_kind

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

    def _resolve_match_and_build(self, resolved_reliability, fastdds_participant):
        """Adopt @c resolved_reliability for a deferred (match-publisher) reader and
        build it. Called on the process-wide reaper (via _build_deferred_subscriber),
        which has already taken the registration mutex + lifecycle lock and passes
        the current Fast-DDS participant (same contract as _register_endpoint's
        initial build) — so this must NOT re-acquire those locks.

        Idempotent: a network-recovery reset occurring BEFORE the writer is
        discovered re-defers this subscriber, so the same handle can be registered —
        and later dispatched — more than once. If the reader already exists it was
        already resolved and built; do nothing rather than build a second reader and
        orphan the first. Mirrors the C++ subscriber_handle::resolve_match_and_build
        idempotency guard (`if (data_reader != nullptr) return;`).

        Either way, once a reader exists the build has succeeded, so deregister our parked
        back-reference to stop the retry loop. _build_state raises on failure (caught + logged
        by _build_deferred_subscriber), which deliberately leaves us parked so the next
        discovered writer on this topic retries the build. _deregister_deferred_subscriber takes
        only _deferred_lock (a leaf), a valid acquire under the held registration + lifecycle
        locks. Mirrors C++ build_deferred_locked's deregister-on-success."""
        if self._reader is not None:
            self._participant._deregister_deferred_subscriber(self._topic_name, self)
            return
        self._resolved_match_reliability = resolved_reliability
        self._build_state(fastdds_participant)
        self._participant._deregister_deferred_subscriber(self._topic_name, self)

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
            # Match-mode subscribers park a non-owning weakref in the participant's deferred
            # registry; remove ours so a never-matched subscriber doesn't leak a dead weakref
            # there (mirrors C++ ~subscriber_handle's deregister_deferred_subscriber). Only
            # subscribers created with the match sentinel ever register (see _build_state), so
            # guard on _captured_reliability_kind to skip the _deferred_lock on every explicit-
            # reliability teardown.
            if self._captured_reliability_kind == MATCH_PUBLISHER_RELIABILITY_QOS:
                try:
                    participant._deregister_deferred_subscriber(self._topic_name, self)
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
        """Return the stable number of matched publishers or -1 if unstable until timeout.

        For a match-publisher (default) subscriber the DataReader is not created until a
        matching writer is discovered. This call honours ``timeout_sec`` in that state too:
        the listener (and its cv) exist before — and survive — the reader, so it blocks
        waiting for a writer to be discovered, the deferred reader to be built, and the
        first match, rather than returning 0 at t=0 while reliability discovery is still in
        progress. (If no writer ever appears the wait simply times out and returns 0.)
        Mirrors the C++ subscriber_handle::get_num_matched_publishers.
        """

        # Read the reader pointer + generation under the lifecycle lock so this can't race a
        # concurrent _build_state swapping the reader in. A generation mismatch is treated as
        # "no usable reader": in the missed-snapshot reset scenario the reader can be non-None
        # but point into an already-freed generation, so its listener count is stale — return 0
        # (mirrors get_guid). A None reader is the deferred match-publisher case: fall through
        # to the timed wait below so a writer discovered within the timeout is observed.
        with self._participant.lifecycle_lock():
            if (
                self._reader is not None
                and self._built_against_generation != self._participant.participant_generation()
            ):
                return 0

        listener = self._listener
        return _get_stable_match_count(
            listener._num_matched_cv,
            lambda: listener._num_matched_publishers,
            timeout_sec,
            settle_time_sec,
        )


async def _await_stable_match(
    loop,
    get_num_subscribers,
    get_num_publishers,
    stable_matches_period_sec,
    service_match_timeout_sec,
):
    """Await request-writer/response-reader readiness, then a settling window.

    Mirrors ROS 2 rmw_fastrtps' rmw_service_server_is_available (writer and reader both matched, in equal
    counts), then holds until the counts are stable for stable_matches_period_sec (capped at
    _SETTLE_CAP_MULTIPLIER x the window after the first match, so churn can't delay readiness indefinitely).
    The settle also lets all services on the topic be discovered -- provizio routes multiple sensors over one
    service name, filtered by frame_id -- so a request reaches all of them. Counts are read as instantaneous
    snapshots. Raises ServiceMatchingTimeoutError if service_match_timeout_sec (> 0) elapses first; 0 waits
    indefinitely. Shared by request() and ServiceClient.
    """
    settle = (
        stable_matches_period_sec
        if stable_matches_period_sec is not None and stable_matches_period_sec > 0.0
        else 0.0
    )
    settle_cap = settle * _SETTLE_CAP_MULTIPLIER
    match_deadline = (
        time.monotonic() + service_match_timeout_sec
        if service_match_timeout_sec and service_match_timeout_sec > 0.0
        else None
    )

    prev_counts = (None, None)
    last_change = time.monotonic()
    first_match = None

    while True:
        # Instantaneous snapshots of the current match counts.
        pub_task = loop.run_in_executor(None, lambda: get_num_subscribers(0.0, 0.0))
        sub_task = loop.run_in_executor(None, lambda: get_num_publishers(0.0, 0.0))
        subscribers, publishers = await asyncio.gather(pub_task, sub_task)
        now = time.monotonic()

        if (subscribers, publishers) != prev_counts:
            last_change = now
            prev_counts = (subscribers, publishers)

        if subscribers > 0 and publishers > 0 and subscribers == publishers:
            if first_match is None:
                first_match = now
            # Stable for the settling window since the last change, or capped after the first match so
            # churn can't delay readiness indefinitely.
            if (
                settle <= 0.0
                or (now - last_change) >= settle
                or (now - first_match) >= settle_cap
            ):
                return

        if match_deadline is not None and now >= match_deadline:
            raise ServiceMatchingTimeoutError(
                f"Request endpoints did not become ready within {service_match_timeout_sec}s "
                f"(endpoints must match and then stay stable for the "
                f"{settle}s settling window)"
            )

        # Poll period while waiting for the first match / for the settle to elapse.
        await asyncio.sleep(_MIN_MATCH_WAIT_SEC)


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
    durability_kind: Optional[Any] = None,
    endpoint_history_depth: int = USE_DEFAULT_HISTORY_DEPTH,
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
    :param durability_kind: Optional, a DDS durability kind (e.g. VOLATILE_DURABILITY_QOS or TRANSIENT_LOCAL_DURABILITY_QOS) applied to the request Publisher and response Subscriber; None keeps the current default (volatile / Fast-DDS / XML default)
    :param int endpoint_history_depth: KEEP_LAST history depth for the request/response endpoints; USE_DEFAULT_HISTORY_DEPTH (-1) keeps the current default
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

    # Client side: the request Publisher and response Subscriber both pass the
    # endpoint params straight through (None / -1 reproduces the historical
    # volatile + default-history behaviour). Mirrors the C++ service_client_basic
    # constructor in request_response_details.h.
    response_subscriber = Subscriber(
        domain_participant,
        response_topic_name,
        response_pub_sub_type,
        response_data_type,
        on_response,
        reliability_kind=RELIABLE_RELIABILITY_QOS,
        max_history_depth=endpoint_history_depth,
        durability_kind=durability_kind,
    )

    request_publisher = Publisher(
        domain_participant,
        request_topic_name,
        request_pub_sub_type,
        reliability_kind=RELIABLE_RELIABILITY_QOS,
        history_depth=endpoint_history_depth,
        durability_kind=durability_kind,
    )

    async def _wait_for_matching():
        # Readiness + settling window logic lives in _await_stable_match (shared with ServiceClient).
        await _await_stable_match(
            loop,
            request_publisher.get_num_matched_subscribers,
            response_subscriber.get_num_matched_publishers,
            stable_matches_period_sec,
            service_match_timeout_sec,
        )

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


class ServiceClient:
    """Persistent request/response client (mirrors C++ provizio::dds::service_client).

    Create the client in advance; it matches the service(s) over time. ``request`` is awaitable and many
    requests may be in flight concurrently (e.g. via ``asyncio.gather``), each resolving to its own response.
    Must be created within a running asyncio event loop. Unlike ROS 2, provizio routes multiple services over
    one service name (by frame_id), so ``wait_for_service`` waits for the settling window, not just the first
    match.

    One of (request_topic_name, response_topic_name) or service_name must be provided.
    """

    def __init__(
        self,
        domain_participant: object,
        request_pub_sub_type,
        response_pub_sub_type,
        response_data_type,
        request_topic_name: str = None,
        response_topic_name: str = None,
        service_name: str = None,
        stable_matches_period_sec: float = _DEFAULT_STABLE_MATCH_WINDOW_SEC,
        service_match_timeout_sec: float = 0.0,
        durability_kind: Optional[Any] = None,
        endpoint_history_depth: int = USE_DEFAULT_HISTORY_DEPTH,
    ):
        # Validate with raise, not assert: `python -O` strips asserts, after which a missing
        # service_name would surface as a confusing TypeError in the string concatenation below
        # instead of a clear argument error.
        if request_topic_name is None:
            if service_name is None:
                raise ValueError("Either of request_topic_name or service_name are required")
            request_topic_name = _request_prefix + service_name + _request_suffix
        if response_topic_name is None:
            if service_name is None:
                raise ValueError("Either of response_topic_name or service_name are required")
            response_topic_name = _response_prefix + service_name + _response_suffix

        self._loop = asyncio.get_running_loop()
        self._stable_matches_period_sec = stable_matches_period_sec
        self._service_match_timeout_sec = service_match_timeout_sec
        self._pending = []  # list of (SampleIdentity, asyncio.Future); loop-thread only
        self._matching = None  # cached readiness task (construction params)
        self._closed = False

        # A ServiceClient issues many requests over one request Publisher / response Subscriber pair, so a
        # depth-1 KEEP_LAST default would let a burst of concurrent requests/responses overwrite undelivered
        # samples. Default the endpoints to KEEP_LAST(SERVICE_CLIENT_DEFAULT_HISTORY_DEPTH) -- a bounded buffer
        # matching the service's default response history -- so a burst up to that many is retained and the
        # reliable writer applies back-pressure beyond it. Raise endpoint_history_depth for higher concurrency.
        # Mirrors the C++ service_client.
        effective_history_depth = (
            SERVICE_CLIENT_DEFAULT_HISTORY_DEPTH
            if endpoint_history_depth == USE_DEFAULT_HISTORY_DEPTH
            else endpoint_history_depth
        )

        self._response_subscriber = Subscriber(
            domain_participant,
            response_topic_name,
            response_pub_sub_type,
            response_data_type,
            self._on_response,
            reliability_kind=RELIABLE_RELIABILITY_QOS,
            max_history_depth=effective_history_depth,
            durability_kind=durability_kind,
        )
        self._request_publisher = Publisher(
            domain_participant,
            request_topic_name,
            request_pub_sub_type,
            reliability_kind=RELIABLE_RELIABILITY_QOS,
            history_depth=effective_history_depth,
            durability_kind=durability_kind,
        )

    def _on_response(self, data, info):
        # Fast-DDS reception thread: copy the related identity (info is reused between calls) and hand off to
        # the loop thread. No cross-thread lock around the identities, so no AB-BA with publish(); mirrors the
        # one-shot request().
        related = SampleIdentity(info.related_sample_identity)
        self._loop.call_soon_threadsafe(self._route, data, related)

    def _route(self, data, related):
        # Loop thread only -- touches self._pending, otherwise touched only by request() (also loop thread).
        # The matched entry is removed by the future's done-callback (which also covers a caller cancelling
        # the await, e.g. an asyncio.wait_for timeout), so we only set the result here.
        for _index, (identity, future) in enumerate(self._pending):
            if related == identity:
                if not future.done():
                    future.set_result(data)
                return

    async def _ensure_ready(self):
        # Cache the construction-parameter readiness so requests issued before matching simply await it
        # (asyncio is the deferral queue) and later requests return instantly.
        if self._matching is None:
            self._matching = self._loop.create_task(
                _await_stable_match(
                    self._loop,
                    self._request_publisher.get_num_matched_subscribers,
                    self._response_subscriber.get_num_matched_publishers,
                    self._stable_matches_period_sec,
                    self._service_match_timeout_sec,
                )
            )
        matching = self._matching
        try:
            await matching
        except BaseException:
            # Don't cache a FAILED readiness. With a finite service_match_timeout_sec the
            # task raises ServiceMatchingTimeoutError once it elapses; leaving it cached would
            # make every later request() re-await the same completed-with-exception task and
            # re-raise forever — even after the service comes up (and inconsistently with
            # wait_for_service, which polls afresh). So if the shared task itself finished with
            # an exception, drop it so the next request() re-creates and re-polls it. But if
            # only THIS await was cancelled (e.g. the caller's own asyncio.wait_for timeout)
            # while the shared task is still running, leave it cached for the other concurrent
            # requests still awaiting it.
            if (
                matching.done()
                and not matching.cancelled()
                and matching.exception() is not None
                and self._matching is matching
            ):
                self._matching = None
            raise

    async def wait_for_service(
        self,
        timeout_sec: float = 0.0,
        stable_matches_period_sec: float = _DEFAULT_STABLE_MATCH_WINDOW_SEC,
    ) -> bool:
        """Await readiness (matched and stable for stable_matches_period_sec), bounded by timeout_sec.

        Returns True once ready, False if timeout_sec elapses first. 0 waits indefinitely. Performs a live
        poll (independent of request()'s cached readiness), so the settling window applies -- important for
        provizio's multiple-services-per-topic routing.
        """
        try:
            await _await_stable_match(
                self._loop,
                self._request_publisher.get_num_matched_subscribers,
                self._response_subscriber.get_num_matched_publishers,
                stable_matches_period_sec,
                timeout_sec,
            )
            return True
        except ServiceMatchingTimeoutError:
            return False

    async def request(self, request_data):
        """Send a request and await its response. Many may run concurrently (e.g. asyncio.gather).

        :param request_data: Concrete data instance to publish as the request
        :returns: The response data instance
        :raises RuntimeError: If the client has been closed
        :raises ServiceMatchingTimeoutError: If endpoints fail to match within the timeout
        :raises RequestPublishError: If publishing the request fails
        """
        if self._closed:
            raise RuntimeError("ServiceClient is closed")
        await self._ensure_ready()
        # Publish and register the pending request synchronously on the loop thread (no await between). A
        # response delivered intra-process during publish() reaches _route only via call_soon_threadsafe, so
        # it cannot run until this coroutine yields at `await future` -- by then the entry is registered.
        response_guid = self._response_subscriber.get_guid()
        params = WriteParams()
        params.related_sample_identity().writer_guid(response_guid)
        if not self._request_publisher.publish(request_data, params):
            raise RequestPublishError("Failed to publish the DDS request")
        identity = SampleIdentity()
        identity.writer_guid(response_guid)
        identity.sequence_number(params.sample_identity().sequence_number())
        future = self._loop.create_future()
        self._pending.append((identity, future))
        # Drop the pending entry once the future completes -- whether it resolved with a response or the
        # caller cancelled the await (e.g. an asyncio.wait_for timeout) -- so _pending never leaks.
        future.add_done_callback(self._discard_pending)
        return await future

    def _discard_pending(self, future):
        # Loop thread (asyncio runs done-callbacks on the loop). Remove this future's entry, if still present.
        self._pending = [entry for entry in self._pending if entry[1] is not future]

    def _shutdown(self):
        # Loop thread only -- cancelling asyncio tasks/futures is not thread-safe. Cancels the
        # background matching task and fails any still-outstanding requests.
        if self._matching is not None and not self._matching.done():
            self._matching.cancel()
        self._matching = None
        for _identity, future in self._pending:
            if not future.done():
                future.cancel()
        self._pending = []

    def close(self):
        """Cancel the background matching task, fail outstanding requests, and release the endpoints."""
        if self._closed:
            return
        self._closed = True
        # The matching task and pending futures may only be cancelled on the loop thread, but close()
        # can run on any thread -- notably __del__, which the garbage collector may invoke on an
        # arbitrary thread. Marshal the cancellation onto the loop thread unless we are already on it.
        # A closed loop has no surviving tasks/futures and rejects callbacks, so there is nothing to
        # cancel and the hop is skipped.
        if not self._loop.is_closed():
            try:
                on_loop_thread = asyncio.get_running_loop() is self._loop
            except RuntimeError:
                on_loop_thread = False
            if on_loop_thread:
                self._shutdown()
            else:
                try:
                    self._loop.call_soon_threadsafe(self._shutdown)
                except RuntimeError:
                    # The loop closed between the is_closed() check above and now; nothing to cancel.
                    pass
        self._request_publisher = None
        self._response_subscriber = None

    def __del__(self):
        try:
            self.close()
        except Exception:  # noqa: BLE001 -- best-effort GC cleanup must never raise
            pass


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
            queue_full = False
            with self._cv:
                if self._requests_queue.qsize() < self._max_queue_size:
                    self._requests_queue.put((request, identity))
                    self._cv.notify()
                else:
                    queue_full = True
            # Logged through the configurable logger (not stdout), outside the lock so a
            # user log callback may safely re-enter provizio_dds. Mirrors the C++
            # request_handler's log_error() for a full queue.
            if queue_full:
                _network_recovery._emit_log(
                    _network_recovery.LogLevel.ERROR,
                    "service requests queue is full; dropping request",
                )

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
                except Exception as exception:  # noqa: BLE001
                    # A raising handler must not kill this worker thread — that would
                    # silently stop the service. Report it through the configurable
                    # logger and keep processing the next request.
                    _network_recovery._emit_log(
                        _network_recovery.LogLevel.ERROR,
                        f"service request handler threw: {exception}; request dropped",
                    )

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
                _network_recovery._emit_log(
                    _network_recovery.LogLevel.ERROR,
                    "service requests queue is full; dropping request",
                )

        async def _process_request(self, request, identity):
            try:
                response = await self._handle_request_function(request)
                self._on_response_function(response, identity)
            except Service.IgnoreRequest:
                # Silently drop
                pass
            except Exception as exception:  # noqa: BLE001
                # A raising coroutine handler must not surface as an unhandled asyncio
                # task exception (logged to stderr, outside our facility). Report it
                # through the configurable logger.
                _network_recovery._emit_log(
                    _network_recovery.LogLevel.ERROR,
                    f"service request handler threw: {exception}; request dropped",
                )

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
        max_history_depth: int = USE_DEFAULT_HISTORY_DEPTH,
        durability_kind: Optional[Any] = None,
        endpoint_history_depth: int = USE_DEFAULT_HISTORY_DEPTH,
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
        :param int max_history_depth: The maximum number of requests to queue (the request QUEUE SIZE): a positive value sets the queue size, MINIMAL_REQUEST_QUEUE (0) uses a minimal queue (1), USE_DEFAULT_HISTORY_DEPTH (-1) uses the default (10). This is distinct from endpoint_history_depth (the underlying endpoints' KEEP_LAST history depth).
        :param durability_kind: Optional, a DDS durability kind (e.g. VOLATILE_DURABILITY_QOS or TRANSIENT_LOCAL_DURABILITY_QOS) applied to the underlying request/response endpoints; None keeps the historical defaults (the request Subscriber stays on the Fast-DDS / XML default durability; the response Publisher defaults to TRANSIENT_LOCAL_DURABILITY_QOS so a just-late client's response reader still receives the response).
        :param int endpoint_history_depth: KEEP_LAST history depth for the underlying request/response endpoints; USE_DEFAULT_HISTORY_DEPTH (-1) uses the defaults (the request Subscriber uses the Fast-DDS default; the response Publisher uses the standard default queue depth of 10, decoupled from max_history_depth so a minimal request queue can't shrink it). Distinct from max_history_depth (request queue size).
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
                if max_history_depth == MINIMAL_REQUEST_QUEUE
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

        # Response Publisher history is DECOUPLED from max_history_depth (the request-queue
        # size): sizing the request queue — in particular MINIMAL_REQUEST_QUEUE (0) → 1 —
        # must not shrink the response writer and risk a reliable depth-1 writer blocking or
        # dropping responses under multiple concurrent clients. When the caller doesn't set
        # an explicit endpoint_history_depth, the response Publisher uses the standard default
        # queue depth, independent of max_queue_size; endpoint_history_depth overrides it.
        # Mirrors the C++ service constructor in request_response.h.
        response_history_depth = (
            default_max_queue_size
            if endpoint_history_depth == USE_DEFAULT_HISTORY_DEPTH
            else endpoint_history_depth
        )
        # Default the response Publisher to TRANSIENT_LOCAL, overridable via durability_kind.
        # NOTE: the DEFAULT client response Subscriber is VOLATILE (ServiceClient / request()
        # pass durability_kind=None → Fast-DDS default), so against a default client this
        # offered durability is inert (a VOLATILE reader never requests historical samples).
        # Not a data-loss path — in the settled flow the reader is matched before the response
        # is sent — it only benefits a client that opts its response reader into TRANSIENT_LOCAL.
        response_durability_kind = (
            TRANSIENT_LOCAL_DURABILITY_QOS if durability_kind is None else durability_kind
        )

        self._publisher = Publisher(
            domain_participant,
            response_topic_name,
            response_pub_sub_type,
            on_has_subscriber_changed_function=self._on_matched,
            reliability_kind=RELIABLE_RELIABILITY_QOS,
            history_depth=response_history_depth,
            durability_kind=response_durability_kind,
        )

        # The request Subscriber (service side) historically used VOLATILE
        # (Fast-DDS / XML default) durability and the default history, so the
        # endpoint params pass straight through (None / -1 reproduces that).
        # Mirrors the C++ make_subscriber call in the service constructor.
        self._subscriber = Subscriber(
            domain_participant,
            request_topic_name,
            request_pub_sub_type,
            request_data_type,
            on_data_function=self._on_data,
            reliability_kind=RELIABLE_RELIABILITY_QOS,
            max_history_depth=endpoint_history_depth,
            durability_kind=durability_kind,
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
