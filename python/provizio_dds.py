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
import threading
import weakref
from typing import Any, Callable, Optional
import time
from queue import Queue

# On Windows, Python 3.8+ restricts DLL search paths (https://bugs.python.org/issue46276).
# Add the directory containing this module so dependent DLLs (provizio_dds, Fast-DDS,
# OpenSSL, etc.) can be found.  All required DLLs must be co-located with this module
# (pip install does this automatically; ctest copies them in test/python/CMakeLists.txt).
# Scanning PATH is intentionally avoided: CI runners carry ABI-incompatible copies of
# common libraries (e.g. MinGW/MySQL/PHP OpenSSL) that cause access violations when
# loaded instead of the MSVC-built copies we ship.
if os.name == "nt":
    os.add_dll_directory(os.path.dirname(os.path.abspath(__file__)))

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
else:
    import point_cloud2
    import accumulation


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
# destructors access already-freed memory → access violation.  On Windows
# this also corrupts DDS shared-memory segments, crashing subsequent tests.
#
# To prevent this we:
#   1. Track every live _DomainParticipant via weak refs.
#   2. Register an atexit handler that calls _cleanup() on each participant
#      *before* the interpreter starts tearing down modules.
#   3. _cleanup() calls delete_contained_entities() + delete_participant()
#      and sets a flag so that child __del__ methods become safe no-ops.
# ---------------------------------------------------------------------------
_live_participants = []


@atexit.register
def _cleanup_all_participants():
    """Explicitly destroy every DomainParticipant before interpreter shutdown
    so that SWIG C++ destructors run in the correct order."""
    for ref in reversed(_live_participants):
        p = ref()
        if p is not None:
            p._cleanup()
    _live_participants.clear()


def make_domain_participant(domain_id: int = 0):
    """Creates a new DDS Domain Participant that automatically cleans up internal objects on deletion

    :param domain_id: DDS domain_id, 0 by default
    :return: A wrapped DDS Domain Participant
    """

    class _DomainParticipant:
        # It's FASTDDS_DEFAULT_PROFILES_FILE in Fast-DDS 3, so needs change when upgrading
        xml_profiles_env_variable = "FASTRTPS_DEFAULT_PROFILES_FILE"

        def __init__(self, domain_id):
            self._cleaned_up = False

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

            self._register_type_mutex = threading.Lock()
            self._registered_types = dict()
            self._register_topic_mutex = threading.Lock()
            self._registered_topics = dict()

            ref = weakref.ref(self, lambda r: _live_participants.remove(r))
            _live_participants.append(ref)

        def _cleanup(self):
            """Deterministic cleanup: delete all C++ entities then the participant."""
            if self._cleaned_up:
                return
            self._cleaned_up = True
            factory = DomainParticipantFactory.get_instance()
            self._participant.delete_contained_entities()
            factory.delete_participant(self._participant)

        def __del__(self):
            self._cleanup()

        def fastdds_participant(self):
            return self._participant

        def register_type(self, pub_sub_type_instance):
            with self._register_type_mutex:
                type_support = self._registered_types.get(
                    pub_sub_type_instance.getName(), None
                )
                if type_support is None:
                    type_support = TypeSupport(pub_sub_type_instance)
                    self._participant.register_type(type_support)
                    self._registered_types[pub_sub_type_instance.getName()] = (
                        type_support
                    )
                return type_support

        def register_topic(self, topic_name, pub_sub_type):
            with self._register_topic_mutex:
                topic_info = self._registered_topics.get(topic_name)

                if topic_info:
                    # Ensure the type matches the already registered topic's type
                    existing_type_name = topic_info["type_support"].get_type_name()
                    requested_type_name = pub_sub_type().getName()
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
                        topic_name, pub_sub_type_instance.getName(), topic_qos
                    )

                    self._registered_topics[topic_name] = {
                        "topic": new_topic,
                        "ref_count": 1,
                        "type_support": type_support,
                    }
                    return new_topic

        def unregister_topic(self, topic_name):
            with self._register_topic_mutex:
                topic_info = self._registered_topics.get(topic_name)
                if topic_info:
                    topic_info["ref_count"] -= 1
                    if topic_info["ref_count"] <= 0:
                        if not self._cleaned_up:
                            self.fastdds_participant().delete_topic(topic_info["topic"])
                        del self._registered_topics[topic_name]

    return _DomainParticipant(domain_id)


class _TopicHandle:
    def __init__(self, domain_participant, topic_name, pub_sub_type):
        self._participant = domain_participant
        self._topic_name = topic_name
        self._topic = self._participant.register_topic(topic_name, pub_sub_type)

    def __del__(self):
        self._participant.unregister_topic(self._topic_name)


class Publisher(_TopicHandle):
    """Provides publishing functionality for a DDS data type and topic name specified when constructing"""

    class _WriterListener(DataWriterListener):
        def __init__(self, publisher, on_has_subscriber_changed_function):
            super().__init__()
            self._publisher = weakref.ref(publisher)
            self._on_has_subscriber_changed_function = (
                on_has_subscriber_changed_function
            )
            try:
                sig = inspect.signature(self._on_has_subscriber_changed_function)
                self._on_has_subscriber_changed_takes_guid = len(sig.parameters) == 3
            except (ValueError, TypeError):
                self._on_has_subscriber_changed_takes_guid = False

        def __del__(self):
            del self._publisher
            del self._on_has_subscriber_changed_function

        def on_publication_matched(self, _, info):
            publisher = self._publisher()
            if publisher is None:
                return

            publisher._update_num_matched_subscribers(info.current_count)

            if self._on_has_subscriber_changed_function:
                if self._on_has_subscriber_changed_takes_guid:
                    if info.current_count_change > 0:
                        self._on_has_subscriber_changed_function(
                            publisher,
                            True,
                            info.last_subscription_handle.get_guid(),
                        )
                    elif info.current_count_change < 0:
                        self._on_has_subscriber_changed_function(
                            publisher,
                            False,
                            info.last_subscription_handle.get_guid(),
                        )
                else:
                    if (
                        info.current_count > 0
                        and info.current_count_change == info.current_count
                    ):
                        # Just matched the first publisher
                        self._on_has_subscriber_changed_function(
                            publisher, True
                        )
                    elif info.current_count == 0 and info.current_count_change < 0:
                        # Just unmatched the last publisher
                        self._on_has_subscriber_changed_function(
                            publisher, False
                        )

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
        :param on_has_subscriber_changed_function: Optional, a function to to be invoked on matching first / unmatching last subscriber, takes two arguments: a Publisher and a bool: True when the first subscriber is matched, False when the last subscriber is unmatched; Note: called from a background Thread
        :param reliability_kind: Optional, a DDS data writer reliability kind to be used: either BEST_EFFORT_RELIABILITY_QOS or RELIABLE_RELIABILITY_QOS; if not specified, QosDefaults for pub_sub_type will be used
        """

        super().__init__(domain_participant, topic_name, pub_sub_type)
        self._num_matched_cv = threading.Condition()
        self._num_matched_subscribers = 0

        qos_defaults = QosDefaults(pub_sub_type)

        if reliability_kind is None:
            reliability_kind = qos_defaults.datawriter_reliability_kind

        # Create Publisher
        self._publisher_qos = PublisherQos()
        self._participant.fastdds_participant().get_default_publisher_qos(
            self._publisher_qos
        )
        self._publisher = self._participant.fastdds_participant().create_publisher(
            self._publisher_qos
        )

        # Create DataWriter
        self._listener = Publisher._WriterListener(
            self, on_has_subscriber_changed_function
        )
        self._writer_qos = DataWriterQos()
        self._publisher.get_default_datawriter_qos(self._writer_qos)
        self._writer_qos.reliability().kind = reliability_kind
        self._writer_qos.endpoint().history_memory_policy = qos_defaults.memory_policy
        if history_depth == USE_DEFAULT_QOS_DURABILITY:
            pass
        elif history_depth == NO_HISTORY:
            self._writer_qos.durability().kind = VOLATILE_DURABILITY_QOS
        elif history_depth > 0:
            self._writer_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS
            self._writer_qos.history().kind = KEEP_LAST_HISTORY_QOS
            self._writer_qos.history().depth = history_depth
        self._writer = self._publisher.create_datawriter(
            self._topic, self._writer_qos, self._listener
        )

    def __del__(self):
        # If the participant was already cleaned up (atexit / early GC),
        # delete_contained_entities() already freed the C++ objects.
        if not self._participant._cleaned_up:
            self._publisher.delete_datawriter(self._writer)
            self._participant.fastdds_participant().delete_publisher(self._publisher)
        super().__del__()

    def publish(self, data: object, params: WriteParams = None):
        """Publishes DDS data

        :param data: actual data (not Pub Sub Type), f.e. provizio_dds.String
        :param params: optional WriteParams to control the write operation
        :return: True if published successfully, and False otherwise
        """
        if params:
            return self._writer.write(data, params)
        return self._writer.write(data)

    def _update_num_matched_subscribers(self, count: int):
        with self._num_matched_cv:
            self._num_matched_subscribers = count
            self._num_matched_cv.notify_all()

    def get_num_matched_subscribers(
        self, timeout_sec: float, settle_time_sec: float
    ) -> int:
        """Return the stable number of matched subscribers or -1 if unstable until timeout."""

        return _get_stable_match_count(
            self._num_matched_cv, lambda: self._num_matched_subscribers, timeout_sec, settle_time_sec
        )

class Subscriber(_TopicHandle):
    """Provides subscription functionality for a DDS data type and topic name specified when constructing"""

    class _ReaderListener(DataReaderListener):
        def __init__(
            self, data_type, on_data_function, on_has_publisher_changed_function, subscriber
        ):
            super().__init__()
            self._data_type = data_type
            self._on_data_function = on_data_function
            self._on_has_publisher_changed_function = on_has_publisher_changed_function
            self._subscriber = weakref.ref(subscriber)
            try:
                sig = inspect.signature(self._on_data_function)
                self._on_data_takes_info = len(sig.parameters) == 2
            except (ValueError, TypeError):
                self._on_data_takes_info = False

        def __del__(self):
            del self._data_type
            del self._on_data_function
            del self._on_has_publisher_changed_function

        def on_data_available(self, reader):
            while True:
                info = SampleInfo()
                data = self._data_type()
                if reader.take_next_sample(data, info) != ReturnCode_t.RETCODE_OK:
                    break
                if not info.valid_data:
                    continue
                if self._on_data_takes_info:
                    self._on_data_function(data, info)
                else:
                    self._on_data_function(data)

        def on_subscription_matched(self, _, info):
            subscriber = self._subscriber()
            if subscriber is None:
                return

            subscriber._update_num_matched_publishers(info.current_count)

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
        """
        super().__init__(domain_participant, topic_name, pub_sub_type)
        self._num_matched_cv = threading.Condition()
        self._num_matched_publishers = 0

        qos_defaults = QosDefaults(pub_sub_type)

        if reliability_kind is None:
            reliability_kind = qos_defaults.datareader_reliability_kind

        # Create Subscriber
        self._subscriber_qos = SubscriberQos()
        self._participant.fastdds_participant().get_default_subscriber_qos(
            self._subscriber_qos
        )
        self._subscriber = self._participant.fastdds_participant().create_subscriber(
            self._subscriber_qos
        )

        # Create DataReader
        self._listener = Subscriber._ReaderListener(
            data_type, on_data_function, on_has_publisher_changed_function, self
        )
        self._reader_qos = DataReaderQos()
        self._subscriber.get_default_datareader_qos(self._reader_qos)
        self._reader_qos.reliability().kind = reliability_kind
        self._reader_qos.endpoint().history_memory_policy = qos_defaults.memory_policy
        if max_history_depth == USE_DEFAULT_QOS_DURABILITY:
            pass
        elif max_history_depth == NO_HISTORY:
            self._reader_qos.durability().kind = VOLATILE_DURABILITY_QOS
        elif max_history_depth > 0:
            self._reader_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS
            self._reader_qos.history().kind = KEEP_LAST_HISTORY_QOS
            self._reader_qos.history().depth = max_history_depth

        self._reader = self._subscriber.create_datareader(
            self._topic, self._reader_qos, self._listener
        )

    def __del__(self):
        # If the participant was already cleaned up (atexit / early GC),
        # delete_contained_entities() already freed the C++ objects.
        if not self._participant._cleaned_up:
            self._subscriber.delete_datareader(self._reader)
            self._participant.fastdds_participant().delete_subscriber(self._subscriber)
        super().__del__()

    def get_guid(self):
        return self._reader.guid()

    def _update_num_matched_publishers(self, count: int):
        with self._num_matched_cv:
            self._num_matched_publishers = count
            self._num_matched_cv.notify_all()

    def get_num_matched_publishers(
        self, timeout_sec: float, settle_time_sec: float
    ) -> int:
        """Return the stable number of matched publishers or -1 if unstable until timeout."""

        return _get_stable_match_count(
            self._num_matched_cv, lambda: self._num_matched_publishers, timeout_sec, settle_time_sec
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

    Args:
        domain_participant: Domain participant wrapper created by `make_domain_participant`.
        request_pub_sub_type: PubSub type for the request.
        response_pub_sub_type: PubSub type for the response.
        response_data_type: Concrete data type of the response.
        request_data: Concrete data instance to publish as the request.
        request_topic_name: Optional explicit request topic name.
        response_topic_name: Optional explicit response topic name.
        service_name: Optional base name to derive request/response topics.
        stable_matches_period_sec: Optional settling window (seconds) that match counts must remain stable before
            sending the first request (defaults to 1.0s). Set to 0 to skip the extra wait.
        service_match_timeout_sec: Optional deadline (seconds) to complete endpoint matching. Set to 0 to wait indefinitely.
    Returns:
        The response data instance.

    Raises:
        ServiceMatchingTimeoutError: If endpoints fail to match within the timeout.
        RequestPublishError: If publishing the request fails.
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
    lock = threading.Lock()

    def set_data(data):
        if not future.done():
            future.set_result(data)

    def on_response(data, info):
        with lock:
            if info.related_sample_identity == request_identity:
                loop.call_soon_threadsafe(lambda: set_data(data))

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

        params = WriteParams()
        params.related_sample_identity().writer_guid(response_subscriber.get_guid())

        with lock:
            if not request_publisher.publish(request_data, params):
                raise RequestPublishError("Failed to publish the DDS request")
            request_identity.writer_guid(response_subscriber.get_guid())
            request_identity.sequence_number(params.sample_identity().sequence_number())

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
            self._thread = threading.Thread(target=self._process_requests)
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
            self._thread = threading.Thread(target=self._run_loop)
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

        Args:
            domain_participant: Domain participant wrapper created by `make_domain_participant`.
            request_pub_sub_type: PubSub type for requests.
            request_data_type: Concrete request data type.
            response_pub_sub_type: PubSub type for responses.
            handle_request_function: Callable or coroutine to process a request and return response data.
            request_topic_name: Optional explicit request topic name, or use `service_name`.
            response_topic_name: Optional explicit response topic name, or use `service_name`.
            service_name: If provided, request topic is rq/<service_name>Request and response is rr/<service_name>Reply.
            max_history_depth: Reader history depth for transient local durability; 0 for no history (volatile durability), USE_DEFAULT_QOS_DURABILITY for default.
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

        self._dispatch_responses_thread = threading.Thread(
            target=self._dispatch_responses
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

                if hasattr(self, '_request_handler'):
                    self._request_handler.stop()

                # Delete in child-before-parent order; their __del__ methods
                # are guarded against participant-already-cleaned-up.
                for attr in ('_subscriber', '_publisher', '_request_handler'):
                    if hasattr(self, attr):
                        delattr(self, attr)

                self._service_cv.notify_all()
                join = True
        if join:
            self._dispatch_responses_thread.join()

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
        with self._service_cv:
            if self._is_subscriber_matched_mutex_prelocked(identity.writer_guid()):
                params = WriteParams()
                params.related_sample_identity(identity)
                self._publisher.publish(data, params)
            else:
                self._ready_responses.append(
                    {"data": data, "identity": identity, "time_ready": time.time()}
                )
                self._service_cv.notify_all()

    def _is_subscriber_matched_mutex_prelocked(self, subscriber_guid):
        return str(subscriber_guid) in self._matched_subscriptions

    def _dispatch_matched(self):
        dispatched = False
        for i in range(len(self._ready_responses) - 1, -1, -1):
            response = self._ready_responses[i]
            if self._is_subscriber_matched_mutex_prelocked(
                response["identity"].writer_guid()
            ):
                params = WriteParams()
                params.related_sample_identity(response["identity"])
                self._publisher.publish(response["data"], params)
                self._ready_responses.pop(i)
                dispatched = True

        return dispatched

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
            with self._service_cv:
                self._service_cv.wait_for(
                    lambda: self._stop
                    or self._dispatch_matched()
                    or self._cleanup_timed_out(),
                    timeout=2,
                )
                if self._stop:
                    return


_request_prefix = "rq/"
_response_prefix = "rr/"
_request_suffix = "Request"
_response_suffix = "Reply"
