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
import inspect
import os
import threading
import weakref
from typing import Any, Callable, Optional
import time
from queue import Queue

# until https://bugs.python.org/issue46276 is fixed we can apply this workaround
# on windows
if os.name == "nt":
    import win32api

    win32api.LoadLibrary("provizio_dds_python_types")

from provizio_dds_python_types import *
from fastdds import *

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


def make_domain_participant(domain_id: int = 0):
    """Creates a new DDS Domain Participant that automatically cleans up internal objects on deletion

    :param domain_id: DDS domain_id, 0 by default
    :return: A wrapped DDS Domain Participant
    """

    class _DomainParticipant:
        # It's FASTDDS_DEFAULT_PROFILES_FILE in Fast-DDS 3, so needs change when upgrading
        xml_profiles_env_variable = "FASTRTPS_DEFAULT_PROFILES_FILE"

        def __init__(self, domain_id):
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

        def __del__(self):
            factory = DomainParticipantFactory.get_instance()
            self._participant.delete_contained_entities()
            factory.delete_participant(self._participant)
            del self._participant

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
            if self._on_has_subscriber_changed_function:
                if self._on_has_subscriber_changed_takes_guid:
                    if info.current_count_change > 0:
                        self._on_has_subscriber_changed_function(
                            self._publisher(),
                            True,
                            info.last_subscription_handle.get_guid(),
                        )
                    elif info.current_count_change < 0:
                        self._on_has_subscriber_changed_function(
                            self._publisher(),
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
                            self._publisher(), True
                        )
                    elif info.current_count == 0 and info.current_count_change < 0:
                        # Just unmatched the last publisher
                        self._on_has_subscriber_changed_function(
                            self._publisher(), False
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
        self._publisher.delete_datawriter(self._writer)
        del self._writer

        self._participant.fastdds_participant().delete_publisher(self._publisher)
        del self._publisher

        del self._listener
        del self._writer_qos
        del self._publisher_qos

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


class Subscriber(_TopicHandle):
    """Provides subscription functionality for a DDS data type and topic name specified when constructing"""

    class _ReaderListener(DataReaderListener):
        def __init__(
            self, data_type, on_data_function, on_has_publisher_changed_function
        ):
            super().__init__()
            self._data_type = data_type
            self._on_data_function = on_data_function
            self._on_has_publisher_changed_function = on_has_publisher_changed_function
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
            data_type, on_data_function, on_has_publisher_changed_function
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
        self._subscriber.delete_datareader(self._reader)
        del self._reader

        self._participant.fastdds_participant().delete_subscriber(self._subscriber)
        del self._subscriber

        del self._listener
        del self._reader_qos
        del self._subscriber_qos

        super().__del__()

    def get_guid(self):
        return self._reader.guid()


async def request(
    domain_participant: object,
    request_pub_sub_type,
    response_pub_sub_type,
    response_data_type,
    request_data,
    request_topic_name: str = None,
    response_topic_name: str = None,
    service_name: str = None,
):
    """Send a request and await the response.

    One of (request_topic_name, response_topic_name) or service_name must be provided.
    Uses RELIABLE reliability for both endpoints and correlates the response via
    related_sample_identity.

    Args:
        domain_participant: Domain participant wrapper created by `make_domain_participant`.
        request_pub_sub_type: PubSub type for the request.
        response_pub_sub_type: PubSub type for the response.
        response_data_type: Concrete data type of the response.
        request_data: Concrete data instance to publish as the request.
        request_topic_name: Optional explicit request topic name.
        response_topic_name: Optional explicit response topic name.
        service_name: Optional base name to derive request/response topics.
    Returns:
        The response data instance, or None if publishing the request failed.
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
    publisher_matched = asyncio.Event()
    subscriber_matched = asyncio.Event()

    def set_data(data):
        if not future.done():
            future.set_result(data)

    def on_response(data, info):
        with lock:
            if info.related_sample_identity == request_identity:
                loop.call_soon_threadsafe(lambda: set_data(data))

    def on_publisher_matched(_, matched):
        if matched:
            loop.call_soon_threadsafe(publisher_matched.set)

    def on_subscriber_matched(matched):
        if matched:
            loop.call_soon_threadsafe(subscriber_matched.set)

    response_subscriber = Subscriber(
        domain_participant,
        response_topic_name,
        response_pub_sub_type,
        response_data_type,
        on_response,
        on_has_publisher_changed_function=on_subscriber_matched,
        reliability_kind=RELIABLE_RELIABILITY_QOS,
    )

    client_publisher_history_depth = 1
    request_publisher = Publisher(
        domain_participant,
        request_topic_name,
        request_pub_sub_type,
        on_has_subscriber_changed_function=on_publisher_matched,
        reliability_kind=RELIABLE_RELIABILITY_QOS,
        history_depth=client_publisher_history_depth,
    )

    await subscriber_matched.wait()
    await publisher_matched.wait()

    params = WriteParams()
    params.related_sample_identity().writer_guid(response_subscriber.get_guid())

    with lock:
        if not request_publisher.publish(request_data, params):
            return None
        request_identity.writer_guid(response_subscriber.get_guid())
        request_identity.sequence_number(params.sample_identity().sequence_number())

    result = await future

    del request_publisher
    del response_subscriber

    return result


class Service:
    """Request/response service.

    Consumes requests from a request topic and publishes responses to the
    corresponding response topic. Supports both synchronous and async request
    handlers, back-pressure via max_history_depth, and delayed dispatch of
    responses until the originating client is matched.
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

                self._request_handler.stop()

                del self._publisher
                del self._subscriber
                del self._request_handler

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
