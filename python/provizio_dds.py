#!/bin/bash

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
import os

# until https://bugs.python.org/issue46276 is fixed we can apply this workaround
# on windows
if os.name == 'nt':
    import win32api
    win32api.LoadLibrary('provizio_dds_python_types')

from provizio_dds_python_types import *
from fastdds import *
if __package__ or "." in __name__:
    from . import point_cloud2
else:
    import point_cloud2



class QosDefaults:
    """Defines default QOS policies. They can be overriden for specific types"""

    """Per type defaults for datawriter_reliability_kind. RELIABLE_RELIABILITY_QOS by default in Fast DDS"""
    datawriter_reliability_kind_per_type = {None: RELIABLE_RELIABILITY_QOS}

    """Per type defaults for datareader_reliability_kind. BEST_EFFORT_RELIABILITY_QOS by default in Fast DDS"""
    datareader_reliability_kind_per_type = {None: BEST_EFFORT_RELIABILITY_QOS}

    """Per type defaults for memory policies, both datawriter and datareader. PREALLOCATED_WITH_REALLOC_MEMORY_MODE in Fast-DDS 2.9+"""
    memory_policy_per_type = {None: PREALLOCATED_WITH_REALLOC_MEMORY_MODE}

    def __init__(self, pub_sub_type):
        """Constructs an instance of QosDefaults for the DDS Pub/Sub type.

        :param pub_sub_type: The DDS PubSub Type, f.e. provizio_dds.StringPubSubType
        """
        try:
            self.datawriter_reliability_kind = QosDefaults.datawriter_reliability_kind_per_type[
                pub_sub_type]
        except KeyError:
            self.datawriter_reliability_kind = QosDefaults.datawriter_reliability_kind_per_type[
                None]
        try:
            self.datareader_reliability_kind = QosDefaults.datareader_reliability_kind_per_type[
                pub_sub_type]
        except KeyError:
            self.datareader_reliability_kind = QosDefaults.datareader_reliability_kind_per_type[
                None]
        try:
            self.memory_policy = QosDefaults.memory_policy_per_type[pub_sub_type]
        except KeyError:
            self.memory_policy = QosDefaults.memory_policy_per_type[None]


def make_domain_participant(domain_id=0):
    """Creates a new DDS Domain Participant that automatically cleans up internal objects on deletion

    :param domain_id: DDS domain_id, 0 by default
    :return: A wrapped DDS Domain Participant
    """

    class _DomainParticipant:
        def __init__(self, domain_id=0):
            factory = DomainParticipantFactory.get_instance()
            self._participant_qos = DomainParticipantQos()
            factory.get_default_participant_qos(self._participant_qos)
            self._participant = factory.create_participant(
                domain_id, self._participant_qos)

        def __del__(self):
            try:
                factory = DomainParticipantFactory.get_instance()
                self._participant.delete_contained_entities()
                factory.delete_participant(self._participant)
            except:
                pass

        def get(self):
            return self._participant

    return _DomainParticipant(domain_id)


class _TopicHandle:
    def __init__(self, domain_participant, topic_name, pub_sub_type):
        self._participant = domain_participant

        # Register Type
        self._topic_data_type = pub_sub_type()
        self._type_support = TypeSupport(self._topic_data_type)
        self._participant.get().register_type(self._type_support)

        # Register Topic
        self._topic_qos = TopicQos()
        self._participant.get().get_default_topic_qos(self._topic_qos)
        self._topic = self._participant.get().create_topic(
            topic_name, self._topic_data_type.getName(), self._topic_qos)

    def __del__(self):
        try:
            self._participant.get().delete_topic(self._topic)
        except:
            pass


class Publisher(_TopicHandle):
    """Provides publishing functionality for a DDS data type and topic name specified when constructing"""

    class _WriterListener(DataWriterListener):
        def __init__(self, publisher, on_has_subscriber_changed_function):
            super().__init__()
            self._publisher = publisher
            self._on_has_subscriber_changed_function = on_has_subscriber_changed_function

        def on_publication_matched(self, _, info):
            if (self._on_has_subscriber_changed_function):
                if (info.current_count > 0 and info.current_count_change == info.current_count):
                    # Just matched the first publisher
                    self._on_has_subscriber_changed_function(
                        self._publisher, True)
                elif (info.current_count == 0 and info.current_count_change < 0):
                    # Just unmatched the last publisher
                    self._on_has_subscriber_changed_function(
                        self._publisher, False)

    def __init__(self, domain_participant, topic_name, pub_sub_type, on_has_subscriber_changed_function=None, reliability_kind=None):
        """Constructs a DDS Publisher

        :param domain_participant: A DDS Domain Participant wrapper object, as created by provizio_dds.make_domain_participant
        :param str topic_name: A string DDS Topic name
        :param pub_sub_type: The DDS PubSub Type to be published, f.e. provizio_dds.StringPubSubType
        :param on_has_subscriber_changed_function: Optional, a function to to be invoked on matching first / umatching last subscriber, takes two arguments: a Publisher and a bool: True when the first subscriber is matched, False when the last subscriber is unmatched; Note: called from a background Thread
        :param reliability_kind: Optional, a DDS data writer reliability kind to be used: either BEST_EFFORT_RELIABILITY_QOS or RELIABLE_RELIABILITY_QOS; if not specified, QosDefaults for pub_sub_type will be used
        """

        super().__init__(domain_participant, topic_name, pub_sub_type)

        qos_defaults = QosDefaults(pub_sub_type)

        if (reliability_kind is None):
            reliability_kind = qos_defaults.datawriter_reliability_kind

        # Create Publisher
        self._publisher_qos = PublisherQos()
        self._participant.get().get_default_publisher_qos(self._publisher_qos)
        self._publisher = self._participant.get().create_publisher(
            self._publisher_qos)

        # Create DataWriter
        self._listener = Publisher._WriterListener(
            self, on_has_subscriber_changed_function)
        self._writer_qos = DataWriterQos()
        self._publisher.get_default_datawriter_qos(self._writer_qos)
        self._writer_qos.reliability().kind = reliability_kind
        self._writer_qos.endpoint().history_memory_policy = qos_defaults.memory_policy
        self._writer = self._publisher.create_datawriter(
            self._topic, self._writer_qos, self._listener)

    def __del__(self):
        try:
            self._subscriber.delete_datawriter(self._writer)
        except:
            pass

        try:
            self._participant.get().delete_publisher(self._publisher)
        except:
            pass

        super().__del__()

    def publish(self, data):
        """Publishes DDS data

        :param data: actual data (not Pub Sub Type), f.e. provizio_dds.String
        :return: True if published successfully, and False otherwise
        """
        return self._writer.write(data)


class Subscriber(_TopicHandle):
    """Provides subscription functionality for a DDS data type and topic name specified when constructing"""

    class _ReaderListener(DataReaderListener):
        def __init__(self, data_type, on_data_function, on_has_publisher_changed_function):
            super().__init__()
            self._data_type = data_type
            self._on_data_function = on_data_function
            self._on_has_publisher_changed_function = on_has_publisher_changed_function

        def on_data_available(self, reader):
            info = SampleInfo()
            data = self._data_type()
            reader.take_next_sample(data, info)
            self._on_data_function(data)

        def on_subscription_matched(self, _, info):
            if (self._on_has_publisher_changed_function):
                if (info.current_count > 0 and info.current_count_change == info.current_count):
                    # Just matched the first publisher
                    self._on_has_publisher_changed_function(True)
                elif (info.current_count == 0 and info.current_count_change < 0):
                    # Just unmatched the last publisher
                    self._on_has_publisher_changed_function(False)

    def __init__(self, domain_participant, topic_name, pub_sub_type, data_type, on_data_function, on_has_publisher_changed_function=None, reliability_kind=None):
        """Constructs a DDS Subscriber

        :param domain_participant: A DDS Domain Participant wrapper object, as created by provizio_dds.make_domain_participant
        :param str topic_name: A string DDS Topic name
        :param pub_sub_type: The DDS PubSub Type to be received, f.e. provizio_dds.StringPubSubType
        :param data_type: The DDS Data Type to be received, f.e. provizio_dds.String
        :param on_data_function: A function to be invoked on receiving published data, takes a single argument of DDS Data Type, f.e. provizio_dds.String; Note: called from a background Thread
        :param on_has_publisher_changed_function: Optional, a function to be invoked on matching first / umatching last publisher, takes a single bool argument: True when the first publisher is matched, False when the last publisher is unmatched; Note: called from a background Thread
        :param reliability_kind: Optional, a DDS data reader reliability kind to be used: either BEST_EFFORT_RELIABILITY_QOS or RELIABLE_RELIABILITY_QOS; if not specified, QosDefaults for pub_sub_type will be used
        """
        super().__init__(domain_participant, topic_name, pub_sub_type)

        qos_defaults = QosDefaults(pub_sub_type)

        if (reliability_kind is None):
            reliability_kind = qos_defaults.datareader_reliability_kind

        # Create Subscriber
        self._subscriber_qos = SubscriberQos()
        self._participant.get().get_default_subscriber_qos(self._subscriber_qos)
        self._subscriber = self._participant.get().create_subscriber(
            self._subscriber_qos)

        # Create DataReader
        self._listener = Subscriber._ReaderListener(
            data_type, on_data_function, on_has_publisher_changed_function)
        self._reader_qos = DataReaderQos()
        self._subscriber.get_default_datareader_qos(self._reader_qos)
        self._reader_qos.reliability().kind = reliability_kind
        self._reader_qos.endpoint().history_memory_policy = qos_defaults.memory_policy
        self._reader = self._subscriber.create_datareader(
            self._topic, self._reader_qos, self._listener)

    def __del__(self):
        try:
            self._subscriber.delete_datareader(self._reader)
        except:
            pass

        try:
            self._participant.get().delete_subscriber(self._subscriber)
        except:
            pass
        super().__del__()
