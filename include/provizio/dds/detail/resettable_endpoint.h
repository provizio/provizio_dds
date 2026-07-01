// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DDS_DETAIL_RESETTABLE_ENDPOINT
#define DDS_DETAIL_RESETTABLE_ENDPOINT

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>

namespace provizio::dds::detail
{
    /**
     * @file resettable_endpoint.h
     * @brief Internal contract between @c domain_participant and the publisher /
     * subscriber handles it owns, used during network-recovery reset.
     *
     * Every publisher_handle and subscriber_handle registers itself as a
     * @c resettable_endpoint with its parent domain_participant in
     * @c make_publisher / @c make_subscriber, and deregisters on destruction.
     *
     * Reset is a three-phase dance, in this order:
     *   1. @c detach_for_reset() is called WITHOUT the participant's lifecycle
     *      lock held. Endpoints stop dispatching listener callbacks and wait for
     *      any in-flight callback to drain. This is the phase that prevents a
     *      user callback (which may call back into provizio APIs that acquire
     *      the lifecycle lock shared) from deadlocking against the reset thread
     *      which is about to take the lock exclusively.
     *   2. @c on_participant_reset(old_participant) is called with the
     *      participant's lifecycle mutex held EXCLUSIVELY. The endpoint releases
     *      every reference into @c old_participant (which is about to be
     *      destroyed). May not throw.
     *   3. @c on_new_participant_started(new_participant) is called with the
     *      lifecycle mutex still held exclusively. The endpoint rebuilds its
     *      Fast-DDS objects against @c new_participant. May throw on
     *      construction failure; the parent domain_participant logs and
     *      continues with other endpoints.
     *
     * The participant pointer is passed by reference in phases 2 and 3 rather
     * than re-fetched, because the runtime already holds the lifecycle mutex
     * exclusively — re-locking would deadlock.
     *
     * A match-mode subscriber registers a NON-owning back-reference (a raw
     * pointer) to itself with the participant's deferred-subscriber registry, so
     * the participant's discovery listener can route a writer-discovered event to
     * the waiting subscriber (see @c start_deferred_build / @c build_deferred_locked
     * below and @c domain_participant::register_deferred_subscriber). The registry
     * is deliberately non-owning — the subscriber removes itself in its destructor
     * under the registry mutex — so no thread but the subscriber's owner ever runs
     * its teardown.
     */
    class resettable_endpoint
    {
      public:
        virtual ~resettable_endpoint() = default;

        /**
         * @brief Disable listener callbacks and block until any in-flight
         * callback returns. Called BEFORE @c reset_mutex is taken exclusively,
         * so callbacks that re-enter provizio APIs can complete. Default
         * implementation is a no-op (suitable for endpoints with no Fast-DDS
         * listener or whose listener never calls back into provizio).
         */
        virtual void detach_for_reset() noexcept
        {
        }

        /**
         * @brief Notification that this endpoint's parent participant is being
         * reset. The endpoint must release every reference into
         * @c old_participant before returning. May not throw.
         */
        virtual void on_participant_reset(eprosima::fastdds::dds::DomainParticipant &old_participant) noexcept = 0;

        /**
         * @brief Notification that a fresh Fast-DDS participant has been
         * created. The endpoint rebuilds its Fast-DDS objects against it. May
         * throw; the parent @c domain_participant logs and continues.
         */
        virtual void on_new_participant_started(eprosima::fastdds::dds::DomainParticipant &new_participant) = 0;

        /**
         * @brief Spawn the deferred DataReader build for a match-mode subscriber
         * that has just had its reliability resolved by a discovered writer.
         * Called by the participant's deferred-subscriber registry while holding
         * @c deferred_mutex (from @c register_deferred_subscriber on a cache hit,
         * or @c resolve_deferred_for_writer on writer discovery). The build itself
         * MUST run off the Fast-DDS discovery thread (building there would deadlock
         * against a concurrent reset — see the AB-BA note in subscriber.h), so the
         * implementation hops it onto its own thread (a stored @c std::future) and
         * returns promptly. It must be idempotent (spawn at most one build per
         * subscriber). Default no-op (only match-mode subscribers override it).
         */
        virtual void start_deferred_build(eprosima::fastdds::dds::ReliabilityQosPolicyKind /*resolved*/)
        {
        }

        /**
         * @brief Build the deferred DataReader now, with the resolved reliability.
         * Called by @c domain_participant::run_deferred_build on the build thread
         * spawned by @c start_deferred_build, which already holds
         * @c registration_mutex and @c reset_mutex — the latter EXCLUSIVELY (unlike
         * @c register_endpoint's shared acquire for an initial build) because the
         * deferred build swaps in a DataReader already visible to other threads and
         * so must fence concurrent get_guid / get_num_matched_publishers readers —
         * so the implementation MUST NOT re-acquire either. Must be idempotent (a
         * network-recovery reset may have rebuilt the reader first). Default no-op
         * (only match-mode subscribers override it).
         */
        virtual void build_deferred_locked(eprosima::fastdds::dds::DomainParticipant & /*participant*/,
                                           eprosima::fastdds::dds::ReliabilityQosPolicyKind /*resolved*/)
        {
        }
    };
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_RESETTABLE_ENDPOINT
