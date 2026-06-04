// Copyright 2023 Provizio Ltd.
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

#ifndef DDS_DOMAIN_PARTICIPANT
#define DDS_DOMAIN_PARTICIPANT

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/topic.h"

namespace provizio::dds::detail
{
    class resettable_endpoint;
}  // namespace provizio::dds::detail

namespace provizio::dds
{
    /**
     * @file domain_participant.h
     * @brief Wrapper over eprosima::fastdds::dds::DomainParticipant taking care of TypeSupport and Topic
     * registration.
     *
     * @note Registration when register_type and register_topic is performed just once per type/topic name, any
     * consequent registration simply reuses the registered TypeSupport/Topic.
     *
     * @note Auto-recovery requires construction via @c make_domain_participant
     * (the participant must be owned by a @c shared_ptr at registration time —
     * the constructor cannot register itself). When created that way with
     * auto-recovery enabled (the default — see @c network_recovery_mode), the
     * participant joins a process-wide registry that watches for host
     * network-interface address changes and recreates the participant's
     * underlying Fast-DDS DomainParticipant on confirmed changes. Publisher /
     * subscriber handles created against this participant remain valid across
     * the reset — their internal Fast-DDS objects are swapped under the
     * caller-held @c shared_ptr. See @c network_recovery.h. A
     * @c domain_participant constructed directly (not via
     * @c make_domain_participant) does NOT receive auto-recovery, even if the
     * @c recovery_mode argument selects it.
     */
    class PROVIZIO_DDS_API domain_participant
    {
      public:
        /**
         * @brief RAII handle returned by @c fastdds_participant(). Holds a shared
         * lock on the participant's lifecycle mutex for its entire lifetime, so the
         * underlying Fast-DDS @c DomainParticipant can't be torn down by a
         * concurrent network-recovery reset while the handle exists.
         *
         * Pointer-like: @c operator-> and @c operator* reach the participant.
         * Move-only — copying would silently widen the lock scope, which is rarely
         * what you want.
         *
         * @warning @c get() / @c operator-> / @c operator* may return @c nullptr
         * (and dereferencing the operators is then undefined behaviour). The
         * "dead" state happens when a network-recovery reset successfully
         * destroyed the old Fast-DDS participant but its subsequent
         * @c create_participant call returned @c nullptr (XML profile parse
         * failure, RLIMIT_NOFILE exhaustion, memory pressure). The participant
         * stays dead until another network-change event triggers a retry.
         * Always check @c get() != nullptr before dereferencing, or use the
         * higher-level @c publish / @c get_guid / @c make_publisher /
         * @c make_subscriber paths — those handle the dead state internally
         * (publish() returns false; make_* throws).
         *
         * @warning The @c locked_participant only keeps the lifecycle
         * @c shared_lock alive — it does NOT extend the lifetime of the owning
         * @c domain_participant. If the last @c shared_ptr<domain_participant>
         * is dropped while a @c locked_participant is still in scope, the
         * lock references a destroyed @c std::shared_mutex (undefined
         * behaviour). In practice this is a non-issue because the
         * @c locked_participant is designed as a short-lived
         * statement-scope handle obtained from a participant the caller is
         * actively using (and therefore holding a @c shared_ptr to), but a
         * pattern like @code{.cpp}
         *   auto fdds = make_domain_participant()->fastdds_participant();
         * @endcode is wrong: the temporary @c shared_ptr from
         * @c make_domain_participant dies at the end of the full expression
         * and @c fdds outlives the participant by one statement. Always
         * keep the @c shared_ptr (or a captured reference to it) in scope
         * for at least as long as any @c locked_participant taken from it.
         */
        class locked_participant
        {
          public:
            locked_participant(locked_participant &&) noexcept = default;
            locked_participant &operator=(locked_participant &&) noexcept = default;
            locked_participant(const locked_participant &) = delete;
            locked_participant &operator=(const locked_participant &) = delete;

            /// May return @c nullptr — see the class-level warning above.
            eprosima::fastdds::dds::DomainParticipant *operator->() const noexcept
            {
                return the_participant;
            }
            /// Dereferences without a null check — see the class-level warning.
            eprosima::fastdds::dds::DomainParticipant &operator*() const noexcept
            {
                return *the_participant;
            }
            /// May return @c nullptr — see the class-level warning above.
            eprosima::fastdds::dds::DomainParticipant *get() const noexcept
            {
                return the_participant;
            }

          private:
            friend class domain_participant;
            locked_participant(std::shared_lock<std::shared_mutex> lock,
                               eprosima::fastdds::dds::DomainParticipant *the_participant)
                : the_lock(std::move(lock)), the_participant(the_participant)
            {
            }

            std::shared_lock<std::shared_mutex> the_lock;
            eprosima::fastdds::dds::DomainParticipant *the_participant;
        };

        /**
         * @brief Construct a new domain participant object
         *
         * @param domain_id DDS domain_id, 0 by default
         * @param recovery_mode Whether this participant participates in network
         * auto-recovery (defaults to env-var-controlled; recovery is on by default
         * but can be globally disabled with @c PROVIZIO_DDS_NETWORK_RECOVERY=off).
         */
        domain_participant(DomainId_t domain_id = 0,
                           network_recovery_mode recovery_mode = network_recovery_mode::env_var_controlled);

        /**
         * @brief Destroys the domain participant, cleaning up all remaining DDS entities.
         */
        ~domain_participant();

        /**
         * @brief Registers the type in the domain participant, only once per domain participant, thread-safe.
         *
         * @tparam data_pub_sub_type PubSub type to register
         * @return TypeSupport that has been registered in the domain participant
         */
        template <typename data_pub_sub_type> TypeSupport register_type();

        /**
         * @brief Registers the topic in the domain participant, only once per domain participant, thread-safe.
         *
         * @param topic_name Name of the topic.
         * @param type_name Name of the type.
         * @param qos QoS profile for the topic.
         * @return std::shared_ptr<topic> A handle to the registered topic.
         * @throws std::runtime_error if the topic has been already registered with a different QoS.
         */
        std::shared_ptr<topic> register_topic(const std::string &topic_name, const std::string &type_name,
                                              const TopicQos &qos);

        /**
         * @brief Variants of @c register_type / @c register_topic that assume the
         * caller already holds the participant's lifecycle mutex (shared or
         * exclusive). Used by @c publisher_handle / @c subscriber_handle::build_state,
         * which always run under the lock — either inside @c register_endpoint
         * (shared) or inside @c trigger_network_recovery_reset (exclusive).
         * Re-acquiring the shared lock recursively on the same thread is
         * undefined behaviour for @c std::shared_mutex (and aborts with
         * @c EDEADLK on glibc); these overloads exist specifically to avoid
         * that.
         *
         * @note Not intended for direct customer use. Prefer the
         * lock-taking @c register_type / @c register_topic from external code.
         */
        template <typename data_pub_sub_type> TypeSupport register_type_locked();
        std::shared_ptr<topic> register_topic_locked(const std::string &topic_name, const std::string &type_name,
                                                     const TopicQos &qos);

        /**
         * @brief Acquire the underlying Fast-DDS DomainParticipant with a held
         * shared lock that prevents an asynchronous network-recovery reset from
         * tearing it down for the lifetime of the returned handle.
         *
         * Safe by construction: there is no way to obtain the raw participant
         * pointer without also holding the lock that keeps it alive.
         */
        locked_participant fastdds_participant()
        {
            // Evaluation order: brace-init guarantees the shared_lock is constructed
            // (acquired) before `participant` is read (C++17 [dcl.init.list]/4 —
            // left-to-right evaluation), so the read sees the value stable under
            // the lock.
            return locked_participant{std::shared_lock<std::shared_mutex>{reset_mutex}, participant};
        }

        /**
         * @brief Trigger a participant recreation: detach listener callbacks,
         * tear down all child endpoints, destroy the current Fast-DDS
         * participant, build a new one with the same QoS, and re-arm all child
         * endpoints. Called by @c detail::network_recovery_coordinator on a
         * confirmed network change; should not normally be called by user code.
         */
        void trigger_network_recovery_reset();

        /**
         * @brief Monotonically increasing identifier for the underlying Fast-DDS
         * participant. Starts at 1 on construction, increments by 1 on every
         * successful recreation in @c trigger_network_recovery_reset. Used by
         * publisher_handle / subscriber_handle to detect "I was built against a
         * previous participant that has since been destroyed" — and skip
         * Fast-DDS-side teardown that would use-after-free.
         */
        std::uint64_t participant_generation() const noexcept
        {
            return generation.load(std::memory_order_acquire);
        }

      public:
        /// @internal Used by make_publisher / make_subscriber. Atomically adds the
        /// endpoint to the recovery registry AND calls
        /// @c endpoint->on_new_participant_started against the current participant.
        /// The two steps are done under the same lifecycle-mutex shared scope so a
        /// concurrent reset cannot observe an "endpoint registered but state
        /// unbuilt" gap.
        void register_endpoint(const std::shared_ptr<detail::resettable_endpoint> &endpoint);
        /// @internal Used by publisher_handle / subscriber_handle on destruction.
        void deregister_endpoint(const detail::resettable_endpoint *endpoint);

      private:
        /// @brief Build (or rebuild) the underlying Fast-DDS DomainParticipant with
        /// the currently stored QoS configuration. Called from the constructor and
        /// from @c trigger_network_recovery_reset.
        eprosima::fastdds::dds::DomainParticipant *create_fastdds_participant();

        DomainId_t domain_id;
        bool recovery_enabled;
        eprosima::fastdds::dds::DomainParticipantQos cached_qos;
        bool used_xml_profile{false};

        // Shared by every operation that touches `participant` and by reset.
        std::shared_mutex reset_mutex;
        eprosima::fastdds::dds::DomainParticipant *participant;
        std::atomic<std::uint64_t> generation{0};

        // Mutual exclusion between `register_endpoint` and
        // `trigger_network_recovery_reset`. Serializes the two so a reset cannot
        // start while a registration is mid-flight (and vice-versa), eliminating
        // the otherwise-possible window where a freshly registered endpoint's
        // listener is firing callbacks that the reset path would have to drain
        // while holding the exclusive lifecycle lock (a guaranteed deadlock).
        std::mutex registration_mutex;

        std::mutex registered_types_mutex;
        std::unordered_map<std::string, TypeSupport> registered_types;
        std::shared_ptr<std::mutex> registered_topics_mutex;
        std::unordered_map<std::string, std::weak_ptr<topic>> registered_topics;

        // Endpoint registry for network-recovery reset.
        std::mutex endpoints_mutex;
        std::vector<std::weak_ptr<detail::resettable_endpoint>> endpoints;
    };
    using DomainParticipant = domain_participant;  // To match DDS domain participant name, as previously used directly

    template <typename data_pub_sub_type> TypeSupport domain_participant::register_type_locked()
    {
        // Caller already holds reset_mutex (shared or exclusive); only
        // registered_types_mutex is needed to serialize the map mutation and
        // the Fast-DDS register_type call.
        const std::lock_guard<std::mutex> lock{registered_types_mutex};

        // If a previous network-recovery reset failed to recreate the Fast-DDS
        // participant (create_participant returned nullptr — typically OOM or
        // an XML profile parse failure), `participant` is null. Surface that
        // rather than null-deref'ing inside the Fast-DDS call below; the next
        // confirmed network change will retry the recreation.
        if (participant == nullptr)
        {
            throw std::runtime_error{"domain_participant: Fast-DDS participant is not available "
                                     "(most likely a network-recovery recreate failed); see logs"};
        }

        auto pub_sub = std::make_unique<data_pub_sub_type>();
        const auto type_it = registered_types.find(pub_sub->get_name());
        if (type_it != registered_types.end())
        {
            return type_it->second;
        }
        else
        {
            TypeSupport type_support{pub_sub.release()};
            participant->register_type(type_support);
            registered_types.insert({type_support->get_name(), type_support});
            return type_support;
        }
    }

    template <typename data_pub_sub_type> TypeSupport domain_participant::register_type()
    {
        // Shared lifecycle lock: serializes us against a reset, so `participant`
        // is stable and the Fast-DDS-side registration we do below targets the
        // current participant (a reset would otherwise null-deref `participant`).
        const std::shared_lock<std::shared_mutex> reset_lock{reset_mutex};
        return register_type_locked<data_pub_sub_type>();
    }

    /**
     * @brief Creates a new DDS Domain Participant as a shared_ptr. The participant is automatically deleted
     * correctly on destroying its last shared_ptr.
     *
     * @param domain_id DDS domain_id, 0 by default
     * @param recovery_mode Whether to participate in network auto-recovery
     * (see @c network_recovery_mode).
     * @return std::shared_ptr<domain_participant>
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
     * @see
     * https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/domain/domainparticipant.html
     */
    PROVIZIO_DDS_API std::shared_ptr<domain_participant> make_domain_participant(
        DomainId_t domain_id = 0, network_recovery_mode recovery_mode = network_recovery_mode::env_var_controlled);
}  // namespace provizio::dds

#endif  // DDS_DOMAIN_PARTICIPANT
