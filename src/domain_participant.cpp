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

#include "provizio/dds/domain_participant.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>

#include "provizio/dds/detail/network_recovery_coordinator.h"
#include "provizio/dds/detail/resettable_endpoint.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/topic.h"
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/rtps/attributes/BuiltinTransports.hpp>

namespace provizio::dds
{
    namespace
    {
        // More reliable participants matching (only 5 multicast announcements are sent 0.1 seconds apart by default
        // and then only once in 3 seconds, which is often not enough when nearing 100% bandwidth load)
        const eprosima::fastdds::dds::Duration_t initial_announcements_period{0.05};        // NOLINT: Doesn't throw
        const eprosima::fastdds::dds::Duration_t lease_duration_announcement_period{1, 0};  // NOLINT: Doesn't throw
        constexpr std::uint32_t num_initial_discovery_announcements = 200;

        // Name of the env variable Fast-DDS reads to locate its XML profiles file.
        // Hardcoded here because Fast-DDS no longer exposes this name as a public
        // constant. Keep provizio_dds.py's _DomainParticipant.xml_profiles_env_variable
        // in sync with this constant.
        constexpr const char *const default_fastdds_env_variable = "FASTDDS_DEFAULT_PROFILES_FILE";
    }  // namespace

    domain_participant::domain_participant(const DomainId_t the_domain_id, const network_recovery_mode mode)
        : domain_id(the_domain_id), recovery_enabled(resolve_network_recovery_enabled(mode)),
          registered_topics_mutex(std::make_shared<std::mutex>())
    {
        if (auto *const file_path = std::getenv(default_fastdds_env_variable))  // NOLINT: getenv required
        {
            used_xml_profile = std::filesystem::exists(file_path) && !std::filesystem::is_directory(file_path);
        }

        auto participant_factory = dds::DomainParticipantFactory::get_shared_instance();
        if (!used_xml_profile)  // Unless configured via the XML profile
        {
            participant_factory->load_profiles();
            participant_factory->get_default_participant_qos(cached_qos);

            cached_qos.wire_protocol().builtin.discovery_config.initial_announcements.count =
                num_initial_discovery_announcements;
            cached_qos.wire_protocol().builtin.discovery_config.initial_announcements.period =
                initial_announcements_period;
            cached_qos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod =
                lease_duration_announcement_period;

#if defined(_MSC_VER) || defined(__APPLE__)
            // Disable shared memory transport on Windows and macOS unless the user
            // has explicitly configured transports via FASTDDS_BUILTIN_TRANSPORTS.
            // Fast-DDS's bundled Boost.Interprocess has a known bug where shared
            // memory segments and named semaphores from a previous DDS participant
            // are not cleaned up promptly. On Windows this causes assertion failures
            // in boost/interprocess/sync/windows/semaphore.hpp when a new participant
            // is created. On macOS, the default system-wide shared memory limits
            // (kern.sysv.shmmni=32) are quickly exhausted when creating participants
            // across multiple domain IDs, causing participant creation to hang
            // indefinitely. UDPv4-only transport avoids this issue with no practical
            // performance impact for the typical single-host or cross-network DDS use
            // cases this library targets.
            if (!std::getenv("FASTDDS_BUILTIN_TRANSPORTS"))  // NOLINT: getenv required
            {
                cached_qos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::UDPv4);
            }
#endif
        }

        participant = create_fastdds_participant();
        if (participant == nullptr)
        {
            // Fast-DDS create_participant returned nullptr — typically a
            // malformed XML profile, RLIMIT_NOFILE exhaustion, or memory
            // pressure. The participant is unusable in this state and
            // every subsequent register_type / register_topic /
            // make_publisher / make_subscriber call would either throw or
            // crash. Surface the failure to the caller now rather than
            // letting a partially-constructed participant escape.
            throw std::runtime_error{"domain_participant: Fast-DDS create_participant returned nullptr "
                                     "(check FASTDDS_DEFAULT_PROFILES_FILE / system limits / logs)"};
        }
        // generation starts at 1 — anything compared against 0 means "never built
        // against any participant", which teardown_state treats as a no-op.
        generation.store(1, std::memory_order_release);
    }

    eprosima::fastdds::dds::DomainParticipant *domain_participant::create_fastdds_participant()
    {
        auto participant_factory = dds::DomainParticipantFactory::get_shared_instance();
        return participant_factory->create_participant(
            domain_id, used_xml_profile ? PARTICIPANT_QOS_DEFAULT : cached_qos, nullptr);
    }

    domain_participant::~domain_participant()
    {
        {
            // Make sure neither of registered topic handles that are still alive won't try to unregister themselves
            // on destruction
            const std::lock_guard<std::mutex> lock{*registered_topics_mutex};
            for (auto &topic_pair : registered_topics)
            {
                auto handle = topic_pair.second.lock();
                if (handle)
                {
                    handle->release_mutex_prelocked();
                }
            }
        }

        // Ownership model: domain_participant is always held by shared_ptr. All
        // publishers/subscribers store a shared_ptr to their participant, so this
        // destructor only runs after every publisher/subscriber has been destroyed
        // (and has individually deleted its own Fast-DDS entities). This call cleans
        // up any residual entities (e.g. topics not individually freed) and is a
        // harmless no-op otherwise.
        if (participant != nullptr)
        {
            participant->delete_contained_entities();
            DomainParticipantFactory::get_instance()->delete_participant(participant);
        }
    }

    std::shared_ptr<topic> domain_participant::register_topic_locked(const std::string &topic_name,
                                                                     const std::string &type_name, const TopicQos &qos)
    {
        // Caller already holds reset_mutex (shared or exclusive). Only
        // registered_topics_mutex is needed here to serialize the map mutation
        // and the Fast-DDS create_topic call against other registrations.
        const std::lock_guard<std::mutex> lock{*registered_topics_mutex};

        if (participant == nullptr)
        {
            // Same rationale as register_type_locked: a prior reset's
            // create_participant failed; the participant is currently dead.
            throw std::runtime_error{"domain_participant: cannot register topic '" + topic_name +
                                     "' because the Fast-DDS participant is not available "
                                     "(most likely a network-recovery recreate failed); see logs"};
        }

        const auto topic_iterator = registered_topics.find(topic_name);
        if (topic_iterator != registered_topics.end())
        {
            auto handle = topic_iterator->second.lock();
            if (handle)
            {
                // Ensure the type matches the already registered topic's type
                const std::string existing_type{handle->get()->get_type_name()};
                if (existing_type != type_name)
                {
                    throw std::runtime_error{"Topic " + topic_name +
                                             " has been already registered, but with a different type (existing: '" +
                                             existing_type + "', requested: '" + type_name + "')!"};
                }

                // Ensure QoS is the same
                if (!(handle->qos() == qos))  // Yep, TopicQos defines operator== but not operator!=
                {
                    throw std::runtime_error{"Topic " + topic_name +
                                             " has been already registered, but with a different QoS!"};
                }
                return handle;
            }
        }

        auto handle = std::make_shared<topic>(participant, registered_topics_mutex,
                                              participant->create_topic(topic_name, type_name, qos), qos);
        registered_topics[topic_name] = handle;
        return handle;
    }

    std::shared_ptr<topic> domain_participant::register_topic(const std::string &topic_name,
                                                              const std::string &type_name, const TopicQos &qos)
    {
        // Shared lifecycle lock for external callers: keeps `participant`
        // stable across the create_topic call inside register_topic_locked.
        const std::shared_lock<std::shared_mutex> reset_lock{reset_mutex};
        return register_topic_locked(topic_name, type_name, qos);
    }

    void domain_participant::register_endpoint(const std::shared_ptr<detail::resettable_endpoint> &endpoint)
    {
        if (!endpoint)
        {
            return;
        }

        // Serialize against any in-flight or pending reset. Holding
        // registration_mutex for the whole body guarantees that
        // trigger_network_recovery_reset cannot observe an "endpoint in the
        // registry but state unbuilt" intermediate state.
        const std::lock_guard<std::mutex> reg_lock{registration_mutex};
        // Shared lifecycle lock keeps `participant` stable while we build state.
        // No recursive lock issue: this is the only place register_endpoint
        // acquires reset_mutex, and the build call below does not re-acquire it.
        const std::shared_lock<std::shared_mutex> reset_lock{reset_mutex};

        // After a failed network-recovery recreate, `participant` is left
        // null (see trigger_network_recovery_reset). Surface that to the
        // caller rather than null-dereferencing inside the build call —
        // consistent with register_type_locked / register_topic_locked.
        if (participant == nullptr)
        {
            throw std::runtime_error{"domain_participant: cannot register endpoint — the Fast-DDS participant "
                                     "is not available (most likely a network-recovery recreate failed); see logs"};
        }

        // Build initial state first. If it throws (topic creation failure,
        // DataWriter/Reader creation failure, etc.), we never add to the registry
        // — make_publisher / make_subscriber will surface the exception and the
        // partially-constructed handle will be destroyed without ever appearing
        // in the reset path.
        endpoint->on_new_participant_started(*participant);

        const std::lock_guard<std::mutex> endpoints_lock{endpoints_mutex};
        // Garbage-collect expired entries inline.
        endpoints.erase(
            std::remove_if(endpoints.begin(), endpoints.end(),
                           [](const std::weak_ptr<detail::resettable_endpoint> &weak) { return weak.expired(); }),
            endpoints.end());
        endpoints.emplace_back(endpoint);
    }

    void domain_participant::deregister_endpoint(const detail::resettable_endpoint *endpoint)
    {
        const std::lock_guard<std::mutex> lock{endpoints_mutex};
        endpoints.erase(std::remove_if(endpoints.begin(), endpoints.end(),
                                       [endpoint](const std::weak_ptr<detail::resettable_endpoint> &weak) {
                                           auto strong = weak.lock();
                                           // Remove expired entries (the typical case from a destructor whose
                                           // last strong reference has already been released) AND any entry
                                           // whose owned object matches the supplied raw pointer (a defensive
                                           // path for callers that still hold a strong reference).
                                           return !strong || strong.get() == endpoint;
                                       }),
                        endpoints.end());
    }

    void domain_participant::trigger_network_recovery_reset()
    {
        if (!recovery_enabled)
        {
            return;
        }

        // Serialize against new endpoint registrations for the whole reset. With
        // this in place, no endpoint can be registered while we are tearing down
        // and rebuilding the participant — so we never have to worry about
        // "endpoint exists but state is built against the old participant we just
        // destroyed".
        const std::lock_guard<std::mutex> reg_lock{registration_mutex};

        // Phase 1: snapshot the registered endpoints.
        std::vector<std::shared_ptr<detail::resettable_endpoint>> live_endpoints;
        {
            const std::lock_guard<std::mutex> lock{endpoints_mutex};
            live_endpoints.reserve(endpoints.size());
            for (const auto &weak : endpoints)
            {
                if (auto strong = weak.lock())
                {
                    live_endpoints.push_back(std::move(strong));
                }
            }
        }

        // Phase 2: detach listener callbacks WITHOUT the lifecycle lock held. A
        // user callback that re-enters provizio APIs (e.g. publish() on a sibling
        // publisher) can still acquire reset_mutex shared at this point and run
        // to completion, so the drain wait does not deadlock. After this phase
        // returns, no Fast-DDS listener callback owned by us is in flight.
        for (const auto &endpoint : live_endpoints)
        {
            endpoint->detach_for_reset();
        }

        // Phase 3: acquire the lifecycle lock exclusively. Other threads
        // holding the shared lock (publish, get_guid, ~publisher_handle,
        // register_topic, register_type, etc.) will let us in once they
        // finish — this acquire is bounded by the longest in-flight shared
        // holder.
        const std::unique_lock<std::shared_mutex> reset_lock{reset_mutex};

        // Phase 4: tear down endpoints against the old participant.
        //
        // If a prior reset left us in the dead state (create_participant
        // failed on the rebuild path → participant == nullptr), skip the
        // endpoint teardown / topic-map-clear / delete_contained_entities
        // entirely. The contained Fast-DDS objects were already freed by
        // the previous reset's delete_contained_entities, and the endpoint
        // handles' built_against_generation no longer matches the current
        // generation — their next teardown_state call would also be a
        // no-op via the generation check. Just fall through to the
        // create_fastdds_participant retry below.
        if (participant != nullptr)
        {
            for (const auto &endpoint : live_endpoints)
            {
                endpoint->on_participant_reset(*participant);
            }

            // Topic registry references the OLD participant — drop all topic handles.
            // The released topics are no-ops on the new participant because their
            // destructor reaches back into the cached topics map; we need to clear
            // both that map and any still-strong topic references before destroying
            // the participant.
            {
                const std::lock_guard<std::mutex> lock{*registered_topics_mutex};
                for (auto &topic_pair : registered_topics)
                {
                    auto handle = topic_pair.second.lock();
                    if (handle)
                    {
                        handle->release_mutex_prelocked();
                    }
                }
                registered_topics.clear();
            }

            // Destroy old participant.
            participant->delete_contained_entities();
            DomainParticipantFactory::get_instance()->delete_participant(participant);
            participant = nullptr;
        }

        // Refresh Fast-DDS's process-wide interface cache before recreating.
        // This is the WHOLE point of the reset on a network change: without
        // it, the new participant would use the same stale interface set
        // that the now-destroyed one bound to (the cache is initialised
        // once in SystemInfo's constructor and not refreshed on any
        // subsequent participant creation). See network_recovery.h's
        // docstring on refresh_fastdds_interface_cache for the full
        // rationale.
        refresh_fastdds_interface_cache();

        // Create new participant with the SAME QoS as the original.
        participant = create_fastdds_participant();
        if (participant == nullptr)
        {
            log_error() << "failed to recreate participant on domain " << domain_id
                        << "; endpoints left in torn-down state";
            // Bump generation anyway so any in-flight teardown observing the
            // mismatch skips its Fast-DDS-side deletes (the contained entities
            // were freed by delete_contained_entities above).
            generation.fetch_add(1, std::memory_order_acq_rel);
            return;
        }

        // Bump generation BEFORE rebuilding endpoints so on_new_participant_started
        // captures the new value when it sets the endpoint's built-against marker.
        generation.fetch_add(1, std::memory_order_acq_rel);

        // Re-register every type the old participant knew so endpoints rebuilt
        // by on_new_participant_started() can create their topics against the
        // new participant. The TypeSupport handles themselves are still valid;
        // only the participant-side registration is participant-scoped.
        // Log non-OK return codes — a rare failure here (e.g. OOM) would
        // otherwise surface only later as an opaque "create_datawriter
        // returned nullptr" when an endpoint tries to use the type.
        {
            const std::lock_guard<std::mutex> lock{registered_types_mutex};
            for (auto &type_pair : registered_types)
            {
                if (participant->register_type(type_pair.second) != RETCODE_OK)
                {
                    log_error() << "type re-registration failed on domain " << domain_id << " for type '"
                                << type_pair.first << "'";
                }
            }
        }

        // Phase 5: rebuild child endpoints against the new participant.
        for (const auto &endpoint : live_endpoints)
        {
            try
            {
                endpoint->on_new_participant_started(*participant);
            }
            catch (const std::exception &exception)
            {
                log_error() << "endpoint rebuild failed on domain " << domain_id << ": " << exception.what()
                            << "; this endpoint stays inactive (publish/take return failure) until the next "
                               "network-recovery reset rebuilds it";
            }
        }
    }

    std::shared_ptr<domain_participant> make_domain_participant(const DomainId_t domain_id,
                                                                const network_recovery_mode recovery_mode)
    {
        auto participant = std::make_shared<domain_participant>(domain_id, recovery_mode);
        if (resolve_network_recovery_enabled(recovery_mode))
        {
            detail::network_recovery_coordinator::instance().register_participant(participant);
        }
        return participant;
    }
}  // namespace provizio::dds
