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
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "provizio/dds/detail/network_recovery_coordinator.h"
#include "provizio/dds/detail/resettable_endpoint.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/topic.h"
#include <fastdds/dds/core/status/StatusMask.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/rtps/attributes/BuiltinTransports.hpp>
#include <fastdds/rtps/builtin/data/PublicationBuiltinTopicData.hpp>
#include <fastdds/rtps/builtin/data/SubscriptionBuiltinTopicData.hpp>
#include <fastdds/rtps/reader/ReaderDiscoveryStatus.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>
#include <fastdds/rtps/writer/WriterDiscoveryStatus.hpp>

namespace provizio::dds
{
    namespace
    {
        // ---- Auto-discovery (SPDP) tuning --------------------------------------------------------
        //
        // Participant discovery announcements are multicast best-effort, so some are lost on a busy
        // or lossy network. Fast-DDS counters this with an INITIAL burst of announcements at
        // participant creation plus a PERIODIC re-announcement thereafter. Set too high, both make
        // the library itself a primary source of UDP congestion once many participants (sensors +
        // clients) run at once: the initial burst is paid on EVERY participant creation (including
        // each network-recovery reset, which recreates the participant), and the periodic
        // re-announcement is paid by every participant forever, so its multicast rate scales with
        // the participant count.
        //
        // These defaults are de-escalated from a former 200-shot / 50 ms initial burst and a 1 s
        // periodic flood to a modest burst and a relaxed cadence that still discovers robustly (the
        // initial count stays well above Fast-DDS's own default of 5, so a lossy link still gets
        // several shots through) without flooding the network. All four are overridable at runtime
        // via the PROVIZIO_DDS_DISCOVERY_* env variables below — e.g. to tune for an unusually large
        // fleet or a particularly lossy link — without recompiling.
        //
        // Keep these defaults and the env-variable names in sync with python/provizio_dds.py
        // (QosDefaults and its _resolve_discovery_* helpers).
        constexpr std::uint32_t default_initial_announcement_count = 15;
        constexpr std::uint32_t default_initial_announcement_period_ms = 100;
        constexpr std::uint32_t default_announcement_period_ms = 3000;
        // Lease duration: how long a peer is considered alive without a fresh announcement. Raised
        // from Fast-DDS's 20 s default to 30 s so the relaxed announcement cadence has more margin
        // before a peer is wrongly declared lost when announcements are dropped on a congested or
        // lossy link (the only cost is detecting a genuinely-dead peer ~10 s later). Must stay
        // longer than the periodic announcement period above (a warning is logged otherwise).
        constexpr std::uint32_t default_lease_duration_ms = 30000;

        // Time-unit conversion factors for milliseconds <-> Fast-DDS Duration_t (seconds + nanosec).
        constexpr std::uint32_t ms_per_second = 1000U;          // milliseconds per second
        constexpr std::uint32_t ns_per_millisecond = 1000000U;  // nanoseconds per millisecond
        constexpr double ns_per_second = 1e9;                   // nanoseconds per second
        constexpr double ms_per_second_f = 1000.0;              // milliseconds per second (floating-point)

        constexpr const char *const initial_announcement_count_env =
            "PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_COUNT";
        constexpr const char *const initial_announcement_period_env =
            "PROVIZIO_DDS_DISCOVERY_INITIAL_ANNOUNCEMENT_PERIOD_MS";
        constexpr const char *const announcement_period_env = "PROVIZIO_DDS_DISCOVERY_ANNOUNCEMENT_PERIOD_MS";
        constexpr const char *const lease_duration_env = "PROVIZIO_DDS_DISCOVERY_LEASE_DURATION_MS";

        // Strictly-positive unsigned integer from env var `name`, else `fallback` (logged at
        // warning). Used by the discovery-tuning resolvers below; rejects unset/empty/non-numeric/
        // zero/overflowing input so a typo can never silently disable or wildly misconfigure
        // discovery.
        std::uint32_t resolve_positive_u32_env(const char *const name, const std::uint32_t fallback)
        {
            const char *const env = std::getenv(name);  // NOLINT: getenv required
            if (env == nullptr || *env == '\0')
            {
                return fallback;
            }
            char *end = nullptr;
            const std::uint64_t parsed = std::strtoull(env, &end, 10);  // NOLINT: C numeric parse
            if (end != nullptr && *end == '\0' && parsed > 0U &&
                parsed <= static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))
            {
                return static_cast<std::uint32_t>(parsed);
            }
            log_warning() << "ignoring invalid " << name << "='" << env << "'; using default " << fallback;
            return fallback;
        }

        // Milliseconds -> Fast-DDS Duration_t. milliseconds is uint32, so seconds (ms / 1000) always
        // fits int32 and nanosec (< 1e9) fits uint32 — no overflow for any accepted value.
        eprosima::fastdds::dds::Duration_t milliseconds_to_duration(const std::uint32_t milliseconds)
        {
            return eprosima::fastdds::dds::Duration_t{
                static_cast<std::int32_t>(milliseconds / ms_per_second),
                static_cast<std::uint32_t>((milliseconds % ms_per_second) * ns_per_millisecond)};
        }

        double duration_to_seconds(const eprosima::fastdds::dds::Duration_t &duration)
        {
            return static_cast<double>(duration.seconds) + static_cast<double>(duration.nanosec) / ns_per_second;
        }

        std::uint32_t resolve_initial_announcement_count()
        {
            return resolve_positive_u32_env(initial_announcement_count_env, default_initial_announcement_count);
        }

        eprosima::fastdds::dds::Duration_t resolve_initial_announcement_period()
        {
            return milliseconds_to_duration(
                resolve_positive_u32_env(initial_announcement_period_env, default_initial_announcement_period_ms));
        }

        eprosima::fastdds::dds::Duration_t resolve_announcement_period()
        {
            return milliseconds_to_duration(
                resolve_positive_u32_env(announcement_period_env, default_announcement_period_ms));
        }

        eprosima::fastdds::dds::Duration_t resolve_lease_duration()
        {
            return milliseconds_to_duration(resolve_positive_u32_env(lease_duration_env, default_lease_duration_ms));
        }

        // Default UDP socket send/recv buffer ceiling (16 MiB). Large enough that a
        // multi-MB sample's fragments don't overflow the receive socket buffer (which
        // would drop fragments and stall reliable delivery); the OS clamps it to
        // net.core.rmem_max / wmem_max (raise those for headroom past the ~208 KB
        // default). Override via PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE (bytes) to tune or
        // lower it on memory-constrained ARM targets.
        constexpr std::uint32_t default_udp_socket_buffer_size = 16U * 1024U * 1024U;

        std::uint32_t resolve_udp_socket_buffer_size()
        {
            const char *const env = std::getenv("PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE");  // NOLINT: getenv required
            if (env != nullptr && *env != '\0')
            {
                char *end = nullptr;
                const std::uint64_t parsed = std::strtoull(env, &end, 10);  // NOLINT: C numeric parse
                if (end != nullptr && *end == '\0' && parsed > 0U &&
                    parsed <= static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))
                {
                    return static_cast<std::uint32_t>(parsed);
                }
                log_error() << "ignoring invalid PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE='" << env << "'; using default "
                            << default_udp_socket_buffer_size;
            }
            return default_udp_socket_buffer_size;
        }

#ifdef __linux__
        // Single unsigned value from a /proc or /sys file, or 0 if it can't be read.
        std::uint64_t read_kernel_value(const char *const path)
        {
            std::ifstream file{path};
            std::uint64_t value = 0;
            if (file >> value)
            {
                return value;
            }
            return 0;
        }
#endif  // __linux__

        // The kernel silently clamps SO_RCVBUF/SO_SNDBUF to net.core.rmem_max/wmem_max
        // (SO_RCVBUFFORCE, which would bypass it, needs CAP_NET_ADMIN). On a stock host
        // those default to ~208 KB, so the buffer chosen above can be reduced by two
        // orders of magnitude with nothing to show for it: large samples then lose
        // fragments under load and reliable delivery stalls or turns bursty. Report the
        // shortfall once per process, naming the sysctl to raise, rather than letting the
        // requested size look effective when it is not.
        void warn_if_socket_buffers_are_capped([[maybe_unused]] const std::uint32_t requested)
        {
#ifdef __linux__
            static std::once_flag warned;
            std::call_once(warned, [requested] {
                const std::uint64_t rmem_max = read_kernel_value("/proc/sys/net/core/rmem_max");
                const std::uint64_t wmem_max = read_kernel_value("/proc/sys/net/core/wmem_max");
                for (const auto &limit :
                     {std::make_pair("net.core.rmem_max", rmem_max), std::make_pair("net.core.wmem_max", wmem_max)})
                {
                    if (limit.second != 0 && limit.second < requested)
                    {
                        log_warning() << "requested " << requested << "-byte UDP socket buffers but " << limit.first
                                      << " is " << limit.second << ", so the kernel caps them there; large samples "
                                      << "(camera frames, point clouds) may lose fragments under load. Raise it, e.g. "
                                      << "sysctl -w " << limit.first << "=" << requested;
                    }
                }
            });
#endif  // __linux__
        }

        // Shared-memory segment size, configured separately from the UDP socket buffer
        // size. Fast-DDS derives the SHM segment from BuiltinTransportsOptions'
        // sockets_buffer_size, so a large UDP buffer silently inflates every
        // participant's segment too (16 MiB of socket buffer produces a ~33.5 MiB
        // segment against Fast-DDS's ~537 KiB default). Multiplied by the participants on
        // a host that is not a big machine, that exhausts /dev/shm — and once it is full,
        // SHM transport registration fails for every new participant, silently degrading
        // all same-host traffic to UDP. Keeping the two knobs separate lets a
        // memory-constrained target shrink the segment without giving up UDP buffering.
        // The default preserves the historical size, so behaviour is unchanged unless set.
        constexpr std::uint32_t unset_shm_segment_size = 0;

        std::uint32_t resolve_shm_segment_size()
        {
            const char *const env = std::getenv("PROVIZIO_DDS_SHM_SEGMENT_SIZE");  // NOLINT: getenv required
            if (env != nullptr && *env != '\0')
            {
                char *end = nullptr;
                const std::uint64_t parsed = std::strtoull(env, &end, 10);  // NOLINT: C numeric parse
                if (end != nullptr && *end == '\0' && parsed > 0U &&
                    parsed <= static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))
                {
                    return static_cast<std::uint32_t>(parsed);
                }
                log_error() << "ignoring invalid PROVIZIO_DDS_SHM_SEGMENT_SIZE='" << env << "'";
            }
            return unset_shm_segment_size;
        }

        // Report a /dev/shm that cannot fit a handful more segments of the size this
        // participant is about to allocate. Exhaustion shows up only as an obscure
        // "Failed to create segment" from Fast-DDS followed by silent UDP fallback.
        void warn_if_shared_memory_is_nearly_full()
        {
#ifdef __linux__
            static std::once_flag warned;
            std::call_once(warned, [] {
                std::error_code error;
                const auto space = std::filesystem::space("/dev/shm", error);
                if (error || space.capacity == 0)
                {
                    return;
                }
                constexpr std::uintmax_t bytes_per_mebibyte = std::uintmax_t{1024} * std::uintmax_t{1024};
                constexpr std::uintmax_t low_water_bytes = std::uintmax_t{256} * bytes_per_mebibyte;
                if (space.available < low_water_bytes)
                {
                    log_warning() << "/dev/shm has only " << (space.available / bytes_per_mebibyte) << " MiB free of "
                                  << (space.capacity / bytes_per_mebibyte)
                                  << " MiB; shared-memory transport registration can fail and fall back to UDP. "
                                  << "Stale fastdds_* segments from participants that did not exit cleanly are the "
                                  << "usual cause";
                }
            });
#endif  // __linux__
        }

        // Name of the env variable Fast-DDS reads to locate its XML profiles file.
        // Hardcoded here because Fast-DDS no longer exposes this name as a public
        // constant. Keep provizio_dds.py's _DomainParticipant.xml_profiles_env_variable
        // in sync with this constant.
        constexpr const char *const default_fastdds_env_variable = "FASTDDS_DEFAULT_PROFILES_FILE";
    }  // namespace

    namespace detail
    {
        // Forwards Fast-DDS DomainParticipantListener discovery events to a
        // domain_participant::on_discovered_endpoint_callback. Owned by
        // domain_participant and created + attached eagerly in its constructor
        // (it also drives the match-publisher deferred-subscriber default, so it
        // cannot be lazy); the optional user callback rides on the same listener.
        class discovery_listener_impl final : public eprosima::fastdds::dds::DomainParticipantListener
        {
          public:
            using callback_type = domain_participant::on_discovered_endpoint_callback;

            // The owner reference is stable for the entire lifetime of this
            // listener: the listener is a member of `owner` and torn down only
            // by ~domain_participant. Across network-recovery resets only the
            // underlying Fast-DDS DomainParticipant is swapped — the
            // provizio::dds::domain_participant C++ object stays the same, so
            // the reference remains valid.
            explicit discovery_listener_impl(domain_participant &owner) noexcept : owner(owner)
            {
            }

            void set_callback(callback_type new_callback, endpoint_kind new_kinds)
            {
                const std::lock_guard<std::mutex> lock{callback_mutex};
                callback = std::move(new_callback);
                active_kinds = new_kinds;
            }

            void on_data_writer_discovery(eprosima::fastdds::dds::DomainParticipant * /*participant*/,
                                          eprosima::fastdds::rtps::WriterDiscoveryStatus reason,
                                          const eprosima::fastdds::dds::PublicationBuiltinTopicData &info,
                                          bool &should_be_ignored) override
            {
                // Purely observational listener: never ask Fast-DDS to ignore a
                // discovered endpoint. (Fast-DDS already pre-initialises this to
                // false at the call site before invoking the listener; we set it
                // explicitly to keep the "never ignore" intent clear.)
                should_be_ignored = false;
                bool discovered = false;
                switch (reason)
                {
                case eprosima::fastdds::rtps::WriterDiscoveryStatus::DISCOVERED_WRITER:
                    discovered = true;
                    break;
                case eprosima::fastdds::rtps::WriterDiscoveryStatus::REMOVED_WRITER:
                    discovered = false;
                    break;
                default:
                    // CHANGED_QOS_WRITER / IGNORED_WRITER — not relevant to a
                    // "is data flowing on this topic?" subscriber.
                    return;
                }
                invoke(info, endpoint_kind::data_writer, discovered);
            }

            void on_data_reader_discovery(eprosima::fastdds::dds::DomainParticipant * /*participant*/,
                                          eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
                                          const eprosima::fastdds::dds::SubscriptionBuiltinTopicData &info,
                                          bool &should_be_ignored) override
            {
                // Observational only — never request the endpoint be ignored.
                should_be_ignored = false;
                bool discovered = false;
                switch (reason)
                {
                case eprosima::fastdds::rtps::ReaderDiscoveryStatus::DISCOVERED_READER:
                    discovered = true;
                    break;
                case eprosima::fastdds::rtps::ReaderDiscoveryStatus::REMOVED_READER:
                    discovered = false;
                    break;
                default:
                    return;
                }
                invoke(info, endpoint_kind::data_reader, discovered);
            }

          private:
            // info_type is Fast-DDS's Publication/SubscriptionBuiltinTopicData;
            // both expose topic_name / type_name with a throwing to_string(), and
            // both carry reliability / durability QoS policies as public members
            // (offered for a DataWriter, requested for a DataReader).
            template <typename info_type> void invoke(const info_type &info, endpoint_kind kind, bool discovered)
            {
                // Wrap the ENTIRE body — the info.*.to_string() conversions, the
                // std::function copy, the lock acquisition, the internal deferred-
                // subscriber resolution, and the user callback — because anything
                // that escapes this method into the Fast-DDS discovery thread
                // triggers std::terminate. to_string() and std::function's copy
                // constructor can allocate (std::bad_alloc); it isn't only the user
                // callback that can throw.
                try
                {
                    // Convert the topic name at most once per call: it is needed by the internal
                    // deferred-subscriber resolution (writers) and again by the user callback, and
                    // to_string() allocates on the Fast-DDS discovery thread for every event. Lazy so a
                    // data_reader event with no user callback still performs zero conversions; the
                    // conversion stays inside this try so a bad_alloc from it cannot escape the thread.
                    bool topic_name_converted = false;
                    std::string topic_name_str;
                    const auto topic_name = [&]() -> const std::string & {
                        if (!topic_name_converted)
                        {
                            topic_name_str = info.topic_name.to_string();
                            topic_name_converted = true;
                        }
                        return topic_name_str;
                    };

                    // INTERNAL FIRST, unconditionally: a discovered remote DataWriter
                    // resolves any match-publisher subscribers parked on its topic
                    // (adopt the writer's offered reliability). This must run even when
                    // no user on_discovered_endpoint callback is registered and even if
                    // the user filtered out data_writer events — the deferred-subscriber
                    // default depends on it. resolve_deferred_for_writer only touches
                    // deferred_mutex-guarded state and launches each parked subscriber's
                    // own build thread; it never builds an endpoint on this discovery
                    // thread (which would deadlock against a concurrent reset).
                    if (kind == endpoint_kind::data_writer)
                    {
                        if (discovered)
                        {
                            owner.resolve_deferred_for_writer(topic_name(), info.reliability.kind);
                        }
                        else
                        {
                            // REMOVED_WRITER: maintain the per-reliability live-writer count so the
                            // adopted reliability is re-derived / dropped as writers leave. The removed
                            // writer's offered reliability is carried on the same discovery info.
                            owner.on_writer_removed(topic_name(), info.reliability.kind);
                        }
                    }

                    // Snapshot under the lock so a concurrent set_callback
                    // can't swap the function out mid-call.
                    callback_type local_callback;
                    endpoint_kind local_kinds{endpoint_kind::data_writer};
                    {
                        const std::lock_guard<std::mutex> lock{callback_mutex};
                        local_callback = callback;
                        local_kinds = active_kinds;
                    }
                    if (!local_callback || !any(local_kinds & kind))
                    {
                        return;
                    }
                    // Convert type_name only now that the callback is known to fire (topic_name()
                    // reuses the at-most-once conversion above), inside this try so a bad_alloc from
                    // the conversion can't escape into the discovery thread. The reliability /
                    // durability kinds are plain enum reads off the discovery info — offered for a
                    // DataWriter, requested for a DataReader — forwarded so a recording bridge can
                    // match QoS per topic.
                    local_callback(owner, topic_name(), info.type_name.to_string(), kind, discovered,
                                   info.reliability.kind, info.durability.kind);
                }
                catch (const std::exception &exception)
                {
                    // A throwing user callback, or any internal allocation failure
                    // (string conversion / std::function copy), must not propagate
                    // out into the Fast-DDS discovery thread. Log and swallow,
                    // mirroring the Python side. The logging path itself allocates
                    // (std::ostringstream growth) and can throw std::bad_alloc under
                    // memory pressure, so it is wrapped too — nothing may escape here.
                    try
                    {
                        log_error() << "on_discovered_endpoint dispatch threw: " << exception.what();
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                    {
                    }
                }
                catch (...)
                {
                    try
                    {
                        log_error() << "on_discovered_endpoint dispatch threw a non-std::exception";
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                    {
                    }
                }
            }

            domain_participant &owner;
            mutable std::mutex callback_mutex;
            callback_type callback;
            endpoint_kind active_kinds{endpoint_kind::data_writer};
        };
    }  // namespace detail

    domain_participant::domain_participant(const DomainId_t the_domain_id, const network_recovery_mode mode,
                                           on_discovered_endpoint_callback initial_discovery_callback,
                                           const endpoint_kind initial_discovery_kinds, const transport_mode transport)
        : domain_id(the_domain_id), recovery_enabled(resolve_network_recovery_enabled(mode)),
          registered_topics_mutex(std::make_shared<std::mutex>())
    {
        // Install the discovery listener EAGERLY, BEFORE create_fastdds_participant,
        // so Fast-DDS attaches it at participant-creation time and it sees every
        // discovery event from the very first SEDP exchange. This is now mandatory
        // (not opt-in): the listener also drives the match-publisher subscriber
        // default — a discovered remote DataWriter resolves the deferred reader of
        // any subscriber parked on that topic (see resolve_deferred_for_writer).
        // Missing the first writer event would leave such a subscriber waiting
        // forever, so the listener can no longer be lazily installed only when a
        // user callback is registered. The optional user on_discovered_endpoint
        // callback rides on the SAME listener; when none is passed the listener
        // still runs but its user-callback slot is empty (a cheap enum-read + map
        // lookup per discovery event).
        discovery_listener = std::make_unique<detail::discovery_listener_impl>(*this);
        if (initial_discovery_callback)
        {
            discovery_listener->set_callback(std::move(initial_discovery_callback), initial_discovery_kinds);
        }

        if (auto *const file_path = std::getenv(default_fastdds_env_variable))  // NOLINT: getenv required
        {
            used_xml_profile = std::filesystem::exists(file_path) && !std::filesystem::is_directory(file_path);
        }

        auto participant_factory = dds::DomainParticipantFactory::get_shared_instance();
        if (!used_xml_profile)  // Unless configured via the XML profile
        {
            participant_factory->load_profiles();
            participant_factory->get_default_participant_qos(cached_qos);

            auto &discovery_config = cached_qos.wire_protocol().builtin.discovery_config;
            discovery_config.initial_announcements.count = resolve_initial_announcement_count();
            discovery_config.initial_announcements.period = resolve_initial_announcement_period();
            discovery_config.leaseDuration = resolve_lease_duration();
            const auto announcement_period = resolve_announcement_period();
            discovery_config.leaseDuration_announcementperiod = announcement_period;
            // Fast-DDS guidance: the periodic re-announcement must stay shorter than the lease
            // duration, or a peer can be declared lost in the gap between announcements. The default
            // 3 s is well clear of the 30 s default lease; warn only if an env override crossed it.
            if (duration_to_seconds(announcement_period) >= duration_to_seconds(discovery_config.leaseDuration))
            {
                log_warning() << announcement_period_env << " ("
                              << duration_to_seconds(announcement_period) * ms_per_second_f
                              << " ms) is >= the participant lease duration ("
                              << duration_to_seconds(discovery_config.leaseDuration) * ms_per_second_f
                              << " ms); peers may be declared lost between announcements";
            }

            // Transport tuning (skipped entirely when FASTDDS_BUILTIN_TRANSPORTS hands
            // control to the user). Two things happen here:
            //
            //   1. Enlarge the UDP socket send/recv buffers so a large sample (camera
            //      frame, point cloud) that fragments into many datagrams isn't dropped
            //      when the receive buffer overflows — the dominant lever for reliable
            //      large-data delivery. It's a ceiling, clamped by the OS to
            //      net.core.rmem_max / wmem_max. Applied on every platform and in BOTH
            //      transport modes, because the UDP path carries any cross-host peer
            //      regardless of shared memory.
            //   2. Choose shared-memory vs UDP-only. SHM (zero-copy for same-host
            //      same-version peers) is on by default on Linux. It is force-disabled on
            //      Windows/macOS because Fast-DDS's bundled Boost.Interprocess leaks SHM
            //      segments/named semaphores between participants — causing assertion
            //      failures on Windows (boost/interprocess/sync/windows/semaphore.hpp) and
            //      system-wide shm-limit (kern.sysv.shmmni) exhaustion hangs on macOS. It
            //      is also disabled on any platform when transport_mode::udp_only is
            //      requested (e.g. a recorder bridging mismatched Fast-DDS major versions,
            //      where cross-major SHM negotiation degrades large-data throughput).
            if (!std::getenv("FASTDDS_BUILTIN_TRANSPORTS"))  // NOLINT: getenv required
            {
                const bool platform_allows_shm =
#if defined(_MSC_VER) || defined(__APPLE__)
                    false;  // Windows/macOS: Boost.Interprocess cleanup bug — UDP-only.
#else
                    true;
#endif
                const bool use_shared_memory = platform_allows_shm && (transport != transport_mode::udp_only);
                eprosima::fastdds::rtps::BuiltinTransportsOptions transport_options;
                const std::uint32_t socket_buffer_size = resolve_udp_socket_buffer_size();
                transport_options.sockets_buffer_size = socket_buffer_size;
                cached_qos.setup_transports(use_shared_memory ? eprosima::fastdds::rtps::BuiltinTransports::DEFAULT
                                                              : eprosima::fastdds::rtps::BuiltinTransports::UDPv4,
                                            transport_options);
                warn_if_socket_buffers_are_capped(socket_buffer_size);

                if (use_shared_memory)
                {
                    // setup_transports leaves the descriptors it built in user_transports, so
                    // the shared-memory segment can be resized here without reimplementing
                    // (and drifting from) the rest of what it configures.
                    const std::uint32_t shm_segment_size = resolve_shm_segment_size();
                    if (shm_segment_size != unset_shm_segment_size)
                    {
                        for (const auto &descriptor : cached_qos.transport().user_transports)
                        {
                            const auto shm =
                                std::dynamic_pointer_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor>(
                                    descriptor);
                            if (shm)
                            {
                                shm->segment_size(shm_segment_size);
                            }
                        }
                    }
                    warn_if_shared_memory_is_nearly_full();
                }
            }
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
        if (detail::consume_forced_participant_creation_failure())
        {
            // Test hook: behave exactly as Fast-DDS does when it cannot create a
            // participant — return null without throwing.
            log_error() << "participant creation failed on domain " << domain_id
                        << " (forced by fail_next_participant_creation_for_test)";
            return nullptr;
        }

        auto participant_factory = dds::DomainParticipantFactory::get_shared_instance();
        // Pass the (possibly null) discovery listener to Fast-DDS at create
        // time so it's attached BEFORE the new participant starts internal
        // discovery — no window during which discovery events arrive without
        // a listener to deliver them. Concurrency: this function runs only
        // (a) from the constructor (single-threaded — no other thread holds a
        // reference yet) and (b) from trigger_network_recovery_reset which
        // holds reset_mutex exclusive, blocking any concurrent
        // on_discovered_endpoint (which takes it shared). So reading
        // discovery_listener.get() here is safe without discovery_listener_mutex.
        // StatusMask::none() — we never use the Fast-DDS *status* callbacks
        // on this listener; discovery callbacks fire regardless of the mask.
        // Attach the listener whenever it exists. It is now created eagerly at
        // construction and stays attached for the participant's whole life because
        // it drives the match-publisher deferred-subscriber default — not just the
        // optional user callback. (It is a member always present after construction;
        // the && guard is belt-and-braces for the brief pre-init window.) A user
        // on_discovered_endpoint(unregister) clears only the user callback, leaving
        // the listener attached for internal use, so attachment no longer gates on
        // whether a user callback is installed.
        auto *const listener = discovery_listener ? discovery_listener.get() : nullptr;
        return participant_factory->create_participant(domain_id,
                                                       used_xml_profile ? PARTICIPANT_QOS_DEFAULT : cached_qos,
                                                       listener, eprosima::fastdds::dds::StatusMask::none());
    }

    void domain_participant::on_discovered_endpoint(on_discovered_endpoint_callback callback, endpoint_kind kinds)
    {
        // reset_mutex shared keeps the underlying Fast-DDS participant pointer
        // stable for the (un)install set_listener call. The reset path takes it
        // exclusive, so an in-flight reset will simply finish first.
        const std::shared_lock<std::shared_mutex> reset_lock{reset_mutex};
        const std::lock_guard<std::mutex> listener_lock{discovery_listener_mutex};

        if (!callback)
        {
            // Unregister the USER callback only: clear the stored callback so any
            // in-flight (or about-to-fire) user dispatch becomes a no-op. Do NOT
            // detach the listener from Fast-DDS — it is now installed eagerly and
            // permanently to drive the match-publisher deferred-subscriber default
            // (a discovered writer must still resolve parked subscribers even with
            // no user callback). The listener object also outlives any unregister
            // until ~domain_participant for the same use-after-free reason as
            // before: Fast-DDS' callback dispatch is not synchronous with
            // set_listener, so a stale dispatch can still arrive — but it now just
            // reads the empty user callback (after doing the cheap internal resolve)
            // and returns.
            if (discovery_listener)
            {
                discovery_listener->set_callback({}, kinds);
            }
            return;
        }

        if (!discovery_listener)
        {
            // Defensive: the listener is created eagerly in the constructor, so this
            // branch is normally dead. Recreate it if somehow absent.
            discovery_listener = std::make_unique<detail::discovery_listener_impl>(*this);
        }
        discovery_listener->set_callback(std::move(callback), kinds);

        // The listener is already attached to Fast-DDS from participant creation
        // (eager install), so re-attaching here is redundant in the common case;
        // keep it for the defensive recreate branch above and as a harmless no-op
        // otherwise.
        if (participant != nullptr)
        {
            participant->set_listener(discovery_listener.get(), eprosima::fastdds::dds::StatusMask::none());
        }
    }

    bool domain_participant::is_known_type(const std::string &type_name) const
    {
        const std::lock_guard<std::mutex> lock{registered_types_mutex};
        return registered_types.find(type_name) != registered_types.end();
    }

    std::vector<std::string> domain_participant::known_types() const
    {
        const std::lock_guard<std::mutex> lock{registered_types_mutex};
        std::vector<std::string> names;
        names.reserve(registered_types.size());
        for (const auto &entry : registered_types)
        {
            names.push_back(entry.first);
        }
        return names;
    }

    domain_participant::~domain_participant()
    {
        // No deferred-subscriber worker to stop here: each match-mode subscriber owns
        // its own std::async build (subscriber_handle::build_future) and joins it in its
        // destructor. Every endpoint holds a strong shared_ptr to this participant, so
        // ~domain_participant only runs after every endpoint is gone — i.e. after every
        // deferred build has already been joined and every subscriber has deregistered
        // from the deferred registry below.

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

    void domain_participant::register_deferred_subscriber(const std::string &topic_name,
                                                          detail::resettable_endpoint *handle)
    {
        if (handle == nullptr)
        {
            return;
        }

        // Called from subscriber_handle::build_state, which already holds reset_mutex
        // (shared). deferred_mutex is taken here as a leaf — order reset→deferred. The
        // build thread (run_deferred_build) never holds deferred_mutex while taking
        // reset_mutex, so there is no inversion (see the lock-order note on the members
        // in domain_participant.h).
        const std::lock_guard<std::mutex> lock{deferred_mutex};

        // Park a NON-owning back-reference BEFORE dispatching, even when a writer was already
        // discovered. Parking is what lets a FAILED build (e.g. create_datareader returns null
        // under resource exhaustion) be retried by the next discovered writer instead of leaving
        // the subscriber inactive until the next reset — a successful build deregisters itself
        // (build_deferred_locked → deregister_deferred_subscriber). Idempotent per (topic, handle):
        // a network-recovery reset re-runs build_state for a still-unresolved subscriber, so skip
        // an already-parked handle rather than accumulate duplicates (which would grow
        // deferred_subscribers across resets and cause redundant dispatch).
        auto range = deferred_subscribers.equal_range(topic_name);
        bool already_parked = false;
        for (auto it = range.first; it != range.second; ++it)
        {
            if (it->second == handle)
            {
                already_parked = true;
                break;
            }
        }
        if (!already_parked)
        {
            deferred_subscribers.emplace(topic_name, handle);
        }

        const auto cached = discovered_writer_reliability.find(topic_name);
        if (cached != discovered_writer_reliability.end())
        {
            // Writer already discovered on this topic before the subscriber registered: dispatch
            // the build immediately (closes the writer-before-subscriber race) in addition to
            // parking. start_deferred_build only spawns the subscriber's own std::async build
            // thread; the actual build re-takes the locks in the canonical order, off this thread.
            handle->start_deferred_build(cached->second.adopted);
        }
        // Otherwise the handle waits parked until resolve_deferred_for_writer fires for this topic.
    }

    void domain_participant::deregister_deferred_subscriber(const std::string &topic_name,
                                                            detail::resettable_endpoint *handle)
    {
        // Called from ~subscriber_handle under the SAME deferred_mutex that
        // resolve_deferred_for_writer / register_deferred_subscriber use to reach a parked
        // handle, so once this returns the discovery thread can no longer dispatch the
        // dying subscriber. A handle that was already dispatched (and thus already erased)
        // is simply not found — harmless.
        const std::lock_guard<std::mutex> lock{deferred_mutex};
        auto range = deferred_subscribers.equal_range(topic_name);
        for (auto it = range.first; it != range.second;)
        {
            if (it->second == handle)
            {
                it = deferred_subscribers.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void domain_participant::resolve_deferred_for_writer(
        const std::string &topic_name, const eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability)
    {
        // Runs on the Fast-DDS discovery thread. Touches only deferred_mutex-guarded
        // state and launches each parked subscriber's own build thread — it never builds
        // an endpoint here (that would deadlock against a concurrent reset on this very
        // thread). Holding deferred_mutex across the dispatch keeps each parked raw
        // pointer valid: ~subscriber_handle takes the same mutex to deregister, so it
        // cannot free a handle mid-dispatch.
        const std::lock_guard<std::mutex> lock{deferred_mutex};

        // Match-first: the FIRST writer to appear on an otherwise-writer-less topic fixes the
        // adopted reliability; while it (and others of its kind) remain a later heterogeneous
        // writer does not change it. A fresh entry has an empty live_counts map (on_writer_removed
        // erases emptied entries), so "was the topic writer-less?" is exactly live_counts.empty().
        auto &state = discovered_writer_reliability[topic_name];
        const bool topic_was_writerless = state.live_counts.empty();
        // Track this writer toward the topic's per-reliability live-writer count, so the adopted
        // value can be re-derived / dropped as writers are removed (see on_writer_removed).
        ++state.live_counts[reliability];
        if (topic_was_writerless)
        {
            state.adopted = reliability;
        }
        // Resolve against the adopted reliability (what every parked and any future subscriber on
        // this topic will adopt), not necessarily this writer's.
        const auto effective = state.adopted;

        // Spawn each parked subscriber's deferred build. Subscribers are NOT erased here: a build
        // that FAILS (e.g. create_datareader returns null under resource exhaustion) leaves the
        // subscriber parked so the next discovered writer on this topic retries it, instead of the
        // subscriber staying inactive until the next network-recovery reset. A subscriber whose
        // build SUCCEEDS deregisters itself (build_deferred_locked → deregister_deferred_subscriber)
        // so it is not re-dispatched. start_deferred_build only relaunches once a prior build has
        // completed, so repeated writer events while a build is in flight (or after one already
        // succeeded, in the brief window before it deregisters) are cheap no-ops, not duplicate builds.
        auto range = deferred_subscribers.equal_range(topic_name);
        for (auto it = range.first; it != range.second; ++it)
        {
            it->second->start_deferred_build(effective);
        }
    }

    void domain_participant::on_writer_removed(const std::string &topic_name,
                                               const eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability)
    {
        // Runs on the Fast-DDS discovery thread; only touches deferred_mutex-guarded state.
        const std::lock_guard<std::mutex> lock{deferred_mutex};

        const auto state_it = discovered_writer_reliability.find(topic_name);
        if (state_it == discovered_writer_reliability.end())
        {
            // A REMOVED without a matching DISCOVERED we counted — nothing to do.
            return;
        }
        auto &state = state_it->second;

        const auto count_it = state.live_counts.find(reliability);
        if (count_it == state.live_counts.end())
        {
            // A REMOVED for a reliability kind we never counted on this topic — nothing to do.
            return;
        }
        if (--count_it->second == 0)
        {
            state.live_counts.erase(count_it);
        }

        if (state.live_counts.empty())
        {
            // Last writer on this topic (of any kind) is gone: drop the entry so a match-mode
            // subscriber created later defers for a fresh writer instead of adopting a stale
            // value (and so the registry can't grow without bound as topics churn).
            discovered_writer_reliability.erase(state_it);
            return;
        }

        // Writers remain. If no live writer still offers the adopted reliability, re-derive it
        // from a still-live kind — otherwise a subscriber created now would adopt a reliability
        // that matches none of the remaining writers (e.g. RELIABLE adopted while only a
        // BEST_EFFORT writer is left, which a RELIABLE reader silently fails to match). Match-first
        // is preserved while the writer that fixed the value is alive; this only fires once it is
        // gone. Any remaining kind is a valid choice — pick an arbitrary one (live_counts is an
        // unordered_map, so begin() is unspecified, which is fine here).
        if (state.live_counts.find(state.adopted) == state.live_counts.end())
        {
            state.adopted = state.live_counts.begin()->first;
        }
    }

    void domain_participant::run_deferred_build(detail::resettable_endpoint *handle,
                                                const eprosima::fastdds::dds::ReliabilityQosPolicyKind resolved)
    {
        // Runs on the std::async thread spawned by subscriber_handle::start_deferred_build.
        // Lock order matches register_endpoint's initial build — registration_mutex
        // (serialize against resets) then reset_mutex — but reset_mutex is taken
        // EXCLUSIVELY here, unlike register_endpoint's shared acquire. The reason:
        // build_deferred_locked → build_state mutates state that is ALREADY user-visible
        // (data_reader, subscriber, the_topic, built_against_generation), while
        // get_guid() / get_num_matched_publishers() read those members under reset_mutex
        // SHARED. The initial build can use a shared lock because the handle hasn't
        // escaped to the caller yet, so nothing reads it concurrently; the deferred build
        // runs long after make_subscriber returned, so it must exclude those concurrent
        // readers to avoid a data race on the swapped-in reader. This mirrors the reset
        // path, which rebuilds endpoints under the same exclusive lock.
        // build_deferred_locked → build_state uses the _locked register_topic variant and
        // the participant ref we pass, so it does NOT re-acquire either lock. The handle is
        // kept alive by the subscriber, which waits on its build_future in ~subscriber_handle
        // before tearing down — so `handle` is valid for the whole of this call.
        try
        {
            const std::lock_guard<std::mutex> reg_lock{registration_mutex};
            const std::unique_lock<std::shared_mutex> reset_lock{reset_mutex};
            if (participant != nullptr)
            {
                handle->build_deferred_locked(*participant, resolved);
            }
            // else: a network-recovery recreate failed and the participant is dead. Skip —
            // the eventual reset rebuilds every endpoint (re-deferring this one, which then
            // re-registers and is resolved again from the cache).
        }
        catch (const std::exception &exception)
        {
            log_error() << "deferred match-publisher subscriber build failed on domain " << domain_id << ": "
                        << exception.what()
                        << "; this subscriber stays inactive (get_num_matched_publishers returns 0) until the "
                           "next discovered writer on its topic retries the build (or a network-recovery reset "
                           "rebuilds it)";
        }
        catch (...)
        {
            log_error() << "deferred match-publisher subscriber build threw a non-std::exception on domain "
                        << domain_id;
        }
    }

    void domain_participant::trigger_network_recovery_reset()
    {
        if (!recovery_enabled)
        {
            return;
        }

        // Declared OUTSIDE the locked block below so this snapshot is destroyed only AFTER
        // both lifecycle locks are released. A match-mode subscriber whose last strong
        // reference is this snapshot would otherwise be destroyed while we still hold
        // registration_mutex, and ~subscriber_handle joins an in-flight deferred build that
        // itself needs registration_mutex → self-deadlock on this thread. Letting the
        // snapshot destruct after the block closes keeps that teardown lock-free. (Early
        // returns inside the block are fine: the block-scope locks unwind before this
        // function-scope vector does.)
        std::vector<std::shared_ptr<detail::resettable_endpoint>> live_endpoints;
        {
            // Serialize against new endpoint registrations for the whole reset. With
            // this in place, no endpoint can be registered while we are tearing down
            // and rebuilding the participant — so we never have to worry about
            // "endpoint exists but state is built against the old participant we just
            // destroyed".
            const std::lock_guard<std::mutex> reg_lock{registration_mutex};

            // Phase 1: snapshot the registered endpoints.
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

            // Phase 2: detach the endpoint listener callbacks WITHOUT the lifecycle lock
            // held. A user callback that re-enters provizio APIs (e.g. publish() on a
            // sibling publisher) can still acquire reset_mutex shared at this point and run
            // to completion, so the drain wait does not deadlock. After this phase returns,
            // no endpoint (publisher/subscriber) listener callback owned by us is in flight.
            // The participant-level discovery listener is deliberately NOT drained here: it
            // stays installed on the old participant and is quiesced instead by the
            // delete_participant() call in Phase 4, which stops and joins Fast-DDS's
            // discovery thread. That is sufficient because the only owner state its
            // callbacks mutate (discovered_writer_reliability) is guarded by deferred_mutex.
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

            // Drop the match-publisher discovery cache: it tracked writers seen by the
            // now-destroyed participant. The new participant re-discovers currently-present
            // writers and repopulates it from scratch, so clearing here prevents the per-reliability
            // writer counts from inflating across resets (the old participant's writers never fire
            // REMOVED_WRITER) — which would otherwise pin the adopted reliability and grow the
            // registry without bound. No participant exists in this window, so no discovery event can
            // race the clear; deferred_mutex is a leaf taken after the registration + reset locks
            // we already hold.
            {
                const std::lock_guard<std::mutex> deferred_lock{deferred_mutex};
                discovered_writer_reliability.clear();
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
                            << "; endpoints left in torn-down state, will be retried by the "
                               "network-recovery safety-net check";
                // Bump generation anyway so any in-flight teardown observing the
                // mismatch skips its Fast-DDS-side deletes (the contained entities
                // were freed by delete_contained_entities above).
                generation.fetch_add(1, std::memory_order_acq_rel);
                // Flag for the coordinator: this participant is inert until a later
                // attempt succeeds, and no further network event is guaranteed to come.
                recovery_retry_needed.store(true, std::memory_order_release);
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
            bool any_endpoint_failed = false;
            for (const auto &endpoint : live_endpoints)
            {
                try
                {
                    endpoint->on_new_participant_started(*participant);
                }
                catch (const std::exception &exception)
                {
                    any_endpoint_failed = true;
                    log_error() << "endpoint rebuild failed on domain " << domain_id << ": " << exception.what()
                                << "; this endpoint stays inactive (publish/take return failure) until the "
                                   "network-recovery safety-net check retries the reset";
                }
            }

            // The participant itself is healthy either way; an endpoint that failed to
            // come back still needs another attempt, which the coordinator's safety-net
            // tick will make (a later network event alone cannot be relied on).
            recovery_retry_needed.store(any_endpoint_failed, std::memory_order_release);
        }  // close the registration_mutex / reset_mutex scope BEFORE live_endpoints destructs
    }

    std::shared_ptr<domain_participant> make_domain_participant(
        const DomainId_t domain_id, const network_recovery_mode recovery_mode,
        domain_participant::on_discovered_endpoint_callback initial_discovery_callback,
        const endpoint_kind initial_discovery_kinds, const transport_mode transport)
    {
        // A plain shared_ptr: the participant owns no threads of its own (each match-mode
        // subscriber owns its deferred-build std::async and joins it in its own destructor),
        // so there is no worker to join on teardown and thus no need for the previous
        // two-control-block ownership trick. The participant outlives its endpoints because
        // every publisher / subscriber holds a strong shared_ptr to it, so ~domain_participant
        // runs only after they are all gone.
        auto participant = std::make_shared<domain_participant>(
            domain_id, recovery_mode, std::move(initial_discovery_callback), initial_discovery_kinds, transport);
        if (resolve_network_recovery_enabled(recovery_mode))
        {
            detail::network_recovery_coordinator::instance().register_participant(participant);
        }
        return participant;
    }
}  // namespace provizio::dds
