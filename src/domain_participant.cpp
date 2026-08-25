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
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

#include "detail/env_utils.h"
#include "provizio/dds/detail/network_recovery_coordinator.h"
#include "provizio/dds/detail/resettable_endpoint.h"
#include "provizio/dds/detail/shm_cleanup.h"
#include "provizio/dds/detail/vpn_interfaces.h"
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
#include <fastdds/rtps/transport/SocketTransportDescriptor.hpp>
#include <fastdds/rtps/transport/network/AllowedNetworkInterface.hpp>
#include <fastdds/rtps/transport/network/NetmaskFilterKind.hpp>
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
            const std::uint32_t parsed = detail::parse_positive_u32(env);
            if (parsed != 0U)
            {
                return parsed;
            }
            log_warning() << "ignoring invalid " << name << "='" << detail::sanitise_env_value_for_log(env)
                          << "'; using default " << fallback;
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
                const std::uint32_t parsed = detail::parse_positive_u32(env);
                if (parsed != 0U)
                {
                    return parsed;
                }
                log_error() << "ignoring invalid PROVIZIO_DDS_UDP_SOCKET_BUFFER_SIZE='"
                            << detail::sanitise_env_value_for_log(env) << "'; using default "
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
                const std::uint32_t parsed = detail::parse_positive_u32(env);
                if (parsed != 0U)
                {
                    return parsed;
                }
                log_error() << "ignoring invalid PROVIZIO_DDS_SHM_SEGMENT_SIZE='"
                            << detail::sanitise_env_value_for_log(env) << "'";
            }
            return unset_shm_segment_size;
        }

        // Send-side cap for RTPS message size (bytes), overridable via
        // PROVIZIO_DDS_MAX_MESSAGE_SIZE. Maps to the Fast-DDS "fastdds.max_message_size"
        // participant property, which caps OUTPUT messages only — reception of larger
        // datagrams from differently-configured peers is unaffected, so mixed deployments
        // (including default-configured ROS 2 peers) stay compatible.
        //
        // The default (1400) keeps every UDP datagram within a single ~1500-byte-MTU link
        // frame: a sample above the cap is sent as individually-NACKable RTPS DATA_FRAG
        // submessages instead of one large UDP datagram that the IP layer splits into many
        // link frames. With IP fragmentation, losing ANY link frame loses the whole datagram
        // (first-try survival (1-p)^N at per-frame loss p), every reliable retransmission
        // re-runs that same gauntlet, and each incomplete datagram parks ~its full size in
        // the receiver kernel's reassembly cache (net.ipv4.ipfrag_high_thresh, 4 MB stock)
        // for net.ipv4.ipfrag_time (30 s stock) — under sustained loss the cache fills within
        // seconds and refuses all new reassemblies host-wide, gating every fragmented topic
        // in ~30 s windows while single-frame topics keep flowing. Measured at 10% injected
        // frame loss with 28 KB point clouds: ~36% delivered in 30 s blackout cycles with the
        // cap left at Fast-DDS's 65500 maximum vs 99%+ with 1400, p90 latency 160 ms.
        //
        // The cost is per-datagram sender/receiver overhead on very large samples: a multi-MB
        // sample pays roughly 10x the CPU at 1400 vs 65500 (a 6 MB frame becomes ~4500
        // datagrams instead of ~100). Hosts publishing such bulk streams (raw camera frames)
        // over clean links can raise the cap up to 65500 (Fast-DDS's own maximum and
        // historical single-datagram behaviour) to trade loss resilience back for CPU.
        constexpr std::uint32_t default_max_message_size = 1400;

        // Floor for the cap: Fast-DDS requires a complete participant-discovery (PDP)
        // announcement to fit in one RTPS message, but validates that only for transport
        // descriptors (~568 bytes with default locator counts), NOT for the
        // fastdds.max_message_size property — a property below it breaks discovery with
        // no diagnostic at all. 576 (the classic IPv4 minimum-reassembly datagram size)
        // sits safely above that requirement. Note the PDP announcement grows with the
        // participant's own locator count (~56 bytes per additional addressed interface),
        // and the PDP writer is best-effort stateless — no fragmentation — so ANY value of
        // this cap has an interface-count ceiling: the 1400 default accommodates roughly
        // 15 addressed interfaces beyond the baseline; hosts with more must raise the cap.
        constexpr std::uint32_t minimum_max_message_size = 576;

        // Fast-DDS's own hard ceiling for an RTPS message (its s_maximumMessageSize):
        // the property is min()'d against the transports' maximum anyway, so anything
        // above it can only be a typo — clamp with a warning rather than let Fast-DDS
        // silently ignore the excess.
        constexpr std::uint32_t maximum_max_message_size = 65500;

        // Accepted values below this leave thin headroom for the participant's own
        // discovery announcement (~612 bytes baseline with FASTDDS_STATISTICS compiled
        // in, as it is here, +~56 bytes per additional addressed interface) — legal,
        // but close enough to the silent-discovery-breakage line to deserve a warning.
        constexpr std::uint32_t thin_discovery_headroom_max_message_size = 1000;

        std::uint32_t resolve_max_message_size()
        {
            const char *const env = std::getenv("PROVIZIO_DDS_MAX_MESSAGE_SIZE");  // NOLINT: getenv required
            if (env != nullptr && *env != '\0')
            {
                const std::uint32_t parsed = detail::parse_positive_u32(env);
                if (parsed >= minimum_max_message_size)
                {
                    if (parsed > maximum_max_message_size)
                    {
                        log_warning() << "PROVIZIO_DDS_MAX_MESSAGE_SIZE=" << detail::sanitise_env_value_for_log(env)
                                      << " exceeds Fast-DDS's maximum RTPS message size; clamping to "
                                      << maximum_max_message_size;
                        return maximum_max_message_size;
                    }
                    if (parsed < thin_discovery_headroom_max_message_size)
                    {
                        log_warning() << "PROVIZIO_DDS_MAX_MESSAGE_SIZE=" << parsed
                                      << " leaves thin headroom for the participant discovery announcement "
                                         "(~612 bytes baseline, ~56 more per additional addressed interface); "
                                         "discovery breaks with no diagnostic if it no longer fits";
                    }
                    return parsed;
                }
                log_error() << "ignoring invalid PROVIZIO_DDS_MAX_MESSAGE_SIZE='"
                            << detail::sanitise_env_value_for_log(env)
                            << "' (must be an integer >= " << minimum_max_message_size
                            << ", so a discovery announcement fits in one message); using default "
                            << default_max_message_size;
            }
            return default_max_message_size;
        }

        // Whether this library configures shared memory on this platform. Windows and macOS
        // are excluded because Fast-DDS's bundled Boost.Interprocess leaks segments and named
        // semaphores there (see the transport setup in the constructor). _WIN32 rather than
        // _MSC_VER so a MinGW build lands on the same side as an MSVC one.
        constexpr bool platform_allows_shared_memory =
#if defined(_WIN32) || defined(__APPLE__)
            false;
#else
            true;
#endif

        // The filesystem backing shared memory on Linux, the only platform where its free
        // space is worth probing (see shared_memory_space).
        constexpr const char *const linux_shm_directory = "/dev/shm";

        constexpr std::uintmax_t bytes_per_mebibyte = std::uintmax_t{1024} * std::uintmax_t{1024};

        // Free space below which /dev/shm cannot fit a handful more segments of the size a
        // participant allocates at the default transport configuration (~33.5 MiB each).
        constexpr std::uintmax_t shm_low_water_bytes = std::uintmax_t{256} * bytes_per_mebibyte;

        // /dev/shm capacity and free space, or an empty capacity when it cannot be determined
        // (or on platforms that do not use shared memory).
        std::filesystem::space_info shared_memory_space()
        {
#ifdef __linux__
            std::error_code error;
            const auto space = std::filesystem::space(linux_shm_directory, error);
            if (!error)
            {
                return space;
            }
#endif  // __linux__
            return {};
        }

        // Reclaim the shared-memory files of participants that died without cleaning up, and
        // report a /dev/shm that still cannot fit a handful more segments afterwards.
        // Exhaustion otherwise shows up only as an obscure "Failed to create segment" from
        // Fast-DDS followed by a silent, host-wide fallback to UDP.
        void manage_shared_memory_space()
        {
            // Before anything else, and exactly once per process: bury the corpses. A service
            // that exits and is restarted in a loop then reclaims its own predecessor's
            // segment on every incarnation, so the steady state is one dead generation rather
            // than unbounded growth — whatever else on the host is or isn't fixed.
            detail::cleanup_shared_memory_once();

            auto space = shared_memory_space();
            if (space.capacity == 0 || space.available >= shm_low_water_bytes)
            {
                return;
            }

            // Still short. Sweep again (rate-limited, and suppressed entirely right after the
            // once-per-process sweep above) so a long-running process — a GUI backend, a
            // recorder — heals the host over its lifetime instead of only complaining about
            // it, and re-check before saying anything.
            if (detail::cleanup_shared_memory_if_due())
            {
                space = shared_memory_space();
                if (space.capacity == 0 || space.available >= shm_low_water_bytes)
                {
                    return;
                }
            }

            static std::once_flag warned;
            std::call_once(warned, [&space] {
                log_warning() << linux_shm_directory << " has only " << (space.available / bytes_per_mebibyte)
                              << " MiB free of " << (space.capacity / bytes_per_mebibyte)
                              << " MiB; shared-memory transport registration can fail and fall back to UDP. "
                              << "What is left is in use, or held by files this process may not remove -- "
                              << "segments of participants that did not exit cleanly are automatically reclaimed "
                              << "(see PROVIZIO_DDS_SHM_CLEANUP)";
            });
        }

        // Name of the env variable Fast-DDS reads to locate its XML profiles file.
        // Hardcoded here because Fast-DDS no longer exposes this name as a public
        // constant. Keep provizio_dds.py's _DomainParticipant.xml_profiles_env_variable
        // in sync with this constant.
        constexpr const char *const default_fastdds_env_variable = "FASTDDS_DEFAULT_PROFILES_FILE";

        // Runs an action when the enclosing scope ends, however it ends. Used to emit the
        // stashed VPN report from a function-scope object, so that every way out of a
        // construction or a reset reports it — the early returns and the exception paths as
        // well as the normal one, which an explicit call at the end would be alone in
        // covering. The action must not throw (see flush_pending_vpn_blocklist_log, which is
        // noexcept for this reason).
        template <typename action_type> class scope_exit final
        {
          public:
            // Conditionally noexcept: moving the action into the member is only nothrow if
            // the action type's move is, and promising more than that would turn a throwing
            // move into a terminate. The destructor stays implicitly noexcept by contrast,
            // deliberately — an action that throws there is a contract violation, which is
            // why the only action used here is noexcept itself.
            explicit scope_exit(action_type action) noexcept(std::is_nothrow_move_constructible_v<action_type>)
                : action{std::move(action)}
            {
            }
            scope_exit(const scope_exit &) = delete;
            scope_exit(scope_exit &&) = delete;
            scope_exit &operator=(const scope_exit &) = delete;
            scope_exit &operator=(scope_exit &&) = delete;
            ~scope_exit()
            {
                action();
            }

          private:
            action_type action;
        };

        // The second XML profiles file Fast-DDS loads: DomainParticipantFactory::load_profiles
        // picks this name up from the working directory, unless SKIP_DEFAULT_XML_FILE is "1"
        // (XMLProfileManager::loadDefaultXMLFile). Neither name is exposed as a public
        // constant. Keep provizio_dds.py's _DEFAULT_FASTDDS_PROFILES_FILE_NAME and
        // _SKIP_DEFAULT_XML_FILE_ENV in sync with these.
        constexpr const char *const default_fastdds_profiles_file_name = "DEFAULT_FASTDDS_PROFILES.xml";
        constexpr const char *const skip_default_fastdds_profiles_env_variable = "SKIP_DEFAULT_XML_FILE";

        // Whether a participant's transports come from XML the caller wrote rather than
        // from this library: either the profile named by FASTDDS_DEFAULT_PROFILES_FILE, or
        // a DEFAULT_FASTDDS_PROFILES.xml in the working directory that load_profiles()
        // auto-loads. Both end up in the QoS returned by get_default_participant_qos, and
        // setup_transports appends to userTransports rather than replacing it — so the
        // caller's own transport descriptors, blocklist included, are sitting in
        // cached_qos alongside ours and must not be rewritten.
        bool probe_working_directory_xml_profile()
        {
            const auto *const raw_skip =
                std::getenv(skip_default_fastdds_profiles_env_variable);  // NOLINT: getenv required
            const std::string_view skip{raw_skip != nullptr ? raw_skip : ""};
            if (!skip.empty() && skip.front() == '1')
            {
                return false;  // Fast-DDS will not load it, so there is nothing of the caller's here.
            }

            std::error_code error;  // Non-throwing overload: an unreadable working directory is just "no profile".
            return std::filesystem::is_regular_file(std::filesystem::path{default_fastdds_profiles_file_name}, error);
        }

        // Only the WORKING-DIRECTORY probe is resolved once per process, and only it needs to
        // be: load_profiles() reads that directory on its FIRST call, so re-deriving the answer
        // later would let a chdir between creations flip it in both directions — a file created
        // afterwards would stop the exclusion over a profile Fast-DDS never read, and a file
        // deleted (or a chdir away) would make a later rebuild rewrite the caller's own
        // descriptors.
        //
        // Whether THIS participant was configured from the profile named by
        // FASTDDS_DEFAULT_PROFILES_FILE is a per-participant fact, decided in the constructor,
        // so it is combined on every call instead of being folded into the cache. Folding it in
        // made the first participant's answer bind every later one, in both directions: a
        // participant created with the variable set skips load_profiles() and takes the caller's
        // XML, yet would still have been told the transports were ours to rewrite — the one
        // thing the surrounding code promises never to do — while one created after a first
        // participant that HAD the variable set would skip the exclusion for good.
        //
        // Mirrors _xml_profile_owns_transports in python/provizio_dds.py.
        bool transports_come_from_user_xml(const bool used_env_xml_profile)
        {
            static const bool working_directory_profile = probe_working_directory_xml_profile();
            return used_env_xml_profile || working_directory_profile;
        }
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
        // Resolved once, here rather than per creation: load_profiles() below reads the
        // working directory exactly once per process, so this is the moment the answer is
        // decided for every participant and every rebuild that follows.
        xml_profile_owns_transports = transports_come_from_user_xml(used_xml_profile);
        // Read here, with the XML answer, and never again: see env_owns_transports. This is
        // the same moment the transport block below decides whether to call setup_transports
        // at all, so the two answers cannot disagree.
        // NOLINTNEXTLINE(concurrency-mt-unsafe): startup-only probe, as everywhere else here
        env_owns_transports = std::getenv("FASTDDS_BUILTIN_TRANSPORTS") != nullptr;

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

            // Output-message-size cap (see resolve_max_message_size). Applied outside the
            // FASTDDS_BUILTIN_TRANSPORTS guard below: it caps the RTPS output path whichever
            // transports carry it, so a user-selected transport set still honours it.
            cached_qos.properties().properties().emplace_back("fastdds.max_message_size",
                                                              std::to_string(resolve_max_message_size()));

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
                const bool use_shared_memory = platform_allows_shared_memory && (transport != transport_mode::udp_only);
                eprosima::fastdds::rtps::BuiltinTransportsOptions transport_options;
                const std::uint32_t socket_buffer_size = resolve_udp_socket_buffer_size();
                transport_options.sockets_buffer_size = socket_buffer_size;
                // setup_transports APPENDS, so anything already in user_transports came from
                // the caller (an XML profile they loaded themselves is handed back by
                // get_default_participant_qos as the very same shared_ptr the factory holds).
                // Remembering where their descriptors end is what lets
                // refresh_vpn_interface_blocklist confine itself to ours.
                const std::size_t descriptors_before_setup = cached_qos.transport().user_transports.size();
                cached_qos.setup_transports(use_shared_memory ? eprosima::fastdds::rtps::BuiltinTransports::DEFAULT
                                                              : eprosima::fastdds::rtps::BuiltinTransports::UDPv4,
                                            transport_options);
                warn_if_socket_buffers_are_capped(socket_buffer_size);

                const auto &configured_transports = cached_qos.transport().user_transports;
                for (std::size_t index = descriptors_before_setup; index < configured_transports.size(); ++index)
                {
                    own_transport_descriptors.insert(configured_transports[index]);
                }

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
                }
            }
        }

        // Shared-memory housekeeping, deliberately outside every branch above: whether the
        // transports came from this library, from FASTDDS_BUILTIN_TRANSPORTS or from an XML
        // profile, the same host-wide directory backs them and the same dead participants
        // litter it. Deliberately NOT gated on platform_allows_shared_memory either: this
        // library does not choose shared memory on macOS, but a participant configured through
        // one of those two routes still can, and it would then leak with nothing to reclaim it.
        // Where shared memory truly cannot be in play the sweep is a no-op that costs one
        // failed directory open (or, on Windows, nothing at all). Only an explicit UDP-only
        // request, which nothing can override, skips it.
        if (transport != transport_mode::udp_only)
        {
            manage_shared_memory_space();
        }

        // A guard rather than a call after create_fastdds_participant(): the throw below,
        // and anything else that leaves this constructor early, must still report what the
        // transports excluded. A host whose participant fails to come up on a tunnel-carrying
        // network is exactly where an operator needs that context, and the reset path already
        // uses the same guard for the same reason. Constructed here, after the last of the
        // locks this function never takes -- flush_pending_vpn_blocklist_log must run with
        // none held, which is trivially true throughout the constructor.
        const scope_exit flush_vpn_blocklist_log{[this] { flush_pending_vpn_blocklist_log(); }};

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

    void domain_participant::refresh_vpn_interface_blocklist()
    {
        // Nothing of ours to configure when the caller took over transport
        // configuration — XML they wrote themselves (see transports_come_from_user_xml:
        // their descriptors, blocklist and all, are the ones sitting in cached_qos) or
        // FASTDDS_BUILTIN_TRANSPORTS (Fast-DDS builds the descriptors itself, and they
        // never reach cached_qos). Checked before enumerating interfaces so neither the
        // syscall nor the log line below happens where it would mean nothing.
        if (xml_profile_owns_transports || env_owns_transports)
        {
            // The snapshot has to learn this: DDS is about to bind and announce the tunnel,
            // so change detection must keep watching it (see
            // report_vpn_exclusion_not_applied). Reported whether or not a tunnel is up
            // right now -- one can come up later, and by then this participant's transports
            // are long since fixed.
            detail::report_vpn_exclusion_not_applied();

            // Said once, and only when it makes a difference: silence here is what a
            // vehicle paying for duplicated traffic over a metered tunnel would otherwise
            // get, with the exclusion documented as on by default and nothing anywhere
            // saying why it did not apply. Note what counts as the caller owning the
            // transports: ANY DEFAULT_FASTDDS_PROFILES.xml in the working directory does,
            // even one that configures no transports at all, which is the case an operator
            // is least likely to guess.
            //
            // The enumeration this costs is the one the applied path performs anyway, and it
            // is asked for only while the line is still unsaid — so a tunnel that comes up
            // later is still reported, and a host that never has one pays a single
            // enumeration per participant creation.
            if (!vpn_exclusion_skip_reported && !detail::vpn_interface_blocklist_entries().empty())
            {
                vpn_exclusion_skip_reported = true;
                stash_vpn_blocklist_log(
                    log_level::info,
                    "not excluding VPN / tunnel interface(s) on domain " + std::to_string(domain_id) +
                        ": the transports are yours (an XML profile, or FASTDDS_BUILTIN_TRANSPORTS), so this "
                        "library configures none of them and cannot exclude an interface from them. DDS will "
                        "bind and announce on the tunnel; exclude it in your own transport descriptors' "
                        "interface_blocklist if that is not what you want.");
            }
            return;
        }

        // A blocklist is only usable together with netmask filtering (see the long comment
        // where netmask_filter is set below), and Fast-DDS refuses to register a socket
        // transport whose descriptor asks for ON while the PARTICIPANT-level filter says
        // OFF: RTPSParticipantImpl validates the two against each other
        // (network::netmask_filter::validate_and_transform) and drops the whole transport
        // when they disagree. Applying the exclusion to a participant configured that way
        // would therefore take UDP away entirely — no cross-host communication at all,
        // reported only as a Fast-DDS error line — which is far worse than the duplicate
        // traffic the exclusion saves. So the exclusion steps aside, and says so.
        //
        // Only OFF collides: a participant-level AUTO adopts the descriptor's value, and
        // ON matches it.
        if (cached_qos.transport().netmask_filter == eprosima::fastdds::rtps::NetmaskFilterKind::OFF)
        {
            // Same reason as the branch above: the tunnel stays bound, so it stays watched.
            detail::report_vpn_exclusion_not_applied();

            // Stashed, not logged, for the reason given further down: this runs with the
            // lifecycle locks held during a rebuild. Reported once per participant, on the
            // first refresh that would have excluded something, so a rebuilt participant on
            // a tunnel-carrying host does not repeat it every time.
            if (!vpn_exclusion_skip_reported && !detail::vpn_interface_blocklist_entries().empty())
            {
                vpn_exclusion_skip_reported = true;
                stash_vpn_blocklist_log(
                    log_level::info, "not excluding VPN / tunnel interface(s) on domain " + std::to_string(domain_id) +
                                         ": this participant's netmask filter is OFF, and excluding an "
                                         "interface requires it (Fast-DDS would refuse to register the UDP "
                                         "transport, leaving no cross-host communication at all). Set the "
                                         "participant's netmask filter to AUTO or ON, or exclude the "
                                         "interface in your own transport descriptors.");
            }
            return;
        }

        // Shared memory has no interfaces, so only the socket transports (UDPv4/UDPv6)
        // carry a blocklist — dynamic_pointer_cast is exactly the filter for that
        // (SharedMemTransportDescriptor derives from PortBasedTransportDescriptor, not
        // from SocketTransportDescriptor).
        // A failed read is not "this host has no tunnels", and the difference matters here
        // in a way it does not at first creation: the loops below ERASE what a previous
        // refresh applied before re-adding from the fresh read, so taking a failure for an
        // empty host would unblock a tunnel that is still up -- on exactly the rebuild a
        // network change triggered, which is when the enumeration is most likely to fail
        // and the exclusion most needed. Leaving every list exactly as it stands is the one
        // answer that cannot be wrong: the tunnel that was blocked stays blocked, and the
        // next creation or rebuild reads the host again.
        bool enumeration_failed = false;
        const auto blocked = detail::vpn_interface_blocklist_entries(&enumeration_failed);
        if (enumeration_failed)
        {
            return;
        }

        // A name in PROVIZIO_DDS_ALLOW_VPN_INTERFACES that re-admitted nothing. Reported
        // here because the enumeration just above is what classified this host's
        // interfaces, so this is the first moment the answer is known -- and reported only
        // where it is actionable: with something excluded, a name that matched none of it
        // is a typo or the wrong identity for the platform ("tailscale" for a device called
        // "tailscale0"), and the deployment that needed DDS over the tunnel is not getting
        // it. With nothing excluded there is nothing the name could have re-admitted, which
        // is the ordinary case for one setting given fleet-wide to hosts whose tunnels
        // differ. take_ hands the names back once per process; the branches above return
        // before it precisely because the exclusion is not applying there either way.
        if (!blocked.empty())
        {
            const auto unmatched = detail::take_unmatched_vpn_allow_override_names();
            if (!unmatched.empty())
            {
                std::string message{detail::allow_vpn_interfaces_env};
                message += " names ";
                bool first = true;
                for (const auto &name : unmatched)
                {
                    message += first ? "" : ", ";
                    // Sanitised because it is an environment value, i.e. arbitrary text
                    // that reaches a log line -- see the same treatment of the OS-supplied
                    // interface names below.
                    message += detail::sanitise_env_value_for_log(name);
                    first = false;
                }
                message += ", which matched no VPN / tunnel interface on this host, so DDS "
                           "still does not use ";
                message += unmatched.size() == 1 ? "it" : "them";
                message += ". Name the interface exactly as this host reports it (on "
                           "Windows, the adapter's friendly name or its description), or "
                           "set the variable to 1 to carry DDS over every tunnel.";
                stash_vpn_blocklist_log(log_level::warning, std::move(message));
            }
        }
        // What the transports are left with, enumerated the way UDPv4Transport enumerates
        // it, and how many of those are real interfaces rather than loopback. Both feed the
        // per-interface netmask filter decided below; read once here rather than per
        // descriptor, since every descriptor of ours faces the same host.
        bool allowed_enumeration_failed = false;
        const auto allowed_interfaces = detail::vpn_allowed_interfaces(blocked, &allowed_enumeration_failed);
        if (allowed_enumeration_failed)
        {
            // The same rule as the failed blocklist read above, and for a sharper reason.
            // These are two independent enumerations, so the second can fail where the first
            // succeeded -- and going on with a non-empty blocklist and no allowlist is the
            // one outcome this whole section may not produce: any non-empty interface list
            // puts UDPv4 into whitelist mode, giving every remaining interface its own
            // sender socket, and without the per-interface netmask filters the allowlist
            // carries, each of them sends its own copy of every unicast datagram. That moves
            // the duplication this feature removes from the tunnel onto the LAN. Leaving
            // every list exactly as it stands cannot be wrong: the next creation or rebuild
            // reads the host again.
            return;
        }
        const auto real_interface_count = static_cast<std::size_t>(
            std::count_if(allowed_interfaces.begin(), allowed_interfaces.end(),
                          [](const detail::allowed_interface &entry) { return !entry.is_loopback; }));
        std::unordered_set<std::string> applied;
        std::unordered_set<std::string> applied_allowlist;
        for (const auto &descriptor : cached_qos.transport().user_transports)
        {
            // Ours only. A descriptor the caller configured is shared process-wide and
            // must be left exactly as they set it — including its own blocklist and its
            // own netmask_filter, neither of which this library may second-guess. Where
            // such a descriptor carries the traffic, the exclusion simply does not reach
            // it: the caller took over transport configuration, so the tunnel is theirs
            // to exclude (PROVIZIO_DDS_ALLOW_VPN_INTERFACES is not what they need — an
            // interface_blocklist entry in their own XML is).
            if (own_transport_descriptors.count(descriptor) == 0)
            {
                continue;
            }

            const auto socket_descriptor =
                std::dynamic_pointer_cast<eprosima::fastdds::rtps::SocketTransportDescriptor>(descriptor);
            if (!socket_descriptor)
            {
                continue;
            }

            // Only OUR entries are replaced, never the whole list. Two reasons the
            // descriptors cannot simply be cleared: they outlive every rebuild, so an
            // append would accumulate tunnels that have since gone away (and, worse, keep
            // blocking an address the OS has meanwhile handed to a real interface) — but a
            // clear would also discard entries that are not ours. A caller who loaded XML
            // through DomainParticipantFactory::load_XML_profiles_file/_string themselves
            // has their blocklist sitting in this very descriptor (get_default_participant_qos
            // hands out the same shared_ptr), and erasing it would re-expose DDS on an
            // interface they deliberately excluded.
            auto &blocklist = socket_descriptor->interface_blocklist;
            blocklist.erase(std::remove_if(blocklist.begin(), blocklist.end(),
                                           [this](const eprosima::fastdds::rtps::BlockedNetworkInterface &entry) {
                                               return last_applied_vpn_blocklist.count(entry.name) != 0;
                                           }),
                            blocklist.end());
            for (const auto &address : blocked)
            {
                if (std::none_of(blocklist.begin(), blocklist.end(),
                                 [&address](const eprosima::fastdds::rtps::BlockedNetworkInterface &entry) {
                                     return entry.name == address;
                                 }))
                {
                    blocklist.emplace_back(address);
                    // Only what this call appended is ours to remove next time. An entry
                    // that was already there is somebody else's — recording it here would
                    // have the erase above delete it once the tunnel goes away.
                    applied.insert(address);
                }
            }

            // Netmask filtering, decided PER INTERFACE, because the two interfaces this
            // leaves behind need opposite answers.
            //
            // Any non-empty interface list puts UDPv4 into whitelist mode: UDPv4Transport's
            // constructor fills interface_whitelist_ with every interface the blocklist did
            // NOT name, and open_output_channel then replaces the single any-address output
            // socket with one UDPSenderResource(..., only_multicast_purpose=false,
            // whitelisted=true) per allowed interface. RTPSParticipantImpl::sendSync calls
            // send() on every one of them, and UDPTransportInterface::send transmits
            // whenever `whitelisted` is set unless eProsimaUDPSocket::should_filter says
            // otherwise -- which it only does with netmask_filter ON.
            //
            // LOOPBACK is in that set and must stay in it: it is how same-host participants
            // reach each other wherever shared memory is off. But it cannot carry a datagram
            // to another host at all, so every attempt it makes at one is a failed sendto
            // and an EPROSIMA_LOG_WARNING -- per datagram, per remote locator. ON is what
            // stops that: 127.0.0.1/8 matches no LAN destination, so the send returns before
            // the syscall. Nothing is lost, because nothing off this host was ever reachable
            // through it.
            //
            // A REAL interface is the opposite case. ON there means a peer outside its
            // subnet -- reachable through a gateway, discovered perfectly well over
            // multicast -- silently receives nothing, so it is worth switching on only where
            // it buys the thing this feature exists for: with two or more real interfaces
            // left, each sends its own copy of every unicast datagram, and filtering is what
            // collapses those copies back to the one socket whose subnet holds the
            // destination. With a single real interface there is nothing to collapse.
            //
            // Fast-DDS applies an allowlist entry's own filter (AllowedNetworkInterface
            // carries one) after validate_and_transform against the descriptor's, which
            // stays AUTO here so that both ON and AUTO entries are accepted.
            const bool filter_real_interfaces = real_interface_count > 1;
            auto &allowlist = socket_descriptor->interface_allowlist;
            allowlist.erase(std::remove_if(allowlist.begin(), allowlist.end(),
                                           [this](const eprosima::fastdds::rtps::AllowedNetworkInterface &entry) {
                                               return last_applied_vpn_allowlist.count(entry.name) != 0;
                                           }),
                            allowlist.end());
            // Written only alongside a blocklist, and only when the host reported something
            // usable. Two degenerate cases it stays out of: with nothing to block there is
            // nothing to filter either, and an allowlist ALONE would put UDPv4 into whitelist
            // mode -- costing the any-address socket for no reason on every host without a
            // tunnel. With nothing enumerated, an allowlist that matches no interface leaves
            // interface_whitelist_ empty, and UDPv4Transport then fills it with a sentinel
            // address that allows nothing through at all (UDPv4Transport.cpp:218-221).
            if (!blocked.empty() && !allowed_interfaces.empty())
            {
                for (const auto &interface : allowed_interfaces)
                {
                    const auto filter = (interface.is_loopback || filter_real_interfaces)
                                            ? eprosima::fastdds::rtps::NetmaskFilterKind::ON
                                            : eprosima::fastdds::rtps::NetmaskFilterKind::AUTO;
                    if (std::none_of(allowlist.begin(), allowlist.end(),
                                     [&interface](const eprosima::fastdds::rtps::AllowedNetworkInterface &entry) {
                                         return entry.name == interface.address;
                                     }))
                    {
                        allowlist.emplace_back(interface.address, filter);
                        applied_allowlist.insert(interface.address);
                    }
                }
            }
            // AUTO at descriptor level, so each entry's own filter is what decides. Assigned
            // rather than left alone because a rebuild may find a different interface set
            // than the one that set it last time.
            socket_descriptor->netmask_filter = eprosima::fastdds::rtps::NetmaskFilterKind::AUTO;
            // ASSIGNED, not latched. It describes what THIS refresh configured, and the
            // report below reads it as such: a host that loses one of two tunnels goes back
            // to a single real interface and to AUTO, and a latched flag would keep telling
            // an operator diagnosing a reachability complaint that a filter is on when it
            // is not. Every descriptor of ours faces the same host, so the last write and
            // the first agree.
            netmask_filtering_applied = filter_real_interfaces;
        }

        // Sorted so the same host always logs the same line (the entries live in an
        // unordered_set) — and so it reads the same as the Python side's. Parentheses,
        // not braces: brace-init would look for an initializer_list<std::string> and
        // reject the iterator pair.
        std::vector<std::string> sorted_entries(blocked.begin(), blocked.end());
        std::sort(sorted_entries.begin(), sorted_entries.end());

        // Worth reporting: an interface silently missing from the locator set is
        // otherwise indistinguishable from a peer that cannot be reached at all, and
        // this is the only place that decision is made. Reported once per distinct set,
        // not once per creation, so a process with several participants — or one rebuilt
        // by network recovery — does not repeat an identical line indefinitely. A
        // genuine change, a tunnel appearing or going away, still reports itself.
        //
        // Stashed rather than logged here: this function runs from
        // create_fastdds_participant, which the recovery path calls while holding
        // registration_mutex AND reset_mutex exclusively, and log_info invokes the
        // caller's log callback — a callback that re-enters any provizio API taking the
        // lifecycle lock shared (publish, get_guid, make_publisher) would deadlock on a
        // shared_mutex that is not recursive. The two callers flush it once their locks
        // are released; the coordinator and listener_drain route their own logs out of
        // locked regions for exactly this reason.
        if (!sorted_entries.empty() && sorted_entries != last_logged_vpn_blocklist)
        {
            std::string message = "excluding VPN / tunnel interface(s) from the DDS transports on domain ";
            message += std::to_string(domain_id);
            message += ": ";
            bool first = true;
            for (const auto &entry : sorted_entries)
            {
                message += first ? "" : ", ";
                // Sanitised for the same reason a rejected environment value is: this text
                // comes from the OS, and on Windows an adapter's friendly name is
                // administrator-settable and far less constrained than a POSIX device name,
                // which the kernel already refuses to give whitespace or control characters.
                message += detail::sanitise_env_value_for_log(entry);
                first = false;
            }
            message += " (set ";
            message += detail::allow_vpn_interfaces_env;
            message += " to carry DDS over them)";
            if (netmask_filtering_applied)
            {
                // Said in the same breath as the exclusion, because it is the exclusion
                // that brings it: with more than one interface left, netmask filtering is
                // what stops every unicast datagram going out all of them, and it also
                // means a peer outside every local subnet no longer receives unicast. A
                // peer that quietly stops being reachable is otherwise indistinguishable
                // from one that went away.
                message += ". More than one interface is left, so netmask filtering is on "
                           "for them and DDS peers outside their subnets are no longer "
                           "reachable by unicast; on a routed network, exclude the tunnel in "
                           "your own transport descriptors instead";
            }
            stash_vpn_blocklist_log(log_level::info, std::move(message));
        }
        last_logged_vpn_blocklist = std::move(sorted_entries);
        // What the next refresh is allowed to remove — see the erase above. This is what
        // was actually appended, NOT everything currently blocked.
        last_applied_vpn_blocklist = std::move(applied);
        last_applied_vpn_allowlist = std::move(applied_allowlist);
    }

    void domain_participant::stash_vpn_blocklist_log(const log_level level, std::string message)
    {
        const std::lock_guard<std::mutex> lock{pending_vpn_blocklist_log_mutex};
        pending_vpn_blocklist_logs.emplace_back(level, std::move(message));
    }

    void domain_participant::flush_pending_vpn_blocklist_log() noexcept
    {
        // Only ever called with no participant lock held — see the rationale on
        // refresh_vpn_interface_blocklist for why the message cannot be emitted where it
        // is produced.
        //
        // Taken out under the mutex and logged without it. Both halves matter: the string
        // is the one piece of this participant's VPN state that is touched OUTSIDE the
        // reset lock -- the scope-exit guards in the constructor and in the reset run after
        // their lock scopes have unwound, deliberately, because the log callback may
        // re-enter this participant -- so a user's trigger_network_recovery_reset() racing
        // the coordinator's would otherwise have two threads move from and clear the same
        // std::string. And holding the mutex across the callback would put a lock this
        // participant owns underneath arbitrary user code.
        std::vector<std::pair<log_level, std::string>> messages;
        {
            const std::lock_guard<std::mutex> lock{pending_vpn_blocklist_log_mutex};
            if (pending_vpn_blocklist_logs.empty())
            {
                return;
            }
            messages = std::move(pending_vpn_blocklist_logs);
            pending_vpn_blocklist_logs.clear();
        }
        // A scope-exit guard calls this during a reset, so a throwing log callback would
        // otherwise leave a destructor throwing. Swallowed for the same reason the
        // listener drain swallows its stall warning: a diagnostic must not decide whether
        // the operation it describes succeeds. Per message, so one callback that throws
        // does not swallow the reports after it.
        for (const auto &message : messages)
        {
            try
            {
                detail::log_stream{message.first} << message.second;
            }
            catch (...)  // NOLINT: a diagnostic must never propagate out of a destructor
            {
            }
        }
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

        // Re-evaluated per creation, so a rebuild after a network change blocks the
        // tunnels that exist at that moment rather than the ones that existed at
        // construction time.
        refresh_vpn_interface_blocklist();

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
            throw std::runtime_error{"domain_participant: cannot register endpoint -- the Fast-DDS participant "
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

        // Declared here, outside the locked block, for the same reason live_endpoints is:
        // the report must go out only once both lifecycle locks have unwound, because the
        // log callback may re-enter this participant (see refresh_vpn_interface_blocklist).
        // A guard rather than a call at the end, so that the early return on a failed
        // re-creation — the path where an operator most needs to know which interfaces the
        // transports left out — reports it too, as does an exception from any phase.
        // Declared AFTER live_endpoints so it runs BEFORE that snapshot is released,
        // keeping the order the explicit call had.
        const scope_exit flush_vpn_blocklist_log{[this] { flush_pending_vpn_blocklist_log(); }};
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

        // The stashed VPN report goes out from flush_vpn_blocklist_log above, on every
        // exit path, once both locks are released.
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
