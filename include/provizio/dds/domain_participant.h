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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/topic.h"

namespace provizio::dds::detail
{
    class resettable_endpoint;
    class discovery_listener_impl;
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

    /**
     * @brief Which remote endpoint kinds to deliver to a
     * @c domain_participant::on_discovered_endpoint callback. The values are
     * bit flags, so e.g. @c data_writer @c | @c data_reader subscribes to both.
     */
    enum class endpoint_kind : std::uint8_t
    {
        none = 0,                ///< Empty mask. Lets callers express "no kinds" without a
                                 ///< @c static_cast and names the result of an @c & of disjoint
                                 ///< kinds; @c any() returns @c false for it.
        data_writer = 1U << 0U,  ///< Remote DataWriters (someone publishing on a topic).
        data_reader = 1U << 1U,  ///< Remote DataReaders (someone subscribing to a topic).
    };

    constexpr endpoint_kind operator|(endpoint_kind a, endpoint_kind b) noexcept
    {
        return static_cast<endpoint_kind>(static_cast<std::uint8_t>(a) | static_cast<std::uint8_t>(b));
    }

    constexpr endpoint_kind operator&(endpoint_kind a, endpoint_kind b) noexcept
    {
        return static_cast<endpoint_kind>(static_cast<std::uint8_t>(a) & static_cast<std::uint8_t>(b));
    }

    /// @brief Returns @c true if @p k has any bit set.
    constexpr bool any(endpoint_kind k) noexcept
    {
        return static_cast<std::uint8_t>(k) != 0U;
    }

    /**
     * @brief Network transport selection for a @c domain_participant.
     *
     * Controls only whether shared memory is used in addition to UDP. The enlarged UDP
     * socket buffers that make reliable delivery of large samples (camera frames, point
     * clouds) work are applied in BOTH modes — they matter for any cross-host UDP path
     * regardless of shared memory.
     */
    enum class transport_mode : std::uint8_t
    {
        /// Platform default: SHM + UDPv4 on Linux; UDPv4-only on Windows/macOS (where
        /// shared memory is disabled to dodge a Boost.Interprocess cleanup bug).
        automatic = 0,
        /// UDPv4-only (shared memory disabled) on every platform. For participants that
        /// bridge mismatched Fast-DDS major versions (e.g. a recorder relaying 2.x
        /// publishers), where cross-major shared-memory negotiation degrades large-sample
        /// throughput.
        udp_only = 1,
    };

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
         * @brief Callback signature for @c on_discovered_endpoint.
         *
         * @param participant The @c domain_participant that observed the event.
         *                    Passed by reference (stable for the participant's
         *                    lifetime; survives network-recovery resets). Lets
         *                    the callback call @c is_known_type / @c known_types
         *                    without having to capture the participant by some
         *                    external mechanism — which would otherwise race
         *                    the very first discovery event that fires before
         *                    @c make_domain_participant has returned.
         * @param topic_name  The topic the remote endpoint operates on.
         * @param type_name   Wire-format type name (e.g. @c "std_msgs::msg::dds_::String_").
         * @param kind        Which kind of endpoint fired this event
         *                    (@c endpoint_kind::data_writer or @c endpoint_kind::data_reader).
         * @param discovered  @c true when the endpoint appears, @c false when it disappears.
         * @param reliability The discovered endpoint's reliability QoS — the
         *                    offered reliability for a remote DataWriter
         *                    (@c endpoint_kind::data_writer) or the requested
         *                    reliability for a remote DataReader
         *                    (@c endpoint_kind::data_reader). A recording bridge
         *                    can use it to create a matching reader/writer per
         *                    topic.
         * @param durability  The discovered endpoint's durability QoS — the
         *                    offered durability for a remote DataWriter or the
         *                    requested durability for a remote DataReader.
         */
        using on_discovered_endpoint_callback = std::function<void(
            domain_participant &participant, const std::string &topic_name, const std::string &type_name,
            endpoint_kind kind, bool discovered, eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability,
            eprosima::fastdds::dds::DurabilityQosPolicyKind durability)>;

        /**
         * @brief Construct a new domain participant object
         *
         * @param domain_id DDS domain_id, 0 by default
         * @param recovery_mode Whether this participant participates in network
         * auto-recovery (defaults to env-var-controlled; recovery is on by default
         * but can be globally disabled with @c PROVIZIO_DDS_NETWORK_RECOVERY=off).
         * @param initial_discovery_callback Optional callback to install BEFORE the
         * underlying Fast-DDS participant starts discovery. Equivalent to calling
         * @c on_discovered_endpoint immediately after construction, but without the
         * tiny race window — endpoints already on the network when this participant
         * joins are seen by the callback from the very first SEDP exchange. The
         * callback additionally receives the discovered endpoint's reliability and
         * durability QoS (offered for a DataWriter, requested for a DataReader) so a
         * recording bridge can create a reader/writer with matching QoS per topic —
         * see @c on_discovered_endpoint_callback. Defaults to empty: the discovery
         * listener is still installed (it drives the match-publisher subscriber
         * default), but its user-callback slot stays empty, so a participant with no
         * callback pays only a cheap enum-read + map-lookup per discovery event.
         * @param initial_discovery_kinds Which endpoint kinds the initial callback
         * fires for. Ignored when @p initial_discovery_callback is empty. Defaults
         * to @c endpoint_kind::data_writer (the right choice for a recorder).
         * @param transport Network transport selection (see @c transport_mode). Defaults
         * to @c transport_mode::automatic (platform default: SHM+UDP on Linux, UDP-only on
         * Windows/macOS). Pass @c transport_mode::udp_only to disable shared memory on all
         * platforms (e.g. when bridging mismatched Fast-DDS major versions).
         */
        domain_participant(DomainId_t domain_id = 0,
                           network_recovery_mode recovery_mode = network_recovery_mode::env_var_controlled,
                           on_discovered_endpoint_callback initial_discovery_callback = {},
                           endpoint_kind initial_discovery_kinds = endpoint_kind::data_writer,
                           transport_mode transport = transport_mode::automatic);

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
         * @brief Whether the last @c trigger_network_recovery_reset left this
         * participant unusable and so needs to be retried.
         *
         * A reset can fail part-way: the old Fast-DDS participant is destroyed and its
         * endpoints torn down, and then either the replacement participant cannot be
         * created or an individual endpoint fails to rebuild. The participant is then
         * inert — publish / take report failure — and nothing in the event-driven path
         * would ever come back to it, because the network need not change again.
         * @c detail::network_recovery_coordinator polls this after every reset and
         * retries the affected participants on its periodic safety-net tick.
         *
         * @return true if a retry is outstanding; false when the participant is intact
         *         (including before any reset has ever run).
         */
        bool needs_network_recovery_retry() const noexcept
        {
            return recovery_retry_needed.load(std::memory_order_acquire);
        }

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

        /**
         * @brief Register a callback invoked when a remote DDS endpoint of one
         * of the requested @p kinds is discovered or removed in the DDS domain.
         * Used by callers (e.g. a recorder) that want to dynamically subscribe to
         * every topic carrying a known type, rather than hard-coding a topic list.
         *
         * @param callback  The handler (see @c on_discovered_endpoint_callback —
         *                  it receives the discovered endpoint's reliability and
         *                  durability QoS in addition to the topic/type/kind, so a
         *                  recording bridge can create a reader/writer with matching
         *                  QoS per topic). Pass an empty function (or default-
         *                  constructed @c std::function ) to clear the user
         *                  callback; the underlying Fast-DDS listener stays
         *                  installed to drive the internal match-publisher default.
         * @param kinds     Which endpoint kinds the callback should fire for.
         *                  Defaults to @c endpoint_kind::data_writer — the right
         *                  choice for a recorder (a topic is worth subscribing to
         *                  only if some remote participant is publishing on it).
         *                  Combine with @c | to receive both, e.g.
         *                  @c endpoint_kind::data_writer @c | @c endpoint_kind::data_reader .
         *
         * The Fast-DDS @c DomainParticipantListener is installed eagerly at
         * construction and stays attached for the participant's whole life — it
         * drives the match-publisher subscriber default (a discovered writer must
         * resolve any subscriber parked on its topic), so it can no longer be lazy.
         * This method only sets or clears the optional USER callback that rides on
         * that listener; calling it with an empty callback clears the user callback
         * but leaves the listener attached for internal use. Because the participant
         * holds that single @c DomainParticipantListener slot unconditionally, you
         * cannot attach a separate @c DomainParticipantListener of your own on the
         * same participant.
         *
         * The callback runs on the Fast-DDS discovery thread. It must not block,
         * must not call provizio APIs that would take the participant lifecycle
         * lock for any non-trivial duration, and must not create endpoints on
         * this participant. In particular it must not call @c on_discovered_endpoint
         * itself: re-registering from within the callback re-enters Fast-DDS'
         * @c set_listener on the discovery thread, which Fast-DDS forbids and
         * which can deadlock. Filter and enqueue; do the real work elsewhere.
         *
         * On a network-recovery reset the listener is re-installed on the freshly
         * created Fast-DDS participant automatically, so the callback continues
         * to fire across resets without re-registration.
         *
         * @note There is a small race window between participant creation and the
         * first call to this method: discoveries that arrive during that window
         * are not delivered to this callback (the listener is attached from
         * construction and Fast-DDS tracks the events, but the user callback
         * wasn't set yet). For a guarantee of "no events missed," pass the
         * callback to @c make_domain_participant / the constructor instead — that
         * sets the callback BEFORE Fast-DDS starts internal discovery, eliminating
         * the window entirely.
         */
        void on_discovered_endpoint(on_discovered_endpoint_callback callback,
                                    endpoint_kind kinds = endpoint_kind::data_writer);

        /**
         * @brief Returns @c true if a type with the given @p type_name has been
         * registered on this participant (via @c register_type or via a
         * Publisher / Subscriber that registered it on construction).
         *
         * Intended companion to @c on_discovered_endpoint: a recorder registers
         * the types it can deserialise, then uses @c is_known_type to filter the
         * stream of discovered (topic, type) pairs down to those it handles.
         */
        bool is_known_type(const std::string &type_name) const;

        /**
         * @brief Returns the names of all types currently registered on this
         * participant. See @c is_known_type for the intended usage.
         */
        std::vector<std::string> known_types() const;

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

        /// @internal Register a match-mode subscriber (created with
        /// @c match_publisher_reliability_qos) that has deferred building its
        /// DataReader. Called from @c subscriber_handle::build_state when the
        /// effective reliability is still the match sentinel. @p handle is a
        /// NON-owning back-reference (the participant must never own its endpoints) —
        /// the subscriber removes itself via @c deregister_deferred_subscriber in its
        /// destructor (and a successful deferred build deregisters it too), under
        /// @c deferred_mutex. The handle is always parked first; if a writer has already
        /// been discovered on @p topic_name its reliability is ALSO dispatched immediately
        /// via @c start_deferred_build (closing the writer-before-subscriber race). Parking
        /// even on the immediate-dispatch path is what lets a failed build be retried by the
        /// next discovered writer; otherwise the handle waits parked until
        /// @c resolve_deferred_for_writer fires for that topic.
        void register_deferred_subscriber(const std::string &topic_name, detail::resettable_endpoint *handle);
        /// @internal Remove a match-mode subscriber's back-reference from the deferred
        /// registry. Called from @c ~subscriber_handle under @c deferred_mutex — the
        /// same mutex @c resolve_deferred_for_writer holds to dispatch — so once this
        /// returns the discovery thread can no longer reach the dying handle. Safe to
        /// call for a handle that was already dispatched (and thus already removed).
        void deregister_deferred_subscriber(const std::string &topic_name, detail::resettable_endpoint *handle);
        /// @internal Called by the discovery listener when a remote DataWriter is
        /// discovered on @p topic_name. Caches the writer's reliability (match-first
        /// — the first discovered writer fixes it) and calls @c start_deferred_build on
        /// every parked match-mode subscriber on that topic so each spawns its reader
        /// build off this thread. Runs on the Fast-DDS discovery thread, so it only
        /// touches @c deferred_mutex-guarded state and launches the build threads — it
        /// never builds an endpoint inline (which would deadlock against a reset).
        void resolve_deferred_for_writer(const std::string &topic_name,
                                         eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability);
        /// @internal Called by the discovery listener when a remote DataWriter on
        /// @p topic_name with offered reliability @p reliability is REMOVED. Decrements the
        /// topic's per-reliability live-writer count. Once the last writer of any kind is gone
        /// it drops the cached entry so a match-mode subscriber created afterwards defers for a
        /// fresh writer instead of adopting a stale value (and so the cache can't grow without
        /// bound as topics churn). If writers remain but the adopted reliability no longer has a
        /// live writer, it re-derives the adopted value from a still-live kind — so a stale
        /// reliability can't outlive every writer that offered it (e.g. RELIABLE cached while
        /// only a BEST_EFFORT writer is left, which a RELIABLE reader would silently fail to
        /// match). Runs on the discovery thread; touches only @c deferred_mutex-guarded state.
        void on_writer_removed(const std::string &topic_name,
                               eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability);
        /// @internal Run a deferred reader build for @p handle with reliability
        /// @p resolved. Called on the @c std::async thread that @c start_deferred_build
        /// spawned. Takes @c registration_mutex then @c reset_mutex — the SAME order as
        /// @c register_endpoint (so no lock-order inversion), but @c reset_mutex is held
        /// EXCLUSIVELY (unlike register_endpoint's shared acquire) because the deferred
        /// build mutates already-user-visible reader state that @c get_guid /
        /// @c get_num_matched_publishers read under the shared lock. If the Fast-DDS
        /// participant is alive it invokes @c handle->build_deferred_locked. The thread
        /// hop is the whole point: building on the Fast-DDS discovery thread would
        /// deadlock against a concurrent reset (see the AB-BA note in subscriber.h).
        void run_deferred_build(detail::resettable_endpoint *handle,
                                eprosima::fastdds::dds::ReliabilityQosPolicyKind resolved);

      private:
        /// @brief Build (or rebuild) the underlying Fast-DDS DomainParticipant with
        /// the currently stored QoS configuration. Called from the constructor and
        /// from @c trigger_network_recovery_reset.
        eprosima::fastdds::dds::DomainParticipant *create_fastdds_participant();

        /// @brief Re-point the stored transports' interface blocklist at the host's
        /// current VPN / tunnel addresses, so this participant neither binds nor
        /// announces a locator on one (see detail/vpn_interfaces.h for why that
        /// matters), and turn their netmask filter on for as long as that blocklist is
        /// non-empty. Called immediately before every participant creation, not once in
        /// the constructor: a tunnel can come up, go down or change address while the
        /// process runs, and a rebuild triggered by a real network change must bind the
        /// interface set that exists at that moment. A no-op when the transports came
        /// from an XML profile or from FASTDDS_BUILTIN_TRANSPORTS — then no descriptor
        /// of ours exists to configure, and the caller has taken over transport
        /// configuration wholesale.
        void refresh_vpn_interface_blocklist();

        /// @brief Stash @p message, to be emitted at @p level by
        /// @c flush_pending_vpn_blocklist_log, under @c pending_vpn_blocklist_log_mutex.
        /// The one route for ANY diagnostic produced while a lifecycle lock is held -- the VPN
        /// exclusion reports it was named for, and every report of the reset path (a
        /// participant that could not be recreated, a type or an endpoint that could not be
        /// rebuilt): logging.h promises a log callback may publish onto a DDS topic, and
        /// publishing takes the lifecycle lock. One writer for every producer so the
        /// synchronisation cannot be forgotten at a new call site.
        void stash_vpn_blocklist_log(log_level level, std::string message);

        /// @brief Emit the reports stashed by @c stash_vpn_blocklist_log, if any. MUST be
        /// called with no participant lock held: it invokes the caller's log callback, which
        /// is free to re-enter provizio APIs that take the lifecycle lock. @c noexcept because
        /// it is called from a scope-exit guard during a reset: a throwing log callback must
        /// not turn a diagnostic into a terminate.
        void flush_pending_vpn_blocklist_log() noexcept;

        DomainId_t domain_id;
        bool recovery_enabled;
        /// Last VPN / tunnel blocklist this participant logged, so an unchanged set is
        /// reported once rather than on every creation and every recovery rebuild.
        /// Touched only from create_fastdds_participant (from the constructor, or from
        /// a reset that holds reset_mutex), so it needs no synchronisation of its own.
        std::vector<std::string> last_logged_vpn_blocklist;
        /// Reports produced under the participant's locks -- by refresh_vpn_interface_blocklist
        /// and by the reset path -- and emitted by flush_pending_vpn_blocklist_log once they
        /// are released.
        /// Guarded, unlike its siblings here: those are touched only under the reset lock,
        /// while these are deliberately read after that lock has been released, so two
        /// concurrent resets of the same participant (a caller's
        /// trigger_network_recovery_reset racing the coordinator's) would otherwise move
        /// from and clear the same strings.
        ///
        /// A list rather than a single message: one refresh can have two independent things
        /// to say -- which interfaces it excluded, and that a name given in
        /// PROVIZIO_DDS_ALLOW_VPN_INTERFACES re-admitted none of them -- and the second is
        /// worth saying precisely on the pass that produces the first.
        std::vector<std::pair<log_level, std::string>> pending_vpn_blocklist_logs;
        /// Guards @c pending_vpn_blocklist_logs, and nothing else. A leaf: never held while
        /// calling out to the log callback or to Fast-DDS.
        std::mutex pending_vpn_blocklist_log_mutex;
        /// Whether this participant has already reported that it is NOT excluding VPN /
        /// tunnel interfaces — because the caller owns the transport configuration, because
        /// a netmask filter of OFF rules the exclusion out, or because the host's interfaces
        /// could not be read at all (see refresh_vpn_interface_blocklist). Without the
        /// report, an operator watching a metered uplink pay for duplicated traffic has
        /// nothing anywhere to explain why the exclusion did not apply.
        ///
        /// One latch for all four lines because they say the same thing to the same reader,
        /// and a participant can only ever produce one of them: the first two conditions are
        /// fixed for the participant's life and return before any enumeration, so a
        /// participant that reaches an enumeration failure is one neither of them applies
        /// to. Once per participant rather than once per refresh, so a rebuilt participant
        /// on a host whose reading keeps failing does not repeat it on every network event.
        bool vpn_exclusion_skip_reported{false};
        /// Whether this participant switched netmask filtering ON to suppress the duplicate
        /// unicast sends whitelist mode would otherwise produce (see
        /// refresh_vpn_interface_blocklist). Kept because it changes what the participant
        /// can reach — a peer outside every local subnet, reachable only through a router,
        /// stops receiving unicast — which is worth saying once, in the same line that
        /// reports the exclusion.
        bool netmask_filtering_applied{false};
        /// The blocklist entries this participant actually appended last time, and
        /// therefore the only ones the next refresh may remove. Recorded per append rather
        /// than from the computed set: an entry already present is deliberately not
        /// re-added, and recording it as ours anyway would have the next refresh delete
        /// somebody else's entry. Same single-threaded access as
        /// last_logged_vpn_blocklist.
        std::unordered_set<std::string> last_applied_vpn_blocklist;
        /// Allowlist entries this participant appended, so a later refresh removes exactly
        /// those and leaves any the caller wrote. The allowlist exists to carry a netmask
        /// filter per interface -- ON for loopback, which can reach no other host, and only
        /// where it collapses real duplicates for the rest (see refresh_vpn_interface_blocklist).
        std::unordered_set<std::string> last_applied_vpn_allowlist;
        /// The transport descriptors this library created (whatever @c setup_transports
        /// appended in the constructor), and the only ones @c
        /// refresh_vpn_interface_blocklist may modify. @c setup_transports appends to
        /// @c user_transports rather than replacing it, so a descriptor a caller
        /// configured — including one from @c load_XML_profiles_file called directly,
        /// which no environment probe can detect — sits in the same vector, is shared
        /// process-wide through @c get_default_participant_qos, and must be left exactly
        /// as the caller set it. The descriptors themselves rather than their addresses,
        /// so "is this one ours?" is a question about the object and needs no argument
        /// about when the vector holding it is reassigned.
        std::unordered_set<std::shared_ptr<eprosima::fastdds::rtps::TransportDescriptorInterface>>
            own_transport_descriptors;
        eprosima::fastdds::dds::DomainParticipantQos cached_qos;
        bool used_xml_profile{false};
        /// Whether the transport descriptors in `cached_qos` are the caller's rather than
        /// this library's — true for FASTDDS_DEFAULT_PROFILES_FILE and for a
        /// DEFAULT_FASTDDS_PROFILES.xml that Fast-DDS auto-loads from the working
        /// directory. Wider than `used_xml_profile`, which gates only the QoS this
        /// library configures for itself; rewriting a blocklist the caller declared is a
        /// different kind of damage, so it gets its own answer.
        bool xml_profile_owns_transports{false};
        /// Whether Fast-DDS built the transports from @c FASTDDS_BUILTIN_TRANSPORTS instead
        /// of this library building them — read ONCE, in the constructor, at the same moment
        /// @c setup_transports either ran or did not.
        ///
        /// Answered once because the answer has to stay the one the descriptors were built
        /// under. Re-reading the variable per refresh would let a value set later in the
        /// process — a Python participant in a mixed application does exactly that, via
        /// @c os.environ.setdefault — turn a participant whose descriptors this library owns
        /// and has already written blocklist entries into into one that takes the
        /// caller-owns early return, so those entries would never be erased again. A tunnel
        /// address left blocked after the tunnel is gone drops genuine traffic if the OS
        /// hands that address to a real interface.
        bool env_owns_transports{false};

        // Shared by every operation that touches `participant` and by reset.
        std::shared_mutex reset_mutex;
        eprosima::fastdds::dds::DomainParticipant *participant{nullptr};
        std::atomic<std::uint64_t> generation{0};

        // See needs_network_recovery_retry(). Written only by
        // trigger_network_recovery_reset (under the exclusive lifecycle lock), read by
        // the coordinator's coalescer thread — hence atomic rather than plain bool.
        std::atomic<bool> recovery_retry_needed{false};

        // Mutual exclusion between `register_endpoint` and
        // `trigger_network_recovery_reset`. Serializes the two so a reset cannot
        // start while a registration is mid-flight (and vice-versa), eliminating
        // the otherwise-possible window where a freshly registered endpoint's
        // listener is firing callbacks that the reset path would have to drain
        // while holding the exclusive lifecycle lock (a guaranteed deadlock).
        std::mutex registration_mutex;

        // `mutable` so the const query methods @c is_known_type / @c known_types
        // can lock it without losing const-correctness on the participant.
        mutable std::mutex registered_types_mutex;
        std::unordered_map<std::string, TypeSupport> registered_types;
        std::shared_ptr<std::mutex> registered_topics_mutex;
        std::unordered_map<std::string, std::weak_ptr<topic>> registered_topics;

        // Endpoint registry for network-recovery reset.
        std::mutex endpoints_mutex;
        std::vector<std::weak_ptr<detail::resettable_endpoint>> endpoints;

        // Discovery listener, installed eagerly in the constructor and attached for the
        // participant's whole life: it drives the match-publisher subscriber default
        // (resolving parked subscribers when a writer is discovered), so it is no longer
        // optional. The user on_discovered_endpoint callback, when set, rides on this same
        // listener; with no user callback the listener still runs (a cheap enum-read +
        // map-lookup per discovery event). Survives @c trigger_network_recovery_reset:
        // re-attached to the new Fast-DDS participant inside @c create_fastdds_participant.
        std::mutex discovery_listener_mutex;
        std::unique_ptr<detail::discovery_listener_impl> discovery_listener;

        // ---------------------------------------------------------------------
        // Deferred-subscriber subsystem (match-publisher reliability default).
        //
        // A subscriber created with @c match_publisher_reliability_qos defers
        // building its DataReader until a matching writer is discovered, then
        // builds with the writer's reliability. The discovery listener runs on a
        // Fast-DDS thread that must NOT build endpoints (it would deadlock against
        // a concurrent reset — see the AB-BA note in subscriber.h), so each
        // subscriber hops its build onto its own std::async thread (the future is
        // owned by the subscriber; see subscriber_handle::build_future). The
        // participant only routes discovery to the parked subscribers — it never
        // owns them (the back-references below are raw pointers) and runs no worker
        // thread of its own.
        //
        // Lock order (must never invert): a build thread takes
        // @c registration_mutex → @c reset_mutex (exclusive) before building (in
        // @c run_deferred_build), the SAME order as @c register_endpoint (which takes
        // @c reset_mutex shared — the deferred build needs it exclusive to fence its
        // reader swap against concurrent @c get_guid / @c get_num_matched_publishers).
        // @c deferred_mutex is a leaf: @c register_deferred_subscriber /
        // @c deregister_deferred_subscriber / @c resolve_deferred_for_writer take it
        // alone (the std::async launch they may do inside it only spawns a thread —
        // it acquires no further lock here). @c reset_mutex / @c registration_mutex
        // are NEVER nested under @c deferred_mutex. build_state→register_deferred_subscriber
        // runs under reset_mutex (shared) and then takes deferred_mutex — order
        // reset→deferred — which does not conflict with the build thread's
        // registration→reset order.
        std::mutex deferred_mutex;
        // Match-mode subscribers parked per topic, waiting for the first writer.
        // NON-owning raw pointers: the participant must never own its endpoints. Each
        // subscriber removes itself here in its destructor (deregister_deferred_subscriber)
        // under deferred_mutex, so the discovery thread never reaches a freed handle.
        std::unordered_multimap<std::string, detail::resettable_endpoint *> deferred_subscribers;
        // Per-topic discovered-writer reliability state for match-publisher resolution.
        // Consolidates what used to be two parallel maps (first-seen reliability + a live-writer
        // count) into one entry per topic, and additionally tracks live writers PER reliability
        // kind so the adopted value can be re-derived if the writer that fixed it leaves while
        // other (differently-configured) writers remain.
        struct writer_reliability_state
        {
            // The reliability a match-mode subscriber on this topic adopts. "Match-first": the
            // first writer to appear on an otherwise-writer-less topic fixes it, and it stays put
            // while that writer (and others of its kind) remain. Re-derived to a still-live kind
            // by on_writer_removed only once no live writer offers the adopted value anymore.
            eprosima::fastdds::dds::ReliabilityQosPolicyKind adopted{};
            // Count of currently-discovered writers on the topic, keyed by offered reliability
            // kind (incremented on DISCOVERED_WRITER, decremented on REMOVED_WRITER). A kind's
            // entry is erased when its count hits zero, and the whole topic entry is dropped when
            // the map empties — so neither this map nor the registry grows without bound as
            // topics churn, and `empty()` cleanly means "no writer has ever / still on this topic".
            // unordered_map (not map): the re-derive in on_writer_removed picks an arbitrary
            // surviving kind, so the key order is irrelevant and the hash map is cheaper.
            std::unordered_map<eprosima::fastdds::dds::ReliabilityQosPolicyKind, std::size_t> live_counts;
        };
        // A subscriber that registers AFTER a writer was discovered resolves immediately from the
        // adopted value here instead of waiting for a writer event that already fired.
        std::unordered_map<std::string, writer_reliability_state> discovered_writer_reliability;
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
     * @param initial_discovery_callback Optional endpoint-discovery callback to
     * install BEFORE the underlying Fast-DDS participant starts discovery — see
     * the @c domain_participant constructor doc and @c on_discovered_endpoint_callback.
     * Pass this (rather than calling @c on_discovered_endpoint after construction)
     * when you must not miss any endpoints already on the network. The callback also
     * receives the discovered endpoint's reliability and durability QoS (offered for
     * a DataWriter, requested for a DataReader). Defaults to empty: the discovery
     * listener is still installed (it drives the match-publisher subscriber default),
     * but its user-callback slot stays empty (a cheap enum-read + map-lookup per event).
     * @param initial_discovery_kinds Which endpoint kinds the initial callback
     * fires for. Ignored when @p initial_discovery_callback is empty. Defaults
     * to @c endpoint_kind::data_writer.
     * @param transport Network transport selection (see @c transport_mode); defaults to
     * @c transport_mode::automatic. Pass @c transport_mode::udp_only to disable shared memory.
     * @return std::shared_ptr<domain_participant>
     * @see https://en.cppreference.com/w/cpp/memory/shared_ptr
     * @see
     * https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/domain/domainparticipant.html
     */
    PROVIZIO_DDS_API std::shared_ptr<domain_participant> make_domain_participant(
        DomainId_t domain_id = 0, network_recovery_mode recovery_mode = network_recovery_mode::env_var_controlled,
        domain_participant::on_discovered_endpoint_callback initial_discovery_callback = {},
        endpoint_kind initial_discovery_kinds = endpoint_kind::data_writer,
        transport_mode transport = transport_mode::automatic);
}  // namespace provizio::dds

#endif  // DDS_DOMAIN_PARTICIPANT
