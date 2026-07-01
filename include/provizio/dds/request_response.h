// Copyright 2025 Provizio Ltd.
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

#ifndef DDS_REQUEST_RESPONSE
#define DDS_REQUEST_RESPONSE

#include <assert.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_set>

#include "provizio/dds/common.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/request_response_details.h"
#include "provizio/dds/subscriber.h"

namespace provizio::dds
{
    /**
     * @file request_response.h
     * @brief Public request/response API built on top of Fast-DDS.
     *
     * The API enables defining services handling requests and returning responses,
     * and issuing requests from clients with correlation tracking and delivery to
     * matched responders only. Both synchronous and asynchronous request handlers
     * (returning std::future) are supported.
     */
    /// @brief Maximum time to keep a response for a client that is not matched yet.
    constexpr std::chrono::seconds max_time_to_keep_ready_responses{10};

    /**
     * @brief Service handling incoming requests and publishing responses.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    class service;

    /**
     * @brief Future-like object returned by request() to await a response.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type> class future_response;

    /**
     * @brief Persistent client for issuing many requests to a service, each returning a future_response.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type> class service_client;

    /**
     * @brief Default KEEP_LAST history depth for a service_client's request/response endpoints.
     *
     * Bounds how many requests can be concurrently in flight before the reliable endpoints apply
     * back-pressure (the request writer blocks briefly on publish once this many are unacknowledged). Matches
     * the service's default response history. Raise it via the service_client / make_service_client
     * @c endpoint_history_depth parameter when you need more than this many requests outstanding at once.
     */
    constexpr std::int32_t service_client_default_history_depth = 10;

    /**
     * @brief Creates a new service.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @tparam handle_request_function_type The type of the request handler function.
     * @param participant The DomainParticipant.
     * @param request_topic_name The name of the request topic.
     * @param response_topic_name The name of the response topic.
     * @param handle_request_function The function to handle requests.
     * @param max_history_depth The maximum number of requests to queue (request queue size). This is distinct from
     * endpoint_history_depth, which controls the KEEP_LAST history depth of the underlying request/response endpoints.
     * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS) applied to the
     * underlying request/response DataWriters/DataReaders; std::nullopt keeps the current default.
     * @param endpoint_history_depth KEEP_LAST history depth for the underlying request/response endpoints;
     * use_default_history_depth keeps the current default. Distinct from max_history_depth (request queue size).
     * @return std::shared_ptr<service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>> A
     * shared pointer to the created service.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    std::shared_ptr<service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>> make_service(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name, handle_request_function_type handle_request_function,
        std::int32_t max_history_depth = use_default_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
        std::int32_t endpoint_history_depth = use_default_history_depth);

    /**
     * @brief Creates a new service from a single name for both topics.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @tparam handle_request_function_type The type of the request handler function.
     * @param participant The DomainParticipant.
     * @param service_name The name of the service.
     * @param handle_request_function The function to handle requests.
     * @param max_history_depth The maximum number of requests to queue (request queue size). This is distinct from
     * endpoint_history_depth, which controls the KEEP_LAST history depth of the underlying request/response endpoints.
     * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS) applied to the
     * underlying request/response DataWriters/DataReaders; std::nullopt keeps the current default.
     * @param endpoint_history_depth KEEP_LAST history depth for the underlying request/response endpoints;
     * use_default_history_depth keeps the current default. Distinct from max_history_depth (request queue size).
     * @return std::shared_ptr<service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>> A
     * shared pointer to the created service.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    std::shared_ptr<service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>> make_service(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        handle_request_function_type handle_request_function,
        std::int32_t max_history_depth = use_default_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
        std::int32_t endpoint_history_depth = use_default_history_depth);

    /**
     * @brief Sends a request to a service.
     *
     * Creates a client that waits for graph discovery to become stable and, optionally, enforces both
     * a post-match settling delay and a finite total matching timeout before publishing the first request.
     * This is useful when multiple services may match on the same topic and you want to allow time for all of
     * them to appear, while still being able to fail fast if nothing matches.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @param participant The DomainParticipant.
     * @param request_topic_name The name of the request topic.
     * @param response_topic_name The name of the response topic.
     * @param request_data The request data.
     * @param stable_matches_period Additional settling window that the match count must remain stable for before
     * sending the first request (0 skips the extra wait). Useful when there are multiple sensors in the network, so we
     * make sure to match all of them prior to sending the request.
     * @param service_match_timeout Optional deadline for the whole readiness wait, which includes the post-match
     * settling window (see stable_matches_period). Use 0 to wait indefinitely; a finite value should comfortably
     * exceed stable_matches_period, or it can expire before settling completes even with a healthy service.
     * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS) applied to the
     * request DataWriter and response DataReader; std::nullopt keeps the current default.
     * @param endpoint_history_depth KEEP_LAST history depth for the request/response endpoints;
     * use_default_history_depth keeps the current default.
     * @return future_response<request_pub_sub_type, response_pub_sub_type> A future object to get the response.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type>
    future_response<request_pub_sub_type, response_pub_sub_type> request(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name, typename request_pub_sub_type::type &request_data,
        std::chrono::milliseconds stable_matches_period = detail::service_client_basic<
            request_pub_sub_type, response_pub_sub_type>::default_wait_for_stable_matches_period,
        std::chrono::milliseconds service_match_timeout = std::chrono::milliseconds{0},
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
        std::int32_t endpoint_history_depth = use_default_history_depth);

    /**
     * @brief Sends a request to a service.
     *
     * Creates a client that waits for graph discovery to become stable and, optionally, enforces both
     * a post-match settling delay and a finite total matching timeout before publishing the first request.
     * This is useful when multiple services may match on the same topic and you want to allow time for all of
     * them to appear, while still being able to fail fast if nothing matches.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @param participant The DomainParticipant.
     * @param service_name The name of the service.
     * @param request_data The request data.
     * @param stable_matches_period Additional settling window that the match count must remain stable for before
     * sending the first request (0 skips the extra wait). Useful when there are multiple sensors in the network, so we
     * make sure to match all of them prior to sending the request.
     * @param service_match_timeout Optional deadline for the whole readiness wait, which includes the post-match
     * settling window (see stable_matches_period). Use 0 to wait indefinitely; a finite value should comfortably
     * exceed stable_matches_period, or it can expire before settling completes even with a healthy service.
     * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS) applied to the
     * request DataWriter and response DataReader; std::nullopt keeps the current default.
     * @param endpoint_history_depth KEEP_LAST history depth for the request/response endpoints;
     * use_default_history_depth keeps the current default.
     * @return future_response<request_pub_sub_type, response_pub_sub_type> A future object to get the response.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type>
    future_response<request_pub_sub_type, response_pub_sub_type> request(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        typename request_pub_sub_type::type &request_data,
        std::chrono::milliseconds stable_matches_period = detail::service_client_basic<
            request_pub_sub_type, response_pub_sub_type>::default_wait_for_stable_matches_period,
        std::chrono::milliseconds service_match_timeout = std::chrono::milliseconds{0},
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
        std::int32_t endpoint_history_depth = use_default_history_depth);

    /**
     * @brief Creates a service_client from explicit request/response topic names.
     *
     * Unlike request(), the client is created up front and matches the service(s) over time. Call
     * wait_for_service() to block until ready, or just call request() (requests issued before matching
     * auto-defer and flush once matched). service_match_timeout defaults to 0 (wait indefinitely), suited
     * to clients created in advance.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @param participant The DomainParticipant.
     * @param request_topic_name The name of the request topic.
     * @param response_topic_name The name of the response topic.
     * @param stable_matches_period Settling window the match count must remain stable for before the
     * background readiness completes and deferred requests flush.
     * @param service_match_timeout Deadline for the background readiness wait; 0 (default) waits
     * indefinitely, suited to clients created in advance.
     * @param durability_kind Optional DDS durability kind applied to the request DataWriter and response
     * DataReader; std::nullopt keeps the current default.
     * @param endpoint_history_depth KEEP_LAST history depth for the request/response endpoints, bounding
     * how many requests may be concurrently in flight before the reliable writer applies back-pressure;
     * use_default_history_depth (the default) selects service_client_default_history_depth. Raise it for
     * higher concurrency.
     * @return std::shared_ptr<service_client<request_pub_sub_type, response_pub_sub_type>>
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type>
    std::shared_ptr<service_client<request_pub_sub_type, response_pub_sub_type>> make_service_client(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name,
        std::chrono::milliseconds stable_matches_period = detail::service_client_basic<
            request_pub_sub_type, response_pub_sub_type>::default_wait_for_stable_matches_period,
        std::chrono::milliseconds service_match_timeout = std::chrono::milliseconds{0},
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
        std::int32_t endpoint_history_depth = use_default_history_depth);

    /**
     * @brief Creates a service_client from a single service name (request/response topics inferred).
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @param participant The DomainParticipant.
     * @param service_name The name of the service (request/response topic names are inferred from it).
     * @param stable_matches_period Settling window the match count must remain stable for before the
     * background readiness completes and deferred requests flush.
     * @param service_match_timeout Deadline for the background readiness wait; 0 (default) waits
     * indefinitely, suited to clients created in advance.
     * @param durability_kind Optional DDS durability kind applied to the request DataWriter and response
     * DataReader; std::nullopt keeps the current default.
     * @param endpoint_history_depth KEEP_LAST history depth for the request/response endpoints, bounding
     * how many requests may be concurrently in flight before the reliable writer applies back-pressure;
     * use_default_history_depth (the default) selects service_client_default_history_depth. Raise it for
     * higher concurrency.
     * @return std::shared_ptr<service_client<request_pub_sub_type, response_pub_sub_type>>
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type>
    std::shared_ptr<service_client<request_pub_sub_type, response_pub_sub_type>> make_service_client(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        std::chrono::milliseconds stable_matches_period = detail::service_client_basic<
            request_pub_sub_type, response_pub_sub_type>::default_wait_for_stable_matches_period,
        std::chrono::milliseconds service_match_timeout = std::chrono::milliseconds{0},
        std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
        std::int32_t endpoint_history_depth = use_default_history_depth);

    /**
     * @brief A request/response service.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @tparam handle_request_function_type The type of the request handler function.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    class service
    {
      public:
        /**
         * @brief Construct a new service object
         *
         * @param participant The DomainParticipant
         * @param request_topic_name The name of the request topic
         * @param response_topic_name The name of the response topic
         * @param handle_request_function The function to handle requests
         * @param max_history_depth The maximum number of requests to queue (request queue size). This is distinct from
         * endpoint_history_depth, which controls the KEEP_LAST history depth of the underlying request/response
         * endpoints.
         * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS) applied to the
         * underlying request/response DataWriters/DataReaders; std::nullopt keeps the current default.
         * @param endpoint_history_depth KEEP_LAST history depth for the underlying request/response endpoints;
         * use_default_history_depth keeps the current default. Distinct from max_history_depth (request queue size).
         */
        service(std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
                const std::string &response_topic_name, handle_request_function_type handle_request_function,
                std::int32_t max_history_depth = use_default_history_depth,
                std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
                std::int32_t endpoint_history_depth = use_default_history_depth);

        /**
         * @brief Construct a new service object from a single name for both topics
         *
         * @param participant The DomainParticipant
         * @param service_name The name of the service
         * @param handle_request_function The function to handle requests
         * @param max_history_depth The maximum number of requests to queue (request queue size). This is distinct from
         * endpoint_history_depth, which controls the KEEP_LAST history depth of the underlying request/response
         * endpoints.
         * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS) applied to the
         * underlying request/response DataWriters/DataReaders; std::nullopt keeps the current default.
         * @param endpoint_history_depth KEEP_LAST history depth for the underlying request/response endpoints;
         * use_default_history_depth keeps the current default. Distinct from max_history_depth (request queue size).
         */
        service(std::shared_ptr<domain_participant> participant, const std::string &service_name,
                handle_request_function_type handle_request_function,
                std::int32_t max_history_depth = use_default_history_depth,
                std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
                std::int32_t endpoint_history_depth = use_default_history_depth);
        ~service();

      private:
        void on_data(const typename request_pub_sub_type::type &data, const SampleInfo &info);
        void on_matched(bool matched, const guid &subscriber_guid);
        void on_response(typename response_pub_sub_type::type data,
                         const eprosima::fastdds::rtps::SampleIdentity &identity);
        bool is_subscriber_matched_mutex_prelocked(const guid &subscriber_guid);
        void dispatch_responses();

        struct ready_response
        {
            ready_response(typename response_pub_sub_type::type data,
                           const eprosima::fastdds::rtps::SampleIdentity &identity,
                           const std::chrono::system_clock::time_point &time_ready);

            typename response_pub_sub_type::type data;
            eprosima::fastdds::rtps::SampleIdentity identity;
            std::chrono::system_clock::time_point time_ready;
        };

        std::mutex mutex;
        bool stop{false};
        std::vector<ready_response> ready_responses;
        std::unordered_set<guid, detail::guid_hash> matched_subscriptions;
        std::condition_variable cv;
        std::shared_ptr<data_publisher<response_pub_sub_type>> publisher;
        detail::request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>
            request_handler;
        std::shared_ptr<subscriber_handle<request_pub_sub_type>> subscriber;
        std::thread dispatch_responses_thread;
    };

    /**
     * @brief A future-like object to get the response of a request.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type> class future_response
    {
      public:
        using response_type = typename response_pub_sub_type::type;

        /**
         * @brief Construct a new future response object.
         */
        future_response(
            std::shared_ptr<detail::service_client_basic<request_pub_sub_type, response_pub_sub_type>> basic_client,
            std::shared_ptr<detail::response_data<response_type>> data);

        /**
         * @brief Waits for the response to be ready.
         *
         * @tparam rep
         * @tparam period
         * @param timeout_duration The maximum time to wait.
         * @return std::future_status The status of the response.
         */
        template <class rep, class period>
        std::future_status wait_for(const std::chrono::duration<rep, period> &timeout_duration) const;

        /**
         * @brief Waits for the response to be ready until a specific time point.
         *
         * @tparam clock
         * @tparam duration
         * @param timeout_time The time to wait until.
         * @return std::future_status The status of the response.
         */
        template <typename clock, typename duration>
        std::future_status wait_until(const std::chrono::time_point<clock, duration> &timeout_time) const;

        /**
         * @brief Gets the response.
         *
         * @return const response_type& The response.
         */
        const response_type &get() const;

      private:
        std::shared_ptr<detail::service_client_basic<request_pub_sub_type, response_pub_sub_type>> basic_client;
        std::shared_ptr<detail::response_data<response_type>> data;
    };

    /**
     * @brief Persistent request/response client.
     *
     * Created in advance; matches the service(s) in the background and supports many concurrent in-flight
     * requests, each returning its own future_response. Thread-safe. Mirrors the ROS 2 service-client
     * workflow. A future_response keeps the underlying client (and its response DataReader) alive, so
     * in-flight requests still resolve even if the service_client is destroyed first.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type> class service_client
    {
      public:
        using request_type = typename request_pub_sub_type::type;
        using response_type = typename response_pub_sub_type::type;

        /**
         * @brief Constructs a client from explicit request/response topic names.
         * @param participant The DomainParticipant.
         * @param request_topic_name The name of the request topic.
         * @param response_topic_name The name of the response topic.
         * @param stable_matches_period Settling window the match count must remain stable for before the
         * background readiness completes and deferred requests flush.
         * @param service_match_timeout Deadline for the background readiness wait; 0 (default) waits
         * indefinitely, suited to clients created in advance.
         * @param durability_kind Optional DDS durability kind applied to the request DataWriter and response
         * DataReader; std::nullopt keeps the current default.
         * @param endpoint_history_depth KEEP_LAST history depth for the request/response endpoints, bounding
         * how many requests may be concurrently in flight before the reliable writer applies back-pressure;
         * use_default_history_depth (the default) selects service_client_default_history_depth. Raise it for
         * higher concurrency.
         */
        service_client(std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
                       const std::string &response_topic_name,
                       std::chrono::milliseconds stable_matches_period = detail::service_client_basic<
                           request_pub_sub_type, response_pub_sub_type>::default_wait_for_stable_matches_period,
                       std::chrono::milliseconds service_match_timeout = std::chrono::milliseconds{0},
                       std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
                       std::int32_t endpoint_history_depth = use_default_history_depth);

        /**
         * @brief Constructs a client from a single service name (request/response topics inferred).
         * @param participant The DomainParticipant.
         * @param service_name The name of the service (request/response topic names are inferred from it).
         * @param stable_matches_period Settling window the match count must remain stable for before the
         * background readiness completes and deferred requests flush.
         * @param service_match_timeout Deadline for the background readiness wait; 0 (default) waits
         * indefinitely, suited to clients created in advance.
         * @param durability_kind Optional DDS durability kind applied to the request DataWriter and response
         * DataReader; std::nullopt keeps the current default.
         * @param endpoint_history_depth KEEP_LAST history depth for the request/response endpoints, bounding
         * how many requests may be concurrently in flight before the reliable writer applies back-pressure;
         * use_default_history_depth (the default) selects service_client_default_history_depth. Raise it for
         * higher concurrency.
         */
        service_client(std::shared_ptr<domain_participant> participant, const std::string &service_name,
                       std::chrono::milliseconds stable_matches_period = detail::service_client_basic<
                           request_pub_sub_type, response_pub_sub_type>::default_wait_for_stable_matches_period,
                       std::chrono::milliseconds service_match_timeout = std::chrono::milliseconds{0},
                       std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
                       std::int32_t endpoint_history_depth = use_default_history_depth);

        /**
         * @brief Issues a request and returns a future_response. Sends immediately if matched, else
         * auto-defers until matched. Thread-safe; many requests may be outstanding at once.
         * @param request_data The request payload to send.
         * @return A future_response that resolves with the response correlated to this request.
         */
        future_response<request_pub_sub_type, response_pub_sub_type> request(request_type &request_data);

        /**
         * @brief Blocks until the service(s) are ready (matched and stable for stable_matches_period),
         * bounded by timeout (0 = wait indefinitely). Returns true once ready, false on timeout/shutdown.
         * Includes the settling window so all services sharing the topic (multiple sensors, by frame_id) are
         * discovered before sending.
         *
         * Stability is bounded, not strict: to avoid blocking indefinitely under sustained churn, settling is
         * hard-capped at 4x stable_matches_period from the first match, after which the call returns ready even
         * if the match count is still changing. Do not treat a true return as a guarantee that no further
         * services will appear.
         * @param timeout Maximum time to wait for readiness; 0 (default) waits indefinitely.
         * @param stable_matches_period Settling window the match count must remain stable for before the
         * service is considered ready (subject to the 4x hard cap above).
         * @return true once the service(s) are ready, false on timeout or shutdown.
         */
        bool wait_for_service(std::chrono::milliseconds timeout = std::chrono::milliseconds{0},
                              std::chrono::milliseconds stable_matches_period = detail::service_client_basic<
                                  request_pub_sub_type, response_pub_sub_type>::default_wait_for_stable_matches_period);

      private:
        std::shared_ptr<detail::response_router<response_type>> router;
        std::shared_ptr<detail::service_client_basic<request_pub_sub_type, response_pub_sub_type>> basic_client;
    };

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>::service(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name, handle_request_function_type handle_request_function,
        const std::int32_t max_history_depth, std::optional<DurabilityQosPolicyKind> durability_kind,
        const std::int32_t endpoint_history_depth)
        : publisher(make_publisher<response_pub_sub_type>(
              participant, response_topic_name,
              [this](data_publisher<response_pub_sub_type> &, const bool matched, const guid &subscriber_guid) {
                  on_matched(matched, subscriber_guid);
              },
              RELIABLE_RELIABILITY_QOS,
              // Response-writer history is DECOUPLED from max_history_depth (the request-queue size):
              // sizing the request queue — in particular minimal_request_queue (0) → KEEP_LAST(1) —
              // must not shrink the response writer and risk a reliable depth-1 writer blocking or
              // dropping responses under multiple concurrent clients. When the caller doesn't set an
              // explicit endpoint_history_depth, the response writer uses the standard default queue
              // depth (to_max_queue_size(use_default_history_depth)), independent of max_history_depth;
              // endpoint_history_depth overrides it.
              (endpoint_history_depth == use_default_history_depth)
                  ? static_cast<std::int32_t>(detail::to_max_queue_size(use_default_history_depth))
                  : endpoint_history_depth,
              // Default the response writer to TRANSIENT_LOCAL durability, overridable via durability_kind.
              // NOTE: the DEFAULT client response reader is VOLATILE (see request_response_details.h —
              // nullopt → Fast-DDS default), so against a default client this offered durability is inert:
              // a VOLATILE reader never requests the writer's historical samples. It is not a data-loss
              // path — in the settled flow the response reader is matched before the response is sent — it
              // only benefits a client that explicitly opts its response reader into TRANSIENT_LOCAL.
              durability_kind ? durability_kind
                              : std::optional<DurabilityQosPolicyKind>{TRANSIENT_LOCAL_DURABILITY_QOS})),
          request_handler(
              std::move(handle_request_function),
              [this](typename response_pub_sub_type::type data,
                     const eprosima::fastdds::rtps::SampleIdentity &identity) { on_response(data, identity); },
              detail::to_max_queue_size(max_history_depth)),
          subscriber(make_subscriber<request_pub_sub_type>(
              participant, request_topic_name,
              [this](const typename request_pub_sub_type::type &data, const SampleInfo &info) { on_data(data, info); },
              RELIABLE_RELIABILITY_QOS, endpoint_history_depth, durability_kind)),
          dispatch_responses_thread(&service::dispatch_responses, this)
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>::service(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        handle_request_function_type handle_request_function, const std::int32_t max_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind, const std::int32_t endpoint_history_depth)
        : service(std::move(participant), detail::request_prefix + service_name + detail::request_suffix,
                  detail::response_prefix + service_name + detail::response_suffix, std::move(handle_request_function),
                  max_history_depth, durability_kind, endpoint_history_depth)
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>::~service()
    {
        // Shutdown order:
        //  1. Stop incoming requests by destroying the subscriber. This must
        //     come before request_handler.stop_and_join(): otherwise DDS
        //     internal threads may still fire on_data into the request_handler
        //     while it is being joined (use-after-free SEGFAULTs were seen on
        //     Windows without this ordering).
        //  2. Drain the request_handler queue. Each pending request runs the
        //     user handler and then service::on_response, which either
        //     publishes the response (if matched_subscriptions has already
        //     learned about the requesting client) or stashes it in
        //     ready_responses.
        //  3. Give the dispatch thread a bounded window to publish anything
        //     left in ready_responses. The response-publisher's
        //     SubscriptionMatchedStatus callback is asynchronous w.r.t. the
        //     request-direction match, so service::on_response can run before
        //     on_matched has populated matched_subscriptions for that client —
        //     in which case the response is stashed and only goes out once
        //     dispatch_responses observes the match. Without this drain, a
        //     fast-shutdown service (e.g. a single-iteration test that
        //     returns shortly after its handler completes) destroys the
        //     publisher with the response still stashed, dropping it and
        //     producing a client-side timeout. The drain is bounded so a
        //     non-arriving match (peer gone away) does not hang the
        //     destructor.
        //  4. Stop the dispatch thread.
        //  5. Destroy the publisher last (no thread reaches it after this).
        subscriber.reset();
        request_handler.stop_and_join();
        {
            // 2 s, not max_time_to_keep_ready_responses (10 s): this is the
            // destructor latency the typical caller is willing to pay for the
            // last response to leave. A response whose match never arrives is
            // a peer that already went away; waiting the full 10 s only to
            // drop it is worse than dropping it 2 s in. The remove_timed_out
            // path in dispatch_responses still collects it on the normal
            // 1 s cleanup tick if dispatch keeps running, and on destructor
            // exit it goes away with the publisher regardless.
            constexpr auto ready_responses_drain_timeout = std::chrono::seconds{2};
            std::unique_lock<std::mutex> lock{mutex};
            cv.wait_for(lock, ready_responses_drain_timeout, [&]() { return ready_responses.empty(); });
        }
        {
            const std::lock_guard<std::mutex> lock{mutex};
            stop = true;
        }
        cv.notify_all();
        dispatch_responses_thread.join();
        publisher.reset();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    void service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>::on_data(
        const typename request_pub_sub_type::type &data, const SampleInfo &info)
    {
        if (info.instance_state == InstanceStateKind::ALIVE_INSTANCE_STATE)
        {
            auto identity = info.sample_identity;
            if (info.related_sample_identity.writer_guid() != guid::unknown())
            {
                identity.writer_guid() = info.related_sample_identity.writer_guid();
            }
            request_handler.handle_request(data, identity);
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    void service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>::on_matched(
        const bool matched, const guid &subscriber_guid)
    {
        {
            const std::lock_guard<std::mutex> lock{mutex};
            if (matched)
            {
                matched_subscriptions.insert(subscriber_guid);
            }
            else
            {
                matched_subscriptions.erase(subscriber_guid);
            }
        }
        if (matched)
        {
            cv.notify_all();
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    void service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>::on_response(
        typename response_pub_sub_type::type data, const eprosima::fastdds::rtps::SampleIdentity &identity)
    {
        const std::lock_guard<std::mutex> lock{mutex};
        if (is_subscriber_matched_mutex_prelocked(identity.writer_guid()))
        {
            WriteParams params;
            params.related_sample_identity(identity);
            publisher->publish(data, params);
        }
        else
        {
            ready_responses.emplace_back(std::move(data), identity, std::chrono::system_clock::now());
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    bool service<request_pub_sub_type, response_pub_sub_type,
                 handle_request_function_type>::is_subscriber_matched_mutex_prelocked(const guid &subscriber_guid)
    {
        if (!detail::is_subscriber(subscriber_guid))
        {
            return true;
        }
        return matched_subscriptions.find(subscriber_guid) != matched_subscriptions.end();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    void service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>::dispatch_responses()
    {
        std::unique_lock<std::mutex> lock{mutex};
        const auto dispatch_matched = [&]() -> bool {
            for (auto it = ready_responses.begin(); it != ready_responses.end(); ++it)
            {
                if (is_subscriber_matched_mutex_prelocked(it->identity.writer_guid()))
                {
                    WriteParams params;
                    params.related_sample_identity(it->identity);
                    publisher->publish(it->data, params);
                    ready_responses.erase(it);
                    // Wake ~service()'s drain wait, which uses cv with predicate
                    // ready_responses.empty(). Notifying here (under the lock)
                    // is safe — the waiter only re-evaluates after we release.
                    cv.notify_all();
                    return true;
                }
            }
            return false;
        };

        const auto remove_timed_out = [&](const std::chrono::system_clock::time_point &time_now) -> bool {
            for (auto it = ready_responses.begin(); it != ready_responses.end(); ++it)
            {
                if (time_now - it->time_ready > max_time_to_keep_ready_responses)
                {
                    ready_responses.erase(it);
                    // Same rationale as dispatch_matched(): wake any drain waiter.
                    cv.notify_all();
                    return true;
                }
            }
            return false;
        };

        // Period to clean up timed out unmatched responses
        constexpr std::chrono::seconds cleanup_period{1};
        while (!stop)
        {
            cv.wait_for(lock, cleanup_period, [&]() {
                return stop || dispatch_matched() || remove_timed_out(std::chrono::system_clock::now());
            });
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>::ready_response::ready_response(
        typename response_pub_sub_type::type data, const eprosima::fastdds::rtps::SampleIdentity &identity,
        const std::chrono::system_clock::time_point &time_ready)
        : data(data), identity(identity), time_ready(time_ready)
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    std::shared_ptr<service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>> make_service(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name, handle_request_function_type handle_request_function,
        const std::int32_t max_history_depth, std::optional<DurabilityQosPolicyKind> durability_kind,
        const std::int32_t endpoint_history_depth)
    {
        return std::make_shared<service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>>(
            std::move(participant), request_topic_name, response_topic_name, std::move(handle_request_function),
            max_history_depth, durability_kind, endpoint_history_depth);
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    std::shared_ptr<service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>> make_service(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        handle_request_function_type handle_request_function, const std::int32_t max_history_depth,
        std::optional<DurabilityQosPolicyKind> durability_kind, const std::int32_t endpoint_history_depth)
    {
        return std::make_shared<service<request_pub_sub_type, response_pub_sub_type, handle_request_function_type>>(
            std::move(participant), service_name, std::move(handle_request_function), max_history_depth,
            durability_kind, endpoint_history_depth);
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    future_response<request_pub_sub_type, response_pub_sub_type>::future_response(
        std::shared_ptr<detail::service_client_basic<request_pub_sub_type, response_pub_sub_type>> basic_client,
        std::shared_ptr<detail::response_data<typename response_pub_sub_type::type>> data)
        : basic_client(std::move(basic_client)), data(std::move(data))
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    future_response<request_pub_sub_type, response_pub_sub_type> request(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name, typename request_pub_sub_type::type &request_data,
        const std::chrono::milliseconds stable_matches_period, std::chrono::milliseconds service_match_timeout,
        std::optional<DurabilityQosPolicyKind> durability_kind, const std::int32_t endpoint_history_depth)
    {
        using response_type = typename response_pub_sub_type::type;
        auto response = std::make_shared<detail::response_data<response_type>>();
        auto context = std::make_shared<detail::request_context<response_type>>();
        context->response = response;

        auto client = std::make_shared<detail::service_client_basic<request_pub_sub_type, response_pub_sub_type>>(
            std::move(participant), request_topic_name, response_topic_name,
            [context](const response_type &data, const SampleInfo &info) {
                const std::lock_guard<std::mutex> lock{context->mutex};
                if (context->identity == info.related_sample_identity)
                {
                    auto response = context->response.lock();
                    if (response)
                    {
                        const std::lock_guard<std::mutex> lock{response->mutex()};
                        if (!response->is_set_mutex_prelocked())
                        {
                            response->set_mutex_prelocked(data);
                        }
                    }
                }
            },
            stable_matches_period, service_match_timeout, durability_kind, endpoint_history_depth);

        client->request(request_data, context);
        return {client, response};
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    future_response<request_pub_sub_type, response_pub_sub_type> request(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        typename request_pub_sub_type::type &request_data, const std::chrono::milliseconds stable_matches_period,
        const std::chrono::milliseconds service_match_timeout, std::optional<DurabilityQosPolicyKind> durability_kind,
        const std::int32_t endpoint_history_depth)
    {
        return request<request_pub_sub_type, response_pub_sub_type>(
            std::move(participant), detail::request_prefix + service_name + detail::request_suffix,
            detail::response_prefix + service_name + detail::response_suffix, request_data, stable_matches_period,
            service_match_timeout, durability_kind, endpoint_history_depth);
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    service_client<request_pub_sub_type, response_pub_sub_type>::service_client(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name, const std::chrono::milliseconds stable_matches_period,
        const std::chrono::milliseconds service_match_timeout, std::optional<DurabilityQosPolicyKind> durability_kind,
        const std::int32_t endpoint_history_depth)
        : router(std::make_shared<detail::response_router<response_type>>())
    {
        // service_client issues many requests over a single request DataWriter / response DataReader pair, so
        // a depth-1 KEEP_LAST default would let a burst of concurrent requests overwrite undelivered samples.
        // Default the endpoints to KEEP_LAST(service_client_default_history_depth) — a bounded buffer matching
        // the service's default response history — so a burst up to that many is retained and the reliable
        // writer applies back-pressure beyond it (rather than dropping). Raise endpoint_history_depth for
        // higher concurrency.
        const std::int32_t effective_history_depth = (endpoint_history_depth == use_default_history_depth)
                                                         ? service_client_default_history_depth
                                                         : endpoint_history_depth;
        auto router_ptr = router;
        basic_client = std::make_shared<detail::service_client_basic<request_pub_sub_type, response_pub_sub_type>>(
            std::move(participant), request_topic_name, response_topic_name,
            [router_ptr](const response_type &data, const SampleInfo &info) { router_ptr->dispatch(data, info); },
            stable_matches_period, service_match_timeout, durability_kind, effective_history_depth);
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    service_client<request_pub_sub_type, response_pub_sub_type>::service_client(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        const std::chrono::milliseconds stable_matches_period, const std::chrono::milliseconds service_match_timeout,
        std::optional<DurabilityQosPolicyKind> durability_kind, const std::int32_t endpoint_history_depth)
        : service_client(std::move(participant), detail::request_prefix + service_name + detail::request_suffix,
                         detail::response_prefix + service_name + detail::response_suffix, stable_matches_period,
                         service_match_timeout, durability_kind, endpoint_history_depth)
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    future_response<request_pub_sub_type, response_pub_sub_type> service_client<
        request_pub_sub_type, response_pub_sub_type>::request(request_type &request_data)
    {
        auto response = std::make_shared<detail::response_data<response_type>>();
        auto context = std::make_shared<detail::request_context<response_type>>();
        context->response = response;
        router->add(context);
        basic_client->request(request_data, context);
        return {basic_client, response};
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    bool service_client<request_pub_sub_type, response_pub_sub_type>::wait_for_service(
        const std::chrono::milliseconds timeout, const std::chrono::milliseconds stable_matches_period)
    {
        return basic_client->wait_for_service(timeout, stable_matches_period);
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    std::shared_ptr<service_client<request_pub_sub_type, response_pub_sub_type>> make_service_client(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name, const std::chrono::milliseconds stable_matches_period,
        const std::chrono::milliseconds service_match_timeout, std::optional<DurabilityQosPolicyKind> durability_kind,
        const std::int32_t endpoint_history_depth)
    {
        return std::make_shared<service_client<request_pub_sub_type, response_pub_sub_type>>(
            std::move(participant), request_topic_name, response_topic_name, stable_matches_period,
            service_match_timeout, durability_kind, endpoint_history_depth);
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    std::shared_ptr<service_client<request_pub_sub_type, response_pub_sub_type>> make_service_client(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        const std::chrono::milliseconds stable_matches_period, const std::chrono::milliseconds service_match_timeout,
        std::optional<DurabilityQosPolicyKind> durability_kind, const std::int32_t endpoint_history_depth)
    {
        return std::make_shared<service_client<request_pub_sub_type, response_pub_sub_type>>(
            std::move(participant), service_name, stable_matches_period, service_match_timeout, durability_kind,
            endpoint_history_depth);
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    template <class rep, class period>
    std::future_status future_response<request_pub_sub_type, response_pub_sub_type>::wait_for(
        const std::chrono::duration<rep, period> &timeout_duration) const
    {
        return wait_until(std::chrono::system_clock::now() + timeout_duration);
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    template <typename clock, typename duration>
    std::future_status future_response<request_pub_sub_type, response_pub_sub_type>::wait_until(
        const std::chrono::time_point<clock, duration> &timeout_time) const
    {
        if (!data)
        {
            throw std::future_error{std::future_errc::no_state};
        }

        std::unique_lock<std::mutex> lock{data->mutex()};
        std::exception_ptr error{};
        if (data->cv().wait_until(lock, timeout_time, [&]() {
                return (error = data->get_error_mutex_prelocked()) != nullptr || data->is_set_mutex_prelocked();
            }))
        {
            if (error)
            {
                std::rethrow_exception(error);
            }

            return std::future_status::ready;
        }

        return std::future_status::timeout;
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    const typename future_response<request_pub_sub_type, response_pub_sub_type>::response_type &future_response<
        request_pub_sub_type, response_pub_sub_type>::get() const
    {
        if (!data)
        {
            throw std::future_error{std::future_errc::no_state};
        }

        return data->get();
    }
}  // namespace provizio::dds

#endif  // DDS_REQUEST_RESPONSE
