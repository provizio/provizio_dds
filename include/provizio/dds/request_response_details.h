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

#ifndef DDS_REQUEST_RESPONSE_DETAILS
#define DDS_REQUEST_RESPONSE_DETAILS

#include <algorithm>
#include <array>
#include <assert.h>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <fastdds/rtps/common/SampleIdentity.hpp>

#include "provizio/dds/common.h"
#include "provizio/dds/function_traits.h"
#include "provizio/dds/ignore_request.h"
#include "provizio/dds/logging.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/subscriber.h"

namespace provizio::dds::detail
{
    /**
     * @file request_response_details.h
     * @brief Internal helpers for request/response implementation.
     *
     * Contains utilities and internal classes used by the public request/response
     * API in `request_response.h`, including request queueing, response dispatching
     * and client-side correlation handling.
     */
    // Header-only constants avoid exporting std::string objects across the DLL
    // boundary (fragile due to allocator/CRT coupling).
    inline constexpr const char request_prefix[] = "rq/";
    inline constexpr const char response_prefix[] = "rr/";
    inline constexpr const char request_suffix[] = "Request";
    inline constexpr const char response_suffix[] = "Reply";
    inline constexpr const char requests_queue_full_error_message[] =
        "The service requests queue is full! A request will be dropped.";

    /**
     * @brief Converts history depth QoS into a bounded request queue size.
     * @param max_history_depth Use negative value to keep default, 0 for minimal, positive for specific max queue size.
     * @return Maximum number of requests to buffer.
     */
    PROVIZIO_DDS_API std::size_t to_max_queue_size(const std::int32_t max_history_depth);

    /**
     * @brief Checks whether a GUID refers to a DataReader endpoint (subscriber GUID).
     */
    PROVIZIO_DDS_API bool is_subscriber(const guid &guid_to_check);

    template <typename function_type, typename = void> struct returns_future : std::false_type
    {
    };

    template <typename function_type>
    struct returns_future<function_type,
                          std::enable_if_t<std::is_convertible_v<
                              decltype(std::declval<typename function_traits<function_type>::return_type>().wait_for(
                                  std::chrono::seconds{0})),
                              std::future_status>>> : std::true_type
    {
    };

    template <typename response_pub_sub_type>
    using on_response_function_type =
        std::function<void(typename response_pub_sub_type::type, const eprosima::fastdds::rtps::SampleIdentity &)>;

    /**
     * @brief Thread-safe storage for a single response value.
     */
    template <typename response_type> class response_data;

    /**
     * @brief Request context storing correlation identity and shared response.
     */
    template <typename response_type> struct request_context;

    /**
     * @brief Lightweight client capable of sending requests and receiving responses.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type> class service_client_basic;

    /**
     * @brief Routes responses received on one response DataReader to the matching pending request.
     */
    template <typename response_type> class response_router;

    /**
     * @brief Hash functor for `guid` to be used in unordered containers.
     */
    struct PROVIZIO_DDS_API guid_hash
    {
        std::size_t operator()(const guid &the_guid) const;
    };

    /**
     * @brief A helper class to handle requests in a service. It supports both synchronous and asynchronous
     * (returning a future) request handlers via appropriate specializations.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @tparam handle_request_function_type The type of the request handler function.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type,
              typename = void>
    class request_handler
    {
      public:
        /**
         * @brief Constructs a request handler for synchronous request processors.
         * @param handle_request_function Function that produces a response from a request.
         * @param on_response_function Callback to publish/dispatch a response with correlation identity.
         * @param max_queue_size Maximum number of queued requests before dropping.
         */
        request_handler(handle_request_function_type handle_request_function,
                        on_response_function_type<response_pub_sub_type> on_response_function,
                        const std::size_t max_queue_size);
        ~request_handler();

        /**
         * @brief Stops the processing thread and joins it. Idempotent.
         */
        void stop_and_join();

        /**
         * @brief Enqueue a new request for processing.
         */
        void handle_request(typename request_pub_sub_type::type request,
                            const eprosima::fastdds::rtps::SampleIdentity &identity);

      private:
        void process_requests();

        using queued_request = std::pair<typename request_pub_sub_type::type, eprosima::fastdds::rtps::SampleIdentity>;

        const std::size_t max_queue_size;
        const handle_request_function_type handle_request_function;
        const on_response_function_type<response_pub_sub_type> on_response_function;

        bool stop{false};
        std::mutex requests_queue_mutex;
        std::condition_variable cv;
        std::queue<queued_request> requests_queue;
        std::thread thread;
    };

    /**
     * @brief A helper class to handle requests in a service. It supports both synchronous and asynchronous
     * request handlers. This is a specialization for asynchronous request handlers that return a future.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     * @tparam handle_request_function_type The type of the request handler function.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    class request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                          std::enable_if_t<returns_future<handle_request_function_type>::value>>
    {
      public:
        /**
         * @brief Constructs a request handler for asynchronous request processors returning std::future.
         */
        request_handler(handle_request_function_type handle_request_function,
                        on_response_function_type<response_pub_sub_type> on_response_function,
                        const std::size_t max_queue_size);
        ~request_handler();

        /**
         * @brief Stops the processing thread and joins it. Idempotent.
         */
        void stop_and_join();

        /**
         * @brief Submits a request whose response is produced asynchronously.
         */
        void handle_request(const typename request_pub_sub_type::type &request,
                            const eprosima::fastdds::rtps::SampleIdentity &identity);

      private:
        using future_type = typename function_traits<handle_request_function_type>::return_type;
        void handle_futures();

        static constexpr std::chrono::milliseconds handler_period{20};
        static constexpr std::chrono::milliseconds dont_wait{0};
        const std::size_t max_queue_size;
        const handle_request_function_type handle_request_function;
        const on_response_function_type<response_pub_sub_type> on_response_function;
        std::mutex mutex;
        bool stop{false};
        std::condition_variable cv;
        std::vector<std::pair<future_type, eprosima::fastdds::rtps::SampleIdentity>> futures;
        std::thread handler_thread;
    };

    template <typename response_type> class response_data
    {
      public:
        /** @return True if a value has been set. The mutex() has to be locked by the caller!*/
        bool is_set_mutex_prelocked() const;
        /** @return Stored exception (if any). The mutex() has to be locked by the caller! */
        std::exception_ptr get_error_mutex_prelocked() const;
        /** @brief Sets the response, notifying any waiters. The mutex() has to be locked by the caller! Throws if
         * already set. */
        void set_mutex_prelocked(const response_type &response);
        /** @brief Stores an exception and notifies waiters. Safe to call without holding mutex(). */
        void set_error(std::exception_ptr error);
        /** @brief Gets the response. Throws std::future_error if unset. */
        const response_type &get() const;
        /** @brief Access to internal mutex for waiting. */
        std::mutex &mutex() const;
        /** @brief Access to internal condition variable for waiting. */
        std::condition_variable &cv() const;

      private:
        mutable std::mutex response_mutex;
        mutable std::condition_variable value_cv;
        response_type value;
        std::exception_ptr error;
        bool has_value{false};
    };

    template <typename response_type> struct request_context
    {
        using sample_identity = eprosima::fastdds::rtps::SampleIdentity;

        /**
         * @brief Propagates an exception to the shared response, if it is still alive.
         * @param the_error Exception pointer to store.
         */
        void error(std::exception_ptr the_error)
        {
            auto response_shared = response.lock();
            if (response_shared)
            {
                response_shared->set_error(the_error);
            }
        }

        std::mutex mutex;
        sample_identity identity;
        std::weak_ptr<response_data<response_type>> response;
    };

    /**
     * @brief A basic service client that can send requests and receive responses. For internal use by
     * future_response.
     *
     * @tparam request_pub_sub_type The type of the request message.
     * @tparam response_pub_sub_type The type of the response message.
     */
    template <typename request_pub_sub_type, typename response_pub_sub_type> class service_client_basic
    {
      public:
        /**
         * @brief Exception thrown when matching publishers/subscribers times out.
         */
        class timeout_exception : public std::runtime_error
        {
          public:
            using std::runtime_error::runtime_error;
        };

        /**
         * @brief Exception thrown when a request is interrupted (e.g., client destruction).
         */
        class interrupted_exception : public std::runtime_error
        {
          public:
            using std::runtime_error::runtime_error;
        };

        /**
         * @brief Exception thrown when the request DataWriter fails to publish.
         */
        class failed_to_publish_exception : public std::runtime_error
        {
          public:
            using std::runtime_error::runtime_error;
        };

        /** @brief Default time window used to verify that match counts stay stable. */
        static constexpr std::chrono::milliseconds default_wait_for_stable_matches_period{1000};

      public:
        /**
         * @brief Basic service client that sends requests and receives responses.
         *
         * Creates a volatile (default) request DataWriter and a reliable response DataReader, and internally
         * waits for graph discovery to become stable (using get_num_matched_subscribers/publishers on both
         * endpoints) before sending the first request. Requests issued before readiness are queued and flushed
         * automatically once matched.
         */
        template <typename handle_response_function_type>
        /**
         * @brief Builds a client from explicit request/response topic names.
         * @param participant Domain participant to create entities with.
         * @param request_topic_name Request topic name.
         * @param response_topic_name Response topic name.
         * @param handle_response_function Callback invoked for every received response sample.
         * @param stable_matches_period Additional settling window that the match count must remain stable for before
         * sending the first request (0 skips the extra wait). Useful when there are multiple sensors in the network, so
         * we make sure to match all of them prior to sending the request.
         * @param service_match_timeout Maximum time for the whole readiness wait — endpoint matching AND the
         * post-match settling window (see @c stable_matches_period); 0 waits indefinitely. A finite value should
         * comfortably exceed @c stable_matches_period, or it can expire before settling completes.
         * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS) applied to the
         * request DataWriter and response DataReader; std::nullopt keeps the current default.
         * @param endpoint_history_depth KEEP_LAST history depth for the request/response endpoints;
         * use_default_history_depth keeps the current default.
         */
        service_client_basic(std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
                             const std::string &response_topic_name,
                             handle_response_function_type handle_response_function,
                             std::chrono::milliseconds stable_matches_period,
                             std::chrono::milliseconds service_match_timeout,
                             std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
                             std::int32_t endpoint_history_depth = use_default_history_depth);
        template <typename handle_response_function_type>
        /**
         * @brief Builds a client using a logical service name (topics are inferred).
         * @param participant Domain participant to create entities with.
         * @param service_name Logical service name (used to derive request/response topics).
         * @param handle_response_function Callback invoked for every received response sample.
         * @param stable_matches_period Additional settling window that the match count must remain stable for before
         * sending the first request (0 skips the extra wait). Useful when there are multiple sensors in the network, so
         * we make sure to match all of them prior to sending the request.
         * @param service_match_timeout Maximum time for the whole readiness wait — endpoint matching AND the
         * post-match settling window (see @c stable_matches_period); 0 waits indefinitely. A finite value should
         * comfortably exceed @c stable_matches_period, or it can expire before settling completes.
         * @param durability_kind Optional DDS durability kind (e.g. TRANSIENT_LOCAL_DURABILITY_QOS) applied to the
         * request DataWriter and response DataReader; std::nullopt keeps the current default.
         * @param endpoint_history_depth KEEP_LAST history depth for the request/response endpoints;
         * use_default_history_depth keeps the current default.
         */
        service_client_basic(std::shared_ptr<domain_participant> participant, const std::string &service_name,
                             handle_response_function_type handle_response_function,
                             std::chrono::milliseconds stable_matches_period,
                             std::chrono::milliseconds service_match_timeout,
                             std::optional<DurabilityQosPolicyKind> durability_kind = std::nullopt,
                             std::int32_t endpoint_history_depth = use_default_history_depth);

        /**
         * @brief Destructor stops the background readiness wait and joins its future to ensure clean shutdown.
         */
        ~service_client_basic();

        /**
         * @brief Sends a request or queues it until matching completes.
         *
         * If endpoints are not matched yet, the request is deferred and gets flushed automatically once the
         * client becomes ready. Failures (matching timeout, publish failure, interruption) are propagated via
         * the @p context by setting an exception on its associated response_data.
         */
        void request(typename request_pub_sub_type::type &request_data,
                     const std::shared_ptr<request_context<typename response_pub_sub_type::type>> &context);

        /**
         * @brief Blocks until the request DataWriter and response DataReader are matched and have been
         * stable for stable_matches_period, bounded by timeout (0 = wait indefinitely). Returns true once
         * ready, false on timeout, client shutdown, or a prior error. Performs a live poll, so it includes
         * the settling window (all currently-present services on the topic discovered).
         *
         * Stability is NOT guaranteed under sustained churn: to avoid blocking indefinitely while the match
         * count keeps changing, settling is hard-capped at 4x stable_matches_period measured from the first
         * match. Once that cap elapses the call returns ready even if matches are still changing — so a
         * caller should not treat a true return as a strict "no further matches will appear" guarantee.
         */
        bool wait_for_service(std::chrono::milliseconds timeout, std::chrono::milliseconds stable_matches_period);

      private:
        using deferred_request = std::pair<typename request_pub_sub_type::type,
                                           std::shared_ptr<request_context<typename response_pub_sub_type::type>>>;

        void do_request_mutex_prelocked(typename request_pub_sub_type::type &request_data,
                                        request_context<typename response_pub_sub_type::type> &context);
        void request_deferred_mutex_prelocked();
        bool stopped() const;
        bool wait_for_matched_and_settled(std::chrono::milliseconds timeout,
                                          std::chrono::milliseconds stable_matches_period);

        /** @brief Minimal waiting time for stable match checking. */
        static constexpr std::chrono::milliseconds min_wait_for_stable_matches_period{50};

        mutable std::mutex mutex;
        bool stop{false};
        bool matched{false};
        std::exception_ptr error;
        std::vector<deferred_request> deferred_requests;
        std::future<bool> wait_till_matched_future;
        std::shared_ptr<data_publisher<request_pub_sub_type>> publisher;
        std::shared_ptr<subscriber_handle<response_pub_sub_type>> subscriber;
    };

    /**
     * @brief Routes responses received on a single response DataReader to the matching pending request,
     * enabling many concurrent in-flight requests over one request/response endpoint pair (used by
     * service_client).
     *
     * Every request from a given client shares that client's response-reader GUID but is published with a
     * distinct request sequence number, so a response's related_sample_identity uniquely identifies the
     * request it answers. First reply wins (mirrors the one-shot request()).
     *
     * @tparam response_type The concrete response message type.
     */
    template <typename response_type> class response_router
    {
      public:
        /** @brief Registers a pending request; first prunes contexts whose future_response was dropped. */
        void add(std::shared_ptr<request_context<response_type>> context)
        {
            const std::lock_guard<std::mutex> lock{mutex};
            prune_abandoned_mutex_held();
            contexts.emplace_back(std::move(context));
        }

        /** @brief Delivers a received response to the first matching pending request, then forgets it. */
        void dispatch(const response_type &data, const SampleInfo &info)
        {
            std::shared_ptr<request_context<response_type>> matched;
            {
                const std::lock_guard<std::mutex> lock{mutex};
                // Reap contexts whose future_response was dropped, so a stream of responses keeps the set
                // bounded even when the caller never adds further requests.
                prune_abandoned_mutex_held();
                for (auto it = contexts.begin(); it != contexts.end(); ++it)
                {
                    const std::lock_guard<std::mutex> context_lock{(*it)->mutex};
                    if ((*it)->identity == info.related_sample_identity)
                    {
                        matched = *it;
                        contexts.erase(it);
                        break;
                    }
                }
            }
            if (matched)
            {
                auto response = matched->response.lock();
                if (response)
                {
                    const std::lock_guard<std::mutex> lock{response->mutex()};
                    if (!response->is_set_mutex_prelocked())
                    {
                        response->set_mutex_prelocked(data);
                    }
                }
            }
        }

      private:
        /** @brief Drops contexts whose future_response (and thus response_data) has been destroyed. The
         * caller must hold @c mutex. Keeps the set bounded to genuinely-outstanding requests. */
        void prune_abandoned_mutex_held()
        {
            contexts.erase(std::remove_if(contexts.begin(), contexts.end(),
                                          [](const std::shared_ptr<request_context<response_type>> &candidate) {
                                              return candidate->response.expired();
                                          }),
                           contexts.end());
        }

        std::mutex mutex;
        std::vector<std::shared_ptr<request_context<response_type>>> contexts;
    };

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type,
              typename sfinae_placeholder>
    request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                    sfinae_placeholder>::request_handler(handle_request_function_type handle_request_function,
                                                         on_response_function_type<response_pub_sub_type>
                                                             on_response_function,
                                                         const std::size_t max_queue_size)
        : max_queue_size(max_queue_size), handle_request_function(std::move(handle_request_function)),
          on_response_function(std::move(on_response_function)), thread(&request_handler::process_requests, this)
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type,
              typename sfinae_placeholder>
    request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                    sfinae_placeholder>::~request_handler()
    {
        stop_and_join();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type,
              typename sfinae_placeholder>
    void request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                         sfinae_placeholder>::stop_and_join()
    {
        {
            const std::lock_guard<std::mutex> lock{requests_queue_mutex};
            stop = true;
        }
        cv.notify_all();
        if (thread.joinable())
        {
            thread.join();
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type,
              typename sfinae_placeholder>
    void request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                         sfinae_placeholder>::handle_request(typename request_pub_sub_type::type request,
                                                             const eprosima::fastdds::rtps::SampleIdentity &identity)
    {
        bool queue_full = false;
        {
            const std::lock_guard<std::mutex> lock{requests_queue_mutex};
            if (requests_queue.size() < max_queue_size)
            {
                requests_queue.emplace(std::move(request), identity);
            }
            else
            {
                queue_full = true;
            }
        }
        // Logged outside the lock: the user-supplied log callback may call back
        // into provizio_dds APIs, so emitting under requests_queue_mutex risks
        // deadlock / unbounded blocking on a hot publish path.
        if (queue_full)
        {
            try
            {
                log_error() << requests_queue_full_error_message;
            }
            catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
            {
            }
        }
        cv.notify_all();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type,
              typename SFINAE_PLACEHOLDER>
    void request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                         SFINAE_PLACEHOLDER>::process_requests()
    {
        std::unique_lock<std::mutex> lock{requests_queue_mutex};
        while (!stop)
        {
            cv.wait(lock, [&]() { return stop || !requests_queue.empty(); });
            if (!stop && !requests_queue.empty())
            {
                const queued_request request{std::move(requests_queue.front())};
                requests_queue.pop();
                lock.unlock();
                try
                {
                    on_response_function(handle_request_function(request.first), request.second);
                }
                catch (const ignore_request &)
                {
                    // Silently drop the request
                }
                catch (const std::exception &exception)
                {
                    // A throwing handler must not escape this handler thread — it would
                    // std::terminate the process and stop the service. Report it through the
                    // configurable logger (we are outside requests_queue_mutex here, so the
                    // user log callback may safely re-enter provizio_dds) and keep the thread
                    // alive for the next request. The logging itself can allocate/throw, so
                    // guard it too — nothing may escape this handler thread.
                    try
                    {
                        log_error() << "service request handler threw: " << exception.what() << "; request dropped";
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                    {
                    }
                }
                catch (...)
                {
                    try
                    {
                        log_error() << "service request handler threw a non-std::exception; request dropped";
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                    {
                    }
                }
                lock.lock();
            }
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                    std::enable_if_t<returns_future<handle_request_function_type>::value>>::
        request_handler(handle_request_function_type handle_request_function,
                        on_response_function_type<response_pub_sub_type> on_response_function,
                        const std::size_t max_queue_size)
        : max_queue_size(max_queue_size), handle_request_function(std::move(handle_request_function)),
          on_response_function(std::move(on_response_function)), handler_thread(&request_handler::handle_futures, this)
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                    std::enable_if_t<returns_future<handle_request_function_type>::value>>::~request_handler()
    {
        stop_and_join();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    void request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                         std::enable_if_t<returns_future<handle_request_function_type>::value>>::stop_and_join()
    {
        {
            const std::lock_guard<std::mutex> lock{mutex};
            stop = true;
        }
        cv.notify_one();
        if (handler_thread.joinable())
        {
            handler_thread.join();
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    void request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                         std::enable_if_t<returns_future<handle_request_function_type>::value>>::
        handle_request(const typename request_pub_sub_type::type &request,
                       const eprosima::fastdds::rtps::SampleIdentity &identity)
    {
        bool queue_full = false;
        std::string handler_error;
        {
            const std::lock_guard<std::mutex> lock{mutex};
            if (futures.size() < max_queue_size)
            {
                try
                {
                    // Handler may throw ignore_request before returning a future
                    futures.emplace_back(handle_request_function(request), identity);
                }
                catch (const ignore_request &)
                {
                    // Silently drop
                }
                catch (const std::exception &exception)
                {
                    // The handler's synchronous part runs on the Fast-DDS reception
                    // thread; a throw must not escape into Fast-DDS (it would
                    // std::terminate the process). Capture here, log outside the lock.
                    // Formatting the message can itself allocate/throw, so guard it —
                    // drop the message rather than let an allocation failure escape.
                    try
                    {
                        handler_error =
                            std::string{"service request handler threw: "} + exception.what() + "; request dropped";
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a formatting failure must not escape
                    {
                        handler_error.clear();
                    }
                }
                catch (...)
                {
                    try
                    {
                        handler_error = "service request handler threw a non-std::exception; request dropped";
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a formatting failure must not escape
                    {
                        handler_error.clear();
                    }
                }
            }
            else
            {
                queue_full = true;
            }
        }
        // Logged outside the lock: the user-supplied log callback may call back
        // into provizio_dds APIs, so emitting under mutex risks deadlock /
        // unbounded blocking on a hot publish path.
        // Best-effort logging only: this still runs on the Fast-DDS reception thread, so a
        // std::bad_alloc from the streaming logger must not escape into Fast-DDS.
        if (!handler_error.empty())
        {
            try
            {
                log_error() << handler_error;
            }
            catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
            {
            }
        }
        if (queue_full)
        {
            try
            {
                log_error() << requests_queue_full_error_message;
            }
            catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
            {
            }
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    void request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                         std::enable_if_t<returns_future<handle_request_function_type>::value>>::handle_futures()
    {
        std::unique_lock<std::mutex> lock{mutex};
        while (!stop)
        {
            if (cv.wait_for(lock, handler_period, [this] { return stop; }))
            {
                assert(stop);
                return;
            }

            std::vector<std::string> handler_errors;
            for (std::size_t i = futures.size(); i > 0; --i)
            {
                auto future_iterator = futures.begin() + (i - 1);
                if (future_iterator->first.wait_for(dont_wait) == std::future_status::ready)
                {
                    try
                    {
                        on_response_function(future_iterator->first.get(), future_iterator->second);
                    }
                    catch (const ignore_request &)
                    {
                        // Silently drop the request
                    }
                    catch (const std::exception &exception)
                    {
                        // The async handler's future resolved to an exception; it must
                        // not escape this handler thread (std::terminate). Capture here,
                        // log outside the lock below. Formatting/storing the message can
                        // itself allocate/throw, so guard it — drop the message rather than
                        // let an allocation failure escape.
                        try
                        {
                            handler_errors.emplace_back(std::string{"service request handler threw: "} +
                                                        exception.what() + "; request dropped");
                        }
                        catch (...)  // NOLINT(bugprone-empty-catch): a formatting failure must not escape
                        {
                        }
                    }
                    catch (...)
                    {
                        try
                        {
                            handler_errors.emplace_back(
                                "service request handler threw a non-std::exception; request dropped");
                        }
                        catch (...)  // NOLINT(bugprone-empty-catch): a formatting failure must not escape
                        {
                        }
                    }
                    futures.erase(future_iterator);
                }
            }

            // Logged outside the lock: the user log callback may re-enter provizio_dds.
            if (!handler_errors.empty())
            {
                lock.unlock();
                for (const auto &message : handler_errors)
                {
                    // Best-effort: a std::bad_alloc from the streaming logger must not
                    // escape this handler thread.
                    try
                    {
                        log_error() << message;
                    }
                    catch (...)  // NOLINT(bugprone-empty-catch): a logging failure must not escape either
                    {
                    }
                }
                lock.lock();
            }
        }
    }

    template <typename response_type> bool response_data<response_type>::is_set_mutex_prelocked() const
    {
        return has_value;
    }

    template <typename response_type> std::exception_ptr response_data<response_type>::get_error_mutex_prelocked() const
    {
        return error;
    }

    template <typename response_type>
    void response_data<response_type>::set_mutex_prelocked(const response_type &response)
    {
        if (has_value)
        {
            throw std::future_error{std::future_errc::promise_already_satisfied};
        }
        value = response;
        has_value = true;
        value_cv.notify_all();
    }

    template <typename response_type> void response_data<response_type>::set_error(std::exception_ptr error)
    {
        {
            std::lock_guard<std::mutex> lock{response_mutex};
            this->error = error;
        }
        value_cv.notify_all();
    }

    template <typename response_type> const response_type &response_data<response_type>::get() const
    {
        const std::lock_guard<std::mutex> lock{response_mutex};
        if (error)
        {
            std::rethrow_exception(error);
        }

        if (!has_value)
        {
            throw std::future_error{std::future_errc::no_state};
        }
        return value;
    }

    template <typename response_type> std::mutex &response_data<response_type>::mutex() const
    {
        return response_mutex;
    }

    template <typename response_type> std::condition_variable &response_data<response_type>::cv() const
    {
        return value_cv;
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    template <typename handle_response_function_type>
    service_client_basic<request_pub_sub_type, response_pub_sub_type>::service_client_basic(
        std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
        const std::string &response_topic_name, handle_response_function_type handle_response_function,
        const std::chrono::milliseconds stable_matches_period, const std::chrono::milliseconds service_match_timeout,
        std::optional<DurabilityQosPolicyKind> durability_kind, const std::int32_t endpoint_history_depth)
        : publisher(make_publisher<request_pub_sub_type>(participant, request_topic_name, RELIABLE_RELIABILITY_QOS,
                                                         endpoint_history_depth, durability_kind)),
          subscriber(make_subscriber<response_pub_sub_type>(
              participant, response_topic_name, std::move(handle_response_function), RELIABLE_RELIABILITY_QOS,
              endpoint_history_depth, durability_kind))
    {
        // Establish readiness in the background so requests issued before matching auto-defer and
        // flush once ready. The settle-poll itself lives in wait_for_matched_and_settled() (shared
        // with the public wait_for_service()).
        wait_till_matched_future =
            std::async(std::launch::async, [this, stable_matches_period, service_match_timeout]() {
                const bool ready = wait_for_matched_and_settled(service_match_timeout, stable_matches_period);
                const std::lock_guard<std::mutex> lock{mutex};
                if (!ready && !stop && !error)
                {
                    // A finite service_match_timeout elapsed before matching+settling completed.
                    error = std::make_exception_ptr(timeout_exception{
                        "Service readiness wait timed out (endpoints must match and then stay stable for the "
                        "settling window)"});
                }
                matched = true;  // OK even on timeout|stop|error: error is already set in those cases.
                request_deferred_mutex_prelocked();
                return matched;
            });
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    template <typename handle_response_function_type>
    service_client_basic<request_pub_sub_type, response_pub_sub_type>::service_client_basic(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        handle_response_function_type handle_response_function, const std::chrono::milliseconds stable_matches_period,
        const std::chrono::milliseconds service_match_timeout, std::optional<DurabilityQosPolicyKind> durability_kind,
        const std::int32_t endpoint_history_depth)
        : service_client_basic(std::move(participant), request_prefix + service_name + request_suffix,
                               response_prefix + service_name + response_suffix, std::move(handle_response_function),
                               stable_matches_period, service_match_timeout, durability_kind, endpoint_history_depth)
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    service_client_basic<request_pub_sub_type, response_pub_sub_type>::~service_client_basic()
    {
        {
            const std::lock_guard<std::mutex> lock{mutex};
            error = std::make_exception_ptr(interrupted_exception{"Request interrupted"});
            stop = true;
        }
        wait_till_matched_future.wait();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    void service_client_basic<request_pub_sub_type, response_pub_sub_type>::request(
        typename request_pub_sub_type::type &request_data,
        const std::shared_ptr<request_context<typename response_pub_sub_type::type>> &context)
    {
        const std::lock_guard<std::mutex> lock{mutex};

        if (error)
        {
            context->error(error);
            return;
        }

        if (matched)
        {
            do_request_mutex_prelocked(request_data, *context);
        }
        else
        {
            deferred_requests.emplace_back(request_data, context);
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    void service_client_basic<request_pub_sub_type, response_pub_sub_type>::do_request_mutex_prelocked(
        typename request_pub_sub_type::type &request_data,
        request_context<typename response_pub_sub_type::type> &context)
    {
        bool success = false;

        if (!error)
        {
            const auto subscriber_guid = subscriber->get_guid();
            WriteParams params;
            params.related_sample_identity().writer_guid() = subscriber_guid;
            const std::lock_guard<std::mutex> lock{context.mutex};
            if (publisher->publish(request_data, params))
            {
                context.identity.writer_guid() = subscriber_guid;
                context.identity.sequence_number() = params.sample_identity().sequence_number();
                success = true;
            }
        }

        if (!success)
        {
            // If an error has already occurred (e.g., timeout or interruption), propagate it. Otherwise, report the
            // publish failure here.
            context.error(error != nullptr
                              ? error
                              : std::make_exception_ptr(failed_to_publish_exception{"Failed to publish the request"}));
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    void service_client_basic<request_pub_sub_type, response_pub_sub_type>::request_deferred_mutex_prelocked()
    {
        for (auto &it : deferred_requests)
        {
            do_request_mutex_prelocked(it.first, *it.second);
        }
        deferred_requests.clear();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    bool service_client_basic<request_pub_sub_type, response_pub_sub_type>::stopped() const
    {
        const std::lock_guard<std::mutex> lock{mutex};
        return stop;
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    bool service_client_basic<request_pub_sub_type, response_pub_sub_type>::wait_for_matched_and_settled(
        const std::chrono::milliseconds timeout, const std::chrono::milliseconds stable_matches_period)
    {
        // Mirrors ROS 2 rmw_fastrtps' rmw_service_server_is_available (request DataWriter and response
        // DataReader both matched, in equal numbers), then holds for a settling window. Fast-DDS fires the
        // writer-side and reader-side match callbacks independently and with potentially large asynchronous
        // skew; publishing before the *service's* request reader is live drops the request (that reader is
        // VOLATILE for ROS 2 interop and does not receive samples published before it matched). The settle
        // also lets additional services on the same service name be discovered (provizio routes multiple
        // sensors over one service name, filtered by frame_id) so the request reaches all of them. To avoid
        // waiting indefinitely under churn, cap the settle at settle_cap_multiplier x the window after the
        // first match. Match counts are instantaneous snapshots. Returns true once matched+settled, false on
        // timeout (finite timeout), client stop, or a concurrently-set error (e.g. the background readiness
        // task's own finite timeout firing mid-wait); does not mutate error/matched.
        constexpr int settle_cap_multiplier = 4;
        const auto settle = stable_matches_period;
        const auto settle_cap = settle * settle_cap_multiplier;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        const bool has_timeout = timeout.count() != 0;

        int prev_subscribers = -1;
        int prev_publishers = -1;
        auto last_change = std::chrono::steady_clock::now();
        bool have_first_match = false;
        auto first_match = std::chrono::steady_clock::now();

        while (!stopped())
        {
            {
                // The background readiness task sets `error` without setting `stop` when a finite
                // service_match_timeout elapses; bail here too so a wait_for_service() call already inside
                // this loop returns false rather than reporting a ready service the now-errored client can
                // no longer issue requests against.
                const std::lock_guard<std::mutex> lock{mutex};
                if (error)
                {
                    return false;
                }
            }

            const int subscribers =
                publisher->get_num_matched_subscribers(std::chrono::milliseconds{0}, std::chrono::milliseconds{0});
            const int publishers =
                subscriber->get_num_matched_publishers(std::chrono::milliseconds{0}, std::chrono::milliseconds{0});
            const auto now = std::chrono::steady_clock::now();

            if (subscribers != prev_subscribers || publishers != prev_publishers)
            {
                last_change = now;
                prev_subscribers = subscribers;
                prev_publishers = publishers;
            }

            if (subscribers > 0 && publishers > 0 && subscribers == publishers)
            {
                if (!have_first_match)
                {
                    have_first_match = true;
                    first_match = now;
                }
                if (settle.count() <= 0 || (now - last_change) >= settle || (now - first_match) >= settle_cap)
                {
                    return true;
                }
            }

            if (has_timeout && now >= deadline)
            {
                return false;
            }

            std::this_thread::sleep_for(min_wait_for_stable_matches_period);
        }

        return false;  // client stopped
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    bool service_client_basic<request_pub_sub_type, response_pub_sub_type>::wait_for_service(
        const std::chrono::milliseconds timeout, const std::chrono::milliseconds stable_matches_period)
    {
        {
            const std::lock_guard<std::mutex> lock{mutex};
            if (error)
            {
                return false;  // already interrupted or a prior (finite) construction timeout fired
            }
        }
        return wait_for_matched_and_settled(timeout, stable_matches_period);
    }

}  // namespace provizio::dds::detail

#endif  // DDS_REQUEST_RESPONSE_DETAILS
