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
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include <fastdds/rtps/common/SampleIdentity.h>

#include "provizio/dds/common.h"
#include "provizio/dds/function_traits.h"
#include "provizio/dds/ignore_request.h"
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
    extern PROVIZIO_DDS_API const std::string request_prefix;
    extern PROVIZIO_DDS_API const std::string response_prefix;
    extern PROVIZIO_DDS_API const std::string request_suffix;
    extern PROVIZIO_DDS_API const std::string response_suffix;
    extern PROVIZIO_DDS_API const std::string requests_queue_full_error_message;

    /**
     * @brief Converts history depth QoS into a bounded request queue size.
     * @param max_history_depth Use negative value to keep default, 0 for minimal, positive for specific max queue size.
     * @return Maximum number of requests to buffer.
     */
    std::size_t to_max_queue_size(const std::int32_t max_history_depth);

    /**
     * @brief Checks whether a GUID refers to a DataReader endpoint (subscriber GUID).
     */
    bool is_subscriber(const guid &guid_to_check);

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
        std::function<void(typename response_pub_sub_type::type, const eprosima::fastrtps::rtps::SampleIdentity &)>;

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
     * @brief Hash functor for `guid` to be used in unordered containers.
     */
    struct guid_hash
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
         * @brief Enqueue a new request for processing.
         */
        void handle_request(typename request_pub_sub_type::type request,
                            const eprosima::fastrtps::rtps::SampleIdentity &identity);

      private:
        void process_requests();

        using queued_request = std::pair<typename request_pub_sub_type::type, eprosima::fastrtps::rtps::SampleIdentity>;

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
         * @brief Submits a request whose response is produced asynchronously.
         */
        void handle_request(const typename request_pub_sub_type::type &request,
                            const eprosima::fastrtps::rtps::SampleIdentity &identity);

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
        std::vector<std::pair<future_type, eprosima::fastrtps::rtps::SampleIdentity>> futures;
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
        using sample_identity = eprosima::fastrtps::rtps::SampleIdentity;

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
         * @param service_match_timeout Maximum time to wait for publisher/subscriber matching (0 waits indefinitely).
         */
        service_client_basic(std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
                             const std::string &response_topic_name,
                             handle_response_function_type handle_response_function,
                             std::chrono::milliseconds stable_matches_period,
                             std::chrono::milliseconds service_match_timeout);
        template <typename handle_response_function_type>
        /**
         * @brief Builds a client using a logical service name (topics are inferred).
         * @param participant Domain participant to create entities with.
         * @param service_name Logical service name (used to derive request/response topics).
         * @param handle_response_function Callback invoked for every received response sample.
         * @param stable_matches_period Additional settling window that the match count must remain stable for before
         * sending the first request (0 skips the extra wait). Useful when there are multiple sensors in the network, so
         * we make sure to match all of them prior to sending the request.
         * @param service_match_timeout Maximum time to wait for publisher/subscriber matching (0 waits indefinitely).
         */
        service_client_basic(std::shared_ptr<domain_participant> participant, const std::string &service_name,
                             handle_response_function_type handle_response_function,
                             std::chrono::milliseconds stable_matches_period,
                             std::chrono::milliseconds service_match_timeout);

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

      private:
        using deferred_request = std::pair<typename request_pub_sub_type::type,
                                           std::shared_ptr<request_context<typename response_pub_sub_type::type>>>;

        void do_request_mutex_prelocked(typename request_pub_sub_type::type &request_data,
                                        request_context<typename response_pub_sub_type::type> &context);
        void request_deferred_mutex_prelocked();
        bool stopped() const;

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
        {
            const std::lock_guard<std::mutex> lock{requests_queue_mutex};
            stop = true;
        }
        cv.notify_all();
        thread.join();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type,
              typename sfinae_placeholder>
    void request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                         sfinae_placeholder>::handle_request(typename request_pub_sub_type::type request,
                                                             const eprosima::fastrtps::rtps::SampleIdentity &identity)
    {
        {
            const std::lock_guard<std::mutex> lock{requests_queue_mutex};
            if (requests_queue.size() < max_queue_size)
            {
                requests_queue.emplace(std::move(request), identity);
            }
            else
            {
                std::cerr << requests_queue_full_error_message << std::endl;
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
        {
            const std::lock_guard<std::mutex> lock{mutex};
            stop = true;
        }
        cv.notify_one();
        handler_thread.join();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type, typename handle_request_function_type>
    void request_handler<request_pub_sub_type, response_pub_sub_type, handle_request_function_type,
                         std::enable_if_t<returns_future<handle_request_function_type>::value>>::
        handle_request(const typename request_pub_sub_type::type &request,
                       const eprosima::fastrtps::rtps::SampleIdentity &identity)
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
        }
        else
        {
            std::cerr << requests_queue_full_error_message << std::endl;
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
                    futures.erase(future_iterator);
                }
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
        const std::chrono::milliseconds stable_matches_period, const std::chrono::milliseconds service_match_timeout)
        : publisher(make_publisher<request_pub_sub_type>(participant, request_topic_name, RELIABLE_RELIABILITY_QOS)),
          subscriber(make_subscriber<response_pub_sub_type>(
              participant, response_topic_name, std::move(handle_response_function), RELIABLE_RELIABILITY_QOS))
    {
        wait_till_matched_future =
            std::async(std::launch::async, [this, stable_matches_period, service_match_timeout]() {
                // Wait for the publisher and subscriber to have the same amount of matches across a period as a way to
                // establish connection to one or more services on the topic
                const auto timeout_point = std::chrono::steady_clock::now() + service_match_timeout;
                const auto check_period =
                    stable_matches_period.count() > 0 ? stable_matches_period : min_wait_for_stable_matches_period;
                while (!stopped())
                {
                    const auto remaining_timeout = service_match_timeout.count() == 0
                                                       ? service_match_timeout
                                                       : std::max(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                                      timeout_point - std::chrono::steady_clock::now()),
                                                                  std::chrono::milliseconds{0});
                    auto matched_subscribers = std::async(std::launch::async, [&] {
                        return publisher->get_num_matched_subscribers(remaining_timeout, check_period);
                    });
                    auto matched_publishers = std::async(std::launch::async, [&] {
                        return subscriber->get_num_matched_publishers(remaining_timeout, check_period);
                    });
                    int num_matched_subscribers = matched_subscribers.get();
                    if (num_matched_subscribers > 0 && num_matched_subscribers == matched_publishers.get())
                    {
                        // Same number of matched endpoints in both the publisher and the subscriber, we're good to
                        // proceed
                        break;
                    }

                    if (service_match_timeout.count() != 0 && remaining_timeout.count() <= 0)
                    {
                        std::unique_lock<std::mutex> lock{mutex};
                        error = std::make_exception_ptr(timeout_exception{"Service matching timed out"});
                        break;
                    }

                    // To avoid too heavy CPU load
                    std::this_thread::sleep_for(min_wait_for_stable_matches_period);
                }

                std::unique_lock<std::mutex> lock{mutex};
                matched = true; // It's OK even if there was an timeout|stop|error as error is already set then
                request_deferred_mutex_prelocked();
                return matched;
            });
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    template <typename handle_response_function_type>
    service_client_basic<request_pub_sub_type, response_pub_sub_type>::service_client_basic(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        handle_response_function_type handle_response_function, const std::chrono::milliseconds stable_matches_period,
        const std::chrono::milliseconds service_match_timeout)
        : service_client_basic(std::move(participant), request_prefix + service_name + request_suffix,
                               response_prefix + service_name + response_suffix, std::move(handle_response_function),
                               stable_matches_period, service_match_timeout)
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

} // namespace provizio::dds::detail

#endif // DDS_REQUEST_RESPONSE_DETAILS
