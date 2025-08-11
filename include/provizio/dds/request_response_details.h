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

#include <iostream>

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
    extern const std::string request_prefix;
    extern const std::string response_prefix;
    extern const std::string request_suffix;
    extern const std::string response_suffix;
    extern const std::string requests_queue_full_error_message;

    /**
     * @brief Converts history depth QoS into a bounded request queue size.
     * @param max_history_depth Use negative value to keep default, 0 for minimal, positive for specific max queue size.
     * @return Maximum number of requests to buffer.
     */
    std::size_t to_max_queue_size(const std::int32_t max_history_depth);

    /**
     * @brief Checks whether a GUID refers to a DataReader endpoint (subscriber GUID).
     */
    bool is_subscriber_guid(const guid &guid_to_check);

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
        const bool is_set_mutex_prelocked() const;
        /** @brief Sets the response, notifying any waiters. The mutex() has to be locked by the caller! Throws if
         * already set. */
        void set_mutex_prelocked(const response_type &response);
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
        bool has_value{false};
    };

    template <typename response_type> struct request_context
    {
        using sample_identity = eprosima::fastrtps::rtps::SampleIdentity;
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
         * @brief Basic service client that sends requests and receives responses.
         *
         * Creates a volatile (default) request DataWriter and a reliable response DataReader, and internally
         * waits for graph discovery to become stable (using wait_till_matched on both endpoints) before sending the
         * first request. Requests issued before readiness are queued and flushed automatically once matched.
         */
        template <typename handle_response_function_type>
        service_client_basic(std::shared_ptr<domain_participant> participant, const std::string &request_topic_name,
                             const std::string &response_topic_name,
                             handle_response_function_type handle_response_function,
                             std::chrono::milliseconds post_match_delay = std::chrono::milliseconds{0});
        template <typename handle_response_function_type>
        service_client_basic(std::shared_ptr<domain_participant> participant, const std::string &service_name,
                             handle_response_function_type handle_response_function,
                             std::chrono::milliseconds post_match_delay = std::chrono::milliseconds{0});

        /**
         * @brief Destructor stops the background readiness wait and joins its future to ensure clean shutdown.
         */
        ~service_client_basic();

        /**
         * @brief Sends a request. If endpoints are not matched yet, defers the request, which then will be sent
         * when the endpoints are matched.
         * @return True if the request has been sent or deferred successfully.
         */
        bool request(typename request_pub_sub_type::type &request_data,
                     const std::shared_ptr<request_context<typename response_pub_sub_type::type>> &context);

      private:
        using deferred_request = std::pair<typename request_pub_sub_type::type,
                                           std::shared_ptr<request_context<typename response_pub_sub_type::type>>>;

        bool do_request(typename request_pub_sub_type::type &request_data,
                        request_context<typename response_pub_sub_type::type> &context);
        void request_deferred_mutex_prelocked();
        bool stopped() const;

        mutable std::mutex mutex;
        bool stop{false};
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

    template <typename response_type> const bool response_data<response_type>::is_set_mutex_prelocked() const
    {
        return has_value;
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

    template <typename response_type> const response_type &response_data<response_type>::get() const
    {
        const std::lock_guard<std::mutex> lock{response_mutex};
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
        const std::chrono::milliseconds post_match_delay)
        : publisher(make_publisher<request_pub_sub_type>(participant, request_topic_name, RELIABLE_RELIABILITY_QOS)),
          subscriber(make_subscriber<response_pub_sub_type>(
              participant, response_topic_name, std::move(handle_response_function), RELIABLE_RELIABILITY_QOS))
    {
        wait_till_matched_future = std::async(std::launch::async, [this, post_match_delay]() {
            const std::chrono::milliseconds iteration_timeout{100};
            while (!stopped() && !publisher->wait_till_matched(iteration_timeout))
            {
            }
            while (!stopped() && !subscriber->wait_till_matched(iteration_timeout))
            {
            }

            std::unique_lock<std::mutex> lock{mutex};
            if (!stop)
            {
                if (post_match_delay > std::chrono::milliseconds{0})
                {
                    lock.unlock();
                    std::this_thread::sleep_for(post_match_delay);
                    lock.lock();
                    if (stop)
                    {
                        return false;
                    }
                }
                request_deferred_mutex_prelocked();
                return true;
            }

            return false;
        });
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    template <typename handle_response_function_type>
    service_client_basic<request_pub_sub_type, response_pub_sub_type>::service_client_basic(
        std::shared_ptr<domain_participant> participant, const std::string &service_name,
        handle_response_function_type handle_response_function, std::chrono::milliseconds post_match_delay)
        : service_client_basic(std::move(participant), request_prefix + service_name + request_suffix,
                               response_prefix + service_name + response_suffix, std::move(handle_response_function),
                               post_match_delay)
    {
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    service_client_basic<request_pub_sub_type, response_pub_sub_type>::~service_client_basic()
    {
        {
            const std::lock_guard<std::mutex> lock{mutex};
            stop = true;
        }
        wait_till_matched_future.wait();
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    bool service_client_basic<request_pub_sub_type, response_pub_sub_type>::request(
        typename request_pub_sub_type::type &request_data,
        const std::shared_ptr<request_context<typename response_pub_sub_type::type>> &context)
    {
        const std::lock_guard<std::mutex> lock{mutex};
        if (wait_till_matched_future.valid() &&
            wait_till_matched_future.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready &&
            wait_till_matched_future.get())
        {
            return do_request(request_data, *context);
        }
        else
        {
            deferred_requests.emplace_back(request_data, context);
            return true;
        }
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    bool service_client_basic<request_pub_sub_type, response_pub_sub_type>::do_request(
        typename request_pub_sub_type::type &request_data,
        request_context<typename response_pub_sub_type::type> &context)
    {
        const auto subscriber_guid = subscriber->get_guid();
        WriteParams params;
        params.related_sample_identity().writer_guid() = subscriber_guid;
        const std::lock_guard<std::mutex> lock{context.mutex};
        const bool success = publisher->publish(request_data, params);
        if (success)
        {
            context.identity.writer_guid() = subscriber_guid;
            context.identity.sequence_number() = params.sample_identity().sequence_number();
        }
        return success;
    }

    template <typename request_pub_sub_type, typename response_pub_sub_type>
    void service_client_basic<request_pub_sub_type, response_pub_sub_type>::request_deferred_mutex_prelocked()
    {
        for (auto &it : deferred_requests)
        {
            do_request(it.first, *it.second);
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
