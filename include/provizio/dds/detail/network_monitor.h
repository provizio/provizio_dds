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

#ifndef DDS_DETAIL_NETWORK_MONITOR
#define DDS_DETAIL_NETWORK_MONITOR

#include <functional>
#include <memory>

#include "provizio/dds/common.h"

namespace provizio::dds::detail
{
    /**
     * @file network_monitor.h
     * @brief OS-portable address-change watcher.
     *
     * Opens a single per-process kernel notification channel for IPv4/IPv6
     * address-add and address-remove events and invokes a user callback once per
     * raw event. The caller handles burst coalescing and snapshot-diff filtering
     * on top.
     *
     * Per-OS backend:
     *  - Linux:   @c AF_NETLINK socket with @c NETLINK_ROUTE family, subscribed to
     *             @c RTMGRP_IPV4_IFADDR | @c RTMGRP_IPV6_IFADDR. A worker thread
     *             blocks in @c poll() and wakes via an internal @c eventfd for
     *             shutdown.
     *  - macOS:   @c PF_ROUTE / @c SOCK_RAW routing socket. The worker blocks in
     *             @c poll() and wakes via a pipe for shutdown.
     *  - Windows: @c NotifyUnicastIpAddressChange registers a callback that the
     *             OS invokes on its own thread; no worker thread.
     */
    class PROVIZIO_DDS_API network_monitor
    {
      public:
        /**
         * @brief Callback invoked by the monitor on each address change event. The
         * callback may be invoked from a worker thread (Linux / macOS) or directly
         * from the kernel's notification thread (Windows). Implementations must be
         * brief and thread-safe; do any heavy work in a separate thread.
         */
        using on_event_callback = std::function<void()>;

        /**
         * @brief Open the kernel notification channel and start delivering events to
         * @c callback. The monitor runs until the instance is destroyed.
         *
         * @param callback Invoked on every raw address change event. May fire many
         * times in rapid succession during a network state transition (DHCP, link
         * flap, etc.) — the caller is expected to coalesce.
         * @throw std::runtime_error if the kernel channel could not be opened.
         */
        explicit network_monitor(on_event_callback callback);

        /**
         * @brief Destroys the monitor, closes the kernel channel, and blocks until
         * any worker thread has joined. No further callbacks fire after destruction
         * completes.
         */
        ~network_monitor();

        network_monitor(const network_monitor &) = delete;
        network_monitor &operator=(const network_monitor &) = delete;
        network_monitor(network_monitor &&) = delete;
        network_monitor &operator=(network_monitor &&) = delete;

      private:
        struct impl;
        std::unique_ptr<impl> the_impl;
    };
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_NETWORK_MONITOR
