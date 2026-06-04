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

// NotifyUnicastIpAddressChange / CancelMibChangeNotify2 /
// PMIB_UNICASTIPADDRESS_ROW / MIB_NOTIFICATION_TYPE are guarded inside
// netioapi.h by `_WIN32_WINNT >= 0x0600` (Windows Vista) / `NTDDI_VERSION
// >= NTDDI_VISTA`. Those pins are set project-wide via
// add_compile_definitions in CMakeLists.txt, so they are in effect for
// every include below.
//
// The Windows SDK headers are pulled in HERE — before network_monitor.h /
// common.h get to it — so the SDK header guards lock in this include
// order and any later transitive include via Fast-DDS sees the same
// declarations.
#if defined(_WIN32)

#include <winsock2.h>
// clang-format off
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <netioapi.h>
// clang-format on
#endif  // _WIN32

#include "provizio/dds/detail/network_monitor.h"

#if defined(_WIN32)

#pragma comment(lib, "Iphlpapi.lib")
#pragma comment(lib, "Ws2_32.lib")

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>

namespace provizio::dds::detail
{
    struct network_monitor::impl
    {
        HANDLE notification_handle{nullptr};
        on_event_callback callback;

        // Synchronisation for safely tearing down the registration. Per Microsoft:
        // "Notifications may continue to be delivered after CancelMibChangeNotify2
        // returns. To safely free the memory containing the function and parameter,
        // the application must call CancelMibChangeNotify2 and wait for any
        // outstanding callbacks to complete." We do exactly that with an in-flight
        // counter the callback maintains.
        std::atomic<bool> stopping{false};
        std::atomic<int> in_flight{0};
        std::mutex drain_mutex;
        std::condition_variable drain_cv;

        static VOID NETIOAPI_API_ on_address_change(PVOID context, PMIB_UNICASTIPADDRESS_ROW row,
                                                    MIB_NOTIFICATION_TYPE type)
        {
            (void)row;
            auto *self = static_cast<impl *>(context);
            if (self == nullptr)
            {
                return;
            }

            // Bump in-flight FIRST so close_channel's drain can observe us.
            // Decrement (and notify on last-out) is via the RAII guard below.
            //
            // Memory ordering: the in_flight bump and the subsequent
            // stopping load both use seq_cst so close_channel's symmetric
            // pair (stopping.store + in_flight.load) cannot Dekker-race
            // past us on weakly-ordered architectures (ARM Windows).
            self->in_flight.fetch_add(1, std::memory_order_seq_cst);
            struct exit_guard
            {
                impl *self;
                ~exit_guard()
                {
                    if (self->in_flight.fetch_sub(1, std::memory_order_acq_rel) == 1)
                    {
                        const std::lock_guard<std::mutex> lock{self->drain_mutex};
                        self->drain_cv.notify_all();
                    }
                }
            } guard{self};

            if (self->stopping.load(std::memory_order_seq_cst))
            {
                return;
            }

            // Filter ParameterNotification (just an admin status change with the same
            // address) — only AddInstance / DeleteInstance are real arrival/departures.
            if (type != MibAddInstance && type != MibDeleteInstance)
            {
                return;
            }

            if (self->callback)
            {
                self->callback();
            }
        }

        bool open_channel()
        {
            // initialNotification = FALSE: don't synthesise events for current state
            // at registration time; the coalescer captures the initial snapshot
            // separately.
            const DWORD result =
                ::NotifyUnicastIpAddressChange(AF_UNSPEC, &impl::on_address_change, this, FALSE, &notification_handle);
            return result == NO_ERROR;
        }

        void close_channel() noexcept
        {
            // 1. Tell the callback to skip work even if it manages to fire after
            //    we've returned from CancelMibChangeNotify2. seq_cst pairs with
            //    the callback-side load.
            stopping.store(true, std::memory_order_seq_cst);

            // 2. Ask Windows to stop scheduling new callbacks. Per MS docs, this
            //    does NOT synchronise with in-flight callbacks — those may still
            //    be running on other threads when this returns.
            if (notification_handle != nullptr)
            {
                ::CancelMibChangeNotify2(notification_handle);
                notification_handle = nullptr;
            }

            // 3. Wait for any in-flight callbacks to return. After this completes,
            //    `this` can be safely freed: the callback function pointer the OS
            //    has is unregistered, any callback that observed `context = self`
            //    has finished, and `stopping` blocks any further OS-scheduled
            //    invocation (which shouldn't happen after CancelMibChangeNotify2
            //    anyway, per MS docs — belt-and-braces).
            std::unique_lock<std::mutex> lock{drain_mutex};
            drain_cv.wait(lock, [this] { return in_flight.load(std::memory_order_seq_cst) == 0; });
        }
    };

    network_monitor::network_monitor(on_event_callback callback) : the_impl(std::make_unique<impl>())
    {
        the_impl->callback = std::move(callback);
        if (!the_impl->open_channel())
        {
            the_impl->callback = nullptr;
            throw std::runtime_error{"network_monitor: NotifyUnicastIpAddressChange failed"};
        }
    }

    network_monitor::~network_monitor()
    {
        if (!the_impl)
        {
            return;
        }
        the_impl->close_channel();
    }
}  // namespace provizio::dds::detail

#endif  // _WIN32
