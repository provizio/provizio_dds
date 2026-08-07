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
        HANDLE interface_notification_handle{nullptr};
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

        // Shared body of both OS callbacks: maintain the in-flight count that
        // close_channel drains on, honour the stopping flag, and forward to the user
        // callback. @p accept_parameter_notification is what differs between the two
        // subscriptions — see each callback's comment.
        static void dispatch(impl *self, MIB_NOTIFICATION_TYPE type, bool accept_parameter_notification)
        {
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

            const bool is_arrival_or_departure = type == MibAddInstance || type == MibDeleteInstance;
            if (!is_arrival_or_departure && !(accept_parameter_notification && type == MibParameterNotification))
            {
                return;
            }

            if (self->callback)
            {
                self->callback();
            }
        }

        static VOID NETIOAPI_API_ on_address_change(PVOID context, PMIB_UNICASTIPADDRESS_ROW row,
                                                    MIB_NOTIFICATION_TYPE type)
        {
            (void)row;
            // ParameterNotification on a unicast address is a lifetime / admin-status
            // change that leaves the address itself in place, so the snapshot could not
            // see it — only AddInstance / DeleteInstance are real arrivals/departures.
            dispatch(static_cast<impl *>(context), type, false);
        }

        static VOID NETIOAPI_API_ on_interface_change(PVOID context, PMIB_IPINTERFACE_ROW row,
                                                      MIB_NOTIFICATION_TYPE type)
        {
            (void)row;
            // Here ParameterNotification IS the one that matters: it is how Windows
            // reports an interface changing operational state (media connect /
            // disconnect), which moves it in or out of the OperStatus == IfOperStatusUp
            // set that capture_address_snapshot is built on. Without this subscription a
            // switch being powered on would produce no notification at all whenever the
            // adapter keeps its address across the outage.
            dispatch(static_cast<impl *>(context), type, true);
        }

        bool open_channel()
        {
            // initialNotification = FALSE on both: don't synthesise events for current
            // state at registration time; the coalescer captures the initial snapshot
            // separately.
            if (::NotifyUnicastIpAddressChange(AF_UNSPEC, &impl::on_address_change, this, FALSE,
                                               &notification_handle) != NO_ERROR)
            {
                return false;
            }

            if (::NotifyIpInterfaceChange(AF_UNSPEC, &impl::on_interface_change, this, FALSE,
                                          &interface_notification_handle) != NO_ERROR)
            {
                // Undo the address registration through the full close_channel path, not
                // a bare CancelMibChangeNotify2: an on_address_change dispatch may already
                // be running (the callback was installed before this call), and the
                // constructor is about to null `callback` and destroy `impl`. Only
                // close_channel sets `stopping` and then DRAINS the in-flight count, which
                // is what makes that destruction safe.
                close_channel();
                return false;
            }

            return true;
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
            if (interface_notification_handle != nullptr)
            {
                ::CancelMibChangeNotify2(interface_notification_handle);
                interface_notification_handle = nullptr;
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
            throw std::runtime_error{"network_monitor: NotifyUnicastIpAddressChange / NotifyIpInterfaceChange failed"};
        }
    }

    bool network_monitor::is_alive() const noexcept
    {
        // The OS owns the notification threads here: there is no worker that can die
        // out from under us, and a cancelled registration only ever happens as part of
        // destruction. So a constructed monitor is always alive.
        return the_impl != nullptr;
    }

    bool network_monitor::kill_for_test() noexcept
    {
        // Nothing to kill: see is_alive(). The monitor-revival test skips on Windows.
        return false;
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
