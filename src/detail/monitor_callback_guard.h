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

#ifndef DDS_DETAIL_MONITOR_CALLBACK_GUARD
#define DDS_DETAIL_MONITOR_CALLBACK_GUARD

#include <atomic>
#include <exception>
#include <string>
#include <utility>

#include "provizio/dds/detail/log_nothrow.h"
#include "provizio/dds/logging.h"

namespace provizio::dds::detail
{
    /**
     * @brief Emits one report unless this streak of failures has already produced one, and
     * keeps the streak open if the report did not actually reach the operator.
     *
     * Two traps here, both hit on the way to this shape.
     *
     * Latching on the ATTEMPT is the first: the first report failing would then silence every
     * later one for the rest of the streak, and what makes a report fail -- memory pressure --
     * is the same condition that makes the callback fail, so the two arrive together. The
     * result was a monitor dropping every interface-change event with not one line to say so,
     * and with the safety-net tick disabled
     * (PROVIZIO_DDS_NETWORK_RECOVERY_SAFETY_NET_SEC=0, which the library's own tests use) that
     * is network auto-recovery silently dead.
     *
     * Latching on "composing the line did not throw" is the second, and looks like a fix
     * without being one: log_stream's destructor swallows an emission failure, as a destructor
     * must, so a line discarded inside it is indistinguishable from a line delivered. That is
     * why this goes through emit_log_line, which reports whether the message reached the
     * callback, rather than through the streaming form.
     *
     * @param failure_reported The streak latch. Claimed by a compare-exchange so two threads
     * cannot both report (Windows delivers interface-change notifications on more than one
     * thread-pool thread), and released again when the claimed report did not get out.
     * @param compose Builds the message. May throw -- composition allocates, and this is
     * called while reporting a failure that may itself be an allocation failure; a throw is
     * caught and treated as "not delivered".
     */
    template <typename compose_function>
    void report_once_per_streak(std::atomic<bool> &failure_reported, compose_function &&compose) noexcept
    {
        bool unreported = false;
        if (!failure_reported.compare_exchange_strong(unreported, true, std::memory_order_relaxed))
        {
            return;
        }

        bool delivered = false;
        emit_log_nothrow([&delivered, &compose] { delivered = emit_log_line(log_level::error, compose()); });
        if (!delivered)
        {
            failure_reported.store(false, std::memory_order_relaxed);
        }
    }

    /**
     * @brief Calls @p callback on a network-monitor thread, letting nothing escape and
     * reporting a failure once per streak.
     *
     * Every backend's read loop invokes the change callback with no exception boundary between
     * it and the thread's entry point, so an escape is @c std::terminate -- and on Windows the
     * frame above is the OS's @c NotifyIpInterfaceChange, which makes an escape worse than a
     * terminate. The callback reads the host's interfaces and allocates as it goes, so it can
     * throw @c bad_alloc under exactly the memory pressure a network change is capable of
     * coinciding with. A dropped event costs one missed wake-up, which the periodic safety-net
     * tick then catches.
     *
     * Reported rather than swallowed: a network change the library never acts on until the next
     * safety-net tick is precisely the symptom someone would come to the log to explain, and the
     * Python mirrors log their equivalents (@c _PollingNetworkMonitor / @c
     * _NetlinkNetworkMonitor in python/network_recovery.py).
     *
     * Once per STREAK, not once per event: this sits in the kernel-event read loop, one call per
     * netlink batch rather than one per timer tick, so a persistent failure across a flapping
     * link would otherwise emit hundreds of allocating log lines a second -- worsening the very
     * memory pressure being reported. The same discipline the coordinator applies to its
     * unreadable-interface warning. @p failure_reported is the streak latch, cleared by the
     * first call that succeeds.
     *
     * The Python mirror has no counterpart to the latch because it has no counterpart to this
     * rate: python/network_recovery.py coalesces inside its own read loop and calls its
     * handler once per settled burst and once per safety-net tick, seconds apart, where this
     * runs once per netlink batch. It sanitises the same text for the same reason.
     *
     * The report goes through @c emit_log_nothrow and the text through @c sanitise_text_for_log:
     * composing the line allocates and calls the user's log callback, so reporting must not
     * become the escape route it exists to close, and @c what() is neither ours nor necessarily
     * ASCII (glibc translates @c std::system_error through the active locale, and
     * @c network_monitor's callback is public API, so its text can carry anything -- newlines
     * included).
     *
     * @param callback The change callback to invoke; may be empty, in which case nothing happens
     * @param failure_reported Streak latch, owned by the calling monitor
     */
    template <typename callback_type>
    void invoke_monitor_callback(const callback_type &callback, std::atomic<bool> &failure_reported) noexcept
    {
        if (!callback)
        {
            return;
        }

        try
        {
            callback();
            failure_reported.store(false, std::memory_order_relaxed);
        }
        catch (const std::exception &exception)
        {
            report_once_per_streak(failure_reported, [&exception] {
                return "network_monitor: interface-change callback threw: " +
                       sanitise_text_for_log(exception.what(), max_logged_exception_text) +
                       " (further failures in this streak are not repeated)";
            });
        }
        catch (...)
        {
            report_once_per_streak(failure_reported, [] {
                return std::string{"network_monitor: interface-change callback threw a non-std exception "
                                   "(further failures in this streak are not repeated)"};
            });
        }
    }
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_MONITOR_CALLBACK_GUARD
