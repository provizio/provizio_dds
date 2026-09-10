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

#ifndef DDS_LOGGING
#define DDS_LOGGING

#include <functional>
#include <sstream>
#include <string>
#include <string_view>

#include "provizio/dds/common.h"

namespace provizio::dds
{
    /**
     * @file logging.h
     * @brief Customer-configurable logging for provizio_dds internals.
     *
     * By default, log messages are emitted to @c std::cout (info / warning) or
     * @c std::cerr (error) with a @c "[provizio_dds]" prefix. Consumers that want
     * to route into their own logging stack install a custom callback via
     * @c set_log_callback.
     *
     * Logging is thread-safe: concurrent emissions from any provizio_dds thread
     * share the installed callback via a read-write lock.
     */

    /**
     * @brief Severity level for log messages emitted from provizio_dds.
     */
    enum class log_level
    {
        /// Something happened that affects the caller, but nothing is wrong: a network change
        /// that rebuilt the participants (which briefly interrupts communication), or -- once
        /// per distinct set, at participant creation on a host with a tunnel up -- which VPN /
        /// tunnel interfaces the transports leave out and whether netmask filtering came with
        /// it (see "VPN and tunnel interfaces" in DETAILS.md).
        info,
        /// The caller should act: a rejected @c PROVIZIO_DDS_* value, a host limit the
        /// library cannot work around on its own (capped socket buffers, a full /dev/shm),
        /// or a requested @c transport_mode the participant could not honour because the
        /// transports are the caller's.
        warning,
        /// Functionality was lost: a participant that could not be created or rebuilt, a
        /// monitor that could not start, or an exception thrown out of a caller's callback.
        error,
    };

    // Note on what is NOT logged: provizio_dds stays silent about its own internals — start-up
    // state, successful operations, and anomalies it handled itself (a coalesced network event
    // that changed nothing, a retried internal fallback). A healthy process produces no output
    // about them, so anything that does appear is worth reading; what a healthy process CAN
    // say is limited to the info lines above, and a callback that forwards every line to an
    // alerting channel should expect the VPN-exclusion report at start-up on every host with a
    // tunnel up.

    /**
     * @brief Callback signature for custom log emitters.
     *
     * @param level    severity of the message.
     * @param message  fully formatted, single-line text. No trailing newline. No
     *                 @c "[provizio_dds]" prefix — add your own if desired.
     *
     * May be invoked from any thread, including the network-monitor worker thread,
     * the coalescer thread, and a participant's reset path. Implementations should
     * be brief and reentrant; do any heavy work in their own background thread.
     *
     * A callback must not emit a provizio_dds log line of its own, and that rules out more
     * than it first appears. Calling @c log_info / @c log_warning / @c log_error directly is
     * the obvious half; the other half is any provizio_dds call that logs, and
     * @c make_domain_participant is one -- it can warn about the domain's UDP ports, report
     * the VPN / tunnel interfaces it excluded, or report a monitor that failed to start. The
     * callback is invoked from the log stream's own destructor, so either route re-enters it
     * with no depth limit and overflows the stack. That failure is a crash, not an exception,
     * so none of the library's exception guards can absorb it.
     *
     * Write to your own sink instead -- a queue, a file, a socket -- and do anything that
     * might log on a thread of your own, outside the callback. Publishing a log line onto a
     * DDS topic remains supported and expected, on entities that ALREADY exist: publishing
     * does not log, and every diagnostic but one is emitted with no lifecycle lock held.
     *
     * Creating or destroying a publisher, subscriber or service from the callback is
     * separately forbidden, for a second reason. The listener-drain stall warning (see
     * detail/listener_drain.h) is the one diagnostic emitted while the participant's
     * endpoint-registration lock is held -- it reports a user data callback that has stopped
     * returning, an unbounded stall, so deferring it until the lock is released would mean
     * never emitting it at all -- and creating or destroying an endpoint takes that same
     * lock. A callback that does it on receiving that warning deadlocks on the calling
     * thread.
     */
    using log_callback = std::function<void(log_level level, std::string_view message)>;

    /**
     * @brief Install a custom log callback for all subsequent log emissions from
     * provizio_dds. Pass a default-constructed (empty) callback to restore the
     * built-in stdout/stderr emitter.
     *
     * @return The previously-installed callback (empty if the default emitter was
     * in use).
     *
     * Safe to call from any thread. Any log emission already in flight finishes
     * with the previous callback; subsequent emissions use the new one.
     */
    PROVIZIO_DDS_API log_callback set_log_callback(log_callback callback);

    namespace detail
    {
        /**
         * @brief Emits one already-composed log line, and says whether it got out.
         *
         * The emission half of @c log_stream, split out because a caller that suppresses
         * repeated reports has to know whether the last one reached the operator. Composing a
         * line and having it silently discarded looks identical to composing and emitting it
         * from the outside -- @c log_stream's destructor swallows an emission failure, as a
         * destructor must -- and a suppressor keyed on that would go quiet after a report
         * nobody saw.
         *
         * @param level Severity
         * @param message The fully composed line, without the "[provizio_dds] " prefix
         * @return Whether the message reached the callback (or the default emitter) without
         * an exception. False means nothing was reported.
         */
        PROVIZIO_DDS_API bool emit_log_line(log_level level, const std::string &message) noexcept;

        /**
         * @brief Streaming-style log message builder; emits when destroyed.
         *
         * @code
         *   provizio::dds::log_info() << "monitor enabled (" << n << " interfaces)";
         * @endcode
         *
         * Non-copyable, non-movable — designed for use as a temporary that lives
         * for the duration of a single statement.
         */
        // No class-level PROVIZIO_DDS_API: the class has an
        // std::ostringstream member, and class-level dllexport on MSVC
        // triggers C4251 for it and binds consumers to the exact same
        // MSVC CRT/STL build. Per-method PROVIZIO_DDS_API on the
        // out-of-line ctor/dtor (the only symbols that cross the DLL
        // boundary) achieves the same export without that liability;
        // operator<< stays inline as a template.
        class log_stream
        {
          public:
            PROVIZIO_DDS_API explicit log_stream(log_level level) noexcept;
            PROVIZIO_DDS_API ~log_stream();

            log_stream(const log_stream &) = delete;
            log_stream &operator=(const log_stream &) = delete;
            log_stream(log_stream &&) = delete;
            log_stream &operator=(log_stream &&) = delete;

            template <typename T> log_stream &operator<<(const T &value)
            {
                buffer << value;
                return *this;
            }

          private:
            log_level level;
            std::ostringstream buffer;
        };
    }  // namespace detail

    inline detail::log_stream log_info() noexcept
    {
        return detail::log_stream{log_level::info};
    }

    inline detail::log_stream log_warning() noexcept
    {
        return detail::log_stream{log_level::warning};
    }

    inline detail::log_stream log_error() noexcept
    {
        return detail::log_stream{log_level::error};
    }

}  // namespace provizio::dds

#endif  // DDS_LOGGING
