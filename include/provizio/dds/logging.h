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
        /// Something happened that affects the caller, but nothing is wrong — currently only a
        /// network change that rebuilt the participants, which briefly interrupts communication.
        info,
        /// The caller should act: a rejected @c PROVIZIO_DDS_* value, or a host limit the
        /// library cannot work around on its own (capped socket buffers, a full /dev/shm).
        warning,
        /// Functionality was lost: a participant that could not be created or rebuilt, a
        /// monitor that could not start, or an exception thrown out of a caller's callback.
        error,
    };

    // Note on what is NOT logged: provizio_dds stays silent about its own internals — start-up
    // state, successful operations, and anomalies it handled itself (a coalesced network event
    // that changed nothing, a retried internal fallback). A healthy process produces no output
    // at all, so anything that does appear is worth reading.

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
