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

#include "provizio/dds/logging.h"

#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <utility>

namespace provizio::dds
{
    namespace
    {
        // Single per-process callback slot. Reads (log emissions) acquire shared;
        // writes (set_log_callback) acquire exclusive. The slot is empty by default;
        // the default emitter is implemented inline rather than installed as a real
        // callback so a consumer that wants the default behaviour back can simply
        // pass an empty std::function.
        // The callback slot is by definition process-wide mutable state; the
        // whole point of @c set_log_callback is to let consumers swap the
        // installed emitter at runtime. Making the holders const would defeat
        // that. NOLINT-ed individually rather than carving out a separate
        // header just to host them.
        std::shared_mutex callback_mutex;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
        log_callback active_callback;      // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

    }  // namespace

    log_callback set_log_callback(log_callback callback)
    {
        const std::unique_lock<std::shared_mutex> lock{callback_mutex};
        log_callback previous = std::move(active_callback);
        active_callback = std::move(callback);
        return previous;
    }

    namespace detail
    {
        log_stream::log_stream(const log_level level) noexcept : level(level)
        {
        }

        log_stream::~log_stream()
        {
            // Whole destructor body in a single try/catch: every operation
            // below — buffer.str() (allocates), copying the std::function
            // snapshot (allocates for non-SBO captures), and the default-emitter
            // stream insertions — can throw bad_alloc or an I/O exception. A
            // destructor that escapes with an exception terminates the process,
            // so we swallow everything here. The catch is intentionally empty —
            // there is no safe recovery path for a logger that itself failed
            // (recursing into log_error would re-enter this very destructor).
            try
            {
                const std::string message = buffer.str();

                // Snapshot the callback under the shared lock so we don't call into
                // user code while holding the lock (avoids unexpected deadlocks if the
                // callback in turn logs from another thread).
                log_callback snapshot;
                {
                    const std::shared_lock<std::shared_mutex> lock{callback_mutex};
                    snapshot = active_callback;
                }

                if (snapshot)
                {
                    snapshot(level, message);
                    return;
                }

                // Default emitter: info / warning -> stdout, error -> stderr.
                //
                // Flushed per line, not left to the stream's own buffering. std::cout is
                // fully buffered whenever it is not a terminal -- which is every CI run,
                // where it is a pipe -- so a process that is killed (a test timing out) or
                // that dies loses whatever it had buffered. The messages that matter most
                // are exactly the ones emitted while something is going wrong, so a
                // diagnostic that only survives a clean exit is not a diagnostic. Logging
                // is rare and already string-formatting per call, so the flush costs
                // nothing that matters.
                auto &stream = (level == log_level::error) ? std::cerr : std::cout;
                stream << "[provizio_dds] " << message << '\n' << std::flush;
            }
            catch (...)  // NOLINT(bugprone-empty-catch): see comment above.
            {
            }
        }
    }  // namespace detail
}  // namespace provizio::dds
