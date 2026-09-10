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

#ifndef DDS_DETAIL_LOG_NOTHROW
#define DDS_DETAIL_LOG_NOTHROW

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>

#include "provizio/dds/logging.h"

// Reporting from a context where an escaping exception would end the process, and the
// text-hygiene that reporting externally-supplied text needs. Internal to provizio_dds;
// installed only because the header-only publisher / subscriber / request-response
// templates report from exactly such contexts.

namespace provizio::dds::detail
{
    /// Default cap for a value quoted into a log line: long enough to identify an
    /// environment variable's value, short enough that a pathological one cannot flood the log.
    constexpr std::size_t default_quoted_log_length = 32;

    /// How much of a throwing callback's what() text a log line keeps. Longer than the
    /// environment-value cap because an exception message is the whole diagnostic rather than
    /// an identifier -- and still bounded, because the text is not ours.
    constexpr std::size_t max_logged_exception_text = 200;

    /**
     * @brief Caps text and reduces it to printable ASCII before quoting it in a log message, so
     * externally-supplied text can neither flood the log, nor forge log lines in whatever
     * ingests them, nor make the line it appears on unprintable on a non-UTF-8 console.
     *
     * Takes a @c string_view and truncates BEFORE folding, so the work and the allocation are
     * both bounded by @p max_quoted_length rather than by the input. That matters most where
     * this is called from: a handler reporting a @c bad_alloc must not begin by allocating a
     * copy of a caller-sized string.
     *
     * @param raw The raw text
     * @param max_quoted_length How many characters to keep before truncating
     * @return The text truncated to @p max_quoted_length characters (with a "..." suffix when
     * longer), every byte outside printable ASCII [0x20, 0x7F) -- C0 controls, DEL, and
     * everything from 0x80 up -- replaced with '?'
     */
    inline std::string sanitise_text_for_log(const std::string_view raw, const std::size_t max_quoted_length)
    {
        // Printable ASCII is [0x20, 0x7F): the C0 controls sit below it and DEL at its top,
        // and everything from 0x80 up is outside it altogether.
        constexpr unsigned char first_printable_ascii = 0x20;
        constexpr unsigned char ascii_delete = 0x7F;

        std::string result{raw.substr(0, std::min(raw.size(), max_quoted_length))};
        for (auto &chr : result)
        {
            const auto value = static_cast<unsigned char>(chr);
            // Everything outside printable ASCII, not only the C0 controls and DEL. Bytes at
            // 0x80 and above used to pass through, which was survivable while every caller fed
            // this an environment value -- but interface identities reach it now, and a Windows
            // adapter's friendly name is administrator-settable arbitrary Unicode. On Windows
            // the Python mirror prints through a cp1252 stdout, where a non-ASCII byte raises
            // UnicodeEncodeError inside a swallowing except, so the whole warning DISAPPEARS
            // rather than merely mis-rendering. Replacing here also removes the second half of
            // that hazard: the cap is a byte count, so a multi-byte sequence straddling it was
            // truncated into an invalid one.
            //
            // A std::exception's what() is the other caller, and it is neither ours nor
            // necessarily ASCII: glibc translates std::system_error's text through the active
            // locale, and a user callback's exception text is whatever the user threw. Folding
            // the C0 controls is what stops it from FORGING a log line -- a what() carrying a
            // newline and a plausible "[provizio_dds] ... succeeded" would otherwise reach the
            // operator as a second, entirely fabricated line, and an ESC would reach their
            // terminal as a control sequence.
            if (value < first_printable_ascii || value >= ascii_delete)
            {
                chr = '?';
            }
        }
        if (raw.size() > max_quoted_length)
        {
            result += "...";
        }
        return result;
    }

    /**
     * @brief @c sanitise_text_for_log for a C string, treating @c nullptr as empty.
     *
     * Its reason for existing is @c std::exception::what(). Constructing a @c string_view from
     * a null pointer is undefined behaviour, and while the standard requires @c what() to
     * return a non-null NTBS, a consumer-derived exception type is free to be wrong about that
     * -- and this is called from the handlers whose whole job is surviving whatever a consumer
     * threw. A crash inside the reporting of a crash is the worst place to learn it.
     *
     * @param raw The raw text, or @c nullptr
     * @param max_quoted_length How many characters to keep before truncating
     * @return The sanitised, capped text; "(null)" when @p raw is @c nullptr
     */
    inline std::string sanitise_text_for_log(const char *const raw, const std::size_t max_quoted_length)
    {
        if (raw == nullptr)
        {
            return "(null)";
        }
        return sanitise_text_for_log(std::string_view{raw}, max_quoted_length);
    }

    /**
     * @brief @c sanitise_text_for_log at the default cap.
     *
     * Named for environment variables because that was its first caller; it takes any
     * externally-supplied identifier quoted into a log line, an OS-supplied interface or
     * adapter name included. Exception text uses @c sanitise_text_for_log directly, with the
     * longer @c max_logged_exception_text cap -- a message is the whole diagnostic, where these
     * are identifiers.
     *
     * @param raw The raw text
     * @return The sanitised, capped value
     */
    inline std::string sanitise_env_value_for_log(const std::string_view raw)
    {
        return sanitise_text_for_log(raw, default_quoted_log_length);
    }

    /**
     * @brief Runs @p emit -- a callable that composes and emits one log line -- swallowing
     * anything it throws, and reporting whether it got through.
     *
     * For the last-resort handlers on threads with no exception boundary above them: a Fast-DDS
     * listener callback, a monitor thread's read loop, an OS-owned callback frame, the
     * coalescer's catch-all around a reset pass. Logging there is not free of the hazard it is
     * reporting, so a bad_alloc reported by one of those handlers could otherwise escape the
     * handler through its own log line and call std::terminate -- the exact outcome the handler
     * exists to prevent, reached by the one path nobody looks at.
     *
     * What it actually catches is narrower than "logging", and worth stating precisely because
     * the obvious answer is wrong. @c log_stream's destructor already wraps its whole body --
     * the @c str() copy, the callback snapshot and the user callback itself -- in its own
     * catch-all, and its constructor is @c noexcept, so neither the emission nor a throwing
     * user callback can reach here. What reaches here is everything evaluated to BUILD the
     * line: the sanitiser's allocation, a @c std::to_string, a concatenation, a consumer's own
     * @c operator<< on a streamed value -- plus @c ostringstream growth inside @c operator<<.
     *
     * The empty catch is deliberate and is confined to this function. There is genuinely
     * nowhere left to report to: the reporting channel is what just failed.
     *
     * @param emit A callable composing and emitting the message, e.g.
     * @code
     *   emit_log_nothrow([&] { log_error() << "callback threw: " << sanitised; });
     * @endcode
     * @return Whether @p emit ran to completion. Callers that suppress repeats must key the
     * suppression on this rather than on having attempted a report -- otherwise the FIRST
     * report failing silences every later one, and the failure this reports is exactly the
     * condition (memory pressure) that makes the report fail.
     *
     */
    template <typename emit_function> bool emit_log_nothrow(emit_function &&emit) noexcept
    {
        try
        {
            emit();
            return true;
        }
        catch (...)
        {
            return false;
        }
    }
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_LOG_NOTHROW
