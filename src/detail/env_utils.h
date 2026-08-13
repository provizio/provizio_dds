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

#ifndef DDS_DETAIL_ENV_UTILS
#define DDS_DETAIL_ENV_UTILS

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>

// Shared helpers for reading configuration from environment variables — used by every
// PROVIZIO_DDS_* resolver so validation and log hygiene stay identical across them.

namespace provizio::dds::detail
{
    /**
     * @brief Caps and de-control-characters an environment variable value before quoting it in a log
     * message, so a pathological value can neither flood the log nor forge log lines in whatever
     * ingests them.
     *
     * @param raw The raw environment variable value
     * @return The value truncated to 32 characters (with a "..." suffix when longer), ASCII control
     * characters replaced with '?'
     */
    inline std::string sanitise_env_value_for_log(const std::string &raw)
    {
        // ASCII C0 controls are everything below the first printable character
        // (space); DEL sits just past the printable range.
        constexpr unsigned char first_printable_ascii = 0x20;
        constexpr unsigned char ascii_delete = 0x7F;
        constexpr std::size_t max_quoted_length = 32;

        std::string result = raw.substr(0, std::min(raw.size(), max_quoted_length));
        for (auto &chr : result)
        {
            const auto value = static_cast<unsigned char>(chr);
            if (value < first_printable_ascii || value == ascii_delete)
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
     * @brief Parses a string as a uint32 (zero accepted): leading whitespace and a single leading
     * '+', then ASCII digits to end-of-string.
     *
     * strtoull alone is NOT a safe validator here: it accepts a leading '-' and negates in unsigned
     * arithmetic, so e.g. "-18446744073709551615" would wrap around to 1 and pass a positive-range
     * check. The explicit digit check after the optional sign rejects any minus. Mirrors
     * _parse_u32 in python/shm_cleanup.py so both languages treat the same environment values as
     * valid.
     *
     * @param text The string to parse (must not be nullptr)
     * @param value Receives the parsed value; left untouched when the input is not a valid uint32
     * @return Whether @p text is a valid uint32
     */
    inline bool try_parse_u32(const char *text, std::uint32_t &value)
    {
        const char *cursor = text;
        while (std::isspace(static_cast<unsigned char>(*cursor)) != 0)
        {
            ++cursor;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic): C-string scan
        }
        if (*cursor == '+')
        {
            ++cursor;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic): C-string scan
        }
        if (std::isdigit(static_cast<unsigned char>(*cursor)) == 0)
        {
            return false;
        }
        char *end = nullptr;
        const std::uint64_t parsed = std::strtoull(cursor, &end, 10);  // NOLINT: C numeric parse
        if (end == nullptr || *end != '\0' ||
            parsed > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))
        {
            return false;
        }
        value = static_cast<std::uint32_t>(parsed);
        return true;
    }

    /**
     * @brief Parses a string as a strictly-positive uint32, using the same grammar as
     * @c try_parse_u32.
     *
     * Zero shares the invalid return value: every caller treats it as "not configured" and falls
     * back to its own default, so the two cases need no distinction. Mirrors _parse_positive_u32 in
     * python/provizio_dds.py.
     *
     * @param text The string to parse (must not be nullptr)
     * @return The parsed value, or 0 when the input is not a strictly-positive uint32
     */
    inline std::uint32_t parse_positive_u32(const char *text)
    {
        std::uint32_t value = 0;
        if (!try_parse_u32(text, value))
        {
            return 0;
        }
        return value;
    }
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_ENV_UTILS
