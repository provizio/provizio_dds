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
#include <string_view>
#include <vector>

// Shared helpers for reading configuration from environment variables — used by every
// PROVIZIO_DDS_* resolver so validation and log hygiene stay identical across them —
// together with the small text utilities that parsing those values, and matching names
// against them, need on both sides.

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
        // Printable ASCII is [0x20, 0x7F): the C0 controls sit below it and DEL at its top,
        // and everything from 0x80 up is outside it altogether.
        constexpr unsigned char first_printable_ascii = 0x20;
        constexpr unsigned char ascii_delete = 0x7F;
        constexpr std::size_t max_quoted_length = 32;

        std::string result = raw.substr(0, std::min(raw.size(), max_quoted_length));
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

    /**
     * @brief Whether @p text begins with @p prefix.
     *
     * C++17 has no @c std::string_view::starts_with -- that arrived in C++20 -- and the
     * ad-hoc forms its absence invites differ in whether they guard the length at all: a
     * size compare against a constructed @c string_view in one place, a bare
     * @c compare(0, n, prefix) in another. Stating it once is what keeps every prefix test
     * over an interface name or an adapter description answering the same question.
     *
     * @param text The text to test
     * @param prefix The prefix to look for; an empty prefix matches anything
     * @return @c true when @p text is at least as long as @p prefix and begins with it
     */
    inline bool starts_with(std::string_view text, std::string_view prefix) noexcept
    {
        return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
    }

    /**
     * @brief Whether @p text is an IPv6 address literal rather than an IPv4 one or a device
     * name.
     *
     * A colon is the whole test, and it is enough for every text this library classifies:
     * the sources are @c getifaddrs / @c GetAdaptersAddresses output and
     * @c IPFinder::info_IP::name, all of which are bare @c inet_ntop presentation forms, and
     * neither an IPv4 literal nor any platform's device name may contain one. Stated once
     * because the rule is one rule: a zone-scoped form (@c fe80::1%eth0) or a bracketed one
     * (@c [::1]:7400) reaching any of these paths would need the same answer in all of them,
     * and three hand-written copies are how one of them would come to disagree.
     *
     * @param text The address text to test
     * @return @c true when @p text is an IPv6 literal
     */
    inline bool is_ipv6_address_text(std::string_view text) noexcept
    {
        return text.find(':') != std::string_view::npos;
    }

    /**
     * @brief Lower-cases the ASCII letters of @p text, leaving every other byte as it is.
     *
     * ASCII-only on purpose, and not @c std::tolower: everything this is used on —
     * interface names, adapter descriptions, on/off keywords — is ASCII by construction on
     * every platform supported, while @c std::tolower answers to the process' locale, so
     * the same value could classify differently in two processes on one host (Turkish
     * locales famously map 'I' outside ASCII).
     *
     * @param text The text to lower-case
     * @return A lower-cased copy
     */
    inline std::string to_lower_ascii(std::string_view text)
    {
        std::string lowered{text};
        std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](const char chr) {
            const auto uchr = static_cast<unsigned char>(chr);
            return static_cast<char>((uchr >= 'A' && uchr <= 'Z') ? uchr - 'A' + 'a' : uchr);
        });
        return lowered;
    }

    /**
     * @brief Splits a comma-separated environment value into its entries, trimming ASCII
     * whitespace around each so that @c "br0, virbr2" behaves like @c "br0,virbr2".
     *
     * Empty entries are dropped rather than returned as an empty string: a trailing comma
     * or an @c "a,,b" typo would otherwise contribute a name that can never match anything
     * — and, where the entries are matched against interface names, one that would be
     * indistinguishable from a platform reporting no name at all.
     *
     * Shared by every PROVIZIO_DDS_* variable that takes a list, so they cannot drift apart
     * on what counts as an entry.
     *
     * @param value The raw environment value
     * @return The trimmed, non-empty entries, in the order they appeared
     */
    inline std::vector<std::string> split_comma_separated(const std::string &value)
    {
        std::vector<std::string> entries;

        const auto is_space = [&value](const std::string::size_type index) {
            const auto chr = static_cast<unsigned char>(value[index]);
            return chr == ' ' || chr == '\t' || chr == '\r' || chr == '\n';
        };

        std::string::size_type start = 0;
        while (start <= value.size())
        {
            const auto comma = value.find(',', start);
            const auto end = (comma == std::string::npos) ? value.size() : comma;

            auto first = start;
            auto last = end;
            while (first < last && is_space(first))
            {
                ++first;
            }
            while (last > first && is_space(last - 1))
            {
                --last;
            }
            if (last > first)
            {
                entries.push_back(value.substr(first, last - first));
            }

            if (comma == std::string::npos)
            {
                break;
            }
            start = comma + 1;
        }

        return entries;
    }

    /**
     * @brief Parses an on/off environment value: @c on / @c 1 / @c true / @c yes and
     * @c off / @c 0 / @c false / @c no, case-insensitively.
     *
     * Shared so that every PROVIZIO_DDS_* switch accepts exactly the same spellings — a
     * variable that quietly took only a subset would look ignored to whoever spelled it the
     * other way.
     *
     * @param raw The raw environment value (must not be nullptr)
     * @param value Receives the parsed value; left untouched when the input is not
     * recognised, so the caller's default survives
     * @return Whether @p raw is a recognised on/off spelling
     */
    inline bool try_parse_bool(const char *raw, bool &value)
    {
        const std::string lowered = to_lower_ascii(raw);
        if (lowered == "off" || lowered == "0" || lowered == "false" || lowered == "no")
        {
            value = false;
            return true;
        }
        if (lowered == "on" || lowered == "1" || lowered == "true" || lowered == "yes")
        {
            value = true;
            return true;
        }
        return false;
    }
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_ENV_UTILS
