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

#include "provizio/dds/detail/address_snapshot.h"

#include <cstdlib>
#include <string>
#include <unordered_set>

namespace provizio::dds::detail
{
    namespace
    {
        constexpr const char *extra_interfaces_env_var_name = "PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES";

        // Splits the comma-separated list once, trimming surrounding whitespace off each
        // entry so that "br0, virbr2" behaves like "br0,virbr2". Empty entries (a trailing
        // comma, "a,,b") are skipped rather than turned into an unmatchable "" name.
        std::unordered_set<std::string> parse_extra_interfaces_once()
        {
            std::unordered_set<std::string> names;

            // Startup-only probe, same as the PROVIZIO_DDS_NETWORK_RECOVERY read in
            // network_recovery.cpp.
            const auto *raw = std::getenv(extra_interfaces_env_var_name);  // NOLINT(concurrency-mt-unsafe)
            if (raw == nullptr || *raw == '\0')
            {
                return names;
            }

            const std::string value{raw};
            std::string::size_type start = 0;
            while (start <= value.size())
            {
                const auto comma = value.find(',', start);
                const auto end = (comma == std::string::npos) ? value.size() : comma;

                auto first = start;
                auto last = end;
                const auto is_space = [&value](const std::string::size_type index) {
                    const auto chr = static_cast<unsigned char>(value[index]);
                    return chr == ' ' || chr == '\t' || chr == '\r' || chr == '\n';
                };
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
                    names.insert(value.substr(first, last - first));
                }

                if (comma == std::string::npos)
                {
                    break;
                }
                start = comma + 1;
            }

            // Deliberately silent. This runs inside the one-shot initializer of
            // force_included_interfaces(), which capture_address_snapshot() calls — and
            // the coordinator calls THAT while holding registry_mutex and monitor_mutex.
            // A log line here would invoke the user's log callback under those locks; a
            // callback that constructs another domain_participant (a documented,
            // supported pattern) would then deadlock re-entering register_participant,
            // and would also re-enter this very function-local static initializer, which
            // is undefined behaviour. The coordinator reports the force-included set from
            // its own after-the-locks log instead.
            return names;
        }
    }  // namespace

    const std::unordered_set<std::string> &force_included_interfaces()
    {
        // Meyers' singleton: parsed on first use, then immutable — so the snapshot
        // filters stay consistent for the process' lifetime and repeated captures
        // (one per coalesced burst and one per safety-net tick) cost no getenv().
        static const std::unordered_set<std::string> names = parse_extra_interfaces_once();
        return names;
    }
}  // namespace provizio::dds::detail
