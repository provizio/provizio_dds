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
#include <utility>

#include "detail/env_utils.h"
#include "detail/immortal.h"

namespace provizio::dds::detail
{
    namespace
    {
        constexpr const char *extra_interfaces_env_var_name = "PROVIZIO_DDS_NETWORK_RECOVERY_EXTRA_INTERFACES";

        // Reads the comma-separated list once. Splitting and trimming is
        // split_comma_separated's job, shared with every other PROVIZIO_DDS_* list so the
        // two cannot drift apart on what counts as an entry.
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

            for (auto &entry : split_comma_separated(std::string{raw}))
            {
                names.insert(std::move(entry));
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
        // Parsed on first use, then immutable — so the snapshot filters stay consistent
        // for the process' lifetime and repeated captures (one per coalesced burst and one
        // per safety-net tick) cost no getenv().
        //
        // Immortal, not a plain static: the network-recovery threads read this set until
        // the coordinator singleton is destroyed at process exit, and a plain static first
        // constructed by the first snapshot -- after that singleton -- would be destroyed
        // before it. See detail/immortal.h for the full account.
        static const immortal<std::unordered_set<std::string>> names{parse_extra_interfaces_once()};
        return *names;
    }
}  // namespace provizio::dds::detail
