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

#include "provizio/dds/network_recovery.h"

#include <cstdlib>
#include <string>

#include "detail/env_utils.h"
#include "provizio/dds/logging.h"

namespace provizio::dds
{
    namespace
    {
        constexpr const char *env_var_name = "PROVIZIO_DDS_NETWORK_RECOVERY";

        /// Everything this process ever concludes about PROVIZIO_DDS_NETWORK_RECOVERY.
        struct env_var_reading final
        {
            /// Whether the variable was set to something non-empty.
            bool explicitly_set{false};
            /// The verdict: what the value says, or the default where it says nothing usable.
            bool enabled{true};
        };

        /// The env var, read and parsed exactly ONCE per process.
        ///
        /// One initialiser holding both answers, not two magic statics holding one each. Two
        /// were primed at different moments -- the verdict from every participant's
        /// constructor, "was it set" only from the loopback-confinement path -- so a
        /// setenv() between them left the pair describing two different environments, and
        /// "off" was reported as an explicit request for ON. No concurrency was needed to
        /// reach it: two sequential participant constructions with a setenv in between were
        /// enough. Reading them together is what makes the NOLINT below honest, too.
        const env_var_reading &read_env_var_once()
        {
            static const env_var_reading reading = [] {
                env_var_reading result;
                const auto *raw = std::getenv(env_var_name);  // NOLINT(concurrency-mt-unsafe): startup-only probe
                result.explicitly_set = raw != nullptr && *raw != '\0';
                if (!result.explicitly_set)
                {
                    return result;  // Default: enabled.
                }

                bool enabled = true;
                if (detail::try_parse_bool(raw, enabled))
                {
                    result.enabled = enabled;
                    return result;
                }

                // Unknown value: log once, treat as default-on.
                // Sanitised before quoting, as everywhere else a rejected PROVIZIO_DDS_* value
                // is echoed: an arbitrarily long or control-character-carrying value must not be
                // able to flood the log or forge lines in whatever ingests it.
                log_warning() << env_var_name << "='" << detail::sanitise_env_value_for_log(raw)
                              << "' is not recognised (use on/off); auto-recovery enabled";
                return result;
            }();
            return reading;
        }
    }  // namespace

    bool network_recovery_explicitly_requested(const network_recovery_mode mode)
    {
        if (mode == network_recovery_mode::on)
        {
            return true;
        }
        if (mode != network_recovery_mode::env_var_controlled)
        {
            return false;
        }
        // Both halves out of ONE reading, never a fresh getenv AND a cached verdict: asking
        // whether the variable is set right now and AND-ing that with an answer cached earlier
        // let the two describe different environments, and "off" was then reported as an
        // explicit request for ON -- watching a loopback-confined participant against the
        // caller's word. Mid-process mutation is not hypothetical here; a Python participant
        // in a mixed application does exactly that through os.environ.setdefault.
        const auto &reading = read_env_var_once();
        return reading.explicitly_set && reading.enabled;
    }

    bool resolve_network_recovery_enabled(const network_recovery_mode mode)
    {
        switch (mode)
        {
        case network_recovery_mode::on:
            return true;
        case network_recovery_mode::off:
            return false;
        case network_recovery_mode::env_var_controlled:
            return read_env_var_once().enabled;
        }
        return true;
    }
}  // namespace provizio::dds

// Forward-declaration for the otherwise-internal Fast-DDS function. Matches the
// definition in src/cpp/utils/SystemInfo.hpp of the bundled Fast-DDS. We avoid
// including that header directly because it lives under Fast-DDS's internal
// source layout (not under its installed include tree); a forward declaration
// keeps the build's include path matrix simple AND avoids picking up further
// SystemInfo internals as part of provizio_dds's translation unit.
//
// On Linux / macOS the symbol is in libfastdds.{so,dylib}'s dynamic symbol
// table by default (no explicit visibility hiding). On Windows SystemInfo
// carries no FASTDDS_EXPORTED_API decoration in Fast-DDS sources, so
// provizio_dds patches that one declaration to add it at Fast-DDS build time
// (cmake/fast_dds/export_system_info.cmake) — see CMakeLists.txt's Fast-DDS
// ExternalProject PATCH_COMMAND.
//
// The class name must be SystemInfo verbatim (it is Fast-DDS's class) for the
// mangled symbol to resolve, so the snake_case identifier-naming convention
// can't apply here — hence the NOLINT.
namespace eprosima
{
    class SystemInfo  // NOLINT(readability-identifier-naming,cppcoreguidelines-special-member-functions)
    {
      public:
        static bool update_interfaces();
    };
}  // namespace eprosima

namespace provizio::dds
{
    bool refresh_fastdds_interface_cache() noexcept
    {
        try
        {
            return eprosima::SystemInfo::update_interfaces();
        }
        catch (...)
        {
            // SystemInfo::update_interfaces() is noexcept in practice (its only
            // operations are a libc call and a mutex-guarded vector swap),
            // but the declaration isn't marked noexcept upstream so we guard
            // against any future change there to keep our own noexcept
            // contract honest.
            return false;
        }
    }
}  // namespace provizio::dds

extern "C"
{
    bool provizio_dds_refresh_fastdds_interface_cache(void)
    {
        return provizio::dds::refresh_fastdds_interface_cache();
    }
}
