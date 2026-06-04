// Copyright 2023 Provizio Ltd.
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

#ifndef DDS_COMMON
#define DDS_COMMON

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/rtps/common/Guid.hpp>

// ---------------------------------------------------------------------------
// Backwards-compat: Fast-DDS 3.x removed the `eprosima::fastrtps` namespace
// entirely — types like `eprosima::fastrtps::rtps::SampleIdentity`,
// `eprosima::fastrtps::rtps::GUID_t`, and `eprosima::fastrtps::rtps::WriteParams`
// all moved to `eprosima::fastdds::rtps`. provizio_dds 1.10.x exposed those
// types in its public callback signatures (via `request_response.h` and
// similar), and consumer code routinely spelled them with the legacy
// namespace. Re-introduce the namespace as an alias so existing 1.10.x
// sources keep compiling against 2.x without per-file fixups.
//
// This assumes Fast-DDS 3.x, where the real `eprosima::fastrtps` namespace no
// longer exists — a namespace-alias declaration is not conditional and would
// fail to compile if `eprosima::fastrtps` had already been declared as a real
// namespace in scope. If a future Fast-DDS release reintroduces it (unlikely
// — the upstream removal in 3.x was deliberate), this header must be updated
// to drop the alias rather than fight the upstream definition. Users who
// need the alias suppressed (e.g. when mixing translation units that
// transitively pull in a real `eprosima::fastrtps`) can define
// `PROVIZIO_DDS_NO_LEGACY_FASTRTPS_NAMESPACE` before including any
// provizio_dds header.
// ---------------------------------------------------------------------------
#ifndef PROVIZIO_DDS_NO_LEGACY_FASTRTPS_NAMESPACE
namespace eprosima
{
    namespace fastrtps = fastdds;
}  // namespace eprosima
#endif  // PROVIZIO_DDS_NO_LEGACY_FASTRTPS_NAMESPACE

/**
 * @brief DLL export/import macro for the provizio_dds shared library.
 *
 * On Windows, this resolves to `__declspec(dllexport)` when building the library
 * and `__declspec(dllimport)` when consuming it. On other platforms it is empty.
 *
 * Apply to non-template public classes and free functions that cross the DLL boundary.
 * Template-only code does not need this macro. Do not define PROVIZIO_DDS_EXPORTS
 * yourself — it is set automatically by the CMake build system.
 */
#ifdef _WIN32
#ifdef PROVIZIO_DDS_EXPORTS
#define PROVIZIO_DDS_API __declspec(dllexport)
#else
#define PROVIZIO_DDS_API __declspec(dllimport)
#endif
#else
#define PROVIZIO_DDS_API
#endif

namespace provizio::dds
{
    /**
     * @file common.h
     * @brief Common aliases and imports for Fast-DDS types used across the API.
     *
     * Brings eProsima Fast-DDS symbols into the `provizio::dds` namespace and
     * defines frequently used aliases such as `guid`.
     */
    /**
     * @brief Make Fast-DDS entities available in `provizio::dds`.
     */
    using namespace eprosima::fastdds::dds;

    /**
     * @brief Alias for the Fast-DDS RTPS GUID type (the legacy `fastrtps::rtps::GUID_t`
     * spelling that 1.10.x consumer code embedded in callback signatures still resolves
     * to this type via the `eprosima::fastrtps = eprosima::fastdds` namespace alias
     * declared above).
     */
    using guid = eprosima::fastdds::rtps::GUID_t;
}  // namespace provizio::dds

#endif  // DDS_COMMON
