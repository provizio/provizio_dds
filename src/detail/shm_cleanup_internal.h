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

#ifndef DDS_DETAIL_SHM_CLEANUP_INTERNAL
#define DDS_DETAIL_SHM_CLEANUP_INTERNAL

#include <string>

#include "provizio/dds/common.h"
#include "provizio/dds/detail/shm_cleanup.h"

/**
 * @file shm_cleanup_internal.h
 * @brief The part of the shared-memory sweep that is NOT installed.
 *
 * Deliberately kept out of include/: the entry point below unlinks files under a directory
 * its caller chooses, which is not something to put in a header shipped to customers, and
 * only the test suite has any reason to call it.
 */

namespace provizio::dds::detail
{
    /**
     * @brief @c sweep_dead_shared_memory over an explicit directory instead of the platform's.
     *
     * Lets a test lay its fixtures out in a directory of its own, where no other process on
     * the host can reclaim them and nothing of the host's can be counted as the test's.
     *
     * @param directory Directory to sweep
     * @return What was reclaimed
     */
    PROVIZIO_DDS_API shm_cleanup_stats sweep_dead_shared_memory_in(const std::string &directory);
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_SHM_CLEANUP_INTERNAL
