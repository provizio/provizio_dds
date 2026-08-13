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

#ifndef DDS_DETAIL_SHM_CLEANUP
#define DDS_DETAIL_SHM_CLEANUP

#include <cstddef>
#include <cstdint>
#include <string>

#include "provizio/dds/common.h"

/**
 * @file shm_cleanup.h
 * @brief Reclaims the shared-memory files of participants that died without destroying
 * themselves.
 *
 * Fast-DDS never garbage-collects them: every unclean process death (SIGKILL, a bare
 * @c exit(), an uncaught exception) leaks that participant's data segment (tens of MiB
 * at our default transport configuration) plus its lock and port files, forever. A
 * service that is restarted in a loop therefore fills the shared-memory filesystem, and
 * once it is full EVERY new participant on the host silently loses shared-memory
 * transport and falls back to UDP. Upstream's answer is a manual @c "fastdds shm clean"
 * CLI run; this is the same algorithm, run automatically.
 *
 * Safety rests entirely on the lock file Fast-DDS keeps beside each object and keeps
 * @c flock()ed for its owner's whole lifetime: the kernel releases flocks on process
 * death (SIGKILL included), so a lock file that can be locked provably has no live
 * owner. No PID scanning, no root, no heuristics — and an age guard on top, so a
 * participant caught mid-creation (its object created, its lock not yet taken) is never
 * mistaken for a corpse.
 *
 * Internal, but declared in an installed header so the test suite can drive a sweep
 * directly instead of only through participant creation.
 */

namespace provizio::dds::detail
{
    /**
     * @brief What one sweep reclaimed.
     */
    struct shm_cleanup_stats
    {
        std::size_t segments{0};  ///< Data segments removed (each with its lock file).
        std::size_t ports{0};     ///< Ports removed (each with its lock file and named semaphore).
        std::uintmax_t bytes{0};  ///< Total size of every removed file.
    };

    /**
     * @brief Whether a sweep removed anything at all.
     *
     * @param stats What the sweep reported
     * @return true when at least one segment or port was reclaimed
     */
    inline bool anything_reclaimed(const shm_cleanup_stats &stats) noexcept
    {
        return stats.segments != 0 || stats.ports != 0;
    }

    /**
     * @brief Removes every Fast-DDS shared-memory segment and port in the platform's
     * shared-memory directory whose owner process is gone, and which is older than
     * @c PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC.
     *
     * Files of live owners, files of other users, data-sharing segments and anything else in
     * the directory are left untouched. Does nothing when @c PROVIZIO_DDS_SHM_CLEANUP is off
     * or on platforms without a POSIX shared-memory directory (Windows). Safe to run
     * concurrently with itself in any number of processes, and with participants starting up.
     * Logs nothing — the callers below own that.
     *
     * @return What was reclaimed
     */
    PROVIZIO_DDS_API shm_cleanup_stats sweep_dead_shared_memory();

    /**
     * @brief Sweeps once per process, on the first call. Subsequent calls do nothing.
     *
     * Silent whatever it reclaims: this is housekeeping the caller neither asked for nor can
     * act on, and what it removes is by definition unreachable by any live process.
     *
     * Called immediately before the process creates its first Fast-DDS participant, so that a
     * service restarted in a loop buries its own predecessor's corpse: the steady state
     * becomes at most one dead generation per service rather than unbounded growth.
     */
    PROVIZIO_DDS_API void cleanup_shared_memory_once();

    /**
     * @brief Sweeps unless a sweep already ran recently (30 s). Silent, as above.
     *
     * Called when a participant finds the shared-memory filesystem nearly full, so a
     * long-running process heals its host instead of only complaining about it. The rate limit
     * is shared with @c cleanup_shared_memory_once, whose sweep therefore also suppresses an
     * immediately-following one.
     *
     * @return Whether a sweep actually ran (false when rate-limited), so the caller can
     * re-check the free space it complained about
     */
    PROVIZIO_DDS_API bool cleanup_shared_memory_if_due();
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_SHM_CLEANUP
