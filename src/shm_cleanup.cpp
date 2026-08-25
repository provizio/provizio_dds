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

#include "detail/shm_cleanup_internal.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include "detail/env_utils.h"
#include "provizio/dds/logging.h"

#if defined(__linux__) || defined(__APPLE__)

#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace provizio::dds::detail
{
    namespace
    {
        // Kill switch, default on. An unrecognised value keeps the sweep enabled (and warns):
        // a typo must not silently reintroduce the leak this exists to stop. Same grammar as
        // PROVIZIO_DDS_NETWORK_RECOVERY.
        constexpr const char *const cleanup_enabled_env = "PROVIZIO_DDS_SHM_CLEANUP";

        // How old (seconds since its last modification) a lock file must be before it may be
        // reclaimed, even when nothing holds its lock.
        constexpr const char *const min_age_env = "PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC";

        // A participant creating a segment writes the object first and takes its lock a few
        // microseconds later; a sweep landing in between would find an unlocked lock file with
        // a very much alive owner, delete its segment, and leave it unreachable over shared
        // memory for the rest of its life without either side noticing. Fast-DDS re-creates a
        // lock file that disappeared under it (RobustExclusiveLock::open_and_lock_file retries
        // on ENOENT precisely because its own clean-up CLI can do this), but nothing re-creates
        // the DATA SEGMENT, so the window is real.
        //
        // Five seconds is ~three orders of magnitude wider than that window even when a large
        // segment's creation is descheduled on a loaded target, and short enough that a service
        // exiting and being restarted every 25-65 s (the case this was written for) has its
        // corpse reclaimed by its very next incarnation — one dead generation at a time rather
        // than the two or three a minute-long guard would leave coexisting. 0 disables the
        // guard entirely, which re-opens the window above.
        constexpr std::uint32_t default_min_age_seconds = 5;

        // Upper bound on the age guard. A day is far past any sane value, and clamping keeps the
        // arithmetic honest: time_t is 32-bit signed on some targets, where a multi-billion-second
        // guard would wrap negative and turn "too young to judge" into "reclaim everything".
        constexpr std::uint32_t max_min_age_seconds = 24U * 60U * 60U;

        // Minimum spacing between sweeps. Only the pressure-triggered path can repeat, and a
        // host that is genuinely out of shared memory will not become reclaimable again within
        // seconds, so this only caps the cost of a process that creates participants in a loop.
        constexpr std::chrono::seconds min_sweep_interval{30};

        // Where Boost.Interprocess — and therefore Fast-DDS — keeps its shared-memory objects
        // (mirrors eProsima's own "fastdds shm clean" directory resolution). Windows uses
        // neither this path nor flock semantics and is handled by the stubs at the bottom of
        // this file.
        constexpr const char *const platform_shm_directory =
#if defined(__APPLE__)
            "/private/tmp/boost_interprocess";
#else
            "/dev/shm";
#endif

        // The domain-name prefixes Fast-DDS gives its shared-memory objects: "fastdds" since
        // 3.x and "fastrtps" in 2.x. Both leak identically and both are still found on hosts
        // running a mixed fleet (a 2.x-based application, or provizio_dds 1.x), and the lock
        // files work the same way in both, so both are swept.
        constexpr std::array<std::string_view, 2> shm_prefixes{"fastdds_", "fastrtps_"};

        // "_el" (exclusive lock, one owner) / "_sl" (shared lock, several attachers).
        constexpr std::size_t lock_suffix_length = 3;
        // Segment names are a 16-character random id.
        constexpr std::size_t segment_id_length = 16;
        // Port names carry a port number, at most 5 digits.
        constexpr std::size_t max_port_digits = 5;
        constexpr std::string_view port_marker = "port";

        // What a directory entry is, as far as this sweep is concerned.
        enum class shm_object_kind : std::uint8_t
        {
            none,     ///< Not a Fast-DDS lock file — never touched.
            segment,  ///< Data-segment lock: <prefix><16 lowercase alphanumerics>(_el|_sl).
            port,     ///< Port lock: <prefix>port<1-5 digits>(_el|_sl).
        };

        bool cleanup_enabled()
        {
            const char *const env = std::getenv(cleanup_enabled_env);  // NOLINT: getenv required
            if (env == nullptr || *env == '\0')
            {
                return true;
            }

            bool enabled = true;
            if (try_parse_bool(env, enabled))
            {
                return enabled;
            }

            log_warning() << cleanup_enabled_env << "='" << sanitise_env_value_for_log(env)
                          << "' is not recognised (use on/off); dead-owner shared-memory cleanup stays enabled";
            return true;
        }

        std::uint32_t min_age_seconds()
        {
            const char *const env = std::getenv(min_age_env);  // NOLINT: getenv required
            if (env == nullptr || *env == '\0')
            {
                return default_min_age_seconds;
            }

            std::uint32_t parsed = 0;
            if (try_parse_u32(env, parsed))
            {
                if (parsed > max_min_age_seconds)
                {
                    log_warning() << min_age_env << "=" << parsed << " exceeds the maximum of " << max_min_age_seconds
                                  << " seconds; clamping to it";
                    return max_min_age_seconds;
                }
                return parsed;
            }

            log_warning() << "ignoring invalid " << min_age_env << "='" << sanitise_env_value_for_log(env)
                          << "'; using default " << default_min_age_seconds;
            return default_min_age_seconds;
        }

        bool is_lowercase_alphanumeric(const char character)
        {
            return (character >= '0' && character <= '9') || (character >= 'a' && character <= 'z');
        }

        bool is_ascii_digit(const char character)
        {
            return character >= '0' && character <= '9';
        }

        // Classifies a directory entry by name alone, fully anchored at both ends:
        // ^<prefix>[0-9a-z]{16}_el$ for a segment and ^<prefix>port[0-9]{1,5}(_el|_sl)$ for a
        // port. Anything else — including the "fast_datasharing_*" segments, whose liveness has
        // entirely different semantics (a live reader may map a dead writer's segment) — is
        // `none` and is left alone.
        //
        // Narrower than eProsima's "fastdds shm clean", which accepts "_sl" for segments too.
        // Fast-DDS only ever gives a SEGMENT an exclusive lock (SharedMemManager builds
        // `segment_name + "_el"`, never "_sl"; only ports take a shared one), so accepting
        // "_sl" there reclaims no real corpse and leaves a name that is free for the taking
        // while the segment is alive — exactly what someone would need to aim the sweep at a
        // live segment. Both suffixes stay valid for ports, where Fast-DDS uses both.
        //
        // Written out rather than handed to std::regex because it runs once per entry of a
        // directory that, in the situation this exists for, holds tens of thousands of them.
        shm_object_kind classify(const std::string_view name)
        {
            if (name.size() <= lock_suffix_length)
            {
                return shm_object_kind::none;
            }
            const auto suffix = name.substr(name.size() - lock_suffix_length);
            const bool exclusive_lock = suffix == "_el";
            if (!exclusive_lock && suffix != "_sl")
            {
                return shm_object_kind::none;
            }

            const auto body = name.substr(0, name.size() - lock_suffix_length);
            for (const auto prefix : shm_prefixes)
            {
                if (body.size() <= prefix.size() || body.substr(0, prefix.size()) != prefix)
                {
                    continue;
                }

                const auto tail = body.substr(prefix.size());
                if (exclusive_lock && tail.size() == segment_id_length &&
                    std::all_of(tail.begin(), tail.end(), is_lowercase_alphanumeric))
                {
                    return shm_object_kind::segment;
                }
                if (tail.size() > port_marker.size() && tail.substr(0, port_marker.size()) == port_marker)
                {
                    const auto digits = tail.substr(port_marker.size());
                    if (digits.size() <= max_port_digits && std::all_of(digits.begin(), digits.end(), is_ascii_digit))
                    {
                        return shm_object_kind::port;
                    }
                }
            }

            return shm_object_kind::none;
        }

        // Seconds + nanoseconds of a file's last modification, spelled differently per platform.
        const timespec &modification_time(const struct stat &info)
        {
#if defined(__APPLE__)
            return info.st_mtimespec;
#else
            return info.st_mtim;
#endif
        }

        // Whether `info` was last modified at least `min_age` seconds ago. Compared at the
        // nanosecond resolution both sides carry, so the configured guard is the guarantee (a
        // whole-second comparison would make a guard of 1 mean anywhere between 0 and 1 second)
        // and so C++ and Python agree. Written without converting to a single count to keep it
        // free of overflow whatever a file's timestamp says; a timestamp in the future yields
        // "too young", which is the safe side to land on.
        bool older_than(const struct stat &info, const std::time_t min_age)
        {
            // system_clock is the realtime clock POSIX file timestamps are taken from, so the
            // two are directly comparable. Preferred over clock_gettime(CLOCK_REALTIME) so this
            // needs nothing from <time.h>, whose POSIX declarations <ctime> does not guarantee.
            const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
            const auto now_seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
            const auto now_nanoseconds =
                std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - now_seconds).count();

            const auto &mtime = modification_time(info);
            const auto seconds = static_cast<std::time_t>(now_seconds.count()) - mtime.tv_sec;
            return seconds > min_age || (seconds == min_age && now_nanoseconds >= mtime.tv_nsec);
        }

        // Unlinks one of a dead owner's companion files — the object a lock file names, or a
        // port's named semaphore — adding its size to @p bytes when it goes.
        //
        // `owner` is the uid of the lock file whose flock proved the owner dead, and the file
        // must belong to the same user to be removed. That check is what stops the whole sweep
        // from being steered by whoever wrote the lock file's NAME: only the lock file's own
        // liveness is proven by the flock, and a name like "<live segment id>_sl" is one
        // Fast-DDS never creates for a segment (SharedMemManager uses "_el" exclusively), so
        // anyone able to write to this world-writable directory could otherwise plant one and
        // have a sweep delete a live participant's segment out from under it. A regular file
        // owned by the same user as its lock file is the only thing Fast-DDS ever creates here.
        //
        // Every failure is a silent skip: ENOENT means a concurrent sweeper (in any process)
        // got there first, and EACCES / EPERM means the sticky bit refused — both are normal,
        // neither is worth a log line.
        bool remove_companion_file(const int dir_fd, const std::string &name, const uid_t owner, std::uintmax_t &bytes)
        {
            struct stat info
            {
            };
            if (::fstatat(dir_fd, name.c_str(), &info, AT_SYMLINK_NOFOLLOW) != 0)
            {
                return false;
            }
            if (S_ISREG(info.st_mode) == 0 || info.st_uid != owner)
            {
                return false;
            }
            if (::unlinkat(dir_fd, name.c_str(), 0) != 0)
            {
                return false;
            }
            bytes += static_cast<std::uintmax_t>(info.st_size);
            return true;
        }

        // Whether the file this fd holds open is a Fast-DDS lock file rather than something
        // that merely took its name. Fast-DDS creates them empty and never writes to them
        // (RobustExclusiveLock / RobustSharedLock only open and flock), so a non-empty one was
        // written by something else; and a link count above one means the name was hardlinked
        // onto another file — the way an attacker would make a lock file whose ownership
        // matches a victim's object. Checked on the DESCRIPTOR, so it describes the file that
        // was actually locked rather than whatever the name resolves to now.
        bool is_lock_file(const struct stat &info)
        {
            return S_ISREG(info.st_mode) != 0 && info.st_size == 0 && info.st_nlink == 1;
        }

        // Sweep bookkeeping shared by both entry points, so a pressure-triggered sweep is also
        // rate-limited against the once-per-process one.
        struct sweep_history
        {
            std::mutex mutex;
            bool has_swept{false};
            std::chrono::steady_clock::time_point last_sweep;
        };

        // Wrapped in an accessor so the state is a function-local static rather than a mutable
        // global (which cppcoreguidelines-avoid-non-const-global-variables rightly rejects).
        sweep_history &sweeps()
        {
            static sweep_history history;
            return history;
        }

    }  // namespace

    shm_cleanup_stats sweep_dead_shared_memory()
    {
        return sweep_dead_shared_memory_in(platform_shm_directory);
    }

    shm_cleanup_stats sweep_dead_shared_memory_in(const std::string &shm_directory)
    {
        shm_cleanup_stats stats;
        if (!cleanup_enabled())
        {
            return stats;
        }

        // O_DIRECTORY | O_NOFOLLOW, and every lookup below made RELATIVE to the resulting
        // descriptor: the sweep is pinned to the one directory it opened for its whole run, so
        // nothing it unlinks can be redirected by swapping that directory for a symlink
        // underneath it. (/dev/shm is a root-owned mount point, but the macOS location lives
        // under a world-writable /private/tmp.) It also removes path assembly from the delete
        // path entirely — the only names used are the ones readdir returned.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,hicpp-vararg): open's variadic mode is unused here
        const int dir_fd = ::open(shm_directory.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
        if (dir_fd < 0)
        {
            // No shared-memory directory, or no permission to open it: nothing this process
            // could clean up even in principle.
            return stats;
        }
        // fdopendir adopts the descriptor; closedir closes both. The unique_ptr keeps that true
        // even if a std::string allocation below throws.
        const std::unique_ptr<DIR, int (*)(DIR *)> directory{::fdopendir(dir_fd), &::closedir};
        if (!directory)
        {
            ::close(dir_fd);
            return stats;
        }

        const auto min_age = static_cast<std::time_t>(min_age_seconds());

        // readdir's per-DIR* buffer is not shared here: the handle is opened, iterated and
        // closed entirely within this call. A null return ends the scan whether it means
        // end-of-directory or an error; the difference would only decide whether a truncated
        // sweep is worth a log line, and the next sweep covers whatever was missed either way.
        // NOLINTNEXTLINE(concurrency-mt-unsafe)
        while (const dirent *const entry = ::readdir(directory.get()))
        {
            const std::string name{static_cast<const char *>(entry->d_name)};
            const shm_object_kind kind = classify(name);
            if (kind == shm_object_kind::none)
            {
                continue;
            }

            // No O_CREAT, so a lock file that vanished between the scan and here is a skip
            // rather than a resurrection; O_NOFOLLOW so a symlink dropped in this
            // world-writable directory under a Fast-DDS-looking name is never opened; and
            // O_NONBLOCK so the open itself cannot wait on something that is not a regular
            // file, which is only established afterwards, from the descriptor.
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,hicpp-vararg): open's variadic mode is unused here
            const int lock_fd = ::openat(dir_fd, name.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
            if (lock_fd < 0)
            {
                // Vanished, a symlink, or not writable by this user — skip either way. Note
                // this is not what keeps another user's files safe: Fast-DDS creates them
                // 0666 before umask, and root ignores the mode entirely. The ownership check
                // on each companion below is the control that matters.
                continue;
            }

            // Everything from here on is judged on the DESCRIPTOR, never on the name: the name
            // could be renamed onto a different file between any two lookups, and this is the
            // file whose lock is about to decide another file's fate.
            struct stat lock_info
            {
            };
            if (::fstat(lock_fd, &lock_info) != 0 || !is_lock_file(lock_info) || !older_than(lock_info, min_age))
            {
                // Not a Fast-DDS lock file (non-empty, hardlinked, or not a regular file), or
                // too young to judge — either a participant starting up right now, or one that
                // died moments ago and will be reclaimed by the next sweep.
                ::close(lock_fd);
                continue;
            }

            if (::flock(lock_fd, LOCK_EX | LOCK_NB) != 0)
            {
                // The owner holds the lock, so the owner is alive. The kernel would have
                // dropped it otherwise, SIGKILL included — this is the whole safety argument.
                // A rarer errno (EINTR, ENOLCK) lands here too and costs at most a reclaim
                // deferred to the next sweep, which is the right way to be wrong.
                ::close(lock_fd);
                continue;
            }

            // Dead owner. Unlink while still holding the lock, so a participant that starts up
            // during this sweep either waits behind it or creates its files afresh afterwards.
            //
            // Holding it does briefly deny the name to anyone else. That is only reachable for
            // PORT names (segment ids are random, so a new participant never asks for a corpse's
            // name), it lasts for the unlinks below, and Fast-DDS already handles it:
            // RobustExclusiveLock re-creates a lock file that vanished under it, and
            // SharedMemGlobal::open_port catches a failed lock as "this port belongs to another
            // reader" and moves on — which is exactly the conclusion its own zombie check reaches
            // when it takes and releases the same lock.
            //
            // The companions are removed only if they belong to the same user as the lock file
            // (see remove_companion_file); the lock file itself is removed regardless, since its
            // own liveness is what was just proven.
            const std::string object_name = name.substr(0, name.size() - lock_suffix_length);
            const bool reclaimed = remove_companion_file(dir_fd, object_name, lock_info.st_uid, stats.bytes);
            if (kind == shm_object_kind::port)
            {
                // Ports also have a POSIX named semaphore, which lives in the same directory as
                // "sem.<port object name>_mutex" and leaks with the rest.
                remove_companion_file(dir_fd, "sem." + object_name + "_mutex", lock_info.st_uid, stats.bytes);
            }
            if (::unlinkat(dir_fd, name.c_str(), 0) == 0)
            {
                stats.bytes += static_cast<std::uintmax_t>(lock_info.st_size);
            }

            // Counted on the OBJECT, not on the lock file: a port can have both an "_el" and an
            // "_sl" lock, and counting per lock file would report one reclaimed port as two. A
            // lock file with no object left is still removed, just not counted — there is no
            // object to have reclaimed.
            if (reclaimed)
            {
                if (kind == shm_object_kind::port)
                {
                    ++stats.ports;
                }
                else
                {
                    ++stats.segments;
                }
            }

            ::close(lock_fd);  // Releases the lock; the file it belonged to is gone by now.
        }

        return stats;
    }

    void cleanup_shared_memory_once()
    {
        static std::once_flag once;
        std::call_once(once, [] {
            {
                auto &history = sweeps();
                const std::lock_guard<std::mutex> lock{history.mutex};
                history.has_swept = true;
                history.last_sweep = std::chrono::steady_clock::now();
            }
            // Nothing is logged, however much is reclaimed: this is housekeeping the caller
            // neither asked for nor can act on. What it reclaims is, by definition, memory no
            // live process can reach.
            sweep_dead_shared_memory();
        });
    }

    bool cleanup_shared_memory_if_due()
    {
        {
            auto &history = sweeps();
            const std::lock_guard<std::mutex> lock{history.mutex};
            const auto now = std::chrono::steady_clock::now();
            if (history.has_swept && now - history.last_sweep < min_sweep_interval)
            {
                return false;
            }
            // Stamped before the sweep rather than after it, so a second thread arriving while
            // this one scans the directory is turned away immediately instead of duplicating
            // the work.
            history.has_swept = true;
            history.last_sweep = now;
        }

        sweep_dead_shared_memory();
        return true;
    }
}  // namespace provizio::dds::detail

#else  // Neither Linux nor macOS

namespace provizio::dds::detail
{
    // Windows keeps its Fast-DDS interprocess files elsewhere and guards them with LockFileEx
    // rather than flock, so none of the above applies. It is also the one platform where
    // provizio_dds disables shared memory outright (Fast-DDS's bundled Boost.Interprocess
    // leaks segments and named semaphores there), so there is nothing of ours to reclaim.
    shm_cleanup_stats sweep_dead_shared_memory()
    {
        return {};
    }

    shm_cleanup_stats sweep_dead_shared_memory_in(const std::string & /*directory*/)
    {
        return {};
    }

    void cleanup_shared_memory_once()
    {
    }

    bool cleanup_shared_memory_if_due()
    {
        return false;
    }
}  // namespace provizio::dds::detail

#endif  // defined(__linux__) || defined(__APPLE__)
