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
//
// Subcommand-driven tests for the dead-owner shared-memory sweep: what it
// reclaims (segments, ports and their named semaphores, under both the 3.x
// "fastdds_" and the 2.x "fastrtps_" naming), what it must never touch (files
// whose owner still holds their lock — including a live participant's, and this
// very process's own — files younger than the age guard, and everything in the
// directory that is not a Fast-DDS lock file), that the kill switch really
// switches it off, that repeat sweeps are rate-limited, and that creating a
// participant runs it. Each ctest entry runs the same binary with a different
// subcommand so per-case failure stays isolated, mirroring the transport_tuning
// and discovery_tuning tests.
//
// Linux-only: shared memory is the default transport there and nowhere else in
// provizio_dds (Windows and macOS force UDP over a Boost.Interprocess cleanup
// bug), so no other platform has anything to reclaim. See the CMakeLists.
//
// Most cases work in a directory of their own and touch nothing else. The three
// that cannot — the two driving a real participant, and the one driving the
// production entry point, which always sweeps the platform directory — do sweep
// the host's real /dev/shm, so running them on a development machine reclaims
// whatever genuine Fast-DDS corpses it happens to be holding.

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <csignal>
#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

#include "detail/shm_cleanup_internal.h"
#include "provizio/dds/domain_participant.h"
#include "provizio/dds/network_recovery.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/subscriber.h"

#include <std_msgs/msg/StringPubSubTypes.hpp>

namespace
{
    // Domains of their own, so the live-participant case can neither feed nor be fed by the
    // rest of the suite, which lives on domain 0. The killed-participant case puts its
    // sweeping participant on a SEPARATE domain from the child it killed: Fast-DDS derives
    // shared-memory port numbers from the domain, so a same-domain participant would
    // immediately recreate the very port files the sweep had just reclaimed.
    constexpr auto k_domain = 71;
    constexpr auto k_sweeper_domain = 72;

    constexpr const char *const k_enabled_env = "PROVIZIO_DDS_SHM_CLEANUP";
    constexpr const char *const k_min_age_env = "PROVIZIO_DDS_SHM_CLEANUP_MIN_AGE_SEC";

    // Where Fast-DDS keeps its shared-memory objects on Linux.
    constexpr const char *const k_platform_shm_dir = "/dev/shm";

    // Most cases sweep a directory of their own (see private_shm_dir) rather than the host's
    // live one: nothing else on the machine can then reclaim their fixtures, nothing of the
    // host's is counted as theirs, and no age guard is needed in either direction.
    constexpr const char *const k_no_age_guard = "0";

    // The age-guard case is the exception that needs the guard itself under test. Its fixtures
    // are still private, so the numbers only have to be far apart: a guard well above the time
    // the case takes to run, and a corpse backdated past it.
    constexpr const char *const k_age_guard_min_age = "30";
    constexpr int k_age_guard_backdate_seconds = 60;

    // The two cases that must work in the REAL shared-memory directory, alongside live
    // participants, instead lower the guard just enough to reach their own fixtures while
    // staying under the 5 s default every other provizio_dds process on the host uses — so a
    // participant starting up elsewhere in a parallel ctest run can never reclaim them
    // mid-case. (Those two are also kept apart from each other by a ctest RESOURCE_LOCK.)
    constexpr const char *const k_shared_dir_min_age = "1";
    constexpr int k_shared_dir_backdate_seconds = 2;

    // The directory the current case works in — the platform one unless it claimed a private
    // directory of its own.
    std::string &shm_dir()
    {
        static std::string directory{k_platform_shm_dir};
        return directory;
    }

    // Lengths of the two name shapes the sweep recognises.
    constexpr std::size_t k_lock_suffix_length = 3;  // "_el" / "_sl"
    constexpr std::size_t k_segment_id_length = 16;
    // Hex digits of the pid a synthetic name carries; the rest of a segment id is its marker.
    constexpr std::size_t k_tag_length = 12;

    bool expect(bool condition, const char *expression, const char *file, int line)
    {
        if (!condition)
        {
            std::cerr << "FAIL " << file << ":" << line << ": " << expression << '\n';
        }
        return condition;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage): captures #cond + __FILE__/__LINE__.
#define EXPECT(cond) expect((cond), #cond, __FILE__, __LINE__)

    void set_env(const char *name, const char *value)
    {
        // NOLINTNEXTLINE: POSIX env API
        setenv(name, value, /*overwrite=*/1);
    }

    void unset_env(const char *name)
    {
        // NOLINTNEXTLINE: POSIX env API
        unsetenv(name);
    }

    std::string shm_path(const std::string &name)
    {
        return shm_dir() + "/" + name;
    }

    bool exists(const std::string &name)
    {
        struct stat info
        {
        };
        return ::lstat(shm_path(name).c_str(), &info) == 0;
    }

    // The file's inode number, or 0 when it does not exist. Identity rather than presence is
    // what "was it reclaimed?" needs: a participant starting up right after a sweep can
    // legitimately recreate a name the sweep just freed (port names are derived from the
    // domain, so they repeat), and tmpfs hands out inode numbers from a monotonic counter, so
    // a different inode under the same name is provably a different file.
    ino_t inode_of(const std::string &name)
    {
        struct stat info
        {
        };
        if (::lstat(shm_path(name).c_str(), &info) != 0)
        {
            return 0;
        }
        return info.st_ino;
    }

    // Sets a file's modification time `seconds_ago` seconds into the past, so a synthetic
    // lock file can pose as a corpse the age guard is willing to reclaim.
    bool backdate(const std::string &name, const int seconds_ago)
    {
        const auto when = ::time(nullptr) - seconds_ago;
        const std::array<timeval, 2> times{timeval{when, 0}, timeval{when, 0}};
        return ::utimes(shm_path(name).c_str(), times.data()) == 0;
    }

    // Hardlinks `existing` to `link_name`, giving both a link count of 2.
    bool hardlink(const std::string &existing, const std::string &link_name)
    {
        return ::link(shm_path(existing).c_str(), shm_path(link_name).c_str()) == 0;
    }

    // Creates a file in the shared-memory directory, failing (rather than truncating) when
    // the name is already taken — a synthetic fixture must never land on a name a real
    // participant is using. Returns the open descriptor, or -1.
    int create_exclusively(const std::string &name)
    {
        // The mode Fast-DDS gives its own shared-memory objects.
        constexpr mode_t fixture_mode = 0644;
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,hicpp-vararg): open's variadic mode argument
        return ::open(shm_path(name).c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC, fixture_mode);
    }

    // Every synthetic file a case created, removed on the way out however the case ends.
    class fixture_files
    {
      public:
        fixture_files() = default;
        fixture_files(const fixture_files &) = delete;
        fixture_files &operator=(const fixture_files &) = delete;
        fixture_files(fixture_files &&) = delete;
        fixture_files &operator=(fixture_files &&) = delete;

        ~fixture_files()
        {
            for (const auto &name : names)
            {
                ::unlink(shm_path(name).c_str());
            }
        }

        // Creates `name` and remembers it for teardown. False when the name was taken (or
        // could not be sized — it is registered for teardown before that can fail, so a
        // half-made fixture is still cleaned up).
        bool add(const std::string &name, const off_t size = 0)
        {
            const int file = create_exclusively(name);
            if (file < 0)
            {
                return false;
            }
            names.push_back(name);
            const bool sized = size == 0 || ::ftruncate(file, size) == 0;
            ::close(file);
            return sized;
        }

      private:
        std::vector<std::string> names;
    };

    // A per-process discriminator, so several test processes (and the rest of a parallel
    // ctest run) can never pick the same synthetic names. 12 hex digits of the pid, which
    // leaves room for a 4-character marker within a 16-character segment id and is
    // unmistakably not a real (random) Fast-DDS id.
    std::string unique_tag()
    {
        std::ostringstream stream;
        stream << std::hex << std::setfill('0') << std::setw(static_cast<int>(k_tag_length))
               << static_cast<std::uint64_t>(::getpid());
        const auto text = stream.str();
        return text.substr(text.size() - k_tag_length);
    }

    // <prefix><4-character marker><12 hex of the pid> — a syntactically valid segment name.
    // The marker distinguishes several fixtures within one case and says what each is for.
    std::string segment_name(const std::string &prefix, const std::string &marker)
    {
        return prefix + marker + unique_tag();
    }

    // Claims a directory of this case's own, inside the shared-memory filesystem so the
    // fixtures keep tmpfs semantics (flock, unlink, monotonic inode numbers), and points every
    // helper above at it for the case's lifetime.
    //
    // This is what makes the synthetic cases hermetic: no other provizio_dds process on the
    // host sweeps a subdirectory (the sweep does not recurse, and the directory's own name
    // matches no lock-file shape), so a fixture cannot be reclaimed by someone else's
    // participant starting up — and equally, a case can assert exact counts because nothing of
    // the host's own leaked shared memory is in scope. Declare it BEFORE the case's
    // fixture_files so the files are removed before the directory is.
    class private_shm_dir
    {
      public:
        explicit private_shm_dir(const std::string &name)
            : path(std::string{k_platform_shm_dir} + "/provizio_dds_shm_cleanup_" + name + "_" + unique_tag()),
              created(::mkdir(path.c_str(), owner_only) == 0)
        {
            if (created)
            {
                shm_dir() = path;
            }
        }
        private_shm_dir(const private_shm_dir &) = delete;
        private_shm_dir &operator=(const private_shm_dir &) = delete;
        private_shm_dir(private_shm_dir &&) = delete;
        private_shm_dir &operator=(private_shm_dir &&) = delete;

        ~private_shm_dir()
        {
            if (created)
            {
                shm_dir() = k_platform_shm_dir;
                ::rmdir(path.c_str());
            }
        }

        [[nodiscard]] bool claimed() const
        {
            return created;
        }

      private:
        // rwx for this user only: nothing else has any business in here.
        static constexpr mode_t owner_only = 0700;

        std::string path;
        bool created;
    };

    // Creates a segment corpse: the object plus its exclusive lock file, locked by nobody.
    // `backdate_seconds` ages it when the case needs it past a guard; 0 leaves the mtime where
    // it is, which is what a case sweeping with no guard at all wants. Returns the object name,
    // or "" on failure.
    std::string create_dead_segment(fixture_files &files, const std::string &prefix, const int backdate_seconds = 0,
                                    const std::string &marker = "dead")
    {
        constexpr off_t segment_size = 4096;
        std::string object = segment_name(prefix, marker);
        if (!files.add(object, segment_size) || !files.add(object + "_el"))
        {
            return {};
        }
        if (backdate_seconds != 0 &&
            (!backdate(object, backdate_seconds) || !backdate(object + "_el", backdate_seconds)))
        {
            return {};
        }
        return object;
    }

    // A synthetic port number in the "...5" tail Fast-DDS never assigns (its own ports end in
    // 0, 1 or 3), spread across processes by pid. `offset` walks further candidates.
    int candidate_port(const int offset)
    {
        constexpr int first_candidate_port = 60005;
        constexpr int stride = 10;
        constexpr int spread = 90;
        return first_candidate_port + (((static_cast<int>(::getpid()) % spread) + offset) * stride);
    }

    // Creates a port corpse: the object, its exclusive lock file and its named semaphore, all
    // unlocked. Names are claimed with O_EXCL, so a live port cannot be shadowed even if the
    // candidate arithmetic ever collides. Returns the object name, or "" on failure.
    std::string create_dead_port(fixture_files &files, const std::string &prefix,
                                 const std::string &lock_suffix = "_el")
    {
        constexpr int candidates = 16;
        constexpr off_t port_size = 52400;
        for (int attempt = 0; attempt < candidates; ++attempt)
        {
            std::string object = prefix + "port" + std::to_string(candidate_port(attempt));
            if (!files.add(object, port_size))
            {
                continue;  // Taken — try the next candidate.
            }
            const std::string semaphore = "sem." + object + "_mutex";
            if (!files.add(object + lock_suffix) || !files.add(semaphore))
            {
                return {};
            }
            return object;
        }
        return {};
    }

    // Takes an exclusive flock on a file and holds it for its whole lifetime — exactly what a
    // live Fast-DDS participant does with its lock files, and what tells the sweep to keep
    // its hands off. Also used the other way round, to ask whether anyone else holds one.
    class scoped_flock
    {
      public:
        explicit scoped_flock(const std::string &name)
        {
            // O_NOFOLLOW to match the sweep's own open exactly — this class is used both to
            // hold a lock and to ask whether the sweep would find one held.
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,hicpp-vararg): open's variadic mode is unused here
            fd = ::open(shm_path(name).c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW);
            if (fd >= 0 && ::flock(fd, LOCK_EX | LOCK_NB) != 0)
            {
                ::close(fd);
                fd = -1;
            }
        }
        scoped_flock(const scoped_flock &) = delete;
        scoped_flock &operator=(const scoped_flock &) = delete;
        scoped_flock(scoped_flock &&) = delete;
        scoped_flock &operator=(scoped_flock &&) = delete;

        ~scoped_flock()
        {
            if (fd >= 0)
            {
                ::close(fd);
            }
        }

        // Whether the lock was taken — which also means nobody else was holding it.
        [[nodiscard]] bool acquired() const
        {
            return fd >= 0;
        }

      private:
        int fd{-1};
    };

    bool is_lowercase_alphanumeric(const char character)
    {
        return (character >= '0' && character <= '9') || (character >= 'a' && character <= 'z');
    }

    bool is_ascii_digit(const char character)
    {
        return character >= '0' && character <= '9';
    }

    // An independent re-implementation of the sweep's name matcher, deliberately not shared
    // with it: a case that asks "what was reclaimable before the sweep?" must not inherit the
    // very classification it is checking. Returns whether `name` is a lock file, and whether
    // it is a port's (as opposed to a segment's) through `is_port`.
    bool is_lock_file_name(const std::string &name, bool &is_port)
    {
        constexpr std::size_t port_marker_length = 4;  // "port"
        constexpr std::size_t max_port_digits = 5;

        if (name.size() <= k_lock_suffix_length)
        {
            return false;
        }
        const auto suffix = name.substr(name.size() - k_lock_suffix_length);
        if (suffix != "_el" && suffix != "_sl")
        {
            return false;
        }
        const auto body = name.substr(0, name.size() - k_lock_suffix_length);
        for (const std::string &prefix : {std::string{"fastdds_"}, std::string{"fastrtps_"}})
        {
            if (body.size() <= prefix.size() || body.compare(0, prefix.size(), prefix) != 0)
            {
                continue;
            }
            const auto tail = body.substr(prefix.size());
            if (tail.size() == k_segment_id_length && std::all_of(tail.begin(), tail.end(), is_lowercase_alphanumeric))
            {
                is_port = false;
                return true;
            }
            if (tail.size() > port_marker_length && tail.size() <= port_marker_length + max_port_digits &&
                tail.compare(0, port_marker_length, "port") == 0 &&
                std::all_of(std::next(tail.begin(), port_marker_length), tail.end(), is_ascii_digit))
            {
                is_port = true;
                return true;
            }
        }
        return false;
    }

    // One Fast-DDS shared-memory object the sweep is obliged to reclaim, identified by the
    // inodes it had when it was found rather than by name alone.
    struct reclaimable_object
    {
        std::string lock_name;
        ino_t lock_inode{0};
        ino_t object_inode{0};
        bool is_port{false};
    };

    // Every Fast-DDS lock file in the shared-memory directory this process could reclaim
    // right now: nothing holds its lock, and it is old enough for the given age guard. The
    // sweep's contract is that ALL of these go and nothing else does — which is what makes
    // the killed-participant case immune to whatever else happens to be running on the host,
    // rather than depending on a remembered list of one child's files.
    std::vector<reclaimable_object> reclaimable_objects(const int min_age_seconds)
    {
        // Deliberately one second stricter than the sweep's own guard. This scan compares
        // whole seconds (::time / st_mtime) while the sweep compares nanoseconds, so a file
        // whose true age is a fraction under the guard can look old enough here and be
        // skipped there. Everything this returns must be something the sweep will certainly
        // take, so it rounds the wrong way on purpose.
        const int strict_min_age = min_age_seconds + 1;
        std::vector<reclaimable_object> result;
        DIR *const directory = ::opendir(shm_dir().c_str());
        if (directory == nullptr)
        {
            return result;
        }
        const auto now = ::time(nullptr);
        // NOLINTNEXTLINE(concurrency-mt-unsafe): this DIR* is opened, read and closed here
        while (const dirent *const entry = ::readdir(directory))
        {
            const std::string name{static_cast<const char *>(entry->d_name)};
            bool is_port = false;
            if (!is_lock_file_name(name, is_port))
            {
                continue;
            }
            struct stat info
            {
            };
            if (::lstat(shm_path(name).c_str(), &info) != 0 || S_ISREG(info.st_mode) == 0 || info.st_size != 0 ||
                info.st_nlink != 1 || now - info.st_mtime < strict_min_age)
            {
                // Same predicate the sweep applies to a lock file, plus the stricter age.
                continue;
            }
            const scoped_flock probe{name};
            if (probe.acquired())
            {
                result.push_back(reclaimable_object{
                    name, info.st_ino, inode_of(name.substr(0, name.size() - k_lock_suffix_length)), is_port});
            }
        }
        ::closedir(directory);
        return result;
    }

    // Whether the file called `name` is gone or has since been replaced by a different one.
    // An inode of 0 means it was not there to begin with, which counts as reclaimed.
    bool reclaimed(const std::string &name, const ino_t previous_inode)
    {
        return previous_inode == 0 || inode_of(name) != previous_inode;
    }

    // ---- Child processes -------------------------------------------------------------

    // Runs in a forked child: creates a participant — whose shared-memory files are what the
    // parent is really testing against — and reports readiness down `ready_fd`. Never returns.
    // The topic comes from the parent (which stamps it with ITS pid, the one the parent's
    // subscriber will use); computing it here would name it after the child's instead.
    // How often the publishing child sends a sample.
    constexpr auto k_publish_period = std::chrono::milliseconds{100};

    [[noreturn]] void run_participant_child(const int ready_fd, const std::string &publish_topic)
    {
        // A child must never sweep: the parent owns every assertion about what disappeared.
        set_env(k_enabled_env, "0");

        const auto participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);

        std::shared_ptr<provizio::dds::publisher_handle<std_msgs::msg::StringPubSubType>> publisher;
        if (!publish_topic.empty())
        {
            publisher = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(participant, publish_topic);
        }

        const char ready = 'R';
        const auto written = ::write(ready_fd, &ready, 1);
        static_cast<void>(written);

        std_msgs::msg::String message;
        message.data("alive");
        for (;;)
        {
            if (publisher)
            {
                publisher->publish(message);
            }
            std::this_thread::sleep_for(k_publish_period);
        }
    }

    // A forked child running a participant, SIGKILLed on the way out — the whole point of the
    // exercise, since the kernel then drops its flocks and Fast-DDS gets no chance to remove
    // anything.
    class participant_child
    {
      public:
        // An empty `publish_topic` means "just hold a participant"; otherwise the child also
        // publishes on that topic.
        explicit participant_child(const std::string &publish_topic = {})
        {
            std::array<int, 2> pipe_fds{-1, -1};
            if (::pipe2(pipe_fds.data(), O_CLOEXEC) != 0)
            {
                return;
            }

            pid = ::fork();
            if (pid == 0)
            {
                ::close(pipe_fds[0]);
                run_participant_child(pipe_fds[1], publish_topic);
            }

            ::close(pipe_fds[1]);
            if (pid > 0)
            {
                // Blocks until the child has its participant up — or dies trying, in which
                // case the read returns 0 and the case fails rather than hanging.
                char ready = '\0';
                ready_seen = ::read(pipe_fds[0], &ready, 1) == 1 && ready == 'R';
            }
            ::close(pipe_fds[0]);
        }
        participant_child(const participant_child &) = delete;
        participant_child &operator=(const participant_child &) = delete;
        participant_child(participant_child &&) = delete;
        participant_child &operator=(participant_child &&) = delete;

        ~participant_child()
        {
            kill_and_reap();
        }

        [[nodiscard]] bool ready() const
        {
            return ready_seen;
        }

        [[nodiscard]] bool alive() const
        {
            return pid > 0 && ::kill(pid, 0) == 0;
        }

        void kill_and_reap()
        {
            if (pid > 0)
            {
                ::kill(pid, SIGKILL);
                int status = 0;
                ::waitpid(pid, &status, 0);
                pid = -1;
            }
        }

      private:
        pid_t pid{-1};
        bool ready_seen{false};
    };

    // ---- Cases -----------------------------------------------------------------------

    // Case: a segment corpse and a port corpse — under both the 3.x and the 2.x naming — are
    // reclaimed with every file that belongs to them, the port's named semaphore included.
    int test_reclaims_synthetic()
    {
        const private_shm_dir directory{"synthetic"};
        set_env(k_min_age_env, k_no_age_guard);

        fixture_files files;
        if (!EXPECT(directory.claimed()))
        {
            return 1;
        }
        const auto fastdds_segment = create_dead_segment(files, "fastdds_");
        const auto fastrtps_segment = create_dead_segment(files, "fastrtps_");
        const auto fastdds_port = create_dead_port(files, "fastdds_");
        const auto fastrtps_port = create_dead_port(files, "fastrtps_");
        bool passed = EXPECT(!fastdds_segment.empty()) && EXPECT(!fastrtps_segment.empty()) &&
                      EXPECT(!fastdds_port.empty()) && EXPECT(!fastrtps_port.empty());
        if (!passed)
        {
            std::cerr << "reclaims_synthetic: could not lay out the fixture files\n";
            return 1;
        }

        const auto stats = provizio::dds::detail::sweep_dead_shared_memory_in(shm_dir());

        for (const auto &segment : {fastdds_segment, fastrtps_segment})
        {
            passed &= EXPECT(!exists(segment));
            passed &= EXPECT(!exists(segment + "_el"));
        }
        for (const auto &port : {fastdds_port, fastrtps_port})
        {
            passed &= EXPECT(!exists(port));
            passed &= EXPECT(!exists(port + "_el"));
            passed &= EXPECT(!exists("sem." + port + "_mutex"));
        }
        // The directory holds nothing but this case's fixtures, so the counts are exact.
        passed &= EXPECT(stats.segments == 2);
        passed &= EXPECT(stats.ports == 2);
        passed &= EXPECT(provizio::dds::detail::anything_reclaimed(stats));

        std::cout << "reclaims_synthetic: " << (passed ? "PASS" : "FAIL") << " (reclaimed " << stats.segments
                  << " segment(s), " << stats.ports << " port(s), " << stats.bytes << " bytes)" << '\n';
        return passed ? 0 : 1;
    }

    // Case: the real thing — a participant is SIGKILLed, and the next participant created in
    // this process reclaims what it left behind, before it creates its own.
    int test_reclaims_dead_participant()
    {
        // Lowered just enough to reach the child's files, which are older than that by the
        // time the sweep runs; a participant starting up elsewhere in a parallel ctest run is
        // still shielded by it. See k_shared_dir_min_age.
        constexpr int min_age = 1;
        set_env(k_min_age_env, k_shared_dir_min_age);

        {
            const participant_child child;
            if (!EXPECT(child.ready()))
            {
                std::cerr << "reclaims_dead_participant: the child never came up\n";
                return 1;
            }
        }  // SIGKILLed here — no cleanup of any kind runs in it.

        // Past the age guard — and past the stricter one reclaimable_objects scans with, or
        // the child's freshly-abandoned files would not make its list.
        constexpr auto scan_margin = std::chrono::milliseconds{500};
        std::this_thread::sleep_for(std::chrono::seconds{min_age + 1} + scan_margin);

        const auto before = reclaimable_objects(min_age);
        const bool any_segment =
            std::any_of(before.begin(), before.end(), [](const reclaimable_object &entry) { return !entry.is_port; });
        if (!EXPECT(!before.empty()) || !EXPECT(any_segment))
        {
            std::cerr << "reclaims_dead_participant: the killed child left no reclaimable segment behind\n";
            return 1;
        }

        // The trigger under test: creating a participant, nothing else. On a domain of its
        // own, so its own port files cannot be confused with the reclaimed ones.
        const auto participant =
            provizio::dds::make_domain_participant(k_sweeper_domain, provizio::dds::network_recovery_mode::off);

        bool passed = true;
        for (const auto &entry : before)
        {
            const std::string object_name = entry.lock_name.substr(0, entry.lock_name.size() - k_lock_suffix_length);
            const bool lock_gone = EXPECT(reclaimed(entry.lock_name, entry.lock_inode));
            passed &= lock_gone;
            passed &= EXPECT(reclaimed(object_name, entry.object_inode));
            if (!lock_gone)
            {
                std::cerr << "  " << entry.lock_name << " survived the sweep\n";
            }
        }

        std::cout << "reclaims_dead_participant: " << (passed ? "PASS" : "FAIL") << " (" << before.size()
                  << " reclaimable file(s) before the sweep)" << '\n';
        return passed ? 0 : 1;
    }

    // Case: a sweep must not disturb anything alive. Three live owners are exercised at once —
    // a synthetic lock file this process holds, the participant of a separate live process,
    // and this process's own participant — and the shared-memory data path between the last
    // two must keep carrying samples across the sweep.
    int test_spares_the_living()
    {
        // Low enough that everything here is old enough to be reclaimed on age by the time the
        // sweep runs, so only the locks can be what saves it. See k_shared_dir_min_age.
        constexpr int min_age = 1;
        set_env(k_min_age_env, k_shared_dir_min_age);

        fixture_files files;
        const std::string held = segment_name("fastdds_", "live");
        bool passed = EXPECT(files.add(held, 4096)) && EXPECT(files.add(held + "_el"));
        passed = passed && EXPECT(backdate(held, k_shared_dir_backdate_seconds)) &&
                 EXPECT(backdate(held + "_el", k_shared_dir_backdate_seconds));
        const scoped_flock lock{held + "_el"};
        passed = passed && EXPECT(lock.acquired());
        if (!passed)
        {
            std::cerr << "spares_the_living: could not lay out the fixture files\n";
            return 1;
        }

        // Stamped with THIS process's pid and handed to the child before it forks, so two
        // hosts running this case at the same time cannot feed each other's subscriber.
        const std::string topic = "provizio_dds_test_shm_cleanup_" + unique_tag();
        const participant_child child{topic};
        passed &= EXPECT(child.ready());

        std::mutex mutex;
        std::condition_variable received;
        int count = 0;
        const auto participants_created = std::chrono::steady_clock::now();
        const auto participant =
            provizio::dds::make_domain_participant(k_domain, provizio::dds::network_recovery_mode::off);
        const auto subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
            participant, topic, [&](const std_msgs::msg::String & /*message*/) {
                const std::lock_guard<std::mutex> guard{mutex};
                ++count;
                received.notify_all();
            });

        const auto wait_for_samples = std::chrono::seconds{20 * PROVIZIO_DDS_TEST_TIMEOUT_SCALE};
        constexpr int samples_before_sweep = 3;
        constexpr int samples_after_sweep = 5;
        const auto wait_for = [&](const int target) {
            std::unique_lock<std::mutex> guard{mutex};
            return received.wait_for(guard, wait_for_samples, [&] { return count >= target; });
        };
        passed &= EXPECT(wait_for(samples_before_sweep));

        // A corpse laid out right next to the living — and only now, after the participant
        // creation above has done its own once-per-process sweep — so the sweep below
        // provably did its rounds rather than bailing out early and "sparing" everything by
        // doing nothing at all.
        const auto corpse = create_dead_segment(files, "fastdds_", k_shared_dir_backdate_seconds);
        passed &= EXPECT(!corpse.empty());

        // Floor: both participants' files must be past the age guard, so nothing but their
        // locks stands between them and the sweep. Waiting for samples above normally covers
        // it already; this only matters on a host fast enough that it did not.
        std::this_thread::sleep_for(std::chrono::seconds{min_age + 1} -
                                    (std::chrono::steady_clock::now() - participants_created));

        const auto stats = provizio::dds::detail::sweep_dead_shared_memory();

        passed &= EXPECT(exists(held));
        passed &= EXPECT(exists(held + "_el"));
        passed &= EXPECT(!exists(corpse));
        passed &= EXPECT(provizio::dds::detail::anything_reclaimed(stats));
        passed &= EXPECT(child.alive());
        int count_at_sweep = 0;
        {
            const std::lock_guard<std::mutex> guard{mutex};
            count_at_sweep = count;
        }
        // The data path survives: samples keep arriving after the sweep.
        passed &= EXPECT(wait_for(count_at_sweep + samples_after_sweep));

        std::cout << "spares_the_living: " << (passed ? "PASS" : "FAIL") << " (" << count_at_sweep
                  << " samples before a sweep that reclaimed " << stats.segments << " segment(s), " << stats.ports
                  << " port(s))" << '\n';
        return passed ? 0 : 1;
    }

    // Case: the age guard. A lock file younger than the threshold is left alone however
    // unlocked it is — that is the window in which a participant has created its segment but
    // not yet taken its lock — while an older one is reclaimed.
    int test_age_guard()
    {
        const private_shm_dir directory{"age"};
        set_env(k_min_age_env, k_age_guard_min_age);

        fixture_files files;
        const std::string fresh = segment_name("fastdds_", "fres");
        const std::string old = segment_name("fastdds_", "dead");
        bool passed = EXPECT(directory.claimed()) && EXPECT(files.add(fresh, 4096)) && EXPECT(files.add(fresh + "_el"));
        passed = passed && EXPECT(files.add(old, 4096)) && EXPECT(files.add(old + "_el"));
        passed = passed && EXPECT(backdate(old, k_age_guard_backdate_seconds)) &&
                 EXPECT(backdate(old + "_el", k_age_guard_backdate_seconds));

        if (!passed)
        {
            std::cerr << "age_guard: could not lay out the fixture files\n";
            return 1;
        }

        provizio::dds::detail::sweep_dead_shared_memory_in(shm_dir());

        passed &= EXPECT(exists(fresh));
        passed &= EXPECT(exists(fresh + "_el"));
        passed &= EXPECT(!exists(old));
        passed &= EXPECT(!exists(old + "_el"));

        std::cout << "age_guard: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: PROVIZIO_DDS_SHM_CLEANUP=0 removes nothing at all, neither through a direct sweep
    // nor through the participant-creation trigger.
    int test_kill_switch()
    {
        const private_shm_dir directory{"kill"};
        set_env(k_min_age_env, k_no_age_guard);
        if (!EXPECT(directory.claimed()))
        {
            return 1;
        }

        bool passed = true;
        for (const char *const value : {"0", "off", "false", "no"})
        {
            set_env(k_enabled_env, value);

            fixture_files files;
            const auto segment = create_dead_segment(files, "fastdds_");
            const auto port = create_dead_port(files, "fastdds_");
            if (!EXPECT(!segment.empty()) || !EXPECT(!port.empty()))
            {
                std::cerr << "kill_switch: could not lay out the fixture files\n";
                return 1;
            }

            const auto stats = provizio::dds::detail::sweep_dead_shared_memory_in(shm_dir());
            passed &= EXPECT(!provizio::dds::detail::anything_reclaimed(stats));
            passed &= EXPECT(stats.bytes == 0);
            passed &= EXPECT(exists(segment));
            passed &= EXPECT(exists(segment + "_el"));
            passed &= EXPECT(exists(port));
            passed &= EXPECT(exists(port + "_el"));
            passed &= EXPECT(exists("sem." + port + "_mutex"));
            if (!passed)
            {
                std::cerr << "  " << k_enabled_env << "='" << value << "' did not switch the sweep off\n";
                return 1;
            }
        }

        // An unrecognised value must leave the sweep ENABLED — a typo must never silently
        // reintroduce the leak this exists to stop.
        set_env(k_enabled_env, "maybe");
        {
            fixture_files files;
            const auto corpse = create_dead_segment(files, "fastdds_");
            passed &= EXPECT(!corpse.empty());
            provizio::dds::detail::sweep_dead_shared_memory_in(shm_dir());
            passed &= EXPECT(!exists(corpse));
            passed &= EXPECT(!exists(corpse + "_el"));
        }

        // Nothing here creates a participant: that would sweep the host's real shared-memory
        // directory, where a corpse of this case's cannot be laid out without another test's
        // participant being free to reclaim it. The end-to-end property is covered by
        // composition instead — reclaims_dead_participant shows that creating a participant
        // runs the sweep, and the four spellings above show the sweep obeys the switch.

        std::cout << "kill_switch: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: everything in the shared-memory directory that is not one of the two recognised
    // lock-file shapes is left strictly alone — however old, however unlocked. Data-sharing
    // segments matter most (a live reader may map a dead writer's, so liveness there means
    // something else entirely), but so does a stranger's file that merely starts with
    // "fastdds_", and so does an object whose lock file is already gone: the algorithm is
    // driven by lock files and deliberately has nothing to say about an object without one
    // (a port opened for writing never gets one, and may well be in use).
    int test_leaves_other_files_alone()
    {
        const private_shm_dir directory{"decoys"};
        set_env(k_min_age_env, k_no_age_guard);
        const auto tag = unique_tag();

        fixture_files files;
        const std::vector<std::string> decoys{
            "provizio_dds_shm_cleanup_decoy_" + tag,  // Nothing to do with Fast-DDS.
            "fast_datasharing_" + tag + "_1.2.3",     // Data-sharing: never ours to judge.
            "fastdds_odd" + tag + "_el",              // 15-character id — not a segment name.
            "fastdds_port1234567_el",                 // 7 digits — not a port name.
            "fastdds_dead" + tag + "_ex",             // Neither "_el" nor "_sl".
            "fastdds_dead" + tag,                     // A segment object with no lock file.
            "sem.fastdds_port" + std::to_string(candidate_port(0)) + "_mutex",  // Semaphore, port long gone.
        };
        bool passed = EXPECT(directory.claimed());
        for (const auto &decoy : decoys)
        {
            passed &= EXPECT(files.add(decoy, 128));
        }
        if (!passed)
        {
            std::cerr << "leaves_other_files_alone: could not lay out the fixture files\n";
            return 1;
        }

        provizio::dds::detail::sweep_dead_shared_memory_in(shm_dir());

        for (const auto &decoy : decoys)
        {
            passed &= EXPECT(exists(decoy));
            if (!exists(decoy))
            {
                std::cerr << "  " << decoy << " was removed\n";
            }
        }

        std::cout << "leaves_other_files_alone: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: a lock file is trusted only when it is one Fast-DDS could actually have written.
    // Whoever can write to this world-writable directory could otherwise plant one beside a
    // LIVE object and have the sweep delete it — the flock proves only that the LOCK FILE's
    // owner is gone, and the object is then reached from its name. Three plants are refused
    // here, with a genuine corpse alongside them to prove the sweep still ran.
    int test_rejects_planted_lock_files()
    {
        const private_shm_dir directory{"planted"};
        set_env(k_min_age_env, k_no_age_guard);

        fixture_files files;
        bool passed = EXPECT(directory.claimed());

        // 1. A segment with a SHARED lock file. Fast-DDS gives a segment an exclusive lock
        //    and nothing else, so this name stays free for the taking while the segment is
        //    alive — the cheapest way to aim a sweep at a live participant.
        const std::string shared_suffix = segment_name("fastdds_", "shrd");
        passed &= EXPECT(files.add(shared_suffix, 4096));
        passed &= EXPECT(files.add(shared_suffix + "_sl"));

        // 2. A lock file with content. Fast-DDS opens its lock files and never writes them,
        //    so anything non-empty was written by something else.
        const std::string written = segment_name("fastdds_", "wrtn");
        passed &= EXPECT(files.add(written, 4096));
        passed &= EXPECT(files.add(written + "_el", 1));

        // 3. A lock file hardlinked onto the object itself — the way to make a planted lock
        //    file whose ownership matches its victim's.
        const std::string linked = segment_name("fastdds_", "hlnk");
        passed &= EXPECT(files.add(linked, 4096));
        passed &= EXPECT(hardlink(linked, linked + "_el"));

        // 4. The control: a real corpse, and specifically a port with a SHARED lock, which
        //    Fast-DDS does create and which nothing else in the suite covers.
        const auto corpse = create_dead_port(files, "fastdds_", "_sl");
        passed &= EXPECT(!corpse.empty());
        if (!passed)
        {
            std::cerr << "rejects_planted_lock_files: could not lay out the fixture files\n";
            return 1;
        }

        const auto stats = provizio::dds::detail::sweep_dead_shared_memory_in(shm_dir());

        passed &= EXPECT(exists(shared_suffix));
        passed &= EXPECT(exists(shared_suffix + "_sl"));
        passed &= EXPECT(exists(written));
        passed &= EXPECT(exists(written + "_el"));
        passed &= EXPECT(exists(linked));
        passed &= EXPECT(exists(linked + "_el"));
        // The control was taken, so the sweep did run over all of the above.
        passed &= EXPECT(!exists(corpse));
        passed &= EXPECT(!exists(corpse + "_sl"));
        passed &= EXPECT(!exists("sem." + corpse + "_mutex"));
        passed &= EXPECT(stats.segments == 0);
        passed &= EXPECT(stats.ports == 1);

        std::cout << "rejects_planted_lock_files: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }

    // Case: the pressure-triggered sweep is rate-limited, so a process creating participants
    // in a loop on a full host rescans the directory occasionally rather than constantly.
    int test_rate_limited()
    {
        // No private directory here: cleanup_shared_memory_if_due is the production entry
        // point and always sweeps the platform's directory, so the fixtures live there under
        // the shared-directory guard choreography.
        set_env(k_min_age_env, k_shared_dir_min_age);

        fixture_files files;
        const auto first = create_dead_segment(files, "fastdds_", k_shared_dir_backdate_seconds);
        if (!EXPECT(!first.empty()))
        {
            std::cerr << "rate_limited: could not lay out the fixture files\n";
            return 1;
        }

        bool passed = EXPECT(provizio::dds::detail::cleanup_shared_memory_if_due());
        passed &= EXPECT(!exists(first));

        // A second corpse appearing immediately afterwards is NOT swept: the rate limit holds
        // for 30 s, far longer than this case runs.
        const auto second = create_dead_segment(files, "fastrtps_", k_shared_dir_backdate_seconds);
        passed &= EXPECT(!second.empty());
        passed &= EXPECT(!provizio::dds::detail::cleanup_shared_memory_if_due());
        passed &= EXPECT(exists(second));
        passed &= EXPECT(exists(second + "_el"));

        std::cout << "rate_limited: " << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    }
}  // namespace

int main(int argc, char **argv)
{
    const std::vector<std::string_view> args(
        argv, std::next(argv, argc));  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    if (args.size() < 2)
    {
        std::cerr << "usage: " << args[0] << " <subcommand>\n";
        return 1;
    }
    const std::string_view subcommand = args[1];

    // Hermetic: the suite's loopback XML profile replaces the built-in transports with a
    // single UDPv4 one, so a participant created under it never touches shared memory at all
    // — and every case below is about shared-memory files. Clear it, as the discovery_tuning
    // and transport_tuning cases do. Cross-host discovery is harmless here: the one case that
    // exchanges samples uses a domain and a pid-stamped topic of its own.
    unset_env("FASTDDS_DEFAULT_PROFILES_FILE");
    // The default, made explicit so the ambient environment cannot switch the sweep off under
    // a case that asserts it happened (kill_switch sets it itself).
    set_env("PROVIZIO_DDS_SHM_CLEANUP", "1");

    if (subcommand == "reclaims_synthetic")
    {
        return test_reclaims_synthetic();
    }
    if (subcommand == "reclaims_dead_participant")
    {
        return test_reclaims_dead_participant();
    }
    if (subcommand == "spares_the_living")
    {
        return test_spares_the_living();
    }
    if (subcommand == "age_guard")
    {
        return test_age_guard();
    }
    if (subcommand == "kill_switch")
    {
        return test_kill_switch();
    }
    if (subcommand == "leaves_other_files_alone")
    {
        return test_leaves_other_files_alone();
    }
    if (subcommand == "rejects_planted_lock_files")
    {
        return test_rejects_planted_lock_files();
    }
    if (subcommand == "rate_limited")
    {
        return test_rate_limited();
    }
    std::cerr << "unknown subcommand: " << subcommand << "\n";
    return 1;
}
