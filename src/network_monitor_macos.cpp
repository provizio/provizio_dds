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

#include "provizio/dds/detail/network_monitor.h"

#if defined(__APPLE__)

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <fcntl.h>
#include <net/if.h>
#include <net/route.h>
#include <stdexcept>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include <atomic>
#include <system_error>
#include <vector>

#include "provizio/dds/logging.h"

namespace provizio::dds::detail
{
    namespace
    {
        // macOS has neither SOCK_CLOEXEC nor pipe2(); set FD_CLOEXEC after
        // the fact so a host process that fork+execs doesn't leak our
        // routing socket or shutdown-pipe fds to the child.
        inline void set_cloexec(int fd) noexcept
        {
            if (fd < 0)
            {
                return;
            }
            const int flags = ::fcntl(fd, F_GETFD, 0);
            if (flags >= 0)
            {
                ::fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
            }
        }
    }  // namespace

    struct network_monitor::impl
    {
        int route_fd{-1};
        int stop_pipe[2]{-1, -1};
        std::thread worker;
        std::atomic<bool> stopping{false};
        // Raised by the constructor, cleared by the worker when it leaves its loop —
        // see network_monitor::is_alive() and the Linux backend's equivalent.
        std::atomic<bool> alive{false};
        on_event_callback callback;

        bool open_channel()
        {
            route_fd = ::socket(PF_ROUTE, SOCK_RAW, 0);
            if (route_fd < 0)
            {
                return false;
            }
            set_cloexec(route_fd);
            if (::pipe(stop_pipe) != 0)
            {
                ::close(route_fd);
                route_fd = -1;
                return false;
            }
            set_cloexec(stop_pipe[0]);
            set_cloexec(stop_pipe[1]);
            return true;
        }

        // Closes the channel even when the object is destroyed without
        // ~network_monitor having run — the constructor can throw after open_channel()
        // succeeded (std::thread construction may fail under thread pressure), and
        // ensure_monitor_alive() retries periodically, so leaking the routing socket
        // and both pipe ends per attempt would eventually exhaust the fd table.
        ~impl()
        {
            close_channel();
        }

        impl() = default;
        impl(const impl &) = delete;
        impl(impl &&) = delete;
        impl &operator=(const impl &) = delete;
        impl &operator=(impl &&) = delete;

        void close_channel()
        {
            if (route_fd >= 0)
            {
                ::close(route_fd);
                route_fd = -1;
            }
            for (int &fd : stop_pipe)
            {
                if (fd >= 0)
                {
                    ::close(fd);
                    fd = -1;
                }
            }
        }

        void run()
        {
            run_loop();
            alive.store(false, std::memory_order_release);
        }

        void run_loop()
        {
            std::vector<char> buffer(2048);
            pollfd fds[2];
            fds[0].fd = route_fd;
            fds[0].events = POLLIN;
            fds[1].fd = stop_pipe[0];
            fds[1].events = POLLIN;

            while (!stopping.load(std::memory_order_acquire))
            {
                fds[0].revents = 0;
                fds[1].revents = 0;
                const int n = ::poll(fds, 2, -1);
                if (n < 0)
                {
                    if (errno == EINTR)
                    {
                        continue;
                    }
                    log_error() << "network_monitor: poll() failed: " << std::system_category().message(errno);
                    return;
                }
                if ((fds[1].revents & POLLIN) != 0)
                {
                    // Stop signal — drain pipe and exit. Return value
                    // intentionally ignored (see Linux equivalent).
                    char drained{};
                    [[maybe_unused]] const ssize_t ignored = ::read(stop_pipe[0], &drained, 1);
                    return;
                }
                if ((fds[0].revents & POLLIN) == 0)
                {
                    continue;
                }

                const ssize_t got = ::recv(route_fd, buffer.data(), buffer.size(), 0);
                // Every routing message, whatever its type, begins with the same three
                // fields: rtm_msglen, rtm_version, rtm_type. Nothing past those is read
                // before the type-specific length check below.
                if (got < static_cast<ssize_t>(offsetof(rt_msghdr, rtm_index)))
                {
                    if (got < 0 && errno != EAGAIN && errno != EINTR)
                    {
                        // Silent: a failed read of one routing message costs at most a
                        // delayed detection, which the periodic re-check covers.
                    }
                    continue;
                }
                const auto *hdr = reinterpret_cast<const rt_msghdr *>(buffer.data());
                // The header a message carries depends on its type: an address event is an
                // ifa_msghdr (20 bytes on macOS) followed by its sockaddrs, an interface event
                // an if_msghdr, and only a route entry an rt_msghdr -- which, with its
                // rt_metrics, is 92 bytes. Measuring every message against rt_msghdr, as this
                // code once did, threw away every RTM_NEWADDR and RTM_DELADDR the kernel sent
                // (a real one for en0 is 80 bytes) and left address changes to the periodic
                // re-check, tens of seconds late.
                std::size_t expected_header = sizeof(rt_msghdr);
                if (hdr->rtm_type == RTM_NEWADDR || hdr->rtm_type == RTM_DELADDR)
                {
                    expected_header = sizeof(ifa_msghdr);
                }
                else if (hdr->rtm_type == RTM_IFINFO)
                {
                    expected_header = sizeof(if_msghdr);
                }
                if (hdr->rtm_msglen > got || hdr->rtm_msglen < expected_header)
                {
                    // Silent: a malformed routing message is skipped; the periodic re-check
                    // still observes whatever the kernel ended up with.
                    continue;
                }
                // Address changes plus RTM_IFINFO, which is how a carrier / interface-flag
                // transition arrives. RTM_IFINFO matters because capture_address_snapshot
                // only admits IFF_RUNNING interfaces (as Fast-DDS' IPFinder does), so
                // plugging a cable back in changes the snapshot without any address event.
                // The coordinator's snapshot diff absorbs the extra wake-ups.
                if (hdr->rtm_type == RTM_NEWADDR || hdr->rtm_type == RTM_DELADDR || hdr->rtm_type == RTM_IFINFO)
                {
                    if (callback)
                    {
                        callback();
                    }
                }
            }
        }
    };

    network_monitor::network_monitor(on_event_callback callback) : the_impl(std::make_unique<impl>())
    {
        if (!the_impl->open_channel())
        {
            const std::string err = std::system_category().message(errno);
            throw std::runtime_error{"network_monitor: failed to open PF_ROUTE socket: " + err};
        }
        the_impl->callback = std::move(callback);
        the_impl->alive.store(true, std::memory_order_release);
        the_impl->worker = std::thread{[impl_ptr = the_impl.get()] { impl_ptr->run(); }};
    }

    bool network_monitor::is_alive() const noexcept
    {
        return the_impl && the_impl->alive.load(std::memory_order_acquire);
    }

    bool network_monitor::kill_for_test() noexcept
    {
        if (!the_impl || the_impl->stop_pipe[1] < 0)
        {
            return false;
        }
        // Deliberately does NOT set `stopping`: the point is to leave the monitor object
        // in the state a dead worker leaves it in, which is what is_alive() reports on.
        // The write result IS propagated: a caller told "signalled" must be able to rely
        // on the worker actually waking, or a failed wake surfaces as a test timeout with
        // no explanation.
        const char one = 1;
        return ::write(the_impl->stop_pipe[1], &one, 1) == 1;
    }

    network_monitor::~network_monitor()
    {
        if (!the_impl)
        {
            return;
        }
        the_impl->stopping.store(true, std::memory_order_release);
        if (the_impl->stop_pipe[1] >= 0)
        {
            // Wake the worker. Return value intentionally ignored
            // (see Linux equivalent).
            const char one = 1;
            [[maybe_unused]] const ssize_t ignored = ::write(the_impl->stop_pipe[1], &one, 1);
        }
        if (the_impl->worker.joinable())
        {
            the_impl->worker.join();
        }
        the_impl->close_channel();
    }
}  // namespace provizio::dds::detail

#endif  // __APPLE__
