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

#if defined(__linux__)

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <linux/rtnetlink.h>
#include <stdexcept>
#include <sys/eventfd.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include <array>
#include <atomic>
#include <system_error>
#include <vector>

#include "provizio/dds/logging.h"

namespace provizio::dds::detail
{
    // Internal pimpl — public members are intentional (private accessors
    // here would add no encapsulation; the type is invisible outside this TU).
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    struct network_monitor::impl
    {
        int netlink_fd{-1};
        int stop_event_fd{-1};
        std::thread worker;
        std::atomic<bool> stopping{false};
        on_event_callback callback;
        // NOLINTEND(misc-non-private-member-variables-in-classes)

        bool open_channel()
        {
            netlink_fd = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
            if (netlink_fd < 0)
            {
                return false;
            }

            sockaddr_nl addr{};
            addr.nl_family = AF_NETLINK;
            // Subscribe ONLY to address-add/del groups; link-state and route-change
            // groups are deliberately ignored (link flap during boot is a known noise
            // source, and DDS transport binding only cares about address presence).
            addr.nl_groups = RTMGRP_IPV4_IFADDR | RTMGRP_IPV6_IFADDR;
            // sockaddr_nl → sockaddr is the canonical BSD-sockets idiom.
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            if (::bind(netlink_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
            {
                ::close(netlink_fd);
                netlink_fd = -1;
                return false;
            }

            stop_event_fd = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
            if (stop_event_fd < 0)
            {
                ::close(netlink_fd);
                netlink_fd = -1;
                return false;
            }

            return true;
        }

        void close_channel()
        {
            if (netlink_fd >= 0)
            {
                ::close(netlink_fd);
                netlink_fd = -1;
            }
            if (stop_event_fd >= 0)
            {
                ::close(stop_event_fd);
                stop_event_fd = -1;
            }
        }

        void run() const
        {
            constexpr std::size_t recv_buffer_bytes = std::size_t{8} * 1024;
            std::vector<char> buffer(recv_buffer_bytes);
            std::array<pollfd, 2> fds{};
            fds[0].fd = netlink_fd;
            fds[0].events = POLLIN;
            fds[1].fd = stop_event_fd;
            fds[1].events = POLLIN;

            while (!stopping.load(std::memory_order_acquire))
            {
                fds[0].revents = 0;
                fds[1].revents = 0;
                const int ready = ::poll(fds.data(), fds.size(), -1);
                if (ready < 0)
                {
                    if (errno == EINTR)
                    {
                        continue;
                    }
                    log_error() << "network_monitor: poll() failed: " << std::system_category().message(errno);
                    return;
                }

                // POLLIN is a signed int macro; the bitwise check needs an
                // unsigned operand to satisfy hicpp-signed-bitwise.
                if ((static_cast<unsigned>(fds[1].revents) & static_cast<unsigned>(POLLIN)) != 0U)
                {
                    // Stop signal — drain eventfd and exit. Return value
                    // intentionally ignored: we're about to exit the
                    // worker regardless, and the read is only there to
                    // empty the eventfd's counter so the kernel marks it
                    // not-readable again (cosmetic — the fd will be
                    // closed in close_channel anyway).
                    std::uint64_t drained{};
                    [[maybe_unused]] const ssize_t ignored = ::read(stop_event_fd, &drained, sizeof(drained));
                    return;
                }

                if ((static_cast<unsigned>(fds[0].revents) & static_cast<unsigned>(POLLIN)) == 0U)
                {
                    continue;
                }

                // A larger-than-buffer datagram is truncated by the kernel
                // (we did not pass MSG_TRUNC). For our purposes this is
                // benign: every burst still ends in a coalescer-driven
                // capture_address_snapshot() which reads the current state
                // directly — the netlink message body is used only as an
                // "anything happened" trigger and the type check below
                // already only inspects nlmsghdr fields which fit in the
                // truncated head. If diagnostic visibility is ever needed,
                // pass MSG_TRUNC and log when (got > buffer.size()).
                const ssize_t got = ::recv(netlink_fd, buffer.data(), buffer.size(), 0);
                if (got == 0)
                {
                    // A netlink recv() does not return 0 in normal operation, and
                    // errno is not set on a zero return — so don't fall through to
                    // the errno branches below, where a stale errno could misreport
                    // the cause or (if it were EINTR/EAGAIN) spin in a tight loop on
                    // a socket that will never deliver data. Treat it as terminal,
                    // like the recv() failure path.
                    log_error() << "network_monitor: recv() on netlink socket returned 0; "
                                   "network auto-recovery disabled for this process";
                    return;
                }
                if (got < 0)
                {
                    if (errno == EINTR || errno == EAGAIN)
                    {
                        continue;
                    }
                    if (errno == ENOBUFS)
                    {
                        // ENOBUFS is the transient "kernel-side multicast
                        // socket buffer overflowed" condition that hits when
                        // address events arrive faster than userspace drains.
                        // The kernel drops the unread events; subsequent recv
                        // calls succeed normally. Log and keep monitoring —
                        // the coalescer's snapshot capture will pick up
                        // whatever state the kernel ends up in.
                        log_warning() << "network_monitor: netlink recv lost events (ENOBUFS); continuing";
                        continue;
                    }
                    log_error() << "network_monitor: recv() on netlink socket failed: "
                                << std::system_category().message(errno)
                                << "; network auto-recovery disabled for this process";
                    return;
                }

                // We don't actually need to parse the rtnetlink message body — every
                // RTM_NEWADDR / RTM_DELADDR is a candidate event, and the coalescer
                // upstream applies all the policy (snapshot-diff, etc). We just need
                // to fire the callback at most once per arrival. NLMSG_NEXT mutates
                // its second argument so it must be a non-const lvalue.
                bool any_addr_event = false;
                // Unsigned to avoid the signed/unsigned comparison inside
                // NLMSG_OK on newer kernel headers; got > 0 was already
                // verified above so the cast is safe.
                auto remaining = static_cast<unsigned int>(got);
                // Raw byte buffer -> nlmsghdr* is the canonical netlink idiom;
                // NLMSG_OK / NLMSG_NEXT require a non-const nlmsghdr*.
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                for (auto *nh = reinterpret_cast<nlmsghdr *>(buffer.data()); NLMSG_OK(nh, remaining);
                     nh = NLMSG_NEXT(nh, remaining))
                {
                    if (nh->nlmsg_type == NLMSG_DONE)
                    {
                        break;
                    }
                    if (nh->nlmsg_type == NLMSG_ERROR)
                    {
                        continue;
                    }
                    if (nh->nlmsg_type == RTM_NEWADDR || nh->nlmsg_type == RTM_DELADDR)
                    {
                        any_addr_event = true;
                        break;
                    }
                }

                if (any_addr_event && callback)
                {
                    callback();
                }
            }
        }
    };

    network_monitor::network_monitor(on_event_callback callback) : the_impl(std::make_unique<impl>())
    {
        if (!the_impl->open_channel())
        {
            const std::string err = std::system_category().message(errno);
            throw std::runtime_error{"network_monitor: failed to open NETLINK_ROUTE socket: " + err};
        }
        the_impl->callback = std::move(callback);
        the_impl->worker = std::thread{[impl_ptr = the_impl.get()] { impl_ptr->run(); }};
    }

    network_monitor::~network_monitor()
    {
        if (!the_impl)
        {
            return;
        }
        the_impl->stopping.store(true, std::memory_order_release);
        if (the_impl->stop_event_fd >= 0)
        {
            // Wake the worker. Return value intentionally ignored — a
            // partial write or transient failure still leaves the worker
            // to discover stopping=true on its next poll cycle (the
            // eventfd write here is just to short-circuit the wait).
            const std::uint64_t one = 1;
            [[maybe_unused]] const ssize_t ignored = ::write(the_impl->stop_event_fd, &one, sizeof(one));
        }
        if (the_impl->worker.joinable())
        {
            the_impl->worker.join();
        }
        the_impl->close_channel();
    }
}  // namespace provizio::dds::detail

#endif  // __linux__
