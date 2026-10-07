// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy of
// the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.
//
// Guards cmake/fast_dds/local_reader_under_writer_mutex.cmake, which makes
// StatefulWriter::intraprocess_heartbeat take the writer's mutex before it reads the reader
// proxy's ReaderLocator::local_reader_ (eProsima/Fast-DDS#6422, backported). Unpatched, the
// heartbeat -- which every match with a same-process reader schedules on the timer thread --
// reads that shared_ptr without the mutex, while ReaderLocator::stop() resets it under the mutex
// as the reader or the writer is deleted; local_reader() re-reads the pointer after testing it,
// and a reset in between is the null dereference that crashed
// network_recovery_concurrent_make_publisher_during_reset.
//
// The real window is a few instructions wide, so rather than hope to land in it this forces the
// order the patch exists to rule out. It holds the writer's mutex, starts a heartbeat, and --
// once the heartbeat has got as far as it can -- does what deleting the DataReader does under that
// same mutex: matched_reader_remove, which stops the proxy. Patched, the heartbeat was waiting for
// the mutex before it looked the reader up, so it looks it up after the removal, finds nothing and
// delivers nothing. Unpatched, it looked the reader up first, and after the removal delivers a
// heartbeat to a reader the writer no longer matches, which is the observable difference asserted
// below (and, under ThreadSanitizer, the very race it reports).
//
// Either way the heartbeat ends up waiting for the mutex, and that is what the removal waits for:
// on Linux, until /proc shows the heartbeat's thread in a futex wait on a word inside the writer's
// mutex -- not merely asleep, as it can be in a lock the unpatched lookup takes on its way, which
// would let a regression through -- so the order is certain. Elsewhere this reads no thread's
// state, and the heartbeat is given a head start instead, from the moment its thread runs.
//
// Built only where Fast-DDS is built from source, since that is the only configuration the patch
// is applied in -- see the CMakeLists. It reaches into Fast-DDS' PRIVATE headers deliberately:
// the patch reaches into those same sources, and a FAST_DDS_VERSION bump that moves them has to
// re-check this patch anyway (see AGENTS.md).

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#if defined(__linux__)
#include <cstddef>
#include <fstream>
#include <optional>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include <fastdds/rtps/common/Guid.hpp>
#include <fastdds/utils/TimedMutex.hpp>
#include <std_msgs/msg/StringPubSubTypes.hpp>

#include "provizio/dds/domain_participant.h"
#include "provizio/dds/publisher.h"
#include "provizio/dds/subscriber.h"
#include "rtps/domain/IDomainImpl.hpp"
#include "rtps/writer/StatefulWriter.hpp"

#include "../detail/test_domain.h"

// The one member of Fast-DDS' RTPSDomainImpl this needs: its instance, whose find_writer maps the
// DataWriter's GUID to the StatefulWriter behind it. Declared rather than included because
// RTPSDomainImpl.hpp drags in Fast-DDS' third-party headers (FileWatch, Boost.Interprocess), which
// are not on this target's include path; only the mangled name has to match, the way
// network_recovery_test.cpp reaches SystemInfo.
namespace eprosima::fastdds::rtps
{
    // NOLINTBEGIN(readability-identifier-naming,cppcoreguidelines-special-member-functions)
    // -- the name must match Fast-DDS' class verbatim for the static member to resolve.
    class RTPSDomainImpl
    {
      public:
        static std::shared_ptr<IDomainImpl> get_instance();
    };
    // NOLINTEND(readability-identifier-naming,cppcoreguidelines-special-member-functions)
}  // namespace eprosima::fastdds::rtps

namespace
{
    constexpr auto k_match_timeout = std::chrono::seconds{10};
    constexpr auto k_heartbeat_timeout = std::chrono::seconds{10};

    int fail(const std::string &reason)
    {
        std::cout << "local_reader_lock: FAIL (" << reason << ")" << '\n';
        return 1;
    }

#if defined(__linux__)
    // The calling thread's id, as /proc/self/task names it
    std::int64_t current_thread_id()
    {
        return static_cast<std::int64_t>(::gettid());
    }

    // Whether thread <thread_id> of this process is in a futex wait on a word inside the <size> bytes
    // at <object>, as one blocked on a mutex is on one inside the mutex: the system call
    // /proc/self/task/<thread_id>/syscall says the thread is in, with its first argument, the futex's
    // address, "running" for a thread in none. Empty where that cannot be read.
    std::optional<bool> thread_waits_inside(std::int64_t thread_id, const void *object, std::size_t size)
    {
        std::ifstream syscall_file{"/proc/self/task/" + std::to_string(thread_id) + "/syscall"};
        std::string number;
        std::string first_argument;
        if (!(syscall_file >> number))
        {
            return std::nullopt;
        }
        bool is_futex = number == std::to_string(SYS_futex);
#if defined(SYS_futex_time64)
        is_futex = is_futex || number == std::to_string(SYS_futex_time64);
#endif
        if (!is_futex || !(syscall_file >> first_argument))
        {
            return false;
        }
        char *end = nullptr;
        const std::uintptr_t address = std::strtoull(first_argument.c_str(), &end, 16);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) -- an address, compared as one
        const auto begin = reinterpret_cast<std::uintptr_t>(object);
        return end != first_argument.c_str() && address >= begin && address < begin + size;
    }
#else
    // How long the heartbeat is given, from the moment its thread runs, to reach the writer's mutex
    // before the reader is removed under it. Only the unpatched order depends on it (the heartbeat
    // must have read the proxy's local reader by then to show the defect), so a slow runner can
    // only make this case miss a regression, never fail a correct build.
    constexpr auto k_heartbeat_head_start = std::chrono::milliseconds{500};

    std::int64_t current_thread_id()
    {
        return 0;
    }
#endif
}  // namespace

int main()
{
    using eprosima::fastdds::RecursiveTimedMutex;
    using eprosima::fastdds::rtps::GUID_t;
    using eprosima::fastdds::rtps::ReaderProxy;
    using eprosima::fastdds::rtps::RTPSDomainImpl;
    using eprosima::fastdds::rtps::StatefulWriter;

    // No recovery: nothing may rebuild the endpoints this case holds raw Fast-DDS pointers into.
    const auto participant = provizio::dds::make_domain_participant(provizio::dds::test::random_test_domain(),
                                                                    provizio::dds::network_recovery_mode::off);
    const std::string topic_name{"provizio_dds_local_reader_lock_test"};
    // Both RELIABLE: a reliable writer is a StatefulWriter (a best-effort one would be stateless
    // and schedule no heartbeat), and an explicit reader reliability builds the DataReader at once
    // instead of deferring it until a writer is discovered.
    const auto publisher = provizio::dds::make_publisher<std_msgs::msg::StringPubSubType>(
        participant, topic_name, eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS);
    const auto subscriber = provizio::dds::make_subscriber<std_msgs::msg::StringPubSubType>(
        participant, topic_name, [](const std_msgs::msg::String &) {},
        eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS);

    if (publisher->get_num_matched_subscribers(k_match_timeout, std::chrono::milliseconds{0}) < 1)
    {
        return fail("the publisher never matched the subscriber of its own participant");
    }

    const GUID_t writer_guid = publisher->get_guid();
    GUID_t reader_guid = subscriber->get_guid();
    auto *const writer = dynamic_cast<StatefulWriter *>(RTPSDomainImpl::get_instance()->find_writer(writer_guid));
    if (writer == nullptr)
    {
        return fail("the DataWriter is not backed by a StatefulWriter");
    }
    ReaderProxy *proxy = nullptr;
    if (!writer->matched_reader_lookup(reader_guid, &proxy) || proxy == nullptr)
    {
        return fail("the StatefulWriter holds no proxy for the matched reader");
    }

    // The control: with nothing in its way, a heartbeat reaches the reader. Without this, the
    // assertion at the end could pass merely because intraprocess delivery was off.
    if (!writer->intraprocess_heartbeat(proxy, /*liveliness=*/true))
    {
        return fail("a heartbeat to a matched reader of the same process was not delivered, so this case "
                    "cannot tell anything apart: is intraprocess delivery disabled?");
    }

    std::promise<std::int64_t> heartbeat_running;
    auto heartbeat_thread = heartbeat_running.get_future();
    std::future<bool> heartbeat;
    {
        const std::lock_guard<RecursiveTimedMutex> lock{writer->getMutex()};
        heartbeat = std::async(std::launch::async, [writer, proxy, &heartbeat_running] {
            heartbeat_running.set_value(current_thread_id());
            return writer->intraprocess_heartbeat(proxy, /*liveliness=*/true);
        });
        if (heartbeat_thread.wait_for(k_heartbeat_timeout) != std::future_status::ready)
        {
            return fail("the heartbeat's thread never ran");
        }
#if defined(__linux__)
        const std::int64_t heartbeat_thread_id = heartbeat_thread.get();
        const auto deadline = std::chrono::steady_clock::now() + k_heartbeat_timeout;
        while (true)
        {
            const auto waits =
                thread_waits_inside(heartbeat_thread_id, &writer->getMutex(), sizeof(RecursiveTimedMutex));
            if (waits.value_or(false))
            {
                break;
            }
            if (heartbeat.wait_for(std::chrono::seconds{0}) == std::future_status::ready)
            {
                return fail("the heartbeat returned without ever waiting for the writer mutex, which was held");
            }
            if (!waits.has_value())
            {
                return fail("/proc/self/task/<thread>/syscall could not be read, so whether the heartbeat waits for "
                            "the writer mutex cannot be told");
            }
            if (std::chrono::steady_clock::now() > deadline)
            {
                return fail("the heartbeat never came to wait for the writer mutex");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
#else
        // The thread id is of no use here, where no thread's state is read
        std::this_thread::sleep_for(k_heartbeat_head_start);
#endif
        // What deleting the DataReader does to the writer: take the proxy out of the matched set
        // and stop it, which resets its local reader. matched_reader_remove takes the writer mutex
        // itself, and the mutex is recursive, so calling it with the mutex already held changes
        // nothing but the timing: the removal lands while the heartbeat is still waiting.
        writer->matched_reader_remove(reader_guid);
    }

    if (heartbeat.wait_for(k_heartbeat_timeout) != std::future_status::ready)
    {
        std::cout << "local_reader_lock: FAIL (the heartbeat never returned once the writer mutex was released)"
                  << '\n';
        std::abort();  // A thread stuck inside Fast-DDS would otherwise hang the whole test binary.
    }
    if (heartbeat.get())
    {
        return fail("a heartbeat delivered to a reader removed while it waited for the writer mutex: it read "
                    "ReaderLocator::local_reader_ before taking that mutex, so "
                    "cmake/fast_dds/local_reader_under_writer_mutex.cmake is not in effect");
    }

    std::cout << "local_reader_lock: PASS" << '\n';
    return 0;
}
