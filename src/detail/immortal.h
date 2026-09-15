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

#ifndef DDS_DETAIL_IMMORTAL
#define DDS_DETAIL_IMMORTAL

#include <memory>
#include <utility>

namespace provizio::dds::detail
{
    /**
     * @brief Holds an object that is constructed once and never destroyed.
     *
     * For state with static storage duration that a background thread may still be reading
     * when the process exits. The network-recovery coordinator is a function-local static
     * whose monitor and coalescer threads are joined only when it is destroyed, at exit; the
     * tables those threads read (the force-included interface names, the excluded interface
     * kinds, the VPN classifier's kinds and override names, the test substitutions) are first
     * constructed by the first snapshot, i.e. AFTER the coordinator. Statics are destroyed in
     * reverse order of construction, so as plain statics every one of them was destroyed
     * BEFORE the coordinator joined the threads reading it -- a use-after-free on any process
     * whose network changes in the last milliseconds of its life, which ThreadSanitizer caught
     * in CI (main thread in a set's destructor, monitor thread in
     * snapshot_policy_excludes_interface).
     *
     * The rule this enforces: state with static storage duration that a background thread
     * reads must be trivially destructible or immortal. An immortal<T> has a trivial
     * destructor, so a function-local static one registers no exit-time destructor and its
     * object lives until the process ends. The object is allocated once and stays reachable
     * from the holder for the life of the process, so a leak checker does not report it.
     *
     * Deliberately not a way to keep a plain static reference: a `static T &x = *new T` is
     * flagged as a non-const global by cppcoreguidelines-avoid-non-const-global-variables,
     * where a function-local static object is not. Only for process-lifetime state; never
     * for anything whose destructor has work to do.
     *
     * @tparam value_type The held type. Default-constructed by the default constructor
     *         (std::mutex and the other non-movable types), or move-constructed from a value.
     */
    template <typename value_type> class immortal final
    {
      public:
        /// Default-constructs the held object.
        immortal() : object(std::make_unique<value_type>().release())
        {
        }

        /// Move-constructs the held object from @p value.
        explicit immortal(value_type value) : object(std::make_unique<value_type>(std::move(value)).release())
        {
        }

        /// Trivial on purpose: the held object is never destroyed (see the class comment).
        ~immortal() = default;
        immortal(const immortal &) = delete;
        immortal &operator=(const immortal &) = delete;
        immortal(immortal &&) = delete;
        immortal &operator=(immortal &&) = delete;

        [[nodiscard]] value_type &operator*() noexcept
        {
            return *object;
        }

        [[nodiscard]] const value_type &operator*() const noexcept
        {
            return *object;
        }

      private:
        value_type *object;
    };
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_IMMORTAL
