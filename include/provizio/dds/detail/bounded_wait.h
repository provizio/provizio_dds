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

#ifndef DDS_DETAIL_BOUNDED_WAIT
#define DDS_DETAIL_BOUNDED_WAIT

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>

namespace provizio::dds::detail
{
    /**
     * @file bounded_wait.h
     * @brief Deadline arithmetic and condition-variable waits that stay correct for the
     * "wait as good as forever" values callers reach for.
     *
     * Two things go wrong with a very large timeout, and neither is theoretical -- both were
     * measured on the aarch64 CI runners (g++ 9.4, libstdc++ 9), and one of them was found
     * as a live hang inside Fast-DDS:
     *
     *  - @c now() @c + @c duration::max() OVERFLOWS. The resulting deadline lands in the
     *    PAST, so the wait it feeds returns immediately and the caller who asked to wait
     *    essentially forever is told at once that the wait timed out.
     *
     *  - @c condition_variable::wait_until with a @c steady_clock deadline at or near
     *    @c time_point::max() SPINS. libstdc++ implements waiting on a clock other than its
     *    native one by adding (deadline - now) to the native clock's now(), which overflows
     *    for a deadline that far out; the underlying wait then returns instantly, the
     *    re-check against the caller's clock says the deadline has not passed, and the
     *    predicate loop goes round again immediately. Measured: 199 of 200 available CPU
     *    ticks over two seconds, i.e. a fully burned core, for as long as the predicate
     *    stays false. @c system_clock deadlines and @c wait_for are unaffected, because
     *    neither needs the cross-clock conversion.
     *
     * Both helpers here take the same approach: never let an unbounded value reach the
     * standard library. Semantics are preserved exactly -- a caller asking to wait forever
     * still waits forever, in slices.
     */

    /// @brief A duration as floating-point seconds, for weighing two of them against each
    /// other without the integer conversion a near-max value cannot survive.
    ///
    /// duration_cast between integer durations multiplies, so converting a near-max value
    /// into a finer unit overflows -- the very hazard this file exists to remove. A cast to
    /// a floating-point duration only loses precision, and precision is irrelevant at the
    /// scale where these comparisons decide anything.
    template <typename rep, typename period>
    std::chrono::duration<double> as_seconds(const std::chrono::duration<rep, period> &value)
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(value);
    }

    /// The longest single wait ever handed to a condition variable. Long enough that
    /// slicing costs nothing measurable (one extra wakeup an hour on a wait that was going
    /// to last longer than that anyway), short enough that no deadline derived from it can
    /// approach the overflow range.
    inline constexpr std::chrono::hours max_wait_slice{1};

    /**
     * @brief @c clock::now() @c + @p timeout, saturating at @c clock::time_point::max()
     * instead of overflowing past it.
     *
     * @tparam clock Clock to take the deadline on.
     * @param timeout How long from now the deadline is.
     * @return The deadline, or @c clock::time_point::max() where @p timeout would overrun it.
     */
    template <typename clock, typename rep, typename period>
    typename clock::time_point saturating_deadline(const std::chrono::duration<rep, period> &timeout)
    {
        const auto now = clock::now();
        // The guard below is one-sided -- it only asks whether the timeout runs off the TOP of
        // the range -- so a negative timeout falls straight through to the cast, and a
        // large-magnitude one overflows there: measured, milliseconds{-9300000000000} wraps to a
        // deadline ~290 years in the FUTURE, turning "already expired" into "wait essentially
        // forever". That is the same inversion this file exists to prevent, arriving from the
        // other end, and it is reachable from the public API (future_response::wait_for is
        // templated on any duration; the matched-count timeouts are unvalidated signed
        // milliseconds).
        //
        // A non-positive timeout can only mean "already expired", so it is answered directly
        // rather than computed -- which is also what makes the cast below safe to reach.
        if (timeout <= std::chrono::duration<rep, period>::zero())
        {
            return now;
        }
        if (as_seconds(timeout) >= as_seconds(clock::time_point::max() - now))
        {
            return clock::time_point::max();
        }
        return now + std::chrono::duration_cast<typename clock::duration>(timeout);
    }

    /**
     * @brief @p deadline moved earlier by @p reserve, floored at @p floor rather than wrapping.
     *
     * The subtraction @c saturating_deadline's callers need afterwards, and it has the same
     * hazard the cast in this file exists to avoid, one step further on. Writing
     * @c deadline @c - @c reserve directly converts @p reserve into the clock's own ticks
     * first -- a multiplication, for a duration expressed in coarser units -- so a caller who
     * passes a near-max settle time overflows there and lands arbitrarily far in the past.
     * The wait that follows then expires at once, which is precisely the "asked to wait as
     * long as it takes, told immediately that nothing matched" behaviour the deadline was
     * saturated to prevent: the saturation would be undone by the very next line.
     *
     * Compared in seconds-as-double before any conversion, exactly as @c saturating_deadline
     * does, so nothing is converted until it is known to be representable.
     *
     * @tparam clock Clock the deadline belongs to.
     * @param deadline Deadline to move earlier.
     * @param reserve How much of the budget to leave unspent; any duration, including one
     * too large to express in the clock's own ticks. A non-positive reserve reserves
     * nothing, leaving @p deadline where it is.
     * @param floor Earliest deadline to return, used whenever @p reserve would consume the
     * whole budget.
     * @return @p deadline @c - @p reserve, or @p floor where that is later or the reserve
     * does not fit.
     */
    template <typename clock, typename rep, typename period>
    typename clock::time_point deadline_reserving(const typename clock::time_point deadline,
                                                  const std::chrono::duration<rep, period> &reserve,
                                                  const typename clock::time_point floor)
    {
        // A negative reserve is clamped away before anything is computed from it, because
        // the comparison below passes it straight through: it is smaller than any positive
        // difference, so control would reach the subtraction, where deadline - reserve is
        // deadline + |reserve| -- an addition that overflows precisely when saturating_deadline
        // has just put the deadline at time_point::max(). That is this function's own hazard
        // arriving from the other side, and under the sanitizers Debug builds enable by
        // default it aborts rather than merely landing in the past.
        //
        // Clamped rather than rejected, because the reserve is a caller-supplied settle time
        // with no documented precondition and the rest of that path already tolerates
        // non-positive values; reserving a negative amount can only mean reserving nothing.
        // The Python mirror opens the same computation the same way, with
        // max(0.0, settle_time_sec).
        using reserve_duration = std::chrono::duration<rep, period>;
        const auto effective_reserve = std::max(reserve, reserve_duration::zero());

        // Also the case where the deadline already sits at or before the floor: the
        // difference is then zero or negative, no non-negative reserve is smaller, and the
        // floor is returned -- which is what the std::max this replaces did.
        if (as_seconds(effective_reserve) >= as_seconds(deadline - floor))
        {
            return floor;
        }
        // Representable by construction: the reserve is strictly smaller than a difference
        // the clock's own duration already holds.
        return deadline - std::chrono::duration_cast<typename clock::duration>(effective_reserve);
    }

    /**
     * @brief @c condition_variable::wait_until, in slices no longer than @c max_wait_slice,
     * so the deadline handed to the standard library is always one it can represent safely.
     *
     * Identical in observable behaviour to @c cv.wait_until(lock, deadline, predicate) --
     * including returning the predicate's final value -- for every deadline the plain call
     * handles correctly, and correct as well for the ones it does not, with ONE difference
     * worth knowing before relying on a realtime deadline: a step of the deadline's own clock
     * is observed at the next slice boundary rather than immediately. libstdc++ has a dedicated
     * @c system_clock overload of @c wait_until that blocks on @c CLOCK_REALTIME and so wakes on
     * a wall-clock step; this always ends in @c cv.wait_for, which is @c CLOCK_MONOTONIC. The
     * loop re-reads the caller's clock each iteration, so the step is honoured rather than lost,
     * just up to @c max_wait_slice late. On the path this library actually uses it is a net
     * improvement -- a forward step during a plain @c wait_for returned an immediate spurious
     * timeout.
     *
     * @param cv Condition variable to wait on.
     * @param lock Lock held on entry, held again on return.
     * @param deadline When to give up. Any clock, and any duration on it -- including one
     * COARSER than the clock's own, which is the shape that makes @c time_point::max()
     * dangerous: weighing such a value against @c clock::now() converts it into the clock's
     * finer unit, and that multiplication overflows. Left to the standard library, the
     * overflow makes a far-future deadline compare as ALREADY PASSED, so the wait returns a
     * timeout at once -- this file's own bug, in a differently shaped deadline. Saturated
     * into the clock's own duration below instead.
     * @param predicate What is being waited for.
     * @return The predicate's value: @c true if satisfied, @c false if the deadline passed.
     */
    template <typename clock, typename duration, typename predicate_t>
    bool wait_until_bounded(std::condition_variable &cv, std::unique_lock<std::mutex> &lock,
                            const std::chrono::time_point<clock, duration> &deadline, predicate_t predicate)
    {
        using native_time_point = typename clock::time_point;

        // Brought into the clock's own duration ONCE, saturating, so every comparison and
        // subtraction below is native and none of them can overflow.
        // Clamped at BOTH ends. The upper comparison alone leaves a deadline far enough in the
        // past to overflow the same time_point_cast, for the same reason a negative timeout
        // overflows saturating_deadline's -- and a deadline that wraps to the far future is a
        // wait that never ends, which is strictly worse than the expiry it should have been.
        const native_time_point native_deadline =
            (as_seconds(deadline.time_since_epoch()) >= as_seconds(native_time_point::max().time_since_epoch()))
                ? native_time_point::max()
            : (as_seconds(deadline.time_since_epoch()) <= as_seconds(native_time_point::min().time_since_epoch()))
                ? native_time_point::min()
                : std::chrono::time_point_cast<typename clock::duration>(deadline);

        while (!predicate())
        {
            const auto now = clock::now();
            if (now >= native_deadline)
            {
                return false;  // Deadline passed and the predicate is false -- a timeout.
            }

            // Subtraction only, never an addition that could overrun the representable
            // range, and the slice handed over is bounded whatever the deadline was.
            const auto remaining = native_deadline - now;
            if (remaining > max_wait_slice)
            {
                cv.wait_for(lock, max_wait_slice, predicate);
            }
            else
            {
                cv.wait_for(lock, remaining, predicate);
            }
        }
        return true;
    }
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_BOUNDED_WAIT
