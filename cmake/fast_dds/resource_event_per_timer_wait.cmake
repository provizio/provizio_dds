# Make ResourceEvent::unregister_timer wait only for the callback of the timer being
# unregistered, instead of for the execution thread to go idle.
#
# Fast-DDS runs every timer callback of a participant on one thread (ResourceEvent,
# "dds.ev.N"). Destroying or recreating a TimedEvent calls unregister_timer(), which -- so that
# the callback of THAT event can never run after it returns -- waits. In Fast-DDS up to and
# including v3.6.2 it waits for the execution thread to finish its whole do_timer_actions()
# pass, i.e. for whatever callback happens to be running right now, whichever timer it belongs
# to. Timer callbacks take endpoint mutexes (the participant-lease timer reaps a dead peer:
# PDP::remove_remote_participant -> EDPSimple::removeRemoteEndpoints ->
# StatefulWriter::matched_reader_remove). So a thread that holds an endpoint mutex and
# unregisters ANY timer deadlocks with a callback that wants that mutex:
#
#   thread A  holds a StatefulWriter's mutex (inside WriterHistory::add_change, or
#             deleteUserEndpoint) -> ... -> WriterProxy::stop -> TimedEvent::recreate_timer
#             -> ResourceEvent::unregister_timer -> waits for the pass to end
#   dds.ev.0  do_timer_actions -> the lease timer -> PDP::remove_remote_participant
#             -> StatefulWriter::matched_reader_remove -> waits for thread A's mutex
#
# This is eProsima/Fast-DDS#6502 (and #6193). provizio_dds hit it in CI as tests that never
# finished -- a 0.6 s test consuming its whole 45 s TIMEOUT -- with both stacks captured on a
# macos-15-intel runner and reproduced on Linux; the trigger is a remote participant whose
# lease EXPIRES (a clean dispose is handled on the receive thread and cannot block this).
# Fast-DDS knows the rule -- StatefulReader releases the reader's own lock before
# WriterProxy::stop() "to avoid deadlock when waiting for event (requiring mutex) to finish"
# -- but no caller can release every lock a foreign callback might want.
#
# The fix keeps exactly the guarantee the two callers rely on and drops the rest:
#   - TimedEvent::~TimedEvent deletes the event right after unregister_timer returns, so the
#     event's own callback must not be running then;
#   - WriterProxy::stop() calls recreate_timer() when the event is BUSY precisely to wait for
#     that event's in-flight callback ("TimedEvent being performed, wait for it to finish").
# So the execution thread records which timer it is running (executing_timer_), and
# unregister_timer waits for that field to stop naming the event being unregistered -- and
# for nothing else. For that to be safe, the timer collections must be consistently
# protected, which today they are not: the trigger loop iterates active_timers_ with mutex_
# released (the idle-wait was what kept foreign threads out). do_timer_actions now takes the
# due timers as a snapshot under mutex_, releases it only around each callback, re-validates
# each snapshot entry against the collection when anything was erased meanwhile (by pointer
# value, never dereferencing -- an erased timer may already be deleted), and sorts under the
# lock. unregister_timer erases at once, under mutex_, then waits only if its event is the one
# executing. Nothing about when callbacks run, or how often, changes.
#
# This runs as the Fast-DDS ExternalProject PATCH_COMMAND after host_id_without_interfaces.cmake.
# Like the other two scripts it is:
#   - Idempotent: re-running it on already patched files is a no-op.
#   - Self-checking: if any anchor is missing (a future Fast-DDS reshapes ResourceEvent) it
#     FAILs loudly rather than silently building a library without the fix.
# A FAST_DDS_VERSION bump must re-check it; once upstream fixes #6502, drop it.
#
# Invoked as:
#   cmake -DRESOURCE_EVENT_H=<path-to-ResourceEvent.h>
#         -DRESOURCE_EVENT_CPP=<path-to-ResourceEvent.cpp> -P resource_event_per_timer_wait.cmake

foreach(_var IN ITEMS RESOURCE_EVENT_H RESOURCE_EVENT_CPP)
    if(NOT DEFINED ${_var})
        message(FATAL_ERROR "resource_event_per_timer_wait.cmake: ${_var} must be defined")
    endif()
    if(NOT EXISTS "${${_var}}")
        message(FATAL_ERROR "resource_event_per_timer_wait.cmake: file not found: ${${_var}}")
    endif()
endforeach()

set(_marker "executing_timer_")

# Replace ONE exact block in ${_contents}, failing loudly if it is not present verbatim.
function(_provizio_replace_or_fail _file _what _anchor _patched)
    string(FIND "${_contents}" "${_anchor}" _pos)
    if(_pos EQUAL -1)
        message(FATAL_ERROR
            "resource_event_per_timer_wait.cmake: anchor for '${_what}' not found in ${_file}. "
            "Fast-DDS has changed shape; re-check this patch against the new sources "
            "(or drop it if eProsima/Fast-DDS#6502 is fixed in this version).")
    endif()
    string(REPLACE "${_anchor}" "${_patched}" _contents "${_contents}")
    set(_contents "${_contents}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------------------------
# ResourceEvent.h
# ---------------------------------------------------------------------------------------------
file(READ "${RESOURCE_EVENT_H}" _contents)
string(FIND "${_contents}" "${_marker}" _already_pos)
if(NOT _already_pos EQUAL -1)
    message(STATUS "resource_event_per_timer_wait: ResourceEvent.h already patched -- no-op")
else()
    # uint64_t below: make its header explicit rather than rely on what ThreadSettings.hpp
    # happens to pull in (libstdc++ 15 dropped several such transitive <cstdint> includes).
    _provizio_replace_or_fail("${RESOURCE_EVENT_H}" "the include block" [==[
#include <atomic>
#include <functional>
#include <memory>
#include <vector>
]==] [==[
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
]==])

    _provizio_replace_or_fail("${RESOURCE_EVENT_H}" "the idle flag" [==[
    //! Used to warn about changes on allow_vector_manipulation_.
    TimedConditionVariable cv_manipulation_;

    //! Flag used to allow a thread to manipulate the timer collections when the execution thread is not using them.
    bool allow_vector_manipulation_ = true;
]==] [==[
    //! Used to warn that executing_timer_ has changed.
    TimedConditionVariable cv_manipulation_;

    //! [provizio_dds] The timer whose callback the execution thread is running right now, or nullptr.
    //! Protected by mutex_. unregister_timer() of exactly this timer waits for it to be cleared;
    //! unregistering any other timer never waits. See resource_event_per_timer_wait.cmake.
    TimedEventImpl* executing_timer_ = nullptr;

    //! [provizio_dds] Bumped under mutex_ on every erase from active_timers_, so the execution
    //! thread can tell whether the snapshot it works from is still exactly the collection.
    uint64_t active_timers_generation_ = 0;
]==])

    _provizio_replace_or_fail("${RESOURCE_EVENT_H}" "the skip flag" [==[
    //! Prevents iterator invalidation when active_timers are manipulated inside loops
    std::atomic<bool> skip_checking_active_timers_;
]==] [==[
    //! [provizio_dds] The timers found due at the start of a do_timer_actions() pass. A member
    //! only to reuse its capacity; meaningful solely within that call, on the execution thread.
    std::vector<TimedEventImpl*> due_timers_;
]==])

    _provizio_replace_or_fail("${RESOURCE_EVENT_H}" "resize_collections" [==[
    void resize_collections()
    {
        pending_timers_.reserve(timers_count_);
        active_timers_.reserve(timers_count_);
    }
]==] [==[
    void resize_collections()
    {
        pending_timers_.reserve(timers_count_);
        active_timers_.reserve(timers_count_);
        due_timers_.reserve(timers_count_);
    }
]==])

    file(WRITE "${RESOURCE_EVENT_H}" "${_contents}")
    message(STATUS "resource_event_per_timer_wait: patched ResourceEvent.h")
endif()

# ---------------------------------------------------------------------------------------------
# ResourceEvent.cpp
# ---------------------------------------------------------------------------------------------
file(READ "${RESOURCE_EVENT_CPP}" _contents)
string(FIND "${_contents}" "${_marker}" _already_pos)
if(NOT _already_pos EQUAL -1)
    message(STATUS "resource_event_per_timer_wait: ResourceEvent.cpp already patched -- no-op")
    return()
endif()

_provizio_replace_or_fail("${RESOURCE_EVENT_CPP}" "unregister_timer" [==[
void ResourceEvent::unregister_timer(
        TimedEventImpl* event)
{
    std::unique_lock<TimedMutex> lock(mutex_);

    bool is_service_thread = thread_->is_calling_thread();

    //! Let the service thread to manipulate resources
    if (!is_service_thread)
    {
        cv_manipulation_.wait(lock, [&]()
                {
                    return allow_vector_manipulation_;
                });
    }

    bool should_notify = false;
    std::vector<TimedEventImpl*>::iterator it;

    // Remove from pending
    it = std::find(pending_timers_.begin(), pending_timers_.end(), event);
    if (it != pending_timers_.end())
    {
        pending_timers_.erase(it);
        should_notify = true;
    }

    // Remove from active
    it = std::find(active_timers_.begin(), active_timers_.end(), event);
    if (it != active_timers_.end())
    {
        active_timers_.erase(it);

        if (is_service_thread)
        {
            //! Warn the do_timer_actions loop to skip checking the rest of active_timers
            //! in this iteration to prevent iterator invalidation
            skip_checking_active_timers_.store(true);
        }

        should_notify = true;
    }

    // Decrement counter of created timers
    --timers_count_;

    if (should_notify)
    {
        // Notify the execution thread that something changed
        cv_.notify_one();
    }
}
]==] [==[
void ResourceEvent::unregister_timer(
        TimedEventImpl* event)
{
    std::unique_lock<TimedMutex> lock(mutex_);

    bool is_service_thread = thread_->is_calling_thread();

    bool should_notify = false;
    std::vector<TimedEventImpl*>::iterator it;

    // Remove from pending
    it = std::find(pending_timers_.begin(), pending_timers_.end(), event);
    if (it != pending_timers_.end())
    {
        pending_timers_.erase(it);
        should_notify = true;
    }

    // Remove from active. [provizio_dds] Safe at any time now: the execution thread only
    // touches active_timers_ under mutex_, and re-validates its snapshot against this
    // generation before triggering anything.
    it = std::find(active_timers_.begin(), active_timers_.end(), event);
    if (it != active_timers_.end())
    {
        active_timers_.erase(it);
        ++active_timers_generation_;
        should_notify = true;
    }

    // Decrement counter of created timers
    --timers_count_;

    // [provizio_dds] The guarantee callers rely on -- TimedEvent::~TimedEvent deletes the
    // event right after this returns, and WriterProxy::stop() calls recreate_timer() exactly
    // to wait for an in-flight callback -- is that THIS event's callback is no longer
    // running. Wait for that, and for nothing else: waiting for the execution thread to
    // finish its whole pass deadlocked whenever the callback in progress needed a mutex the
    // calling thread already held (eProsima/Fast-DDS#6502).
    if (!is_service_thread)
    {
        cv_manipulation_.wait(lock, [&]()
                {
                    return executing_timer_ != event;
                });
    }

    if (should_notify)
    {
        // Notify the execution thread that something changed
        cv_.notify_one();
    }
}
]==])

_provizio_replace_or_fail("${RESOURCE_EVENT_CPP}" "event_service" [==[
        // If pending timers exist, there is some work to be done, so no need to wait.
        if (!pending_timers_.empty())
        {
            continue;
        }

        // Allow other threads to manipulate the timer collections while we wait.
        allow_vector_manipulation_ = true;
        cv_manipulation_.notify_all();

        // Wait for the first timer to be triggered
        std::chrono::steady_clock::time_point next_trigger =
                active_timers_.empty() ?
                current_time_ + std::chrono::seconds(1) :
                active_timers_[0]->next_trigger_time();

        auto current_time = std::chrono::steady_clock::now();
        if (current_time > next_trigger)
        {
            next_trigger = current_time + std::chrono::microseconds(10);
        }

        cv_.wait_until(lock, next_trigger);

        // Don't allow other threads to manipulate the timer collections
        allow_vector_manipulation_ = false;
        resize_collections();
    }

    // Thread being stopped. Allow other threads to manipulate the timer collections.
    {
        std::lock_guard<TimedMutex> guard(mutex_);
        allow_vector_manipulation_ = true;
    }
    cv_manipulation_.notify_all();
}
]==] [==[
        // If pending timers exist, there is some work to be done, so no need to wait.
        if (!pending_timers_.empty())
        {
            continue;
        }

        // Wait for the first timer to be triggered
        std::chrono::steady_clock::time_point next_trigger =
                active_timers_.empty() ?
                current_time_ + std::chrono::seconds(1) :
                active_timers_[0]->next_trigger_time();

        auto current_time = std::chrono::steady_clock::now();
        if (current_time > next_trigger)
        {
            next_trigger = current_time + std::chrono::microseconds(10);
        }

        cv_.wait_until(lock, next_trigger);

        resize_collections();
    }

    // Thread being stopped. [provizio_dds] executing_timer_ is already nullptr here; wake anyone
    // still waiting in unregister_timer so it re-checks and returns.
    cv_manipulation_.notify_all();
}
]==])

_provizio_replace_or_fail("${RESOURCE_EVENT_CPP}" "do_timer_actions" [==[
    bool did_something = false;

    // Process pending orders
    {
        std::lock_guard<TimedMutex> lock(mutex_);
        for (TimedEventImpl* tp : pending_timers_)
        {
            // Remove item from active timers
            auto current_pos = std::lower_bound(active_timers_.begin(), active_timers_.end(), tp, event_compare);
            current_pos = std::find(current_pos, active_timers_.end(), tp);
            if (current_pos != active_timers_.end())
            {
                active_timers_.erase(current_pos);
            }

            // Update timer info
            if (tp->update(current_time_, cancel_time))
            {
                // Timer has to be activated: add to active timers
                std::vector<TimedEventImpl*>::iterator low_bound;

                // Insert on correct position
                low_bound = std::lower_bound(active_timers_.begin(), active_timers_.end(), tp, event_compare);
                active_timers_.emplace(low_bound, tp);
            }
        }
        pending_timers_.clear();
    }

    // Trigger active timers
    skip_checking_active_timers_.store(false);
    for (TimedEventImpl* tp : active_timers_)
    {
        if (tp->next_trigger_time() <= current_time_)
        {
            did_something = true;
            tp->trigger(current_time_, cancel_time);

            //! skip this iteration as active_timers has been manipulated
            if (skip_checking_active_timers_.load())
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    // If an action was made, keep active_timers_ sorted
    if (did_something)
    {
        sort_timers();
        active_timers_.erase(
            std::lower_bound(active_timers_.begin(), active_timers_.end(), nullptr,
            [cancel_time](
                TimedEventImpl* a,
                TimedEventImpl* b)
            {
                (void)b;
                return a->next_trigger_time() < cancel_time;
            }),
            active_timers_.end()
            );
    }
}
]==] [==[
    bool did_something = false;

    // [provizio_dds] mutex_ is held for everything here except the callbacks themselves, so a
    // thread unregistering a timer can always get in; it then waits only if the timer it is
    // unregistering is the one whose callback is running.
    std::unique_lock<TimedMutex> lock(mutex_);

    // Process pending orders
    for (TimedEventImpl* tp : pending_timers_)
    {
        // Remove item from active timers
        auto current_pos = std::lower_bound(active_timers_.begin(), active_timers_.end(), tp, event_compare);
        current_pos = std::find(current_pos, active_timers_.end(), tp);
        if (current_pos != active_timers_.end())
        {
            active_timers_.erase(current_pos);
            ++active_timers_generation_;
        }

        // Update timer info
        if (tp->update(current_time_, cancel_time))
        {
            // Timer has to be activated: add to active timers
            std::vector<TimedEventImpl*>::iterator low_bound;

            // Insert on correct position
            low_bound = std::lower_bound(active_timers_.begin(), active_timers_.end(), tp, event_compare);
            active_timers_.emplace(low_bound, tp);
        }
    }
    pending_timers_.clear();

    // Trigger active timers. active_timers_ is sorted, so the due ones are a prefix; take them
    // as a snapshot, because the collection may change while a callback runs.
    due_timers_.clear();
    for (TimedEventImpl* tp : active_timers_)
    {
        if (tp->next_trigger_time() > current_time_)
        {
            break;
        }
        due_timers_.push_back(tp);
    }
    const uint64_t snapshot_generation = active_timers_generation_;

    for (TimedEventImpl* tp : due_timers_)
    {
        // Still registered? Free when nothing was erased since the snapshot. Otherwise the
        // pointer is looked up by VALUE, never dereferenced: an erased timer may already have
        // been deleted by its owner.
        if (snapshot_generation != active_timers_generation_ &&
                std::find(active_timers_.begin(), active_timers_.end(), tp) == active_timers_.end())
        {
            continue;
        }

        did_something = true;
        executing_timer_ = tp;
        lock.unlock();
        tp->trigger(current_time_, cancel_time);
        lock.lock();
        executing_timer_ = nullptr;
        // Wake an unregister_timer() that is waiting for exactly this callback to finish.
        cv_manipulation_.notify_all();
    }
    due_timers_.clear();

    // If an action was made, keep active_timers_ sorted
    if (did_something)
    {
        sort_timers();
        auto first_cancelled = std::lower_bound(active_timers_.begin(), active_timers_.end(), nullptr,
                        [cancel_time](
                            TimedEventImpl* a,
                            TimedEventImpl* b)
                        {
                            (void)b;
                            return a->next_trigger_time() < cancel_time;
                        });
        if (first_cancelled != active_timers_.end())
        {
            active_timers_.erase(first_cancelled, active_timers_.end());
            ++active_timers_generation_;
        }
    }
}
]==])

_provizio_replace_or_fail("${RESOURCE_EVENT_CPP}" "init_thread" [==[
    std::lock_guard<TimedMutex> lock(mutex_);

    allow_vector_manipulation_ = false;
    stop_.store(false);
    resize_collections();
]==] [==[
    std::lock_guard<TimedMutex> lock(mutex_);

    stop_.store(false);
    resize_collections();
]==])

file(WRITE "${RESOURCE_EVENT_CPP}" "${_contents}")
message(STATUS "resource_event_per_timer_wait: patched ResourceEvent.cpp")
