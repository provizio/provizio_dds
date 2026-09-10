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
# The fix keeps exactly the guarantee the callers rely on and drops the rest:
#   - TimedEvent::~TimedEvent deletes the event right after unregister_timer returns, so the
#     event's own callback must not be running then;
#   - WriterProxy::stop() calls recreate_timer() when the proxy is BUSY to wait for the
#     in-flight callback ("TimedEvent being performed, wait for it to finish").
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
# One caller leaned on the old, wider wait: WriterProxy owns two timers and its stop() waited
# on initial_acknack_ alone, "it does not matter which of the two events is the one on
# execution" -- true only while unregister_timer waited for the whole pass. With the wait
# narrowed to one event, stop() would return while a heartbeat_response_ callback is still
# running and clear() the proxy under it (a StatefulReader::send_acknack reading the proxy's
# cleared locators and sequence sets), so WriterProxy.cpp is patched too: stop() recreates
# both timers, which is exactly what the old wait fenced for it, and no more.
#
# This runs as the Fast-DDS ExternalProject PATCH_COMMAND after host_id_without_interfaces.cmake.
# Like the other two scripts it is:
#   - Idempotent: re-running it on already patched files is a no-op.
#   - Self-checking: if any anchor is missing (a future Fast-DDS reshapes ResourceEvent) it
#     FAILs loudly rather than silently building a library without the fix.
# A FAST_DDS_VERSION bump must re-check it; once upstream fixes #6502, drop it.
#
# Nothing is written until every anchor in every file has been found, so a failed run leaves
# the sources pristine rather than half patched.
#
# Invoked as:
#   cmake -DRESOURCE_EVENT_H=<path-to-ResourceEvent.h>
#         -DRESOURCE_EVENT_CPP=<path-to-ResourceEvent.cpp>
#         -DWRITER_PROXY_CPP=<path-to-WriterProxy.cpp> -P resource_event_per_timer_wait.cmake

foreach(_var IN ITEMS RESOURCE_EVENT_H RESOURCE_EVENT_CPP WRITER_PROXY_CPP)
    if(NOT DEFINED ${_var})
        message(FATAL_ERROR "resource_event_per_timer_wait.cmake: ${_var} must be defined")
    endif()
    if(NOT EXISTS "${${_var}}")
        message(FATAL_ERROR "resource_event_per_timer_wait.cmake: file not found: ${${_var}}")
    endif()
endforeach()

# Every replacement below carries this tag in a comment; a file that has it has been patched by
# some revision of this script.
set(_marker "[provizio_dds]")

# ...but "patched by some revision" is not "patched by THIS one, and a source tree carries no
# other record of which. Treating the plain marker as done is how a corrected defect keeps
# shipping: a tree patched before the fix below carries the marker, so re-running this script on
# it reported "already patched -- no-op" and left the old, broken implementation in place. Bump
# this whenever a replacement changes in a way an existing tree must pick up, and give the
# affected file a migration from the revision before it.
set(_revision "2")
set(_revision_marker "[provizio_dds r${_revision}]")

# What every marker starts with, whatever revision wrote it: the bare "[provizio_dds]" of the
# revisions before the marker existed, and "[provizio_dds rN]" of every one since. Used to ask
# "was this file patched by SOME revision of this script?" without keeping a list of past ones.
set(_marker_prefix "[provizio_dds")

# Stamp every marker this script wrote into ${_contents} with the current revision, so the file
# records WHICH revision produced it. The literals below all carry the plain marker; this is the
# single place the revision is attached, so no literal has to spell it and none can be missed.
function(_provizio_stamp_revision _out_contents)
    # Normalised BEFORE promoting. A tree stamped by an earlier revision carries
    # "[provizio_dds rN]" for some other N, which the plain REPLACE below would not touch --
    # so the file would keep the old revision's stamp and be diagnosed as stale for ever after.
    # Demoting every versioned marker back to the bare form first is what makes the stamp work
    # from ANY revision rather than only from the unversioned one.
    string(REGEX REPLACE "\\[provizio_dds r[0-9]+\\]" "${_marker}" _normalised "${${_out_contents}}")
    string(REPLACE "${_marker}" "${_revision_marker}" _stamped "${_normalised}")
    set(${_out_contents} "${_stamped}" PARENT_SCOPE)
endfunction()

# Sets _provizio_stale_pos in the CALLER's scope to -1 when every marker in the contents held
# by ${_contents_var} is THIS revision's, and to a non-negative position when at least one was
# written by a different one.
#
# Asked by removing this revision's markers and looking for what is left, rather than by
# testing for the shapes a previous revision happened to write. Enumerating those shapes is
# what broke: the check recognised the bare marker and this revision's, so the moment
# _revision is bumped, every existing tree -- stamped with the revision before it -- matched
# neither, fell through to the pristine-source branch and failed the configure blaming
# Fast-DDS for a shape change that had not happened.
#
# A function taking the NAME of the variable, never a macro taking the text -- the same shape
# _provizio_stamp_revision uses, for the same reason. A macro's parameters are textual
# substitutions, so the whole Fast-DDS source file would be pasted into the body and RE-
# EVALUATED: every ${...} in it dereferenced (to nothing, silently) and every backslash
# escape processed, before the marker search ever ran. Today's three files happen to carry
# neither in the region that matters, so the bug is latent rather than live -- which is
# exactly the kind that surfaces on a Fast-DDS bump, as a patch that reports a pristine tree
# and fails the configure blaming eProsima.
function(_provizio_find_stale_marker _contents_var)
    string(REPLACE "${_revision_marker}" "" _without_current "${${_contents_var}}")
    string(FIND "${_without_current}" "${_marker_prefix}" _stale_pos)
    set(_provizio_stale_pos "${_stale_pos}" PARENT_SCOPE)
endfunction()

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
function(_provizio_patch_resource_event_h _out)
file(READ "${RESOURCE_EVENT_H}" _contents)
string(FIND "${_contents}" "${_revision_marker}" _current_pos)
# Any marker NOT written by this revision -- the bare form, or another rN. See
# _provizio_find_stale_marker for why the question is asked that way round.
_provizio_find_stale_marker(_contents)
if(NOT _current_pos EQUAL -1 AND _provizio_stale_pos EQUAL -1)
    message(STATUS "resource_event_per_timer_wait: ResourceEvent.h already patched at revision "
                   "${_revision} -- no-op")
    # Returns WITHOUT setting ${_out}, which is how the caller knows not to write the file.
    # Falling through to the write instead round-tripped byte-identical content but moved the
    # mtime -- and since the patch step now depends on this script, that rebuilt every
    # translation unit reaching ResourceEvent.h whenever the script was merely touched.
    return()
elseif(NOT _provizio_stale_pos EQUAL -1)
    # Patched by an earlier revision. This file's replacements have not changed between
    # revisions, so its migration is the stamp alone -- but it must still HAPPEN, because the
    # three files this script writes are one fix: WriterProxy::stop()'s two-timer fence and
    # unregister_timer's per-timer wait are halves of each other. Gating only ResourceEvent.cpp
    # on the revision, as this did, meant a bump upgraded that file and silently left the other
    # two behind, which is worse than a wholly stale tree.
    message(STATUS "resource_event_per_timer_wait: stamping ResourceEvent.h at revision ${_revision}")
    _provizio_stamp_revision(_contents)
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

    _provizio_stamp_revision(_contents)
endif()

set(${_out} "${_contents}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------------------------
# ResourceEvent.cpp
# ---------------------------------------------------------------------------------------------
function(_provizio_patch_resource_event_cpp _out)
file(READ "${RESOURCE_EVENT_CPP}" _contents)
string(FIND "${_contents}" "${_revision_marker}" _current_pos)
# "Already at this revision" means EVERY marker in the file says so. A file carrying a marker
# from any other revision -- the bare form, or another rN -- is migrated; taking this
# revision's mere presence as done would leave a half-stamped file that way for good.
_provizio_find_stale_marker(_contents)
if(NOT _current_pos EQUAL -1 AND _provizio_stale_pos EQUAL -1)
    message(STATUS "resource_event_per_timer_wait: ResourceEvent.cpp already patched at revision "
                   "${_revision} -- no-op")
    return()
endif()
if(NOT _provizio_stale_pos EQUAL -1)
    # Patched by an earlier revision of this script. Only the tail of unregister_timer changed,
    # so migrate that in place rather than making the developer find and delete a Fast-DDS build
    # tree -- and fail loudly, as everything here does, if it is none of the shapes an earlier
    # revision left behind.
    message(STATUS "resource_event_per_timer_wait: upgrading ResourceEvent.cpp to revision ${_revision}")

    # The tail r1 left behind, and the one r2 wants in its place.
    set(_wait_tail_r1 [==[
    if (!is_service_thread)
    {
        cv_manipulation_.wait(lock, [&]()
                {
                    return executing_timer_ != event;
                });
    }
]==])
    set(_wait_tail_swept [==[
    if (!is_service_thread)
    {
        cv_manipulation_.wait(lock, [&]()
                {
                    return executing_timer_ != event;
                });

        // [provizio_dds] That wait RELEASED the mutex, and a callback is free to restart its
        // own timer while it runs -- ResourceEvent::notify() then puts this very pointer back
        // into pending_timers_ behind us. Erasing it before the wait is therefore not enough
        // on its own: TimedEvent::~TimedEvent deletes the event the moment this returns, and
        // the requeued pointer is dereferenced on the execution thread's next pass, through
        // event_compare() and TimedEventImpl::next_trigger_time() (heap-use-after-free, seen
        // under ASan with a callback that rearms while another thread unregisters it). So
        // sweep both collections again here, where the callback has provably finished and the
        // mutex is held again, and nothing can put it back. Dropping that late restart is the
        // correct outcome: the caller is destroying the timer.
        it = std::find(pending_timers_.begin(), pending_timers_.end(), event);
        if (it != pending_timers_.end())
        {
            pending_timers_.erase(it);
            should_notify = true;
        }

        it = std::find(active_timers_.begin(), active_timers_.end(), event);
        if (it != active_timers_.end())
        {
            active_timers_.erase(it);
            ++active_timers_generation_;
            should_notify = true;
        }
    }
]==])

    # The FILE is normalised to the bare marker before anything is compared against it. That is
    # what makes the decision below marker-agnostic: whichever revision stamped this tree, the
    # sweep either is present or it is not, and asking which tag it happens to carry is exactly
    # the mistake the outer gate made -- enumerate the shapes you know about, and the first
    # shape you did not think of (a tree stamped by the revision before the next bump) falls
    # through to "pristine" and aborts the configure blaming Fast-DDS.
    string(REGEX REPLACE "\\[provizio_dds r[0-9]+\\]" "${_marker}" _contents "${_contents}")

    # Both sides of this comparison carry the BARE marker -- the literal above spells no
    # revision at all, and the file was just normalised -- so it holds whatever revision either
    # was stamped with. The literal used to spell "r2", which meant deriving its bare form
    # silently produced nothing once _revision moved on: every existing tree then took the
    # pre-sweep branch below and aborted the configure.
    string(FIND "${_contents}" "${_wait_tail_swept}" _swept_pos)
    if(_swept_pos EQUAL -1)
        # The sweep really is absent, so this is the pre-sweep shape: insert it. Where it is
        # already there the content needs nothing and the stamp below is the whole migration.
        _provizio_replace_or_fail("${RESOURCE_EVENT_CPP}" "unregister_timer post-wait sweep"
                                  "${_wait_tail_r1}" "${_wait_tail_swept}")
    endif()
    # Every marker in the file, not only the one the r2 tail carries: the whole file records
    # which revision produced it, so the next bump can tell this tree from the one before it.
    _provizio_stamp_revision(_contents)
    set(${_out} "${_contents}" PARENT_SCOPE)
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

        // [provizio_dds] That wait RELEASED the mutex, and a callback is free to restart its
        // own timer while it runs -- ResourceEvent::notify() then puts this very pointer back
        // into pending_timers_ behind us. Erasing it before the wait is therefore not enough
        // on its own: TimedEvent::~TimedEvent deletes the event the moment this returns, and
        // the requeued pointer is dereferenced on the execution thread's next pass, through
        // event_compare() and TimedEventImpl::next_trigger_time() (heap-use-after-free, seen
        // under ASan with a callback that rearms while another thread unregisters it). So
        // sweep both collections again here, where the callback has provably finished and the
        // mutex is held again, and nothing can put it back. Dropping that late restart is the
        // correct outcome: the caller is destroying the timer.
        it = std::find(pending_timers_.begin(), pending_timers_.end(), event);
        if (it != pending_timers_.end())
        {
            pending_timers_.erase(it);
            should_notify = true;
        }

        it = std::find(active_timers_.begin(), active_timers_.end(), event);
        if (it != active_timers_.end())
        {
            active_timers_.erase(it);
            ++active_timers_generation_;
            should_notify = true;
        }
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

_provizio_stamp_revision(_contents)
set(${_out} "${_contents}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------------------------
# WriterProxy.cpp
# ---------------------------------------------------------------------------------------------
function(_provizio_patch_writer_proxy_cpp _out)
file(READ "${WRITER_PROXY_CPP}" _contents)
string(FIND "${_contents}" "${_revision_marker}" _current_pos)
_provizio_find_stale_marker(_contents)
if(NOT _current_pos EQUAL -1 AND _provizio_stale_pos EQUAL -1)
    message(STATUS "resource_event_per_timer_wait: WriterProxy.cpp already patched at revision "
                   "${_revision} -- no-op")
    return()
endif()
if(NOT _provizio_stale_pos EQUAL -1)
    # Patched by an earlier revision; stamp it. See ResourceEvent.h for why every file this
    # script writes is gated on the revision and not on the bare marker.
    message(STATUS "resource_event_per_timer_wait: stamping WriterProxy.cpp at revision ${_revision}")
    _provizio_stamp_revision(_contents)
    set(${_out} "${_contents}" PARENT_SCOPE)
    return()
endif()

_provizio_replace_or_fail("${WRITER_PROXY_CPP}" "WriterProxy::stop" [==[
    if ((prev_code = state_.exchange(StateCode::STOPPED)) == StateCode::BUSY)
    {
        // TimedEvent being performed, wait for it to finish.
        // It does not matter which of the two events is the one on execution, but we must wait on initial_acknack_ as
        // it could be restarted if only cancelled while its callback is being triggered.
        initial_acknack_->recreate_timer();
    }
]==] [==[
    if ((prev_code = state_.exchange(StateCode::STOPPED)) == StateCode::BUSY)
    {
        // TimedEvent being performed, wait for it to finish.
        // [provizio_dds] ResourceEvent::unregister_timer waits for the callback of the event being
        // unregistered only (eProsima/Fast-DDS#6502), no longer for the timer thread's whole pass, so
        // recreating initial_acknack_ alone would not fence a heartbeat_response_ callback in flight:
        // recreate both. initial_acknack_ has to be recreated rather than cancelled as it could be
        // restarted if only cancelled while its callback is being triggered.
        initial_acknack_->recreate_timer();
        heartbeat_response_->recreate_timer();
    }
]==])

_provizio_stamp_revision(_contents)
set(${_out} "${_contents}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------------------------
# Check every anchor first, then write: a missing one fails the configure with all three files
# still pristine.
# ---------------------------------------------------------------------------------------------
_provizio_patch_resource_event_h(_patched_resource_event_h)
_provizio_patch_resource_event_cpp(_patched_resource_event_cpp)
_provizio_patch_writer_proxy_cpp(_patched_writer_proxy_cpp)

if(DEFINED _patched_resource_event_h)
    file(WRITE "${RESOURCE_EVENT_H}" "${_patched_resource_event_h}")
    message(STATUS "resource_event_per_timer_wait: patched ResourceEvent.h")
endif()
if(DEFINED _patched_resource_event_cpp)
    file(WRITE "${RESOURCE_EVENT_CPP}" "${_patched_resource_event_cpp}")
    message(STATUS "resource_event_per_timer_wait: patched ResourceEvent.cpp")
endif()
if(DEFINED _patched_writer_proxy_cpp)
    file(WRITE "${WRITER_PROXY_CPP}" "${_patched_writer_proxy_cpp}")
    message(STATUS "resource_event_per_timer_wait: patched WriterProxy.cpp")
endif()
