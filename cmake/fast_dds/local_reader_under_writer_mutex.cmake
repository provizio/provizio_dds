# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Make a Fast-DDS DataWriter hold its own mutex while it looks up a reader of the same process.
#
# A writer hands samples and heartbeats to a matched reader of its own process directly, without
# the transports. The reader is reached through ReaderLocator::local_reader_, a shared_ptr in the
# ReaderLocator a reliable writer keeps inside each ReaderProxy and a best-effort one keeps per
# reader, and ReaderLocator -- which documents itself as "always protected by writer's mutex" --
# resets it in stop(), under that mutex: StatefulWriter::matched_reader_remove stops the proxy
# when the reader is deleted, local_actions_on_writer_removed stops every proxy when the writer
# is. Two callers read it WITHOUT the mutex:
#
#   - StatefulWriter::intraprocess_heartbeat, on the participant's timer thread: every ReaderProxy
#     matched to a same-process reader arms an initial-heartbeat event with a 0 ms delay, and
#     this is what it runs;
#   - StatelessWriter::intraprocess_delivery.
#
# ReaderLocator::local_reader() tests the shared_ptr and then reads it again to dereference it,
# so a stop() landing between the two reads makes the second one null:
#
#   dds.ev.N   ReaderProxy initial-heartbeat event -> StatefulWriter::intraprocess_heartbeat
#              -> ReaderLocator::local_reader(): local_reader_ tested non-null, re-read as null
#              -> SIGSEGV at address 0
#   thread A   ~publisher_handle -> delete_datawriter -> ... -> RTPSParticipantImpl::deleteUserEndpoint
#              -> StatefulWriter::local_actions_on_writer_removed -> ReaderProxy::stop()
#              -> ReaderLocator::stop(): local_reader_.reset(), holding the writer mutex
#
# That is the SegFault network_recovery_concurrent_make_publisher_during_reset hit in a full
# ctest run on Linux, both stacks read from its core dump. The test creates a publisher and a
# subscriber on one topic and destroys them straight away, so the heartbeat their match arms
# fires while they are being torn down -- on every iteration. Nothing about it needs a reset: any
# same-process publisher and subscriber destroyed right after they match open the same window,
# which is merely a few instructions wide, and ThreadSanitizer had reported the race itself
# before it was ever seen to crash (the suppression it had in test/sanitizers/tsan.supp is gone,
# so a TSan run now reports it again should this patch stop applying).
#
# The fix is upstream's, eProsima/Fast-DDS#6422 (merged for v3.6.3.0, which was unreleased at
# the time of writing), applied verbatim: both functions take mp_mutex around the local_reader()
# call. It adds no lock order Fast-DDS does not already have -- deliver_sample_nts calls
# local_reader() with the writer mutex held on every sample delivered to a same-process reader --
# and it cannot deadlock against the heartbeat's own timer: a ReaderProxy is deleted only once
# local_actions_on_writer_removed has released the mutex (its ~TimedEvent waits for this very
# callback to return), and matched_reader_remove never deletes one.
#
# Like the other scripts in this directory it runs as the Fast-DDS ExternalProject
# PATCH_COMMAND, is idempotent, and FAILs loudly when an anchor has moved; like
# resource_event_per_timer_wait.cmake it writes nothing until the anchors of both files have been
# found, so a failed run leaves the sources pristine rather than half patched. A FAST_DDS_VERSION
# bump must re-check it; drop it once that version carries #6422 (v3.6.3.0 and later), which the
# failure message says outright when it finds upstream's text in place of the anchor.
#
# Invoked as:
#   cmake -DSTATEFUL_WRITER_CPP=<path-to-rtps/writer/StatefulWriter.cpp>
#         -DSTATELESS_WRITER_CPP=<path-to-rtps/writer/StatelessWriter.cpp>
#         -P local_reader_under_writer_mutex.cmake

foreach(_var IN ITEMS STATEFUL_WRITER_CPP STATELESS_WRITER_CPP)
    if(NOT DEFINED ${_var})
        message(FATAL_ERROR "local_reader_under_writer_mutex.cmake: ${_var} must be defined")
    endif()
    if(NOT EXISTS "${${_var}}")
        message(FATAL_ERROR "local_reader_under_writer_mutex.cmake: file not found: ${${_var}}")
    endif()
endforeach()

# Sources are read and written through patch_io.cmake, which keeps the line endings of the
# checkout and of the host from mattering (see there).
include("${CMAKE_CURRENT_LIST_DIR}/patch_io.cmake")

# NO REVISION MARKER HERE, deliberately, and there is a rule attached to that. A tree
# carries no record of WHICH revision of a patch script wrote it, so a bare "already patched"
# marker means only "some revision did" -- and the moment the replacement text below changes,
# every existing build tree keeps the OLD text while reporting itself patched, and the
# corrected defect ships. resource_event_per_timer_wait.cmake hit exactly that and now carries
# a _revision / _revision_marker pair plus a migration. This script does not, because its
# replacement text has never changed and a version nobody has had to bump buys nothing.
#
# So: IF YOU CHANGE THE REPLACEMENT TEXT BELOW, add that mechanism first, and make the
# migration decide from what the file CONTAINS rather than from which marker it carries --
# deciding from the marker is the second bug resource_event_per_timer_wait.cmake had, because
# a tree patched after the new text was written but before the marker existed then looks like
# an old one and the configure aborts blaming Fast-DDS for a shape change.

# Both replacements carry this tag in a comment; a file that has it is already patched. Longer
# than the bare "[provizio_dds]" of the other scripts, so that a later script patching either
# of these files cannot mistake this one's tag for its own.
set(_marker "[provizio_dds] local_reader_ under mp_mutex")

set(_stateful_anchor [==[
bool StatefulWriter::intraprocess_heartbeat(
        ReaderProxy* reader_proxy,
        bool liveliness)
{
    bool returned_value = false;
    LocalReaderPointer::Instance local_reader = reader_proxy->local_reader();

    if (local_reader)
    {
        std::unique_lock<RecursiveTimedMutex> lockW(mp_mutex);
]==])
set(_stateful_patched [==[
bool StatefulWriter::intraprocess_heartbeat(
        ReaderProxy* reader_proxy,
        bool liveliness)
{
    bool returned_value = false;
    // [provizio_dds] local_reader_ under mp_mutex: eProsima/Fast-DDS#6422, backported. This runs on
    // the timer thread (the ReaderProxy's initial heartbeat) while ReaderLocator::stop() resets
    // local_reader_ under mp_mutex, and local_reader() reads that pointer again after testing it --
    // unlocked, a stop() in between made the second read null and the dereference crash.
    std::unique_lock<RecursiveTimedMutex> lockW(mp_mutex);
    LocalReaderPointer::Instance local_reader = reader_proxy->local_reader();
    lockW.unlock();

    if (local_reader)
    {
        lockW.lock();
]==])
# How upstream's own fix reads, so a failure can say "drop this script" instead of "re-check it".
set(_stateful_upstream [==[
    std::unique_lock<RecursiveTimedMutex> lockW(mp_mutex);
    LocalReaderPointer::Instance local_reader = reader_proxy->local_reader();
    lockW.unlock();
]==])

set(_stateless_anchor [==[
bool StatelessWriter::intraprocess_delivery(
        CacheChange_t* change,
        ReaderLocator& reader_locator)
{
    LocalReaderPointer::Instance local_reader = reader_locator.local_reader();
]==])
set(_stateless_patched [==[
bool StatelessWriter::intraprocess_delivery(
        CacheChange_t* change,
        ReaderLocator& reader_locator)
{
    // [provizio_dds] local_reader_ under mp_mutex: eProsima/Fast-DDS#6422, backported.
    // ReaderLocator::stop() resets the pointer local_reader() reads under this mutex. (The one caller,
    // deliver_sample_nts, holds it already; the mutex is recursive.)
    std::lock_guard<RecursiveTimedMutex> guard(mp_mutex);
    LocalReaderPointer::Instance local_reader = reader_locator.local_reader();
]==])
set(_stateless_upstream [==[
    std::lock_guard<RecursiveTimedMutex> guard(mp_mutex);
    LocalReaderPointer::Instance local_reader = reader_locator.local_reader();
]==])

# Patch one file in memory: leave its new contents in _patched_<key> (unset when it needs no
# change), or append the reason it cannot be patched to _failures. Nothing is written here -- see
# below.
function(_local_reader_patch_in_memory key anchor replacement upstream_text)
    set(_path "${${key}}")
    get_filename_component(_name "${_path}" NAME)
    provizio_dds_patch_read("${_path}" _contents)

    string(FIND "${_contents}" "${_marker}" _already_pos)
    if(NOT _already_pos EQUAL -1)
        message(STATUS "local_reader_under_writer_mutex: ${_name} already patched -- no-op")
        return()
    endif()

    string(FIND "${_contents}" "${anchor}" _anchor_pos)
    if(_anchor_pos EQUAL -1)
        string(FIND "${_contents}" "${upstream_text}" _upstream_pos)
        if(NOT _upstream_pos EQUAL -1)
            string(CONCAT _reason "it already takes mp_mutex around local_reader() -- this Fast-DDS carries "
                                  "eProsima/Fast-DDS#6422 itself, so drop this script and its PATCH_COMMAND entry")
        else()
            set(_reason "Fast-DDS has changed shape; re-check this patch against the new sources")
        endif()
        set(_failures "${_failures}\n  ${_path}: anchor not found: ${_reason}" PARENT_SCOPE)
        return()
    endif()

    string(REPLACE "${anchor}" "${replacement}" _contents "${_contents}")
    set(_patched_${key} "${_contents}" PARENT_SCOPE)
endfunction()

set(_failures "")
_local_reader_patch_in_memory(STATEFUL_WRITER_CPP "${_stateful_anchor}" "${_stateful_patched}"
                              "${_stateful_upstream}")
_local_reader_patch_in_memory(STATELESS_WRITER_CPP "${_stateless_anchor}" "${_stateless_patched}"
                              "${_stateless_upstream}")

if(_failures)
    message(FATAL_ERROR "local_reader_under_writer_mutex.cmake: nothing was written:${_failures}")
endif()

foreach(_key IN ITEMS STATEFUL_WRITER_CPP STATELESS_WRITER_CPP)
    if(DEFINED _patched_${_key})
        provizio_dds_patch_write("${${_key}}" "${_patched_${_key}}")
        get_filename_component(_name "${${_key}}" NAME)
        message(STATUS "local_reader_under_writer_mutex: patched ${_name}")
    endif()
endforeach()
