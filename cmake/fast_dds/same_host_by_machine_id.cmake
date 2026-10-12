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

# Decide "is this remote participant on this host" the way the rest of Fast-DDS does -- by
# the machine id the participant announces -- when ParticipantFilteringFlags are applied.
#
# PDPSimple::createParticipantProxyData applies ignoreParticipantFlags -- FILTER_DIFFERENT_HOST,
# which provizio_dds sets for transport_mode::localhost_only -- by comparing the first four
# bytes of the local and remote GUID prefixes: the vendor id and the 16-bit "host id" Fast-DDS
# derives ONCE per process, at its first participant, from the IPv4 addresses of the interfaces
# that have carrier at that moment (Host::Host() in src/cpp/utils/Host.hpp). That id is
# therefore not a property of the host but of the host's network at the moment a process
# happened to start: a process started before any interface had carrier and one started after
# it, on the very same machine, carry different ids, are each other's "different host", and
# refuse each other's announcements for as long as both live. (See also
# host_id_without_interfaces.cmake, which only keeps cold processes of DIFFERENT hosts from
# sharing one id; it cannot make a cold id equal a warm one.)
#
# Everything else in Fast-DDS that needs to know whether a participant is local -- which
# remote locators to transform to loopback, the type lookup service, discovery server/client,
# security -- asks ParticipantProxyData::is_from_this_host(), which compares the machine id
# carried in the participant's announcement (PID_MACHINE_ID) and falls back to the GUID
# prefix only for a participant that announces none. The filter is the one place that skips it.
# The patch makes it ask too: "same host" is the machine id matching OR the GUID host bytes
# matching.
#
# Consequences to know:
#   - Where no machine id can be read, or two processes of one machine announce different ones
#     (a 32-bit Windows process reads another registry view than a 64-bit one; containers of
#     different images each carry their own /etc/machine-id), the GUID comparison still decides
#     as it did, so every pair that was on the same host before still is.
#   - The result also feeds the FILTER_DIFFERENT_PROCESS / FILTER_SAME_PROCESS branch below the
#     host test. Where a caller sets FILTER_DIFFERENT_PROCESS WITHOUT FILTER_DIFFERENT_HOST, a
#     peer that used to be judged "another host" -- and so accepted -- is now judged another
#     process of this host, and refused: a process of this machine whose GUID host bytes differ,
#     or a host with the same machine id. The documented recipe,
#     FILTER_DIFFERENT_HOST | FILTER_DIFFERENT_PROCESS, is unaffected.
#   - It relies on machine ids being unique per host, as Fast-DDS's own locator transformation
#     already does. A host cloned from an image with its /etc/machine-id (or the Windows
#     MachineGuid) left in place is taken for this one: FILTER_DIFFERENT_HOST no longer refuses
#     it, so under transport_mode::localhost_only its participant can become a proxy here -- a
#     peer that merely announces itself, since it never receives our announcements back, and
#     whose samples loopback-only transports keep from arriving. Give such hosts their own
#     ids (systemd-machine-id-setup).
#
# Limits, neither of them new: a cold and a warm process still do not share SHM (the SHM locator
# embeds the GUID host id; they fall back to UDP over loopback), and a peer built with stock
# Fast-DDS still compares GUID bytes on its own side wherever IT applies FILTER_DIFFERENT_HOST
# (a participant with that flag set by its own code or XML), so such a peer started under
# another network state still refuses us. A ROS 2 node does not apply the flag in its localhost-only
# mode (rmw_fastrtps confines Humble by a loopback UDP allowlist and Jazzy and later by dropping
# multicast and peering with 127.0.0.1), so it is not such a peer. Discovery Server / Client modes
# never read the filter.
#
# Upstream: reported on eProsima/Fast-DDS#6524 (the same per-process host id, with a suggested one-line
# fix and a stable-machine-id proposal). A FAST_DDS_VERSION bump must re-check this patch, and it can be
# dropped once PDPSimple::createParticipantProxyData asks is_from_this_host() -- the anchor below then no
# longer exists and the configure says so.
#
# This runs as a Fast-DDS ExternalProject patch step (see PROVIZIO_DDS_FAST_DDS_PATCHES in the
# top-level CMakeLists.txt). Like the others it is:
#   - Idempotent: re-running it on an already patched file is a no-op.
#   - Self-checking: if the anchor is missing (a future Fast-DDS reshuffles PDPSimple) it
#     FAILs loudly rather than silently building a library without the fix.
#   - Covered end to end by test/network_recovery/same_host_filter_test.sh via
#     test/network_recovery/CMakeLists.txt.
#
# Invoked as:
#   cmake -DPDP_SIMPLE_CPP=<path-to-PDPSimple.cpp> -P same_host_by_machine_id.cmake

if(NOT DEFINED PDP_SIMPLE_CPP)
    message(FATAL_ERROR "same_host_by_machine_id.cmake: PDP_SIMPLE_CPP must be defined")
endif()

if(NOT EXISTS "${PDP_SIMPLE_CPP}")
    message(FATAL_ERROR "same_host_by_machine_id.cmake: file not found: ${PDP_SIMPLE_CPP}")
endif()

# Sources are read and written through patch_io.cmake, which keeps the line endings of the
# checkout and of the host from mattering (see there).
include("${CMAKE_CURRENT_LIST_DIR}/patch_io.cmake")
provizio_dds_patch_read("${PDP_SIMPLE_CPP}" _contents)

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
string(FIND "${_contents}" "same_host_by_machine_id.cmake" _already_pos)
if(NOT _already_pos EQUAL -1)
    message(STATUS "same_host_by_machine_id: PDPSimple.cpp already patched -- no-op")
    return()
endif()

set(_anchor [==[
    bool is_same_host = local.is_on_same_host_as(remote);
]==])
set(_patched [==[
    // [provizio_dds] Also accept a remote that announces this machine's id: the GUID host bytes
    // are derived once per process from whichever interfaces had carrier when it started, so
    // two processes of one machine started before and after the network came up disagree on
    // them. OR-ed with the GUID comparison, never replacing it: whoever matched before still
    // does, e.g. where one process cannot read the machine id another can. See
    // same_host_by_machine_id.cmake in provizio_dds.
    bool is_same_host = participant_data.is_from_this_host() || local.is_on_same_host_as(remote);
]==])

string(FIND "${_contents}" "${_anchor}" _pos)
if(_pos EQUAL -1)
    message(FATAL_ERROR
        "same_host_by_machine_id: could not find the expected code\n"
        "${_anchor}\n"
        "in ${PDP_SIMPLE_CPP}. Fast-DDS may have changed PDPSimple::createParticipantProxyData; "
        "update this patch, or drop it if upstream now asks is_from_this_host() there "
        "(eProsima/Fast-DDS#6524), so FILTER_DIFFERENT_HOST keeps recognising a process of this host "
        "that started under another network state (provizio_dds transport_mode::localhost_only "
        "depends on it -- see test/network_recovery/same_host_filter_test.sh).")
endif()

string(REPLACE "${_anchor}" "${_patched}" _contents "${_contents}")
provizio_dds_patch_write("${PDP_SIMPLE_CPP}" "${_contents}")
message(STATUS "same_host_by_machine_id: FILTER_DIFFERENT_HOST judges the host by machine id")
