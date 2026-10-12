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

# Decide whether an address a participant of this host announces is one of this host's own from
# the interfaces as they are now, not as they were when the interface list was last refreshed.
#
# When a remote participant announces this machine's id (ParticipantProxyData::is_from_this_host(),
# which compares the PID_MACHINE_ID eProsima Fast-DDS announces from 3.2 on, and falls back to the
# GUID's host bytes for a participant announcing none), Fast-DDS passes each unicast address it
# announces through UDPTransportInterface::transform_remote_locator. That turns an address of this
# host into 127.0.0.1 when the peer announces that it accepts localhost and this transport may use
# loopback -- so that a participant of this host is reached over loopback, whichever of its addresses
# it named -- and leaves any other address as it is. (It is also asked once about each initial peer
# of a participant being created, where only TCP uses the answer.) Whether the address is this
# host's is asked of is_local_locator, which compares it with the process-wide interface list
# SystemInfo caches: the interfaces that had carrier when it was last refreshed. Fast-DDS refreshes it once, when
# the process first uses SystemInfo, and never again; provizio_dds refreshes it before creating each
# participant (src/domain_participant.cpp) and so on every network-recovery rebuild, but
# transport_mode::localhost_only takes no part in recovery. Between two refreshes the list stays as
# it was.
#
# So a process whose last participant was created before the network came up -- or before the host's
# address changed -- holds a list without the host's current addresses until it creates another, and
# takes every address a participant started after that announces for another host's. A participant
# whose UDP transport has no interface list sends to the address as it is, and the kernel delivers it
# locally. But one whose transport has a list -- an allowlist, or a blocklist, which Fast-DDS turns
# into an allowlist of the interfaces left when the transport is built -- can only use the address if
# it is turned into 127.0.0.1: left as it is, ReaderProxyData / WriterProxyData drop it
# (is_locator_remote_or_allowed refuses a local participant's address that the list does not hold).
# Above all that is provizio_dds transport_mode::localhost_only, whose allowlist is 127.0.0.1; also
# provizio_dds's VPN exclusion while a tunnel is up (until a network-recovery rebuild re-reads the
# peer, or for good without recovery), and a caller's own allowlist. A peer that announced no loopback address of its own -- a recorder in the
# ordinary transport, started once the network was up -- is then left with no address at all: the
# confined participant sends it nothing beyond its periodic announcements to the loopback initial
# peers -- no reply, no endpoint, no sample, no acknowledgement -- while that peer, which can reach
# it, keeps answering them. A recording made that way holds no message. same_host_by_machine_id.cmake
# made the discovery filter take such a peer for this host's; this makes its addresses this host's too.
#
# Both need the peer to announce this machine's id, i.e. to run on eProsima Fast-DDS 3.2 or later. A
# peer on an older one -- a ROS 2 Humble, Iron or Jazzy node (Fast-DDS 2.x), provizio_dds 1.x --
# announces none, so whether it is this host's is decided by the GUID host bytes, which differ between
# a process started before the network came up and one started after it: such a pair is still taken
# for two hosts, refused by FILTER_DIFFERENT_HOST and never transformed. This patch cannot change that.
#
# The patch, in two files:
#   - UDPTransportInterface.cpp: when an announced unicast address is not in the list, refresh the
#     list (SystemInfo::update_interfaces(), the very call provizio_dds makes) and ask again. Only an
#     address this host really has is turned into loopback, as before. The refreshes this makes are
#     rate-limited, process-wide: a burst of up to 8, then 20 a second. Whenever it declines, a
#     refresh that started less than 50 ms earlier has completed, so an address is only taken for
#     another host's by a list at most 50 ms old -- or by the last one an enumeration managed to read,
#     should enumerating fail (a refresh that fails leaves the list as it was and still counts). An
#     address the list holds is taken for this host's however old the list, as before.
#   - SystemInfo.cpp: refreshes run one at a time, enumeration and store together, so the list left is
#     that of the last enumeration to succeed, and none older can replace it. As shipped
#     the enumeration ran outside any lock, so of two concurrent refreshes the older could finish last
#     and replace the newer list. Refreshes now also run on discovery threads, alongside
#     provizio_dds's own before a participant is created, whose transports are built from that list.
#   - Both mutexes the patch adds are deliberately never destroyed: a participant still alive when
#     the process exits is deleted by DomainParticipantFactory's destructor, and until then its
#     receive threads keep parsing announcements, so a function-local static mutex -- destroyed
#     before that factory, having been constructed after it -- could be locked after its end (with
#     libc++ off glibc, as on macOS, the lock then throws on a thread that catches nothing).
#
# Consequences to know:
#   - The refresh replaces the process-wide list, as provizio_dds's own refresh does, and every
#     participant of the process sees the current interfaces from then on where it would have seen
#     those of the last refresh: when a transport next resolves its interfaces (its first output
#     channel, an allowlist being matched) and in later address checks. Already open send resources
#     are untouched.
#   - Cost: one interface enumeration (getifaddrs, or GetAdaptersAddresses on Windows -- tens of
#     microseconds on Linux, possibly milliseconds on Windows) per announced unicast address the list
#     lacks, and none once a refresh has added it; plus one for each initial peer the list lacks, as a
#     participant is created. Fast-DDS parses every copy of an announcement that arrives, under its
#     discovery mutex, so an address that stays unknown is looked for again on each copy -- that of a
#     participant in another network namespace or container that shares the machine id (a
#     bind-mounted /etc/machine-id), of every host of a fleet cloned with one machine id, of a peer
#     still announcing an address the host has lost, or an announcement crafted to name thousands of
#     addresses. The rate limit bounds all of it to 8 + 20 a second, which is also what an attacker
#     able to announce this machine's id (it is sent in clear by every participant of the host) can
#     cost, in place of one per address named. A declined refresh still costs a second look at the
#     list, as an accepted one does.
#   - The limit is process-wide, so whatever keeps it drained -- such a stream, or an attacker -- also
#     leaves a real peer of this host whose address came up less than about 50 ms before this process
#     first reads its announcement judged by a list without it, as a stock Fast-DDS would judge it.
#   - "Has it now" means what Fast-DDS's interface list means: the address of an interface with
#     carrier. An address announced by a peer whose interface has lost carrier by the time this
#     process first reads its announcement is still taken for another host's. A peer's proxy keeps
#     what that first reading found until the peer announces itself anew with other data (a restart,
#     or a provizio_dds rebuild): later copies of the same announcement are not re-applied. That holds
#     for this case and the one above alike. The peer can only have announced the address while its
#     interface had carrier, so this takes a link going down between the peer's start and this
#     process first hearing of it.
#   - A participant of another network namespace that shares the machine id keeps the addresses this
#     host does not have, as it should; one using an address this host ALSO has is taken for a
#     participant of this host and reached at 127.0.0.1 -- where a process of this host bound to the
#     same port may be. That is how Fast-DDS already treated an address its list held.
#   - A process started before the network came up still carries another host id than one started
#     after it, so the two share no shared memory (the shared-memory locator embeds that id, see
#     host_id_without_interfaces.cmake): they reach each other over loopback UDP.
#
# Upstream: the same per-process view of the host as eProsima/Fast-DDS#6524, though no upstream issue
# covers this path yet. A FAST_DDS_VERSION bump must re-check this patch -- not only whether its
# anchors still exist: an upstream fix could land in is_local_locator or SystemInfo instead and leave
# the anchor in transform_remote_locator as it is, which the configure cannot notice. Drop it once
# Fast-DDS decides an address's locality from interfaces it refreshes and refreshes them one at a time.
#
# This runs as a Fast-DDS ExternalProject patch step (see PROVIZIO_DDS_FAST_DDS_PATCHES in the
# top-level CMakeLists.txt). Like the others it is:
#   - Idempotent: re-running it on an already patched file is a no-op.
#   - Self-checking: if an anchor is missing (a future Fast-DDS reshuffles either file) it FAILs
#     loudly rather than silently building a library without the fix.
#   - Covered end to end, on Linux, by the privileged test/network_recovery/same_host_filter_test.sh
#     via test/network_recovery/CMakeLists.txt, which needs network namespaces; and, unprivileged, on
#     Linux and macOS by test/fast_dds_patches/local_address_test.cpp (fast_dds_patch_local_address),
#     which empties SystemInfo's list -- the state a process created cold is in -- and drives
#     transform_remote_locator: the host's own address is turned into loopback, another or a multicast
#     address is not, and the refreshes stay within the rate limit. The serialisation of concurrent
#     refreshes is not tested: nothing short of injecting enumerations could order two of them.
#
# Invoked as:
#   cmake -DUDP_TRANSPORT_INTERFACE_CPP=<path-to-UDPTransportInterface.cpp>
#         -DSYSTEM_INFO_CPP=<path-to-SystemInfo.cpp> -P local_address_by_current_interfaces.cmake

foreach(_var IN ITEMS UDP_TRANSPORT_INTERFACE_CPP SYSTEM_INFO_CPP)
    if(NOT DEFINED ${_var})
        message(FATAL_ERROR "local_address_by_current_interfaces.cmake: ${_var} must be defined")
    endif()
    if(NOT EXISTS "${${_var}}")
        message(FATAL_ERROR "local_address_by_current_interfaces.cmake: file not found: ${${_var}}")
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

# _patch_file(<path> <name> <anchor var> <replacement var> [<anchor var> <replacement var>]...):
# applies each replacement to <path> unless the file already names this script, failing loudly
# when an anchor is missing, and writes the file once.
function(_patch_file path name)
    provizio_dds_patch_read("${path}" _contents)
    string(FIND "${_contents}" "local_address_by_current_interfaces.cmake" _already_pos)
    if(NOT _already_pos EQUAL -1)
        message(STATUS "local_address_by_current_interfaces: ${name} already patched -- no-op")
        return()
    endif()
    set(_pairs ${ARGN})
    while(_pairs)
        list(POP_FRONT _pairs _anchor_var _patched_var)
        string(FIND "${_contents}" "${${_anchor_var}}" _pos)
        if(_pos EQUAL -1)
            message(FATAL_ERROR
                "local_address_by_current_interfaces: could not find the expected code\n"
                "${${_anchor_var}}\n"
                "in ${path}. Fast-DDS may have changed ${name}; update this patch, or drop it if upstream "
                "now decides whether an address is this host's from interfaces it refreshes, one refresh at a "
                "time, so a participant confined to loopback keeps reaching a peer of this host started after "
                "the network came up (provizio_dds transport_mode::localhost_only depends on it -- see "
                "test/network_recovery/same_host_filter_test.sh).")
        endif()
        string(REPLACE "${${_anchor_var}}" "${${_patched_var}}" _contents "${_contents}")
    endwhile()
    provizio_dds_patch_write("${path}" "${_contents}")
    message(STATUS "local_address_by_current_interfaces: patched ${name}")
endfunction()

# UDPTransportInterface.cpp: what the rate-limited refresh needs, after an include it has; the refresh
# itself, ahead of the function that asks for it; and the question asked again after it.
set(_include_anchor [==[
#include <rtps/transport/UDPSenderResource.hpp>
]==])
set(_include_patched [==[
#include <rtps/transport/UDPSenderResource.hpp>
// [provizio_dds] The rate-limited SystemInfo::update_interfaces(), see local_address_by_current_interfaces.cmake
#include <mutex>
#include <utils/SystemInfo.hpp>
]==])

set(_refresh_anchor [==[
bool UDPTransportInterface::transform_remote_locator(
        const Locator& remote_locator,
]==])
set(_refresh_patched [==[
namespace {

// [provizio_dds] Refreshes SystemInfo's interface list for transform_remote_locator, process-wide at
// most interface_refreshes_per_second on average, in bursts of up to interface_refresh_burst, so that
// announcements naming addresses this host does not have cost a bounded number of enumerations however
// many they name. A token is taken under the mutex and the refresh made under it too, so whenever this
// declines, a refresh that started less than 1 / interface_refreshes_per_second earlier has completed.
// The mutex is never destroyed: receive threads of a participant still alive at exit run until
// DomainParticipantFactory's destructor deletes it, after a function-local static would have ended.
// See local_address_by_current_interfaces.cmake in provizio_dds.
void provizio_dds_refresh_interfaces_rate_limited()
{
    constexpr double interface_refreshes_per_second = 20.0;
    constexpr double interface_refresh_burst = 8.0;
    static std::mutex& mutex = *new std::mutex();
    static double tokens = interface_refresh_burst;
    static std::chrono::steady_clock::time_point last_refill = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(mutex);
    const auto now = std::chrono::steady_clock::now();
    tokens = (std::min)(interface_refresh_burst,
                    tokens + std::chrono::duration<double>(now - last_refill).count() * interface_refreshes_per_second);
    last_refill = now;
    if (tokens < 1.0)
    {
        return;
    }
    tokens -= 1.0;
    SystemInfo::update_interfaces();
}

} // namespace

bool UDPTransportInterface::transform_remote_locator(
        const Locator& remote_locator,
]==])

set(_transform_anchor [==[
        result_locator = remote_locator;
        if (!is_local_locator(result_locator))
        {
            // is_local_locator will return false for multicast addresses as well as remote unicast ones.
            return true;
        }
]==])
set(_transform_patched [==[
        result_locator = remote_locator;
        // [provizio_dds] is_local_locator compares with the interface list as it was last refreshed, which
        // for a process whose last participant was created before the network came up (or before the
        // host's address changed) lacks the addresses this host has now. This is asked about a participant
        // announcing this machine's id (and about each initial peer of a participant being created), so a
        // unicast address that the list lacks may well be one the host gained since, and taking it for
        // another host's would leave a participant confined to loopback no way to reach that peer. Look at
        // the interfaces as they are now before deciding, as often as the rate limit allows (see
        // local_address_by_current_interfaces.cmake in provizio_dds for what that guarantees).
        bool is_local = is_local_locator(result_locator);
        if (!is_local && !IPLocator::isMulticast(result_locator))
        {
            provizio_dds_refresh_interfaces_rate_limited();
            is_local = is_local_locator(result_locator);
        }
        if (!is_local)
        {
            // is_local_locator will return false for multicast addresses as well as remote unicast ones.
            return true;
        }
]==])

_patch_file("${UDP_TRANSPORT_INTERFACE_CPP}" "UDPTransportInterface::transform_remote_locator"
    _include_anchor _include_patched _refresh_anchor _refresh_patched _transform_anchor _transform_patched)

# SystemInfo.cpp: one refresh at a time, enumeration and store together
set(_update_anchor [==[
bool SystemInfo::update_interfaces()
{
    std::vector<IPFinder::info_IP> ifaces;
]==])
set(_update_patched [==[
bool SystemInfo::update_interfaces()
{
    // [provizio_dds] One refresh at a time, the enumeration and the store together, so that no
    // enumeration older than the list already stored can replace it. Refreshes run on discovery threads (see
    // UDPTransportInterface::transform_remote_locator) as well as before a participant is created, and
    // with the enumeration outside any lock an older one finishing later replaced a newer list. Never
    // destroyed, as receive threads of a participant still alive at exit can refresh until
    // DomainParticipantFactory's destructor deletes it. See local_address_by_current_interfaces.cmake in
    // provizio_dds.
    static std::mutex& refresh_mtx = *new std::mutex();
    std::lock_guard<std::mutex> refresh_lock(refresh_mtx);
    std::vector<IPFinder::info_IP> ifaces;
]==])

_patch_file("${SYSTEM_INFO_CPP}" "SystemInfo::update_interfaces" _update_anchor _update_patched)

# The tests reach SystemInfo's interface list by declaring its members themselves
# (test/fast_dds_patches/local_address_test.cpp, test/network_recovery/network_recovery_test.cpp). A
# static data member's mangled name carries no type, so a Fast-DDS that changed one of them would still
# link, and those tests would then clear or lock memory as the wrong type: fail here instead.
provizio_dds_patch_read("${SYSTEM_INFO_CPP}" _system_info)
foreach(_definition IN ITEMS
        "bool SystemInfo::cached_interfaces_;"
        "std::vector<IPFinder::info_IP> SystemInfo::interfaces_;"
        "std::mutex SystemInfo::interfaces_mtx_;")
    string(FIND "${_system_info}" "${_definition}" _pos)
    if(_pos EQUAL -1)
        message(FATAL_ERROR
            "local_address_by_current_interfaces: ${SYSTEM_INFO_CPP} no longer defines\n  ${_definition}\n"
            "test/fast_dds_patches/local_address_test.cpp and test/network_recovery/network_recovery_test.cpp "
            "declare that member themselves to reach Fast-DDS's interface list; update their declarations to "
            "match, then this list.")
    endif()
endforeach()
