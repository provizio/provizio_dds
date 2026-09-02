# Give a process that starts before any network interface has carrier a host id of its
# own, instead of the one fixed value Fast-DDS otherwise assigns every such process.
#
# Fast-DDS derives a process-wide 16-bit "host id" once, in Host's constructor
# (src/cpp/utils/Host.hpp), from the IPv4 addresses of the interfaces that have carrier at
# that moment -- IPFinder::getIPs skips everything without IFF_RUNNING -- and falls back
# to a fixed value, 0x017F, when there are none. That id goes into every participant GUID
# prefix (GuidUtils) and into every shared-memory locator the participant announces
# (SHMLocator::create_locator), and a REMOTE shared-memory locator is taken to be
# reachable exactly when it carries this process's id and names a port that exists
# locally (SHMLocator::is_shm_and_from_this_host, SharedMemTransport::is_locator_reachable
# via ProxyDataFilters::filter_locators). The machine id Fast-DDS also exchanges in
# discovery does not enter that check.
#
# Why provizio_dds needs it (APT-12250): a service that starts at boot, before the
# network switch has powered on, creates its first participant while no interface has
# carrier. Every such process, on every host, then carries 0x017F. Network auto-recovery
# rebuilds the participant once the link comes up -- but the id is a process-lifetime
# singleton, so the rebuilt participant carries it too. Two radars booted together each
# took the other's shared-memory locators for their own host's (the port numbers exist
# locally: both derive from the same participant index), sent the other's traffic into
# their own segment, and never heard each other for as long as both processes lived --
# while a freshly started process, whose id was right, talked to both at once.
#
# The patch: where the interface list is empty, derive the id from the machine id instead
# (compute_machine_id: /etc/machine-id, the Windows MachineGuid, the macOS platform UUID),
# folded to 16 bits exactly as compute_id folds the address hash. That is the identity
# Fast-DDS itself already relies on for participant-level same-host detection, so it is
# unique per host and stable for the life of the process. Only where no machine id can be
# read either does the fixed value remain. A process started WITH an interface is left
# exactly as before, so same-host detection between it and a stock Fast-DDS participant
# on the same host (a ROS 2 node, say) is unchanged.
#
# Deliberately not a refresh on network recovery: recomputing the id then would change,
# mid-life, the identity of every participant already created in the process and of every
# shared-memory locator they have announced. A cold-started process keeps the
# machine-id-derived id; it differs from the address-derived one a process started after
# the network came up computes, so the two reach each other over loopback rather than
# shared memory -- which is exactly what they do today, and correct.
#
# This runs as the Fast-DDS ExternalProject PATCH_COMMAND, after export_system_info.cmake.
# Like that script it is:
#   - Idempotent: re-running it on an already patched file is a no-op.
#   - Self-checking: if either anchor is missing (a future Fast-DDS reshuffles Host.hpp) it
#     FAILs loudly rather than silently building a library without the fix.
#   - Covered end to end by test/network_recovery/cold_start_hosts_test.sh.
#
# Invoked as:
#   cmake -DHOST_HPP=<path-to-Host.hpp> -P host_id_without_interfaces.cmake

if(NOT DEFINED HOST_HPP)
    message(FATAL_ERROR "host_id_without_interfaces.cmake: HOST_HPP must be defined")
endif()

if(NOT EXISTS "${HOST_HPP}")
    message(FATAL_ERROR "host_id_without_interfaces.cmake: file not found: ${HOST_HPP}")
endif()

file(READ "${HOST_HPP}" _contents)

set(_marker "compute_id_without_interfaces")
string(FIND "${_contents}" "${_marker}" _already_pos)
if(NOT _already_pos EQUAL -1)
    message(STATUS "host_id_without_interfaces: Host.hpp already patched -- no-op")
    return()
endif()

# Anchor 1: the constructor's id computation. The fallback is appended right after it.
set(_ctor_anchor [==[
        fastdds::rtps::IPFinder::getIP4Address(&loc);
        id_ = compute_id(loc);
]==])
set(_ctor_patched [==[
        fastdds::rtps::IPFinder::getIP4Address(&loc);
        id_ = compute_id(loc);

        // [provizio_dds] No interface has carrier yet: derive the id from the machine id,
        // so this process is not given the one fixed value every such process on every
        // host would otherwise share. See host_id_without_interfaces.cmake in provizio_dds.
        if (loc.size() == 0)
        {
            id_ = compute_id_without_interfaces();
        }
]==])

# Anchor 2: the private helper declarations. The new helper is inserted before the first.
set(_decl_anchor [==[
    static fastcdr::string_255 compute_machine_id();
]==])
set(_decl_patched [==[
    // [provizio_dds] The host id of a process none of whose interfaces has carrier: the
    // machine id folded to 16 bits the way compute_id folds the address hash, or
    // compute_id's own fixed fallback where no machine id can be read either. See
    // host_id_without_interfaces.cmake in provizio_dds.
    static uint16_t compute_id_without_interfaces()
    {
        const fastcdr::string_255 machine_id = compute_machine_id();
        if (machine_id.size() == 0)
        {
            return compute_id(fastdds::rtps::LocatorList());
        }

        fastdds::MD5 md5;
        md5.update(machine_id.c_str(), static_cast<fastdds::MD5::size_type>(machine_id.size()));
        md5.finalize();

        uint16_t ret_val = 0;
        for (size_t i = 0; i < sizeof(md5.digest); i += 2)
        {
            uint16_t tmp = static_cast<uint16_t>(md5.digest[i]);
            tmp = (tmp << 8) | static_cast<uint16_t>(md5.digest[i + 1]);
            ret_val ^= tmp;
        }
        return ret_val;
    }

    static fastcdr::string_255 compute_machine_id();
]==])

foreach(_anchor IN ITEMS _ctor_anchor _decl_anchor)
    string(FIND "${_contents}" "${${_anchor}}" _pos)
    if(_pos EQUAL -1)
        message(FATAL_ERROR
            "host_id_without_interfaces: could not find the expected code\n"
            "${${_anchor}}\n"
            "in ${HOST_HPP}. Fast-DDS may have changed Host's layout; update this patch so a "
            "process started before any interface has carrier keeps getting a host id of its "
            "own (provizio_dds network auto-recovery depends on it -- see APT-12250 and "
            "test/network_recovery/cold_start_hosts_test.sh).")
    endif()
endforeach()

string(REPLACE "${_ctor_anchor}" "${_ctor_patched}" _contents "${_contents}")
string(REPLACE "${_decl_anchor}" "${_decl_patched}" _contents "${_contents}")
file(WRITE "${HOST_HPP}" "${_contents}")
message(STATUS "host_id_without_interfaces: Host derives its id from the machine id where no interface has carrier")
