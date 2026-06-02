# Decorate eprosima::SystemInfo::update_interfaces() with FASTDDS_EXPORTED_API
# so the symbol is present in the Windows fastdds.dll export table.
#
# Why provizio_dds needs it: the network-auto-recovery feature (APT-11792)
# calls SystemInfo::update_interfaces() before recreating a participant on a
# network change, to refresh Fast-DDS's process-wide interface cache (which is
# otherwise populated once and never refreshed — so a rebuilt participant
# would bind to stale interfaces). SystemInfo is a Fast-DDS-internal class with
# zero FASTDDS_EXPORTED_API decoration, so on Windows the symbol isn't in the
# DLL export table and provizio_dds can't link against it.
#
# Why this targeted patch rather than CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS: Fast-DDS
# is large enough that auto-exporting every symbol overflows the Windows
# per-DLL export ceiling (LNK1189: "library limit of 65535 objects exceeded").
# Decorating the single symbol we need adds exactly one export.
#
# This runs as the Fast-DDS ExternalProject PATCH_COMMAND. It is:
#   - Cross-platform: on Linux / macOS FASTDDS_EXPORTED_API expands to the
#     default-visibility attribute, which is already the default — so the
#     patch is a harmless no-op there. We still apply it unconditionally to
#     keep one code path.
#   - Idempotent: if the symbol is already decorated (e.g. a re-run of the
#     patch step) it returns without modifying the file.
#   - Self-checking: if the expected declaration can't be found (a future
#     Fast-DDS reshuffles SystemInfo.hpp) it FAILs loudly rather than
#     silently leaving the symbol unexported and shipping a broken Windows
#     build.
#
# FASTDDS_EXPORTED_API is already in scope in SystemInfo.hpp transitively via
# its `#include <fastdds/utils/IPFinder.hpp>` (IPFinder decorates its own API
# with the macro), so no extra include is needed.
#
# Invoked as:
#   cmake -DSYSTEMINFO_HPP=<path-to-SystemInfo.hpp> -P export_system_info.cmake

if(NOT DEFINED SYSTEMINFO_HPP)
    message(FATAL_ERROR "export_system_info.cmake: SYSTEMINFO_HPP must be defined")
endif()

if(NOT EXISTS "${SYSTEMINFO_HPP}")
    message(FATAL_ERROR "export_system_info.cmake: file not found: ${SYSTEMINFO_HPP}")
endif()

set(_undecorated "    static bool update_interfaces();")
set(_decorated "    FASTDDS_EXPORTED_API static bool update_interfaces();")

file(READ "${SYSTEMINFO_HPP}" _contents)

string(FIND "${_contents}" "${_decorated}" _already_pos)
if(NOT _already_pos EQUAL -1)
    message(STATUS "export_system_info: update_interfaces already exported — no-op")
    return()
endif()

string(FIND "${_contents}" "${_undecorated}" _pos)
if(_pos EQUAL -1)
    message(FATAL_ERROR
        "export_system_info: could not find the expected declaration\n"
        "  '${_undecorated}'\n"
        "in ${SYSTEMINFO_HPP}. Fast-DDS may have changed SystemInfo's layout; "
        "update this patch so the Windows export of "
        "eprosima::SystemInfo::update_interfaces is preserved (provizio_dds "
        "network auto-recovery depends on it — see APT-11792).")
endif()

string(REPLACE "${_undecorated}" "${_decorated}" _contents "${_contents}")
file(WRITE "${SYSTEMINFO_HPP}" "${_contents}")
message(STATUS "export_system_info: decorated SystemInfo::update_interfaces with FASTDDS_EXPORTED_API")
