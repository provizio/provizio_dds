# Places the shared OpenSSL libraries that Fast-DDS's installed libraries load next to them, and
# removes the ones placed there before. Run as a step of the Fast-DDS ExternalProject, after its
# install, by provizio_dds's CMakeLists.txt, which says why they go there.
#
# Which libraries is read from the libraries in DESTINATION with TOOL, as TOOL_KIND says:
#   dumpbin  (Windows)  the OpenSSL DLLs they import;
#   readelf  (Linux)    the OpenSSL SONAMEs they need, but those found in one of SYSTEM_DIRS, the
#                       system's own directories, where the loader looks anyway;
#   otool    (macOS)    the OpenSSL libraries they name by an @rpath-relative install name, dyld
#                       loading one named by an absolute path from that path;
# and so on for the libraries placed, as libssl imports libcrypto. Each is looked for in SEARCH_DIRS,
# the directories of the OpenSSL that Fast-DDS was built against, and copied under the name it is
# loaded by. A static OpenSSL is loaded as none, so nothing is placed for it.
#
#   cmake -DDESTINATION=<directory> -DSEARCH_DIRS=<directory>[|<directory>...]
#         [-DSYSTEM_DIRS=<directory>[|<directory>...]] [-DTOOL=<command>] [-DTOOL_KIND=<kind>]
#         -P openssl_runtime.cmake
#
# Lists arrive |-separated, or ;-separated where ExternalProject has put back its LIST_SEPARATOR.
# TOOL can be a command with arguments of its own, which its own arguments follow. Without a TOOL
# the script only removes.

cmake_minimum_required(VERSION 3.15)

if(NOT DESTINATION)
    message(FATAL_ERROR "openssl_runtime.cmake: DESTINATION is required")
endif()
foreach(_list IN ITEMS SEARCH_DIRS SYSTEM_DIRS TOOL)
    string(REPLACE "|" ";" ${_list} "${${_list}}")
endforeach()

# Fast-DDS installs no OpenSSL of its own, so any here is one this script placed
file(GLOB _stale "${DESTINATION}/libssl*" "${DESTINATION}/libcrypto*")
if(_stale)
    file(REMOVE ${_stale})
endif()
if(NOT TOOL)
    return()
endif()

if(TOOL_KIND STREQUAL "dumpbin")
    # dumpbin takes its options with - as well as /, and - is the one no shell mistakes for a path
    set(_tool_args -NOLOGO -DEPENDENTS)
    set(_library_regex "\\.[dD][lL][lL]$")
elseif(TOOL_KIND STREQUAL "readelf")
    set(_tool_args -d)
    set(_library_regex "\\.so(\\.[0-9]+)*$")
elseif(TOOL_KIND STREQUAL "otool")
    set(_tool_args -L)
    set(_library_regex "\\.dylib$")
else()
    message(FATAL_ERROR "openssl_runtime.cmake: unknown TOOL_KIND '${TOOL_KIND}'")
endif()

# The OpenSSL libraries a file loads, by the names the loader looks them up by
function(_openssl_imports file out)
    execute_process(COMMAND ${TOOL} ${_tool_args} "${file}"
        OUTPUT_VARIABLE _output RESULT_VARIABLE _result ERROR_VARIABLE _error)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "openssl_runtime.cmake: '${TOOL}' could not read ${file}: ${_error}")
    endif()
    string(REPLACE "\r" "" _output "${_output}")
    string(REPLACE ";" "," _output "${_output}")
    string(REPLACE "\n" ";" _lines "${_output}")
    set(_names)
    foreach(_line IN LISTS _lines)
        if(TOOL_KIND STREQUAL "dumpbin")
            # One per line, indented, under "Image has the following dependencies:"
            string(TOLOWER "${_line}" _line)
            if(_line MATCHES "^[ \t]+(lib(ssl|crypto)[^ \t]*\\.dll)[ \t]*$")
                list(APPEND _names "${CMAKE_MATCH_1}")
            endif()
        elseif(TOOL_KIND STREQUAL "readelf")
            # " 0x0000000000000001 (NEEDED)  Shared library: [libssl.so.3]"
            if(_line MATCHES "\\(NEEDED\\).*\\[(lib(ssl|crypto)\\.so[^]]*)\\]")
                list(APPEND _names "${CMAKE_MATCH_1}")
            endif()
        else()
            # A tab, then "@rpath/libssl.3.dylib (compatibility version 3.0.0, current version 3.0.0)"
            if(_line MATCHES "^[ \t]+@rpath/(lib(ssl|crypto)[^ \t/(]*\\.dylib)")
                list(APPEND _names "${CMAKE_MATCH_1}")
            endif()
        endif()
    endforeach()
    set(${out} "${_names}" PARENT_SCOPE)
endfunction()

set(_system_dirs)
foreach(_dir IN LISTS SYSTEM_DIRS)
    get_filename_component(_dir "${_dir}" REALPATH)
    list(APPEND _system_dirs "${_dir}")
endforeach()

# The libraries installed here, each once: a link names the file it points at
set(_queue)
file(GLOB _candidates "${DESTINATION}/*")
foreach(_candidate IN LISTS _candidates)
    if(NOT IS_DIRECTORY "${_candidate}" AND _candidate MATCHES "${_library_regex}")
        get_filename_component(_candidate "${_candidate}" REALPATH)
        list(APPEND _queue "${_candidate}")
    endif()
endforeach()
list(REMOVE_DUPLICATES _queue)

set(_seen)
while(_queue)
    list(POP_FRONT _queue _file)
    _openssl_imports("${_file}" _names)
    foreach(_name IN LISTS _names)
        if(_name IN_LIST _seen)
            continue()
        endif()
        list(APPEND _seen "${_name}")
        set(_found)
        foreach(_dir IN LISTS SEARCH_DIRS)
            if(EXISTS "${_dir}/${_name}")
                set(_found "${_dir}/${_name}")
                break()
            endif()
        endforeach()
        if(NOT _found)
            message(WARNING "openssl_runtime.cmake: ${_name}, which ${_file} loads, is in none of "
                "${SEARCH_DIRS}, the directories of the OpenSSL Fast-DDS was built against, so it is not "
                "placed next to Fast-DDS, and the loader will take whichever ${_name} it finds elsewhere")
            continue()
        endif()
        get_filename_component(_found_dir "${_found}" DIRECTORY)
        get_filename_component(_found_dir "${_found_dir}" REALPATH)
        if(_found_dir IN_LIST _system_dirs)
            continue()
        endif()
        # The file itself under the name it is loaded by, which on Linux can be a link to a file
        # named for the full version: cmake -E copy copies what a link points at
        execute_process(COMMAND "${CMAKE_COMMAND}" -E copy "${_found}" "${DESTINATION}/${_name}"
            RESULT_VARIABLE _result)
        if(NOT _result EQUAL 0)
            message(FATAL_ERROR "openssl_runtime.cmake: failed to copy ${_found} to ${DESTINATION}")
        endif()
        message(STATUS "Placed ${_name} from ${_found_dir}")
        list(APPEND _queue "${DESTINATION}/${_name}")
    endforeach()
endwhile()
