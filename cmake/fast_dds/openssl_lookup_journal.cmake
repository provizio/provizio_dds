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

# The journal the FindOpenSSL written for an OpenSSL package (FindOpenSSL.cmake.in) keeps in the cache
# of the Fast-DDS build around its lookup, and what puts that cache back from it. Included by that
# FindOpenSSL, by openssl_lookup_recovery.cmake, and by openssl_package.cmake, for
# _provizio_dds_openssl_get_names.
#
# For its duration, the lookup has provizio_dds's lookup results and command-line values written into
# that cache, over the build's own entries of the same names, and puts them back after it. A
# configure that stops inside it -- a dependency missing, say -- leaves the cache as it stands then:
# provizio_dds's values in it, and whatever the lookup cached before it stopped. So first the names
# the cache holds (_PROVIZIO_DDS_OPENSSL_BEFORE), then each entry of its own that a value is written
# over (_PROVIZIO_DDS_OPENSSL_SEEDED, with its properties in _PROVIZIO_DDS_OPENSSL_SEEDED_<property>_<name>),
# are written into the cache too. A configure that finds them there puts the cache back from them
# before anything else reads it: openssl_lookup_recovery.cmake, given first to every configure of the
# Fast-DDS build that provizio_dds runs, ahead of its command line, whatever OpenSSL that configure
# is given. One it does not run -- by hand in that build, or by that build regenerating itself --
# has the FindOpenSSL put the cache back before its lookup instead: later, after code of Fast-DDS's
# own and that configure's command line, which it may undo.

# For IN_LIST, wherever this is read: before any project's own policies, too
cmake_policy(VERSION 3.15)

# The names <property> (VARIABLES or CACHE_VARIABLES) lists, in <out>, but any holding a [, a ] or a
# \, as no package location's or lookup result's does: in a list, one such name would join every
# name after it into one
macro(_provizio_dds_openssl_get_names out property)
    get_cmake_property(${out} ${property})
    foreach(_provizio_dds_openssl_character IN ITEMS "[" "]" "\\")
        string(REPLACE "${_provizio_dds_openssl_character}" "<unlisted>" ${out} "${${out}}")
    endforeach()
    list(FILTER ${out} EXCLUDE REGEX "<unlisted>")
endmacro()

# Begins the journal, before anything is written into the cache for the lookup: the names the cache
# holds, the entries of every other of which are taken out again when it ends
function(_provizio_dds_openssl_journal_begin)
    _provizio_dds_openssl_get_names(_names CACHE_VARIABLES)
    set(_PROVIZIO_DDS_OPENSSL_BEFORE "${_names}" CACHE INTERNAL "")
endfunction()

# Writes <name> into the cache for the lookup, as provizio_dds has it: of <type> where the cache has
# no entry of the name, which goes again when the journal ends; of the entry's own type where it has
# one, whose properties -- all but CMake's own MODIFIED -- the journal keeps first, as the lookup may
# change or remove any of them
function(_provizio_dds_openssl_seed name type value)
    if(DEFINED CACHE{${name}})
        set(_seeded "$CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED}")
        list(APPEND _seeded "${name}")
        set(_PROVIZIO_DDS_OPENSSL_SEEDED "${_seeded}" CACHE INTERNAL "")
        foreach(_property IN ITEMS VALUE TYPE HELPSTRING ADVANCED STRINGS)
            get_property(_kept CACHE "${name}" PROPERTY ${_property})
            set("_PROVIZIO_DDS_OPENSSL_SEEDED_${_property}_${name}" "${_kept}" CACHE INTERNAL "")
        endforeach()
        set_property(CACHE "${name}" PROPERTY VALUE "${value}")
    else()
        set("${name}" "${value}" CACHE STRING "" FORCE)
        set_property(CACHE "${name}" PROPERTY TYPE ${type})
    endif()
endfunction()

# Ends the journal, where one was begun: every entry it keeps the properties of made again with
# them, and every entry the cache did not hold when it began -- one written for the lookup, one the
# lookup cached -- removed, and the journal's own entries, whatever of them there are, however they
# came to be there. After the lookup, and before anything else in a configure that finds the journal
# of one that stopped inside it.
function(_provizio_dds_openssl_journal_end)
    if(NOT DEFINED CACHE{_PROVIZIO_DDS_OPENSSL_BEFORE} AND NOT DEFINED CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED})
        return()
    endif()
    set(_seeded "$CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED}")
    foreach(_name IN LISTS _seeded)
        if(NOT DEFINED CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED_TYPE_${_name}})
            # Never journalled whole
            continue()
        endif()
        # Made afresh, as an entry's being advanced can be set but not taken away again
        unset("${_name}" CACHE)
        set("${_name}" "$CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED_VALUE_${_name}}"
            CACHE STRING "$CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED_HELPSTRING_${_name}}" FORCE)
        set_property(CACHE "${_name}" PROPERTY TYPE "$CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED_TYPE_${_name}}")
        if("$CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED_ADVANCED_${_name}}")
            set_property(CACHE "${_name}" PROPERTY ADVANCED 1)
        endif()
        if(NOT "$CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED_STRINGS_${_name}}" STREQUAL "")
            set_property(CACHE "${_name}" PROPERTY STRINGS "$CACHE{_PROVIZIO_DDS_OPENSSL_SEEDED_STRINGS_${_name}}")
        endif()
    endforeach()
    set(_sweep FALSE)
    if(DEFINED CACHE{_PROVIZIO_DDS_OPENSSL_BEFORE})
        set(_sweep TRUE)
        set(_before "$CACHE{_PROVIZIO_DDS_OPENSSL_BEFORE}")
    endif()
    _provizio_dds_openssl_get_names(_names CACHE_VARIABLES)
    foreach(_name IN LISTS _names)
        if(_name MATCHES "^_PROVIZIO_DDS_OPENSSL_" OR (_sweep AND NOT _name IN_LIST _before))
            unset("${_name}" CACHE)
        endif()
    endforeach()
endfunction()
