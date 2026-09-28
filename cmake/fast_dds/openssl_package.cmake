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

# The FindOpenSSL given to the Fast-DDS that provizio_dds builds when provizio_dds found OpenSSL as a
# package (OpenSSL_CONFIG set -- Conan's, say): see FindOpenSSL.cmake.in for what it does and why.
#
#   provizio_dds_write_openssl_find_module(<directory> <package_config>)
#
# writes <directory>/FindOpenSSL.cmake from what the calling scope has: the package found
# (OpenSSL_CONFIG, its version, its include directory as below), the package search settings it
# was found with, and <package_config>, the configuration (build type) of the package's imported
# targets that Fast-DDS is to take, empty for no mapping. configure_file() rewrites it only when
# that changes, which is when Fast-DDS has to configure again. Package locations, lookup results and
# values given on the command line are among it, so one first cached after this call -- by a package
# or a program that provizio_dds, or a project around it, looks up later on -- changes it on the next
# configure of a new build tree, which configures Fast-DDS once more; from then on it settles, until
# one of them changes.
#
# Called by the top-level CMakeLists.txt, and by the fast_dds_openssl_package test, with stand-in
# packages, to generate the module exactly as the build does.
#
#   PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY
#
# is the argument (-C and a script) to give every configure of the Fast-DDS build first, with such a
# module or without one: it puts back what a configure that stopped inside the module's lookup left
# in that build's cache, before anything reads it (see openssl_lookup_journal.cmake).
#
#   provizio_dds_openssl_include_dir(<out>)
#
# sets <out> to the directory of the headers of the OpenSSL found, however it was found, with /
# separators and without a trailing one: the first of OPENSSL_INCLUDE_DIR, which FindOpenSSL always
# sets, and where that is not set -- a package's configuration need not set it, and one may define
# its imported targets and nothing else -- the first include directory OpenSSL::SSL or
# OpenSSL::Crypto names as an existing absolute path. One a target names through a generator
# expression (a package naming each configuration's apart, as Conan's does) can be neither read nor
# checked before generation, so it is passed over. Empty when there is no such directory. Nothing
# fails for that: the Fast-DDS build is handed such a package's own configuration, whose targets
# carry its headers. The search for the OpenSSL runtime to place next to Fast-DDS loses the
# directories beside the headers, keeping those of the library files the package names, and warns
# at build time where it is left with none; the Windows Python steps copying the OpenSSL DLLs from
# beside the headers are skipped, those placed next to a Fast-DDS built here reaching the package
# all the same.
#
#   provizio_dds_openssl_runtime_dirs(<out> <build_type> [<mapped_config>])
#
# sets <out> to the directories where the shared libraries of the OpenSSL found can be, for those
# Fast-DDS loads to be placed next to it (the openssl_runtime step): the directories of the library
# files found, with the bin directory next to each, where an import library's DLL is, then those of
# the root, the parent of the include directory above. The files are, first, OpenSSL's own:
# FindOpenSSL's results (OPENSSL_SSL_LIBRARY / OPENSSL_CRYPTO_LIBRARY, keyworded debug / optimized
# where it found both) and the locations of OpenSSL::SSL and OpenSSL::Crypto. Then those reached
# from the two through INTERFACE_LINK_LIBRARIES -- the targets named there, inside generator
# expressions too, and the files named there by absolute path, transitively -- as a package may make
# the two interface targets that name no file, wrapping others that do (Conan's are), or linking
# the files themselves (FindPkgConfig's IMPORTED_TARGET): an item of its own, or inside a generator
# expression, whose condition cannot be evaluated before generation -- one of a plain configuration
# condition ($<$<CONFIG:Release>:file>) naming the configuration Fast-DDS links is taken with that
# configuration's files, and one of any other after every other file, so that it cannot come ahead
# of the files of the configuration Fast-DDS links. Of those, only files named as OpenSSL's libraries are taken, as the same links reach its
# dependencies (zlib, say), whose directories can hold another OpenSSL, a system's own; and they
# come after OpenSSL's own, so that no such directory can come first. Within each of the two, the
# files of the configuration Fast-DDS links come first, as a library of one name can be in both:
# with Fast-DDS built as <build_type>, a target's MAP_IMPORTED_CONFIG_<build_type> -- the first of
# the configurations it names that the target has a file of, as CMake takes it, an empty one naming
# the file of no configuration -- where an OpenSSL package (OpenSSL_CONFIG) maps that build type (the
# map of a target FindOpenSSL found reaches no Fast-DDS), or <mapped_config> where given (the
# configuration the FindOpenSSL given to Fast-DDS maps it onto), or <build_type> itself; all of it
# whatever its case, as CMake takes a configuration's name. A target's files are read in the order CMake takes them in: its
# IMPORTED_LOCATION / IMPORTED_IMPLIB of no configuration, then those of that configuration, listed
# in its IMPORTED_CONFIGURATIONS or not, then those of the configurations it lists there (in upper
# case, as CMake reads them) or, where it lists none (Conan's), of CMake's four. The condition of a
# generator expression -- a configuration, mostly -- cannot be evaluated before generation, so every
# configuration's targets are taken.

# Where the template is, as a function knows only its caller's directory before CMake 3.17
set(_PROVIZIO_DDS_OPENSSL_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}")

# The argument to give every configure of the Fast-DDS build first, whatever OpenSSL it is given,
# ahead of all the others (see openssl_lookup_recovery.cmake)
set(PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY "-C${CMAKE_CURRENT_LIST_DIR}/openssl_lookup_recovery.cmake")

include("${CMAKE_CURRENT_LIST_DIR}/openssl_lookup_journal.cmake")

function(provizio_dds_openssl_include_dir out)
    set(_dir "")
    if(OPENSSL_INCLUDE_DIR)
        list(GET OPENSSL_INCLUDE_DIR 0 _dir)
    else()
        foreach(_target IN ITEMS OpenSSL::SSL OpenSSL::Crypto)
            if(TARGET ${_target})
                get_target_property(_target_dirs ${_target} INTERFACE_INCLUDE_DIRECTORIES)
                if(_target_dirs)
                    # Each generator expression whole, before the value is taken as a list: a ;
                    # inside one would cut it into pieces, and one between two others is a plain
                    # path
                    string(GENEX_STRIP "${_target_dirs}" _target_dirs)
                    foreach(_target_dir IN LISTS _target_dirs)
                        if(IS_ABSOLUTE "${_target_dir}" AND IS_DIRECTORY "${_target_dir}")
                            set(_dir "${_target_dir}")
                            break()
                        endif()
                    endforeach()
                endif()
                if(NOT _dir STREQUAL "")
                    break()
                endif()
            endif()
        endforeach()
    endif()
    # Without a trailing separator, which would make the include directory its own parent. Only
    # Windows separators are converted: file(TO_CMAKE_PATH) would also split the path at a : on
    # other platforms, where that is a character a directory name may have, as is a backslash.
    if(WIN32)
        string(REPLACE "\\" "/" _dir "${_dir}")
    endif()
    string(REGEX REPLACE "/+$" "" _dir "${_dir}")
    set(${out} "${_dir}" PARENT_SCOPE)
endfunction()

function(provizio_dds_openssl_runtime_dirs out build_type)
    string(TOUPPER "${build_type}" _build_type_upper)
    # The configuration of a target's files Fast-DDS links where the target maps none
    set(_default_config "${_build_type_upper}")
    if(ARGC GREATER 2 AND NOT "${ARGV2}" STREQUAL "")
        string(TOUPPER "${ARGV2}" _default_config)
    endif()
    # The files in four lists: OpenSSL's own and those reached through links, each of <build_type>
    # first and of any other configuration after
    set(_own_first)
    set(_own)
    set(_linked_first)
    set(_linked)
    set(_linked_later)
    set(_keyword)
    foreach(_item IN LISTS OPENSSL_SSL_LIBRARY OPENSSL_CRYPTO_LIBRARY)
        if(_item MATCHES "^(debug|optimized|general)$")
            set(_keyword "${_item}")
            continue()
        endif()
        if(IS_ABSOLUTE "${_item}" AND EXISTS "${_item}")
            # A configuration's name taken whatever its case, as CMake takes it
            if((_keyword STREQUAL "debug" AND _default_config STREQUAL "DEBUG") OR
                    (_keyword STREQUAL "optimized" AND NOT _default_config STREQUAL "DEBUG"))
                list(APPEND _own_first "${_item}")
            else()
                list(APPEND _own "${_item}")
            endif()
        endif()
        set(_keyword)
    endforeach()

    set(_roots OpenSSL::SSL OpenSSL::Crypto)
    set(_targets)
    set(_pending ${_roots})
    list(LENGTH _pending _pending_count)
    while(_pending_count GREATER 0)
        list(GET _pending 0 _target)
        list(REMOVE_AT _pending 0)
        list(LENGTH _pending _pending_count)
        if(NOT TARGET "${_target}" OR _target IN_LIST _targets)
            continue()
        endif()
        list(APPEND _targets "${_target}")
        get_target_property(_links "${_target}" INTERFACE_LINK_LIBRARIES)
        if(NOT _links)
            continue()
        endif()
        foreach(_link IN LISTS _links)
            if(IS_ABSOLUTE "${_link}" AND EXISTS "${_link}" AND NOT IS_DIRECTORY "${_link}")
                # A file named by its path
                list(APPEND _linked "${_link}")
                continue()
            endif()
            if(_link MATCHES "\\$<")
                # Files named by their paths inside a generator expression: with the configuration
                # Fast-DDS links where the expression is a plain condition naming it, after every
                # other file otherwise (see the top of this file)
                set(_into _linked_later)
                if(_link MATCHES "^\\$<\\$<CONFIG:([^<>]*)>:[^<>]*>$")
                    string(TOUPPER "${CMAKE_MATCH_1}" _conditions)
                    string(REPLACE "," ";" _conditions "${_conditions}")
                    if(_build_type_upper IN_LIST _conditions OR _default_config IN_LIST _conditions)
                        set(_into _linked_first)
                    endif()
                endif()
                # The expression taken apart at its own syntax -- $<NAME:, >: and >, the , between
                # operands -- leaving the operands, among them the paths
                string(REGEX REPLACE "\\$<[A-Za-z_]+:" "|" _operands "${_link}")
                string(REPLACE ">:" "|" _operands "${_operands}")
                string(REPLACE ">" "|" _operands "${_operands}")
                string(REPLACE "," "|" _operands "${_operands}")
                string(REPLACE "$<" "|" _operands "${_operands}")
                string(REPLACE "|" ";" _operands "${_operands}")
                foreach(_path IN LISTS _operands)
                    if(IS_ABSOLUTE "${_path}" AND EXISTS "${_path}" AND NOT IS_DIRECTORY "${_path}")
                        list(APPEND ${_into} "${_path}")
                    endif()
                endforeach()
            endif()
            # The names of the targets it links: what is left once any path is taken out, whose
            # components would otherwise pass for names
            string(REGEX REPLACE "[^<>,:$]*[/\\][^<>,$]*" "" _link "${_link}")
            string(REGEX MATCHALL "[A-Za-z0-9_.+-]+(::[A-Za-z0-9_.+-]+)*" _names "${_link}")
            list(APPEND _pending ${_names})
        endforeach()
        list(LENGTH _pending _pending_count)
    endwhile()

    foreach(_target IN LISTS _targets)
        get_target_property(_type "${_target}" TYPE)
        # Read only past this: before CMake 3.19 an interface library's other properties cannot be
        if(_type STREQUAL "INTERFACE_LIBRARY")
            continue()
        endif()
        get_target_property(_configs "${_target}" IMPORTED_CONFIGURATIONS)
        if(_configs)
            string(TOUPPER "${_configs}" _configs)
        else()
            set(_configs DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        endif()
        # The configuration Fast-DDS links of it: of an OpenSSL package -- the map of a target found
        # by FindOpenSSL reaches no Fast-DDS, which takes the files of its own build type -- the first
        # its MAP_IMPORTED_CONFIG_<build type> names that it has a file of, where it maps that build
        # type, as CMake takes it: an empty one is the file of no configuration (<none>)
        set(_linked_config "${_default_config}")
        get_property(_mapped_set TARGET "${_target}" PROPERTY MAP_IMPORTED_CONFIG_${_build_type_upper} SET)
        if(OpenSSL_CONFIG AND _mapped_set)
            get_target_property(_mapped "${_target}" MAP_IMPORTED_CONFIG_${_build_type_upper})
            if("${_mapped}" STREQUAL "")
                # Not a list of one empty element, which iterates as none
                set(_mapped ";")
            endif()
            string(TOUPPER "${_mapped}" _mapped)
            foreach(_candidate IN LISTS _mapped)
                if(_candidate STREQUAL "")
                    set(_suffix "")
                else()
                    set(_suffix "_${_candidate}")
                endif()
                get_target_property(_location "${_target}" IMPORTED_LOCATION${_suffix})
                get_target_property(_implib "${_target}" IMPORTED_IMPLIB${_suffix})
                if(_location OR _implib)
                    set(_linked_config "${_candidate}")
                    if(_candidate STREQUAL "")
                        set(_linked_config "<none>")
                    endif()
                    break()
                endif()
            endforeach()
        endif()
        if(NOT _linked_config STREQUAL "<none>")
            set(_configs "${_linked_config}" ${_configs})
        endif()
        list(REMOVE_DUPLICATES _configs)
        if(_target IN_LIST _roots)
            set(_to_first _own_first)
            set(_to _own)
        else()
            set(_to_first _linked_first)
            set(_to _linked)
        endif()
        foreach(_property IN ITEMS IMPORTED_LOCATION IMPORTED_IMPLIB)
            get_target_property(_item "${_target}" ${_property})
            if(_item AND IS_ABSOLUTE "${_item}" AND EXISTS "${_item}")
                if(_linked_config STREQUAL "<none>")
                    list(APPEND ${_to_first} "${_item}")
                else()
                    list(APPEND ${_to} "${_item}")
                endif()
            endif()
            foreach(_config IN LISTS _configs)
                get_target_property(_item "${_target}" ${_property}_${_config})
                if(_item AND IS_ABSOLUTE "${_item}" AND EXISTS "${_item}")
                    if(_config STREQUAL _linked_config)
                        list(APPEND ${_to_first} "${_item}")
                    else()
                        list(APPEND ${_to} "${_item}")
                    endif()
                endif()
            endforeach()
        endforeach()
    endforeach()

    # Of the files reached through links, OpenSSL's libraries only
    foreach(_list IN ITEMS _linked_first _linked _linked_later)
        set(_kept)
        foreach(_item IN LISTS ${_list})
            get_filename_component(_name "${_item}" NAME)
            string(TOLOWER "${_name}" _name)
            if(_name MATCHES "^(lib)?(ssl|crypto)([-._0-9]|$)")
                list(APPEND _kept "${_item}")
            endif()
        endforeach()
        set(${_list} ${_kept})
    endforeach()

    set(_dirs)
    foreach(_file IN LISTS _own_first _own _linked_first _linked _linked_later)
        get_filename_component(_dir "${_file}" DIRECTORY)
        get_filename_component(_bin_dir "${_dir}/../bin" ABSOLUTE)
        list(APPEND _dirs "${_dir}")
        if(IS_DIRECTORY "${_bin_dir}")
            list(APPEND _dirs "${_bin_dir}")
        endif()
    endforeach()
    provizio_dds_openssl_include_dir(_include_dir)
    if(NOT _include_dir STREQUAL "" AND IS_DIRECTORY "${_include_dir}")
        get_filename_component(_root "${_include_dir}" DIRECTORY)
        foreach(_subdir IN ITEMS bin lib lib64)
            if(IS_DIRECTORY "${_root}/${_subdir}")
                list(APPEND _dirs "${_root}/${_subdir}")
            endif()
        endforeach()
    endif()
    if(_dirs)
        list(REMOVE_DUPLICATES _dirs)
    endif()
    set(${out} "${_dirs}" PARENT_SCOPE)
endfunction()

# The = of a bracket argument that the values of all the variables named after <out> can stand in
# whole, in <out>: [==[ ]==] but where one holds ]==] or ends in ]==, which would close it early.
# Its own variables are named so that no variable it is given the name of can be one of them.
function(_provizio_dds_bracket_level out)
    set(_provizio_dds_bracket_level "==")
    set(_provizio_dds_bracket_closed TRUE)
    while(_provizio_dds_bracket_closed)
        set(_provizio_dds_bracket_closed FALSE)
        foreach(_provizio_dds_bracket_name IN LISTS ARGN)
            string(FIND "${${_provizio_dds_bracket_name}}]" "]${_provizio_dds_bracket_level}]" _provizio_dds_bracket_at)
            if(NOT _provizio_dds_bracket_at EQUAL -1)
                set(_provizio_dds_bracket_closed TRUE)
            endif()
        endforeach()
        if(_provizio_dds_bracket_closed)
            string(APPEND _provizio_dds_bracket_level "=")
        endif()
    endwhile()
    set(${out} "${_provizio_dds_bracket_level}" PARENT_SCOPE)
endfunction()

function(provizio_dds_write_openssl_find_module directory package_config)
    get_filename_component(PROVIZIO_DDS_OPENSSL_CONFIG_DIR "${OpenSSL_CONFIG}" DIRECTORY)
    set(PROVIZIO_DDS_OPENSSL_JOURNAL "${_PROVIZIO_DDS_OPENSSL_PACKAGE_DIR}/openssl_lookup_journal.cmake")
    # For the header only
    set(PROVIZIO_DDS_OPENSSL_CONFIG_FILE "${OpenSSL_CONFIG}")
    set(PROVIZIO_DDS_OPENSSL_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
    set(PROVIZIO_DDS_OPENSSL_MODULE_PATH "${CMAKE_MODULE_PATH}")
    set(PROVIZIO_DDS_OPENSSL_PACKAGE_CONFIG "${package_config}")
    # For the header only
    provizio_dds_openssl_include_dir(PROVIZIO_DDS_OPENSSL_INCLUDE_DIR)
    if(PROVIZIO_DDS_OPENSSL_INCLUDE_DIR STREQUAL "")
        set(PROVIZIO_DDS_OPENSSL_INCLUDE_DIR "(none its configuration names as a plain directory)")
    endif()
    # For the header only: not every package's configuration sets the upper-case one
    if(DEFINED OPENSSL_VERSION)
        set(PROVIZIO_DDS_OPENSSL_VERSION "${OPENSSL_VERSION}")
    else()
        set(PROVIZIO_DDS_OPENSSL_VERSION "${OpenSSL_VERSION}")
    endif()
    # ...where each is named in a comment, which a line break would end
    foreach(_name IN ITEMS PROVIZIO_DDS_OPENSSL_CONFIG_FILE PROVIZIO_DDS_OPENSSL_INCLUDE_DIR PROVIZIO_DDS_OPENSSL_VERSION)
        string(REGEX REPLACE "\r?\n|\r" "\n#   " ${_name} "${${_name}}")
    endforeach()

    # Every package location this configure was given or found, as a package's own lookups of its
    # dependencies may take them from: a <Package>_DIR that holds that package's configuration, and
    # a <Package>_ROOT -- cache entries, which find_package records and either can be given on the
    # command line as, and ordinary variables as well, as a toolchain or a project around
    # provizio_dds may set either without the cache, which the Fast-DDS build gets neither of; an
    # ordinary variable's value comes first, as it does for find_package. Each is set as a variable
    # of the lookup there (PROVIZIO_DDS_OPENSSL_LOCATIONS).
    #
    # And every result of a lookup in the cache -- an entry of type FILEPATH or PATH, as find_library,
    # find_path, find_file and find_program record theirs (a ZLIB_INCLUDE_DIR), a <Package>_DIR, and a
    # value given on the command line that one of them has read -- found, NOTFOUND or empty -- and
    # every value given on the command line that nothing has declared (UNINITIALIZED), as a hint a
    # find module reads without a lookup of its own is (a ZLIB_LIBRARY), empty ones too: written into the Fast-DDS build's cache
    # for its lookup, and taken out again or put back as that build had it after it
    # (PROVIZIO_DDS_OPENSSL_RESULTS), so that a lookup of the same thing there takes provizio_dds's
    # result, or searches again where provizio_dds found nothing, rather than take what that build
    # cached before. Into the cache rather than as a variable, as a lookup may unset() the cache
    # entry of a name and search again, as for each of several files, which a variable of the name
    # would outlive and answer every search with. A name no lookup there reads changes nothing.
    #
    # CMake's own are no package's, nor are names starting with _, a project's own (provizio_dds's
    # _OPENSSL_ROOT). Each is written on its own rather than as a pair in one list -- a value can be a
    # list itself (a <Package>_ROOT of two directories), and its ; would then split it across the
    # pairs -- and as bracket arguments of a level no name or value can close early.
    set(PROVIZIO_DDS_OPENSSL_LOCATIONS "")
    set(PROVIZIO_DDS_OPENSSL_RESULTS "")
    _provizio_dds_openssl_get_names(_entries CACHE_VARIABLES)
    _provizio_dds_openssl_get_names(_variables VARIABLES)
    list(APPEND _entries ${_variables})
    list(REMOVE_DUPLICATES _entries)
    list(SORT _entries)
    foreach(_entry IN LISTS _entries)
        if(_entry MATCHES "^(CMAKE_|_)")
            continue()
        endif()
        set(_value "${${_entry}}")
        get_property(_type CACHE "${_entry}" PROPERTY TYPE)
        _provizio_dds_bracket_level(_level _entry _value)
        if(_type MATCHES "^(FILEPATH|PATH|UNINITIALIZED)$")
            string(APPEND PROVIZIO_DDS_OPENSSL_RESULTS
                "    _provizio_dds_openssl_seed([${_level}[${_entry}]${_level}] ${_type} [${_level}[${_value}]${_level}])\n")
            continue()
        elseif(_entry MATCHES "^(.+)_DIR$")
            set(_package "${CMAKE_MATCH_1}")
            string(TOLOWER "${_package}" _package_lower)
            if(NOT IS_DIRECTORY "${_value}"
                    OR NOT (EXISTS "${_value}/${_package}Config.cmake" OR EXISTS "${_value}/${_package_lower}-config.cmake"))
                continue()
            endif()
        elseif(NOT _entry MATCHES "_ROOT$" OR "${_value}" STREQUAL "")
            continue()
        endif()
        string(APPEND PROVIZIO_DDS_OPENSSL_LOCATIONS
            "    set([${_level}[${_entry}]${_level}] [${_level}[${_value}]${_level}])\n"
            "    list(APPEND _provizio_dds_openssl_names [${_level}[${_entry}]${_level}])\n")
    endforeach()

    # And one level for the template's own bracket arguments
    set(_prefer_config "${CMAKE_FIND_PACKAGE_PREFER_CONFIG}")
    _provizio_dds_bracket_level(PROVIZIO_DDS_OPENSSL_BRACKET PROVIZIO_DDS_OPENSSL_CONFIG_DIR PROVIZIO_DDS_OPENSSL_JOURNAL
        PROVIZIO_DDS_OPENSSL_PREFIX_PATH PROVIZIO_DDS_OPENSSL_MODULE_PATH PROVIZIO_DDS_OPENSSL_PACKAGE_CONFIG
        _prefer_config)

    configure_file("${_PROVIZIO_DDS_OPENSSL_PACKAGE_DIR}/FindOpenSSL.cmake.in" "${directory}/FindOpenSSL.cmake" @ONLY)
endfunction()
