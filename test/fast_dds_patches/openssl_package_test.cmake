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

# Coverage for the FindOpenSSL given to Fast-DDS when provizio_dds finds OpenSSL as a package
# (cmake/fast_dds/openssl_package.cmake, FindOpenSSL.cmake.in): the package's own lookups of its
# dependencies, run inside Fast-DDS's configure, must find what provizio_dds's did -- through the
# package locations and module path provizio_dds was given, not only its CMAKE_PREFIX_PATH -- and must
# not be decided by what an earlier configure of that build cached. A package need not set any of
# FindOpenSSL's variables, and one that does not must be found all the same, its headers named as
# its targets name them (provizio_dds_openssl_include_dir), and its runtime looked for where the
# targets its own link name its library files (provizio_dds_openssl_runtime_dirs).
#
# openssl_package/provizio stands in for provizio_dds: it finds a stand-in OpenSSL package as a
# package-manager toolchain has it found (CMAKE_FIND_PACKAGE_PREFER_CONFIG) and writes the module with
# provizio_dds's own function. openssl_package/fast_dds stands in for the Fast-DDS build: it looks
# OpenSSL up through that module and says which stand-in each dependency was. The stand-ins are named
# for this test alone, so that no package of the host's, and no find module of CMake's, can answer
# for them.
#
#   cmake -DSOURCE_DIR=<repository> -DWORK_DIR=<scratch dir> -DGENERATOR=<generator>
#         -P openssl_package_test.cmake

cmake_minimum_required(VERSION 3.15)

foreach(_var IN ITEMS SOURCE_DIR WORK_DIR GENERATOR)
    if(NOT ${_var})
        message(FATAL_ERROR "openssl_package_test.cmake: ${_var} is required")
    endif()
endforeach()

file(REMOVE_RECURSE "${WORK_DIR}")
set(_packages "${WORK_DIR}/packages")

# For PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY, the argument the Fast-DDS build is configured with first
include("${SOURCE_DIR}/cmake/fast_dds/openssl_package.cmake")

# An OpenSSL package whose configuration looks up <dependency> (with <how>, "" or MODULE) and links it.
# Its headers are named as <headers>: "variable" for FindOpenSSL's OPENSSL_INCLUDE_DIR, which such a
# configuration need not set, and otherwise by its targets alone, with none of FindOpenSSL's
# upper-case variables, OPENSSL_VERSION included. Either way the targets name a directory of their
# own first, for one configuration only, through a generator expression with a ; inside it, whose
# middle piece taken on its own would be an existing directory; "targets" then names a relative
# directory, which exists from wherever the configure runs, and the include directory plainly, with
# a trailing separator, and leaves OPENSSL_INCLUDE_DIR undefined, while "generator_expression" names
# nothing more and sets that variable to nothing. Each fails its lookup, once the dependency is
# found, where PROVIZIO_TEST_FAIL_LOOKUP is on, as a configure that stops inside it does.
function(_openssl_package name dependency how headers)
    set(_variables "set(OPENSSL_VERSION 3.9.9)
set(OPENSSL_INCLUDE_DIR \"\${CMAKE_CURRENT_LIST_DIR}/../../../include\")")
    set(_include_dirs "")
    if(NOT headers STREQUAL "variable")
        file(MAKE_DIRECTORY "${_packages}/${name}/include" "${_packages}/${name}/debug/include")
        set(_include_dirs "\$<\$<CONFIG:Debug>:\${CMAKE_CURRENT_LIST_DIR}/../../../debug")
        string(APPEND _include_dirs ";\${CMAKE_CURRENT_LIST_DIR}/../../../debug/include")
        string(APPEND _include_dirs ";\${CMAKE_CURRENT_LIST_DIR}/../../../debug>")
        if(headers STREQUAL "targets")
            set(_variables "")
            string(APPEND _include_dirs ";.;\${CMAKE_CURRENT_LIST_DIR}/../../../include/")
        else()
            set(_variables "set(OPENSSL_INCLUDE_DIR \"\")")
        endif()
    endif()
    file(WRITE "${_packages}/${name}/lib/cmake/OpenSSL/OpenSSLConfig.cmake" "
include(CMakeFindDependencyMacro)
find_dependency(${dependency} ${how})
if(PROVIZIO_TEST_FAIL_LOOKUP)
    message(FATAL_ERROR \"openssl_package_test: the lookup fails, as asked\")
endif()
if(NOT TARGET OpenSSL::Crypto)
    add_library(OpenSSL::Crypto INTERFACE IMPORTED)
    set_target_properties(OpenSSL::Crypto PROPERTIES INTERFACE_LINK_LIBRARIES ${dependency}::${dependency}
        INTERFACE_INCLUDE_DIRECTORIES \"${_include_dirs}\")
    add_library(OpenSSL::SSL INTERFACE IMPORTED)
    set_target_properties(OpenSSL::SSL PROPERTIES INTERFACE_LINK_LIBRARIES OpenSSL::Crypto)
endif()
set(OpenSSL_VERSION 3.9.9)
${_variables}
")
endfunction()
_openssl_package(openssl_z ProvizioTestZ "" variable)
_openssl_package(openssl_foo ProvizioTestFoo MODULE variable)
_openssl_package(openssl_bar ProvizioTestBar MODULE variable)
_openssl_package(openssl_parts ProvizioTestParts MODULE variable)
_openssl_package(openssl_targets ProvizioTestZ "" targets)
_openssl_package(openssl_generator_expression ProvizioTestZ "" generator_expression)
if(CMAKE_HOST_UNIX)
    # A directory name holding a line break, which no Windows one can
    _openssl_package("openssl_line\nbreak" ProvizioTestZ "" variable)
endif()

# And one whose OpenSSL::SSL and OpenSSL::Crypto are interface targets wrapping others that name its
# library files, for one configuration, through a generator expression, listing no
# IMPORTED_CONFIGURATIONS, as Conan's CMakeDeps lists none; the libcrypto one links a dependency
# whose directory holds another OpenSSL, as a system's library directory does
set(_wrapped "${_packages}/openssl_wrapped")
set(_system "${_packages}/system")
file(MAKE_DIRECTORY "${_wrapped}/bin")
foreach(_file IN ITEMS "${_wrapped}/lib/libssl.so.3" "${_wrapped}/lib/libcrypto.so.3" "${_system}/lib/libz.so.1"
        "${_system}/lib/libssl.so.3")
    file(WRITE "${_file}" "")
endforeach()
file(WRITE "${_wrapped}/lib/cmake/OpenSSL/OpenSSLConfig.cmake" "
if(NOT TARGET OpenSSL::Crypto)
    add_library(ProvizioTestOpenSSL::z UNKNOWN IMPORTED)
    set_target_properties(ProvizioTestOpenSSL::z PROPERTIES IMPORTED_LOCATION [==[${_system}/lib/libz.so.1]==])
    add_library(ProvizioTestOpenSSL::crypto UNKNOWN IMPORTED)
    set_target_properties(ProvizioTestOpenSSL::crypto PROPERTIES
        IMPORTED_LOCATION_RELEASE [==[${_wrapped}/lib/libcrypto.so.3]==] INTERFACE_LINK_LIBRARIES ProvizioTestOpenSSL::z)
    add_library(ProvizioTestOpenSSL::ssl UNKNOWN IMPORTED)
    set_target_properties(ProvizioTestOpenSSL::ssl PROPERTIES
        IMPORTED_LOCATION_RELEASE [==[${_wrapped}/lib/libssl.so.3]==])
    add_library(OpenSSL::Crypto INTERFACE IMPORTED)
    set_target_properties(OpenSSL::Crypto PROPERTIES
        INTERFACE_LINK_LIBRARIES \"\$<\$<CONFIG:Release>:ProvizioTestOpenSSL::crypto>\")
    add_library(OpenSSL::SSL INTERFACE IMPORTED)
    set_target_properties(OpenSSL::SSL PROPERTIES
        INTERFACE_LINK_LIBRARIES \"OpenSSL::Crypto;\$<\$<CONFIG:Release>:ProvizioTestOpenSSL::ssl>\")
endif()
set(OpenSSL_VERSION 3.9.9)
")

# And one whose OpenSSL::Crypto names its file itself, as one of no configuration, and links a target
# naming another libssl for the configuration Fast-DDS takes, listing that configuration in mixed
# case; one naming a libcrypto for that configuration though listing only another; one naming a
# libssl of no configuration and one of another configuration; and a dependency by its path inside
# a generator expression, written with / and with \, whose directory's name is a target's too,
# naming the system's libssl. Its OpenSSL::SSL, an interface target, links its file by its path, as
# FindPkgConfig's IMPORTED_TARGET does.
set(_ranked "${_packages}/openssl_ranked")
set(_other "${_packages}/other")
set(_pkgconf "${_packages}/pkgconf")
set(_unlisted "${_packages}/unlisted")
set(_plain "${_packages}/plain")
set(_debug "${_packages}/debug")
foreach(_file IN ITEMS "${_ranked}/lib/libcrypto.so.3" "${_other}/lib/libssl.so.3" "${_pkgconf}/lib/libssl.so.3"
        "${_unlisted}/lib/libcrypto.so.3" "${_plain}/lib/libssl.so.3" "${_debug}/lib/libssl.so.3")
    file(WRITE "${_file}" "")
endforeach()
file(WRITE "${_ranked}/lib/cmake/OpenSSL/OpenSSLConfig.cmake" "
if(NOT TARGET OpenSSL::Crypto)
    add_library(lib UNKNOWN IMPORTED)
    set_target_properties(lib PROPERTIES IMPORTED_LOCATION_RELEASE [==[${_system}/lib/libssl.so.3]==])
    add_library(ProvizioTestOpenSSL::other UNKNOWN IMPORTED)
    set_target_properties(ProvizioTestOpenSSL::other PROPERTIES IMPORTED_CONFIGURATIONS Release
        IMPORTED_LOCATION_RELEASE [==[${_other}/lib/libssl.so.3]==])
    add_library(ProvizioTestOpenSSL::unlisted UNKNOWN IMPORTED)
    set_target_properties(ProvizioTestOpenSSL::unlisted PROPERTIES IMPORTED_CONFIGURATIONS DEBUG
        IMPORTED_LOCATION_RELEASE [==[${_unlisted}/lib/libcrypto.so.3]==])
    add_library(ProvizioTestOpenSSL::ordered UNKNOWN IMPORTED)
    set_target_properties(ProvizioTestOpenSSL::ordered PROPERTIES IMPORTED_LOCATION [==[${_plain}/lib/libssl.so.3]==]
        IMPORTED_LOCATION_DEBUG [==[${_debug}/lib/libssl.so.3]==])
    add_library(OpenSSL::Crypto UNKNOWN IMPORTED)
    set_target_properties(OpenSSL::Crypto PROPERTIES IMPORTED_LOCATION [==[${_ranked}/lib/libcrypto.so.3]==]
        INTERFACE_LINK_LIBRARIES [==[ProvizioTestOpenSSL::other;$<LINK_ONLY:${_system}/lib/libz.so.1>;$<LINK_ONLY:C:\\nowhere\\lib\\z.lib>;ProvizioTestOpenSSL::unlisted;ProvizioTestOpenSSL::ordered]==])
    add_library(OpenSSL::SSL INTERFACE IMPORTED)
    set_target_properties(OpenSSL::SSL PROPERTIES
        INTERFACE_LINK_LIBRARIES [==[OpenSSL::Crypto;${_pkgconf}/lib/libssl.so.3]==])
endif()
set(OpenSSL_VERSION 3.9.9)
")

# And three whose OpenSSL::Crypto names a file of each of three configurations and one of none: one
# mapping Release onto MinSizeRel, of which it has none, then RelWithDebInfo, as a package may map one
# onto another; one mapping it onto the file of no configuration; and one mapping nothing
foreach(_mapping IN ITEMS mapped mapped_none unmapped)
    foreach(_config IN ITEMS release relwithdebinfo debug none)
        file(WRITE "${_packages}/openssl_${_mapping}/${_config}/lib/libcrypto.so.3" "")
    endforeach()
    set(_map "")
    if(_mapping STREQUAL "mapped")
        set(_map "MAP_IMPORTED_CONFIG_RELEASE \"MinSizeRel;RelWithDebInfo\"")
    elseif(_mapping STREQUAL "mapped_none")
        set(_map "MAP_IMPORTED_CONFIG_RELEASE \"\"")
    endif()
    file(WRITE "${_packages}/openssl_${_mapping}/lib/cmake/OpenSSL/OpenSSLConfig.cmake" "
if(NOT TARGET OpenSSL::Crypto)
    add_library(OpenSSL::Crypto UNKNOWN IMPORTED)
    set_target_properties(OpenSSL::Crypto PROPERTIES IMPORTED_CONFIGURATIONS \"RELEASE;RELWITHDEBINFO;DEBUG\"
        IMPORTED_LOCATION_RELEASE [==[${_packages}/openssl_${_mapping}/release/lib/libcrypto.so.3]==]
        IMPORTED_LOCATION_RELWITHDEBINFO [==[${_packages}/openssl_${_mapping}/relwithdebinfo/lib/libcrypto.so.3]==]
        IMPORTED_LOCATION_DEBUG [==[${_packages}/openssl_${_mapping}/debug/lib/libcrypto.so.3]==]
        IMPORTED_LOCATION [==[${_packages}/openssl_${_mapping}/none/lib/libcrypto.so.3]==]
        ${_map})
    add_library(OpenSSL::SSL INTERFACE IMPORTED)
    set_target_properties(OpenSSL::SSL PROPERTIES INTERFACE_LINK_LIBRARIES OpenSSL::Crypto)
endif()
set(OpenSSL_VERSION 3.9.9)
")
endforeach()

# And one whose OpenSSL::Crypto is an interface target naming no file of its own, but linking its
# files by their paths inside generator expressions only: the Debug one's ahead of the Release one's,
# and another inside one of no configuration
set(_genex "${_packages}/openssl_genex_paths")
foreach(_config IN ITEMS release debug link_only)
    file(WRITE "${_genex}/${_config}/lib/libcrypto.so.3" "")
endforeach()
file(WRITE "${_genex}/lib/cmake/OpenSSL/OpenSSLConfig.cmake" "
if(NOT TARGET OpenSSL::Crypto)
    add_library(OpenSSL::Crypto INTERFACE IMPORTED)
    set_target_properties(OpenSSL::Crypto PROPERTIES INTERFACE_LINK_LIBRARIES
        [==[$<$<CONFIG:Debug>:${_genex}/debug/lib/libcrypto.so.3>;$<$<CONFIG:Release>:${_genex}/release/lib/libcrypto.so.3>;$<LINK_ONLY:${_genex}/link_only/lib/libcrypto.so.3>]==])
    add_library(OpenSSL::SSL INTERFACE IMPORTED)
    set_target_properties(OpenSSL::SSL PROPERTIES INTERFACE_LINK_LIBRARIES OpenSSL::Crypto)
endif()
set(OpenSSL_VERSION 3.9.9)
")

# Two copies of a dependency found as a package, told apart by what their targets carry
foreach(_copy IN ITEMS A B)
    file(WRITE "${_packages}/z${_copy}/lib/cmake/ProvizioTestZ/ProvizioTestZConfig.cmake" "
if(NOT TARGET ProvizioTestZ::ProvizioTestZ)
    add_library(ProvizioTestZ::ProvizioTestZ INTERFACE IMPORTED)
    set_target_properties(ProvizioTestZ::ProvizioTestZ PROPERTIES INTERFACE_COMPILE_DEFINITIONS ${_copy})
endif()
")
endforeach()

# And one by a find module of the consumer's that looks for its file under its <Package>_ROOT, which
# find_package has it search first, and caches no <Package>_DIR to find it by, but an entry of its
# own, ProvizioTestBar_CHECKED, as FindPkgConfig caches what it has checked
file(WRITE "${_packages}/bar_root/share/provizio_test_bar.marker" "under_root")
file(WRITE "${WORK_DIR}/modules/FindProvizioTestBar.cmake" "
find_file(ProvizioTestBar_MARKER provizio_test_bar.marker PATH_SUFFIXES share)
set(ProvizioTestBar_FOUND FALSE)
set(ProvizioTestBar_CHECKED TRUE CACHE INTERNAL \"Looked for\")
if(ProvizioTestBar_MARKER)
    set(ProvizioTestBar_FOUND TRUE)
    file(READ \"\${ProvizioTestBar_MARKER}\" _provizio_test_bar_which)
    if(NOT TARGET ProvizioTestBar::ProvizioTestBar)
        add_library(ProvizioTestBar::ProvizioTestBar INTERFACE IMPORTED)
        set_target_properties(ProvizioTestBar::ProvizioTestBar PROPERTIES
            INTERFACE_COMPILE_DEFINITIONS \"\${_provizio_test_bar_which}\")
    endif()
endif()
")
# And one by a find module looking several files up under one name, unsetting it in between, and
# one more it does without where it finds none: "A" and "B" under its root, of which it says "AB"
# (or "AB+extra" with the extra one). Where PROVIZIO_TEST_PARTS_FORGET is on, it removes the entry
# of the extra one from the cache once done with it.
file(WRITE "${_packages}/parts_root/share/part_a.marker" "A")
file(WRITE "${_packages}/parts_root/share/part_b.marker" "B")
file(WRITE "${_packages}/stale_parts/share/part_extra.marker" "")
file(WRITE "${WORK_DIR}/modules/FindProvizioTestParts.cmake" "
set(_provizio_test_parts_which \"\")
foreach(_part IN ITEMS a b)
    unset(ProvizioTestParts_PART CACHE)
    find_file(ProvizioTestParts_PART part_\${_part}.marker PATH_SUFFIXES share)
    if(ProvizioTestParts_PART)
        file(READ \"\${ProvizioTestParts_PART}\" _provizio_test_parts_part)
        string(APPEND _provizio_test_parts_which \"\${_provizio_test_parts_part}\")
    endif()
endforeach()
find_file(ProvizioTestParts_EXTRA part_extra.marker PATH_SUFFIXES share)
if(ProvizioTestParts_EXTRA)
    string(APPEND _provizio_test_parts_which \"+extra\")
endif()
if(PROVIZIO_TEST_PARTS_FORGET)
    unset(ProvizioTestParts_EXTRA CACHE)
endif()
set(ProvizioTestParts_FOUND TRUE)
if(NOT TARGET ProvizioTestParts::ProvizioTestParts)
    add_library(ProvizioTestParts::ProvizioTestParts INTERFACE IMPORTED)
    set_target_properties(ProvizioTestParts::ProvizioTestParts PROPERTIES
        INTERFACE_COMPILE_DEFINITIONS \"\${_provizio_test_parts_which}\")
endif()
")
# And one found by a find module of the consumer's own, which says "module", or what it is given as
# ProvizioTestFoo_FLAVOUR: a hint it reads as it is, with no lookup of its own, as FindZLIB reads a
# ZLIB_LIBRARY
file(WRITE "${WORK_DIR}/modules/FindProvizioTestFoo.cmake" "
set(ProvizioTestFoo_FOUND TRUE)
set(_provizio_test_foo_which module)
if(ProvizioTestFoo_FLAVOUR)
    set(_provizio_test_foo_which \"\${ProvizioTestFoo_FLAVOUR}\")
endif()
if(NOT TARGET ProvizioTestFoo::ProvizioTestFoo)
    add_library(ProvizioTestFoo::ProvizioTestFoo INTERFACE IMPORTED)
    set_target_properties(ProvizioTestFoo::ProvizioTestFoo PROPERTIES
        INTERFACE_COMPILE_DEFINITIONS \"\${_provizio_test_foo_which}\")
endif()
")

# Configures <project> (provizio or fast_dds) in <binary_dir> with the options given -- the Fast-DDS
# stand-in with PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY first, as provizio_dds configures Fast-DDS --
# failing the test on a failed configure, and leaves its output in _output
function(_configure project binary_dir)
    set(_first)
    if(project STREQUAL "fast_dds")
        set(_first "${PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY}")
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/openssl_package/${project}"
            -B "${binary_dir}" -G "${GENERATOR}" ${_first} "-DREPOSITORY=${SOURCE_DIR}" ${ARGN}
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "Configuring the ${project} stand-in in ${binary_dir} failed:\n${_output}")
    endif()
    set(_output "${_output}" PARENT_SCOPE)
endfunction()

# Runs provizio_dds's side with <options> in a fresh directory of <case>, then Fast-DDS's in
# <case>/fast_dds -- made afresh unless <reuse> -- as provizio_dds configures it, with the options in
# _FAST_DDS_OPTIONS as well, and checks that its dependencies were <expected>, leaving the output of
# the Fast-DDS side in _output
function(_check case reuse expected)
    file(REMOVE_RECURSE "${WORK_DIR}/${case}/provizio")
    _configure(provizio "${WORK_DIR}/${case}/provizio" "-DOUT_DIR=${WORK_DIR}/${case}/module"
        -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON ${ARGN})
    if(NOT reuse)
        file(REMOVE_RECURSE "${WORK_DIR}/${case}/fast_dds")
    endif()
    _configure(fast_dds "${WORK_DIR}/${case}/fast_dds" "-DCMAKE_MODULE_PATH=${WORK_DIR}/${case}/module"
        -DCMAKE_REQUIRE_FIND_PACKAGE_OpenSSL=ON ${_FAST_DDS_OPTIONS})
    # The module written must read back as written: CMake only warns of an argument a bracket ended early
    if(_output MATCHES "Syntax (Warning|Error)")
        message(FATAL_ERROR "Case ${case}: the module written for the Fast-DDS build does not parse cleanly:\n${_output}")
    endif()
    if(NOT _output MATCHES "openssl_package_fast_dds: OpenSSL 3\\.9\\.9 with \\[([^]]*)\\]")
        message(FATAL_ERROR "Case ${case}: the Fast-DDS stand-in did not report its OpenSSL:\n${_output}")
    endif()
    # Expanded, as an empty group leaves CMAKE_MATCH_1 undefined, which if() then reads as its name
    if(NOT "${CMAKE_MATCH_1}" STREQUAL "${expected}")
        message(FATAL_ERROR "Case ${case}: the package's dependencies were [${CMAKE_MATCH_1}] in the Fast-DDS "
            "build, where provizio_dds's were [${expected}]")
    endif()
    set(_output "${_output}" PARENT_SCOPE)
endfunction()

# 1. A dependency found through its <Package>_DIR
_check(package_dir FALSE "ProvizioTestZ::ProvizioTestZ=A"
    "-DOpenSSL_DIR=${_packages}/openssl_z/lib/cmake/OpenSSL"
    "-DProvizioTestZ_DIR=${_packages}/zA/lib/cmake/ProvizioTestZ")

# 2. A dependency found through its <Package>_ROOT
_check(package_root FALSE "ProvizioTestZ::ProvizioTestZ=B"
    "-DOpenSSL_DIR=${_packages}/openssl_z/lib/cmake/OpenSSL"
    "-DProvizioTestZ_ROOT=${_packages}/zB")

# 3. A dependency found by a module on the consumer's CMAKE_MODULE_PATH
_check(module_path FALSE "ProvizioTestFoo::ProvizioTestFoo=module"
    "-DOpenSSL_DIR=${_packages}/openssl_foo/lib/cmake/OpenSSL"
    "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules")

# 4. A dependency that has moved since the Fast-DDS build was configured: provizio_dds finds the new
#    one, and so must the Fast-DDS build configured again -- not the one its cache last held
_check(moved FALSE "ProvizioTestZ::ProvizioTestZ=A" "-DPREFIXES=${_packages}/openssl_z|${_packages}/zA")
_check(moved TRUE "ProvizioTestZ::ProvizioTestZ=B" "-DPREFIXES=${_packages}/openssl_z|${_packages}/zB")
file(STRINGS "${WORK_DIR}/moved/fast_dds/CMakeCache.txt" _cached REGEX "^(ProvizioTestZ|OpenSSL)_DIR:")
if(_cached)
    message(FATAL_ERROR "The OpenSSL lookup left [${_cached}] in the Fast-DDS build's cache, for its next "
        "configure to take over what provizio_dds finds then")
endif()

# 5. A package location whose value is a list, ahead of the rest: each location keeps its own value,
#    so the dependency still comes from its _DIR rather than from the other copy on the prefix path.
#    Values holding the closing bracket of a bracket argument, or ending in the most of one, too,
#    which must not end it, a location's or a search setting's -- each with as many [ as ], as CMake
#    splits no list at a ; that unpaired square brackets come before, which would join these
#    arguments into one.
_check(list_value FALSE "ProvizioTestZ::ProvizioTestZ=A"
    "-DOpenSSL_DIR=${_packages}/openssl_z/lib/cmake/OpenSSL"
    "-DProvizioTestZ_DIR=${_packages}/zA/lib/cmake/ProvizioTestZ"
    "-DPREFIXES=${_packages}/zB|${WORK_DIR}/nowhere/[d]=="
    "-DLIST_ROOT=${WORK_DIR}/nowhere/[[a]==]|${WORK_DIR}/nowhere/b"
    "-DPLAIN_LOCATION=AProvizioTestEnd_ROOT=${WORK_DIR}/nowhere/[c]==")

# Checks that the module written for <case> names <headers> as the package's headers
function(_expect_headers case headers)
    file(READ "${WORK_DIR}/${case}/module/FindOpenSSL.cmake" _module)
    string(FIND "${_module}" "# with its headers in\n#   ${headers}\n" _at)
    if(_at EQUAL -1)
        message(FATAL_ERROR "Case ${case}: the module does not name [${headers}] as the package's headers:\n${_module}")
    endif()
endfunction()

# 6. A package setting none of FindOpenSSL's variables, whose targets name its headers: found as any
#    other, its headers taken from the targets -- as the absolute directory they name plainly,
#    without its trailing separator, and not any piece of the generator expression, which cannot be
#    read before generation
_check(targets_only FALSE "ProvizioTestZ::ProvizioTestZ=A"
    "-DOpenSSL_DIR=${_packages}/openssl_targets/lib/cmake/OpenSSL"
    "-DProvizioTestZ_DIR=${_packages}/zA/lib/cmake/ProvizioTestZ")
_expect_headers(targets_only "${_packages}/openssl_targets/lib/cmake/OpenSSL/../../../include")

# 7. And one setting OPENSSL_INCLUDE_DIR to nothing, whose targets name its headers through a
#    generator expression alone: found just the same, with no directory to name for them
_check(generator_expression_only FALSE "ProvizioTestZ::ProvizioTestZ=A"
    "-DOpenSSL_DIR=${_packages}/openssl_generator_expression/lib/cmake/OpenSSL"
    "-DProvizioTestZ_DIR=${_packages}/zA/lib/cmake/ProvizioTestZ")
_expect_headers(generator_expression_only "(none its configuration names as a plain directory)")

# 8. The package of interface targets wrapping those naming its files: where its runtime is looked for
#    is where those files are, and the bin directory beside them -- not where its dependency is, whose
#    OpenSSL is another
_check(wrapped FALSE "" "-DOpenSSL_DIR=${_wrapped}/lib/cmake/OpenSSL")
file(READ "${WORK_DIR}/wrapped/module/runtime_dirs.txt" _runtime_dirs)
if(NOT _runtime_dirs STREQUAL "${_wrapped}/lib;${_wrapped}/bin")
    message(FATAL_ERROR "Case wrapped: the OpenSSL runtime would be looked for in [${_runtime_dirs}], not in "
        "[${_wrapped}/lib;${_wrapped}/bin]")
endif()

# 9. The package naming its file itself as well as linking others': its own directory comes first,
#    though the linked targets' files are of the configuration Fast-DDS takes; then those, listed
#    for it or not; then that of the file linked by its path, then that of no configuration, then
#    that of another configuration -- and nothing from the dependency's directory, though a target
#    bears the name of a component of its path
_check(ranked FALSE "" "-DOpenSSL_DIR=${_ranked}/lib/cmake/OpenSSL")
file(READ "${WORK_DIR}/ranked/module/runtime_dirs.txt" _runtime_dirs)
set(_expected "${_ranked}/lib;${_other}/lib;${_unlisted}/lib;${_pkgconf}/lib;${_plain}/lib;${_debug}/lib")
if(NOT _runtime_dirs STREQUAL _expected)
    message(FATAL_ERROR "Case ranked: the OpenSSL runtime would be looked for in [${_runtime_dirs}], not in "
        "[${_expected}]")
endif()

# 10. A dependency found through a <Package>_ROOT or a <Package>_DIR that is an ordinary variable,
#     not a cache entry, as a toolchain or a project around provizio_dds may set one: the Fast-DDS
#     build, which gets neither, finds it there all the same. The _ROOT one's by a find module, as
#     a package configuration found through its _ROOT has its _DIR cached, which would do as well.
_check(plain_root FALSE "ProvizioTestBar::ProvizioTestBar=under_root"
    "-DOpenSSL_DIR=${_packages}/openssl_bar/lib/cmake/OpenSSL" "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules"
    "-DPLAIN_LOCATION=ProvizioTestBar_ROOT=${_packages}/bar_root")
_check(plain_dir FALSE "ProvizioTestZ::ProvizioTestZ=A"
    "-DOpenSSL_DIR=${_packages}/openssl_z/lib/cmake/OpenSSL"
    "-DPLAIN_LOCATION=ProvizioTestZ_DIR=${_packages}/zA/lib/cmake/ProvizioTestZ")

# 11. A Fast-DDS build that holds a result of the dependency's lookup in its cache already, another
#     than provizio_dds's own: the lookup there takes provizio_dds's, not that one
file(WRITE "${_packages}/stale_bar/share/provizio_test_bar.marker" "stale")
set(_FAST_DDS_OPTIONS "-DProvizioTestBar_MARKER:FILEPATH=${_packages}/stale_bar/share/provizio_test_bar.marker")
_check(stale_result FALSE "ProvizioTestBar::ProvizioTestBar=under_root"
    "-DOpenSSL_DIR=${_packages}/openssl_bar/lib/cmake/OpenSSL" "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules"
    "-DProvizioTestBar_ROOT=${_packages}/bar_root")
unset(_FAST_DDS_OPTIONS)
# ...and its own result is as it was after the lookup, provizio_dds's written there for it alone
file(STRINGS "${WORK_DIR}/stale_result/fast_dds/CMakeCache.txt" _cached REGEX "^ProvizioTestBar_MARKER:")
if(NOT _cached STREQUAL "ProvizioTestBar_MARKER:FILEPATH=${_packages}/stale_bar/share/provizio_test_bar.marker")
    message(FATAL_ERROR "Case stale_result: the Fast-DDS build's own result is [${_cached}] after the lookup")
endif()

# 12. A dependency whose find module looks several files up under one name, unsetting it in between:
#     the Fast-DDS build's lookup finds each, as provizio_dds's did, not the last of them every time;
#     and one more file, which provizio_dds's lookup found none of, and the Fast-DDS build holds a
#     result for in its cache already: looked for again there, not taken from that
set(_FAST_DDS_OPTIONS "-DProvizioTestParts_EXTRA:FILEPATH=${_packages}/stale_parts/share/part_extra.marker")
_check(several_results FALSE "ProvizioTestParts::ProvizioTestParts=AB"
    "-DOpenSSL_DIR=${_packages}/openssl_parts/lib/cmake/OpenSSL" "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules"
    "-DProvizioTestParts_ROOT=${_packages}/parts_root")
# ...and where the lookup removes an entry the Fast-DDS build had, that entry is put back as it was:
# its value, its type, its being advanced and the choice it offers
set(_FAST_DDS_OPTIONS "-DProvizioTestParts_EXTRA:PATH=${_packages}/stale_parts/share" -DPROVIZIO_TEST_PARTS_FORGET=ON
    -DDRESSED=ProvizioTestParts_EXTRA)
_check(forgotten_result FALSE "ProvizioTestParts::ProvizioTestParts=AB"
    "-DOpenSSL_DIR=${_packages}/openssl_parts/lib/cmake/OpenSSL" "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules"
    "-DProvizioTestParts_ROOT=${_packages}/parts_root")
unset(_FAST_DDS_OPTIONS)
file(STRINGS "${WORK_DIR}/forgotten_result/fast_dds/CMakeCache.txt" _cached REGEX "^ProvizioTestParts_EXTRA[-:]")
list(SORT _cached)
if(NOT _cached STREQUAL "ProvizioTestParts_EXTRA-ADVANCED:INTERNAL=1;ProvizioTestParts_EXTRA-STRINGS:INTERNAL=offered;ProvizioTestParts_EXTRA:PATH=${_packages}/stale_parts/share")
    message(FATAL_ERROR "Case forgotten_result: the Fast-DDS build's own entry is [${_cached}] after the lookup")
endif()

# 13. A hint given to provizio_dds on its command line, which nothing declares or looks up -- the find
#     module reads it as it is: the Fast-DDS build's lookup reads it too, and keeps nothing of it, nor
#     any entry named as the journal's are, however it got there
set(_FAST_DDS_OPTIONS -D_PROVIZIO_DDS_OPENSSL_SEEDED=Stray -D_PROVIZIO_DDS_OPENSSL_SEEDED_VALUE_Stray=left)
_check(given_value FALSE "ProvizioTestFoo::ProvizioTestFoo=given"
    "-DOpenSSL_DIR=${_packages}/openssl_foo/lib/cmake/OpenSSL" "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules"
    -DProvizioTestFoo_FLAVOUR=given)
unset(_FAST_DDS_OPTIONS)
file(STRINGS "${WORK_DIR}/given_value/fast_dds/CMakeCache.txt" _cached
    REGEX "^(ProvizioTestFoo_FLAVOUR|_PROVIZIO_DDS_OPENSSL_[^:]*|Stray):")
if(_cached)
    message(FATAL_ERROR "Case given_value: the lookup left [${_cached}] in the Fast-DDS build's cache")
endif()

# ...and one given to provizio_dds empty, which the Fast-DDS build holds a value of already: read
# there empty, as provizio_dds's lookup read it, and that build's own value back after
set(_FAST_DDS_OPTIONS -DProvizioTestFoo_FLAVOUR=stale)
_check(emptied_value FALSE "ProvizioTestFoo::ProvizioTestFoo=module"
    "-DOpenSSL_DIR=${_packages}/openssl_foo/lib/cmake/OpenSSL" "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules"
    -DProvizioTestFoo_FLAVOUR=)
unset(_FAST_DDS_OPTIONS)
file(STRINGS "${WORK_DIR}/emptied_value/fast_dds/CMakeCache.txt" _cached REGEX "^ProvizioTestFoo_FLAVOUR:")
if(NOT _cached STREQUAL "ProvizioTestFoo_FLAVOUR:UNINITIALIZED=stale")
    message(FATAL_ERROR "Case emptied_value: the Fast-DDS build's own value is [${_cached}] after the lookup")
endif()

# 14. A Fast-DDS configure that stops inside the lookup, and the next one: that one finds the cache as
#     it was before the first, ahead of its own command line and of any code of Fast-DDS's -- the
#     Fast-DDS build's own entry as that build had it, but where its command line sets it anew, and
#     none of provizio_dds's results the first left there, nor any of the lookup's own -- whether it
#     looks OpenSSL up as a package again or not; and one not given the recovery first, as a configure
#     by hand would not be, which the lookup puts the cache back for itself. Each in a build of its
#     own, the first configure of each stopping alike.
set(_stale "${_packages}/stale_bar/share/provizio_test_bar.marker")
set(_given_anew "${_packages}/stale_bar/share/given_anew.marker")
set(_options "-DOpenSSL_DIR=${_packages}/openssl_bar/lib/cmake/OpenSSL" "-DCMAKE_MODULE_PATH=${WORK_DIR}/modules"
    "-DProvizioTestBar_ROOT=${_packages}/bar_root")
foreach(_case IN ITEMS interrupted interrupted_otherwise interrupted_by_hand)
    _configure(provizio "${WORK_DIR}/${_case}/provizio" "-DOUT_DIR=${WORK_DIR}/${_case}/module"
        -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON ${_options})
    execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/openssl_package/fast_dds"
            -B "${WORK_DIR}/${_case}/fast_dds" -G "${GENERATOR}" "${PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY}"
            "-DREPOSITORY=${SOURCE_DIR}" "-DCMAKE_MODULE_PATH=${WORK_DIR}/${_case}/module"
            -DCMAKE_REQUIRE_FIND_PACKAGE_OpenSSL=ON "-DProvizioTestBar_MARKER:FILEPATH=${_stale}"
            -DPROVIZIO_TEST_FAIL_LOOKUP=ON
        RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
    if(_result EQUAL 0 OR NOT _output MATCHES "the lookup fails, as asked")
        message(FATAL_ERROR "Case ${_case}: the Fast-DDS stand-in's lookup was to stop the configure:\n${_output}")
    endif()
endforeach()
# ...the next looking OpenSSL up as a package again, and given an entry of the build's own anew,
# which is to read as given, before the lookup and after it
set(_FAST_DDS_OPTIONS -DPROVIZIO_TEST_FAIL_LOOKUP=OFF "-DProvizioTestBar_MARKER:FILEPATH=${_given_anew}"
    -DREPORT_BEFORE=ProvizioTestBar_ROOT|ProvizioTestBar_CHECKED|ProvizioTestBar_MARKER)
_check(interrupted TRUE "ProvizioTestBar::ProvizioTestBar=under_root" ${_options})
unset(_FAST_DDS_OPTIONS)
if(NOT _output MATCHES "openssl_package_fast_dds: before the lookup: \\[ProvizioTestBar_ROOT=;ProvizioTestBar_CHECKED=;ProvizioTestBar_MARKER=([^]]*)\\]"
        OR NOT CMAKE_MATCH_1 STREQUAL _given_anew)
    message(FATAL_ERROR "Case interrupted: Fast-DDS's own code found the cache as the stopped lookup left it:\n${_output}")
endif()
# ...the next not given the recovery first
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/openssl_package/fast_dds"
        -B "${WORK_DIR}/interrupted_by_hand/fast_dds" -G "${GENERATOR}" -DPROVIZIO_TEST_FAIL_LOOKUP=OFF
    RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
if(NOT _result EQUAL 0)
    message(FATAL_ERROR "Case interrupted_by_hand: configuring the Fast-DDS stand-in failed:\n${_output}")
endif()
# ...and the next looking it up otherwise, without the module
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_LIST_DIR}/openssl_package/fast_dds"
        -B "${WORK_DIR}/interrupted_otherwise/fast_dds" -G "${GENERATOR}" "${PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY}"
        "-DREPOSITORY=${SOURCE_DIR}" -UCMAKE_MODULE_PATH -DPROVIZIO_TEST_FAIL_LOOKUP=OFF -DPROVIZIO_TEST_SKIP_LOOKUP=ON
    RESULT_VARIABLE _result OUTPUT_VARIABLE _output ERROR_VARIABLE _output)
if(NOT _result EQUAL 0)
    message(FATAL_ERROR "Case interrupted_otherwise: configuring the Fast-DDS stand-in failed:\n${_output}")
endif()
foreach(_case IN ITEMS interrupted interrupted_otherwise interrupted_by_hand)
    if(_case STREQUAL "interrupted")
        set(_expected "ProvizioTestBar_MARKER:FILEPATH=${_given_anew}")
    else()
        set(_expected "ProvizioTestBar_MARKER:FILEPATH=${_stale}")
    endif()
    file(STRINGS "${WORK_DIR}/${_case}/fast_dds/CMakeCache.txt" _cached
        REGEX "^(ProvizioTestBar_[A-Z]*|OpenSSL_DIR|_PROVIZIO_DDS_OPENSSL_[^:]*)[-:]")
    if(NOT _cached STREQUAL _expected)
        message(FATAL_ERROR "Case ${_case}: the Fast-DDS build's cache holds [${_cached}], not [${_expected}]")
    endif()
endforeach()

# 15. A package whose directory's name holds a line break, which the comments of the module written
#     name it by: the module reads back all the same, on a POSIX host
if(CMAKE_HOST_UNIX)
    _check(line_break FALSE "ProvizioTestZ::ProvizioTestZ=A"
        "-DOpenSSL_DIR=${_packages}/openssl_line\nbreak/lib/cmake/OpenSSL"
        "-DProvizioTestZ_DIR=${_packages}/zA/lib/cmake/ProvizioTestZ")
endif()

# 17. A package whose target maps the configuration Fast-DDS builds in onto another: the runtime is
#     looked for beside the file of the one it maps it onto first, the first of those it names that
#     it has a file of -- but where FindOpenSSL found it, whose maps reach no Fast-DDS -- and one
#     mapping it onto the file of no configuration; and one that maps nothing, of which the module
#     given to Fast-DDS maps it onto another
_check(mapped FALSE "" "-DOpenSSL_DIR=${_packages}/openssl_mapped/lib/cmake/OpenSSL")
_check(mapped_none FALSE "" "-DOpenSSL_DIR=${_packages}/openssl_mapped_none/lib/cmake/OpenSSL")
_check(unmapped FALSE "" "-DOpenSSL_DIR=${_packages}/openssl_unmapped/lib/cmake/OpenSSL" -DMAPPED_CONFIG=debug)
foreach(_case_expected IN ITEMS "mapped|runtime_dirs|openssl_mapped/relwithdebinfo"
        "mapped|runtime_dirs_findopenssl|openssl_mapped/release" "mapped_none|runtime_dirs|openssl_mapped_none/none"
        "unmapped|runtime_dirs|openssl_unmapped/debug")
    string(REPLACE "|" ";" _case_expected "${_case_expected}")
    list(GET _case_expected 0 _case)
    list(GET _case_expected 1 _list)
    list(GET _case_expected 2 _expected)
    file(READ "${WORK_DIR}/${_case}/module/${_list}.txt" _runtime_dirs)
    list(GET _runtime_dirs 0 _first)
    if(NOT _first STREQUAL "${_packages}/${_expected}/lib")
        message(FATAL_ERROR "Case ${_case}: the OpenSSL runtime would be looked for in [${_runtime_dirs}], "
            "not first in [${_packages}/${_expected}/lib]")
    endif()
endforeach()

# 18. A package linking its files by their paths inside generator expressions alone: the runtime is
#     looked for beside the file of the configuration Fast-DDS links first, and beside the others
#     after, in the order they come in
_check(genex_paths FALSE "" "-DOpenSSL_DIR=${_genex}/lib/cmake/OpenSSL")
file(READ "${WORK_DIR}/genex_paths/module/runtime_dirs.txt" _runtime_dirs)
if(NOT _runtime_dirs STREQUAL "${_genex}/release/lib;${_genex}/debug/lib;${_genex}/link_only/lib")
    message(FATAL_ERROR "Case genex_paths: the OpenSSL runtime would be looked for in [${_runtime_dirs}], not "
        "[${_genex}/release/lib;${_genex}/debug/lib;${_genex}/link_only/lib]")
endif()

# 16. FindOpenSSL's results naming a library of each configuration, as with MSVC (debug <file>
#     optimized <file>): the runtime is looked for beside the build type's first, whatever the case
#     of its name, as CMake takes a configuration's name
file(WRITE "${WORK_DIR}/keywords/debug/lib/libssld.lib" "")
file(WRITE "${WORK_DIR}/keywords/release/lib/libssl.lib" "")
set(OPENSSL_SSL_LIBRARY debug "${WORK_DIR}/keywords/debug/lib/libssld.lib"
    optimized "${WORK_DIR}/keywords/release/lib/libssl.lib")
set(OPENSSL_CRYPTO_LIBRARY "")
foreach(_build_type IN ITEMS Debug debug DEBUG Release release RelWithDebInfo)
    set(_expected "${WORK_DIR}/keywords/release/lib")
    if(_build_type MATCHES "^[Dd][Ee][Bb][Uu][Gg]$")
        set(_expected "${WORK_DIR}/keywords/debug/lib")
    endif()
    provizio_dds_openssl_runtime_dirs(_runtime_dirs "${_build_type}")
    list(GET _runtime_dirs 0 _first)
    if(NOT _first STREQUAL _expected)
        message(FATAL_ERROR "Case keywords (${_build_type}): the OpenSSL runtime would be looked for in "
            "[${_runtime_dirs}], not first in [${_expected}]")
    endif()
endforeach()
unset(OPENSSL_SSL_LIBRARY)
unset(OPENSSL_CRYPTO_LIBRARY)

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "fast_dds_openssl_package: the Fast-DDS build finds what provizio_dds found, in every case")
