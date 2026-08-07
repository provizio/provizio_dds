# Defines provizio_dds_bin_cache_host_abi_compatible(), which decides whether the prebuilt Linux
# binaries of a bin cache can be used on this host machine.
#
# What actually decides it is the *libc-level ABI*, not the kernel:
#
# - The cached binaries have their libc dependencies deliberately left out of the cache
#   (.github/workflows/build_cache.sh excludes libc / libm / librt / libpthread / libdl from the
#   libraries it collects), so every one of them resolves its libc symbols against the host's
#   glibc. Those references are version-tagged (e.g. `pthread_kill@GLIBC_2.34`), and a glibc only
#   ever provides tags up to its own version, so a host older than the one the binaries were built
#   against fails at link or load time with "version `GLIBC_2.35' not found".
# - The same holds for libstdc++ (`...@GLIBCXX_3.4.30`, `...@CXXABI_1.3.13`). A copy is shipped in
#   the cache, but it is not a reliable substitute: DONT_INSTALL_STDCPP_LIBS (ON by default) keeps
#   it out of the install tree, and even when it is installed, the consumer executable normally
#   makes the loader resolve libstdc++.so.6 from the default search path first — so the host's copy
#   is what has to satisfy those references.
# - Everything else a cached binary needs is either shipped in the cache and installed alongside it
#   (Fast-DDS, Fast-CDR, OpenSSL - hence OPENSSL_* tags need nothing from the host) or trivially
#   satisfied (the GCC_* tags needed from libgcc top out at GCC_4.2.0, released in 2007, so any
#   libgcc paired with a glibc new enough to pass the glibc check provides them).
#
# The kernel version of the machine that built a cache, in contrast, has no bearing on any of this:
# none of the shipped ELF objects carries an NT_GNU_ABI_TAG note, i.e. none declares a minimum
# kernel at all. The only kernel floor in play is the one the host's own glibc was configured with,
# and that glibc is by definition already running on the host.
#
# Because symbol versioning is strictly backwards compatible, the check is a simple comparison:
# every cache records the maximum GLIBC / GLIBCXX / CXXABI level its binaries require in an
# `abi_requirements` file (computed with readelf when the cache is built, so consumers need no
# binutils of their own), and this module compares that against what the host provides.
#
# Every failure to establish either side of the comparison results in a FALSE verdict (build from
# sources) rather than an error, so a host missing the tools this module probes with just loses the
# cache instead of failing to configure.

# Reads one `<key>=<version>` entry of a cache's abi_requirements file into _out_var (empty if the
# key is absent or has no value).
function(_provizio_dds_read_abi_requirement _requirements_file _key _out_var)
    set(${_out_var} "" PARENT_SCOPE)
    file(STRINGS "${_requirements_file}" _matches REGEX "^${_key}=")
    foreach(_match IN LISTS _matches)
        if(_match MATCHES "^${_key}=([0-9]+(\\.[0-9]+)*)$")
            set(${_out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
        endif()
    endforeach()
endfunction()

# Sets _out_var to the greater of its current value and _candidate (either may be empty).
function(_provizio_dds_max_version _candidate _out_var)
    if(_candidate AND(NOT ${_out_var} OR _candidate VERSION_GREATER "${${_out_var}}"))
        set(${_out_var} "${_candidate}" PARENT_SCOPE)
    endif()
endfunction()

# Sets _out_var to the host's glibc version, or leaves it empty if it can't be established (no
# glibc at all, e.g. musl, or a system too stripped down to report it).
function(_provizio_dds_host_glibc_version _out_var)
    set(${_out_var} "" PARENT_SCOPE)

    # getconf reports the version of the glibc it is running against, and the GNU_LIBC_VERSION key
    # exists nowhere else, so a successful answer is unambiguous
    find_program(PROVIZIO_DDS_GETCONF_EXECUTABLE NAMES getconf)
    if(PROVIZIO_DDS_GETCONF_EXECUTABLE)
        execute_process(COMMAND "${PROVIZIO_DDS_GETCONF_EXECUTABLE}" GNU_LIBC_VERSION
            OUTPUT_VARIABLE _getconf_output RESULT_VARIABLE _getconf_result
            ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
        if(_getconf_result EQUAL 0 AND _getconf_output MATCHES "glibc ([0-9]+\\.[0-9]+(\\.[0-9]+)?)")
            set(${_out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
            return()
        endif(_getconf_result EQUAL 0 AND _getconf_output MATCHES "glibc ([0-9]+\\.[0-9]+(\\.[0-9]+)?)")
    endif(PROVIZIO_DDS_GETCONF_EXECUTABLE)

    # Fall back to ldd, whose first line ends in the glibc version ("ldd (Ubuntu GLIBC
    # 2.35-0ubuntu3) 2.35"). Other implementations (musl) also have an ldd, so only trust the
    # output if it names glibc.
    find_program(PROVIZIO_DDS_LDD_EXECUTABLE NAMES ldd)
    if(PROVIZIO_DDS_LDD_EXECUTABLE)
        execute_process(COMMAND "${PROVIZIO_DDS_LDD_EXECUTABLE}" --version
            OUTPUT_VARIABLE _ldd_output RESULT_VARIABLE _ldd_result ERROR_QUIET)
        string(REGEX REPLACE "\n.*" "" _ldd_first_line "${_ldd_output}")
        string(TOUPPER "${_ldd_first_line}" _ldd_first_line_upper)
        if(_ldd_result EQUAL 0
           AND(_ldd_first_line_upper MATCHES "GLIBC" OR _ldd_first_line_upper MATCHES "GNU LIBC")
           AND _ldd_first_line MATCHES "([0-9]+\\.[0-9]+(\\.[0-9]+)?)[ \t]*$")
            set(${_out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
        endif()
    endif(PROVIZIO_DDS_LDD_EXECUTABLE)
endfunction()

# Sets _path_var to the host's libstdc++.so.6 and _glibcxx_var / _cxxabi_var to the highest
# GLIBCXX_ / CXXABI_ versions it provides, or leaves them empty if it can't be found or read.
#
# Both the copy the compiler links against and the one in the loader's default search path are
# considered, and the lower of the two wins: they are the same file on a stock distribution, and
# where they aren't (a hand-installed toolchain, a Conda prefix) there is no telling which one a
# consumer's process will end up with, so the pessimistic answer is the safe one.
function(_provizio_dds_host_libstdcxx_versions _path_var _glibcxx_var _cxxabi_var)
    set(${_path_var} "" PARENT_SCOPE)
    set(${_glibcxx_var} "" PARENT_SCOPE)
    set(${_cxxabi_var} "" PARENT_SCOPE)

    set(_candidates "")
    set(_lowest_glibcxx_path "")
    set(_lowest_cxxabi_path "")
    set(_lowest_glibcxx "")
    set(_lowest_cxxabi "")

    if(CMAKE_CXX_COMPILER)
        execute_process(COMMAND "${CMAKE_CXX_COMPILER}" -print-file-name=libstdc++.so.6
            OUTPUT_VARIABLE _compiler_libstdcxx RESULT_VARIABLE _compiler_result
            ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
        # Both GCC and Clang echo the name back unchanged when they can't locate the library
        if(_compiler_result EQUAL 0 AND IS_ABSOLUTE "${_compiler_libstdcxx}" AND EXISTS "${_compiler_libstdcxx}")
            list(APPEND _candidates "${_compiler_libstdcxx}")
        endif(_compiler_result EQUAL 0 AND IS_ABSOLUTE "${_compiler_libstdcxx}" AND EXISTS "${_compiler_libstdcxx}")
    endif(CMAKE_CXX_COMPILER)

    # The loader's default search path: multiarch (Debian derivatives), lib64 (Red Hat / SUSE
    # derivatives) and plain lib layouts. CMAKE_LIBRARY_ARCHITECTURE is empty in CMake script mode,
    # hence the wildcard too.
    set(_default_path_globs "")
    if(CMAKE_LIBRARY_ARCHITECTURE)
        list(APPEND _default_path_globs
            "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/libstdc++.so.6"
            "/lib/${CMAKE_LIBRARY_ARCHITECTURE}/libstdc++.so.6")
    endif(CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND _default_path_globs
        "/usr/lib/*-linux-gnu*/libstdc++.so.6"
        "/lib/*-linux-gnu*/libstdc++.so.6"
        "/usr/lib64/libstdc++.so.6"
        "/lib64/libstdc++.so.6"
        "/usr/lib/libstdc++.so.6"
        "/lib/libstdc++.so.6")
    foreach(_glob IN LISTS _default_path_globs)
        file(GLOB _found "${_glob}")
        if(_found)
            list(APPEND _candidates ${_found})
            break()
        endif(_found)
    endforeach(_glob IN LISTS _default_path_globs)

    # Resolve to real paths so that the same library found twice (the compiler's copy usually IS the
    # one in the default search path, just reached through a relative chain) is only read once
    set(_resolved_candidates "")
    foreach(_candidate IN LISTS _candidates)
        get_filename_component(_candidate "${_candidate}" REALPATH)
        list(APPEND _resolved_candidates "${_candidate}")
    endforeach(_candidate IN LISTS _candidates)
    list(REMOVE_DUPLICATES _resolved_candidates)

    foreach(_candidate IN LISTS _resolved_candidates)
        # The symbol version names live in .dynstr as plain NUL-terminated ASCII, so they can be
        # read without binutils. libstdc++ requires no GLIBCXX_ / CXXABI_ version of anything else,
        # so every such string in it is one it defines.
        file(STRINGS "${_candidate}" _glibcxx_versions REGEX "^GLIBCXX_[0-9]+\\.[0-9]+(\\.[0-9]+)?$")
        file(STRINGS "${_candidate}" _cxxabi_versions REGEX "^CXXABI_[0-9]+\\.[0-9]+(\\.[0-9]+)?$")

        set(_candidate_glibcxx "")
        foreach(_version IN LISTS _glibcxx_versions)
            string(REGEX REPLACE "^GLIBCXX_" "" _version "${_version}")
            _provizio_dds_max_version("${_version}" _candidate_glibcxx)
        endforeach(_version IN LISTS _glibcxx_versions)

        set(_candidate_cxxabi "")
        foreach(_version IN LISTS _cxxabi_versions)
            string(REGEX REPLACE "^CXXABI_" "" _version "${_version}")
            _provizio_dds_max_version("${_version}" _candidate_cxxabi)
        endforeach(_version IN LISTS _cxxabi_versions)

        if(NOT _candidate_glibcxx OR NOT _candidate_cxxabi)
            # Not a readable libstdc++ after all, don't let it mask a usable candidate
            continue()
        endif(NOT _candidate_glibcxx OR NOT _candidate_cxxabi)

        # Take the minimum of each field independently: a candidate can have a higher GLIBCXX
        # but a lower CXXABI than another, and using only the GLIBCXX-lowest candidate's CXXABI
        # would then overstate what the host guarantees. Each minimum remembers the file it came
        # from so the rejection message can never attribute a version to a library that does not
        # actually provide it.
        if(NOT _lowest_glibcxx OR _candidate_glibcxx VERSION_LESS "${_lowest_glibcxx}")
            set(_lowest_glibcxx "${_candidate_glibcxx}")
            set(_lowest_glibcxx_path "${_candidate}")
        endif(NOT _lowest_glibcxx OR _candidate_glibcxx VERSION_LESS "${_lowest_glibcxx}")
        if(NOT _lowest_cxxabi OR _candidate_cxxabi VERSION_LESS "${_lowest_cxxabi}")
            set(_lowest_cxxabi "${_candidate_cxxabi}")
            set(_lowest_cxxabi_path "${_candidate}")
        endif(NOT _lowest_cxxabi OR _candidate_cxxabi VERSION_LESS "${_lowest_cxxabi}")
    endforeach(_candidate IN LISTS _resolved_candidates)

    if(_lowest_glibcxx)
        # Usually both minima come from the same library and it is named once. When they do not (a
        # hand-installed or Conda toolchain sitting alongside the system one), name both: the
        # consumer of this output pairs it with "GLIBCXX_<x> / CXXABI_<y>", so listing the two
        # paths in the same order keeps every version attributed to the file that provides it.
        if(_lowest_glibcxx_path STREQUAL "${_lowest_cxxabi_path}")
            set(${_path_var} "${_lowest_glibcxx_path}" PARENT_SCOPE)
        else(_lowest_glibcxx_path STREQUAL "${_lowest_cxxabi_path}")
            set(${_path_var} "${_lowest_glibcxx_path} / ${_lowest_cxxabi_path}" PARENT_SCOPE)
        endif(_lowest_glibcxx_path STREQUAL "${_lowest_cxxabi_path}")
        set(${_glibcxx_var} "${_lowest_glibcxx}" PARENT_SCOPE)
        set(${_cxxabi_var} "${_lowest_cxxabi}" PARENT_SCOPE)
    endif(_lowest_glibcxx)
endfunction()

# provizio_dds_bin_cache_host_abi_compatible(<result_var> <reason_var> <cache_dir>...)
#
# Sets <result_var> to TRUE when this host provides everything the prebuilt binaries in all of the
# given (already extracted) cache directories require, and FALSE otherwise. On FALSE, <reason_var>
# is set to a message explaining what was required and what was found, phrased to complete a
# sentence like "won't be used as ...".
function(provizio_dds_bin_cache_host_abi_compatible _result_var _reason_var)
    set(${_result_var} FALSE PARENT_SCOPE)

    # What the caches require
    set(_required_glibc "")
    set(_required_glibcxx "")
    set(_required_cxxabi "")
    foreach(_cache_dir IN LISTS ARGN)
        set(_requirements_file "${_cache_dir}/abi_requirements")
        if(NOT EXISTS "${_requirements_file}")
            set(${_reason_var}
                "they don't record the ABI level they require (no ${_requirements_file}), so their compatibility with this host can't be established"
                PARENT_SCOPE)
            return()
        endif(NOT EXISTS "${_requirements_file}")

        _provizio_dds_read_abi_requirement("${_requirements_file}" glibc _cache_glibc)
        if(NOT _cache_glibc)
            set(${_reason_var}
                "${_requirements_file} declares no glibc requirement, so their compatibility with this host can't be established"
                PARENT_SCOPE)
            return()
        endif(NOT _cache_glibc)
        _provizio_dds_read_abi_requirement("${_requirements_file}" glibcxx _cache_glibcxx)
        _provizio_dds_read_abi_requirement("${_requirements_file}" cxxabi _cache_cxxabi)
        # A C++ cache always requires versioned GLIBCXX_/CXXABI_ symbols, and the cache
        # builder hard-fails rather than record empty values — so a requirements file
        # declaring neither can only be a failed scan or a hand-edited file. Accepting it
        # would silently skip the libstdc++ gate below and fail at load time instead.
        if(NOT _cache_glibcxx AND NOT _cache_cxxabi)
            set(${_reason_var}
                "${_requirements_file} declares no libstdc++ (GLIBCXX/CXXABI) requirement, so their compatibility with this host can't be established"
                PARENT_SCOPE)
            return()
        endif(NOT _cache_glibcxx AND NOT _cache_cxxabi)

        _provizio_dds_max_version("${_cache_glibc}" _required_glibc)
        _provizio_dds_max_version("${_cache_glibcxx}" _required_glibcxx)
        _provizio_dds_max_version("${_cache_cxxabi}" _required_cxxabi)
    endforeach(_cache_dir IN LISTS ARGN)

    # What the host provides
    _provizio_dds_host_glibc_version(_host_glibc)
    if(NOT _host_glibc)
        set(${_reason_var}
            "they require glibc ${_required_glibc} and this host's glibc version couldn't be determined"
            PARENT_SCOPE)
        return()
    endif(NOT _host_glibc)

    if(_host_glibc VERSION_LESS "${_required_glibc}")
        set(${_reason_var}
            "they require glibc ${_required_glibc} or newer, while this host provides glibc ${_host_glibc}"
            PARENT_SCOPE)
        return()
    endif(_host_glibc VERSION_LESS "${_required_glibc}")

    if(_required_glibcxx OR _required_cxxabi)
        _provizio_dds_host_libstdcxx_versions(_host_libstdcxx _host_glibcxx _host_cxxabi)
        if(NOT _host_libstdcxx)
            set(${_reason_var}
                "they require libstdc++ providing GLIBCXX_${_required_glibcxx} / CXXABI_${_required_cxxabi} and this host's libstdc++ couldn't be located"
                PARENT_SCOPE)
            return()
        endif(NOT _host_libstdcxx)

        if((_required_glibcxx AND _host_glibcxx VERSION_LESS "${_required_glibcxx}")
           OR(_required_cxxabi AND _host_cxxabi VERSION_LESS "${_required_cxxabi}"))
            set(${_reason_var}
                "they require libstdc++ providing GLIBCXX_${_required_glibcxx} / CXXABI_${_required_cxxabi}, while this host's ${_host_libstdcxx} provides GLIBCXX_${_host_glibcxx} / CXXABI_${_host_cxxabi}"
                PARENT_SCOPE)
            return()
        endif()

        message(STATUS
            "Prebuilt binaries ABI check: require glibc ${_required_glibc}, GLIBCXX_${_required_glibcxx}, "
            "CXXABI_${_required_cxxabi}; host provides glibc ${_host_glibc}, GLIBCXX_${_host_glibcxx}, "
            "CXXABI_${_host_cxxabi}")
    else()
        message(STATUS
            "Prebuilt binaries ABI check: require glibc ${_required_glibc}; host provides glibc ${_host_glibc}")
    endif(_required_glibcxx OR _required_cxxabi)

    set(${_result_var} TRUE PARENT_SCOPE)
    set(${_reason_var} "" PARENT_SCOPE)
endfunction()
