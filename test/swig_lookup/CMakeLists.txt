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

# provizio_dds_find_swig (cmake/python_bindings/swig_lookup.cmake): configures of a stand-in project
# with stand-in SWIGs. Registered wherever tests are, whatever this tree was configured with: what it
# checks is the source, not this build -- on Windows too, which looks SWIG up the same way, and whose
# install_dependencies.ps1 replaces C:\swig in place as one of the cases does.
#
# The SWIGs are shell scripts where a script runs as a program (Linux, macOS), and copies of the
# swig_stand_in built here where only a program image does (Windows, whose CreateProcess, which
# FindSWIG's execute_process goes through, runs no #! script). Linux runs both kinds, so that the
# built one, which only Windows needs, is exercised on a host where it can also be debugged; macOS
# needs no second kind, the scripts covering the lookup there as they do on Linux.
set(_swig_stand_in_arg)
if(NOT APPLE)
    add_executable(swig_stand_in swig_stand_in.cpp)
    # The program alone: not the object every test executable here links (see link_libraries() in
    # test/CMakeLists.txt), built against MSVC's DLL runtime, which an executable of the static one
    # cannot link; and the static runtime, so that a copy loads no runtime DLL wherever it runs from --
    # the debug one is found only through the developer environment's search path, which the cases
    # replace with the stand-ins' directories
    set_property(TARGET swig_stand_in PROPERTY LINK_LIBRARIES "")
    set_property(TARGET swig_stand_in PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set(_swig_stand_in_arg "-DSTAND_IN=$<TARGET_FILE:swig_stand_in>")
endif()

add_test(NAME swig_lookup
    COMMAND "${CMAKE_COMMAND}"
            "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}"
            "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/swig_lookup"
            "-DGENERATOR=${CMAKE_GENERATOR}"
            "-DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
            ${_swig_stand_in_arg}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/swig_lookup_test.cmake")
# Seconds. Twelve configures of a project that enables no language, for each kind of stand-in.
set_tests_properties(swig_lookup PROPERTIES TIMEOUT 120)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
