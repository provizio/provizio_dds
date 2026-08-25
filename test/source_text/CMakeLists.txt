# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

# Source-text checks. These read the repository rather than the build, so they are
# registered outside the PYTHON_BINDINGS guard: they need an interpreter (already required
# above for run_parallel.py), not the bindings, and the rule they enforce applies to the
# C++ sources too.
add_test(NAME runtime_text_is_ascii COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/runtime_text_is_ascii.py" "${CMAKE_SOURCE_DIR}"
)
set_tests_properties(runtime_text_is_ascii PROPERTIES TIMEOUT 60)

# Scale this directory's test timeouts under a sanitizer, and let a stuck test report
# itself before CTest kills it (see provizio_dds_finalize_tests in test/CMakeLists.txt).
provizio_dds_finalize_tests()
