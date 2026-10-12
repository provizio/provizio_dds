#!/usr/bin/env python3

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

"""Parity of what a Python bin cache must hold: setup.py's BIN_CACHE_PYTHON_REQUIRED (one entry per
file, each with its spelling on every platform) against the lists of cmake/bin_cache/python_required.cmake
(one per platform) that the top-level CMakeLists.txt checks a cache by.

The two checks guard the same thing -- a truncated or malformed cache must not be installed as a package
that cannot import -- for the pip package and for the CMake install, and a file added to one list and
forgotten in the other would leave one of them accepting what the other refuses.

usage: bin_cache_python_required_parity.py <repository>
"""

import ast
import os
import re
import sys


def setup_py_required(repository):
    """The alternatives tuples of BIN_CACHE_PYTHON_REQUIRED, read from setup.py without running it."""
    with open(os.path.join(repository, "setup.py"), encoding="utf-8") as source:
        tree = ast.parse(source.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(target, "id", None) == "BIN_CACHE_PYTHON_REQUIRED"
                                                for target in node.targets):
            return ast.literal_eval(node.value)
    sys.exit("bin_cache_python_required_parity: setup.py has no BIN_CACHE_PYTHON_REQUIRED")


def cmake_lists(repository):
    """The names of each set(PROVIZIO_DDS_BIN_CACHE_PYTHON_REQUIRED_<PLATFORM> ...) of the CMake file."""
    with open(os.path.join(repository, "cmake", "bin_cache", "python_required.cmake"), encoding="utf-8") as source:
        text = re.sub(r"#[^\n]*", "", source.read())
    lists = {}
    for name, body in re.findall(r"set\(\s*PROVIZIO_DDS_BIN_CACHE_PYTHON_REQUIRED_(\w+)\s+([^)]*)\)", text):
        names = [token.strip('"') for token in body.split()]
        if len(names) != len(set(names)):
            sys.exit(f"bin_cache_python_required_parity: {name} lists a file twice")
        lists[name] = set(names)
    return lists


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: bin_cache_python_required_parity.py <repository>")
    repository = sys.argv[1]
    required = setup_py_required(repository)
    lists = cmake_lists(repository)
    if set(lists) != {"LINUX", "WINDOWS"}:
        sys.exit(f"bin_cache_python_required_parity: expected a LINUX and a WINDOWS list, found {sorted(lists)}")

    def spelling(name):
        """Which platform a spelling is for: None for one every platform shares (a module, version.txt)."""
        if name.endswith((".pyd", ".dll")) or ".dll" in name:
            return "WINDOWS"
        if ".dylib" in name:
            return "MACOS"
        if name.endswith(".so") or ".so." in name:
            return "LINUX"
        return None

    expected = {"LINUX": set(), "WINDOWS": set()}
    for alternatives in required:
        for name in alternatives:
            platform = spelling(name)
            for target in expected:
                if platform in (None, target):
                    expected[target].add(name)

    failed = False
    for target in expected:
        missing = sorted(expected[target] - lists[target])
        extra = sorted(lists[target] - expected[target])
        if missing or extra:
            failed = True
            print(f"bin_cache_python_required_parity: the {target} list of cmake/bin_cache/python_required.cmake "
                  f"differs from setup.py's BIN_CACHE_PYTHON_REQUIRED: missing {missing}, not in setup.py {extra}")
    if failed:
        sys.exit(1)
    print("bin_cache_python_required_parity: setup.py and the CMake lists require the same Python files")


if __name__ == "__main__":
    main()
