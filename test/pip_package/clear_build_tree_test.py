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

"""Coverage for setup.py's clear_build_tree: what is left of a CMake build tree built for another
configuration -- its cache, the toolchain and package results in it, the ExternalProject checkouts with the
read-only files git writes -- is gone, and the directory itself is there, empty, for the next configure.

usage:
    clear_build_tree_test.py <repository> <scratch dir>
"""

import ast
import os
import shutil
import stat
import sys


def load(repository):
    """Returns setup.py's clear_build_tree."""
    with open(os.path.join(repository, "setup.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "clear_build_tree"]
    if not functions:
        sys.exit("clear_build_tree_test: setup.py has no clear_build_tree")
    namespace = {"os": os, "shutil": shutil, "sys": sys}
    exec(compile(ast.Module(body=functions, type_ignores=[]), "setup.py", "exec"), namespace)
    return namespace["clear_build_tree"]


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: clear_build_tree_test.py <repository> <scratch dir>")
    repository, scratch = sys.argv[1], sys.argv[2]
    clear_build_tree = load(repository)

    build_dir = os.path.join(scratch, "python_packaging")
    shutil.rmtree(scratch, ignore_errors=True)
    os.makedirs(os.path.join(build_dir, "fast_dds", ".git", "objects", "pack"))
    os.makedirs(os.path.join(build_dir, "packages"))
    for name in ("CMakeCache.txt", "pip_package_built_for.txt", "version.txt",
                 os.path.join("fast_dds", ".git", "objects", "pack", "pack-1.idx"),
                 os.path.join("packages", "marker")):
        with open(os.path.join(build_dir, name), "w", encoding="utf-8") as f:
            f.write("stale\n")
    # What git writes its objects as, which Windows refuses to delete as they are
    read_only = os.path.join(build_dir, "fast_dds", ".git", "objects", "pack", "pack-1.idx")
    os.chmod(read_only, stat.S_IREAD)
    # A link is removed as it is, never followed into what it leads to
    outside = os.path.join(scratch, "outside")
    os.makedirs(outside)
    with open(os.path.join(outside, "kept"), "w", encoding="utf-8") as f:
        f.write("not part of the tree\n")
    try:
        os.symlink(outside, os.path.join(build_dir, "link_to_outside"))
    except (OSError, NotImplementedError):
        pass  # no symbolic links here (Windows without the privilege)

    clear_build_tree(build_dir)

    if not os.path.isdir(build_dir):
        sys.exit("clear_build_tree_test: the directory is not there after clearing")
    left = os.listdir(build_dir)
    if left:
        sys.exit(f"clear_build_tree_test: left behind: {sorted(left)}")
    if not os.path.isfile(os.path.join(outside, "kept")):
        sys.exit("clear_build_tree_test: a link in the tree was followed into what it leads to")

    print("clear_build_tree: a stale CMake tree is removed whole, the read-only files and links among it included")


if __name__ == "__main__":
    main()
