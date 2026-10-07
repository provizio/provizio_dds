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
"""Coverage for setup.py's extract_bin_cache: what extracting a bin cache archive leaves in the build
directory -- the one directory the archive is to hold, and nothing else of it, however malformed --
with zipfile, as on Windows, and with unzip on a POSIX host that has one, as elsewhere; and for its
bin_cache_python_missing: what of its python/ directory a package needs, and lacks.

    extract_bin_cache_test.py <repository> <scratch dir>

setup.py runs its build when it is read, so the function alone is taken from its source.
"""

import ast
import fnmatch
import os
import shutil
import subprocess
import sys
import zipfile


def load_extract_bin_cache(repository, build_dir, platform):
    """Returns setup.py's extract_bin_cache, working in <build_dir> as on <platform>."""
    with open(os.path.join(repository, "setup.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "extract_bin_cache")
    namespace = {"os": os, "shutil": shutil, "subprocess": subprocess, "build_dir": build_dir,
                 "platform": platform}
    exec(compile(ast.Module(body=[function], type_ignores=[]), "setup.py", "exec"), namespace)
    return namespace["extract_bin_cache"]


def write_archive(path, entries, links=None):
    """Writes a zip archive at <path> holding <entries>, {name: content}, and <links>, {name: target},
    as symbolic links, as zip -y stores them."""
    with zipfile.ZipFile(path, "w") as zf:
        for name, content in entries.items():
            zf.writestr(name, content)
        for name, target in (links or {}).items():
            info = zipfile.ZipInfo(name)
            info.create_system = 3
            info.external_attr = 0o120777 << 16
            zf.writestr(info, target)


def held(directory):
    """Every file and link under <directory>, relative to it, with / separators, and every directory
    left of a staging one."""
    found = []
    for root, dirs, files in os.walk(directory):
        for name in files + [d for d in dirs if os.path.islink(os.path.join(root, d))]:
            found.append(os.path.relpath(os.path.join(root, name), directory).replace(os.sep, "/"))
        found += [d + "/" for d in dirs if root == directory and d.startswith("bin_cache_")]
    return sorted(found)


def stale_directory(build_dir, outside):
    """What an earlier extraction left of the directory, which must not survive this one."""
    os.makedirs(os.path.join(build_dir, "cache_key"))
    with open(os.path.join(build_dir, "cache_key", "stale.txt"), "w") as f:
        f.write("an earlier extraction's")


def check(case, platform, scratch, entries, expected_return, expected_held, links=None, before=stale_directory):
    build_dir = os.path.join(scratch, case + "_" + platform, "build")
    os.makedirs(build_dir)
    # A directory outside the build directory, which no link may lead the extraction into
    outside = os.path.join(scratch, case + "_" + platform, "outside")
    os.makedirs(outside)
    with open(os.path.join(outside, "kept.txt"), "w") as f:
        f.write("outside")
    archive = os.path.join(scratch, case + "_" + platform, "cache.zip")
    if entries is None:
        with open(archive, "wb") as f:
            f.write(b"not a zip archive")
    else:
        write_archive(archive, entries, {name: outside for name in (links or [])})
    before(build_dir, outside)
    returned = load_extract_bin_cache(sys.argv[1], build_dir, platform)(archive, "cache_key")
    expected = os.path.join(build_dir, "cache_key") if expected_return else ""
    if returned != expected:
        sys.exit(f"extract_bin_cache_test ({case}, {platform}): returned [{returned}], not [{expected}]")
    if held(build_dir) != sorted(expected_held):
        sys.exit(f"extract_bin_cache_test ({case}, {platform}): the build directory holds "
                 f"{held(build_dir)}, not {sorted(expected_held)}")
    if held(outside) != ["kept.txt"]:
        sys.exit(f"extract_bin_cache_test ({case}, {platform}): the directory outside holds {held(outside)}")


def check_python_missing(repository, scratch):
    """bin_cache_python_missing on python/ directories as each platform's caches hold them."""
    with open(os.path.join(repository, "setup.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    nodes = [node for node in tree.body
             if isinstance(node, ast.FunctionDef) and node.name == "bin_cache_python_missing"
             or isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "BIN_CACHE_PYTHON_REQUIRED"
                                                     for t in node.targets)]
    namespace = {"os": os}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "setup.py", "exec"), namespace)
    missing = namespace["bin_cache_python_missing"]
    layouts = {
        "linux": ["version.txt", "fastdds/fastdds.py", "fastdds/_fastdds_python.so", "provizio_dds/provizio_dds.py",
                  "provizio_dds/libprovizio_dds.so", "provizio_dds/libfastdds.so.3.6", "provizio_dds/libfastcdr.so.2",
                  "provizio_dds_python_types/provizio_dds_python_types.py",
                  "provizio_dds_python_types/_provizio_dds_python_types.so",
                  "provizio_dds_python_types/libprovizio_dds_types.so"],
        "windows": ["version.txt", "fastdds/fastdds.py", "fastdds/_fastdds_python.pyd", "provizio_dds/provizio_dds.py",
                    "provizio_dds/provizio_dds.dll", "provizio_dds/fastdds-3.6.dll", "provizio_dds/fastcdr-2.3.dll",
                    "provizio_dds_python_types/provizio_dds_python_types.py",
                    "provizio_dds_python_types/_provizio_dds_python_types.pyd",
                    "provizio_dds_python_types/provizio_dds_types.dll"],
    }
    required = namespace["BIN_CACHE_PYTHON_REQUIRED"]
    for layout, files in layouts.items():
        # Each file left out in turn is said to be missing, by the first name it can have; none, and
        # nothing is
        for left_out in [None] + files:
            case = str(files.index(left_out)) if left_out else "all"
            python_dir = os.path.join(scratch, "python_missing", layout, case)
            for name in files:
                if name != left_out:
                    os.makedirs(os.path.dirname(os.path.join(python_dir, name)), exist_ok=True)
                    with open(os.path.join(python_dir, name), "w") as f:
                        f.write("")
            expected = next((group[0] for group in required
                             if any(fnmatch.fnmatchcase(left_out, name) for name in group)), None) if left_out else ""
            if expected is None:
                sys.exit(f"extract_bin_cache_test (python_missing, {layout}): {left_out} is not required")
            said = missing(python_dir)
            if said != expected:
                sys.exit(f"extract_bin_cache_test (python_missing, {layout}, {left_out}): said [{said}] is "
                         f"missing, not [{expected}]")

    # A versioned name is a pattern, which only a file matching it satisfies -- not the unversioned
    # link of a development install, nor a directory -- and the cache's own path is taken as it is,
    # though it holds glob characters
    python_dir = os.path.join(scratch, "python_missing", "patterns [x]")
    for name in layouts["linux"] + ["provizio_dds/libfastdds.so"]:
        if name != "provizio_dds/libfastdds.so.3.6":
            os.makedirs(os.path.dirname(os.path.join(python_dir, name)), exist_ok=True)
            with open(os.path.join(python_dir, name), "w") as f:
                f.write("")
    os.makedirs(os.path.join(python_dir, "provizio_dds", "libfastdds.so.3.6.2.0"))
    said = missing(python_dir)
    if said != "provizio_dds/libfastdds.so.*":
        sys.exit(f"extract_bin_cache_test (python_missing, patterns): said [{said}] is missing, not "
                 "[provizio_dds/libfastdds.so.*]")
    with open(os.path.join(python_dir, "provizio_dds", "libfastdds.so.3.6"), "w") as f:
        f.write("")
    said = missing(python_dir)
    if said:
        sys.exit(f"extract_bin_cache_test (python_missing, patterns): said [{said}] is missing, of a complete "
                 "directory whose path holds glob characters")

    # Where links can be made: the libraries of a directory beyond the cache, that a directory of the
    # cache is a link to, stand for none of its own
    if os.name == "posix":
        foreign = os.path.join(scratch, "python_missing", "foreign")
        os.rename(os.path.join(python_dir, "provizio_dds"), foreign)
        os.symlink(foreign, os.path.join(python_dir, "provizio_dds"))
        said = missing(python_dir)
        if said != "provizio_dds/provizio_dds.py":
            sys.exit(f"extract_bin_cache_test (python_missing, link beyond): said [{said}] is missing, not "
                     "[provizio_dds/provizio_dds.py]")


def main():
    scratch = sys.argv[2]
    shutil.rmtree(scratch, ignore_errors=True)
    os.makedirs(scratch)
    # unzip as setup.py runs it, on a POSIX host only: it takes zipfile on Windows, where an unzip on
    # the search path (Git's) restores no link as a link
    platforms = ["win32"]
    if shutil.which("unzip") and os.name == "posix":
        platforms.append("linux")
    for platform in platforms:
        # A well-formed archive: its directory, as it holds it
        check("well_formed", platform, scratch, {"cache_key/python/module.py": "x", "cache_key/lib/a.so": "y"},
              True, ["cache_key/python/module.py", "cache_key/lib/a.so"])
        # Entries beside its directory: none of them lands in the build directory
        check("extra_entries", platform, scratch,
              {"cache_key/python/module.py": "x", "CMakeCache.txt": "stale", "packages/provizio_dds.py": "z"},
              True, ["cache_key/python/module.py"])
        # No directory of the name: nothing of it at all
        check("no_directory", platform, scratch, {"CMakeCache.txt": "stale", "other/file": "z"}, False, [])
        # Not an archive: nothing
        check("damaged", platform, scratch, None, False, [])
        # A link left where the directory goes, leading outside: removed itself, never followed
        def link_left(build_dir, outside):
            os.symlink(outside, os.path.join(build_dir, "cache_key"), target_is_directory=True)
        if os.name == "posix":
            check("link_left", platform, scratch, {"cache_key/python/module.py": "x"}, True,
                  ["cache_key/python/module.py"], before=link_left)
        if platform != "win32":
            # Its directory a link in the archive, as unzip restores it: nothing taken from there
            check("directory_link", platform, scratch, {}, False, [], links=["cache_key"])
        if os.name == "posix" and os.geteuid() != 0:
            # What an earlier extraction left that cannot be removed: nothing extracted over it
            def locked_left(build_dir, outside):
                locked = os.path.join(build_dir, "cache_key", "locked")
                os.makedirs(locked)
                with open(os.path.join(locked, "old.txt"), "w") as f:
                    f.write("old")
                os.chmod(locked, 0o500)
            check("locked_left", platform, scratch, {"cache_key/python/module.py": "x"}, False,
                  ["cache_key/locked/old.txt"], before=locked_left)
            os.chmod(os.path.join(scratch, "locked_left_" + platform, "build", "cache_key", "locked"), 0o700)
    check_python_missing(sys.argv[1], scratch)
    shutil.rmtree(scratch, ignore_errors=True)
    print("extract_bin_cache: the build directory holds the archive's own directory, and nothing else of it")


if __name__ == "__main__":
    main()
