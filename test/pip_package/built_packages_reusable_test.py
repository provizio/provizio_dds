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
"""Coverage for setup.py's built_packages_reusable: packages left in its packages directory are
packaged as they are only where setup.py recorded building them for the configuration at hand --
these sources, their revision, this Python and the environment the build reads, CMAKE_ARGUMENTS
among it -- and built again otherwise: of sources since edited, of another version (a tag added),
left by an older setup.py, which recorded nothing, built with other CMAKE_ARGUMENTS (a system
Fast-DDS, say) or another compiler, or for another Python (another version, ABI or platform) --
though not for a variable of the environment the build does not read, which pip's own runs of
setup.py differ by.

    built_packages_reusable_test.py <repository> <scratch dir>

setup.py runs its build when it is read, so the functions alone are taken from its source.
"""

import ast
import os
import platform as platform_module
import re
import shutil
import stat
import subprocess
import sys
import sysconfig


def load(repository):
    """Returns setup.py's pip_package_configuration and built_packages_reusable."""
    with open(os.path.join(repository, "setup.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    names = ("FINGERPRINTED_SOURCES", "source_fingerprint", "source_revision", "build_environment",
             "pip_package_configuration", "built_packages_reusable")
    functions = [node for node in tree.body
                 if isinstance(node, ast.FunctionDef) and node.name in names
                 or isinstance(node, ast.Assign) and any(getattr(t, "id", None) in names for t in node.targets)]
    namespace = {"os": os, "re": re, "sys": sys, "subprocess": subprocess}
    exec(compile(ast.Module(body=functions, type_ignores=[]), "setup.py", "exec"), namespace)
    return namespace["pip_package_configuration"], namespace["built_packages_reusable"]


def remove_tree(path):
    """Removes <path>, the read-only files git writes its objects as among it, which Windows refuses
    to delete as they are. A link is removed as it is, never followed."""
    for root, _, files in os.walk(path):
        for name in files:
            file = os.path.join(root, name)
            if not os.path.islink(file):
                os.chmod(file, stat.S_IREAD | stat.S_IWRITE)
    shutil.rmtree(path, ignore_errors=True)


def main():
    configuration, reusable = load(sys.argv[1])
    scratch = sys.argv[2]
    remove_tree(scratch)
    packages = os.path.join(scratch, "packages")
    record = os.path.join(scratch, "pip_package_built_for.txt")
    # The sources, as a checkout holds them
    source = os.path.join(scratch, "source")
    for name, content in (("CMakeLists.txt", "project(x)"), ("src/library.cpp", "int f();"),
                          ("python/provizio_dds.py", "x = 1")):
        os.makedirs(os.path.dirname(os.path.join(source, name)), exist_ok=True)
        with open(os.path.join(source, name), "w") as f:
            f.write(content)
    os.environ.pop("CMAKE_ARGUMENTS", None)

    def configuration_now():
        return configuration(source)

    def expect(case, expected):
        if reusable(packages, record, source) != expected:
            sys.exit(f"built_packages_reusable_test ({case}): reusable is not {expected}")

    # Nothing built: nothing to reuse
    expect("nothing_built", False)
    # Built, but recorded for nothing, as an older setup.py left it
    os.makedirs(os.path.join(packages, "provizio_dds_python_types"))
    with open(os.path.join(packages, "provizio_dds_python_types", "libprovizio_dds_types.so"), "w") as f:
        f.write("")
    expect("no_record", False)
    # Recorded for this configuration: reused
    with open(record, "w", encoding="utf-8") as f:
        f.write(configuration_now())
    expect("this_configuration", True)
    # Recorded for other CMAKE_ARGUMENTS: built again
    os.environ["CMAKE_ARGUMENTS"] = "-DLOOK_FOR_FAST_DDS=TRUE"
    expect("other_arguments", False)
    with open(record, "w", encoding="utf-8") as f:
        f.write(configuration_now())
    expect("those_arguments", True)
    os.environ.pop("CMAKE_ARGUMENTS")
    expect("arguments_gone", False)
    # Built with another compiler, or another package root: built again; but not for a variable the
    # build does not read, as pip's runs of setup.py for one install differ by some
    with open(record, "w", encoding="utf-8") as f:
        f.write(configuration_now())
    for variable, value in (("CC", "/opt/other/cc"), ("OPENSSL_ROOT_DIR", "/opt/openssl"), ("ZLIB_ROOT", "/opt/z")):
        # As the build had it, which a CI job may set itself
        before = os.environ.get(variable)
        os.environ[variable] = value
        expect(f"{variable}_set", False)
        if before is None:
            os.environ.pop(variable)
        else:
            os.environ[variable] = before
        expect(f"{variable}_restored", True)
    os.environ["PIP_BUILD_TRACKER"] = os.path.join(scratch, "pip-build-tracker-random")
    os.environ["PATH"] = os.path.join(scratch, "pip-build-env-random") + os.pathsep + os.environ.get("PATH", "")
    expect("unread_variables", True)
    # Recorded for another Python: built again
    with open(record, "w", encoding="utf-8") as f:
        f.write(configuration_now().replace(f"Python {sys.version_info.major}.{sys.version_info.minor}", "Python 2.7"))
    expect("other_python", False)
    # Recorded for another build of this Python (a free-threaded one), another platform, or another
    # machine (a universal2 one under Rosetta): built again
    suffix, platform = sysconfig.get_config_var("EXT_SUFFIX"), sysconfig.get_platform()
    machine = platform_module.machine()
    for case, other in (("other_abi", configuration_now().replace(f"({suffix} on", "(.cpython-399t-other.so on")),
                        ("other_platform", configuration_now().replace(f" on {platform},", " on other-platform,")),
                        ("other_machine", configuration_now().replace(f", {machine})", ", other-machine)"))):
        if other == configuration_now():
            sys.exit(f"built_packages_reusable_test ({case}): the configuration names no {case[6:]} to change")
        with open(record, "w", encoding="utf-8") as f:
            f.write(other)
        expect(case, False)
    # The sources edited since: built again
    with open(record, "w", encoding="utf-8") as f:
        f.write(configuration_now())
    expect("before_the_edit", True)
    with open(os.path.join(source, "src", "library.cpp"), "w") as f:
        f.write("int f(int);")
    expect("sources_edited", False)
    # ...but not Python's caches, which running it writes among them
    with open(record, "w", encoding="utf-8") as f:
        f.write(configuration_now())
    os.makedirs(os.path.join(source, "python", "__pycache__"))
    with open(os.path.join(source, "python", "__pycache__", "provizio_dds.cpython-312.pyc"), "wb") as f:
        f.write(b"cache")
    expect("python_caches", True)
    # A tag added to the commit the sources are, which the version is read from: built again; and so
    # with a commit of the same files, where there is a git to make one with
    if shutil.which("git"):
        # Run from a git hook, the test would be given the variables locating that hook's repository
        # (GIT_DIR, GIT_INDEX_FILE...), and commit into that one
        for variable in subprocess.check_output(["git", "rev-parse", "--local-env-vars"],
                                                universal_newlines=True).split():
            os.environ.pop(variable, None)
        no_hooks = os.path.join(scratch, "no_hooks")
        os.makedirs(no_hooks)

        def git(*arguments):
            # Whatever the user's own configuration asks for (signed tags and commits, hooks of theirs),
            # which these throwaway ones could not satisfy or should not run
            settings = ("user.name=test", "user.email=test@example.com", "commit.gpgSign=false", "tag.gpgSign=false",
                        f"core.hooksPath={no_hooks}")
            run = subprocess.run(["git", *(f for s in settings for f in ("-c", s)), *arguments], cwd=source,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            if run.returncode != 0:
                sys.exit(f"built_packages_reusable_test: git {' '.join(arguments)} failed:\n{run.stdout}")
        git("init", "-q")
        git("add", "-A")
        git("commit", "-q", "-m", "sources")
        with open(record, "w", encoding="utf-8") as f:
            f.write(configuration_now())
        expect("before_the_tag", True)
        git("tag", "9.9.9")
        expect("tag_added", False)
        with open(record, "w", encoding="utf-8") as f:
            f.write(configuration_now())
        git("commit", "-q", "--allow-empty", "-m", "the same files")
        expect("same_files_committed", False)
    # A Windows build's library, recorded for this configuration: reused
    os.remove(os.path.join(packages, "provizio_dds_python_types", "libprovizio_dds_types.so"))
    with open(os.path.join(packages, "provizio_dds_python_types", "provizio_dds_types.dll"), "w") as f:
        f.write("")
    with open(record, "w", encoding="utf-8") as f:
        f.write(configuration_now())
    expect("dll", True)

    remove_tree(scratch)
    print("built_packages_reusable: only packages built for the configuration at hand are packaged as they are")


if __name__ == "__main__":
    main()
