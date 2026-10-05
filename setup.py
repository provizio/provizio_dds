# Copyright 2023 Provizio Ltd.
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

from setuptools import setup
import ctypes
import os
import os.path
import re
import shutil
import subprocess
import sys
from sys import platform


class CMakeBuildError(Exception):
    """Raised when failed to build the CMake project"""

    pass


def version_tuple(version):
    """Turns a dotted numeric version into a tuple of ints for comparison."""
    match = re.search(r"(\d+(?:\.\d+)*)", version)
    if not match:
        raise ValueError(f"Invalid version format: {version}")
    return tuple(int(part) for part in match.group(1).split("."))


def cache_abi_requirements(cache_dir):
    """Reads the ABI levels the prebuilt binaries of a cache require of the host.

    Returns a {"glibc": ..., "glibcxx": ..., "cxxabi": ...} dict of version strings (missing keys
    for requirements the cache doesn't declare), or None if the cache doesn't record them at all -
    in which case its compatibility with this host can't be established.
    See cmake/bin_cache/host_abi_compatibility.cmake for what these mean and why they, rather than
    the kernel version of the machine that built the cache, are what decides usability.
    """
    requirements_file = os.path.join(cache_dir, "abi_requirements")
    if not os.path.isfile(requirements_file):
        return None

    requirements = {}
    with open(requirements_file, "r", encoding="utf-8") as file:
        for line in file:
            match = re.match(r"^(glibc|glibcxx|cxxabi)=(\d+(?:\.\d+)*)\s*$", line)
            if match:
                requirements[match.group(1)] = match.group(2)

    return requirements if "glibc" in requirements else None


def host_glibc_version():
    """Returns the host's glibc version, or None when there is no glibc / it can't be determined."""
    try:
        # Only glibc answers this configuration key
        confstr = os.confstr("CS_GNU_LIBC_VERSION")
    except (AttributeError, ValueError, OSError):
        confstr = None
    if confstr:
        match = re.search(r"glibc (\d+\.\d+(?:\.\d+)?)", confstr)
        if match:
            return match.group(1)

    # Aliased, as the module-level `from sys import platform` already took the plain name
    import platform as platform_module

    name, version = platform_module.libc_ver()
    return version if name == "glibc" and version else None


def host_libstdcxx_versions():
    """Returns (path, highest GLIBCXX_, highest CXXABI_) of the libstdc++ this interpreter loads.

    (None, None, None) when it can't be located or read. Loading it the same way the prebuilt
    extension modules will is the point: whichever libstdc++ ends up in this process (a system one,
    or one from a Conda / virtualenv prefix taking precedence) is the one that has to satisfy them.
    """
    try:
        ctypes.CDLL("libstdc++.so.6")
    except OSError:
        return None, None, None

    library_path = None
    try:
        with open("/proc/self/maps", "r", encoding="utf-8") as maps:
            for line in maps:
                match = re.search(r"(/\S*/libstdc\+\+\.so\.6[^\s]*)$", line.rstrip())
                if match:
                    library_path = match.group(1)
                    break
    except OSError:
        return None, None, None

    if not library_path or not os.path.isfile(library_path):
        return None, None, None

    # Symbol version names live in .dynstr as plain ASCII, and libstdc++ requires no GLIBCXX_ /
    # CXXABI_ version of anything else, so every such string in it is one it provides
    try:
        with open(library_path, "rb") as library:
            contents = library.read()
    except OSError:
        return None, None, None

    def highest(tag):
        versions = re.findall((tag + r"_(\d+(?:\.\d+)+)\x00").encode(), contents)
        if not versions:
            return None
        return max((version.decode() for version in versions), key=version_tuple)

    glibcxx = highest("GLIBCXX")
    cxxabi = highest("CXXABI")
    if not glibcxx or not cxxabi:
        return None, None, None

    return library_path, glibcxx, cxxabi


def bin_cache_incompatibility(cache_dir):
    """Returns None if this host can use the prebuilt binaries of a cache, or why it can't."""
    requirements = cache_abi_requirements(cache_dir)
    if requirements is None:
        return "they don't record the ABI level they require, so their compatibility with this host can't be established"

    required_glibc = requirements["glibc"]
    glibc = host_glibc_version()
    if not glibc:
        return f"they require glibc {required_glibc} and this host's glibc version couldn't be determined"
    if version_tuple(glibc) < version_tuple(required_glibc):
        return f"they require glibc {required_glibc} or newer, while this host provides glibc {glibc}"

    required_glibcxx = requirements.get("glibcxx")
    required_cxxabi = requirements.get("cxxabi")
    if not required_glibcxx and not required_cxxabi:
        # A C++ cache always requires versioned GLIBCXX_/CXXABI_ symbols, and the cache
        # builder hard-fails rather than record empty values — so a requirements file
        # declaring neither can only be a failed scan or a hand-edited file. Accepting it
        # would silently skip the libstdc++ gate below and fail at load time instead.
        return (
            "they declare no libstdc++ (GLIBCXX/CXXABI) requirement, "
            "so their compatibility with this host can't be established"
        )

    libstdcxx, glibcxx, cxxabi = host_libstdcxx_versions()
    # Name only the requirements actually recorded: a cache that pins one of the two
    # would otherwise be reported as needing "GLIBCXX_None", which reads like a bug in
    # the check rather than a property of the cache.
    required = " / ".join(
        part
        for part in (
            f"GLIBCXX_{required_glibcxx}" if required_glibcxx else None,
            f"CXXABI_{required_cxxabi}" if required_cxxabi else None,
        )
        if part
    )
    if not libstdcxx:
        return f"they require libstdc++ providing {required} and this host's libstdc++ couldn't be located"
    if (required_glibcxx and version_tuple(glibcxx) < version_tuple(required_glibcxx)) or (
        required_cxxabi and version_tuple(cxxabi) < version_tuple(required_cxxabi)
    ):
        return (
            f"they require libstdc++ providing {required}, while this host's {libstdcxx} "
            f"provides GLIBCXX_{glibcxx} / CXXABI_{cxxabi}"
        )

    return None


def resolve_bin_cache_name(command, **kwargs):
    """Returns the bin cache name the key script prints, or "" when there is none to use.

    None to use when the script cannot run or fails - it asks GitHub for the IDLs revision, so it
    fails on any machine without egress, and it refuses a host it does not support - which costs a
    Fast-DDS compile, where letting the error out of here would cost the whole install. None either
    when what it printed is not a plausible name.
    """
    try:
        name = subprocess.check_output(command, text=True, **kwargs).strip()
    except (subprocess.CalledProcessError, OSError) as e:
        print(f"Warning: failed to resolve the bin cache name: {e}", flush=True)
        return ""

    # The name is interpolated into paths that are extracted into and later removed, so check its
    # shape here rather than inheriting the guarantee from how the script builds it. Its parts are
    # a platform and architecture, two hashes and a build type, and nothing else belongs in it; nor
    # can it start with a dot, which is what keeps "." and ".." - a name that would make those paths
    # build_dir itself, or its parent - from passing for one.
    if name and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._]*", name):
        print(f"Warning: refusing an implausible bin cache name: {name!r}", flush=True)
        return ""
    return name


def extract_bin_cache(archive, name):
    """Extracts a bin cache archive into build_dir, and returns the directory it is to hold, or "".

    The archive is extracted into a staging directory of its own, and only <name> -- the one directory
    it is to hold -- is taken from there into build_dir: whatever else a malformed archive holds goes
    with the staging directory rather than land in build_dir, where a CMakeCache.txt or a package tree
    would be taken for this build's own, now and on every later install. Nothing is left of an earlier
    extraction of <name> first, which could be mistaken for part of this one. "" when the archive
    cannot be extracted -- unzip missing or failing, a damaged archive -- or holds no <name>, nothing
    of it left behind, as a build from source still serves: the same the CMake side does for an
    archive it cannot extract.
    """
    import tempfile
    import zipfile

    extracted = os.path.join(build_dir, name)
    # A link of that name is removed itself, never followed: rmtree refuses one without a word
    if os.path.islink(extracted):
        os.unlink(extracted)
    shutil.rmtree(extracted, ignore_errors=True)
    if os.path.lexists(extracted):
        # Over what is left, the extraction would land inside it, and the stale tree be taken for it
        print(f"Warning: could not remove {extracted}, left of an earlier extraction", flush=True)
        return ""
    staging = tempfile.mkdtemp(prefix="bin_cache_", dir=build_dir)
    try:
        try:
            if platform == "win32":
                with zipfile.ZipFile(archive, "r") as zf:
                    zf.extractall(staging)
            elif subprocess.call(["unzip", "-q", archive, "-d", staging]) != 0:
                raise OSError(f"unzip failed on {archive}")
        except (OSError, zipfile.BadZipFile) as e:
            print(f"Warning: failed to extract the bin cache {archive}: {e}", flush=True)
            return ""
        staged = os.path.join(staging, name)
        # A directory itself, not a link to one elsewhere, which would be taken for the archive's
        if os.path.islink(staged) or not os.path.isdir(staged):
            print(f"Warning: the bin cache {archive} holds no {name} directory", flush=True)
            return ""
        shutil.move(staged, extracted)
        return extracted
    finally:
        shutil.rmtree(staging, ignore_errors=True)


# What the python/ directory of a bin cache holds that a package needs: each a file, or one of
# several, as on Linux, macOS and Windows (see bin_cache_python_missing)
BIN_CACHE_PYTHON_REQUIRED = (
    ("version.txt",),
    ("fastdds/fastdds.py",),
    ("fastdds/_fastdds_python.so", "fastdds/_fastdds_python.pyd"),
    ("provizio_dds/provizio_dds.py",),
    ("provizio_dds/libprovizio_dds.so", "provizio_dds/libprovizio_dds.dylib", "provizio_dds/provizio_dds.dll"),
    # The Fast-DDS stack the libraries load, bundled beside them under its versioned names
    ("provizio_dds/libfastdds.so.*", "provizio_dds/libfastdds.*.dylib", "provizio_dds/fastdds*.dll"),
    ("provizio_dds/libfastcdr.so.*", "provizio_dds/libfastcdr.*.dylib", "provizio_dds/fastcdr*.dll"),
    ("provizio_dds_python_types/provizio_dds_python_types.py",),
    ("provizio_dds_python_types/_provizio_dds_python_types.so",
     "provizio_dds_python_types/_provizio_dds_python_types.pyd"),
    ("provizio_dds_python_types/libprovizio_dds_types.so", "provizio_dds_python_types/libprovizio_dds_types.dylib",
     "provizio_dds_python_types/provizio_dds_types.dll"),
)


def bin_cache_python_missing(python_dir):
    """The first of BIN_CACHE_PYTHON_REQUIRED the python/ directory of a bin cache lacks, or "".

    An archive published truncated or malformed can extract with a python/ directory all the same;
    packaged without its version, its bindings or the libraries they load, it makes a wheel that
    cannot be built, or one that installs and fails to import, where a build from source still serves.
    Each name may be a glob pattern, which any file it matches satisfies -- one the directory holds,
    not one a link in it leads to elsewhere.
    """
    import glob

    inside = os.path.join(os.path.realpath(python_dir), "")
    for alternatives in BIN_CACHE_PYTHON_REQUIRED:
        if not any(os.path.isfile(path) and os.path.realpath(path).startswith(inside) for name in alternatives
                   for path in glob.glob(os.path.join(glob.escape(python_dir), *name.split("/")))):
            return alternatives[0]
    return ""


# What a build of the packages is made from, under the source directory (see source_fingerprint)
FINGERPRINTED_SOURCES = ("CMakeLists.txt", "setup.py", "bin_cache_config_name.sh", "bin_cache_config_name.ps1",
                         "src", "include", "cmake", "python")


def source_fingerprint(source):
    """A hash of what a build of the packages is made from: every file of FINGERPRINTED_SOURCES under
    <source>, its path and its content -- but Python's own caches, which running it writes there."""
    import hashlib

    fingerprint = hashlib.sha256()
    for top in FINGERPRINTED_SOURCES:
        path = os.path.join(source, top)
        if os.path.isfile(path):
            files = [top]
        else:
            files = []
            for root, dirs, names in os.walk(path):
                dirs[:] = sorted(d for d in dirs if d != "__pycache__")
                files += [os.path.relpath(os.path.join(root, name), source) for name in names
                          if not name.endswith(".pyc")]
        for name in sorted(files):
            fingerprint.update(name.replace(os.sep, "/").encode("utf-8") + b"\0")
            with open(os.path.join(source, name), "rb") as source_file:
                fingerprint.update(hashlib.sha256(source_file.read()).digest())
    return fingerprint.hexdigest()


def source_revision(source):
    """What the version of a build is derived from, as the configure derives it: the commit checked out
    under <source> and the tags at it (git), or "" where there is no git or no checkout of one."""
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, stderr=subprocess.DEVNULL,
                                         universal_newlines=True).strip()
        tags = subprocess.check_output(["git", "tag", "--points-at", "HEAD"], cwd=source,
                                       stderr=subprocess.DEVNULL, universal_newlines=True).split()
    except (OSError, subprocess.CalledProcessError):
        return ""
    return f"{commit} {','.join(sorted(tags))}"


def build_environment():
    """The variables of the environment a build of the packages reads, as NAME=value, sorted: the
    compilers and their flags, CMake's own (CMAKE_ARGUMENTS among them, which this script hands to
    the configure), the roots of packages a lookup is given (<Package>_ROOT, OPENSSL_ROOT_DIR), the
    macOS deployment target, and IGNORE_BIN_CACHE. Never the search path, which pip's isolated build
    environment gives a directory of its own on every run."""
    names = re.compile(r"(CC|CXX|CFLAGS|CXXFLAGS|CPPFLAGS|LDFLAGS|MACOSX_DEPLOYMENT_TARGET|ARCHFLAGS|IGNORE_BIN_CACHE"
                       r"|CMAKE_\w+|\w+_ROOT|\w+_ROOT_DIR)")
    return sorted(f"{name}={value}" for name, value in os.environ.items() if names.fullmatch(name))


def pip_package_configuration(source):
    """What a build of the packages is for: the sources under <source> as they are, the revision their
    version is derived from, this Python -- its version, and the ABI and platform its extension modules
    are built for -- and the variables of this install's environment the build reads (see
    build_environment), CMAKE_ARGUMENTS among them.

    Recorded beside the packages once they are made, so that packages built for another -- of other
    sources (another commit checked out, a file edited), of another version (a tag added, a commit of
    the same files), an older setup.py's, which recorded nothing, one built against a system Fast-DDS
    (LOOK_FOR_FAST_DDS), with other CMAKE_ARGUMENTS, compilers or package roots, or for another
    Python (of another version, a debug or free-threaded build of the same one, or another
    architecture) -- are built again rather than packaged. What the system holds is not recorded: a
    library installed or upgraded on it since a build takes removing build/python_packaging.
    """
    import platform as platform_module
    import sysconfig

    # The suffix of an extension module names the interpreter ABI it is for (cpython-313t-..., _d...)
    # and, on Linux and Windows, the architecture. On macOS neither it nor the platform does for a
    # universal2 interpreter, which reads macosx-...-universal2 whether it runs natively or under
    # Rosetta, where the build is for x86_64: the machine the process runs as does.
    return (f"pip package of sources {source_fingerprint(source)}, revision [{source_revision(source)}], for Python "
            f"{sys.version_info.major}.{sys.version_info.minor} ({sysconfig.get_config_var('EXT_SUFFIX')} on "
            f"{sysconfig.get_platform()}, {platform_module.machine()}), "
            f"environment [{'; '.join(build_environment())}]")


def built_packages_reusable(packages_dir, record, source):
    """Whether packages_dir holds packages this script built, for the configuration at hand.

    The libraries built there, and the configuration recorded for them in <record> the same as
    pip_package_configuration(<source>): pip runs this script more than once for one install, and the
    packages the first run built serve the others.
    """
    names = ("libprovizio_dds_types.so", "provizio_dds_types.dll", "libprovizio_dds_types.dylib")
    if not any(os.path.isfile(os.path.join(packages_dir, "provizio_dds_python_types", name)) for name in names):
        return False
    try:
        with open(record, "r", encoding="utf-8") as record_file:
            return record_file.read() == pip_package_configuration(source)
    except OSError:
        return False


# Build the CMake project and copy its artifacts to the destination directory
source_dir = os.path.dirname(os.path.realpath(__file__))
build_dir = source_dir + "/build/python_packaging"
install_dir = build_dir + "/install"
target_dir = build_dir + "/packages"
os.makedirs(build_dir, exist_ok=True)
# What the packages in target_dir were built for (see pip_package_configuration)
built_for_record = os.path.join(build_dir, "pip_package_built_for.txt")
if built_packages_reusable(target_dir, built_for_record, source_dir):
    print(f"Already built in {build_dir}, only packaging...", flush=True)
else:
    # Nothing of packages built for another configuration, nor its record, is kept: none of it is
    # what this install packages
    if os.path.lexists(built_for_record):
        os.remove(built_for_record)
    # Nor the version an earlier build recorded, which a build that writes none would package
    if os.path.lexists(os.path.join(build_dir, "version.txt")):
        os.remove(os.path.join(build_dir, "version.txt"))
    if os.path.isdir(target_dir):
        print(f"{target_dir} was not built for this configuration: building it again", flush=True)
        shutil.rmtree(target_dir)
    needs_building = True
    cmake_arguments = os.environ.get("CMAKE_ARGUMENTS", "")

    # Check if there is a prebuilt cache for our configuration (unless custom cmake_arguments are required)

    if platform == "linux" and cmake_arguments == "":
        # On Linux, 3.8-3.13 share ABI (tag "3"), 3.14+ broke ABI (tag "3_14")
        python_abi_tag = "3_14" if sys.version_info >= (3, 14) else "3"
        python_cache_config_name = resolve_bin_cache_name(
            [source_dir + "/bin_cache_config_name.sh", "", "", python_abi_tag]
        )

        if python_cache_config_name:
            python_cache_zip = source_dir + "/cache/" + python_cache_config_name + ".zip"
            if os.path.isfile(python_cache_zip):
                extracted = extract_bin_cache(python_cache_zip, python_cache_config_name)

                incompatibility = (
                    bin_cache_incompatibility(extracted) if extracted else "it could not be extracted"
                )

                extracted_python = os.path.join(build_dir, python_cache_config_name, "python")
                missing = bin_cache_python_missing(extracted_python) if incompatibility is None else ""
                if incompatibility is None and not os.path.isdir(extracted_python):
                    # Extracted, and of an ABI this host provides, yet malformed or truncated: see the
                    # Windows branch below for why it is said rather than passed over
                    print(f"Bin cache {python_cache_config_name} carries no python directory: "
                          "building from source", flush=True)
                elif incompatibility is None and missing:
                    print(f"Bin cache {python_cache_config_name} lacks python/{missing}: building from source",
                          flush=True)
                elif incompatibility is None:
                    if os.path.isdir(target_dir):
                        shutil.rmtree(target_dir)
                    shutil.move(extracted_python, target_dir)
                    version_txt = os.path.join(target_dir, "version.txt")
                    if os.path.isfile(version_txt):
                        shutil.copy2(version_txt, build_dir)
                    print(f"Bin cache located and will be used: {python_cache_config_name}")
                    needs_building = False
                else:
                    print(f"Bin cache located, but won't be used as {incompatibility}")
                # What is left of the archive, used or not, an archive without its own directory included
                shutil.rmtree(os.path.join(build_dir, python_cache_config_name), ignore_errors=True)
            else:
                # Name the key that was looked for. A key naming no archive is otherwise
                # indistinguishable from a configuration for which no cache was ever
                # published, which is what let an architecture silently stop matching any.
                print(f"No bin cache for {python_cache_config_name}: building from source")

    elif platform == "win32" and cmake_arguments == "":
        # On Windows, .pyd files link against specific pythonXY.dll, so each version needs its own cache
        python_ver_tag = f"{sys.version_info.major}{sys.version_info.minor}"
        ps_script = os.path.join(source_dir, "bin_cache_config_name.ps1")
        python_cache_config_name = resolve_bin_cache_name(
            ["powershell", "-ExecutionPolicy", "Bypass", "-File", ps_script, "-PythonVersionTag", python_ver_tag],
            cwd=source_dir,
        )

        if python_cache_config_name:
            python_cache_zip = os.path.join(source_dir, "cache", python_cache_config_name + ".zip")
            if os.path.isfile(python_cache_zip):
                extracted = extract_bin_cache(python_cache_zip, python_cache_config_name)

                extracted_python = os.path.join(build_dir, python_cache_config_name, "python")
                missing = bin_cache_python_missing(extracted_python) if extracted else ""
                if not extracted:
                    print(f"Bin cache {python_cache_config_name} could not be extracted: building from source",
                          flush=True)
                elif os.path.isdir(extracted_python) and missing:
                    # Malformed or truncated: see below for why it is said rather than passed over
                    print(f"Bin cache {python_cache_config_name} lacks python/{missing}: building from source",
                          flush=True)
                    shutil.rmtree(os.path.join(build_dir, python_cache_config_name), ignore_errors=True)
                elif os.path.isdir(extracted_python):
                    if os.path.isdir(target_dir):
                        shutil.rmtree(target_dir)
                    shutil.move(extracted_python, target_dir)
                    # Copy version.txt to build_dir for later use
                    version_txt = os.path.join(target_dir, "version.txt")
                    if os.path.isfile(version_txt):
                        shutil.copy2(version_txt, build_dir)
                    shutil.rmtree(os.path.join(build_dir, python_cache_config_name), ignore_errors=True)
                    print(f"Bin cache located and will be used: {python_cache_config_name}")
                    needs_building = False
                else:
                    # The archive was published malformed or truncated: it extracted, but carries
                    # no python/ directory. Removing it without a word would make a packaging bug
                    # on the publishing side look exactly like no cache having been published.
                    print(f"Bin cache {python_cache_config_name} carries no python directory: "
                          "building from source", flush=True)
                    shutil.rmtree(os.path.join(build_dir, python_cache_config_name), ignore_errors=True)
            else:
                # See the Linux branch above for why a miss must name its key
                print(f"No bin cache for {python_cache_config_name}: building from source")

    if needs_building:
        print("Building C++ libraries from source...", flush=True)
        cmake_configure = [
            "cmake", "-G", "Ninja",
            "-DCMAKE_BUILD_TYPE=Release",
            # Given every time, and ahead of CMAKE_ARGUMENTS, whose value comes later and so wins:
            # build_dir is configured again by the next install, and would otherwise keep a value
            # an earlier one's CMAKE_ARGUMENTS gave it - LOOK_FOR_FAST_DDS, say, which a pip
            # package refuses, and would go on refusing after CMAKE_ARGUMENTS no longer asks for it.
            "-DLOOK_FOR_FAST_DDS=OFF",
            "-DENABLE_CHECK_FORMAT=OFF",
            "-DENABLE_TESTS=OFF",
            "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON",
            "-DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=OFF",
            f"-DPython3_EXECUTABLE={sys.executable}",
        ]
        # What the package is built as and packaged from, given after CMAKE_ARGUMENTS too, so that
        # its value wins over any there: a pip package is always provizio_dds's own Fast-DDS stack,
        # bundled whole, which PYTHON_PIP_PACKAGE has the configure insist on (refusing
        # LOOK_FOR_FAST_DDS among others), and it is packaged from where this script looks.
        enforced_arguments = {
            "PYTHON_BINDINGS": "ON",
            "PYTHON_PIP_PACKAGE": "ON",
            "CMAKE_INSTALL_PREFIX": install_dir,
            "PYTHON_PACKAGES_INSTALL_DIR": target_dir,
        }
        if cmake_arguments:
            # cmake_arguments is a user-provided string - split into args, shell-style. On Windows
            # its backslashes are path separators rather than escapes, so they are kept as they are.
            import shlex
            user_arguments = shlex.split(
                cmake_arguments.replace("\\", "\\\\") if platform == "win32" else cmake_arguments
            )
            for index, argument in enumerate(user_arguments):
                # -DNAME=value, -DNAME:TYPE=value, and the same as two arguments after a -D
                if argument == "-D" and index + 1 < len(user_arguments):
                    argument = "-D" + user_arguments[index + 1]
                name = re.match(r"-D([A-Za-z0-9_]+)(:[A-Za-z]+)?=", argument)
                if name and name.group(1) in enforced_arguments:
                    # As ASCII, as the user's value need not be, nor the console's code page take it
                    shown = argument.encode("ascii", "backslashreplace").decode("ascii")
                    print(f"Warning: CMAKE_ARGUMENTS sets {name.group(1)}, which a pip package sets itself: "
                          f"{shown} is overridden", flush=True)
            cmake_configure.extend(user_arguments)
        cmake_configure.extend(f"-D{name}={value}" for name, value in enforced_arguments.items())
        cmake_configure.append(source_dir)

        cmake_build = ["cmake", "--build", ".", "--target", "install", "--", "-j8"]

        if (
            subprocess.call(cmake_configure, cwd=build_dir) != 0
            or subprocess.call(cmake_build, cwd=build_dir) != 0
        ):
            raise CMakeBuildError()

    with open(built_for_record, "w", encoding="utf-8") as record_file:
        record_file.write(pip_package_configuration(source_dir))

# Read README.md text
with open(source_dir + "/README.md", "r", encoding="utf-8") as readme_file:
    readme = readme_file.read()

# Read Version
with open(build_dir + "/version.txt", "r", encoding="utf-8") as version_file:
    version = version_file.read().rstrip()

setup(
    name="provizio_dds",
    version=version,
    author="Provizio",
    author_email="support@provizio.ai",
    description="Library for DDS communication in Provizio customer facing APIs and internal Provizio software components",
    license="License :: OSI Approved :: Apache Software License",
    platforms=[
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS :: MacOS X",
        "Operating System :: Microsoft :: Windows",
    ],
    url="https://github.com/provizio/provizio_dds",
    long_description=readme,
    long_description_content_type="text/markdown",
    install_requires=[
        "numpy>=1.16",
        "transforms3d>=0.4.1",
        # Fast-DDS-Python 2.x's generated fastdds.py calls
        # win32api.LoadLibrary('fastdds-X.Y.dll') at import time on
        # Windows, so the wheel depends on pywin32 there. Linux/macOS
        # use the ctypes preload in provizio_dds/__init__.py instead.
        'pywin32; sys_platform == "win32"',
    ],
    packages=["fastdds", "provizio_dds_python_types", "provizio_dds"],
    package_dir={
        "fastdds": f"{target_dir}/fastdds",
        "provizio_dds_python_types": f"{target_dir}/provizio_dds_python_types",
        "provizio_dds": f"{target_dir}/provizio_dds",
    },
    package_data={"": ["*.so*", "*.dll", "*.pyd", "*.dylib"]},
)
