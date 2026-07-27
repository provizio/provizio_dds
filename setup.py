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
        return None

    libstdcxx, glibcxx, cxxabi = host_libstdcxx_versions()
    required = f"GLIBCXX_{required_glibcxx} / CXXABI_{required_cxxabi}"
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

# Build the CMake project and copy its artifacts to the destination directory
source_dir = os.path.dirname(os.path.realpath(__file__))
build_dir = source_dir + "/build/python_packaging"
install_dir = build_dir + "/install"
target_dir = build_dir + "/packages"
os.makedirs(build_dir, exist_ok=True)
# Check for already-built libraries (platform-specific extension)
_lib_names = (
    f"{target_dir}/provizio_dds_python_types/libprovizio_dds_types.so",
    f"{target_dir}/provizio_dds_python_types/provizio_dds_types.dll",
    f"{target_dir}/provizio_dds_python_types/libprovizio_dds_types.dylib",
)
if any(os.path.isfile(lib) for lib in _lib_names):
    print(f"Already built in {build_dir}, only packaging...", flush=True)
else:
    needs_building = True
    cmake_arguments = os.environ.get("CMAKE_ARGUMENTS", "")

    # Check if there is a prebuilt cache for our configuration (unless custom cmake_arguments are required)

    if platform == "linux" and cmake_arguments == "":
        # On Linux, 3.8-3.13 share ABI (tag "3"), 3.14+ broke ABI (tag "3_14")
        python_abi_tag = "3_14" if sys.version_info >= (3, 14) else "3"
        python_cache_config_name = subprocess.check_output(
            [source_dir + "/bin_cache_config_name.sh", "", "", python_abi_tag],
            text=True
        ).strip()
        python_cache_zip = source_dir + "/cache/" + python_cache_config_name + ".zip"
        if os.path.isfile(python_cache_zip):
            if subprocess.call(["unzip", "-q", python_cache_zip, "-d", build_dir]) != 0:
                raise Exception("Failed to extract Python bin cache!")

            incompatibility = bin_cache_incompatibility(f"{build_dir}/{python_cache_config_name}")

            if incompatibility is None:
                extracted_python = os.path.join(build_dir, python_cache_config_name, "python")
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
                shutil.rmtree(f"{build_dir}/{python_cache_config_name}")
                needs_building = True

    elif platform == "win32" and cmake_arguments == "":
        # On Windows, .pyd files link against specific pythonXY.dll, so each version needs its own cache
        python_ver_tag = f"{sys.version_info.major}{sys.version_info.minor}"
        ps_script = os.path.join(source_dir, "bin_cache_config_name.ps1")
        try:
            python_cache_config_name = subprocess.check_output(
                ["powershell", "-ExecutionPolicy", "Bypass", "-File", ps_script,
                 "-PythonVersionTag", python_ver_tag],
                text=True, cwd=source_dir
            ).strip()
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            print(f"Warning: failed to resolve Windows cache name: {e}", flush=True)
            python_cache_config_name = ""

        if python_cache_config_name:
            python_cache_zip = os.path.join(source_dir, "cache", python_cache_config_name + ".zip")
            if os.path.isfile(python_cache_zip):
                import zipfile
                with zipfile.ZipFile(python_cache_zip, "r") as zf:
                    zf.extractall(build_dir)

                extracted_python = os.path.join(build_dir, python_cache_config_name, "python")
                if os.path.isdir(extracted_python):
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
                    shutil.rmtree(os.path.join(build_dir, python_cache_config_name), ignore_errors=True)

    if needs_building:
        print("Building C++ libraries from source...", flush=True)
        cmake_configure = [
            "cmake", "-G", "Ninja",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DPYTHON_BINDINGS=ON",
            "-DENABLE_CHECK_FORMAT=OFF",
            "-DENABLE_TESTS=OFF",
            "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON",
            "-DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=OFF",
            f"-DCMAKE_INSTALL_PREFIX={install_dir}",
            f"-DPYTHON_PACKAGES_INSTALL_DIR={target_dir}",
            f"-DPython3_EXECUTABLE={sys.executable}",
        ]
        if cmake_arguments:
            # cmake_arguments is a user-provided string — split into args
            import shlex
            cmake_configure.extend(shlex.split(cmake_arguments))
        cmake_configure.append(source_dir)

        cmake_build = ["cmake", "--build", ".", "--target", "install", "--", "-j8"]

        if (
            subprocess.call(cmake_configure, cwd=build_dir) != 0
            or subprocess.call(cmake_build, cwd=build_dir) != 0
        ):
            raise CMakeBuildError()

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
