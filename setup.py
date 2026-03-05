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
import os
import os.path
import re
import shutil
import sys
from sys import platform


class CMakeBuildError(Exception):
    """Raised when failed to build the CMake project"""

    pass


def extract_version(version):
    """Extracts the version number (x.y.z) from a version string."""
    match = re.search(r"^(\d+\.\d+\.\d+)", version)
    if match:
        return list(map(int, match.group(1).split(".")))
    else:
        raise ValueError("Invalid version format")


def compare_versions(version1, version2):
    """Compares two versions based on the numerical part (x.y.z)."""
    v1 = extract_version(version1)
    v2 = extract_version(version2)

    for v1_part, v2_part in zip(v1, v2):
        if v1_part < v2_part:
            return -1  # version1 is older
        elif v1_part > v2_part:
            return 1  # version1 is newer

    return 0  # versions are equal

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
        python_cache_config_name = (
            os.popen(source_dir + "/bin_cache_config_name.sh '' '' " + python_abi_tag).read().strip()
        )
        python_cache_zip = source_dir + "/cache/" + python_cache_config_name + ".zip"
        if os.path.isfile(python_cache_zip):
            if (
                os.system(
                    f'unzip -q "{python_cache_zip}" -d "{build_dir}"'
                )
                != 0
            ):
                raise Exception("Failed to extract Python bin cache!")

            with open(f"{build_dir}/{python_cache_config_name}/kernel_version", "r") as kernel_version_file:
                cache_kernel_version = kernel_version_file.read().strip()
            host_kernel_version = os.popen("uname -r").read().strip()

            if compare_versions(host_kernel_version, cache_kernel_version) >= 0:
                if (
                    os.system(
                        f'mv -f "{build_dir}/{python_cache_config_name}/python" "{target_dir}" && cp -f "{target_dir}/version.txt" "{build_dir}/"'
                    )
                    != 0
                ):
                    raise Exception("Failed to move Python bin cache!")
                print(f"Bin cache located and will be used: {python_cache_config_name}")
                needs_building = False
            else:
                print(f"Bin cache located but built using newer Linux kernel")
                shutil.rmtree(f"{build_dir}/{python_cache_config_name}")
                needs_building = True

    elif platform == "win32" and cmake_arguments == "":
        # On Windows, .pyd files link against specific pythonXY.dll, so each version needs its own cache
        python_ver_tag = f"{sys.version_info.major}{sys.version_info.minor}"
        import subprocess
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
        if (
            os.system(
                f'cd "{build_dir}" && cmake -G Ninja "-DCMAKE_BUILD_TYPE=Release" "-DPYTHON_BINDINGS=ON" "-DENABLE_CHECK_FORMAT=OFF" "-DENABLE_TESTS=OFF" "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON" "-DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=OFF" "-DCMAKE_INSTALL_PREFIX={install_dir}" "-DPYTHON_PACKAGES_INSTALL_DIR={target_dir}" "-DPython3_EXECUTABLE={sys.executable}" {cmake_arguments} "{source_dir}" && cmake --build . --target install -- -j8'
            )
            != 0
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
    install_requires=["numpy>=1.16", "transforms3d>=0.4.1"],
    packages=["fastdds", "provizio_dds_python_types", "provizio_dds"],
    package_dir={
        "fastdds": f"{target_dir}/fastdds",
        "provizio_dds_python_types": f"{target_dir}/provizio_dds_python_types",
        "provizio_dds": f"{target_dir}/provizio_dds",
    },
    package_data={"": ["*.so*", "*.dll", "*.pyd", "*.dylib"]},
)
