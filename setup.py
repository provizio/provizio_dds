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
from sys import platform


class CMakeBuildError(Exception):
    """Raised when failed to build the CMake project"""

    pass


# Build the CMake project and copy its artifacts to the destination directory
source_dir = os.path.dirname(os.path.realpath(__file__))
build_dir = source_dir + "/build/python_packaging"
install_dir = build_dir + "/install"
target_dir = build_dir + "/packages"
os.makedirs(build_dir, exist_ok=True)
# TODO: Windows support
if os.path.isfile(f"{target_dir}/provizio_dds_python_types/libprovizio_dds_types.so"):
    print(f"Already built in {build_dir}, only packaging...", flush=True)
else:
    needs_building = True

    # Check if there is a prebuilt cache for our configuration
    if platform == "linux":
        bin_cache_config_name = (
            os.popen(source_dir + "/bin_cache_config_name.sh").read().strip()
        )
        cache_zip = source_dir + "/cache/" + bin_cache_config_name + ".zip"
        if os.path.isfile(cache_zip):
            if (
                os.system(
                    f'unzip -q "{cache_zip}" -d "{build_dir}" && mv -f "{build_dir}/{bin_cache_config_name}/python" "{target_dir}" && cp -f "{target_dir}/version.txt" "{build_dir}/"'
                )
                != 0
            ):
                raise Exception("Failed to extract/move bin cache!")
            print(f"Bin cache located and will be used: {bin_cache_config_name}")
            needs_building = False

    if needs_building:
        print("Building C++ libraries from source...", flush=True)
        if (
            os.system(
                f'cd "{build_dir}" && cmake -G Ninja "-DCMAKE_BUILD_TYPE=Release" "-DPYTHON_BINDINGS=ON" "-DENABLE_CHECK_FORMAT=OFF" "-DENABLE_TESTS=OFF" "-DCMAKE_INSTALL_PREFIX={install_dir}" "-DPYTHON_PACKAGES_INSTALL_DIR={target_dir}" "{source_dir}" && cmake --build . --target install -- -j8'
            )
            != 0
        ):
            raise CMakeBuildError()

# Read README.md text
with open(source_dir + "/README.md", "r") as readme_file:
    readme = readme_file.read()

# Read Version
with open(build_dir + "/version.txt", "r") as version_file:
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
        "OPERATING SYSTEM :: MACOS :: MACOS X",
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
    package_data={"": ["*.so*", "*.dll", "*.dylib"]},
)
