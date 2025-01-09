# Copyright 2025 Provizio Ltd.
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

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git


class provizio_ddsRecipe(ConanFile):
    name = "provizio_dds"
    package_type = "library"

    # Optional metadata
    license = "Provizio 2025 All right reserved"
    author = "Ivan Fabek ivan@provizio.ai"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "Provizio DDS library"
    topics = ("Provizio", "DDS")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False], 
        "fPIC": [True, False],
        "dds_idls_version": ["ANY"],
        "python_bindings": [True, False]
    }
    default_options = {
        "shared": False, 
        "fPIC": True,
        "dds_idls_version": "master",
        "python_bindings": False
    }

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*", "include/*", "cmake/*", ".git/*"

    def set_version(self):
        git = Git(self)
        tag = git.run("describe --tags")
        # if self.version is already defined from CLI --version arg, it will
        # not load version.txt
        self.version = self.version or tag

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self)
    
    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.cache_variables["PROVIZIO_DDS_IDLS_VERSION"] = self.options.dds_idls_version
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.components["provizio_dds"].libs = ["provizio_dds"]
        self.cpp_info.components["provizio_dds_types"].libs = ["provizio_dds_types"]
        self.cpp_info.components["fastrtps"].libs = ["fastrtps"]
        self.cpp_info.components["fastcdr"].libs = ["fastcdr"]

