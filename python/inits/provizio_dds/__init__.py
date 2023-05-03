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

from sys import platform

if platform != "win32":
    extension = "dylib" if platform == "darwin" else "so"

    # Load libraries in the same folder, as Python itself won't look in it
    import ctypes
    import os
    module_dir = os.path.dirname(__file__)
    if os.path.isfile(module_dir + "/libfastcdr." + extension):
        ctypes.cdll.LoadLibrary(module_dir + "/libfastcdr." + extension)
    if os.path.isfile(module_dir + "/libfastrtps." + extension):
        ctypes.cdll.LoadLibrary(module_dir + "/libfastrtps." + extension)

from provizio_dds.provizio_dds import *
