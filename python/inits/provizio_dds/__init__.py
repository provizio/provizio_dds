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

    # Preload the Fast-CDR + Fast-DDS shared objects bundled inside this
    # package so the dynamic loader can resolve them when the sibling
    # `fastdds._fastdds_python` extension is imported below. Without
    # this, `_fastdds_python.so`'s DT_NEEDED entry for libfastdds.so.X.Y
    # isn't satisfied — the file lives here in `provizio_dds/`, not in
    # `fastdds/` alongside the .so. `ctypes.cdll.LoadLibrary` brings
    # them into the process global symbol set so subsequent `dlopen()`s
    # by the loader find them by SONAME. Order matters: fastcdr is a
    # dependency of fastdds, so load it first. Fast-DDS 3.x renamed
    # libfastrtps → libfastdds; the legacy name is kept here as a
    # fallback so this preload keeps working if the file is backported
    # against an older Fast-DDS build.
    #
    # The default install (and the pip-packaged wheel) ships the
    # unversioned `lib*.so` / `lib*.dylib` symlink, so checking the bare
    # filename suffices in the common case. Builds that opt into
    # INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=ON strip those symlinks
    # and leave only the `lib*.so.MAJOR.MINOR.PATCH` form on disk; we
    # fall back to a glob for that case so the preload still finds the
    # shared object.
    import ctypes
    import glob
    import os
    module_dir = os.path.dirname(__file__)
    for libname in ("libfastcdr", "libfastdds", "libfastrtps"):
        unversioned = os.path.join(module_dir, libname + "." + extension)
        if os.path.isfile(unversioned):
            ctypes.cdll.LoadLibrary(unversioned)
            continue
        # INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=ON (Linux only)
        # strips the unversioned .so symlink, leaving only the fully-
        # qualified `.so.MAJOR.MINOR.PATCH` filename. Pick whichever
        # versioned file is present; dlopen by SONAME still resolves to
        # this object once it is in the process address space.
        versioned = glob.glob(
            os.path.join(module_dir, libname + "." + extension + ".*")
        )
        if versioned:
            # Sort by numeric version components, not lexicographic — so
            # `lib*.so.3.6.10` sorts after `lib*.so.3.6.9`. Non-numeric
            # suffix components (none expected in practice — would mean
            # something like a "debug" suffix) sort to the *front* so
            # `versioned[-1]` always lands on the highest numeric build
            # even if such an unexpected variant slips into the install.
            def _version_key(path):
                suffix = path.rsplit("." + extension + ".", 1)[-1]
                parts = []
                for part in suffix.split("."):
                    parts.append((1, int(part)) if part.isdigit() else (0, part))
                return parts
            versioned.sort(key=_version_key)
            ctypes.cdll.LoadLibrary(versioned[-1])

from provizio_dds.provizio_dds import *
