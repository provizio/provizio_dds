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
    import re
    module_dir = os.path.dirname(__file__)

    def _highest_symbol_versions(library_path, tags):
        """Highest ``<tag>_x.y.z`` this ELF DEFINES per tag, as comparable tuples.

        Read out of the file rather than by loading it, which is the whole point: deciding
        whether to load a library must not require having loaded it. Symbol version names sit
        in .dynstr as plain ASCII, and neither libstdc++ nor libgcc requires a GLIBCXX_ / CXXABI_
        / GCC_ version of anything else, so every such string in one of them is one it provides.

        Every tag is answered from ONE read. These libraries are multi-megabyte, this runs on
        every Linux import of the package, and asking per tag read each of them once per tag.

        @param library_path The ELF file to scan
        @param tags The version-name prefixes to look for, e.g. ``("GLIBCXX", "CXXABI")``
        @return ``{tag: highest version tuple}``, with a tag absent when the file defines none
                of its versions or could not be read at all
        """
        try:
            with open(library_path, "rb") as library:
                contents = library.read()
        except OSError:
            return {}
        highest = {}
        for tag in tags:
            found = re.findall((tag + r"_(\d+(?:\.\d+)+)\x00").encode(), contents)
            if found:
                highest[tag] = max(
                    tuple(int(part) for part in v.decode().split(".")) for v in found
                )
        return highest

    def _prefer_system_stdlib():
        """Bind the host's libstdc++ / libgcc BEFORE anything reaches for the bundled copies,
        but only where the host's are at least as capable.

        These wheels carry their own libstdc++.so.6 and libgcc_s.so.1 so that a host whose own
        are too old for the prebuilt binaries can still run them. The cost, unmanaged, is that
        the bundled ones ALWAYS win: every shipped object has $ORIGIN first on its RUNPATH, so
        the copy sitting beside them is found ahead of the system's however new the system's is.
        The first libstdc++.so.6 loaded into a process is the one the whole process gets, so
        importing this package used to downgrade the interpreter's C++ runtime -- and any
        extension imported AFTER it that needed a newer GLIBCXX (a manylinux wheel built with a
        newer GCC: torch, recent opencv-python) then failed to load, with an error naming a
        symbol version rather than anything that would point back here. Import order decided
        whether an application worked.

        So: where the host's runtime provides at least what the bundled one does, load the
        host's first, by explicit path, and the bundled copies are simply never opened. Where it
        does not -- older, or absent -- do nothing at all, and the bundled copies serve exactly
        as they always have. Either way nothing is warned about, because in neither case is
        anything wrong.
        """
        # Only Linux ships these; macOS wheels carry no libstdc++ (libc++ is the system runtime)
        # and Windows is excluded above.
        if platform != "linux":
            return
        # Someone has already bound one -- another extension imported before us, or a re-import.
        # The process is committed either way and loading a second copy could only confuse it.
        try:
            with open("/proc/self/maps", "r", encoding="utf-8") as maps:
                if "libstdc++.so.6" in maps.read():
                    return
        except OSError:
            return

        # Nothing bundled means nothing to pre-empt: a from-source install resolves these
        # normally and this whole question does not arise.
        bundled = os.path.join(module_dir, "libstdc++.so.6")
        if not os.path.isfile(bundled):
            return

        def _elf_identity(library_path):
            """(ELF class, machine) of a file, or None if it is not a readable ELF.

            Byte 4 of e_ident is 32/64-bit and bytes 18-19 are the architecture. Both are at
            fixed offsets in every ELF, whatever its contents.
            """
            try:
                with open(library_path, "rb") as library:
                    header = library.read(20)
            except OSError:
                return None
            if len(header) < 20 or header[:4] != b"\x7fELF":
                return None
            return header[4], header[18:20]

        # The architecture the bundled copy was built for is, by construction, this package's.
        # Multiarch hosts carry libstdc++ for several: /usr/lib/i386-linux-gnu sorts before
        # /usr/lib/x86_64-linux-gnu, and taking the first match handed a 64-bit interpreter a
        # 32-bit library, whose load then failed and left the bundled copy to be found anyway.
        wanted_identity = _elf_identity(bundled)
        if wanted_identity is None:
            return

        def _first_existing(soname):
            """The host's ``soname``, searched through the standard glibc layouts in order:
            Debian/Ubuntu multiarch, then the lib64 distributions, then the flat one.

            One list for every library looked up here, because they must stay in step: a
            layout added for libstdc++ but not for libgcc_s would resolve the pair from two
            different places on that host, or leave the bundled libgcc_s beside a system
            libstdc++. Located by path rather than by SONAME on purpose -- resolving it the
            loader's way would mean loading it, which is the decision not yet made.

            (The same layouts are globbed by cmake/bin_cache/host_abi_compatibility.cmake,
            which asks a related question at configure time about a prebuilt cache. The two
            cannot share code -- one is CMake run from the source tree, the other Python run
            from an installed wheel that ships no CMake -- so a layout added here belongs
            there too.)
            """
            # Both flat layouts, /usr/lib and /lib, exactly as host_abi_compatibility.cmake's
            # list ends. Stopping at /usr/lib left the docstring's "then the flat one" half
            # implemented, and a host carrying libstdc++ only under a bare /lib unfound.
            for directory in ("/usr/lib/*-linux-gnu*", "/usr/lib64", "/lib/*-linux-gnu*",
                              "/lib64", "/usr/lib", "/lib"):
                for candidate in sorted(glob.glob(directory + "/" + soname)):
                    if os.path.isfile(candidate) and _elf_identity(candidate) == wanted_identity:
                        return candidate
            return None

        system = _first_existing("libstdc++.so.6")
        if not system:
            return  # Cannot see one to compare: leave the bundled copy to do its job.

        # "At least as capable" on both version namespaces libstdc++ carries. An older host on
        # either one keeps the bundled copy -- which is the case the bundling exists for.
        tags = ("GLIBCXX", "CXXABI")
        system_versions = _highest_symbol_versions(system, tags)
        bundled_versions = _highest_symbol_versions(bundled, tags)
        for tag in tags:
            system_version = system_versions.get(tag)
            bundled_version = bundled_versions.get(tag)
            if system_version is None or bundled_version is None or system_version < bundled_version:
                return

        # libgcc comes from the same place, since it is libstdc++'s own dependency and the two
        # are a matched pair from one toolchain -- so where BOTH copies exist, the host's is
        # preferred only if it too is at least as capable. Checked rather than assumed: a host
        # can carry a newer libstdc++ beside an older libgcc, and a shipped object needing a
        # GCC_x.y.z the host lacks then fails to load with a bare symbol-version error naming
        # nothing that points back here. Where either copy is missing there is nothing to
        # compare and nothing to get wrong: the bundled libgcc satisfies a newer libstdc++,
        # whose demands on it are stable.
        system_gcc = _first_existing("libgcc_s.so.1")
        bundled_gcc = os.path.join(module_dir, "libgcc_s.so.1")
        if system_gcc and os.path.isfile(bundled_gcc):
            system_gcc_version = _highest_symbol_versions(system_gcc, ("GCC",)).get("GCC")
            bundled_gcc_version = _highest_symbol_versions(bundled_gcc, ("GCC",)).get("GCC")
            if (
                system_gcc_version is None
                or bundled_gcc_version is None
                or system_gcc_version < bundled_gcc_version
            ):
                return

        # libstdc++ FIRST, and libgcc only once it has succeeded. The other order left a FAILED
        # libstdc++ load with the host's libgcc already pinned and the bundled libstdc++ still
        # to be found behind it -- the mismatched pair this whole function exists to prevent,
        # and the one state in which "nothing was changed" below would have been untrue.
        try:
            ctypes.CDLL(system, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            # It could not be loaded after all, and nothing has been loaded in its place: the
            # bundled copies are still there to be found the ordinary way, exactly as before
            # this ran.
            return
        if system_gcc:
            # Explicit, though loading libstdc++ by absolute path has already brought the
            # host's libgcc in through its own DT_NEEDED: it costs nothing and keeps the pair
            # coming from one place if that resolution ever changes.
            try:
                ctypes.CDLL(system_gcc, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass

    # Before the preloads below, whose DT_NEEDED libstdc++ is what would otherwise pull the
    # bundled copy in and fix it as the process's C++ runtime.
    try:
        _prefer_system_stdlib()
    except Exception:  # noqa: BLE001 -- an import must not fail over an optimisation
        pass
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
