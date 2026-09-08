#!/usr/bin/env python3

# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
"""Guards the bundled-vs-system C++ runtime preference in provizio_dds/__init__.py.

The wheels carry their own libstdc++.so.6 so a host too old for the prebuilt binaries can
still run them, and every shipped object has $ORIGIN first on its RUNPATH -- so without the
preference that bundled copy always wins, downgrading the whole interpreter's C++ runtime and
breaking any newer-toolchain extension imported afterwards.

What this file can check is the part that decides: the symbol-version reader, and that running
the preference is harmless and repeatable. It cannot check the preference actually taking
effect, because that needs a bundled libstdc++ beside the package and a source-tree layout has
none -- the version reader is where a silent misjudgement would hide, which is why it is what
is pinned here. (A reader that quietly answers None makes the preference a no-op and the
bundled copy wins again, with nothing logged; that is exactly the shape of the bug this test
would catch.)
"""

import sys

import provizio_dds

_failures = []


def _expect(condition, description):
    if not condition:
        _failures.append(description)
        print(f"FAIL: {description}")
    return condition


def main():
    """Check the reader and the no-op semantics of the runtime preference."""
    if sys.platform != "linux":
        print(f"stdlib_preference: SKIPPED (bundled C++ runtimes are a Linux concern, not {sys.platform})")
        return 0

    reader = getattr(provizio_dds, "_highest_symbol_versions", None)
    prefer = getattr(provizio_dds, "_prefer_system_stdlib", None)
    if reader is None or prefer is None:
        # A build whose __init__ took the Windows branch, or a layout without it. Nothing to
        # guard rather than a failure.
        print("stdlib_preference: SKIPPED (this package exposes no runtime preference)")
        return 0

    # Whatever libstdc++ this interpreter ended up with, it is an ELF that declares both
    # version namespaces -- so the reader must produce comparable tuples for it.
    mapped = [line.split()[-1] for line in open("/proc/self/maps", encoding="utf-8") if "libstdc++.so.6" in line]
    _expect(bool(mapped), "the interpreter has a libstdc++ mapped to read")
    if mapped:
        # Both tags come out of ONE read of the file -- the reader is asked for them together
        # precisely so a multi-megabyte .so is not read once per tag on every import.
        versions = reader(mapped[0], ("GLIBCXX", "CXXABI"))
        for tag in ("GLIBCXX", "CXXABI"):
            version = versions.get(tag)
            _expect(
                isinstance(version, tuple) and len(version) >= 2 and all(isinstance(p, int) for p in version),
                f"{tag} read from {mapped[0]} is a comparable tuple of ints, got {version!r}",
            )
        # Comparability is the property the decision rests on.
        _expect(versions.get("GLIBCXX", ()) >= (3, 4), "the GLIBCXX read is a sane version")

    # Anything that is not a readable ELF yields no version for the tag, which the caller
    # reads (via .get) as "cannot judge" and leaves the bundled copy alone -- never as
    # "version zero".
    _expect(
        reader("/nonexistent/libstdc++.so.6", ("GLIBCXX",)) == {},
        "a missing file yields no versions",
    )
    _expect(reader(__file__, ("GLIBCXX",)) == {}, "a non-ELF file yields no versions")

    # Idempotent and harmless: by now a libstdc++ is mapped, so this must take its early
    # return rather than loading a second copy of the C++ runtime into the process.
    before = len({line.split()[-1] for line in open("/proc/self/maps", encoding="utf-8") if "libstdc++" in line})
    prefer()
    after = len({line.split()[-1] for line in open("/proc/self/maps", encoding="utf-8") if "libstdc++" in line})
    _expect(before == after, f"re-running the preference loaded nothing new ({before} -> {after})")

    print(f"stdlib_preference: {'PASS' if not _failures else 'FAIL'} ({len(_failures)} failure(s))")
    return 0 if not _failures else 1


if __name__ == "__main__":
    sys.exit(main())
