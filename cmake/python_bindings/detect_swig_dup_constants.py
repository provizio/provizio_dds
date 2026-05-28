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

"""Detect cross-file constant collisions in Fast-DDS-Gen v4 output for SWIG.

Fast-DDS-Gen v4 emits a `<X>_Constants` C++ namespace inside each generated
`.hpp` for every constant block in the source `.msg` / `.srv`. ROS service
files conventionally re-declare msg-side constants in both the Request and
Response halves (e.g. provizio's `set_radar_range.srv` mirrors `radar_info.msg`'s
range constants). C++ keeps them distinct via namespace scoping, but SWIG's
Python target flattens namespaces into the module dictionary and errors with
`<NAME> is multiply defined in the generated target language module` when the
bundled `.i` files include both `.hpp` files.

This script scans every `.hpp` paired with one of the input `.i` files and
emits SWIG `%ignore` directives for the qualified duplicates so that only the
first occurrence — typically the msg-side declaration since msg/ sorts before
srv/ — reaches the Python module.

Usage:
    detect_swig_dup_constants.py --idls-root <DIR> --output <FILE> <.i files...>

Output is a sequence of `%ignore` lines suitable for prepending to the bundled
SWIG `.i` file. Exits 0 even when no duplicates are found.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


# Matches a `namespace <X>_Constants { ... }` block whose body holds only
# `const T NAME = VALUE;` declarations (no nested braces, which is true for
# fastddsgen-emitted constant blocks).
_NS_BLOCK_RE = re.compile(
    r"namespace\s+(?P<ns>[A-Za-z0-9_]+_Constants)\s*\{(?P<body>[^}]*)\}",
    re.DOTALL,
)

# Matches `const <type> <NAME> = <value>;` inside a namespace body. The type
# can be qualified (`std::uint32_t`, `uint8_t`, etc.).
_CONST_DECL_RE = re.compile(
    r"^\s*const\s+[A-Za-z0-9_:]+\s+(?P<name>[A-Z_][A-Z0-9_]*)\s*=",
    re.MULTILINE,
)


def _pkg_kind_from_path(idl_path: Path, idls_root: Path) -> tuple[str, str] | None:
    """Return ``(pkg, kind)`` derived from ``<idls_root>/<pkg>/<kind>/<file>``,
    or ``None`` if the file does not live at that depth."""
    try:
        rel = idl_path.relative_to(idls_root)
    except ValueError:
        return None
    parts = rel.parts
    if len(parts) < 3:
        return None
    return parts[0], parts[1]


def _collect_constants(hpp_path: Path) -> list[tuple[str, str]]:
    """Return a list of ``(namespace_name, constant_name)`` tuples extracted
    from a generated header."""
    content = hpp_path.read_text(encoding="utf-8", errors="replace")
    constants: list[tuple[str, str]] = []
    for ns_match in _NS_BLOCK_RE.finditer(content):
        ns = ns_match.group("ns")
        for const_match in _CONST_DECL_RE.finditer(ns_match.group("body")):
            constants.append((ns, const_match.group("name")))
    return constants


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--idls-root",
        required=True,
        type=Path,
        help="Root directory of the generated IDL tree (used to derive pkg::kind:: qualifiers).",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="File to write %%ignore directives to (overwritten; created empty if no duplicates).",
    )
    parser.add_argument(
        "idl_files",
        nargs="+",
        type=Path,
        help="The .i files whose corresponding .hpp files should be scanned.",
    )
    args = parser.parse_args()

    idls_root = args.idls_root.resolve()
    # Sort to make the "first occurrence wins" choice deterministic across
    # runs. msg/ sorts before srv/ alphabetically, which is what we want.
    idl_files = sorted(p.resolve() for p in args.idl_files)

    seen_first_fq: dict[str, str] = {}
    duplicates: set[str] = set()

    for idl_path in idl_files:
        hpp_path = idl_path.with_suffix(".hpp")
        if not hpp_path.is_file():
            continue
        pkg_kind = _pkg_kind_from_path(idl_path, idls_root)
        if pkg_kind is None:
            continue
        pkg, kind = pkg_kind
        for ns, const_name in _collect_constants(hpp_path):
            fq = f"{pkg}::{kind}::{ns}::{const_name}"
            prev_fq = seen_first_fq.get(const_name)
            if prev_fq is None:
                seen_first_fq[const_name] = fq
            elif prev_fq != fq:
                duplicates.add(fq)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as out:
        for fq in sorted(duplicates):
            out.write(f"%ignore {fq};\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
