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

"""Fails if any string literal that can be emitted at runtime contains a raw non-ASCII
character.

Why this is a test and not a style preference: a Python interpreter on Windows encodes
``stdout`` with the ANSI code page (cp1252 in CI), not UTF-8, and ``print`` RAISES
``UnicodeEncodeError`` on a character that page cannot represent. So an em dash or an arrow
in a log line or an assertion message is not a cosmetic choice -- it is a crash on one
platform, hidden until that exact line runs there. It reached CI as a test that failed
*after* passing, on the line that reported its own success. The same text through
``logging`` is worse than a crash: the handler swallows the error, so the diagnostic
silently disappears on the one platform where someone is reading it.

C++ is checked for the same reason one step removed: those literals reach a Windows console
that is rendering a non-UTF-8 code page, where the UTF-8 bytes come out as mojibake. It
does not crash, but a shipped library's log output is no place for it.

Comments are deliberately NOT checked. They are never encoded to a stream, the prose in
this codebase leans on em dashes and arrows heavily, and the whole point of the rule is the
emitted text. Docstrings (and bare-string "attribute docstrings") are exempt for the same
reason: nothing here prints ``__doc__``.

Deliberate non-ASCII test data stays legal -- spell it as an escape (``"\\u00a0"``) rather
than as a raw character. That keeps the intent explicit at the call site AND keeps the
source itself ASCII, which is what this check reads.

Shell scripts are out of scope: their output goes to a CI log or a UTF-8 terminal, never
through an encoder that can refuse it.

Usage: runtime_text_is_ascii.py <repo-root>
"""

import ast
import os
import re
import sys
import tokenize

# Scanned source roots, relative to the repository root. An allowlist rather than a walk of
# everything minus a denylist: a missing entry is caught below (each must exist), whereas a
# denylist silently starts scanning the next vendored dependency that appears.
SOURCE_ROOTS = ("include", "src", "python", "test", "cmake")

PYTHON_SUFFIXES = (".py",)
CPP_SUFFIXES = (".h", ".hpp", ".cpp", ".i")

# String-ish token kinds. Python 3.12+ splits an f-string into FSTRING_START / FSTRING_MIDDLE
# / FSTRING_END; earlier versions emit the whole thing as one STRING. Naming both keeps this
# working across every interpreter CI runs without a version check.
STRING_TOKEN_KINDS = frozenset({"STRING", "FSTRING_MIDDLE"})

# The opening of a C++ raw string literal: an optional encoding prefix, ``R``, a quote, a
# delimiter of up to 16 characters that may not contain parentheses, backslashes or
# whitespace, and the opening parenthesis. The literal then runs, escapes and all, to the
# first ``)delimiter"``.
RAW_STRING_OPENER = re.compile(r'(?:u8|u|U|L)?R"([^()\\\s]{0,16})\(')

# Suggested ASCII spellings for the characters that actually turn up, so a failure tells the
# author what to write instead of only what not to.
# Keyed by escape, not by the character itself: this file is scanned by its own check, and
# spelling them as escapes is exactly what the check asks of any deliberate non-ASCII datum.
SUGGESTIONS = {
    "\u2192": "->",
    "\u2190": "<-",
    "\u2014": "--",
    "\u2013": "-",
    "\u2248": "~=",
    "\u00b0": " deg",
    "\u00a0": "a plain space (or the \\u00a0 escape, if the NBSP is the point)",
    "\u2026": "...",
    "\u201c": '"',
    "\u201d": '"',
    "\u2018": "'",
    "\u2019": "'",
}


def _non_ascii(text):
    """The distinct non-ASCII characters in ``text``, in first-seen order."""
    seen = []
    for char in text:
        if ord(char) > 127 and char not in seen:
            seen.append(char)
    return seen


def _describe(chars):
    parts = []
    for char in chars:
        hint = SUGGESTIONS.get(char)
        # Reported as a code point and never as the character itself. This message is
        # printed, and printing it on the very platform the check protects (Windows, ANSI
        # code page) would raise UnicodeEncodeError -- the check would crash instead of
        # reporting, which is the bug it exists to catch.
        name = f"U+{ord(char):04X}"
        parts.append(f"{name} -> use {hint}" if hint else name)
    return "; ".join(parts)


def _documentation_line_ranges(source):
    """Line ranges of string literals that stand alone as a statement.

    That covers both real docstrings and the bare-string "attribute docstring" convention
    this codebase uses to document module- and class-level constants. Neither is ever
    emitted, and both are prose.
    """
    ranges = []
    for node in ast.walk(ast.parse(source)):
        for child in ast.iter_child_nodes(node):
            if (
                isinstance(child, ast.Expr)
                and isinstance(child.value, ast.Constant)
                and isinstance(child.value.value, str)
            ):
                ranges.append((child.value.lineno, child.value.end_lineno))
    return ranges


def _check_python(path):
    """Offending ``(line, characters)`` pairs in a Python file."""
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    documentation = _documentation_line_ranges(source)

    offences = []
    with open(path, "rb") as handle:
        for token in tokenize.tokenize(handle.readline):
            if tokenize.tok_name[token.type] not in STRING_TOKEN_KINDS:
                continue
            if any(start <= token.start[0] <= end for start, end in documentation):
                continue
            # token.string is the literal AS WRITTEN, so a "\u00a0" escape reads as ASCII
            # here and only a raw character is reported -- which is the rule.
            chars = _non_ascii(token.string)
            if chars:
                offences.append((token.start[0], chars))
    return offences


def _check_cpp(path):
    """Offending ``(line, characters)`` pairs in a C/C++ file.

    Reports non-ASCII anywhere OUTSIDE a comment, rather than trying to recognise string
    literals precisely. In C++ a raw non-ASCII character outside a comment has essentially
    nowhere legal to be except a string or character literal, so the two rules catch the
    same thing -- but this one needs to get only comments right, and it stays correct for
    constructs the narrower rule would have to learn one by one: character literals and the
    wide/u8 prefixes need no handling at all.

    String and character literals are still tracked, for one reason: so that ``//`` in
    ``"http://..."`` and a quote inside ``'"'`` cannot open a comment or a literal that was
    never there. Raw string literals (``R"delimiter(...)delimiter"``) are recognised for the
    same reason: nothing inside one is an escape and only its own closing sequence ends it,
    so an ordinary string tracker would take a quote inside one as its end and could then read
    a ``//`` in the remainder as a comment, hiding whatever followed on the line.
    """
    with open(path, encoding="utf-8") as handle:
        source = handle.read()

    offences = {}
    index = 0
    length = len(source)
    line = 1
    in_block_comment = False
    in_line_comment = False
    in_string = False
    in_char = False

    def record(at_line, character):
        """Note one offending character, de-duplicated per line.

        Takes the line explicitly rather than closing over the loop's variable, so the
        recorded position cannot drift from the character being recorded.
        """
        found = offences.setdefault(at_line, [])
        if character not in found:
            found.append(character)

    while index < length:
        char = source[index]
        pair = source[index:index + 2]

        if char == "\n":
            line += 1
            in_line_comment = False
            index += 1
            continue

        if in_block_comment:
            if pair == "*/":
                in_block_comment = False
                index += 2
                continue
            index += 1
            continue

        if in_line_comment:
            index += 1
            continue

        if in_string or in_char:
            if char == "\\":
                # An escape, including \" and \', is two source characters and cannot end
                # the literal. Count a line continuation's newline so line numbers hold.
                if source[index + 1:index + 2] == "\n":
                    line += 1
                index += 2
                continue
            if (in_string and char == '"') or (in_char and char == "'"):
                in_string = False
                in_char = False
                index += 1
                continue
            if ord(char) > 127:
                record(line, char)
            index += 1
            continue

        if pair == "/*":
            in_block_comment = True
            index += 2
            continue
        if pair == "//":
            in_line_comment = True
            index += 2
            continue
        raw_string = RAW_STRING_OPENER.match(source, index)
        previous = source[index - 1] if index else ""
        if raw_string and not (previous.isalnum() or previous == "_"):
            # Consumed whole, so that neither a quote nor a ``//`` inside it can end the
            # literal or open a comment. An unterminated one runs to the end of the file, as
            # it does for the compiler.
            closing = ")" + raw_string.group(1) + '"'
            end = source.find(closing, raw_string.end())
            end = length if end == -1 else end + len(closing)
            for literal_char in source[index:end]:
                if literal_char == "\n":
                    line += 1
                elif ord(literal_char) > 127:
                    record(line, literal_char)
            index = end
            continue
        if char == '"':
            in_string = True
            index += 1
            continue
        if char == "'":
            # Not every apostrophe opens a character literal: C++14 lets one separate digits
            # (1'000'000). Mistaking that for a literal would flip the parser's state for the
            # rest of the file, so a quote preceded by an identifier or digit character is
            # read as a separator instead.
            previous = source[index - 1] if index else ""
            if not (previous.isalnum() or previous == "_"):
                in_char = True
            index += 1
            continue
        if ord(char) > 127:
            record(line, char)
        index += 1

    return sorted(offences.items())


def main(argv):
    if len(argv) != 2:
        print(f"Usage: {os.path.basename(argv[0])} <repo-root>", file=sys.stderr)
        return 2
    root = argv[1]

    offences = []
    scanned = 0
    for source_root in SOURCE_ROOTS:
        absolute_root = os.path.join(root, source_root)
        if not os.path.isdir(absolute_root):
            print(
                f"runtime_text_is_ascii: '{source_root}' is not a directory under {root}. "
                f"If it was renamed or removed, update SOURCE_ROOTS -- silently scanning "
                f"nothing would let the rule rot.",
                file=sys.stderr,
            )
            return 2
        for directory, subdirectories, files in os.walk(absolute_root):
            subdirectories[:] = [
                name
                for name in subdirectories
                if name != "__pycache__" and not name.startswith(".")
            ]
            for name in sorted(files):
                path = os.path.join(directory, name)
                if name.endswith(PYTHON_SUFFIXES):
                    checker = _check_python
                elif name.endswith(CPP_SUFFIXES):
                    checker = _check_cpp
                else:
                    continue
                scanned += 1
                try:
                    found = checker(path)
                except (OSError, SyntaxError, UnicodeDecodeError, tokenize.TokenError) as error:
                    print(f"runtime_text_is_ascii: cannot read {path}: {error}", file=sys.stderr)
                    return 2
                for line, chars in found:
                    offences.append((os.path.relpath(path, root), line, chars))

    if offences:
        print(
            f"runtime_text_is_ascii: FAILED -- {len(offences)} site(s) carry a raw non-ASCII "
            f"character:",
            file=sys.stderr,
        )
        for path, line, chars in offences:
            print(f"  {path}:{line}: {_describe(chars)}", file=sys.stderr)
        print(
            "\nA string literal can be printed, and Python's stdout on Windows is encoded "
            "with the ANSI code page, which raises on characters it cannot represent. Use "
            "the ASCII spelling in emitted text; if a non-ASCII character IS the point (test "
            "data), write it as an escape so the source stays ASCII.\n"
            "Exempt, and not checked: comments in both languages, and Python docstrings "
            "(including the bare-string form used to document constants). In C++ everything "
            "outside a comment is checked, which is where a string or character literal is "
            "the only place a raw non-ASCII character could legally be.",
            file=sys.stderr,
        )
        return 1

    print(f"runtime_text_is_ascii: PASS ({scanned} source file(s) scanned)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
