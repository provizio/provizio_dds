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
"""Cross-language parity of the test-domain policy.

The policy lives twice: in test/detail/test_domain.h for the C++ suites and, hand-mirrored,
in test/python/provizio_test_domain.py for the Python ones. Nothing but this test ties the
two together, and a drift between them is invisible in both directions:

  - a domain pinned in one language only stays in the other's random pool, so two suites
    that must not share a domain eventually draw the same one and cross-match -- the
    intermittent port-collision flake the pool exists to prevent, back for one draw in
    sixty-six;
  - a ceiling raised in one language only puts that language's ports back inside the OS
    dynamic range, which is the silent 15 s "failed to match in time" the ceiling exists
    to prevent.

Both suites' own tests stay green either way, because each asserts against its own
language's list. So this compares the lists themselves, and also checks each language's
pool against the RULE it claims to follow (1 .. SEED_BAND_FIRST - 1 minus the pinned
domains) -- an edit that adds a pinned domain without also removing it from that language's
pool fails here even if the other language was edited identically.

A source-text check: it parses the C++ header and imports the Python module, so it needs an
interpreter and nothing else -- no build, no bindings, no DDS participant.
"""
import os
import re
import sys

CPP_HEADER = os.path.join("test", "detail", "test_domain.h")
PYTHON_MODULE_DIR = os.path.join("test", "python")


def _parse_int_constant(source, name):
    """The value of ``constexpr int <name> = <literal>;`` in ``source``."""
    match = re.search(r"\b" + re.escape(name) + r"\s*=\s*(-?\d+)\s*;", source)
    if match is None:
        raise LookupError(
            f"{name} not found in {CPP_HEADER}. If it was renamed, rename it here too "
            f"rather than leaving this check to pass on nothing."
        )
    return int(match.group(1))


def _parse_int_array(source, name):
    """The values of ``constexpr std::array<int, N> <name>{...};`` in ``source``.

    The declared N is checked against the number of entries as well: a std::array whose
    initialiser is shorter than its size zero-fills the tail, so a list that lost an entry
    would otherwise silently gain domain 0 -- the one domain the pool must never contain.
    """
    match = re.search(
        r"constexpr\s+std::array<\s*int\s*,\s*(\d+)\s*>\s+" + re.escape(name) + r"\s*\{([^}]*)\}",
        source,
    )
    if match is None:
        raise LookupError(
            f"{name} not found in {CPP_HEADER}. If it was renamed or its type changed, "
            f"update this check rather than leaving it to pass on nothing."
        )
    declared_size = int(match.group(1))
    values = [int(entry) for entry in re.findall(r"-?\d+", match.group(2))]
    if len(values) != declared_size:
        raise ValueError(
            f"{name} declares std::array<int, {declared_size}> but lists {len(values)} "
            f"entries; the tail would be zero-filled"
        )
    return values


def main(argv):
    if len(argv) != 2:
        print(f"Usage: {os.path.basename(argv[0])} <repo-root>", file=sys.stderr)
        return 2
    root = argv[1]

    header_path = os.path.join(root, CPP_HEADER)
    if not os.path.isfile(header_path):
        print(
            f"test_domain_parity: {CPP_HEADER} not found under {root}. If it moved, update "
            f"this check -- silently comparing nothing would let the two sides drift.",
            file=sys.stderr,
        )
        return 2
    with open(header_path, encoding="utf-8") as handle:
        header = handle.read()

    # Imported rather than parsed: the Python side's real values, whatever they are spelled
    # from. It pulls in nothing but the standard library, so this works in a build without
    # the Python bindings.
    sys.path.insert(0, os.path.join(root, PYTHON_MODULE_DIR))
    import provizio_test_domain as python_side

    failures = []

    def expect(condition, description):
        if not condition:
            failures.append(description)

    cpp_highest_safe = _parse_int_constant(header, "k_highest_safe_domain")
    cpp_seed_band_first = _parse_int_constant(header, "k_seed_band_first")
    cpp_pinned = _parse_int_array(header, "k_pinned_domains")
    cpp_pool = _parse_int_array(header, "k_random_domain_pool")

    expect(
        cpp_highest_safe == python_side.HIGHEST_SAFE_DOMAIN,
        f"highest safe domain: C++ {cpp_highest_safe} vs Python "
        f"{python_side.HIGHEST_SAFE_DOMAIN}",
    )
    expect(
        cpp_seed_band_first == python_side.SEED_BAND_FIRST,
        f"first seed-band domain: C++ {cpp_seed_band_first} vs Python "
        f"{python_side.SEED_BAND_FIRST}",
    )
    expect(
        sorted(cpp_pinned) == sorted(python_side.PINNED_DOMAINS),
        f"pinned domains: C++ {sorted(cpp_pinned)} vs Python "
        f"{sorted(python_side.PINNED_DOMAINS)}",
    )
    expect(
        list(cpp_pool) == list(python_side.RANDOM_DOMAIN_POOL),
        f"random domain pool: C++ {list(cpp_pool)} vs Python "
        f"{list(python_side.RANDOM_DOMAIN_POOL)}",
    )

    # Each language's pool against the rule it documents, so that a pinned domain added on
    # both sides but removed from neither pool is still a failure.
    for language, seed_band_first, pinned, pool in (
        ("C++", cpp_seed_band_first, cpp_pinned, cpp_pool),
        ("Python", python_side.SEED_BAND_FIRST, python_side.PINNED_DOMAINS,
         python_side.RANDOM_DOMAIN_POOL),
    ):
        expected_pool = [
            domain for domain in range(1, seed_band_first) if domain not in set(pinned)
        ]
        expect(
            list(pool) == expected_pool,
            f"{language} pool is not 1 .. {seed_band_first - 1} minus the pinned domains: "
            f"missing {sorted(set(expected_pool) - set(pool))}, unexpected "
            f"{sorted(set(pool) - set(expected_pool))}",
        )

    # The seed band has to leave room for a band and stay under the ceiling; a mirror that
    # inverted the two would make seed_band_domain() return domains nobody checked.
    expect(
        1 < cpp_seed_band_first <= cpp_highest_safe,
        f"seed band {cpp_seed_band_first} .. {cpp_highest_safe} is not a band below the "
        f"ceiling",
    )

    if failures:
        for failure in failures:
            print(f"test_domain_parity: FAIL: {failure}", file=sys.stderr)
        print(
            f"test_domain_parity: {len(failures)} disagreement(s) between {CPP_HEADER} and "
            f"{os.path.join(PYTHON_MODULE_DIR, 'provizio_test_domain.py')}; keep both in step.",
            file=sys.stderr,
        )
        return 1

    print(
        f"test_domain_parity: PASS (ceiling {cpp_highest_safe}, seed band from "
        f"{cpp_seed_band_first}, {len(cpp_pinned)} pinned, {len(cpp_pool)} in the pool)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
