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
"""The DDS domains a test process may pick when it needs one of its own, and why the choice is
narrower than "anything but 0". Mirrors test/detail/test_domain.h: keep both in step.

Fast-DDS maps a domain onto UDP ports by formula: domain d owns 7400 + 250 * d .. + 249, with the
metatraffic unicast port of participant slot s at 7400 + 250 * d + 10 + 2 * s and the user unicast
port one above it (successive slots are two ports apart). From domain 101 upwards those ports lie
inside the OS's dynamic (ephemeral) port range -- 32768 and up on Linux, 49152 and up on Windows
and macOS -- so they belong to whatever the OS hands out to any socket, and on Windows to the
200-port blocks Hyper-V / WinNAT reserve for itself (the GitHub windows-2022 and windows-2025
runners each carry one, at an image-dependent position). A port that is taken does not fail
participant creation: Fast-DDS moves the participant to the next slot, silently, up to a hundred
times. Peers that discover each other by unicast initial peers -- every test under
test/fast_dds_localhost_profile.xml -- probe slots 0 .. maxInitialPeersRange - 1 (50 there) of the
domain they expect, so when a reserved block covers all fifty, the two participants of one test
process land on slots 50 and 51, neither probes the other, and the test waits out its whole match
budget with nothing logged. That was the once-in-a-few-hundred-processes 15 s "a subscriber failed
to match in time" on the Windows CI jobs, reproduced deterministically on Linux by occupying the
fifty slots by hand. Domains 0 .. 100 keep every port below 32768 on every platform (ROS 2
documents the same limit for the same reason).

Below that ceiling some domains are spoken for: 0 is the default that resident software on a shared
host lives on; 14, 42 (the cross-version compatibility scripts, which run beside the same-version
suite by design), 44, 46, 71 and 72 are pinned by suites that must agree on one; and 73 .. 100 is
the band the request/response reliability and transport-tuning suites derive a domain from by seed.
A random draw avoids all of them, so two suites side by side (ctest -j, or two jobs on one runner)
cannot cross-match by a standing collision -- only by a 1-in-66 coincidence between two draws.
"""
import random

#: Highest domain whose UDP ports stay below every supported OS's dynamic port range.
HIGHEST_SAFE_DOMAIN = 100

#: First domain of the seed-derived band shared by the request/response reliability and
#: transport-tuning suites: SEED_BAND_FIRST .. HIGHEST_SAFE_DOMAIN.
SEED_BAND_FIRST = 73
SEED_BAND_SIZE = HIGHEST_SAFE_DOMAIN - SEED_BAND_FIRST + 1

#: The domains suites pin, so a random draw must not return any of them: 0 is the default that
#: resident software lives on, and each of the rest is agreed on by a suite whose halves have to
#: meet there (see the module docstring). RANDOM_DOMAIN_POOL below is exactly
#: 1 .. SEED_BAND_FIRST - 1 minus these, which test/source_text/test_domain_parity.py checks --
#: in both languages, against each language's own copy of this list.
PINNED_DOMAINS = (0, 14, 42, 44, 46, 71, 72)

#: The domains a random per-process draw may return: 1 .. 100 minus the pinned ones and the seed
#: band (see the module docstring).
RANDOM_DOMAIN_POOL = (
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
    38, 39, 40, 41, 43, 45, 47, 48, 49, 50, 51, 52,
    53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64,
    65, 66, 67, 68, 69, 70,
)


def seed_band_domain(seed: int) -> int:
    """A domain derived from a seed (a pid, an iteration index), inside the seed band. Any
    integer works; Python's % already wraps a negative one into the band, which the C++
    mirror reproduces."""
    return SEED_BAND_FIRST + (seed % SEED_BAND_SIZE)


def random_test_domain() -> int:
    """A domain drawn at random from RANDOM_DOMAIN_POOL; call once per process."""
    return random.choice(RANDOM_DOMAIN_POOL)
