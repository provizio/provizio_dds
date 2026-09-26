#!/bin/bash

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

# Guards the architecture the binary cache key is built from.
#
# Linux's struct utsname carries a machine field and nothing else: the "hardware platform" and
# "processor" options exist for Solaris, and on Linux an implementation is free to answer
# "unknown" for them. GNU coreutils synthesises a plausible value, so a dependency on them can sit
# unnoticed for years; uutils coreutils - the default coreutils of Ubuntu 26.04 - answers
# "unknown", and the key then names an archive that cannot exist. Nothing reports that, because a
# key naming no archive looks exactly like a configuration for which no cache was ever published:
# the only symptom is every build on such a host quietly recompiling Fast-DDS from source.
#
# Use as: config_name_architecture_test.sh <REPO_ROOT> <WORK_DIR>
#   REPO_ROOT  the repository to check, i.e. the one holding bin_cache_config_name.sh
#   WORK_DIR   an absolute scratch path, created and removed by this script

set -eu
set -o pipefail

if [ "$#" -ne 2 ]; then
  echo "Use as: $(basename "$0") <REPO_ROOT> <WORK_DIR>"
  exit 1
fi

REPO_ROOT="$(cd "$1" && pwd -P)"
WORK_DIR="$2"

# WORK_DIR is deleted outright below, so refuse anything that is not an absolute path well
# inside a filesystem: a relative or empty one would delete whatever the caller happened to be
# standing in.
case "${WORK_DIR}" in
/?*/?*) ;;
*)
  echo "WORK_DIR must be an absolute path at least two levels deep, got '${WORK_DIR}'"
  exit 1
  ;;
esac

# Registered here rather than at the end of the script: everything below can fail, and a scratch
# directory left in the build tree (with a uname shim first on its PATH) would outlive the run.
# WORK_DIR has just been checked to be an absolute path at least two levels deep.
trap 'rm -rf "${WORK_DIR}"' EXIT

FAILURES=0
fail() {
  echo "FAILED: $*"
  FAILURES=$((FAILURES + 1))
}

REAL_UNAME="$(command -v uname)" || REAL_UNAME=""
if [ -z "${REAL_UNAME}" ]; then
  echo "Cannot run: this host has no uname for the shim to stand in for"
  exit 1
fi
EXPECTED_ARCH="$("${REAL_UNAME}" -m)"
if [ -z "${EXPECTED_ARCH}" ]; then
  echo "Cannot run: the machine option reported nothing on this host"
  exit 1
fi

# Not world-writable, and not readable by other users: everything the script under test runs -
# git, grep, awk, sha256sum - resolves through this directory while it is first on PATH, so
# anything droppable into it would be executed by this test. The mode is given to mkdir rather
# than applied afterwards, because between the two there would be a window in which the directory
# carries whatever the umask allows - 0777 under "umask 000". One mkdir per level: -p applies -m
# to the last component only.
rm -rf "${WORK_DIR}"
mkdir -p "$(dirname "${WORK_DIR}")"
mkdir -m 700 "${WORK_DIR}"
mkdir -m 700 "${WORK_DIR}/bin"

# 1. The key must keep naming this host's architecture when the non-portable options report
#    nothing. Stand in for a uutils-style coreutils with a uname that answers the POSIX option and
#    nothing else, exactly as uutils does on Linux.
#
# The shim reads its two values from the environment rather than having them substituted into its
# text, so no character in an architecture name or in a path can corrupt the script it becomes.
cat >"${WORK_DIR}/bin/uname" <<'SHIM'
#!/bin/bash
# Stands in for uutils coreutils' uname on Linux: the POSIX machine option is answered, the two
# Solaris ones have no field to read and say so, and anything else goes to the real one.
case "${1:-}" in
-m | --machine) echo "${PROVIZIO_TEST_ARCH}" ;;
-i | --hardware-platform | -p | --processor) echo "unknown" ;;
*) exec "${PROVIZIO_TEST_REAL_UNAME}" "$@" ;;
esac
SHIM
chmod 700 "${WORK_DIR}/bin/uname"

# WILDCARD keeps this offline and fast: it is the cleanup mode of the script under test, and it
# replaces both content hashes with "*" instead of resolving the IDLs revision over the network.
KEY=""
KEY_STATUS=0
# The status is captured rather than allowed to propagate: under "set -e" a non-zero exit of the
# script under test would end this one at the assignment, with no failure text, no summary line
# and nothing to say which check was running - a silent failure in the test written to make a
# silent failure loud.
KEY="$(PATH="${WORK_DIR}/bin:${PATH}" \
  PROVIZIO_TEST_ARCH="${EXPECTED_ARCH}" \
  PROVIZIO_TEST_REAL_UNAME="${REAL_UNAME}" \
  "${REPO_ROOT}/bin_cache_config_name.sh" Release WILDCARD)" || KEY_STATUS=$?

EXPECTED_KEY="linux_${EXPECTED_ARCH}.*.idls_*.Release"
if [ "${KEY_STATUS}" -ne 0 ]; then
  fail "bin_cache_config_name.sh exited with ${KEY_STATUS}, having said: ${KEY}"
elif [ "${KEY}" != "${EXPECTED_KEY}" ]; then
  fail "the cache key is '${KEY}', expected '${EXPECTED_KEY}'."
  echo "       The architecture must come from the POSIX machine option. A key that names"
  echo "       'unknown', or anything else this host is not, matches no published archive and"
  echo "       silently costs every build here a full Fast-DDS compile."
else
  echo "OK: the cache key names ${EXPECTED_ARCH} even where only the POSIX option reports it"
fi

# The pattern the scan below is built on. Both spellings of each option, and any bundling of the
# short ones with other letters, so that swapping one spelling for another is not a way through:
# of uname's short options only -i and -p are the non-portable pair, so a bundle containing either
# letter is one of them whatever else it carries.
NON_PORTABLE='uname[[:space:]]+(-[a-zA-Z]*[ip][a-zA-Z]*|--hardware-platform|--processor)([^a-zA-Z0-9_-]|$)'

# A line may name the options on purpose - the comments that explain this very rule do, and so
# does the test data just below. Without a way to say so, the rule could not be written down in
# the files it governs, and the next person to document it would fail this test for doing so.
EXEMPTION_MARKER='portable-uname-exempt'

# 2. The pattern is the whole of the scan that follows, and it fails silently in both directions:
#    a spelling it misses retires the guard without anyone noticing, and a mention it wrongly
#    catches makes the rule impossible to write down. So pin both sides of it.
MUST_MATCH=(
  'uname -i'                    # portable-uname-exempt: test data, not a use
  'uname -p'                    # portable-uname-exempt: test data, not a use
  'uname --hardware-platform'   # portable-uname-exempt: test data, not a use
  'uname --processor'           # portable-uname-exempt: test data, not a use
  'uname -is'                   # portable-uname-exempt: bundled, banned letter first
  'uname -mp'                   # portable-uname-exempt: bundled, banned letter last
  'ARCH="$(uname -i)"'          # portable-uname-exempt: as it would really be written
)
MUST_NOT_MATCH=(
  'uname -m'
  'uname --machine'
  'uname -a'
  'uname -sr'
  'basename -i'
)

FAILURES_BEFORE_PATTERN_CHECK="${FAILURES}"
for probe in "${MUST_MATCH[@]}"; do
  if ! printf '%s\n' "${probe}" | grep -qE "${NON_PORTABLE}"; then
    fail "the guard below does not catch '${probe}', so that spelling could be reintroduced."
  fi
done
for probe in "${MUST_NOT_MATCH[@]}"; do
  if printf '%s\n' "${probe}" | grep -qE "${NON_PORTABLE}"; then
    fail "the guard below wrongly catches '${probe}', which is portable and must stay usable."
  fi
done
if [ "${FAILURES}" -eq "${FAILURES_BEFORE_PATTERN_CHECK}" ]; then
  echo "OK: the guard catches ${#MUST_MATCH[@]} non-portable spellings and none of the ${#MUST_NOT_MATCH[@]} portable ones"
fi

# 3. Nothing in the build may reach for the two non-portable options again - here or anywhere
#    else. This is the check that generalises: what check 1 catches is invisible on a host whose
#    coreutils still guesses, so a reintroduction elsewhere would pass every gate we have.
TRACKED="${WORK_DIR}/tracked_scripts"
if (cd "${REPO_ROOT}" && git ls-files -- \
  '*.sh' '*.bash' '*.ps1' '*.bat' '*.cmd' '*.cmake' '*CMakeLists.txt' \
  '*.yml' '*.yaml' '*.py' '*.dockerfile' '*Dockerfile') >"${TRACKED}" 2>/dev/null; then
  SCRIPTS=()
  while IFS= read -r tracked; do
    SCRIPTS+=("${tracked}")
  done <"${TRACKED}"

  if [ "${#SCRIPTS[@]}" -eq 0 ]; then
    fail "found no tracked build scripts to check in ${REPO_ROOT}."
  else
    # grep answers 0 for a match, 1 for none and 2 for a failure of its own. Only 1 is a pass:
    # treating 2 as one would let an unreadable or mis-resolved path retire this check quietly,
    # which is the very shape of silent failure it exists to catch.
    MATCHES=""
    GREP_STATUS=0
    MATCHES="$(cd "${REPO_ROOT}" && grep -nE "${NON_PORTABLE}" -- "${SCRIPTS[@]}")" || GREP_STATUS=$?

    if [ "${GREP_STATUS}" -gt 1 ]; then
      fail "could not scan the tracked build scripts (grep exited with ${GREP_STATUS})."
    else
      # A match on a line that carries the marker is a deliberate mention, not a use.
      OFFENDERS=""
      if [ "${GREP_STATUS}" -eq 0 ]; then
        OFFENDERS="$(printf '%s\n' "${MATCHES}" | grep -vF "${EXEMPTION_MARKER}")" || true
      fi

      if [ -n "${OFFENDERS}" ]; then
        fail "a build script asks uname for a hardware platform or a processor:"
        echo "${OFFENDERS}"
        echo "       Linux populates neither, so the answer may be 'unknown'. Use the POSIX machine"
        echo "       option, which the kernel always fills in. If the line names the options rather"
        echo "       than using them, mark it '${EXEMPTION_MARKER}'."
      else
        echo "OK: none of the ${#SCRIPTS[@]} tracked build scripts uses a non-portable uname option"
      fi
    fi
  fi
else
  # Reported rather than failed: this half needs the list of tracked files, and an unpacked
  # source tree is not a checkout. Check 1 above stands on its own, and CI always builds from a
  # checkout, so the guard always runs where a reintroduction would be caught.
  echo "SKIPPED: ${REPO_ROOT} is not a readable git checkout, so the tracked scripts"
  echo "         cannot be listed and only the key itself was checked"
fi

if [ "${FAILURES}" -ne 0 ]; then
  echo "${FAILURES} check(s) failed"
  exit 1
fi

echo "All checks passed"
