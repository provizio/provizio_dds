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

# Coverage for fully_qualified_fastdds_libs.sh, which an install with
# INSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS -- every Linux prebuilt binaries build among them -- runs
# on the lib/ it installed into: every library there linking a Fast-DDS library by a name that is a
# link must link the file itself instead, and the links go. It runs on a lib/ of stand-ins built
# here: a Fast-DDS library, a library linking it by its SONAME as provizio_dds does, and an
# executable ldd cannot read, as a lib/ shared with other software (/usr/local/lib) holds -- sorted
# first, as that once ended the script before anything was patched.
#
# Use as: fully_qualified_fastdds_libs_test.sh <REPO_ROOT> <WORK_DIR> <C_COMPILER>
#   REPO_ROOT   the repository holding fully_qualified_fastdds_libs.sh
#   WORK_DIR    an absolute scratch path, created and removed by this script
#   C_COMPILER  a C compiler to build the stand-ins with

set -eu
set -o pipefail

if [ "$#" -ne 3 ]; then
  echo "Use as: $(basename "$0") <REPO_ROOT> <WORK_DIR> <C_COMPILER>"
  exit 1
fi

SCRIPT="$1/fully_qualified_fastdds_libs.sh"
WORK_DIR="$2"
C_COMPILER="$3"

# WORK_DIR is deleted outright below, so refuse anything that is not an absolute path well inside a
# filesystem: a relative or empty one would delete whatever the caller happened to be standing in.
case "${WORK_DIR}" in
/?*/?*) ;;
*)
  echo "WORK_DIR must be an absolute path at least two levels deep, got '${WORK_DIR}'"
  exit 1
  ;;
esac

for tool in patchelf readelf ldd; do
  if ! command -v "${tool}" >/dev/null; then
    echo "${tool} is needed to run this test, and the script it covers"
    exit 1
  fi
done

trap 'rm -rf "${WORK_DIR}"' EXIT
rm -rf "${WORK_DIR}"
mkdir -p "${WORK_DIR}/lib"
LIB="${WORK_DIR}/lib"

echo 'int fast_stand_in(void) { return 42; }' >"${WORK_DIR}/fast.c"
"${C_COMPILER}" -shared -fPIC -Wl,-soname,libfaststandin.so.1 -o "${LIB}/libfaststandin.so.1.2.3" "${WORK_DIR}/fast.c"
ln -s libfaststandin.so.1.2.3 "${LIB}/libfaststandin.so.1"
ln -s libfaststandin.so.1 "${LIB}/libfaststandin.so"
echo 'int fast_stand_in(void); int user(void) { return fast_stand_in(); }' >"${WORK_DIR}/user.c"
"${C_COMPILER}" -shared -fPIC -o "${LIB}/libuser.so" "${WORK_DIR}/user.c" -L"${LIB}" -lfaststandin
printf '#!/bin/sh\necho not a library\n' >"${LIB}/aaa_script"
chmod 755 "${LIB}/aaa_script"

FAILURES=0
fail() {
  echo "FAILED: $*"
  FAILURES=$((FAILURES + 1))
}

needed() {
  readelf -d "$1" | sed -n 's/.*(NEEDED).*\[\(.*\)\]/\1/p' | grep '^libfast' || true
}

[ "$(needed "${LIB}/libuser.so")" = "libfaststandin.so.1" ] ||
  fail "the stand-in does not link the Fast-DDS stand-in by its SONAME to begin with: $(needed "${LIB}/libuser.so")"

if ! OUTPUT="$("${SCRIPT}" "${LIB}" 2>&1)"; then
  fail "the script failed on a lib/ holding an executable ldd cannot read. It said:"
  printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
fi
[ "$(needed "${LIB}/libuser.so")" = "libfaststandin.so.1.2.3" ] ||
  fail "libuser.so links '$(needed "${LIB}/libuser.so")', not the file libfaststandin.so.1.2.3"
for link in libfaststandin.so.1 libfaststandin.so; do
  [ ! -e "${LIB}/${link}" ] && [ ! -L "${LIB}/${link}" ] || fail "${link} was left in place"
done
[ "$(cat "${LIB}/aaa_script")" = "$(printf '#!/bin/sh\necho not a library')" ] || fail "aaa_script was changed"

# And back: "revert" makes the links again, down to the unversioned name
"${SCRIPT}" "${LIB}" revert >/dev/null
for link in libfaststandin.so.1 libfaststandin.so libfaststandin.so.1.2; do
  [ "$(readlink "${LIB}/${link}")" = "libfaststandin.so.1.2.3" ] || fail "revert did not link ${link} to the library"
done

if [ "${FAILURES}" -ne 0 ]; then
  echo "${FAILURES} check(s) failed"
  exit 1
fi
echo "fully_qualified_fastdds_libs.sh links the libraries themselves, whatever else lib/ holds"
