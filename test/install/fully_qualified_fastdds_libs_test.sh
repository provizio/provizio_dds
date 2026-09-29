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
# here: a Fast-DDS library, a library linking it by its SONAME as provizio_dds does, a copy of that
# library marked as built for another architecture, as a cross-compiled install's is, which ldd
# cannot read, and files that link nothing, as a lib/ shared with other software (/usr/local/lib)
# holds: a script and, where the compiler can link one, a static executable, both with the
# executable bit, an object file named as a library, and a library stripped of its section headers,
# which patchelf takes for a packed executable, that links no Fast-DDS library -- sorted first, as
# such a file once ended the script before anything was patched. A library that cannot be read at
# all, being cut short, one whose bytes this user cannot read, one stripped of its section headers
# that links Fast-DDS, and one linking Fast-DDS by a link that leads to no file must fail the script
# instead, not be left unpatched.
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
if echo 'int main(void) { return 0; }' >"${WORK_DIR}/static.c" &&
  "${C_COMPILER}" -static -o "${LIB}/aaa_static" "${WORK_DIR}/static.c" 2>/dev/null; then
  STATIC="${LIB}/aaa_static"
  cp "${STATIC}" "${WORK_DIR}/static.orig"
else
  STATIC=""
  echo "No static executable among the stand-ins: ${C_COMPILER} cannot link one here"
fi
"${C_COMPILER}" -c -fPIC -o "${LIB}/aaa_object.so" "${WORK_DIR}/fast.c"
cp "${LIB}/aaa_object.so" "${WORK_DIR}/object.orig"
# Stripped of its section headers by zeroing its e_shnum, at byte 60 of a 64-bit ELF header and 48 of
# a 32-bit one
strip_section_headers() {
  local offset=60
  [ "$(od -An -tx1 -j 4 -N 1 "$1" | tr -d ' ')" = "01" ] && offset=48
  printf '\x00\x00' | dd of="$1" bs=1 seek="${offset}" conv=notrunc status=none
}
echo 'int libc_only(void) { return 7; }' >"${WORK_DIR}/libc_only.c"
"${C_COMPILER}" -shared -fPIC -o "${LIB}/aaa_stripped.so" "${WORK_DIR}/libc_only.c"
strip_section_headers "${LIB}/aaa_stripped.so"
cp "${LIB}/aaa_stripped.so" "${WORK_DIR}/stripped.orig"
# The ELF header's e_machine, at byte 18, made AArch64's (0xb7), or x86-64's (0x3e) on an AArch64 host
cp "${LIB}/libuser.so" "${LIB}/libforeign.so"
if [ "$(od -An -tx1 -j 18 -N 1 "${LIB}/libforeign.so" | tr -d ' ')" = "b7" ]; then
  printf '\x3e' | dd of="${LIB}/libforeign.so" bs=1 seek=18 conv=notrunc status=none
else
  printf '\xb7' | dd of="${LIB}/libforeign.so" bs=1 seek=18 conv=notrunc status=none
fi

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
if ldd "${LIB}/libforeign.so" >/dev/null 2>&1; then
  fail "ldd reads the stand-in built for another architecture, so it shows nothing"
fi

if ! OUTPUT="$("${SCRIPT}" "${LIB}" 2>&1)"; then
  fail "the script failed on a lib/ holding executables that link nothing. It said:"
  printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
fi
for library in libuser.so libforeign.so; do
  [ "$(needed "${LIB}/${library}")" = "libfaststandin.so.1.2.3" ] ||
    fail "${library} links '$(needed "${LIB}/${library}")', not the file libfaststandin.so.1.2.3"
done
for link in libfaststandin.so.1 libfaststandin.so; do
  [ ! -e "${LIB}/${link}" ] && [ ! -L "${LIB}/${link}" ] || fail "${link} was left in place"
done
[ "$(cat "${LIB}/aaa_script")" = "$(printf '#!/bin/sh\necho not a library')" ] || fail "aaa_script was changed"
if [ -n "${STATIC}" ] && ! cmp -s "${STATIC}" "${WORK_DIR}/static.orig"; then
  fail "aaa_static was changed"
fi
cmp -s "${LIB}/aaa_object.so" "${WORK_DIR}/object.orig" || fail "aaa_object.so was changed"
cmp -s "${LIB}/aaa_stripped.so" "${WORK_DIR}/stripped.orig" || fail "aaa_stripped.so was changed"

# A library that cannot be read -- its ELF header whole, the rest cut off -- fails the script, as
# leaving it unpatched would leave it linking Fast-DDS by a name another Fast-DDS can take
mkdir -p "${WORK_DIR}/broken"
head -c 64 "${WORK_DIR}/lib/libuser.so" >"${WORK_DIR}/broken/libbroken.so"
if OUTPUT="$("${SCRIPT}" "${WORK_DIR}/broken" 2>&1)" || [[ "${OUTPUT}" != *"Cannot read the libraries"* ]]; then
  fail "the script did not fail on a lib/ holding a library it cannot read, saying so. It said:"
  printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
fi

# A library stripped of its section headers, which patchelf takes for a packed executable linking
# nothing, but linking Fast-DDS: it fails the script rather than be passed over
mkdir -p "${WORK_DIR}/stripped"
cp "${WORK_DIR}/lib/libuser.so" "${WORK_DIR}/stripped/libuser.so"
patchelf --replace-needed libfaststandin.so.1.2.3 libfaststandin.so.1 "${WORK_DIR}/stripped/libuser.so"
ln -s libfaststandin.so.1.2.3 "${WORK_DIR}/stripped/libfaststandin.so.1"
cp "${WORK_DIR}/lib/libfaststandin.so.1.2.3" "${WORK_DIR}/stripped/"
strip_section_headers "${WORK_DIR}/stripped/libuser.so"
if OUTPUT="$("${SCRIPT}" "${WORK_DIR}/stripped" 2>&1)" || [[ "${OUTPUT}" != *"Cannot read the libraries"*"no section headers"* ]]; then
  fail "the script did not fail on a library stripped of its section headers that links Fast-DDS. It said:"
  printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
fi

# A library that cannot be read could link Fast-DDS by one of the links removed: it fails the script.
# Not where this user reads it all the same, as root does.
mkdir -p "${WORK_DIR}/unreadable"
cp "${WORK_DIR}/lib/libuser.so" "${WORK_DIR}/unreadable/libuser.so"
ln -s libfaststandin.so.1.2.3 "${WORK_DIR}/unreadable/libfaststandin.so.1"
cp "${WORK_DIR}/lib/libfaststandin.so.1.2.3" "${WORK_DIR}/unreadable/"
chmod 000 "${WORK_DIR}/unreadable/libuser.so"
if [ -r "${WORK_DIR}/unreadable/libuser.so" ]; then
  echo "Skipping the unreadable library case: this user reads whatever it is denied"
elif OUTPUT="$("${SCRIPT}" "${WORK_DIR}/unreadable" 2>&1)" || [[ "${OUTPUT}" != *"Cannot read"* ]]; then
  fail "the script did not fail on a lib/ holding a library it cannot read the bytes of, saying so. It said:"
  printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
fi
chmod 644 "${WORK_DIR}/unreadable/libuser.so"

# But where there is no Fast-DDS link to remove, a file that cannot be read can break nothing, and is
# passed over
mkdir -p "${WORK_DIR}/unreadable_alone"
cp "${WORK_DIR}/lib/libuser.so" "${WORK_DIR}/unreadable_alone/libuser.so"
chmod 000 "${WORK_DIR}/unreadable_alone/libuser.so"
if [ -r "${WORK_DIR}/unreadable_alone/libuser.so" ]; then
  echo "Skipping the unreadable library case with no Fast-DDS link: this user reads whatever it is denied"
elif ! OUTPUT="$("${SCRIPT}" "${WORK_DIR}/unreadable_alone" 2>&1)"; then
  fail "the script failed on a lib/ holding a file it cannot read and no Fast-DDS link. It said:"
  printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
fi
chmod 644 "${WORK_DIR}/unreadable_alone/libuser.so"

# A library linking Fast-DDS by a link that leads to no file -- in a directory there is and in one
# there is not -- or round in a loop, fails the script too, rather than have that library link a
# name made empty, or a file that is not there
for kind in dangling missing_dir loop; do
  mkdir -p "${WORK_DIR}/${kind}"
  # The stand-in patched above, made to link the Fast-DDS stand-in by its SONAME again
  cp "${WORK_DIR}/lib/libuser.so" "${WORK_DIR}/${kind}/libuser.so"
  patchelf --replace-needed libfaststandin.so.1.2.3 libfaststandin.so.1 "${WORK_DIR}/${kind}/libuser.so"
  case "${kind}" in
  dangling) ln -s libfaststandin.so.1.2.3 "${WORK_DIR}/${kind}/libfaststandin.so.1" ;;
  missing_dir) ln -s "${WORK_DIR}/nowhere/libfaststandin.so.1.2.3" "${WORK_DIR}/${kind}/libfaststandin.so.1" ;;
  loop) ln -s libfaststandin.so.1 "${WORK_DIR}/${kind}/libfaststandin.so.1" ;;
  esac
  if OUTPUT="$("${SCRIPT}" "${WORK_DIR}/${kind}" 2>&1)" || [[ "${OUTPUT}" != *"leads to no file"* ]]; then
    fail "the script did not fail on a library linking Fast-DDS by a ${kind} link, saying so. It said:"
    printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
  fi
  [ "$(needed "${WORK_DIR}/${kind}/libuser.so")" = "libfaststandin.so.1" ] ||
    fail "the library linking Fast-DDS by a ${kind} link now links '$(needed "${WORK_DIR}/${kind}/libuser.so")'"
done

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
