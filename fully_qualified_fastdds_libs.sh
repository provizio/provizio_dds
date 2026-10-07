#!/bin/bash

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

# Use as:
# fully_qualified_fastdds_libs.sh LIB_DIR_TO_PATCH [revert]

set -eu
set -o pipefail

# Bytes, not characters, wherever a file's content is read
export LC_ALL=C

# linux-gnu, but also linux-gnueabihf, linux-musl and the like
if [[ "${OSTYPE}" != linux* ]]; then
    echo "Only Linux is supported"
    exit 1
fi

LIB_DIR_TO_PATCH="$1"
REVERT="${2:-}"

if [[ "${REVERT}" == "revert" ]]; then
    echo "Adding softlinks for Fast-DDS libs in ${LIB_DIR_TO_PATCH}..."
    for file in "${LIB_DIR_TO_PATCH}"/libfast*.so.*.*.*; do
        if [[ ! -L "${file}" ]]; then
            file_basename="$(basename "${file}")"
            # Create the whole chain of softlinks down to the unversioned .so,
            # whatever the number of version components: Fast-DDS 2 libs use
            # 3 (e.g. libfastrtps.so.2.10.1) while Fast-DDS 3 libs use 4
            # (e.g. libfastdds.so.3.6.2.0)
            link_name="${file_basename}"
            while [[ "${link_name}" == *.so.* ]]; do
                link_name="${link_name%.*}"
                ln -fs "${file_basename}" "${LIB_DIR_TO_PATCH}/${link_name}"
            done
            echo "Softlinks added for ${file_basename}"
        fi
    done
else
    echo "Patching all libraries in ${LIB_DIR_TO_PATCH} to link Fast-DDS libs by fully qualified names..."

    # The names of the Fast-DDS links in this directory, by which a library is patched below, each a
    # pattern of grep's: whether a file names one, as a whole string, is told by
    # names_fast_dds_link <file>, with grep's status (0 it does, 1 it does not, 2 it cannot tell)
    fast_dds_links=()
    for link in "${LIB_DIR_TO_PATCH}"/libfast*.so*; do
        if [[ -L "${link}" ]]; then
            fast_dds_links+=(-e "$(basename "${link}")")
        fi
    done
    names_fast_dds_link() {
        [ "${#fast_dds_links[@]}" -gt 0 ] || return 1
        grep -qazxF "${fast_dds_links[@]}" "$1"
    }

    for file in "${LIB_DIR_TO_PATCH}"/*; do
        # Skip if the file is a soft link or not an executable/shared object
        if [ ! -L "${file}" ] && [ -f "${file}" ] && { [[ -x "${file}" ]] || [[ "${file}" == *.so* ]]; }; then
            # Only an ELF file links libraries, and a lib/ shared with other software (/usr/local/lib)
            # can hold others with the executable bit: scripts, say. Read by bash itself, so that no
            # tool missing or failing can make every file look like one of those.
            # What cannot be read cannot be told from a library linking Fast-DDS by one of the links
            # removed below, which would leave it unable to load. Where there are none, it can be
            # nothing of the kind.
            if [ ! -r "${file}" ]; then
                if [ "${#fast_dds_links[@]}" -eq 0 ]; then
                    continue
                fi
                echo "Cannot read ${file}, which could link Fast-DDS by one of the links this removes:" \
                    "make it readable to the user installing, or install as one who can read it" >&2
                exit 1
            fi
            # Up to a NUL byte, which bash cannot hold, so that none is dropped from what is compared
            magic=""
            IFS= read -r -d '' -n 4 magic <"${file}" || true
            if [ "${magic}" != $'\x7fELF' ]; then
                continue
            fi
            # The libraries it links as the file itself names them, read with patchelf rather than
            # resolved by ldd: ldd runs the host's loader, which reads no file built for another
            # architecture -- a cross-compiled install's -- though one can link Fast-DDS all the same.
            # What links nothing patchelf answers by failing, and says why: a statically linked
            # executable, one packed (by UPX, say) and so stripped of its section headers, which
            # patchelf takes for one, or an ELF file that is neither an executable nor a shared
            # library (an object file, say). A shared library stripped of its section headers draws
            # the packed one's answer too, though it links libraries all the same, so one naming a
            # Fast-DDS link of this directory -- as a whole string, as its table of names holds the
            # names it links -- is not passed over, nor one grep cannot tell of. Any other failure
            # would leave a library unpatched, so it ends the script, and the install with it.
            if ! needed="$(patchelf --print-needed "${file}" 2>/dev/null)"; then
                patchelf_error="$(patchelf --print-needed "${file}" 2>&1 >/dev/null || true)"
                names_link=0
                if [[ "${patchelf_error}" == *"no section headers"* ]]; then
                    names_fast_dds_link "${file}" || names_link=$?
                fi
                if [[ "${patchelf_error}" == *"cannot find section"*dynamic* ||
                    "${patchelf_error}" == *"wrong ELF type"* ]] ||
                    { [[ "${patchelf_error}" == *"no section headers"* ]] && [ "${names_link}" -eq 1 ]; }; then
                    echo "Skipping ${file}, which links nothing: ${patchelf_error}"
                    continue
                fi
                echo "Cannot read the libraries ${file} links: ${patchelf_error}" >&2
                exit 1
            fi
            # Each Fast-DDS library linked by a name that is a link in this directory, which goes below
            while read -r lib_basename; do
                lib="${LIB_DIR_TO_PATCH}/${lib_basename}"
                if [[ "${lib_basename}" == libfast*.so* && -L "${lib}" ]]; then
                    # A link that leads nowhere, or round in a loop, would otherwise make an empty name
                    # of the library linked, and the install succeed with a library that cannot load
                    if ! lib_full_path="$(realpath "${lib}")" || [ ! -f "${lib_full_path}" ]; then
                        echo "${lib}, which ${file} links, leads to no file" >&2
                        exit 1
                    fi
                    lib_full_name="$(basename "${lib_full_path}")"

                    echo "Patching ${file} so it links against ${lib_full_name} instead of ${lib_basename}"
                    patchelf --replace-needed "${lib_basename}" "${lib_full_name}" "${file}"
                fi
            done <<<"${needed}"
        fi
    done

    # Now remove the extra softlinks
    for file in "${LIB_DIR_TO_PATCH}"/libfast*.so*; do
        if [[ -L "${file}" ]]; then
            rm "${file}"
            echo "Extra softlink ${file} removed"
        fi
    done
fi
