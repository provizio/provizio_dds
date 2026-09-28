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

# How the patch scripts in this directory read and write the Fast-DDS sources they edit: the one
# way every one of them does it, so that neither the line endings of a checkout nor those of the
# host decide whether a patch applies, or what it leaves behind.
#
# Both line endings are to be expected. Fast-DDS' .gitattributes marks its sources `text`, so git
# checks them out in the machine's own style -- CRLF under core.autocrlf=true, as a Windows
# machine commonly has it -- and provizio_dds's own .gitattributes sets no text or eol attribute, so
# on such a machine the scripts themselves come out CRLF as well. What makes that harmless is CMake, in ways it does not
# document, so they are stated here as measured (CMake 3.15.7 and 4.4.3 on Linux, 4.2.3 on
# Windows):
#
#   - file(READ) without HEX drops the CR of a CRLF: it reads line by line through KWSys'
#     GetLineFromStream, which strips a CR that ends a line. A CR anywhere else is content, and
#     stays.
#   - A bracket argument -- every anchor and replacement text here -- holds LF line endings even
#     in a script checked out CRLF.
#   - file(WRITE) writes a text-mode stream on Windows: every LF becomes CRLF there, and a CRLF
#     becomes CR CR LF. On other hosts it writes the bytes it is given.
#
# So anchors are matched as LF text whatever the checkout, and a patched file comes out in the
# host's own line ending, uniformly, whatever it had before: CRLF on Windows, LF elsewhere. The
# compilers accept either. What they would not survive is a file with doubled or mixed line
# endings, and that is what the two functions below rule out, on top of the behaviour above:
#
#   provizio_dds_patch_read() removes every run of CRs before an LF itself. On Linux, file(READ)
#   of a line ending CR CR LF keeps one CR, which would leave that line matching no anchor and
#   surviving as the one CRLF line of an otherwise LF file.
#
#   provizio_dds_patch_write() refuses text that still carries a CRLF -- the one input file(WRITE)
#   corrupts -- writes to a temporary file, checks that it reads back as the text it was given in
#   a single line ending, and only then renames it over the target. A write that fails, or is
#   interrupted, leaves the original in place rather than a half-written source that the next
#   configure would blame on a change in Fast-DDS.
#
# The fast_dds_patch_line_endings test applies every patch script to LF and to CRLF sources, from
# LF and from CRLF copies of the scripts, on whichever host CI runs it.
#
# Included by the patch scripts as include("${CMAKE_CURRENT_LIST_DIR}/patch_io.cmake").

# Read <path> into <out_var> as LF text: every run of CRs before an LF removed, any other CR kept.
function(provizio_dds_patch_read path out_var)
    file(READ "${path}" _contents)
    string(REGEX REPLACE "\r+\n" "\n" _contents "${_contents}")
    set(${out_var} "${_contents}" PARENT_SCOPE)
endfunction()

# Replace <path> with <contents>, which must be LF text such as provizio_dds_patch_read returns.
# FATAL_ERROR, with <path> left as it was, if the result would not be <contents> in one line ending.
function(provizio_dds_patch_write path contents)
    string(FIND "${contents}" "\r\n" _crlf_pos)
    if(NOT _crlf_pos EQUAL -1)
        message(FATAL_ERROR
            "provizio_dds_patch_write: refusing to write text containing CRLF to ${path}: on Windows "
            "file(WRITE) turns each of them into CR CR LF. Read it with provizio_dds_patch_read.")
    endif()

    # Beside the target, so the rename cannot cross file systems, and under a short fixed name rather
    # than the target's own plus a suffix: that would make it the longest path the patch step
    # touches, and a build directory deep enough to put it past Windows' 260 characters would fail
    # where the target itself does not. No two writes overlap: the scripts run one after another,
    # each writing its files in turn.
    get_filename_component(_directory "${path}" DIRECTORY)
    if(_directory STREQUAL "")
        # A bare file name, as a script run by hand may be given, is one in the current directory,
        # which a temporary made of the empty directory and a slash would not be: it would be one
        # in the root of the file system.
        set(_directory ".")
    endif()
    set(_temporary "${_directory}/provizio_dds_patch.tmp")
    file(WRITE "${_temporary}" "${contents}")

    # It must read back as the same text...
    provizio_dds_patch_read("${_temporary}" _read_back)
    string(COMPARE EQUAL "${_read_back}" "${contents}" _same_text)
    # ...and in one line ending: exactly the text's own size (LF throughout), or one byte more per
    # line (CRLF throughout). A CR CR LF, or a mix of the two, is neither.
    file(SIZE "${_temporary}" _size)
    string(LENGTH "${contents}" _length)
    string(REPLACE "\n" "" _without_lf "${contents}")
    string(LENGTH "${_without_lf}" _length_without_lf)
    math(EXPR _crlf_size "2 * ${_length} - ${_length_without_lf}")
    unset(_problem)  # a function sees its caller's variables, and this one must start out undefined
    if(NOT _same_text)
        set(_problem "does not read back as the text it was given")
    elseif(NOT (_size EQUAL _length OR _size EQUAL _crlf_size))
        set(_problem "is ${_size} bytes, where one line ending throughout would make it ${_length} (LF) or "
                     "${_crlf_size} (CRLF)")
    endif()
    if(DEFINED _problem)
        file(REMOVE "${_temporary}")
        string(CONCAT _problem ${_problem})
        message(FATAL_ERROR
            "provizio_dds_patch_write: writing ${path} failed -- the file written ${_problem}. Nothing "
            "was changed. This is a problem with CMake's file(WRITE) on this host, not with the Fast-DDS "
            "sources.")
    endif()

    file(RENAME "${_temporary}" "${path}")
endfunction()
