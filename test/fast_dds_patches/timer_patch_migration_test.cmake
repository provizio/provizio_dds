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

# Migration coverage for cmake/fast_dds/resource_event_per_timer_wait.cmake.
#
# That script runs as the Fast-DDS ExternalProject PATCH_COMMAND, so it meets not only pristine
# upstream sources but whatever an EXISTING build tree was left holding by an earlier revision
# of itself -- and it has to carry each of those forward without a developer deleting the tree.
# Getting that wrong is invisible in a fresh build and in CI, both of which start pristine; it
# surfaces only on a machine that has built this branch before, as a configure that fails
# blaming Fast-DDS for a shape change that never happened. That is exactly what shipped once:
# the sweep the current revision adds was written BEFORE the revision marker existed, so a tree
# patched in that window already had it while still carrying the plain marker, and the
# migration -- which decided from the marker alone -- looked for a tail that was no longer
# there.
#
# So the invariant under test is convergence: every state an earlier revision could have left
# behind must migrate to a tree byte-identical to the one a fresh application produces.
#
# Deliberately no git, and no pristine upstream copy. The build tree's sources are already at
# the current revision by the time tests run, which is the one fixture this needs: the fresh
# path is proven by the build itself, while the states worth testing here are derived from that
# tree. Everything happens on copies -- the real build tree is never written to.
#
# Invoked as:
#   cmake -DPATCH_SCRIPT=<path> -DFAST_DDS_SOURCE_DIR=<path> -DWORK_DIR=<scratch dir>
#         -P timer_patch_migration_test.cmake

foreach(_var IN ITEMS PATCH_SCRIPT FAST_DDS_SOURCE_DIR WORK_DIR)
    if(NOT DEFINED ${_var})
        message(FATAL_ERROR "timer_patch_migration_test.cmake: ${_var} must be defined")
    endif()
endforeach()

set(_relative_sources
    "src/cpp/rtps/resources/ResourceEvent.h"
    "src/cpp/rtps/resources/ResourceEvent.cpp"
    "src/cpp/rtps/reader/WriterProxy.cpp")

# The markers the script itself uses, read out of it rather than restated here, so a revision
# bump does not silently leave this test asserting against the previous one.
file(READ "${PATCH_SCRIPT}" _script)
if(NOT _script MATCHES "set\\(_revision \"([0-9]+)\"\\)")
    message(FATAL_ERROR "timer_patch_migration_test: could not read _revision from ${PATCH_SCRIPT}")
endif()
set(_revision "${CMAKE_MATCH_1}")
set(_marker "[provizio_dds]")
set(_revision_marker "[provizio_dds r${_revision}]")

# Lay out one tree of copies under WORK_DIR/<name>/ and return the -D arguments for it.
# _from is either FAST_DDS_SOURCE_DIR (whose files sit at their full relative paths) or a
# previously staged tree under WORK_DIR (whose files sit flat, by leaf name).
function(_stage_from _from _name _out_args)
    foreach(_relative IN LISTS _relative_sources)
        get_filename_component(_leaf "${_relative}" NAME)
        if(EXISTS "${_from}/${_relative}")
            configure_file("${_from}/${_relative}" "${WORK_DIR}/${_name}/${_leaf}" COPYONLY)
        else()
            configure_file("${_from}/${_leaf}" "${WORK_DIR}/${_name}/${_leaf}" COPYONLY)
        endif()
    endforeach()
    set(${_out_args}
        "-DRESOURCE_EVENT_H=${WORK_DIR}/${_name}/ResourceEvent.h"
        "-DRESOURCE_EVENT_CPP=${WORK_DIR}/${_name}/ResourceEvent.cpp"
        "-DWRITER_PROXY_CPP=${WORK_DIR}/${_name}/WriterProxy.cpp"
        PARENT_SCOPE)
endfunction()

macro(_stage _name _out_args)
    _stage_from("${FAST_DDS_SOURCE_DIR}" "${_name}" ${_out_args})
endmacro()

macro(_stage_from_reference _name _out_args)
    _stage_from("${WORK_DIR}/reference" "${_name}" ${_out_args})
endmacro()

function(_apply_patch _name _args)
    execute_process(COMMAND "${CMAKE_COMMAND}" ${_args} -P "${PATCH_SCRIPT}"
                    RESULT_VARIABLE _result OUTPUT_VARIABLE _stdout ERROR_VARIABLE _stderr)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR
            "timer_patch_migration_test: the patch failed on the '${_name}' tree, which an "
            "earlier revision of the script could legitimately have left behind. A developer "
            "whose Fast-DDS tree is in that state cannot configure at all.\n${_stdout}${_stderr}")
    endif()
endfunction()

# Byte-identity against the reference, which is what "the migration carried this tree forward
# correctly" means -- a run that merely SUCCEEDS could have left the fix half applied.
function(_expect_same_as_reference _name)
    foreach(_relative IN LISTS _relative_sources)
        get_filename_component(_leaf "${_relative}" NAME)
        file(READ "${WORK_DIR}/${_name}/${_leaf}" _actual)
        file(READ "${WORK_DIR}/reference/${_leaf}" _expected)
        if(NOT _actual STREQUAL _expected)
            message(FATAL_ERROR
                "timer_patch_migration_test: after migrating the '${_name}' tree, ${_leaf} "
                "differs from what a tree already at revision ${_revision} holds. The states "
                "the patch can meet must all converge on one result.")
        endif()
    endforeach()
endfunction()

file(REMOVE_RECURSE "${WORK_DIR}")

# The reference: a staged copy with the patch applied to it, NOT the build tree as found. The
# build tree's own state depends on when its patch step last ran relative to a change in the
# script -- mid-rewrite it can legitimately be half-stamped -- and a reference that moves with
# it would make this test assert against whatever it happened to catch. Applying the patch here
# defines the reference as "what this script produces", which is the thing every other case has
# to converge on.
_stage(reference _reference_args)
_apply_patch(reference "${_reference_args}")
foreach(_relative IN LISTS _relative_sources)
    get_filename_component(_leaf "${_relative}" NAME)
    file(READ "${WORK_DIR}/reference/${_leaf}" _reference_contents)
    string(FIND "${_reference_contents}" "${_revision_marker}" _reference_pos)
    string(FIND "${_reference_contents}" "${_marker}" _reference_plain_pos)
    if(_reference_pos EQUAL -1 OR NOT _reference_plain_pos EQUAL -1)
        message(FATAL_ERROR
            "timer_patch_migration_test: after patching, ${_leaf} must carry "
            "'${_revision_marker}' and no bare '${_marker}'. Every file the script writes is "
            "revision-stamped, so that a later bump can tell this tree from the one before it "
            "-- and so that the demote-the-tag cases below genuinely migrate each file rather "
            "than comparing one that was identical by construction.")
    endif()
endforeach()

# 1. Idempotent: re-running on a tree already at this revision changes nothing. This is what
#    every incremental build does, since the patch step re-runs whenever a script changes.
_stage_from_reference(idempotent _idempotent_args)
_apply_patch(idempotent "${_idempotent_args}")
_expect_same_as_reference(idempotent)

# 2. The state the revision before the marker left behind: this revision's own output, but
#    tagged with the plain marker, because the tag did not exist when that revision wrote it.
#    Synthesised by demoting the tag, which IS the whole difference between the two -- and the
#    case that used to abort the configure. Every file the script writes carries the marker, so
#    every file is genuinely migrated here rather than being identical by construction.
_stage_from_reference(untagged _untagged_args)
foreach(_relative IN LISTS _relative_sources)
    get_filename_component(_leaf "${_relative}" NAME)
    file(READ "${WORK_DIR}/untagged/${_leaf}" _contents)
    string(FIND "${_contents}" "${_revision_marker}" _marker_pos)
    if(_marker_pos EQUAL -1)
        message(FATAL_ERROR
            "timer_patch_migration_test: ${_leaf} carries no '${_revision_marker}', so demoting "
            "the tag changes nothing and this case would assert against a file identical to the "
            "reference by construction. Every file the patch writes must be revision-stamped.")
    endif()
    string(REPLACE "${_revision_marker}" "${_marker}" _contents "${_contents}")
    file(WRITE "${WORK_DIR}/untagged/${_leaf}" "${_contents}")
endforeach()
_apply_patch(untagged "${_untagged_args}")
_expect_same_as_reference(untagged)

# 3. A MIXED tree: stamped in one place and not in others, which is what a script that gated on
#    the revision marker's mere presence produced. Reproduced by demoting every tag except the
#    one inside the block whose content changed with the revision -- so the file looks "already
#    at this revision" to a naive check while most of it is still marked by the older one.
#    Taking that for done left the tree mixed for good.
_stage_from_reference(mixed _mixed_args)
foreach(_relative IN LISTS _relative_sources)
    get_filename_component(_leaf "${_relative}" NAME)
    file(READ "${WORK_DIR}/mixed/${_leaf}" _contents)
    string(REPLACE "${_revision_marker}" "${_marker}" _demoted "${_contents}")
    # Put the first tag back, so the file carries both forms.
    string(FIND "${_demoted}" "${_marker}" _first)
    string(LENGTH "${_marker}" _marker_length)
    math(EXPR _after "${_first} + ${_marker_length}")
    string(SUBSTRING "${_demoted}" 0 "${_first}" _head)
    string(SUBSTRING "${_demoted}" "${_after}" -1 _tail)
    file(WRITE "${WORK_DIR}/mixed/${_leaf}" "${_head}${_revision_marker}${_tail}")
endforeach()
_apply_patch(mixed "${_mixed_args}")
_expect_same_as_reference(mixed)

# 4. A tree stamped by a DIFFERENT revision -- which, the moment _revision is next bumped, is
#    every existing build tree. Synthesised by rewriting this revision's tag to a neighbouring
#    one, since that is exactly what such a tree carries.
#
#    This is the case the earlier fixtures cannot reach: they only ever demote to the bare
#    marker, so a check that recognised "bare" and "current" and nothing else passed them all
#    while failing every real tree on the next bump -- falling through to the pristine-source
#    branch and aborting the configure blaming Fast-DDS for a shape change that had not
#    happened.
math(EXPR _other_revision "${_revision} + 1")
set(_other_marker "[provizio_dds r${_other_revision}]")
_stage_from_reference(other_revision _other_revision_args)
foreach(_relative IN LISTS _relative_sources)
    get_filename_component(_leaf "${_relative}" NAME)
    file(READ "${WORK_DIR}/other_revision/${_leaf}" _contents)
    string(REPLACE "${_revision_marker}" "${_other_marker}" _contents "${_contents}")
    file(WRITE "${WORK_DIR}/other_revision/${_leaf}" "${_contents}")
endforeach()
_apply_patch(other_revision "${_other_revision_args}")
_expect_same_as_reference(other_revision)

file(REMOVE_RECURSE "${WORK_DIR}")
message(STATUS "timer_patch_migration: PASS (revision ${_revision}: idempotent, and the "
               "pre-marker tree migrates to the same result)")
