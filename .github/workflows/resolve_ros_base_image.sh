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

# Decides, per ROS 2 distro, which image the compatibility matrix takes its container from.
#
# Docker Hub rate-limits UNAUTHENTICATED pulls per source IP over a window hours long, and answers
# anything past the limit with an immediate HTTP 429 that no amount of retrying can wait out. The
# ROS 2 matrix pulls a ros:<distro> image once per job from hosted runners whose egress IP is
# shared with every other customer, so it is throttled by a limit it cannot influence. This org
# mirrors the same images into its own GHCR namespace, which is not subject to that limit.
#
# The job's container is chosen by the runner before any of its steps run, so the choice cannot be
# made inside the job that needs it -- hence this script, run by a job the matrix depends on, which
# writes the answer to GITHUB_OUTPUT as a distro -> image map.
#
# Reads from the environment (all set by ci.yml):
#   ROS_DISTROS               - space-separated distros to resolve; also the matrix's own list
#   MIRROR_REPO               - the preferred repository, e.g. ghcr.io/<org>/ros
#   FALLBACK_REPO             - where to go when the mirror cannot serve a distro, e.g.
#                               mirror.gcr.io/library/ros. Empty means there is no second choice:
#                               a distro the mirror cannot serve is then handed to the matrix as the
#                               mirror's own tag, unpinned, for its jobs to pull -- and to fail on
#                               if the mirror truly cannot serve it -- while every other distro's
#                               jobs run as usual
#   CONTAINER_REGISTRY_PREFIX - non-empty when the operator has pinned the images to a registry of
#                               their own, which also clears FALLBACK_REPO. Applied here rather
#                               than in the workflow because GitHub's "a && b || c" expression
#                               idiom cannot yield an empty string: '' is falsy, so the "|| c" arm
#                               wins and a workflow expression can never clear a value this way
#   REQUIRED_PLATFORM         - the platform the matrix's runner needs, default linux/amd64
#   GHCR_USER                 - actor to authenticate to ghcr.io as
#   GHCR_TOKEN                - that actor's token. Only needed while the mirror package is
#                               private, and only used when the mirror is on ghcr.io at all

set -eu
set -o pipefail

: "${ROS_DISTROS:?the distros to resolve must be set}"
: "${MIRROR_REPO:?a preferred repository must be set}"
FALLBACK_REPO="${FALLBACK_REPO:-""}"
CONTAINER_REGISTRY_PREFIX="${CONTAINER_REGISTRY_PREFIX:-""}"
REQUIRED_PLATFORM="${REQUIRED_PLATFORM:-"linux/amd64"}"
GHCR_USER="${GHCR_USER:-""}"
GHCR_TOKEN="${GHCR_TOKEN:-""}"

# Pinning the images to one registry means taking them from there and from there ONLY, so it also
# clears the fallback: a distro that registry cannot serve then fails its own jobs instead of
# quietly redirecting the matrix at Docker Hub. An air-gapped or policy-restricted setup pins the
# registry precisely so that no such redirect can happen.
if [ -n "${CONTAINER_REGISTRY_PREFIX}" ]; then
    FALLBACK_REPO=""
fi

# The mirror this organisation keeps, which the "Mirror ROS base images" workflow of
# provizio_radar_api_ros2 populates. Only for telling apart, in the warning at the end, a mirror that
# workflow can repopulate from a fork's own namespace, which nothing populates.
readonly ORGANISATION_MIRROR_REPO="ghcr.io/provizio/ros"

# Everything below ends up either in a container image reference the runner executes or in a
# GITHUB_OUTPUT line and a ::warning workflow command. All three are parsed, none is escaped, and
# the distro list additionally reaches "run:" blocks through the matrix - so the shapes are pinned
# here, at the one place they enter the workflow, rather than escaped at each use.
readonly DISTRO_PATTERN='^[a-z][a-z0-9]*$'
readonly REPO_PATTERN='^[a-z0-9][a-z0-9.-]*(:[0-9]+)?(/[a-z0-9._-]+)+$'

# A repository name is lowercase by convention and a GitHub login is not: github.repository_owner
# reports the case its owner registered with, so MIRROR_REPO arrives capitalised for any fork whose
# owner has a capital letter in their login. Lowercasing is what a container tool does with the
# name anyway, and doing it here is what keeps such a fork's matrix from being rejected below.
MIRROR_REPO_AS_GIVEN="${MIRROR_REPO}"
MIRROR_REPO="${MIRROR_REPO,,}"
FALLBACK_REPO="${FALLBACK_REPO,,}"

# Anything echoed here that this script did not produce - a repository name an operator set, a
# registry's own error text, a platform a manifest states - is text, not a workflow command. The
# runner reads a line beginning "::name::value" as one, and it ends a line at a CR as well as at a
# LF, so a value carrying either could open such a line of its own (::error::, ::add-mask::,
# ::stop-commands::). Values reported inline have both removed; multi-line output worth keeping as
# such has every line prefixed instead, a CR counting as a line break there too. The runner also
# still reads the older "##[name]value" form, and that one anywhere in a line, so it is broken up
# wherever it appears, in either kind of output: a "##[stop-commands]" there would otherwise
# silence the warnings this script exists to raise.
without_legacy_commands() {
    sed 's/##\[/# #[/g'
}
single_line() {
    printf '%s' "$1" | tr -d '\r\n' | without_legacy_commands
}
prefixed_lines() {
    tr '\r' '\n' <"$1" | without_legacy_commands | sed 's/^/    | /'
}

# How the mirror can be read, if at all: "authenticated", "anonymous", "auth-failed" or
# "malformed". Only the first two are consulted, and each of the other two needs a different thing
# done about it, so they are reported apart rather than as one "no image" outcome.
mirror_state="anonymous"

# The mirror as messages name it. Once it has matched REPO_PATTERN it cannot carry a line break, and
# until then only the single-line form of what was given may be printed.
mirror_shown="${MIRROR_REPO}"

if ! [[ "${MIRROR_REPO}" =~ ${REPO_PATTERN} ]]; then
    # Reported as it was given rather than as it was lowercased: this is the one message whose job
    # is to help whoever set the value find their mistake in it. Only an operator's
    # CONTAINER_REGISTRY_PREFIX gets here -- the name ci.yml builds otherwise, from the repository
    # owner's login, always matches -- and that prefix also clears the fallback, so this is a
    # configuration error of the whole matrix rather than of any one distro: no image reference can
    # be made from the name for any of them.
    mirror_shown="$(single_line "${MIRROR_REPO_AS_GIVEN}")"
    echo "MIRROR_REPO is not a plain registry/namespace/name: '${mirror_shown}'"
    mirror_state="malformed"
    if [ -z "${FALLBACK_REPO}" ]; then
        echo "::error title=ROS base image mirror unusable::'${mirror_shown}' is not a usable repository name, so no ROS 2 base image can be taken from it, and no fallback is configured. Check the CONTAINER_REGISTRY_PREFIX variable."
        exit 1
    fi
fi
if [ -n "${FALLBACK_REPO}" ] && ! [[ "${FALLBACK_REPO}" =~ ${REPO_PATTERN} ]]; then
    # This one is fatal: ci.yml sets it to a literal, so a malformed value is a broken workflow
    # rather than a property of whoever is running it, and there is nothing to fall back to.
    echo "FALLBACK_REPO is not a plain registry/namespace/name: '$(single_line "${FALLBACK_REPO}")'"
    exit 1
fi

# stderr of each manifest read is kept so that a failure can say what the registry said, and the
# ghcr.io credential is written to this runner's docker config, so drop both however the script
# leaves. The hosted runner this job pins is discarded anyway; the trap is what keeps that from
# being load-bearing if the job is ever moved to one of the self-hosted runners.
inspect_errors="$(mktemp)"
logged_in_to_ghcr=""
cleanup() {
    rm -f "${inspect_errors}"
    if [ -n "${logged_in_to_ghcr}" ]; then
        docker logout ghcr.io >/dev/null 2>&1 || true
    fi
}
# EXIT alone, and that covers a signal too: bash runs the EXIT trap when INT or TERM ends the script.
# Trapping those two as well would do worse than nothing, replacing their default action -- ending
# the script -- with running cleanup and then carrying on, logged out of the mirror halfway through.
trap cleanup EXIT

# The mirror is a private package of this organisation, so it has to be read as somebody.
# Skipped outright when the mirror has been pointed somewhere other than ghcr.io, where a GitHub
# token means nothing - and there "anonymous" is simply how such a registry is read.
if [ "${mirror_state}" = "anonymous" ] && [ "${MIRROR_REPO#ghcr.io/}" != "${MIRROR_REPO}" ]; then
    if [ -z "${GHCR_TOKEN}" ]; then
        # Said out loud: with no credential this job can only see the mirror if the package is
        # public, so it is the first thing to check when the reads below come back empty-handed.
        echo "No ghcr.io credential was given, so ${MIRROR_REPO} can only be read anonymously."
    else
        if printf '%s' "${GHCR_TOKEN}" | docker login ghcr.io -u "${GHCR_USER}" --password-stdin >/dev/null 2>&1; then
            # Only once the login has succeeded: logging out after a failed one would erase a
            # ghcr.io credential that was in this runner's docker config before, not one of ours.
            logged_in_to_ghcr="yes"
            mirror_state="authenticated"
        else
            echo "Could not authenticate to ghcr.io as '$(single_line "${GHCR_USER}")'."
            mirror_state="auth-failed"
        fi
    fi
fi

# The manifest of <reference>, on stdout, with what the registry said about it in inspect_errors.
# Asked again after a failure unless the answer was one, rather than a hiccup: a tag the registry does
# not have, or a credential it refuses, comes back the same however often it is asked, whereas a 5xx
# or a dropped connection passed on as it is would send the distro to the fallback -- rate-limited
# Docker Hub content -- or, where there is none, fail its jobs, over one bad second.
readonly INSPECT_ATTEMPTS=3
inspect_manifest() {
    local attempt=1 output
    while :; do
        if output="$(docker buildx imagetools inspect "$1" 2>"${inspect_errors}")"; then
            printf '%s\n' "${output}"
            return 0
        fi
        if [ "${attempt}" -ge "${INSPECT_ATTEMPTS}" ] ||
            grep -qiE 'not found|manifest unknown|unauthorized|denied|forbidden' "${inspect_errors}"; then
            return 1
        fi
        sleep $((attempt * 5))
        attempt=$((attempt + 1))
    done
}

distros_json=""
images_json=""
# Distros sent to the fallback, and, where there is none, those left to pull the mirror's tag unpinned
unavailable=""
unpinned=""

# Word splitting on ROS_DISTROS is wanted; globbing is not, or a "*" in the list would expand
# against the working directory and feed filenames into an image reference.
set -f
for distro in ${ROS_DISTROS}; do
    if ! [[ "${distro}" =~ ${DISTRO_PATTERN} ]]; then
        echo "'$(single_line "${distro}")' is not a plain ROS distro name; refusing to put it in an image"
        echo "reference or a matrix that interpolates it into shell."
        exit 1
    fi

    image=""
    if [ "${mirror_state}" = "authenticated" ] || [ "${mirror_state}" = "anonymous" ]; then
        # imagetools, not "docker pull": it reads the manifest and nothing else, so resolving five
        # distros costs a few HTTP requests rather than five image downloads on a runner that will
        # not use them.
        if inspected="$(inspect_manifest "${MIRROR_REPO}:${distro}")"; then
            # Pin what the matrix pulls to the digest resolved here, so all of its jobs run the
            # identical image even if the tag is re-pushed underneath them, and so the log records
            # exactly what was run.
            # A here-string rather than a pipe into awk: awk exits at the first match, which
            # closes the read end, and a printf killed by the resulting SIGPIPE would fail the
            # pipeline under pipefail and end this script with it.
            digest="$(awk '/^Digest:/ { print $2; exit }' <<<"${inspected}")"

            # Only platforms the manifest actually states can be checked. A multi-platform index
            # states one per image it carries, and the required one must be among them. A plain
            # single-image manifest states none at all, and is accepted rather than rejected: the
            # two mistakes are not equally expensive. An image that turns out to be built for
            # another architecture fails the job loudly when the runner pulls it, whereas
            # rejecting an image the mirror could have served sends all twenty jobs to Docker Hub
            # quietly - which is the one outcome this script exists to prevent.
            platforms="$(printf '%s\n' "${inspected}" |
                sed -n 's/^[[:space:]]*Platform:[[:space:]]*//p' | sed 's/[[:space:]]*$//')"

            if ! [[ "${digest}" =~ ^sha256:[0-9a-f]{64}$ ]]; then
                echo "${distro}: the mirror's manifest carries no usable digest"
            elif [ -n "${platforms}" ] && ! printf '%s\n' "${platforms}" | grep -qxF "${REQUIRED_PLATFORM}"; then
                echo "${distro}: ${MIRROR_REPO}:${distro} states no ${REQUIRED_PLATFORM} image; it states" \
                    "$(single_line "$(printf '%s' "${platforms}" | tr '\r\n' '  ')")"
            else
                image="${MIRROR_REPO}@${digest}"
                echo "${distro}: ${MIRROR_REPO}:${distro} -> ${image}"
            fi
        else
            # Say what the registry said. An empty mirror, a token that cannot read the package
            # and an unreachable registry each need a different thing done about them, and the
            # warning at the end of this script cannot tell them apart by itself.
            echo "${distro}: could not read ${MIRROR_REPO}:${distro}:"
            if [ -s "${inspect_errors}" ]; then
                prefixed_lines "${inspect_errors}"
            else
                echo "    (the command failed without saying why)"
            fi
        fi
    fi

    if [ -z "${image}" ]; then
        if [ -n "${FALLBACK_REPO}" ]; then
            unavailable="${unavailable}${distro} "
            image="${FALLBACK_REPO}:${distro}"
            echo "${distro}: ${image} (fallback)"
        else
            # No second choice was configured, so quietly running something else is not an option
            # this script gets to take -- and neither is failing the other distros' jobs over this
            # one, which exiting here would do: every job of the matrix needs this one's output.
            # So the distro gets the mirror's own tag, unpinned, and its jobs pull that, failing on
            # it if the mirror truly cannot serve it -- as they would with no resolver at all -- or
            # running as usual if it was only this read that failed.
            unpinned="${unpinned}${distro} "
            image="${MIRROR_REPO}:${distro}"
            echo "${distro}: ${image} (unpinned: no fallback is configured)"
        fi
    fi

    distros_json="${distros_json}${distros_json:+, }\"${distro}\""
    images_json="${images_json}${images_json:+, }\"${distro}\": \"${image}\""
done
set +f

# A post-condition on what is handed to the matrix, rather than a check on the input: an empty list
# would leave the matrix with no combinations at all, so every compatibility job would be skipped
# and the workflow would still conclude successfully, which is a worse outcome than any failure
# this script can report. The expansion at the top of this script already rejects an unset or empty
# ROS_DISTROS, so the only input that reaches this today is one made entirely of whitespace.
if [ -z "${distros_json}" ]; then
    echo "ROS_DISTROS named no distros, so the compatibility matrix would be empty."
    exit 1
fi

# A fallback that quietly papered over a broken mirror would be worse than no fallback at all:
# every job would keep passing while the thing meant to keep Docker Hub off the critical path sat
# empty, and nobody would find out until the matrix was throttled again. Say it on the job, and
# say which of the causes it was, because they need different things done about them.
if [ -n "${unavailable}" ]; then
    case "${mirror_state}" in
    auth-failed)
        warning_title="ROS base image mirror unreachable"
        warning_reason="Could not authenticate to ghcr.io as '$(single_line "${GHCR_USER}")', so ${MIRROR_REPO} was not consulted at all. The mirror itself may be fine; check the job's token and its packages: read permission."
        ;;
    malformed)
        warning_title="ROS base image mirror unusable"
        warning_reason="'${mirror_shown}' is not a usable repository name, so no mirror was consulted at all. Check the CONTAINER_REGISTRY_PREFIX variable."
        ;;
    anonymous)
        warning_title="ROS base image fallback used"
        warning_reason="${MIRROR_REPO} served no usable image, and this job held no credential for it, so it was read anonymously - which on its own explains the outcome if the package is a private one."
        ;;
    *)
        warning_title="ROS base image fallback used"
        if [ "${MIRROR_REPO}" = "${ORGANISATION_MIRROR_REPO}" ]; then
            warning_reason="${MIRROR_REPO} served no usable image. If it is merely missing them, run the 'Mirror ROS base images' workflow of provizio_radar_api_ros2 to repopulate it."
        else
            # With no CONTAINER_REGISTRY_PREFIX the mirror is named after the repository's owner, so
            # for a fork it is a namespace of the fork's own that nothing populates.
            warning_reason="${MIRROR_REPO} served no usable image. A fork has no mirror of its own unless one is set up: point the CONTAINER_REGISTRY_PREFIX variable at one to take the images from there only, or leave the fallback to serve them."
        fi
        ;;
    esac

    echo "::warning title=${warning_title}::${warning_reason} Affected: ${unavailable}- falling back to ${FALLBACK_REPO}, which serves Docker Hub content and is rate-limited per source IP. This job's log says what the registry answered for each distro."
fi

# Unpinned distros are only possible with no fallback, which is an operator's pinned registry
if [ -n "${unpinned}" ]; then
    echo "::warning title=ROS base image not pinned::${MIRROR_REPO} served no usable image for: ${unpinned}- and no fallback is configured, so their jobs pull the tag from it unpinned, and fail there if it cannot serve them. The other distros' jobs are unaffected. This job's log says what the registry answered for each distro."
fi

# Both values are single-line by construction: every distro matched DISTRO_PATTERN and every
# repository REPO_PATTERN, so neither can carry a newline that would inject further outputs.
if [ -n "${GITHUB_OUTPUT:-}" ]; then
    {
        echo "distros=[${distros_json}]"
        echo "images={${images_json}}"
    } >>"${GITHUB_OUTPUT}"
fi

echo "distros=[${distros_json}]"
echo "images={${images_json}}"
