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

# Coverage for .github/workflows/resolve_ros_base_image.sh, which decides which image each ROS 2
# distro's compatibility jobs run in. Every job of that matrix needs its output, so what it does
# with a registry that cannot serve a distro, answers badly or says something hostile decides
# whether the other distros are tested at all.
#
# The script runs as CI runs it, with docker and sleep replaced by stand-ins on PATH: docker answers
# each image reference from a script of outcomes the case sets up (a manifest, a 503, "not found",
# "denied", an error text carrying a CR or a workflow command of the runner's older form), and sleep
# returns at once, so the retries cost nothing.
#
# Use as: resolve_ros_base_image_test.sh <REPO_ROOT> <WORK_DIR>
#   REPO_ROOT  the repository holding .github/workflows/resolve_ros_base_image.sh
#   WORK_DIR   an absolute scratch path, created and removed by this script

set -eu
set -o pipefail

if [ "$#" -ne 2 ]; then
  echo "Use as: $(basename "$0") <REPO_ROOT> <WORK_DIR>"
  exit 1
fi

REPO_ROOT="$(cd "$1" && pwd -P)"
WORK_DIR="$2"
RESOLVER="${REPO_ROOT}/.github/workflows/resolve_ros_base_image.sh"

# WORK_DIR is deleted outright below, so refuse anything that is not an absolute path well inside a
# filesystem: a relative or empty one would delete whatever the caller happened to be standing in.
case "${WORK_DIR}" in
/?*/?*) ;;
*)
  echo "WORK_DIR must be an absolute path at least two levels deep, got '${WORK_DIR}'"
  exit 1
  ;;
esac
trap 'rm -rf "${WORK_DIR}"' EXIT
rm -rf "${WORK_DIR}"
# Not writable by other users: the stand-ins in it are first on PATH while the resolver runs, so
# anything droppable there would be run by this test in place of docker. The mode is given to mkdir
# rather than applied afterwards, which would leave a window with whatever the umask allows, and
# per level, as -p applies -m to the last component only.
mkdir -p "$(dirname "${WORK_DIR}")"
mkdir -m 700 "${WORK_DIR}"
mkdir -m 700 "${WORK_DIR}/bin"

DISTROS="humble iron jazzy kilted lyrical"

# docker, as the resolver uses it. "buildx imagetools inspect <reference>" takes the next outcome
# from ${STUB_DIR}/<reference with / and : as _>, one per line, the last repeating; every call is
# logged to ${STUB_DIR}/calls.
cat >"${WORK_DIR}/bin/docker" <<'STUB'
#!/bin/bash
set -eu
echo "$*" >>"${STUB_DIR}/calls"
case "$1" in
login)
  cat >/dev/null
  [ "${STUB_LOGIN_RESULT:-ok}" = "ok" ]
  exit
  ;;
logout)
  exit 0
  ;;
buildx) ;;
*)
  echo "docker stand-in: unexpected command: $*" >&2
  exit 2
  ;;
esac
reference="$4"
script="${STUB_DIR}/$(printf '%s' "${reference}" | tr '/:' '__')"
counter="${script}.count"
count=0
[ -f "${counter}" ] && count="$(cat "${counter}")"
count=$((count + 1))
echo "${count}" >"${counter}"
outcome="$(sed -n "${count}p" "${script}" 2>/dev/null)"
[ -n "${outcome}" ] || outcome="$(tail -n 1 "${script}" 2>/dev/null)"
[ -n "${outcome}" ] || outcome="notfound"
# One digest per reference: its bytes in hex, padded, as a registry's digest is 64 hex digits
hex="$(printf '%s' "${reference}" | od -An -tx1 | tr -d ' \n')$(printf '0%.0s' {1..64})"
digest="sha256:${hex:0:64}"
case "${outcome}" in
ok | arm64-only | platform-injection)
  platform="linux/amd64"
  [ "${outcome}" = "arm64-only" ] && platform="linux/arm64/v8"
  [ "${outcome}" = "platform-injection" ] && platform="linux/arm64 ##[add-mask]from-the-manifest"
  cat <<MANIFEST
Name:      ${reference}
MediaType: application/vnd.oci.image.index.v1+json
Digest:    ${digest}

Manifests:
  Name:        ${reference}@sha256:$(printf 'a%.0s' {1..64})
  MediaType:   application/vnd.oci.image.manifest.v1+json
  Platform:    ${platform}

  Name:        ${reference}@sha256:$(printf 'b%.0s' {1..64})
  MediaType:   application/vnd.oci.image.manifest.v1+json
  Platform:    unknown/unknown
  Annotations:
    vnd.docker.reference.type: attestation-manifest
MANIFEST
  ;;
503)
  echo "ERROR: unexpected status from HEAD request to https://registry/v2/${reference}: 503 Service Unavailable" >&2
  exit 1
  ;;
notfound)
  echo "ERROR: ${reference}: not found" >&2
  exit 1
  ;;
denied)
  echo "ERROR: denied: permission_denied: read_package" >&2
  exit 1
  ;;
cr-injection)
  printf 'ERROR: 500 Oops\r::error title=Injected::this annotation came from the registry\n' >&2
  exit 1
  ;;
legacy-injection)
  echo "ERROR: 500 Oops ##[stop-commands]from-the-registry" >&2
  exit 1
  ;;
term)
  # Signals the resolver itself, as a Ctrl-C or a timeout(1) would, then fails like a read cut short
  kill -TERM "$(cat "${STUB_DIR}/resolver.pid")"
  exit 1
  ;;
esac
STUB
cat >"${WORK_DIR}/bin/sleep" <<'STUB'
#!/bin/bash
echo "sleep $*" >>"${STUB_DIR}/calls"
STUB
chmod 755 "${WORK_DIR}/bin/docker" "${WORK_DIR}/bin/sleep"

FAILURES=0
CASE=""
fail() {
  echo "FAILED (${CASE}): $*"
  FAILURES=$((FAILURES + 1))
}

# Sets up a case: a fresh stand-in state and output file, and every distro served by <mirror>
# unless the case says otherwise with serve.
new_case() {
  CASE="$1"
  CASE_DIR="${WORK_DIR}/cases/${CASE}"
  mkdir -p "${CASE_DIR}"
  : >"${CASE_DIR}/calls"
  : >"${CASE_DIR}/github_output"
  MIRROR="$2"
  local distro
  for distro in ${DISTROS}; do
    serve "${distro}" ok
  done
}

# serve <distro> <outcome>... : what the mirror answers for the distro, call after call
serve() {
  local distro="$1"
  shift
  printf '%s\n' "$@" >"${CASE_DIR}/$(printf '%s' "${MIRROR}:${distro}" | tr '/:' '__')"
}

# Runs the resolver for the current case, with the environment ci.yml gives it plus <VAR=value>...
# Started through a bash that records its PID first, so that the "term" outcome can signal it.
run_resolver() {
  STATUS=0
  # shellcheck disable=SC2016 # expanded by that bash, not this one
  OUTPUT="$(cd "${CASE_DIR}" && env PATH="${WORK_DIR}/bin:${PATH}" STUB_DIR="${CASE_DIR}" \
    ROS_DISTROS="${DISTROS}" MIRROR_REPO="${MIRROR}" FALLBACK_REPO="mirror.gcr.io/library/ros" \
    CONTAINER_REGISTRY_PREFIX="" REQUIRED_PLATFORM="linux/amd64" GHCR_USER="ci-actor" GHCR_TOKEN="token" \
    GITHUB_OUTPUT="${CASE_DIR}/github_output" "$@" \
    bash -c 'echo $$ >"${STUB_DIR}/resolver.pid"; exec "$0"' "${RESOLVER}" 2>&1)" || STATUS=$?
}

expect_status() {
  if [ "${STATUS}" -ne "$1" ]; then
    fail "the resolver exited with ${STATUS}, not $1. It said:"
    printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
  fi
}

# The image the resolver handed the matrix for <distro>, "" if none
image_of() {
  sed -n 's/^images=//p' "${CASE_DIR}/github_output" | tr ',' '\n' | sed -n "s/.*\"$1\": \"\\([^\"]*\\)\".*/\\1/p"
}

expect_image() {
  local image
  image="$(image_of "$1")"
  if ! [[ "${image}" =~ $2 ]]; then
    fail "$1 was resolved to '${image}', expected to match '$2'"
  fi
}

expect_distros_output() {
  local distros
  distros="$(sed -n 's/^distros=//p' "${CASE_DIR}/github_output")"
  if [ "${distros}" != '["humble", "iron", "jazzy", "kilted", "lyrical"]' ]; then
    fail "the matrix was handed distros=${distros:-(nothing)}"
  fi
}

expect_output() {
  if ! printf '%s\n' "${OUTPUT}" | grep -qF -- "$1"; then
    fail "the output does not say '$1'. It said:"
    printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
  fi
}

expect_no_output() {
  if printf '%s\n' "${OUTPUT}" | grep -qF -- "$1"; then
    fail "the output says '$1', which it must not. It said:"
    printf '%s\n' "${OUTPUT}" | sed 's/^/    /'
  fi
}

inspections_of() {
  grep -c "imagetools inspect ${MIRROR}:$1\$" "${CASE_DIR}/calls" || true
}

logouts() {
  grep -c '^logout ghcr.io$' "${CASE_DIR}/calls" || true
}

# No workflow command but the ones the resolver means to issue: the runner ends a line at a CR as
# well as at a LF, reads any line starting "::" as a command once its leading whitespace is trimmed,
# and "##[" anywhere in a line as one of the older form, which the resolver never issues.
expect_only_own_commands() {
  local line
  while IFS= read -r line; do
    line="${line#"${line%%[![:space:]]*}"}"
    case "${line}" in
    *"##["*) fail "the output carries a workflow command of the older form: ${line}" ;;
    "::warning title=ROS base image"* | "::error title=ROS base image"*) ;;
    "::"*) fail "the output carries a workflow command it did not mean to issue: ${line}" ;;
    esac
  done < <(printf '%s\n' "${OUTPUT}" | tr '\r' '\n')
}

ORG="ghcr.io/provizio/ros"
PINNED="^${ORG//./\\.}@sha256:[0-9a-f]{64}\$"

# 1. The organisation's mirror serves every distro: all pinned, nothing to warn about, and the
#    credential it logged in with removed again.
new_case org_mirror "${ORG}"
run_resolver
expect_status 0
expect_distros_output
for distro in ${DISTROS}; do
  expect_image "${distro}" "${PINNED}"
done
expect_no_output "::warning"
[ "$(logouts)" -eq 1 ] || fail "the resolver logged out of ghcr.io $(logouts) times, not once"

# 2. One 503 is a hiccup, not an answer: asked again, and pinned.
new_case transient "${ORG}"
serve jazzy 503 ok
run_resolver
expect_status 0
expect_image jazzy "${PINNED}"
expect_no_output "::warning"
[ "$(inspections_of jazzy)" -eq 2 ] || fail "jazzy was inspected $(inspections_of jazzy) times, not 2"

# 3. A fork: the mirror is the fork owner's own namespace, which nothing populates. Its "denied" is an
#    answer, so not asked again, and the fallback's warning must not send a fork to this
#    organisation's repopulation workflow.
new_case fork "ghcr.io/some-fork-owner42/ros"
for distro in ${DISTROS}; do
  serve "${distro}" denied
done
run_resolver
expect_status 0
expect_distros_output
for distro in ${DISTROS}; do
  expect_image "${distro}" '^mirror\.gcr\.io/library/ros:[a-z]+$'
  [ "$(inspections_of "${distro}")" -eq 1 ] || fail "${distro} was inspected $(inspections_of "${distro}") times, not once"
done
expect_output "A fork has no mirror of its own"
expect_no_output "Mirror ROS base images"

# 4. A registry the operator pinned the images to, missing one distro: there is no fallback, and
#    that distro's own jobs are the ones to fail -- every other distro still resolves and runs.
new_case pinned_missing "registry.example.com/mirrors/ros"
serve lyrical notfound
run_resolver CONTAINER_REGISTRY_PREFIX="registry.example.com/mirrors"
expect_status 0
expect_distros_output
expect_image humble '^registry\.example\.com/mirrors/ros@sha256:[0-9a-f]{64}$'
expect_image lyrical '^registry\.example\.com/mirrors/ros:lyrical$'
expect_output "::warning title=ROS base image not pinned::"
expect_no_output "mirror.gcr.io"
[ "$(inspections_of lyrical)" -eq 1 ] || fail "lyrical was inspected $(inspections_of lyrical) times, not once"

# 5. The same registry failing every read of one distro: asked as often as the resolver asks, then
#    handed on unpinned like a missing one rather than failing the whole matrix.
new_case pinned_failing "registry.example.com/mirrors/ros"
serve iron 503
run_resolver CONTAINER_REGISTRY_PREFIX="registry.example.com/mirrors"
expect_status 0
expect_distros_output
expect_image iron '^registry\.example\.com/mirrors/ros:iron$'
expect_image jazzy '^registry\.example\.com/mirrors/ros@sha256:[0-9a-f]{64}$'
[ "$(inspections_of iron)" -eq 3 ] || fail "iron was inspected $(inspections_of iron) times, not 3"

# 6. A prefix no image reference can be made from, carrying a line break and a workflow command of
#    its own: every distro is affected alike, so the job fails -- saying why, on one line.
new_case malformed "registry.example.com/mirrors"
bad_prefix="registry.example.com/mirrors
::error title=Injected::this annotation came from the variable"
MIRROR="${bad_prefix}/ros"
run_resolver CONTAINER_REGISTRY_PREFIX="${bad_prefix}"
expect_status 1
expect_output "::error title=ROS base image mirror unusable::"
expect_only_own_commands
if [ -s "${CASE_DIR}/github_output" ]; then
  fail "a matrix was handed out for a mirror no reference can be made from"
fi

# 7. A registry whose error text carries a CR and a workflow command after it: reported, prefixed
#    line by line, and never issued.
new_case cr_injection "registry.example.com/mirrors/ros"
serve humble cr-injection
run_resolver CONTAINER_REGISTRY_PREFIX="registry.example.com/mirrors"
expect_status 0
expect_output "    | ::error title=Injected::this annotation came from the registry"
expect_only_own_commands

# 8. A signal ends the resolver: the credential is removed on the way out, and nothing after it runs.
new_case signalled "${ORG}"
serve iron term
run_resolver
expect_status 143
[ "$(logouts)" -eq 1 ] || fail "a signalled resolver logged out of ghcr.io $(logouts) times, not once"
[ "$(inspections_of jazzy)" -eq 0 ] || fail "the resolver carried on after the signal, to inspect jazzy"
if [ -s "${CASE_DIR}/github_output" ]; then
  fail "a signalled resolver still handed out a matrix"
fi

# 9. A login that fails: the mirror is not consulted, and no credential this job did not write is
#    logged out of.
new_case login_failed "${ORG}"
run_resolver STUB_LOGIN_RESULT=fail
expect_status 0
expect_distros_output
expect_image humble '^mirror\.gcr\.io/library/ros:humble$'
expect_output "::warning title=ROS base image mirror unreachable::"
if grep -q '^logout' "${CASE_DIR}/calls"; then
  fail "the resolver logged out of ghcr.io after a login of its own had failed"
fi

# 10. A registry whose error text carries a command of the runner's older form, which it reads
#     anywhere in a line: reported broken up, and the fallback's warning, which a stop-commands would
#     have silenced, still raised.
new_case legacy_injection "${ORG}"
serve kilted legacy-injection
run_resolver
expect_status 0
expect_image kilted '^mirror\.gcr\.io/library/ros:kilted$'
expect_output "    | ERROR: 500 Oops # #[stop-commands]from-the-registry"
expect_output "::warning title=ROS base image fallback used::"
expect_only_own_commands

# 11. The same carried by a platform a manifest states, which the resolver reports inline.
new_case platform_injection "${ORG}"
serve humble platform-injection
run_resolver
expect_status 0
expect_image humble '^mirror\.gcr\.io/library/ros:humble$'
expect_output "linux/arm64 # #[add-mask]from-the-manifest"
expect_only_own_commands

if [ "${FAILURES}" -ne 0 ]; then
  echo "${FAILURES} check(s) failed"
  exit 1
fi
echo "All resolve_ros_base_image.sh cases passed"
