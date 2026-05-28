#!/usr/bin/env bash
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
#
# Sets up a Python venv with provizio_dds 1.10.1 installed from the
# matching GitHub tag, ready to be used as the wire-interop reference by
# `cross_version_compat_test.py`.
#
# The 1.10.1 wheel ships its own Fast-DDS 2.14.x runtime alongside its
# Python bindings, so the venv is fully self-contained: no system Fast-DDS
# install required, no environment changes leaking back to the caller.
#
# Usage:
#   setup_legacy_provizio_dds_venv.sh [VENV_DIR]
#
# VENV_DIR defaults to ${TMPDIR:-/tmp}/provizio_dds_1_10_1_venv. The script
# is idempotent: if the venv already exists and the install marker file is
# present, it exits 0 without reinstalling. Pass --force to force a
# reinstall.

set -euo pipefail

LEGACY_TAG="1.10.1"
GIT_URL="https://github.com/provizio/provizio_dds.git"

# Parse args: --force is a flag, everything else is the positional VENV_DIR.
# Keeping these separated matters because callers consume stdout via command
# substitution to pick up the venv path — accepting --force as a positional
# would silently turn the venv path into the string "--force".
force_reinstall=0
venv_dir_arg=""
for arg in "$@"; do
    case "${arg}" in
        --force)
            force_reinstall=1
            ;;
        *)
            if [[ -n "${venv_dir_arg}" ]]; then
                echo "setup_legacy_provizio_dds_venv: unexpected extra argument: ${arg}" >&2
                exit 2
            fi
            venv_dir_arg="${arg}"
            ;;
    esac
done
VENV_DIR="${venv_dir_arg:-${TMPDIR:-/tmp}/provizio_dds_1_10_1_venv}"
MARKER="${VENV_DIR}/.installed.${LEGACY_TAG}"

# Cross-platform venv layout: POSIX puts the interpreter at bin/python3,
# Windows puts it at Scripts/python.exe. Detect via OSTYPE so this script
# also works under Git Bash on the Windows CI runners (which is where we
# need to bootstrap the legacy venv for the cross-version compat test).
case "${OSTYPE:-}" in
    msys*|cygwin*|win32*)
        VENV_BIN_SUBDIR="Scripts"
        VENV_PYTHON_NAME="python.exe"
        VENV_PIP_NAME="pip.exe"
        ;;
    *)
        VENV_BIN_SUBDIR="bin"
        VENV_PYTHON_NAME="python3"
        VENV_PIP_NAME="pip"
        ;;
esac
VENV_PYTHON="${VENV_DIR}/${VENV_BIN_SUBDIR}/${VENV_PYTHON_NAME}"
VENV_PIP="${VENV_DIR}/${VENV_BIN_SUBDIR}/${VENV_PIP_NAME}"

# Stdout must contain only the final venv interpreter path so callers can
# capture it via $(...). Everything else — pip output, build logs, the
# smoke-test prints — goes to stderr.
if [[ -f "${MARKER}" && "${force_reinstall}" -eq 0 ]]; then
    echo "${VENV_PYTHON}"
    exit 0
fi

# Building 1.10.1 from source goes through its bundled CMake + Fast-DDS
# externalproject, which expects the usual system toolchain. Check the
# bare minimum upfront so failures point at the actual missing pieces
# rather than a deep CMake log line. On Windows the python launcher is
# `python` (not `python3`), so probe both forms.
if command -v python3 >/dev/null 2>&1; then
    HOST_PYTHON="python3"
elif command -v python >/dev/null 2>&1; then
    HOST_PYTHON="python"
else
    echo "setup_legacy_provizio_dds_venv: missing required tool: python3 (or python)" >&2
    exit 2
fi
for tool in git cmake; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "setup_legacy_provizio_dds_venv: missing required tool: ${tool}" >&2
        exit 2
    fi
done
# The C/C++ toolchain is whatever the caller (or CI) has configured via
# CC/CXX, falling back to the `cc`/`c++` symlinks every Linux distro and
# macOS provide. Don't hard-code gcc/g++ — the clang CI jobs export
# CC=clang and CXX=clang++ and the system may not have gcc installed
# at all.
CC_CANDIDATE="${CC:-cc}"
CXX_CANDIDATE="${CXX:-c++}"
if ! command -v "${CC_CANDIDATE}" >/dev/null 2>&1; then
    echo "setup_legacy_provizio_dds_venv: missing C compiler: ${CC_CANDIDATE}" >&2
    exit 2
fi
if ! command -v "${CXX_CANDIDATE}" >/dev/null 2>&1; then
    echo "setup_legacy_provizio_dds_venv: missing C++ compiler: ${CXX_CANDIDATE}" >&2
    exit 2
fi

# Re-create the venv fresh whenever we cross the install-marker boundary so
# stale builds from earlier tag attempts don't shadow the new install.
rm -rf "${VENV_DIR}" >&2
"${HOST_PYTHON}" -m venv "${VENV_DIR}" >&2

# Resolve a Python interpreter the venv can actually use to run pip — on
# minimal containers pip itself sometimes needs an explicit upgrade
# before it can resolve git+https URLs cleanly.
"${VENV_PIP}" install --quiet --upgrade pip setuptools wheel >&2

# Build from the immutable tag, not the moving branch — the whole point
# of this test is to compare against the deployed-fleet baseline that
# 1.10.1 represents.
"${VENV_PIP}" install -v "git+${GIT_URL}@${LEGACY_TAG}" >&2

# Smoke-test the install so a subtle compile issue doesn't surface as a
# mystery import failure inside the actual cross-version test runner.
# Run from the venv directory so the implicit `python -c` sys.path[0] (CWD)
# doesn't shadow the venv-bundled provizio_dds_python_types package with the
# in-progress build's same-named module — callers of this script (CI's
# test.sh) chdir to the build tree before invoking us, and the build tree
# contains the SWIG-generated provizio_dds_python_types.py for the *current*
# Fast-DDS version, ABI-incompatible with the 1.10.1 wheel's bundled libs.
(cd "${VENV_DIR}" && "${VENV_PYTHON}" -c "
import provizio_dds
assert hasattr(provizio_dds, 'Publisher')
assert hasattr(provizio_dds, 'Subscriber')
assert hasattr(provizio_dds, 'request')
# The wire-name fingerprint we lock in across versions:
name = provizio_dds.StringPubSubType().getName()
assert name == 'std_msgs::msg::dds_::String_', f'unexpected wire name: {name!r}'
print('legacy provizio_dds smoke test passed:', provizio_dds.__file__)
") >&2

touch "${MARKER}"
echo "${VENV_PYTHON}"
