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
# install_dependencies.sh [PYTHON=OFF|ON] [STATIC_ANALYSIS=OFF|ON] [INSTALL_ROS=OFF|ON] [FAST_DDS_INSTALL=OFF|ON|install_path]
# (Ubuntu 18.04+ or macOS X required)

set -e

PYTHON=${1:-"OFF"}
STATIC_ANALYSIS=${2:-"OFF"}
INSTALL_ROS=${3:-"OFF"}
FAST_DDS_INSTALL=${4:-"OFF"}
CC=${CC:-"gcc"}

FAST_DDS_VERSION=${FAST_DDS_VERSION:-v3.6.2.0}
FAST_CDR_VERSION=${FAST_CDR_VERSION:-v2.3.5}
# The same tag the CMake build pins (FOONATHAN_MEMORY_VENDOR_VERSION in CMakeLists.txt), so a
# system-wide install and a from-source build ship one foonathan_memory.
FOONATHAN_MEMORY_VENDOR_VERSION=${FOONATHAN_MEMORY_VENDOR_VERSION:-v1.4.1}
SWIG_VERSION=${SWIG_VERSION:-4.4.1}

if [[ "${OSTYPE}" == "darwin"* ]]; then
  # macOS

  if [[ "${INSTALL_ROS}" != "OFF" ]]; then
    echo "INSTALL_ROS installation option is not supported in macOS yet"
    exit 1
  fi

  if [[ "${FAST_DDS_INSTALL}" != "OFF" ]]; then
    echo "FAST_DDS_INSTALL option is not supported in macOS yet"
    exit 1
  fi

  # Install GCC/clang
  if [[ "${CC}" == "gcc" ]]; then
    brew install gcc || echo "Skipping, as it's likely already installed"
  else
    brew install llvm || echo "Skipping, as it's likely already installed"
  fi

  # Install CMake and Ninja
  brew install cmake || echo "Skipping, as it's likely already installed"
  brew install ninja || echo "Skipping, as it's likely already installed"

  # Install Eigen3 (optional provizio_dds dependency: accelerates point clouds accumulation linear algebra)
  brew install eigen || echo "Skipping, as it's likely already installed"

  # Install openssl
  brew install openssl || echo "Skipping, as it's likely already installed"

  if [[ "${PYTHON}" != "OFF" ]]; then
    # Install Python and related dependencies
    brew install python3 || echo "Skipping, as it's likely already installed"

    # Install SWIG and its dependencies, if not yet installed
    brew install swig || echo "Skipping, as it's likely already installed"

    # Make a virtual environment to avoid "error: externally-managed-environment"
    python3 -m venv /tmp/provizio_dds.venv
    source /tmp/provizio_dds.venv/bin/activate
    python3 -m pip install wheel setuptools "numpy>=1.16" "transforms3d>=0.4.1"
    deactivate
  fi

  if [[ "${STATIC_ANALYSIS}" != "OFF" ]]; then
    if [[ "${CC}" == "gcc" ]]; then
      # Despite building with GCC, llvm tools are required
      brew install llvm || echo "Skipping, as it's likely already installed"
    fi

    # Install cppcheck
    brew install cppcheck || echo "Skipping, as it's likely already installed"

    # Install clang-format and clang-tidy
    ln -s "$(brew --prefix llvm)/bin/clang-format" "/usr/local/bin/clang-format"
    ln -s "$(brew --prefix llvm)/bin/clang-tidy" "/usr/local/bin/clang-tidy"
  fi
else
  # Linux (Ubuntu 18+ assumed)

  if [[ "${EUID}" != "0" ]]; then
    echo "Root permissions required"
    exit 1
  fi

  # apt on CI hosts fails transiently far more often than it fails meaningfully: a mirror
  # returning 5xx or closing a connection mid-download, a stale package list after a mirror
  # rotation, and — on GitHub-hosted runners — unattended-upgrades holding the dpkg lock for the
  # first minute or two of the job. Every one of those aborts an otherwise healthy build, and
  # this script is re-run from scratch inside the congested-network Docker image on every CI
  # run, so it is the single largest apt surface in the project.
  #
  # Two layers of defence, because they cover different failures:
  #   - Acquire::Retries makes apt itself retry an individual failed download.
  #   - DPkg::Lock::Timeout waits for the dpkg lock instead of failing instantly. Unknown to
  #     apt < 1.9 (Ubuntu 18.04), which ignores unrecognised -o keys, so it is safe there.
  #   - The outer loop covers what neither does: a refreshed package list between attempts,
  #     with exponential backoff, for the "404 on a package version" case after a rotation.
  APT_MAX_ATTEMPTS=${APT_MAX_ATTEMPTS:-5}
  APT_OPTIONS=(-o "Acquire::Retries=3" -o "DPkg::Lock::Timeout=180")

  # apt_get <args...>: run apt-get with those options, retrying transient failures.
  apt_get() {
    local attempt=1
    local delay=5
    while true; do
      if apt-get "${APT_OPTIONS[@]}" "$@"; then
        return 0
      fi
      if [[ "${attempt}" -ge "${APT_MAX_ATTEMPTS}" ]]; then
        echo "apt-get $* failed after ${APT_MAX_ATTEMPTS} attempts" >&2
        return 1
      fi
      echo "apt-get $* failed (attempt ${attempt}/${APT_MAX_ATTEMPTS}); retrying in ${delay}s..." >&2
      sleep "${delay}"
      delay=$((delay * 2))
      # A stale package list is a common cause; refresh it before trying again. Its own
      # failure is not fatal here — the retry of the real command is what matters.
      apt-get "${APT_OPTIONS[@]}" update || true
      attempt=$((attempt + 1))
    done
  }

  # apt_get_optional <args...>: same options but NO retries, for calls whose failure is an
  # expected outcome the caller handles (e.g. a held package) rather than a transient fault.
  apt_get_optional() {
    apt-get "${APT_OPTIONS[@]}" "$@"
  }

  # Downloads fail transiently for the same reasons apt does — a 503 from GitHub or a mirror,
  # a connection reset mid-stream, a DNS blip — and every one of them aborted the job. Observed:
  # "HTTP request sent, awaiting response... 503 Service Unavailable" fetching the SWIG tarball,
  # which then fed a truncated stream straight into tar ("gzip: stdin: unexpected end of file").
  #
  # wget does NOT retry HTTP 5xx by default, hence --retry-on-http-error; the outer loop covers
  # what its own retries do not (a stream that dies mid-transfer, a name-resolution failure).
  # Everything lands in a FILE first and is only then unpacked, so a truncated download can
  # never be piped into tar as if it were complete. All options predate Ubuntu 18.04's wget
  # 1.19 / curl 7.58, which the jetson-18.04 runners still use.
  DOWNLOAD_MAX_ATTEMPTS=${DOWNLOAD_MAX_ATTEMPTS:-5}

  # download <url> <output-path>
  download() {
    local url="$1"
    local output="$2"
    local attempt=1
    local delay=5
    while true; do
      if wget --tries=3 --timeout=30 --waitretry=10 \
              --retry-on-http-error=408,429,500,502,503,504 \
              -O "${output}" "${url}"; then
        return 0
      fi
      rm -f "${output}"  # A partial file must never be mistaken for a complete one.
      if [[ "${attempt}" -ge "${DOWNLOAD_MAX_ATTEMPTS}" ]]; then
        echo "Failed to download ${url} after ${DOWNLOAD_MAX_ATTEMPTS} attempts" >&2
        return 1
      fi
      echo "Download of ${url} failed (attempt ${attempt}/${DOWNLOAD_MAX_ATTEMPTS}); retrying in ${delay}s..." >&2
      sleep "${delay}"
      delay=$((delay * 2))
      attempt=$((attempt + 1))
    done
  }

  # Update apt cache
  apt_get update

  # Install lsb-release for checking Ubuntu version and accessing https
  apt_get install -y --no-install-recommends lsb-release ca-certificates

  # Install build-essential
  apt_get install -y --no-install-recommends build-essential

  # Install patchelf
  apt_get install -y --no-install-recommends patchelf

  # Install unzip
  apt_get install -y --no-install-recommends unzip

  # Install Eigen3 (optional provizio_dds dependency: accelerates point clouds accumulation linear algebra)
  apt_get install -y --no-install-recommends libeigen3-dev

  # Check if running in Ubuntu 18
  UBUNTU_18=false
  if lsb_release -a | grep -q 18; then
    echo "Running in Ubuntu 18 detected..."
    UBUNTU_18=true
  fi

  # Check if running in Ubuntu 20
  UBUNTU_20=false
  if lsb_release -a | grep -q 20; then
    echo "Running in Ubuntu 20 detected..."
    UBUNTU_20=true
  fi

  # Check if running in Ubuntu 22
  UBUNTU_22=false
  if lsb_release -a | grep -q 22; then
    echo "Running in Ubuntu 22 detected..."
    UBUNTU_22=true
  fi

  # Check if running in Ubuntu 24
  UBUNTU_24=false
  if lsb_release -a | grep -q 24; then
    echo "Running in Ubuntu 24 detected..."
    UBUNTU_24=true
  fi

  # Install GCC/clang
  if [[ "${CC}" == "gcc" ]]; then
    if [ "${UBUNTU_18}" = true ]; then
      apt_get install -y software-properties-common
      add-apt-repository -y ppa:ubuntu-toolchain-r/test
      apt_get update
      apt_get install -y --no-install-recommends gcc-9 g++-9
      update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100
      update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100
    else
      apt_get install -y --no-install-recommends gcc g++
    fi
  else
    if [ "${UBUNTU_18}" = true ]; then
      apt_get install -y --no-install-recommends clang-10
      update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 100
      update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-10 100
    else
      apt_get install -y --no-install-recommends clang
    fi
  fi

  # Install make
  apt_get install -y --no-install-recommends make ninja-build

  # Install CMake
  if [ "${UBUNTU_18}" = true ] || [ "${UBUNTU_20}" = true ]; then
      if [ "${UBUNTU_18}" = true ]; then
          CMAKE_VERSION=3.25.2-0kitware1ubuntu18.04.1
      else
          CMAKE_VERSION=3.25.2-0kitware1ubuntu20.04.1
      fi
      apt_get install -y software-properties-common lsb-release wget
      download https://apt.kitware.com/keys/kitware-archive-latest.asc /tmp/kitware-archive-latest.asc
      gpg --dearmor - < /tmp/kitware-archive-latest.asc | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
      rm -f /tmp/kitware-archive-latest.asc
      apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
      apt_get update
      apt_get install -y --no-install-recommends kitware-archive-keyring
      apt_get_optional install -y --no-install-recommends --allow-downgrades cmake=${CMAKE_VERSION} cmake-data=${CMAKE_VERSION} || echo "Skipping installing cmake, it's likely already installed and held"
  else
      apt_get install -y --no-install-recommends cmake
  fi

  # Install git 2.18+
  if [ "${UBUNTU_18}" = true ]; then
    apt-add-repository ppa:git-core/ppa
    apt_get update
  fi
  apt_get install -y --no-install-recommends git

  # Install libssl-dev
  apt_get install -y --no-install-recommends libssl-dev

  if [[ "${STATIC_ANALYSIS}" != "OFF" ]]; then
    # Install cppcheck, clang-format and clang-tidy (and clang for proper clang-tidy checks)
    if [ "${UBUNTU_18}" = true ]; then
      apt_get install -y --no-install-recommends clang-10 clang-format-10 clang-tidy-10

      update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-10 100
      update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 100
      update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-10 100
      update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-10 100
    else
      apt_get install -y --no-install-recommends clang clang-format clang-tidy cppcheck
    fi
  fi

  if [[ "${FAST_DDS_INSTALL}" != "OFF" ]]; then
    # Install Fast-DDS instead of building it with provizio_dds, if enabled
    (
      if [[ "${FAST_DDS_INSTALL}" == "ON" ]]; then
        FAST_DDS_INSTALL="/usr/local/"
      fi

      apt_get install -y --no-install-recommends wget python3-pip libasio-dev libtinyxml2-dev
      rm -rf /tmp/fastdds # In case of previous installation
      mkdir /tmp/fastdds

      # Foonathan memory
      # --depth 1 throughout this block: none of these builds reads its git history, and the
      # transfer is where a clone breaks — a stalled connection or a DNS failure partway
      # through a large pack fails the whole install. The refs below are tags, which is what
      # --branch needs; a raw SHA would need the depth dropped with it.
      cd /tmp/fastdds
      git clone --depth 1 --branch "${FOONATHAN_MEMORY_VENDOR_VERSION}" https://github.com/eProsima/foonathan_memory_vendor.git
      mkdir foonathan_memory_vendor/build
      cd foonathan_memory_vendor/build
      cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX="${FAST_DDS_INSTALL}" -DBUILD_SHARED_LIBS=ON
      cmake --build . --target install

      # Fast CDR
      cd /tmp/fastdds
      git clone --depth 1 --branch "${FAST_CDR_VERSION}" https://github.com/eProsima/Fast-CDR.git
      cd Fast-CDR
      mkdir build
      cd build
      cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX="${FAST_DDS_INSTALL}" -DBUILD_SHARED_LIBS=ON
      cmake --build . --target install

      # Fast DDS
      cd /tmp/fastdds
      git clone --depth 1 --branch "${FAST_DDS_VERSION}" https://github.com/eProsima/Fast-DDS.git
      cd Fast-DDS
      mkdir build
      cd build
      cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX="${FAST_DDS_INSTALL}" -DBUILD_SHARED_LIBS=ON
      cmake --build . --target install
    )
  fi

  if [[ "${INSTALL_ROS}" != "OFF" ]]; then
    # Install ROS2 Dependencies and configure appropriately

    if [[ "${ROS2_VERSION:-}" == "" ]]; then
      if [ "${UBUNTU_18}" = true ]; then
        ROS2_VERSION="eloquent"
      elif [ "${UBUNTU_20}" = true ]; then
        ROS2_VERSION="galactic"
      elif [ "${UBUNTU_22}" = true ]; then
        ROS2_VERSION="humble"
      else
        ROS2_VERSION="jazzy"
      fi
    fi

    echo "Installing ROS2: ${ROS2_VERSION}..."

    apt_get install  -y --no-install-recommends locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    apt_get install -y --no-install-recommends software-properties-common
    add-apt-repository universe
    apt_get install -y --no-install-recommends curl
    # curl's --retry already covers transient HTTP errors (408, 429, 5xx) as well as connection
    # failures, so it needs no outer loop of its own.
    curl -sSL --retry 5 --retry-delay 5 --retry-connrefused \
         https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    apt_get update
    apt_get install -y --no-install-recommends ros-${ROS2_VERSION}-desktop
  fi

  if [[ "${PYTHON}" != "OFF" ]]; then
    # Install Python and related dependencies
    apt_get install -y --no-install-recommends python3 python3-pip python3-venv libpython3-dev python3-setuptools

    # Install SWIG >= 4.4 from source (apt versions are too old or broken across Ubuntu releases)
    CURRENT_SWIG_VERSION=$(swig -version 2>/dev/null | grep -oP 'SWIG Version \K[0-9.]+' || echo "0.0.0")
    if [ "$(printf '%s\n' "4.4.0" "${CURRENT_SWIG_VERSION}" | sort -V | head -n1)" != "4.4.0" ]; then
      echo "Installing SWIG v${SWIG_VERSION} from source (current: ${CURRENT_SWIG_VERSION})..."
      apt_get_optional remove -y swig 2>/dev/null || true
      (
        set -eu

        apt_get install -y --no-install-recommends wget libpcre2-dev automake bison byacc
        cd /tmp
        download "https://github.com/swig/swig/archive/refs/tags/v${SWIG_VERSION}.tar.gz" "/tmp/swig-${SWIG_VERSION}.tar.gz"
        tar -xzf "/tmp/swig-${SWIG_VERSION}.tar.gz"
        rm -f "/tmp/swig-${SWIG_VERSION}.tar.gz"
        cd swig-${SWIG_VERSION}
        ./autogen.sh
        ./configure
        make -j8
        make install
      )
    else
      echo "SWIG ${CURRENT_SWIG_VERSION} already installed, skipping..."
    fi

    # Install specific Cython in older versions of Python (see https://github.com/numpy/numpy/issues/24377)
    python_version=$(python3 -c 'import sys; print("".join(map(str, sys.version_info[:2])))')
    if [[ "${python_version}" -lt "39" ]]; then
        python3 -m pip install "Cython<3"
    fi

    # Install Python runtime dependencies
    python3 -m pip install "numpy>=1.16" "transforms3d>=0.4.1" --break-system-packages || python3 -m pip install "numpy>=1.16" "transforms3d>=0.4.1"
  fi
fi

if [ "${UBUNTU_18}" = true ]; then
    # Install patchelf v0.18 (v0.9 shipped in 18.04 breaks binaries on --replace-needed)
    cd /tmp
    PATCHELF_VERSION="0.18.0"
    download "https://github.com/NixOS/patchelf/releases/download/${PATCHELF_VERSION}/patchelf-${PATCHELF_VERSION}.tar.gz" "patchelf-${PATCHELF_VERSION}.tar.gz"
    tar -xf patchelf-${PATCHELF_VERSION}.tar.gz
    cd patchelf-${PATCHELF_VERSION}/
    ./configure --prefix=/usr --docdir=/usr/share/doc/patchelf-${PATCHELF_VERSION} && make && make install
    cd /tmp
    rm -rf patchelf-${PATCHELF_VERSION}*
fi

echo "Done installing provizio_dds build dependencies!"
