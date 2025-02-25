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

FAST_DDS_VERSION=${FAST_DDS_VERSION:-v2.14.2}
FAST_CDR_VERSION=${FAST_CDR_VERSION:-v2.2.2}
SWIG_VERSION=${SWIG_VERSION:-4.2.1}

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
    brew install gcc
  else
    brew install llvm
  fi

  # Install CMake
  brew install cmake ninja

  # Install openssl
  brew install openssl

  if [[ "${PYTHON}" != "OFF" ]]; then
    # Install Python and related dependencies
    brew install python3

    # Install SWIG and its dependencies, if not yet installed
    brew install swig

    # Make a virtual environment to avoid "error: externally-managed-environment"
    python3 -m venv /tmp/provizio_dds.venv
    source /tmp/provizio_dds.venv/bin/activate
    python3 -m pip install wheel setuptools
    deactivate
  fi

  if [[ "${STATIC_ANALYSIS}" != "OFF" ]]; then
    if [[ "${CC}" == "gcc" ]]; then
      # Despite building with GCC, llvm tools are required
      brew install llvm
    fi

    # Install cppcheck
    brew install cppcheck

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

  # Update apt cache
  apt update

  # Install lsb-release for checking Ubuntu version and accessing https
  apt install -y --no-install-recommends lsb-release ca-certificates

  # Install build-essential
  apt install -y --no-install-recommends build-essential

  # Install patchelf
  apt install -y --no-install-recommends patchelf

  # Install unzip
  apt install -y --no-install-recommends unzip

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
      apt install -y software-properties-common
      add-apt-repository -y ppa:ubuntu-toolchain-r/test
      apt update
      apt install -y --no-install-recommends gcc-9 g++-9
      update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100
      update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100
    else
      apt install -y --no-install-recommends gcc g++
    fi
  else
    if [ "${UBUNTU_18}" = true ]; then
      apt install -y --no-install-recommends clang-10
      update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 100
      update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-10 100
    else
      apt install -y --no-install-recommends clang
    fi
  fi

  # Install make
  apt install -y --no-install-recommends make ninja-build

  # Install CMake
  if [ "${UBUNTU_18}" = true ] || [ "${UBUNTU_20}" = true ]; then
      apt install -y software-properties-common lsb-release wget
      wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
      apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
      apt update && apt install -y --no-install-recommends kitware-archive-keyring
      apt update && apt install -y --no-install-recommends cmake
  else
      apt install -y --no-install-recommends cmake
  fi

  # Install git 2.18+
  if [ "${UBUNTU_18}" = true ]; then
    apt-add-repository ppa:git-core/ppa
    apt update
  fi
  apt install -y --no-install-recommends git

  # Install libssl-dev
  apt install -y --no-install-recommends libssl-dev

  if [[ "${STATIC_ANALYSIS}" != "OFF" ]]; then
    # Install cppcheck, clang-format and clang-tidy (and clang for proper clang-tidy checks)
    if [ "${UBUNTU_18}" = true ]; then
      apt install -y --no-install-recommends clang-10 clang-format-10 clang-tidy-10

      update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-10 100
      update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 100
      update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-10 100
      update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-10 100
    else
      apt install -y --no-install-recommends clang clang-format clang-tidy cppcheck
    fi
  fi

  if [[ "${FAST_DDS_INSTALL}" != "OFF" ]]; then
    # Install Fast-DDS instead of building it with provizio_dds, if enabled
    (
      if [[ "${FAST_DDS_INSTALL}" == "ON" ]]; then
        FAST_DDS_INSTALL="/usr/local/"
      fi

      apt install -y --no-install-recommends wget python3-pip libasio-dev libtinyxml2-dev
      rm -rf /tmp/fastdds # In case of previous installation
      mkdir /tmp/fastdds
      
      # Foonathan memory
      cd /tmp/fastdds
      git clone https://github.com/eProsima/foonathan_memory_vendor.git
      mkdir foonathan_memory_vendor/build
      cd foonathan_memory_vendor/build
      cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX="${FAST_DDS_INSTALL}" -DBUILD_SHARED_LIBS=ON
      cmake --build . --target install

      # Fast CDR
      cd /tmp/fastdds
      git clone https://github.com/eProsima/Fast-CDR.git
      cd Fast-CDR
      git checkout ${FAST_CDR_VERSION}
      mkdir build
      cd build
      cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX="${FAST_DDS_INSTALL}" -DBUILD_SHARED_LIBS=ON
      cmake --build . --target install

      # Fast DDS
      cd /tmp/fastdds
      git clone https://github.com/eProsima/Fast-DDS.git
      cd Fast-DDS
      git checkout ${FAST_DDS_VERSION}
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

    apt install  -y --no-install-recommends locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    apt install -y --no-install-recommends software-properties-common
    add-apt-repository universe
    apt install -y --no-install-recommends curl
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    apt update
    apt install -y --no-install-recommends ros-${ROS2_VERSION}-desktop
  fi

  if [[ "${PYTHON}" != "OFF" ]]; then
    # Install Python and related dependencies
    apt install -y --no-install-recommends python3 python3-pip python3-venv libpython3-dev
    python3 -m pip install setuptools

    # Install SWIG (ubuntu 18 has too old swig in apt, ubuntu 24 has a broken version of swig in apt, see https://github.com/swig/swig/issues/2794)
    if [ "${UBUNTU_18}" = true ] || [ "${UBUNTU_24}" = true ]; then
      echo "Installing SWIG v${SWIG_VERSION} from source..."
      apt remove -y swig # But first, make sure to delete any version that may already be installed
      if ! swig -version; then
      (
        set -eu

        apt install -y --no-install-recommends wget libpcre2-dev automake bison byacc
        cd /tmp
        wget -c https://github.com/swig/swig/archive/refs/tags/v${SWIG_VERSION}.tar.gz -O - | tar -xz
        cd swig-${SWIG_VERSION}
        ./autogen.sh
        ./configure
        make -j8
        make install
      )
      fi
    else
      echo "Installing SWIG from apt..."
      apt install -y --no-install-recommends swig
    fi

    # Install specific Cython in older versions of Python (see https://github.com/numpy/numpy/issues/24377)
    python_version=$(python3 -c 'import sys; print("".join(map(str, sys.version_info[:2])))')
    if [[ "${python_version}" -lt "39" ]]; then
        python3 -m pip install "Cython<3"
        python3 -m pip install "numpy>=1.16"
    fi
  fi
fi

echo "Done installing provizio_dds build dependencies!"
