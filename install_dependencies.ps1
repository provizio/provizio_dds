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

# Use as:
# .\install_dependencies.ps1 [-WithPython OFF|ON] [-StaticAnalysis OFF|ON]

param(
    [string]$WithPython = "OFF",
    [string]$StaticAnalysis = "OFF"
)

$ErrorActionPreference = "Stop"

if ($WithPython -ne "OFF") {
    # SWIG 4.4+ is required for Python 3.14 support (heap type / ht_token changes).
    # Chocolatey only provides 4.3.1 as of Feb 2026, so we download directly.
    $swigVersion = "4.4.1"
    $swigInstallDir = "C:\swig"
    $swigZip = Join-Path $env:TEMP "swigwin-$swigVersion.zip"
    # The generic redirect lets SourceForge pick a geo mirror, and a single mirror can
    # refuse connections for hours (recurring CI failure: "curl: (28) Failed to connect
    # to pilotfiber.dl.sourceforge.net"). Retry each URL, then fall back to the master
    # mirror and the plain downloads redirector rather than failing on the first pick.
    $swigUrls = @(
        "https://sourceforge.net/projects/swig/files/swigwin/swigwin-$swigVersion/swigwin-$swigVersion.zip/download",
        "https://master.dl.sourceforge.net/project/swig/swigwin/swigwin-$swigVersion/swigwin-$swigVersion.zip?viasf=1",
        "https://downloads.sourceforge.net/project/swig/swigwin/swigwin-$swigVersion/swigwin-$swigVersion.zip"
    )

    Write-Host "Installing SWIG $swigVersion..."
    # Use curl.exe (available on all modern Windows) — Invoke-WebRequest chokes on
    # SourceForge redirects in PowerShell 7 on Windows Server.
    foreach ($swigUrl in $swigUrls) {
        & curl.exe -fsSL --connect-timeout 20 --retry 3 --retry-all-errors --retry-delay 3 -o $swigZip $swigUrl
        if ($LASTEXITCODE -eq 0 -and (Test-Path $swigZip)) { break }
        Write-Host "Download failed from $swigUrl (curl exit $LASTEXITCODE); trying the next mirror..."
    }
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $swigZip)) { throw "Failed to download SWIG $swigVersion" }

    Expand-Archive -Path $swigZip -DestinationPath $env:TEMP -Force
    if (Test-Path $swigInstallDir) { Remove-Item -Recurse -Force $swigInstallDir }
    Move-Item -Path (Join-Path $env:TEMP "swigwin-$swigVersion") -Destination $swigInstallDir

    # Make SWIG available for subsequent steps
    $env:PATH = "$swigInstallDir;$env:PATH"
    if ($env:GITHUB_PATH) {
        Add-Content -Path $env:GITHUB_PATH -Value $swigInstallDir
    }
    Write-Host "SWIG $swigVersion installed to $swigInstallDir"

    # Install Python build dependencies
    Write-Host "Installing Python build dependencies (wheel, setuptools)..."
    python -m pip install wheel setuptools "numpy>=1.16" "transforms3d>=0.4.1"
    if ($LASTEXITCODE -ne 0) { throw "Failed to install Python build dependencies" }
}

# Eigen3 (optional provizio_dds dependency: accelerates point clouds accumulation linear algebra).
# Header-only: "install" = configure + install headers and CMake config files, no compilation.
$eigenVersion = "3.4.0"
$eigenZip = Join-Path $env:TEMP "eigen-$eigenVersion.zip"
$eigenSrc = Join-Path $env:TEMP "eigen-$eigenVersion"
$eigenInstallDir = "C:\eigen3"
Write-Host "Installing Eigen $eigenVersion..."
& curl.exe -fsSL -o $eigenZip "https://gitlab.com/libeigen/eigen/-/archive/$eigenVersion/eigen-$eigenVersion.zip"
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $eigenZip)) { throw "Failed to download Eigen $eigenVersion" }
Expand-Archive -Path $eigenZip -DestinationPath $env:TEMP -Force
cmake -S $eigenSrc -B "$eigenSrc\build" -DBUILD_TESTING=OFF
if ($LASTEXITCODE -ne 0) { throw "Failed to configure Eigen $eigenVersion" }
cmake --install "$eigenSrc\build" --prefix $eigenInstallDir
if ($LASTEXITCODE -ne 0) { throw "Failed to install Eigen $eigenVersion" }
# Make find_package(Eigen3) work in subsequent steps
if ($env:GITHUB_ENV) {
    Add-Content -Path $env:GITHUB_ENV -Value "Eigen3_DIR=$eigenInstallDir\share\eigen3\cmake"
} else {
    $env:Eigen3_DIR = "$eigenInstallDir\share\eigen3\cmake"
}
Write-Host "Eigen $eigenVersion installed to $eigenInstallDir"

if ($StaticAnalysis -ne "OFF") {
    Write-Host "Installing LLVM (clang-format, clang-tidy)..."
    choco install llvm -y
    if ($LASTEXITCODE -ne 0) { throw "Failed to install LLVM" }

    Write-Host "Installing cppcheck..."
    choco install cppcheck -y
    if ($LASTEXITCODE -ne 0) { throw "Failed to install cppcheck" }
}

# OpenSSL is required for a proper Fast-DDS build.
# In CI the ilammy/msvc-dev-cmd@v1 action already provides it, but for local
# development we install it via Chocolatey if it's not already available.
$openssl = Get-Command openssl -ErrorAction SilentlyContinue
if (-not $openssl) {
    Write-Host "Installing OpenSSL..."
    choco install openssl -y
    if ($LASTEXITCODE -ne 0) { throw "Failed to install OpenSSL" }
} else {
    Write-Host "OpenSSL already available at $($openssl.Source), skipping install."
}

Write-Host "Done installing provizio_dds build dependencies!"
