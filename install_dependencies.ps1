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
    $swigUrl = "https://sourceforge.net/projects/swig/files/swigwin/swigwin-$swigVersion/swigwin-$swigVersion.zip/download"

    Write-Host "Installing SWIG $swigVersion..."
    # Use curl.exe (available on all modern Windows) — Invoke-WebRequest chokes on
    # SourceForge redirects in PowerShell 7 on Windows Server.
    & curl.exe -fsSL -o $swigZip $swigUrl
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
}

if ($StaticAnalysis -ne "OFF") {
    Write-Host "Installing LLVM (clang-format, clang-tidy)..."
    choco install llvm -y
    if ($LASTEXITCODE -ne 0) { throw "Failed to install LLVM" }

    Write-Host "Installing cppcheck..."
    choco install cppcheck -y
    if ($LASTEXITCODE -ne 0) { throw "Failed to install cppcheck" }
}

Write-Host "Done installing provizio_dds build dependencies!"
