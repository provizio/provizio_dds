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
    Write-Host "Installing SWIG..."
    choco install swig -y
    if ($LASTEXITCODE -ne 0) { throw "Failed to install SWIG" }

    Write-Host "Installing setuptools..."
    pip install setuptools
    if ($LASTEXITCODE -ne 0) { throw "Failed to install setuptools" }
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
