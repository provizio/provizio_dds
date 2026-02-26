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
# build_cache_windows.ps1 [BUILD_TYPE=Release] [PYTHON=OFF|ON]

param(
    [string]$BuildType = "Release",
    [string]$Python = "OFF"
)

$ErrorActionPreference = "Stop"

Push-Location $PSScriptRoot
try {
    Set-Location ..\..

    $binCacheConfigName = & .\bin_cache_config_name.ps1 -BuildType $BuildType
    $binCachePath = Resolve-Path ".\cache"
    $targetPath = Join-Path $binCachePath $binCacheConfigName
    $pythonTargetPath = Join-Path $targetPath "python"

    # Detect Python version tag when building with Python.
    # On Windows, .pyd files link against a specific pythonXY.dll, so each minor version needs
    # its own cache (unlike Linux where 3.10-3.13 share ABI compatibility).
    $pythonVersionTag = ""
    if ($Python -eq "ON") {
        $pythonVersionTag = python -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')"
    }

    $pythonCacheConfigName = ""
    if ($pythonVersionTag) {
        $pythonCacheConfigName = & .\bin_cache_config_name.ps1 -BuildType $BuildType -PythonVersionTag $pythonVersionTag
    }

    $provizioCheckFile = Join-Path $targetPath "lib\provizio_dds.lib"

    # Check if it's already built
    $alreadyBuilt = $false
    $zipFile = "${targetPath}.zip"

    if ($Python -eq "ON") {
        # When building with Python, both general and python caches must exist
        $pythonZipFile = Join-Path $binCachePath "${pythonCacheConfigName}.zip"
        if ((Test-Path $zipFile) -and (Test-Path $pythonZipFile)) {
            $alreadyBuilt = $true

            # Verify the general cache contains expected files
            Expand-Archive -Path $zipFile -DestinationPath $binCachePath -Force
            if (-not (Test-Path $provizioCheckFile)) {
                $alreadyBuilt = $false
            }
            Remove-Item -Recurse -Force $targetPath -ErrorAction SilentlyContinue

            # Verify the python cache contains expected files
            if ($alreadyBuilt) {
                Expand-Archive -Path $pythonZipFile -DestinationPath $binCachePath -Force
                $pythonCacheExtracted = Join-Path $binCachePath $pythonCacheConfigName
                $pythonCheckFile = Join-Path $pythonCacheExtracted "python\provizio_dds_python_types\_provizio_dds_python_types.pyd"
                if (-not (Test-Path $pythonCheckFile)) {
                    $alreadyBuilt = $false
                }
                Remove-Item -Recurse -Force $pythonCacheExtracted -ErrorAction SilentlyContinue
            }
        }
    } else {
        # When building without Python, only the general cache must exist
        if (Test-Path $zipFile) {
            $alreadyBuilt = $true
            Expand-Archive -Path $zipFile -DestinationPath $binCachePath -Force

            if (-not (Test-Path $provizioCheckFile)) {
                $alreadyBuilt = $false
            }

            if (-not $alreadyBuilt) {
                Remove-Item -Recurse -Force $targetPath -ErrorAction SilentlyContinue
            }
        }
    }

    if ($alreadyBuilt) {
        Write-Host "The bin cache is already built for $binCacheConfigName"
    } else {
        Write-Host "Building bin cache for ${binCacheConfigName}..."

        # Delete any obsolete version (targeted to avoid removing Python caches for
        # other Python versions that share the same base cache name)
        $wildcardName = & .\bin_cache_config_name.ps1 -BuildType $BuildType -ProvizioIdlsVersion "WILDCARD"
        # Remove only base cache dir and zip, not python-versioned ones
        Get-ChildItem -Path $binCachePath -Filter $wildcardName -ErrorAction SilentlyContinue |
            Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
        Get-ChildItem -Path $binCachePath -Filter "${wildcardName}.zip" -ErrorAction SilentlyContinue |
            Remove-Item -Force -ErrorAction SilentlyContinue
        if ($pythonVersionTag) {
            $wildcardPythonName = & .\bin_cache_config_name.ps1 -BuildType $BuildType -ProvizioIdlsVersion "WILDCARD" -PythonVersionTag $pythonVersionTag
            Get-ChildItem -Path $binCachePath -Filter $wildcardPythonName -ErrorAction SilentlyContinue |
                Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
            Get-ChildItem -Path $binCachePath -Filter "${wildcardPythonName}.zip" -ErrorAction SilentlyContinue |
                Remove-Item -Force -ErrorAction SilentlyContinue
        }

        # Build
        $buildDir = "build"
        New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
        Push-Location $buildDir
        try {
            $cmakeArgs = @(
                "..", "-G", "Ninja",
                "-DCMAKE_BUILD_TYPE=$BuildType",
                "-DIGNORE_BIN_CACHE=ON",
                "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON",
                "-DENABLE_TESTS=OFF",
                "-DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=OFF",
                "-DPYTHON_BINDINGS=$Python",
                "-DCMAKE_INSTALL_PREFIX=$targetPath"
            )
            if ($Python -eq "ON") {
                $cmakeArgs += "-DPYTHON_PACKAGES_INSTALL_DIR=$pythonTargetPath"
            }
            cmake @cmakeArgs
            if ($LASTEXITCODE -ne 0) { throw "CMake configure failed" }

            cmake --build . -- -j 16
            if ($LASTEXITCODE -ne 0) { throw "CMake build failed" }

            cmake --install .
            if ($LASTEXITCODE -ne 0) { throw "CMake install failed" }
        } finally {
            Pop-Location
        }

        # Delete extra copy of python-specific libs produced by Fast-DDS Python wrapper
        $pythonLibDirs = Get-ChildItem -Path (Join-Path $targetPath "lib") -Filter "python*" -Directory -ErrorAction SilentlyContinue
        if ($pythonLibDirs) {
            $pythonLibDirs | Remove-Item -Recurse -Force
        }

        if ($Python -eq "ON") {
            # Create python-versioned cache zip (python/ only)
            $pythonCacheDir = Join-Path $binCachePath $pythonCacheConfigName
            New-Item -ItemType Directory -Path $pythonCacheDir -Force | Out-Null
            Move-Item -Path $pythonTargetPath -Destination (Join-Path $pythonCacheDir "python")
            $pythonZipDest = Join-Path $binCachePath "${pythonCacheConfigName}.zip"
            Compress-Archive -Path $pythonCacheDir -DestinationPath $pythonZipDest -Force
            Remove-Item -Recurse -Force $pythonCacheDir
        }

        # Create general cache zip (include/, lib/, bin/ — no python/)
        # Skip if it already exists (e.g., when only rebuilding for a new Python version)
        if (-not (Test-Path $zipFile)) {
            Compress-Archive -Path $targetPath -DestinationPath $zipFile -Force
        }
        Remove-Item -Recurse -Force $targetPath
    }
} finally {
    Pop-Location
}
