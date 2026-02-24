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
# build_cache_windows.ps1 [BUILD_TYPE=Release]

param(
    [string]$BuildType = "Release"
)

$ErrorActionPreference = "Stop"

Push-Location $PSScriptRoot
try {
    Set-Location ..\..

    $binCacheConfigName = & .\bin_cache_config_name.ps1 -BuildType $BuildType
    $binCachePath = Resolve-Path ".\cache"
    $targetPath = Join-Path $binCachePath $binCacheConfigName

    $provizioCheckFile = Join-Path $targetPath "lib\provizio_dds.lib"

    # Check if it's already built
    $alreadyBuilt = $false
    $zipFile = "${targetPath}.zip"
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

    if ($alreadyBuilt) {
        Write-Host "The bin cache is already built for $binCacheConfigName"
    } else {
        Write-Host "Building bin cache for ${binCacheConfigName}..."

        # Delete any obsolete version
        $wildcardName = & .\bin_cache_config_name.ps1 -BuildType $BuildType -ProvizioIdlsVersion "WILDCARD"
        Get-ChildItem -Path $binCachePath -Filter "${wildcardName}*" -ErrorAction SilentlyContinue |
            Remove-Item -Recurse -Force -ErrorAction SilentlyContinue

        # Build
        $buildDir = "build"
        New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
        Push-Location $buildDir
        try {
            cmake .. -G Ninja `
                "-DCMAKE_BUILD_TYPE=$BuildType" `
                "-DIGNORE_BIN_CACHE=ON" `
                "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON" `
                "-DENABLE_TESTS=OFF" `
                "-DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=OFF" `
                "-DCMAKE_INSTALL_PREFIX=$targetPath"
            if ($LASTEXITCODE -ne 0) { throw "CMake configure failed" }

            cmake --build . -- -j 16
            if ($LASTEXITCODE -ne 0) { throw "CMake build failed" }

            cmake --install .
            if ($LASTEXITCODE -ne 0) { throw "CMake install failed" }
        } finally {
            Pop-Location
        }

        # Delete extra cmake files
        $cmakeDir = Join-Path $targetPath "lib\cmake"
        if (Test-Path $cmakeDir) {
            Remove-Item -Recurse -Force $cmakeDir
        }

        # Copy Fast-DDS DLLs from the build tree to cache bin/
        $fastDdsBinDir = Join-Path $buildDir "fast_dds_build\install\bin"
        if (Test-Path $fastDdsBinDir) {
            $cacheBinDir = Join-Path $targetPath "bin"
            New-Item -ItemType Directory -Path $cacheBinDir -Force | Out-Null
            Copy-Item -Path (Join-Path $fastDdsBinDir "*.dll") -Destination $cacheBinDir -Force
        }

        # Zip it
        Compress-Archive -Path $targetPath -DestinationPath $zipFile -Force
        Remove-Item -Recurse -Force $targetPath
    }
} finally {
    Pop-Location
}
