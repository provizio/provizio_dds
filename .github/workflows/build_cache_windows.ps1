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
# build_cache_windows.ps1 [BUILD_TYPE=Release] [PYTHON=OFF|ON]

param(
    [string]$BuildType = "Release",
    [string]$Python = "OFF"
)

$ErrorActionPreference = "Stop"

# Collect transitive DLL dependencies for a single binary (mirrors Linux collect_libs in build_cache.sh).
# Uses dumpbin /dependents to resolve the PE import table, then recursively copies non-system DLLs.
function Collect-DllDependencies {
    param(
        [string]$Binary,
        [string]$OutputDir,
        [hashtable]$DllIndex = @{},
        [string]$AdditionalSearchDir = ""
    )

    # Parse dumpbin output to get dependency DLL names.
    # Only collect regular dependencies, not delay-load dependencies (which are optional).
    $dumpbinOutput = & dumpbin /dependents $Binary 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "dumpbin failed for ${Binary}: $dumpbinOutput"
    }

    $inDelayLoad = $false
    $dependencies = @($dumpbinOutput | ForEach-Object {
        if ($_ -match "delay load dependencies") {
            $inDelayLoad = $true
        }
        if (-not $inDelayLoad -and $_ -match '^\s+(\S+\.dll)\s*$') {
            $Matches[1]
        }
    })

    foreach ($dllName in $dependencies) {
        if (-not $dllName) { continue }

        # Skip if already in output directory (also breaks circular dependencies)
        if (Test-Path (Join-Path $OutputDir $dllName)) { continue }

        # Skip API set forwarders (always present on any Windows installation)
        if ($dllName -like "api-ms-win-*" -or $dllName -like "ext-ms-*") { continue }

        # Skip Python interpreter DLLs — the consumer must have their own Python installation
        if ($dllName -like "python*.dll") { continue }

        # MSVC runtime DLLs should be included even if found in system directories,
        # as they may be absent on machines without the VC++ redistributable
        $isMsvcRuntime = ($dllName -like "vcruntime*" -or $dllName -like "msvcp*" -or
                          $dllName -like "concrt*" -or $dllName -like "vcomp*")

        # Search for the DLL in priority order.
        # The DLL index is pre-built with build-dir first, then MSVC redist, then PATH
        # directories (excluding C:\Windows). Build-dir entries take precedence to avoid
        # ABI-incompatible copies (e.g. MinGW OpenSSL on PATH).
        $resolvedPath = $null

        # 1. Additional search directory (e.g., C++ bin/ when processing Python dirs)
        if (-not $resolvedPath -and $AdditionalSearchDir -and
            (Test-Path (Join-Path $AdditionalSearchDir $dllName))) {
            $resolvedPath = (Join-Path $AdditionalSearchDir $dllName)
        }

        # 2. Pre-built index of build directory and MSVC redist (O(1) lookup)
        if (-not $resolvedPath) {
            $indexKey = $dllName.ToLower()
            if ($DllIndex.ContainsKey($indexKey)) {
                $resolvedPath = $DllIndex[$indexKey]
            }
        }

        # Skip system DLLs (under C:\Windows) unless they are MSVC runtime DLLs.
        # This check must happen before the "not found" throw because dumpbin lists
        # system DLLs like KERNEL32.dll by name — they won't be in the build/redist
        # index but do exist under C:\Windows\System32.
        $windowsDir = if ($env:SystemRoot) { $env:SystemRoot } else { "C:\Windows" }
        if (-not $resolvedPath) {
            $systemPath = Join-Path "${windowsDir}\System32" $dllName
            if (Test-Path $systemPath) {
                if ($isMsvcRuntime) {
                    $resolvedPath = $systemPath
                } else {
                    continue
                }
            }
        }

        if (-not $resolvedPath) {
            throw "Dependency '$dllName' of '$(Split-Path $Binary -Leaf)' not found in any search location"
        }

        # Also skip if resolved from index/additional dir but path is under C:\Windows
        if ($resolvedPath -like "${windowsDir}\*" -and -not $isMsvcRuntime) {
            continue
        }

        # Copy to output directory and recurse
        Copy-Item -Path $resolvedPath -Destination (Join-Path $OutputDir $dllName) -Force
        Write-Host "  Collected: $dllName (from $resolvedPath)"

        Collect-DllDependencies -Binary (Join-Path $OutputDir $dllName) -OutputDir $OutputDir `
            -DllIndex $DllIndex -AdditionalSearchDir $AdditionalSearchDir
    }
}

# Process all DLLs/PYDs in a directory (mirrors Linux collect_all_libs in build_cache.sh).
function Collect-AllDllDependencies {
    param(
        [string]$Directory,
        [hashtable]$DllIndex = @{},
        [string]$AdditionalSearchDir = ""
    )

    Write-Host "Collecting transitive DLL dependencies in: $Directory"

    Get-ChildItem -Path $Directory -File | Where-Object {
        $_.Extension -in @('.dll', '.pyd')
    } | ForEach-Object {
        Write-Host "Processing: $($_.Name)"
        Collect-DllDependencies -Binary $_.FullName -OutputDir $Directory `
            -DllIndex $DllIndex -AdditionalSearchDir $AdditionalSearchDir
    }
}

# Build a DLL index from search directories for O(1) lookups.
# Directories are processed in priority order — first entry wins.
function Build-DllIndex {
    param(
        [string[]]$SearchRoots
    )

    $index = @{}
    foreach ($root in $SearchRoots) {
        if (-not $root -or -not (Test-Path $root)) { continue }
        Get-ChildItem -Path $root -Filter "*.dll" -Recurse -File -ErrorAction SilentlyContinue | ForEach-Object {
            $name = $_.Name.ToLower()
            if (-not $index.ContainsKey($name)) {
                $index[$name] = $_.FullName
            }
        }
    }
    Write-Host "  DLL index: $($index.Count) entries from $($SearchRoots.Count) search roots"
    return $index
}

Push-Location $PSScriptRoot
try {
    Set-Location ..\..

    $binCacheConfigName = & .\bin_cache_config_name.ps1 -BuildType $BuildType
    $binCachePath = Resolve-Path ".\cache"
    $targetPath = Join-Path $binCachePath $binCacheConfigName
    $pythonTargetPath = Join-Path $targetPath "python"

    # Detect Python version tag when building with Python.
    # On Windows, .pyd files link against a specific pythonXY.dll, so each minor version needs
    # its own cache (unlike Linux where 3.8-3.13 share ABI compatibility).
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

        # Collect transitive DLL dependencies (mirrors Linux collect_all_libs in build_cache.sh).
        # Build the DLL index once and reuse across all calls.
        $dllSearchRoots = @()
        if (Test-Path $buildDir) { $dllSearchRoots += (Resolve-Path $buildDir).Path }
        if ($env:VCToolsRedistDir) {
            $redistX64 = Join-Path $env:VCToolsRedistDir "x64"
            if (Test-Path $redistX64) { $dllSearchRoots += $redistX64 }
        }
        $dllIndex = Build-DllIndex -SearchRoots $dllSearchRoots

        # Also index DLLs found on PATH (non-recursive, lowest priority).
        # This finds DLLs from system-installed packages (e.g. OpenSSL) that the
        # build linked against but that aren't in the build directory or MSVC redist.
        # Build-dir and redist entries take precedence (first-entry-wins in the index).
        $windowsDirForPath = if ($env:SystemRoot) { $env:SystemRoot } else { "C:\Windows" }
        foreach ($pathDir in ($env:PATH -split ';')) {
            if (-not $pathDir -or -not (Test-Path $pathDir -PathType Container)) { continue }
            if ($pathDir -like "${windowsDirForPath}\*" -or $pathDir -eq $windowsDirForPath) { continue }
            Get-ChildItem -Path $pathDir -Filter "*.dll" -File -ErrorAction SilentlyContinue | ForEach-Object {
                $name = $_.Name.ToLower()
                if (-not $dllIndex.ContainsKey($name)) {
                    $dllIndex[$name] = $_.FullName
                }
            }
        }
        Write-Host "  DLL index after PATH scan: $($dllIndex.Count) entries"

        Collect-AllDllDependencies -Directory (Join-Path $targetPath "bin") `
            -DllIndex $dllIndex
        if ($Python -eq "ON") {
            # Collect DLL dependencies for the provizio_dds Python package only.
            # This matches the runtime DLL loading model: provizio_dds.py adds only its own
            # directory to the DLL search path via os.add_dll_directory() on Windows.
            $binDir = Join-Path $targetPath "bin"
            Collect-AllDllDependencies -Directory (Join-Path $pythonTargetPath "provizio_dds") `
                -DllIndex $dllIndex -AdditionalSearchDir $binDir
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
