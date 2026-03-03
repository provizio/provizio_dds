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
# bin_cache_config_name.ps1 [<BUILD_TYPE>] [<PROVIZIO_DDS_IDLS_VERSION>/WILDCARD] [<PYTHON_VERSION_TAG>]

param(
    [string]$BuildType = "Release",
    [string]$ProvizioIdlsVersion = "",
    [string]$PythonVersionTag = ""
)

$ErrorActionPreference = "Stop"

Push-Location $PSScriptRoot
try {
    if (-not $ProvizioIdlsVersion) {
        $cmakeContent = Get-Content -Path ".\CMakeLists.txt" -Raw
        if ($cmakeContent -match 'set\(PROVIZIO_DDS_IDLS_VERSION\s+"([^"]+)"') {
            $ProvizioIdlsVersion = $Matches[1]
        } else {
            throw "Failed to extract PROVIZIO_DDS_IDLS_VERSION from CMakeLists.txt"
        }
    }

    $cpuArch = $env:PROCESSOR_ARCHITECTURE

    if ($ProvizioIdlsVersion -eq "WILDCARD") {
        $contentsHash = "*"
        $idlsCommitHash = "*"
    } else {
        $idlsCommitHash = (git ls-remote https://github.com/provizio/provizio_dds_idls.git |
            Select-String -Pattern "\b$([regex]::Escape($ProvizioIdlsVersion))\b" |
            ForEach-Object { ($_ -split "\s+")[0] } |
            Select-Object -First 1)

        if (-not $idlsCommitHash) {
            throw "Failed to resolve IDLS commit hash for version $ProvizioIdlsVersion"
        }
        # Truncate to 12 chars — keeps paths under MAX_PATH while remaining unique
        $idlsCommitHash = $idlsCommitHash.Substring(0, 12)

        # sha256 hash of all non-ignored files in this repo except "media" and "cache" directories
        $files = git ls-files | Where-Object { $_ -notmatch '(^|\/)media(\/)' -and $_ -notmatch '(^|\/)cache(\/)' }
        $sha256 = [System.Security.Cryptography.SHA256]::Create()
        $stream = New-Object System.IO.MemoryStream
        foreach ($file in $files) {
            # Resolve to absolute path — Push-Location changes the PS location
            # but not the .NET process CWD, so ReadAllBytes needs a full path.
            $fullPath = Join-Path (Get-Location) $file
            if (Test-Path $fullPath) {
                $bytes = [System.IO.File]::ReadAllBytes($fullPath)
                $stream.Write($bytes, 0, $bytes.Length)
            }
        }
        $stream.Position = 0
        $hashBytes = $sha256.ComputeHash($stream)
        # Truncate to 16 chars — keeps paths under MAX_PATH while remaining unique
        $contentsHash = (-join ($hashBytes | ForEach-Object { $_.ToString("x2") })).Substring(0, 16)
        $stream.Dispose()
        $sha256.Dispose()
    }

    $suffix = ""
    if ($PythonVersionTag) {
        $suffix = ".python${PythonVersionTag}"
    }

    Write-Output "windows_${cpuArch}.${contentsHash}.idls_${idlsCommitHash}.${BuildType}${suffix}"
} finally {
    Pop-Location
}
