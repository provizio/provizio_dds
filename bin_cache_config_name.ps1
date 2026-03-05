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

    # NOTE: $env:PROCESSOR_ARCHITECTURE reflects the OS architecture, not the build
    # target. If cross-compiling (e.g. x86 on ARM64 Windows), this would produce a
    # wrong cache name. Currently only AMD64 is targeted, so this is safe.
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
        # Truncate to 12 chars — keeps the cache directory name under Windows MAX_PATH (260).
        # The Linux script (bin_cache_config_name.sh) uses full hashes because Linux has no
        # practical path length limit.
        $idlsCommitHash = $idlsCommitHash.Substring(0, 12)

        # sha256 hash of all non-ignored files in this repo except "media" and "cache" directories.
        # CR bytes (0x0D) are stripped before hashing so the result is identical regardless of
        # core.autocrlf setting (CRLF vs LF on disk).
        $files = git ls-files | Where-Object { $_ -notmatch '(^|\/)media(\/)' -and $_ -notmatch '(^|\/)cache(\/)' }
        $sha256 = [System.Security.Cryptography.SHA256]::Create()
        $stream = New-Object System.IO.MemoryStream
        foreach ($file in $files) {
            # Resolve to absolute path — Push-Location changes the PS location
            # but not the .NET process CWD, so ReadAllBytes needs a full path.
            $fullPath = Join-Path (Get-Location) $file
            if (Test-Path $fullPath) {
                $raw = [System.IO.File]::ReadAllBytes($fullPath)
                # Strip CR bytes so CRLF and LF produce the same hash
                $filtered = [System.Linq.Enumerable]::Where([byte[]]$raw, [Func[byte,bool]]{ param($b) $b -ne 13 })
                $bytes = [System.Linq.Enumerable]::ToArray([System.Collections.Generic.IEnumerable[byte]]$filtered)
                $stream.Write($bytes, 0, $bytes.Length)
            }
        }
        $stream.Position = 0
        $hashBytes = $sha256.ComputeHash($stream)
        # Truncate to 16 chars — same MAX_PATH rationale as the IDLS hash above.
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
