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

# A C++ consumer's install from the prebuilt binaries on Windows, as test_bin_cache_install.sh checks
# it on Linux: the plain Release configure of an unmodified checkout, on the commit CI publishes the
# binaries in, then an install into a prefix of its own. Checks that the configure took the prebuilt
# binaries; that every DLL the installed ones import is installed beside them in bin/, or is the
# system's -- but the OpenSSL their Fast-DDS was built against and the C++ runtime, which they carry,
# always beside them; that no DLL went to lib/, which holds import libraries; and that
# provizio_dds.dll, loaded from there, loads Fast-DDS and OpenSSL from there too.
#
# Use as: test_bin_cache_install_windows.ps1 (from anywhere; builds in <repository>\build), in a shell
# that has dumpbin, as the Developer PowerShell of Visual Studio does

$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..\..")

function Fail([string]$message) {
    Write-Host "::error::$message"
    exit 1
}

$configureLog = (New-TemporaryFile).FullName
$prefix = Join-Path ([System.IO.Path]::GetTempPath()) ("provizio_dds_install_" + [guid]::NewGuid().ToString("N"))
try {
    cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON 2>&1 |
        Tee-Object -FilePath $configureLog
    if ($LASTEXITCODE -ne 0) { Fail "The configure failed" }
    if (-not (Select-String -LiteralPath $configureLog -Pattern "Prebuilt binaries are located and will be used" -SimpleMatch -Quiet)) {
        Fail "The configure did not take the prebuilt binaries, so their install cannot be checked"
    }
    cmake --install build --prefix $prefix
    if ($LASTEXITCODE -ne 0) { Fail "The install failed" }

    $bin = Join-Path $prefix "bin"
    $installed = @(Get-ChildItem -LiteralPath $bin -Filter "*.dll" -File | ForEach-Object { $_.Name })
    Write-Host "Installed in bin/: $($installed -join ', ')"
    foreach ($pattern in "provizio_dds.dll", "provizio_dds_types.dll", "fastdds*.dll", "fastcdr*.dll", "libssl-3*.dll", "libcrypto-3*.dll") {
        if (-not ($installed | Where-Object { $_ -like $pattern })) {
            Fail "The install holds no $pattern in bin/, though the prebuilt binaries carry one"
        }
    }
    $misplaced = @(Get-ChildItem -LiteralPath (Join-Path $prefix "lib") -Filter "*.dll" -File -Recurse -ErrorAction SilentlyContinue)
    if ($misplaced.Count -gt 0) {
        Fail "The install put DLLs in lib/, where nothing loads them from: $(($misplaced | ForEach-Object { $_.FullName }) -join ', ')"
    }

    # What each installed DLL imports, eagerly or delay-loaded: beside it, or the system's (an API set
    # such as api-ms-win-*, or a DLL of the system directory). Not the C++ runtime, which the prebuilt
    # binaries carry for the machines without the Visual C++ redistributable (see
    # build_cache_windows.ps1): one this runner has in its system directory is still one they must hold.
    $system = [Environment]::SystemDirectory
    foreach ($dll in $installed) {
        $dependents = & dumpbin /nologo /dependents (Join-Path $bin $dll)
        if ($LASTEXITCODE -ne 0) { Fail "dumpbin could not read $dll" }
        foreach ($line in $dependents) {
            if ($line -notmatch '^\s+(\S+\.dll)\s*$') { continue }
            $import = $Matches[1]
            if ($installed -contains $import) { continue }
            if ($import -like "vcruntime*" -or $import -like "msvcp*" -or $import -like "concrt*" -or $import -like "vcomp*") {
                Fail "$dll imports $import, of the C++ runtime the prebuilt binaries carry, which the install does not hold"
            }
            if ($import -like "api-ms-win-*" -or $import -like "ext-ms-*") { continue }
            if (Test-Path -LiteralPath (Join-Path $system $import)) { continue }
            Fail "$dll imports $import, which is neither installed beside it nor the system's"
        }
    }
    Write-Host "Verified: every DLL the installed ones import is installed beside them or is the system's"

    # Loaded in a process of its own, which this one could then not remove the install from while it
    # held the DLLs; with what it loads looked for in its own directory first, as a consumer's program
    # finds bin/ beside itself (LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR, then the default directories). The
    # Fast-DDS and OpenSSL that come with it must be the ones that process has afterwards.
    $loaded = @(& pwsh -NoProfile -NonInteractive -Command {
            param($dll)
            Add-Type -Namespace ProvizioDdsTest -Name Loader -MemberDefinition '[DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)] public static extern IntPtr LoadLibraryExW(string path, IntPtr file, uint flags);'
            # LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR (0x100) | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS (0x1000)
            $handle = [ProvizioDdsTest.Loader]::LoadLibraryExW($dll, [IntPtr]::Zero, 0x1100)
            if ($handle -eq [IntPtr]::Zero) {
                [Console]::Error.WriteLine("LoadLibraryExW failed with error $([Runtime.InteropServices.Marshal]::GetLastWin32Error())")
                exit 1
            }
            (Get-Process -Id $PID).Modules | ForEach-Object { $_.FileName }
        } -args (Join-Path $bin "provizio_dds.dll"))
    if ($LASTEXITCODE -ne 0) { Fail "provizio_dds.dll could not be loaded from the install (see the error above)" }
    $ours = @($loaded | Where-Object {
            $leaf = Split-Path -Leaf $_
            $leaf -like "provizio_dds*" -or $leaf -like "fastdds*" -or $leaf -like "fastcdr*" -or
            $leaf -like "libssl*" -or $leaf -like "libcrypto*" })
    foreach ($kind in "provizio_dds.dll", "fastdds*", "libssl*", "libcrypto*") {
        if (-not ($ours | Where-Object { (Split-Path -Leaf $_) -like $kind })) {
            Fail "Loading provizio_dds.dll loaded no $kind"
        }
    }
    $binPrefix = $bin.TrimEnd('\') + '\'
    foreach ($path in $ours) {
        Write-Host "Loaded: $path"
        if (-not $path.StartsWith($binPrefix, [StringComparison]::OrdinalIgnoreCase)) {
            Fail "$(Split-Path -Leaf $path) was loaded from $path, not from the install's bin/"
        }
    }
    Write-Host "Verified: the installed provizio_dds loads Fast-DDS and its OpenSSL runtime from the install"
} finally {
    Remove-Item -LiteralPath $configureLog -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $prefix -Recurse -Force -ErrorAction SilentlyContinue
}
