# Track 0a — build a portable, versioned Siril-catalog snapshot that serves the REAL corpus with
# zero network access. The full per-user AppData cache is ~11 GB of ranges accumulated across every
# corpus ever run; this extracts just the subset the REAL suite touches by warming a fresh snapshot
# dir online once, then proves it is hermetic offline.
#
#   1. Seed the snapshot with the base catalog files (everything under the AppData camera dir except
#      the big accumulated siril byte caches).
#   2. Run REAL against the snapshot with OFFLINE off -> the solver fetches ONLY the ranges REAL
#      needs (Hugging Face / Zenodo) into <snapshot>\siril-spcc-cache.
#   3. Re-run REAL against the snapshot with OFFLINE on -> must be hermetic (zero offline misses).
#
# The snapshot is regenerable Gaia SPCC data, so it is gitignored (like the corpus images); this
# script + the OFFLINE gate are the durable, committed pieces. Run it when the REAL corpus or the
# upstream SPCC catalog changes.
#
# Usage:  powershell -Command "& .\refresh-catalog-snapshot.ps1 -SnapshotDir D:\snap\camera"
param([Parameter(Mandatory=$true)][string]$SnapshotDir)
$ErrorActionPreference='Stop'
$appCamera = Join-Path $env:APPDATA 'f4exb\SDRangel\camera'
if (-not (Test-Path $appCamera)) { throw "AppData camera dir not found: $appCamera (run the plate solver once online first)" }

Write-Host "1/3 Seeding base catalog into $SnapshotDir (excluding accumulated byte caches)..."
New-Item -ItemType Directory -Force $SnapshotDir | Out-Null
# robocopy /XD excludes the large regenerable byte-range caches; base catalog files are small.
robocopy $appCamera $SnapshotDir /E /XD siril-spcc-cache siril-region-cache siril-astro-region-cache /NFL /NDL /NJH /NJS /NP | Out-Null

Write-Host "2/3 Warming REAL ranges from the network into the snapshot (OFFLINE off)..."
& (Join-Path $PSScriptRoot 'validate.ps1') -Labels REAL 2>&1 | Out-Null  # populates AppData; see note below
# NOTE: validate.ps1 uses the AppData cache. To warm the SNAPSHOT specifically, run the exe with
# SDRANGEL_CAMERA_PLATE_SOLVER_CACHE_DIR set and OFFLINE unset:
$wt='C:\Users\jon\source\repos\srcejon_sdrangel_fix-review'
$exe="$wt\build-qt6\bin\plugins\featurecamera_star_tests.exe"
$test="$wt\plugins\feature\camera\test"
$env:QT_PLUGIN_PATH='C:\Qt\6.11.0\msvc2022_64\plugins'
$env:PATH=@("$wt\external\windows\fftw-3","$wt\external\windows\libsigmf\lib","$wt\build-qt6\bin",'C:\Qt\6.11.0\msvc2022_64\bin','C:\Users\jon\source\repos\sdrangel-windows-libraries\opencv4\x64\vc17\bin',$env:PATH) -join ';'
$env:SDRANGEL_CAMERA_PLATE_SOLVER_CACHE_DIR=$SnapshotDir
Remove-Item Env:\SDRANGEL_CAMERA_PLATE_SOLVER_OFFLINE -ErrorAction SilentlyContinue
& $exe "$test\star-tests.csv" 2>&1 | Out-Null
$sizeMB = [math]::Round(((Get-ChildItem -Recurse -File $SnapshotDir | Measure-Object Length -Sum).Sum/1MB),1)
Write-Host ("   snapshot size: {0} MB" -f $sizeMB)

Write-Host "3/3 Verifying the snapshot is hermetic (OFFLINE on)..."
& (Join-Path $PSScriptRoot 'hermetic.ps1') -Labels REAL -CacheDir $SnapshotDir
