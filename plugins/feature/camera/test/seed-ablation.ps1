# Seed-engine ablation harness.
# Runs the corpora with each surviving seed engine disabled in turn and reports, per engine,
# which test cases regress (PASS -> FAIL) = that engine's marginal/unique contribution.
#
# Engines: brighttriangle (triangle, guided), brightpair (pair, wide-blind), vectorquad (quad,
# blind). blindquad was deleted (zero marginal) and blindtriangle's standalone use was removed;
# the seed layer was found NOT unifiable (engines are regime-specialised) -- see
# doc/camera/plate-solver-notes.md "Seed-engine unification -- ABANDONED". This harness stays for
# regression/marginal-contribution checks when the seed stage is touched.
$bd = "C:\Users\jon\source\repos\srcejon_sdrangel_fix\build-qt6"
$env:PATH = "C:\Qt\6.11.0\msvc2022_64\bin;$bd\bin;" + $env:PATH
$exe = "$bd\bin\plugins\featurecamera_star_tests.exe"
$tdir = "C:\Users\jon\source\repos\srcejon_sdrangel_fix\plugins\feature\camera\test"
$suites = [ordered]@{
  REAL    = "$tdir\star-tests.csv"
  FISHEYE = "$tdir\star-tests-synthetic-fisheye.csv"
  WIDE    = "$tdir\star-tests-synthetic-wide.csv"
  RAND    = "$tdir\star-tests-synthetic-rand.csv"
  RAND2   = "$tdir\star-tests-synthetic-rand2.csv"
}

# config name -> @{ env-var = value }
$configs = [ordered]@{
  baseline       = @{}
  brighttriangle = @{ SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_SEEDS = "brighttriangle" }
  brightpair     = @{ SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_SEEDS = "brightpair" }
  vectorquad     = @{ SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_QUAD_INDEX = "1" }
}

function Clear-AblationEnv {
  Remove-Item Env:\SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_SEEDS -ErrorAction SilentlyContinue
  Remove-Item Env:\SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_QUAD_INDEX -ErrorAction SilentlyContinue
}

# Run one suite, return @{ pass=<int>; fails=@(<image names, one per FAIL line>) }
function Run-Suite($csv) {
  if (-not (Test-Path $csv)) { return @{ pass = -1; fails = @() } }
  $out = & $exe $csv 2>&1
  $passLines = @($out | Select-String -Pattern "^PASS " -CaseSensitive)
  $failLines = @($out | Select-String -Pattern "^FAIL " -CaseSensitive)
  $fails = $failLines | ForEach-Object { ($_.ToString() -split ":")[0] -replace "^FAIL ", "" }
  return @{ pass = $passLines.Count; fails = @($fails) }
}

$results = [ordered]@{}
foreach ($cfg in $configs.Keys) {
  Clear-AblationEnv
  foreach ($kv in $configs[$cfg].GetEnumerator()) { Set-Item -Path "Env:\$($kv.Key)" -Value $kv.Value }
  $perSuite = [ordered]@{}
  foreach ($s in $suites.Keys) { $perSuite[$s] = Run-Suite $suites[$s] }
  $results[$cfg] = $perSuite
  $summary = ($suites.Keys | ForEach-Object { "$_=$($perSuite[$_].pass)" }) -join " "
  Write-Output "[done] $cfg : $summary"
}
Clear-AblationEnv

Write-Output ""
Write-Output "================ ABLATION REPORT ================"
$base = $results["baseline"]
Write-Output ("baseline PASS: " + (($suites.Keys | ForEach-Object { "$_=$($base[$_].pass)" }) -join "  "))
Write-Output "(suite = -1 means its CSV is absent -- synthetic corpora are gitignored/regenerable)"
Write-Output ""
foreach ($cfg in $configs.Keys) {
  if ($cfg -eq "baseline") { continue }
  Write-Output "--- disable: $cfg ---"
  $totalRegressed = 0
  foreach ($s in $suites.Keys) {
    $baseFails = @($base[$s].fails)
    $cfgFails  = @($results[$cfg][$s].fails)
    $newFails = @()
    $baseRemaining = New-Object System.Collections.ArrayList
    $baseFails | ForEach-Object { [void]$baseRemaining.Add($_) }
    foreach ($img in $cfgFails) {
      if ($baseRemaining.Contains($img)) { [void]$baseRemaining.Remove($img) }
      else { $newFails += $img }
    }
    $delta = $base[$s].pass - $results[$cfg][$s].pass
    $totalRegressed += [Math]::Max(0, $delta)
    $nf = if ($newFails.Count) { ($newFails -join ", ") } else { "(none)" }
    Write-Output ("  {0,-7} PASS {1} -> {2}  (delta {3})  newly-failing: {4}" -f $s, $base[$s].pass, $results[$cfg][$s].pass, $delta, $nf)
  }
  Write-Output ("  => marginal cases lost by disabling {0}: {1}" -f $cfg, $totalRegressed)
}
Write-Output "================================================="
