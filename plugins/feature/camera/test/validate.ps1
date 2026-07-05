# Run one or more corpus splits and diff verdicts vs the Phase 0 baseline.
# Usage: powershell -Command "& .\validate.ps1 -Labels REAL,FISHEYE-mode1,WIDE [-V2]"
param([Parameter(Mandatory=$true)][string[]]$Labels, [switch]$V2, [switch]$Legacy)
$ErrorActionPreference='Continue'
# Detector V2 is now DEFAULT ON. -Legacy sets the kill-switch to run the legacy detector (matches
# the baseline verdict files). -V2 is a no-op kept for older call sites.
if ($Legacy) { $env:SDRANGEL_CAMERA_STAR_DETECTOR_DISABLE_V2='1' }
$wt='C:\Users\jon\source\repos\srcejon_sdrangel_fix-review'
$exe="$wt\build-qt6\bin\plugins\featurecamera_star_tests.exe"
$test="$wt\plugins\feature\camera\test"; $base="$test\baseline-2026-07"
$env:QT_PLUGIN_PATH='C:\Qt\6.11.0\msvc2022_64\plugins'
$env:PATH=@('C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.2\bin',"$wt\external\windows\fftw-3","$wt\external\windows\libsigmf\lib","$wt\build-qt6\bin",'C:\Qt\6.11.0\msvc2022_64\bin','C:\Users\jon\source\repos\sdrangel-windows-libraries\opencv4\x64\vc17\bin',$env:PATH) -join ';'
$csvFor=@{ 'REAL'='star-tests.csv'; 'FISHEYE'='star-tests-synthetic-fisheye.csv';
  'FISHEYE-mode1'='star-tests-synthetic-fisheye-mode1.csv'; 'FISHEYE-mode4'='star-tests-synthetic-fisheye-mode4.csv';
  'WIDE'='star-tests-synthetic-wide.csv'; 'RAND'='star-tests-synthetic-rand.csv'; 'RAND2'='star-tests-synthetic-rand2.csv' }
foreach ($label in $Labels) {
  $csv="$test\$($csvFor[$label])"
  $sw=[System.Diagnostics.Stopwatch]::StartNew()
  $r = & $exe $csv 2>&1
  $sw.Stop()
  $v = $r | Where-Object { $_ -match '^(PASS|FAIL)\s' } | ForEach-Object { ($_ -split '\s+')[0,1] -join ' ' }
  $pass=($v|?{$_ -like 'PASS *'}).Count; $fail=($v|?{$_ -like 'FAIL *'}).Count
  $basefile="$base\$label.verdicts.txt"
  $diffMsg='(no baseline to diff)'
  if (Test-Path $basefile) {
    $d = Compare-Object (Get-Content $basefile) $v
    if ($d) { $diffMsg = "CHANGED: " + (($d | ForEach-Object { "$($_.SideIndicator) $($_.InputObject)" }) -join '; ') }
    else { $diffMsg = 'verdict set IDENTICAL to baseline' }
  }
  "{0,-14} PASS={1} FAIL={2} time={3:n0}s  {4}" -f $label,$pass,$fail,$sw.Elapsed.TotalSeconds,$diffMsg
}
