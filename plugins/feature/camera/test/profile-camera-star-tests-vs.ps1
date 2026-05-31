[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Executable,

    [Parameter(Mandatory = $true)]
    [string]$Csv,

    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,

    [Parameter(Mandatory = $true)]
    [string]$ProfileOutputDirectory,

    [string]$QtPluginPath,

    [string]$RuntimePath = "",

    [string]$AgentConfigName = "CpuUsageHigh.json",

    [switch]$Warmup
)

$ErrorActionPreference = "Stop"

function Resolve-RequiredFile([string]$Path, [string]$Description)
{
    $resolved = Resolve-Path -LiteralPath $Path -ErrorAction SilentlyContinue
    if (-not $resolved) {
        throw "$Description does not exist: $Path"
    }
    return $resolved.ProviderPath
}

function Resolve-VisualStudioInstall()
{
    $candidates = @()

    if ($env:VSINSTALLDIR) {
        $candidates += $env:VSINSTALLDIR
    }

    $programFilesX86 = ${env:ProgramFiles(x86)}
    if ($programFilesX86) {
        $vswhere = Join-Path $programFilesX86 "Microsoft Visual Studio\Installer\vswhere.exe"
        if (Test-Path -LiteralPath $vswhere) {
            $installPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
            if ($installPath) {
                $candidates += $installPath
            }
            $installPath = & $vswhere -latest -products * -property installationPath 2>$null
            if ($installPath) {
                $candidates += $installPath
            }
        }
    }

    $programFiles = $env:ProgramFiles
    if ($programFiles) {
        $candidates += Join-Path $programFiles "Microsoft Visual Studio\2022\Community"
        $candidates += Join-Path $programFiles "Microsoft Visual Studio\2022\Professional"
        $candidates += Join-Path $programFiles "Microsoft Visual Studio\2022\Enterprise"
        $candidates += Join-Path $programFiles "Microsoft Visual Studio\2022\BuildTools"
    }

    foreach ($candidate in ($candidates | Where-Object { $_ } | Select-Object -Unique)) {
        $collector = Join-Path $candidate "Team Tools\DiagnosticsHub\Collector\VSDiagnostics.exe"
        if (Test-Path -LiteralPath $collector) {
            return $candidate
        }
    }

    throw "Could not find Visual Studio Diagnostics tools. Install Visual Studio with the Diagnostics Tools components, or run from a Developer Command Prompt with VSINSTALLDIR set."
}

function Resolve-WindowsPerformanceToolkitTool([string]$ToolName)
{
    $candidates = @()
    $programFilesX86 = ${env:ProgramFiles(x86)}
    if ($programFilesX86) {
        $candidates += Join-Path $programFilesX86 "Windows Kits\10\Windows Performance Toolkit\$ToolName"
        $candidates += Join-Path $programFilesX86 "Windows Kits\8.1\Windows Performance Toolkit\$ToolName"
    }

    $pathCommand = Get-Command $ToolName -ErrorAction SilentlyContinue
    if ($pathCommand) {
        $candidates += $pathCommand.Source
    }

    foreach ($candidate in ($candidates | Where-Object { $_ } | Select-Object -Unique)) {
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).ProviderPath
        }
    }

    return ""
}

function Invoke-LoggedCommand([string]$Program, [string[]]$Arguments, [string]$LogFile)
{
    $display = "`"$Program`" " + ($Arguments -join " ")
    Add-Content -LiteralPath $LogFile -Value $display
    $oldErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & $Program @Arguments 2>&1 | Tee-Object -FilePath $LogFile -Append | Out-Host
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $oldErrorActionPreference
    }
    return $exitCode
}

function Test-WindowsLoaderFailure([int]$ExitCode)
{
    return (($ExitCode -eq -1073741515) -or ($ExitCode -eq -1073741511))
}

function Get-NormalizedLogText([string]$LogFile)
{
    if (-not (Test-Path -LiteralPath $LogFile)) {
        return ""
    }

    return (Get-Content -LiteralPath $LogFile -Raw) -replace "`0", ""
}

$Executable = Resolve-RequiredFile $Executable "Camera star test executable"
$Csv = Resolve-RequiredFile $Csv "Camera star test CSV"

New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
New-Item -ItemType Directory -Force -Path $ProfileOutputDirectory | Out-Null

if ($QtPluginPath) {
    $env:QT_PLUGIN_PATH = $QtPluginPath
}

$runtimePaths = @()
foreach ($path in ($RuntimePath -split "\|")) {
    if ($path -and (Test-Path -LiteralPath $path)) {
        $runtimePaths += (Resolve-Path -LiteralPath $path).ProviderPath
    }
}

if ($runtimePaths.Count -gt 0) {
    $env:PATH = ($runtimePaths -join ";") + ";" + $env:PATH
}

$vsInstall = Resolve-VisualStudioInstall
$vsDiagnostics = Resolve-RequiredFile (Join-Path $vsInstall "Team Tools\DiagnosticsHub\Collector\VSDiagnostics.exe") "VSDiagnostics.exe"
$agentConfig = Resolve-RequiredFile (Join-Path $vsInstall "Team Tools\DiagnosticsHub\Collector\AgentConfigs\$AgentConfigName") "Visual Studio diagnostics agent config"

$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
$sessionId = [Guid]::NewGuid().ToString()
$scratchDirectory = Join-Path $ProfileOutputDirectory "scratch-$timestamp"
$diagsession = Join-Path $ProfileOutputDirectory "camera-star-tests-$timestamp.diagsession"
$logFile = Join-Path $ProfileOutputDirectory "camera-star-tests-$timestamp.log"

New-Item -ItemType Directory -Force -Path $scratchDirectory | Out-Null
New-Item -ItemType File -Force -Path $logFile | Out-Null

$env:SDRANGEL_CAMERA_PLATE_SOLVER_PROFILE = "1"

if ($Warmup) {
    Write-Host "Running unprofiled warmup pass..."
    $warmupExitCode = Invoke-LoggedCommand $Executable @($Csv, $OutputDirectory) $logFile
    if ($warmupExitCode -ne 0) {
        if (Test-WindowsLoaderFailure $warmupExitCode) {
            throw "Warmup test run failed with Windows loader exit code $warmupExitCode. Check the runtime PATH for missing DLLs. See $logFile"
        }
        Write-Warning "Warmup test run returned exit code $warmupExitCode; continuing so the profiled run can still be captured."
    }
}

Write-Host "Starting Visual Studio Diagnostics session..."
Write-Host "  Test: $Executable"
Write-Host "  CSV:  $Csv"
Write-Host "  Out:  $diagsession"

$testWorkingDirectory = Split-Path -Parent $Executable
$testOutputFile = Join-Path $ProfileOutputDirectory "camera-star-tests-$timestamp-test-output.log"
$testErrorFile = Join-Path $ProfileOutputDirectory "camera-star-tests-$timestamp-test-error.log"
Remove-Item -LiteralPath $testOutputFile, $testErrorFile -Force -ErrorAction SilentlyContinue

$testProcess = Start-Process `
    -FilePath $Executable `
    -ArgumentList @($Csv, $OutputDirectory) `
    -WorkingDirectory $testWorkingDirectory `
    -NoNewWindow `
    -PassThru `
    -RedirectStandardOutput $testOutputFile `
    -RedirectStandardError $testErrorFile

$startArgs = @(
    "start",
    $sessionId,
    "/attach:$($testProcess.Id)",
    "/loadConfig:$agentConfig",
    "/scratchLocation:$scratchDirectory",
    "/package:opt"
)

$startExitCode = Invoke-LoggedCommand $vsDiagnostics $startArgs $logFile
if ($startExitCode -ne 0) {
    Write-Warning "Visual Studio Diagnostics start returned exit code $startExitCode. Attempting to stop/package the session anyway."
}

# Avoid /monitor because it can fail with a COM registration error on some
# installs, and avoid /lifetimeProcess because it can close the session before
# we package it. Wait explicitly, then stop/package the session ourselves.
Write-Host "Waiting for camera star tests to finish..."
$testProcess.WaitForExit()
$testProcess.Refresh()
$testExitCode = $testProcess.ExitCode
$testOutputText = ""
if (Test-Path -LiteralPath $testOutputFile) {
    $testOutputText = Get-Content -LiteralPath $testOutputFile -Raw
    $testOutputText | Tee-Object -FilePath $logFile -Append | Out-Host
}
if (Test-Path -LiteralPath $testErrorFile) {
    Get-Content -LiteralPath $testErrorFile -Raw | Tee-Object -FilePath $logFile -Append | Out-Host
}

$stopArgs = @("stop", $sessionId, "/output:$diagsession")
$stopExitCode = Invoke-LoggedCommand $vsDiagnostics $stopArgs $logFile
if (($stopExitCode -ne 0) -and -not (Test-Path -LiteralPath $diagsession)) {
    $logText = Get-NormalizedLogText $logFile
    if ($logText -match "Class not registered") {
        throw "Visual Studio Diagnostics could not create a collector session because a DiagnosticsHub COM component is not registered. Try repairing Visual Studio Diagnostics Tools, or run this target from a full Visual Studio Developer Command Prompt. See $logFile"
    }
    throw "Visual Studio Diagnostics stop failed with exit code $stopExitCode and no .diagsession was created. See $logFile"
}

if (-not (Test-Path -LiteralPath $diagsession)) {
    $logText = Get-NormalizedLogText $logFile
    if ($logText -match "Class not registered") {
        throw "Visual Studio Diagnostics did not create a .diagsession because a DiagnosticsHub COM component is not registered. This script avoids /monitor, so check that the CPU Usage DiagnosticsHub components are installed and registered. See $logFile"
    }
    throw "Visual Studio Diagnostics completed but did not create $diagsession. See $logFile"
}

if ($null -eq $testExitCode) {
    if (($testOutputText -match "\d+ camera star test\(s\) passed") -and ($testOutputText -notmatch "(?m)^FAIL ")) {
        Write-Warning "Profiled test process exit code was not reported by Start-Process; treating the completed PASS output as success."
        $testExitCode = 0
    } else {
        throw "Profiled test run finished, but PowerShell did not report an exit code. Visual Studio profile was written to $diagsession. See $logFile"
    }
}

if ($testExitCode -ne 0) {
    if (Test-WindowsLoaderFailure $testExitCode) {
        throw "Profiled test run failed with Windows loader exit code $testExitCode. Check the runtime PATH for missing DLLs. See $logFile"
    }
    throw "Profiled test run failed with exit code $testExitCode. Visual Studio profile was written to $diagsession. See $logFile"
}

$expandedSessionDirectory = [System.IO.Path]::ChangeExtension($diagsession, $null)
$profileDetailFile = Join-Path $ProfileOutputDirectory "camera-star-tests-$timestamp-xperf-profile-detail-local.txt"
$stackButterflyFile = Join-Path $ProfileOutputDirectory "camera-star-tests-$timestamp-xperf-stack-butterfly.txt"
$xperf = Resolve-WindowsPerformanceToolkitTool "xperf.exe"

if ($xperf) {
    Write-Host "Expanding Visual Studio profile..."
    $expandExitCode = Invoke-LoggedCommand $vsDiagnostics @("expandDiagSession", $diagsession) $logFile
    if ($expandExitCode -ne 0) {
        Write-Warning "Visual Studio Diagnostics expandDiagSession returned exit code $expandExitCode. Skipping xperf exports."
    } else {
        $etlFile = Get-ChildItem -LiteralPath $expandedSessionDirectory -Recurse -Filter "*.etl" -ErrorAction SilentlyContinue |
            Sort-Object Length -Descending |
            Select-Object -First 1

        if ($etlFile) {
            $symbolPaths = @($testWorkingDirectory)
            $symbolPaths += $runtimePaths
            $symbolPaths = $symbolPaths |
                Where-Object { $_ -and (Test-Path -LiteralPath $_) } |
                Select-Object -Unique

            $oldSymbolPath = $env:_NT_SYMBOL_PATH
            $oldSymCachePath = $env:_NT_SYMCACHE_PATH
            $env:_NT_SYMBOL_PATH = $symbolPaths -join ";"
            $env:_NT_SYMCACHE_PATH = Join-Path $ProfileOutputDirectory "symcache"
            New-Item -ItemType Directory -Force -Path $env:_NT_SYMCACHE_PATH | Out-Null

            try {
                Write-Host "Exporting local-symbol sampled CPU profile..."
                $profileExitCode = Invoke-LoggedCommand $xperf @(
                    "-i", $etlFile.FullName,
                    "-o", $profileDetailFile,
                    "-symbols",
                    "-a", "profile",
                    "-detail"
                ) $logFile
                if ($profileExitCode -ne 0) {
                    Write-Warning "xperf sampled CPU profile export returned exit code $profileExitCode."
                }

                Write-Host "Exporting local-symbol stack butterfly profile..."
                $stackExitCode = Invoke-LoggedCommand $xperf @(
                    "-i", $etlFile.FullName,
                    "-o", $stackButterflyFile,
                    "-symbols",
                    "-a", "stack",
                    "-butterfly", "20",
                    "-process", "featurecamera_star_tests"
                ) $logFile
                if ($stackExitCode -ne 0) {
                    Write-Warning "xperf stack butterfly export returned exit code $stackExitCode."
                }
            }
            finally {
                $env:_NT_SYMBOL_PATH = $oldSymbolPath
                $env:_NT_SYMCACHE_PATH = $oldSymCachePath
            }
        } else {
            Write-Warning "No ETL file found under expanded profile directory $expandedSessionDirectory. Skipping xperf exports."
        }
    }
} else {
    Write-Warning "xperf.exe was not found. Install Windows Performance Toolkit to generate text profile exports."
}

Write-Host "Visual Studio profile written to:"
Write-Host "  $diagsession"
if (Test-Path -LiteralPath $profileDetailFile) {
    Write-Host "xperf sampled CPU profile written to:"
    Write-Host "  $profileDetailFile"
}
if (Test-Path -LiteralPath $stackButterflyFile) {
    Write-Host "xperf stack butterfly profile written to:"
    Write-Host "  $stackButterflyFile"
}
Write-Host "Log written to:"
Write-Host "  $logFile"
