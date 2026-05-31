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

$launchArgs = "`"$Csv`" `"$OutputDirectory`""
$startArgs = @(
    "start",
    $sessionId,
    "/launch:$Executable",
    "/launchArgs:$launchArgs",
    "/loadConfig:$agentConfig",
    "/monitor",
    "/scratchLocation:$scratchDirectory",
    "/package:opt"
)

$startExitCode = Invoke-LoggedCommand $vsDiagnostics $startArgs $logFile
if ($startExitCode -ne 0) {
    Write-Warning "Visual Studio Diagnostics start returned exit code $startExitCode. Attempting to stop/package the session anyway."
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
        throw "Visual Studio Diagnostics did not create a .diagsession because a DiagnosticsHub COM component is not registered. Try repairing Visual Studio Diagnostics Tools, or run this target from a full Visual Studio Developer Command Prompt. See $logFile"
    }
    throw "Visual Studio Diagnostics completed but did not create $diagsession. See $logFile"
}

Write-Host "Visual Studio profile written to:"
Write-Host "  $diagsession"
Write-Host "Log written to:"
Write-Host "  $logFile"
