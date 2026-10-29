param(
  [ValidateSet('unit','startup','navigation','m2','m3','faults','obstacles','preflight','experiments','all')]
  [string[]]$Stages = @('m3','navigation','preflight','experiments'),
  [string]$Distro = 'Ubuntu-24.04',
  [ValidateSet('rmw_fastrtps_cpp','rmw_cyclonedds_cpp')]
  [string]$Middleware = 'rmw_cyclonedds_cpp',
  [double]$RealTimeFactor = 1,
  [string]$OutputDirectory = 'artifacts/acceptance_final',
  [string]$EvidenceName = 'final_acceptance'
)
$ErrorActionPreference = 'Stop'
$projectPath = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$wslProjectPath = (& wsl.exe -d $Distro -- wslpath -a $projectPath | Select-Object -Last 1).Trim()
if ($LASTEXITCODE -ne 0 -or -not $wslProjectPath.StartsWith('/')) { throw 'Cannot resolve WSL project path' }
$linuxUserHome = (& wsl.exe -d $Distro -- printenv HOME | Select-Object -Last 1).Trim()
if ($LASTEXITCODE -ne 0 -or -not $linuxUserHome.StartsWith('/home/')) { throw 'Cannot resolve Linux home' }
$linuxWorkspace = "$linuxUserHome/industrial_amr_ws"
$linuxOutput = if ($OutputDirectory.StartsWith('/')) { $OutputDirectory } else { "$linuxWorkspace/$OutputDirectory" }
$logPath = Join-Path $projectPath ('artifacts/runner_logs/' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssZ'))
[IO.Directory]::CreateDirectory($logPath) | Out-Null
[ordered]@{ middleware=$Middleware; target_real_time_factor=$RealTimeFactor; stages=$Stages; output=$linuxOutput } |
  ConvertTo-Json -Depth 3 | Set-Content -LiteralPath (Join-Path $logPath 'runtime_selection.json') -Encoding utf8
if (-not ('AmrValidation.PowerRequest' -as [type])) {
  Add-Type -TypeDefinition @'
using System.Runtime.InteropServices;
namespace AmrValidation {
  public static class PowerRequest {
    [DllImport("kernel32.dll")]
    public static extern uint SetThreadExecutionState(uint flags);
  }
}
'@
}
# A thread-scoped idle-sleep request; never changes the power plan or display.
$powerResult = [AmrValidation.PowerRequest]::SetThreadExecutionState([uint32]2147483649)
if ($powerResult -eq 0) { throw 'Cannot acquire temporary idle-sleep prevention' }
try {
  foreach ($stage in $Stages) {
    Write-Host "START $stage; log: $logPath/$stage.log"
    & wsl.exe -d $Distro -- env "RMW_IMPLEMENTATION=$Middleware" "AMR_REAL_TIME_FACTOR=$RealTimeFactor" "AMR_VALIDATION_DIR=$OutputDirectory" bash "$wslProjectPath/tools/validate.sh" $stage 2>&1 |
      Tee-Object -FilePath (Join-Path $logPath "$stage.log")
    if ($LASTEXITCODE -ne 0) { throw "Stage $stage failed; retained logs at $logPath" }
  }
  if ('experiments' -in $Stages -or 'all' -in $Stages) {
    & wsl.exe -d $Distro -- python3 "$wslProjectPath/tools/plot_experiment.py" --input "$linuxOutput/m4" --output "$linuxOutput/m4_paired"
    if ($LASTEXITCODE -ne 0) { throw 'Experiment plot failed' }
  }
  & wsl.exe -d $Distro -- mkdir -p "$linuxOutput/runner_logs"
  if ($LASTEXITCODE -ne 0) { throw 'Cannot create runner log directory' }
  $wslLogPath = "$wslProjectPath/artifacts/runner_logs/$([IO.Path]::GetFileName($logPath))/."
  & wsl.exe -d $Distro -- cp -a $wslLogPath "$linuxOutput/runner_logs/"
  if ($LASTEXITCODE -ne 0) { throw 'Cannot preserve runner logs' }
  & wsl.exe -d $Distro -- python3 "$wslProjectPath/tools/collect_evidence.py" --source $linuxOutput --name $EvidenceName
  if ($LASTEXITCODE -ne 0) { throw 'Evidence collection failed' }
  Write-Host "DONE; evidence: artifacts/validation/$EvidenceName"
} finally {
  [AmrValidation.PowerRequest]::SetThreadExecutionState([uint32]2147483648) | Out-Null
  Write-Host 'Released temporary idle-sleep request.'
}
