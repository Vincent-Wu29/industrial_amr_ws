param([Parameter(Mandatory=$true)][string]$Log)
if (-not (Test-Path -LiteralPath $Log)) {
  @{status='not_started'} | ConvertTo-Json -Compress
  return
}
$records=@(); $running=''
foreach ($line in Get-Content -LiteralPath $Log) {
  if ($line.StartsWith('RUN ')) { $running=($line -split ' -> ')[0] }
  if ($line.StartsWith('{"scenario"')) {
    try { $records+=($line | ConvertFrom-Json -ErrorAction Stop) } catch { }
  }
}
$lastRecord=$records | Select-Object -Last 1
[ordered]@{
  recorded=$records.Count
  passed=@($records | Where-Object passed).Count
  last_dispatch=$running
  latest=$lastRecord | Select-Object scenario,seed,passed,collision_count,elapsed_sim_s
} | ConvertTo-Json -Depth 4 -Compress
