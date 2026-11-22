param([string]$Pilot = 'docs/p22-final-audit-logs/pilot')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$pilotRoot = Join-Path $projectRoot $Pilot
$sampleFiles = Get-ChildItem -LiteralPath (Join-Path $pilotRoot 'paired/host') -Filter samples.jsonl -Recurse
$hostTimes = foreach ($sampleFile in $sampleFiles) {
    foreach ($sampleLine in Get-Content -LiteralPath $sampleFile.FullName) {
        ($sampleLine | ConvertFrom-Json).clock.utc_s
    }
}
if (-not $hostTimes) { throw 'No host samples; cannot certify the power-event window' }
$limits = $hostTimes | Measure-Object -Minimum -Maximum
$windowStart = [DateTimeOffset]::FromUnixTimeMilliseconds([long]($limits.Minimum * 1000)).AddSeconds(-1)
$windowEnd = [DateTimeOffset]::FromUnixTimeMilliseconds([long]($limits.Maximum * 1000)).AddSeconds(1)
$queryStatus = 'PASS'
$queryError = $null
$events = @()
try {
    $events = @(Get-WinEvent -FilterHashtable @{
        LogName='System'; ProviderName='Microsoft-Windows-Kernel-Power'; Id=42,107,506,507
        StartTime=$windowStart.LocalDateTime; EndTime=$windowEnd.LocalDateTime
    } | Select-Object TimeCreated,Id,@{Name='Message';Expression={($_.Message -split '\r?\n')[0]}})
} catch {
    if ($_.FullyQualifiedErrorId -notlike 'NoMatchingEventsFound*') {
        $queryStatus = 'UNVERIFIED'; $queryError = $_.FullyQualifiedErrorId
    }
}
$evidence = [ordered]@{
    version='kernel-power-window-1'; query_status=$queryStatus; query_error=$queryError
    provider='Microsoft-Windows-Kernel-Power'; event_ids=@(42,107,506,507)
    window_start_utc_s=$windowStart.ToUnixTimeMilliseconds()/1000
    window_end_utc_s=$windowEnd.ToUnixTimeMilliseconds()/1000
    sample_files=$sampleFiles.Count; events=$events; power_settings_changed=$false
    limitation='Retrospective event-log query, bounded host sampling; not a complete scheduler trace.'
}
$evidence | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $pilotRoot 'host-power-events.json') -Encoding utf8
Write-Output "query_status=$queryStatus sample_files=$($sampleFiles.Count) power_events=$($events.Count)"
if ($queryStatus -ne 'PASS' -or $events.Count -ne 0) { exit 1 }
