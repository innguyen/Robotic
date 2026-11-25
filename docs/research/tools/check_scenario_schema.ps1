param([Parameter(Mandatory = $true)][string]$OutFile)

$ErrorActionPreference = 'Stop'
$s3Root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../..'))
$s3Schema = Join-Path $s3Root 'docs/research/contracts/scenario.schema.json'
$s3Example = Get-Content -LiteralPath (Join-Path $s3Root 'docs/research/contracts/scenario.example.json') -Raw
$s3Cases = [System.Collections.Generic.List[object]]::new()

function Check-Case([string]$Name, [scriptblock]$Mutate, [bool]$Expected) {
    $s3Value = $s3Example | ConvertFrom-Json -AsHashtable
    & $Mutate $s3Value
    $s3Json = $s3Value | ConvertTo-Json -Depth 30
    $s3Actual = $s3Json | Test-Json -SchemaFile $s3Schema -ErrorAction SilentlyContinue
    $s3Cases.Add([ordered]@{name = $Name; expected = $Expected; actual = [bool]$s3Actual; passed = ([bool]$s3Actual -eq $Expected)})
}

Check-Case 'valid_design' { param($s) } $true
Check-Case 'unknown_top_level' { param($s) $s['undeclared'] = 1 } $false
Check-Case 'private_field_in_public_profile' { param($s) $s.public_profile['victims'] = @() } $false
Check-Case 'too_many_victims' { param($s) $s.private_scenario.victims = @($s.private_scenario.victims[0]) * 4 } $false
Check-Case 'unsupported_version' { param($s) $s.schema_version = 'future' } $false
Check-Case 'unsafe_identifier' { param($s) $s.episode_id = '../escape' } $false
Check-Case 'empirical_without_calibration' { param($s) $s.public_profile.sensor_model_provenance = 'empirical' } $false
Check-Case 'runtime_without_freeze' { param($s) $s.design_only = $false } $false
Check-Case 'invalid_condition_level' { param($s) $s.private_scenario.condition.odom_severity = 3 } $false

$s3Failed = @($s3Cases | Where-Object { -not $_.passed })
$s3Result = [ordered]@{
    validator = 'PowerShell Test-Json'; powershell_version = $PSVersionTable.PSVersion.ToString()
    scope = 'Structural JSON Schema validation and negative mutations; no simulation'
    status = $(if ($s3Failed.Count -eq 0) { 'PASS' } else { 'FAIL' })
    cases = $s3Cases.ToArray()
}
$s3Dest = [IO.Path]::GetFullPath((Join-Path $s3Root $OutFile))
if (-not $s3Dest.StartsWith($s3Root + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Evidence output must remain inside the repository'
}
$s3File = [IO.File]::Open($s3Dest, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write)
$s3Writer = [IO.StreamWriter]::new($s3File, [Text.UTF8Encoding]::new($false))
try { $s3Writer.WriteLine(($s3Result | ConvertTo-Json -Depth 30)) }
finally { $s3Writer.Dispose() }
Write-Output ("{0}: {1}/{2} schema cases" -f $s3Result.status, ($s3Cases.Count - $s3Failed.Count), $s3Cases.Count)
if ($s3Failed.Count -ne 0) { exit 1 }
