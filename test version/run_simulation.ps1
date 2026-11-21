param([Parameter(ValueFromRemainingArguments=$true)][string[]]$SimulationArgs)
$ErrorActionPreference = 'Stop'
$pythonCandidates = @()
$pathPython = Get-Command python -ErrorAction SilentlyContinue
if ($pathPython) { $pythonCandidates += $pathPython.Source }
$installRoot = Join-Path $env:LOCALAPPDATA 'Programs\Python'
if (Test-Path -LiteralPath $installRoot) {
    $pythonCandidates += Get-ChildItem -LiteralPath $installRoot -Filter python.exe -Recurse -File |
        Sort-Object FullName -Descending | Select-Object -ExpandProperty FullName
}
foreach ($pythonCandidate in ($pythonCandidates | Select-Object -Unique)) {
    & $pythonCandidate -c 'import numpy, tkinter, yaml' 2>$null
    if ($LASTEXITCODE -eq 0) {
        Push-Location $PSScriptRoot
        try {
            & $pythonCandidate (Join-Path $PSScriptRoot 'simulate_navigation.py') @SimulationArgs
            exit $LASTEXITCODE
        } finally { Pop-Location }
    }
}
Write-Host 'Python with numpy, tkinter and PyYAML was not found.' -ForegroundColor Red
Write-Host 'Install dependencies in your Python: python -m pip install numpy pyyaml'
exit 1
