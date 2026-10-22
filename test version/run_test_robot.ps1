param(
    [ValidateSet('preview', 'live')]
    [string]$Mode = 'preview',
    [ValidatePattern('^[a-z][a-z ]{0,39}$')]
    [string]$Target = 'bottle',
    [ValidateRange(1024, 65535)]
    [int]$Port = 5010
)
$ErrorActionPreference = 'Stop'
$robotAddress = 'jetauto@192.168.149.1'
$remoteFolder = '/home/jetauto/test_version'
$repoFolder = Split-Path -Parent $PSScriptRoot

try {
    foreach ($commandName in @('ssh', 'scp')) {
        if (-not (Get-Command $commandName -ErrorAction SilentlyContinue)) {
            throw "Missing $commandName. Enable Windows OpenSSH Client first."
        }
    }
    $payload = @(
        'jetauto_find_object.py', 'collision_monitor_ros1.py',
        'navigation_core_ros1.py', 'navigation_ros1.py', 'navigation_status.js',
        'collision_monitor_ros1.yaml', 'navigation_ros1.yaml',
        'start_navigation_ros1.sh', 'start_all_ros1.sh', 'launcher_lock_ros1.py'
    ) | ForEach-Object { Join-Path $PSScriptRoot $_ }
    $payload += Join-Path $repoFolder 'start_server.sh'
    $payload += Join-Path $repoFolder 'models'
    foreach ($payloadPath in $payload) {
        if (-not (Test-Path -LiteralPath $payloadPath)) { throw "Missing: $payloadPath" }
    }
    Write-Host "[1/3] Preparing robot ($Mode)..."
    & ssh $robotAddress "mkdir -p $remoteFolder"
    if ($LASTEXITCODE -ne 0) { throw 'Cannot connect to robot or create test directory.' }
    Write-Host '[2/3] Uploading test code and models...'
    & scp -r @payload "${robotAddress}:$remoteFolder/"
    if ($LASTEXITCODE -ne 0) { throw 'Upload failed.' }
    Write-Host "[3/3] Starting stack. Camera: http://192.168.149.1:$Port"
    Write-Host 'Keep this window open. Ctrl+C stops the test stack.'
    $remoteCommand = "sed -i 's/\r$//' $remoteFolder/start_all_ros1.sh $remoteFolder/start_navigation_ros1.sh $remoteFolder/start_server.sh && bash -l $remoteFolder/start_all_ros1.sh $Mode '$Target' $Port"
    & ssh -t -o ServerAliveInterval=10 -o ServerAliveCountMax=3 $robotAddress $remoteCommand
    exit $LASTEXITCODE
} catch {
    Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
