param(
    [ValidateSet('preview', 'live')]
    [string]$Mode = 'preview',
    [ValidatePattern('^[a-z][a-z ]{0,39}$')]
    [string]$Target = 'bottle',
    [ValidateRange(1024, 65535)]
    [int]$Port = 5010,
    [ValidateSet('local', 'mapping', 'localization')]
    [string]$MapMode = 'local',
    [ValidatePattern('^$|^/[a-zA-Z0-9_./-]+[.]yaml$')]
    [string]$RemoteMap = '',
    [switch]$NoRecord
)
$ErrorActionPreference = 'Stop'
$robotAddress = 'jetauto@192.168.149.1'
$remoteFolder = '/home/jetauto/test_version'
$repoFolder = Split-Path -Parent $PSScriptRoot

try {
    if ($MapMode -eq 'localization' -and -not $RemoteMap) { throw 'Localization requires -RemoteMap /absolute/path/map.yaml' }
    foreach ($commandName in @('ssh', 'scp')) {
        if (-not (Get-Command $commandName -ErrorAction SilentlyContinue)) {
            throw "Missing $commandName. Enable Windows OpenSSH Client first."
        }
    }
    $payload = @(
        'jetauto_find_object.py', 'collision_monitor_ros1.py',
        'navigation_core_ros1.py', 'navigation_ros1.py', 'search_core_ros1.py', 'navigation_status.js',
        'collision_monitor_ros1.yaml', 'navigation_ros1.yaml',
        'start_navigation_ros1.sh', 'start_all_ros1.sh', 'launcher_lock_ros1.py',
        'stop_test_stack_ros1.sh',
        'map_core_ros1.py', 'map_frames_ros1.py', 'mapping_scan_ros1.py',
        'mapping_ros1.launch', 'start_mapping_ros1.sh', 'session_recorder_ros1.py',
        'replay_navigation.py', 'simulate_navigation.py'
    ) | ForEach-Object { Join-Path $PSScriptRoot $_ }
    $payload += Join-Path $repoFolder 'start_server.sh'
    $payload += Join-Path $repoFolder 'models'
    foreach ($payloadPath in $payload) {
        if (-not (Test-Path -LiteralPath $payloadPath)) { throw "Missing: $payloadPath" }
    }
    Write-Host "[1/3] Preparing robot ($Mode)..."
    & ssh -o ConnectTimeout=10 $robotAddress "mkdir -p $remoteFolder"
    if ($LASTEXITCODE -ne 0) { throw 'Cannot connect to robot or create test directory.' }
    Write-Host '[2/3] Uploading test code and models...'
    & scp -o ConnectTimeout=10 -r @payload "${robotAddress}:$remoteFolder/"
    if ($LASTEXITCODE -ne 0) { throw 'Upload failed.' }
    Write-Host "[3/3] Starting stack. Camera: http://192.168.149.1:$Port"
    Write-Host 'Keep this window open. Ctrl+C stops the test stack.'
    $recordOption = if ($NoRecord) { 'false' } else { 'true' }
    $remoteCommand = "sed -i 's/\r$//' $remoteFolder/start_all_ros1.sh $remoteFolder/start_navigation_ros1.sh $remoteFolder/start_server.sh $remoteFolder/stop_test_stack_ros1.sh $remoteFolder/start_mapping_ros1.sh && bash -l $remoteFolder/start_all_ros1.sh $Mode '$Target' $Port $MapMode '$RemoteMap' $recordOption"
    $stackExit = 1
    try {
        & ssh -t -o ConnectTimeout=10 -o ServerAliveInterval=10 -o ServerAliveCountMax=3 $robotAddress $remoteCommand
        $stackExit = $LASTEXITCODE
    } finally {
        # Windows Ctrl+C often kills only the local ssh; the robot stack would keep
        # running and block the next start. Stop it explicitly (drivers stay up).
        Write-Host 'Stopping test stack on robot (sensor drivers stay running)...'
        & ssh -o ConnectTimeout=10 $robotAddress "bash $remoteFolder/stop_test_stack_ros1.sh"
    }
    exit $stackExit
} catch {
    Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
