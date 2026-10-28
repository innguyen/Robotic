param(
    [string]$RobotAddress = 'jetauto@192.168.149.1'
)
# One-time setup: after this, run_test_robot*.bat connect without asking for the password.
$ErrorActionPreference = 'Stop'
try {
    foreach ($commandName in @('ssh', 'ssh-keygen')) {
        if (-not (Get-Command $commandName -ErrorAction SilentlyContinue)) {
            throw "Missing $commandName. Enable Windows OpenSSH Client first."
        }
    }
    $sshDir = Join-Path $HOME '.ssh'
    $keyPath = Join-Path $sshDir 'id_ed25519'
    if (-not (Test-Path -LiteralPath $sshDir)) { New-Item -ItemType Directory -Path $sshDir | Out-Null }
    if (Test-Path -LiteralPath $keyPath) {
        Write-Host "[1/2] Using existing key: $keyPath"
    } else {
        Write-Host "[1/2] Creating key: $keyPath"
        # Empty passphrase so the launcher never prompts; the key stays in your user profile.
        & ssh-keygen -q -t ed25519 -N '""' -f $keyPath
        if ($LASTEXITCODE -ne 0) { throw 'ssh-keygen failed.' }
    }
    $publicKey = (Get-Content -LiteralPath "$keyPath.pub" -Raw).Trim()
    Write-Host '[2/2] Installing key on robot. Enter the robot password ONE last time.'
    # Append only if missing, with the permissions sshd requires. sshd (StrictModes)
    # ignores keys when the home directory is group/world-writable, common on robot images.
    $remote = "chmod go-w ~ && mkdir -p ~/.ssh && chmod 700 ~/.ssh && touch ~/.ssh/authorized_keys && " +
              "chmod 600 ~/.ssh/authorized_keys && (grep -qxF '$publicKey' ~/.ssh/authorized_keys || echo '$publicKey' >> ~/.ssh/authorized_keys) && " +
              "echo '--- robot diagnostics ---' && ls -ld ~ ~/.ssh ~/.ssh/authorized_keys && " +
              "echo matching keys: `$(grep -cxF '$publicKey' ~/.ssh/authorized_keys) && " +
              "(grep -iE '^[[:space:]]*(PubkeyAuthentication|AuthorizedKeysFile|StrictModes|PasswordAuthentication)' /etc/ssh/sshd_config || echo 'sshd_config: defaults')"
    & ssh $RobotAddress $remote
    if ($LASTEXITCODE -ne 0) { throw 'Cannot install key on robot.' }
    & ssh -o BatchMode=yes -o ConnectTimeout=8 $RobotAddress 'echo ok' | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Host '--- local ssh attempt (no password needed) ---'
        & ssh -v -o BatchMode=yes -o ConnectTimeout=8 $RobotAddress 'echo ok' 2>&1 |
            Select-String -Pattern 'Offering|Server accepts|can continue|denied|identity file .*type [0-9]|UNPROTECTED|bad permissions|OpenSSH_' |
            ForEach-Object { $_.Line }
        throw 'Key installed but passwordless login still fails. Send the lines above.'
    }
    Write-Host 'Done: run_test_robot.bat will no longer ask for the password.' -ForegroundColor Green
    exit 0
} catch {
    Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
