param(
    [string]$App = "jetauto_find_object.py"
)

# Đồng bộ code lên JetAuto. Một lần SCP để chuyển file, một lần SSH để khởi động.
if ($App -eq 'jetauto_smart_search.py') {
    $UploadSources = @(
        "$PSScriptRoot\navigation_safety.py",
        "$PSScriptRoot\navigation_config.json",
        "$PSScriptRoot\robot_voice.py",
        "$PSScriptRoot\jetauto_find_object.py",
        "$PSScriptRoot\jetauto_smart_search.py",
        "$PSScriptRoot\coco.names",
        "$PSScriptRoot\start_server.sh",
        "$PSScriptRoot\templates\smart_dashboard.html"
    )
} else {
    $UploadSources = @(
        "$PSScriptRoot\navigation_safety.py",
        "$PSScriptRoot\navigation_config.json",
        "$PSScriptRoot\robot_voice.py",
        "$PSScriptRoot\jetauto_find_object.py",
        "$PSScriptRoot\jetauto_smart_search.py",
        "$PSScriptRoot\coco.names",
        "$PSScriptRoot\start_server.sh"
    )
}
scp @UploadSources jetauto@192.168.149.1:/home/jetauto/
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
if ($App -eq 'jetauto_smart_search.py') {
    $RemoteSetup = 'mkdir -p /home/jetauto/templates && mv -f /home/jetauto/smart_dashboard.html /home/jetauto/templates/smart_dashboard.html && '
} else {
    $RemoteSetup = ''
}
$RemoteCommand = $RemoteSetup + "sed -i 's/\r$//' /home/jetauto/start_server.sh && chmod +x /home/jetauto/start_server.sh && bash -l /home/jetauto/start_server.sh $App"
ssh -t jetauto@192.168.149.1 $RemoteCommand
