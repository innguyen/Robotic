param(
    [string]$App = "jetauto_find_object.py"
)

# Dong bo code (bao gom ca ban co ban va ban nang cao) sang Robot JetAuto
scp "$PSScriptRoot\jetauto_find_object.py" "$PSScriptRoot\jetauto_smart_search.py" "$PSScriptRoot\coco.names" "$PSScriptRoot\start_server.sh" jetauto@192.168.149.1:/home/jetauto/
ssh -t jetauto@192.168.149.1 "sed -i 's/\r$//' /home/jetauto/start_server.sh && chmod +x /home/jetauto/start_server.sh && bash -l /home/jetauto/start_server.sh $App"
