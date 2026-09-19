# Dong bo code va chay thang tren Robot JetAuto
scp "$PSScriptRoot\jetauto_find_object.py" "$PSScriptRoot\coco.names" "$PSScriptRoot\start_server.sh" jetauto@192.168.149.1:/home/jetauto/
ssh -t jetauto@192.168.149.1 "sed -i 's/\r$//' /home/jetauto/start_server.sh && chmod +x /home/jetauto/start_server.sh && bash -l /home/jetauto/start_server.sh"
