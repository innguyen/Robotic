@echo off
rem Dong bo code va chay tren Robot JetAuto
scp "%~dp0jetauto_find_object.py" "%~dp0coco.names" "%~dp0start_server.sh" jetauto@192.168.149.1:/home/jetauto/
ssh -t jetauto@192.168.149.1 "sed -i 's/\r$//' /home/jetauto/start_server.sh && chmod +x /home/jetauto/start_server.sh && bash -l /home/jetauto/start_server.sh"
