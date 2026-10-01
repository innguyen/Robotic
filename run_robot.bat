@echo off
rem Chay ban co ban (Dual Vision + Mecanum Strafing ne vat can)
scp "%~dp0navigation_safety.py" "%~dp0navigation_config.json" "%~dp0robot_voice.py" "%~dp0jetauto_find_object.py" "%~dp0jetauto_smart_search.py" "%~dp0coco.names" "%~dp0start_server.sh" jetauto@192.168.149.1:/home/jetauto/
if errorlevel 1 exit /b 1
ssh -t jetauto@192.168.149.1 "sed -i 's/\r$//' /home/jetauto/start_server.sh && chmod +x /home/jetauto/start_server.sh && bash -l /home/jetauto/start_server.sh jetauto_find_object.py"
