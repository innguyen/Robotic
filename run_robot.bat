@echo off
rem Upload the application, avoidance module and model, then start JetAuto.
scp -r "%~dp0jetauto_find_object.py" "%~dp0obstacle_avoidance.py" "%~dp0models" "%~dp0start_server.sh" jetauto@192.168.149.1:/home/jetauto/
if errorlevel 1 exit /b 1
ssh -t jetauto@192.168.149.1 "sed -i 's/\r$//' /home/jetauto/start_server.sh && chmod +x /home/jetauto/start_server.sh && bash -l /home/jetauto/start_server.sh"
