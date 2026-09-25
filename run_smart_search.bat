@echo off
rem Chay ban nang cao Smart Search (Ban do 2D LiDAR + Bo nho muc tieu)
scp "%~dp0jetauto_find_object.py" "%~dp0jetauto_smart_search.py" "%~dp0coco.names" "%~dp0start_server.sh" "%~dp0templates\smart_dashboard.html" jetauto@192.168.149.1:/home/jetauto/
if errorlevel 1 exit /b 1
ssh -t jetauto@192.168.149.1 "mkdir -p /home/jetauto/templates && mv /home/jetauto/smart_dashboard.html /home/jetauto/templates/smart_dashboard.html && sed -i 's/\r$//' /home/jetauto/start_server.sh && chmod +x /home/jetauto/start_server.sh && bash -l /home/jetauto/start_server.sh jetauto_smart_search.py"
