@echo off
echo ========================================================
echo [1/3] Dang dong bo code len Raspberry Pi...
echo ========================================================
scp d:\robotic\app_yolo_server_v2.py pi@192.168.149.1:/home/pi/app_yolo_server_v2.py
scp -r d:\robotic\templates pi@192.168.149.1:/home/pi/
scp d:\robotic\start_server.sh pi@192.168.149.1:/home/pi/start_server.sh
if exist "d:\robotic\yolo11n.onnx" (
    echo Dang dong bo model yolo11n.onnx...
    scp d:\robotic\yolo11n.onnx pi@192.168.149.1:/home/pi/yolo11n.onnx
)
if %ERRORLEVEL% NEQ 0 (
    echo [LOI] Khong the ket noi toi Robot! Kiem tra lai WiFi.
    pause
    exit /b
)

echo.
echo ========================================================
echo [2/2] Khoi chay AI Server tren Robot...
echo ========================================================
ssh -t pi@192.168.149.1 "bash /home/pi/start_server.sh"

echo.
echo Da dung chuong trinh.
