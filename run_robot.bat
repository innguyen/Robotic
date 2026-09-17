@echo off
echo ========================================================
echo [1/3] Dang dong bo code len Raspberry Pi...
echo ========================================================
scp d:\robotic\jetauto_find_object.py jetauto@192.168.149.1:/home/jetauto/jetauto_find_object.py
scp d:\robotic\coco.names jetauto@192.168.149.1:/home/jetauto/coco.names
scp d:\robotic\start_server.sh jetauto@192.168.149.1:/home/jetauto/start_server.sh
if exist "d:\robotic\yolov5n.onnx" (
    echo Dang dong bo model yolov5n.onnx...
    scp d:\robotic\yolov5n.onnx jetauto@192.168.149.1:/home/jetauto/yolov5n.onnx
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
ssh -t jetauto@192.168.149.1 "chmod +x /home/jetauto/start_server.sh && bash -ic '/home/jetauto/start_server.sh'"

echo.
echo Da dung chuong trinh.
