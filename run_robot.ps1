Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "[1/2] Dang dong bo code len Raspberry Pi..." -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan

scp d:\robotic\app_yolo_server_v2.py pi@192.168.149.1:/home/pi/app_yolo_server_v2.py
scp -r d:\robotic\templates pi@192.168.149.1:/home/pi/
scp d:\robotic\start_server.sh pi@192.168.149.1:/home/pi/start_server.sh
if (Test-Path "d:\robotic\yolo11n.onnx") {
    Write-Host "Dang dong bo model yolo11n.onnx (10MB)..." -ForegroundColor Yellow
    scp d:\robotic\yolo11n.onnx pi@192.168.149.1:/home/pi/yolo11n.onnx
}
if ($LASTEXITCODE -ne 0) {
    Write-Host "[LOI] Khong the ket noi toi Robot! Kiem tra lai WiFi robot." -ForegroundColor Red
    exit
}

Write-Host "`n========================================================" -ForegroundColor Green
Write-Host "[2/2] Khoi chay AI Server tren Robot..." -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Green

ssh -t pi@192.168.149.1 "bash /home/pi/start_server.sh"
