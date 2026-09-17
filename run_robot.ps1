Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "[1/2] Dang dong bo code len Robot..." -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan

# Dong bo tat ca file code cung 1 luc (CHI CAN NHAP PASSWORD 1 LAN)
scp d:\robotic\jetauto_find_object.py d:\robotic\coco.names d:\robotic\start_server.sh jetauto@192.168.149.1:/home/jetauto/

if ($LASTEXITCODE -ne 0) {
    Write-Host "[LOI] Dong bo that bai! Kiem tra lai WiFi hoac mat khau." -ForegroundColor Red
    exit
}

Write-Host "`n========================================================" -ForegroundColor Green
Write-Host "[2/2] Khoi chay he thong tren Robot..." -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Green

ssh -t jetauto@192.168.149.1 "chmod +x /home/jetauto/start_server.sh && bash -l /home/jetauto/start_server.sh"
