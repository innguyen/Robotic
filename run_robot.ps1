Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "[1/2] Dang kiem tra ket noi toi Robot (192.168.149.1)..." -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan

# Kiem tra xem may tinh da ping duoc toi Robot chua (doi toi da 8 giay cho DHCP cap IP)
$pingOk = $false
for ($i = 1; $i -le 8; $i++) {
    $test = Test-Connection -ComputerName 192.168.149.1 -Count 1 -Quiet -ErrorAction SilentlyContinue
    if ($test) {
        $pingOk = $true
        break
    }
    Write-Host "   Dang doi may tinh nhan IP tu Robot (lan $i/8)..." -ForegroundColor Yellow
    Start-Sleep -Seconds 1
}

if (-not $pingOk) {
    Write-Host "`n[CHUA KET NOI DUOC VOI ROBOT!]" -ForegroundColor Red
    $currentWifi = (netsh wlan show interfaces | Select-String "^\s+SSID\s+:" | ForEach-Object { ($_ -split ":")[1].Trim() })
    $myIP = (Get-NetIPAddress -InterfaceAlias "*Wi-Fi*" -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object -ExpandProperty IPAddress)
    Write-Host " - Mang WiFi hien tai: $currentWifi" -ForegroundColor Yellow
    Write-Host " - Dia chi IP may tinh: $myIP" -ForegroundColor Yellow
    Write-Host "`nHUONG DAN KHAC PHUC:" -ForegroundColor Cyan
    Write-Host " 1. Mo danh sach WiFi tren Windows -> Chon 'Disconnect' o mang truong (UIT Public)." -ForegroundColor White
    Write-Host " 2. Chon ket noi vao 'HW-66151611' (Mat khau neu hoi: hiwonder)." -ForegroundColor White
    Write-Host " 3. Neu Windows hien thong bao 'No Internet, stay connected?' -> Chon YES." -ForegroundColor White
    Write-Host " 4. Neu van bi, hay TAT WiFi tren may tinh 5 giay roi BAT LAI de nhan IP moi." -ForegroundColor White
    exit
}

Write-Host ">> Da ket noi thanh cong voi Robot (192.168.149.1)!" -ForegroundColor Green
Write-Host "Mat khau Robot (khi duoc hoi): jetauto" -ForegroundColor Green
Write-Host "--------------------------------------------------------"

# Dong bo tat ca file code cung 1 luc
scp -o ConnectTimeout=10 "$PSScriptRoot\jetauto_find_object.py" "$PSScriptRoot\coco.names" "$PSScriptRoot\start_server.sh" "$PSScriptRoot\oakd_second_camera.py" jetauto@192.168.149.1:/home/jetauto/

if ($LASTEXITCODE -ne 0) {
    Write-Host "`n[LOI] Dong bo that bai! Kiem tra mat khau (jetauto)." -ForegroundColor Red
    exit
}

Write-Host "`n========================================================" -ForegroundColor Green
Write-Host "[2/2] Khoi chay he thong tren Robot..." -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Green

ssh -t jetauto@192.168.149.1 "sed -i 's/\r$//' /home/jetauto/start_server.sh; chmod +x /home/jetauto/start_server.sh /home/jetauto/oakd_second_camera.py; bash -l /home/jetauto/start_server.sh"
