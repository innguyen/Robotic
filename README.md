# Hướng Dẫn Nạp Code Lên PuppyPi

File mã nguồn: `auto_find_object.py`

## Cách 1: Copy trực tiếp bằng lệnh PowerShell (Nhanh nhất từ máy tính)
Mở PowerShell trên máy tính của bạn tại thư mục này và gõ:

```powershell
scp ./auto_find_object.py pi@<DIA_CHI_IP_ROBOT>:~/auto_find_object.py
```
*(Ví dụ IP mặc định: `scp ./auto_find_object.py pi@192.168.149.1:~/auto_find_object.py`)*
* Mật khẩu nếu được hỏi: `raspberrypi`

---

## Cách 2: Copy bằng WinSCP hoặc VNC
1. Mở **WinSCP** kết nối tới IP robot (User: `pi`, Pass: `raspberrypi`).
2. Kéo thả file `auto_find_object.py` vào thư mục `/home/pi/`.

---

## Cách Chạy Trên Robot (Qua Terminal VNC):
```bash
python3 ~/auto_find_object.py
```

* **Dừng chương trình:** Nhấn `Ctrl + C`.
* **Đổi màu vật thể:** Mở file và đổi dòng `TARGET_COLOR = 'red'` thành `'blue'`, `'green'`, hoặc `'yellow'`.
"# Robotic" 
