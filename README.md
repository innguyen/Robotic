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

## JetAuto Smart Search

Chạy `run_smart_search.bat` để chép chương trình và dashboard lên JetAuto. Mở `http://192.168.149.1:5005`, chọn lớp vật thể COCO (ví dụ `person`), rồi bấm **Tìm kiếm**. Robot khởi động ở trạng thái tạm dừng.

Script hỏi mật khẩu JetAuto hai lần: một lần để chép các tệp bằng SCP, một lần để đăng nhập và khởi động server.

Khi chưa thấy mục tiêu, robot quay một vòng để camera quan sát. Sau đó nó chọn một rìa giữa vùng đã quét và vùng chưa biết trên bản đồ LiDAR, lập đường đi qua ô trống có khoảng cách an toàn và chạy chậm tới đó. Đến nơi, robot quét lại. Nếu không có lối đi đã được xác nhận, hoặc mất dữ liệu LiDAR/odometry/Astra, robot dừng và hiển thị lý do trên dashboard. Khi thấy mục tiêu, chế độ bám mục tiêu được ưu tiên; khi mất dấu, robot thử tới vị trí cuối trong 12 giây rồi quay lại khám phá.

Nếu LiDAR chỉ có góc nhìn khoảng 150°, robot xoay chậm để quét các hướng khác; dashboard hiển thị góc quét thực tế từ `/scan`. LiDAR không nhìn được phần thân xe hoặc vật quá thấp, nên vùng mù phía sau lúc bắt đầu xoay vẫn phải được người vận hành kiểm tra. Nếu có vật trong vùng LiDAR nhìn thấy gần hơn 40 cm, hoặc `/scan` không có bản quét mới, robot sẽ không xoay tự động.

`ROBOT_CLEARANCE_M` trong `jetauto_smart_search.py` là khoảng cách từ tâm tới mép thân xe cộng biên an toàn; hãy đo và chỉnh theo robot thực tế, nhất là khi có tay gắp. Kiểm tra hướng 0° của LiDAR có trùng hướng tiến của robot; nếu lệch, chỉnh `LIDAR_YAW_OFFSET_RAD`. Bản đồ hiện chỉ tích lũy LiDAR theo odometry trong vùng 12 × 12 m, chưa phải SLAM có hiệu chỉnh sai số. Lần chạy đầu nên dùng phòng nhỏ, không có bậc thang hoặc vật cản thấp hơn mặt phẳng quét LiDAR.

OAK-D hiện mặc định chỉ phát ảnh phụ (`USE_OAKD_FOR_AVOIDANCE = False`), vì góc lắp thấp khiến mặt sàn bị báo là vật cản gần. Robot dùng LiDAR để chặn lệnh di chuyển. Trên dashboard, `Cam FPS` là tốc độ nhận ảnh Astra, còn `AI FPS` là số lần YOLO xử lý mỗi giây; model `yolov5n.onnx` trong dự án có đầu vào cố định 640 × 640 nên giảm kích thước đầu vào trực tiếp sẽ làm lỗi model. Sau khi chép code, server khởi động ở trạng thái tạm dừng; chỉ bấm Tìm kiếm khi vùng quanh robot đã được kiểm tra.
