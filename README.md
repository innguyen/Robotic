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

Không dùng nhận diện khuôn mặt. Khi đang Tìm kiếm, YOLO thấy đúng lớp mục tiêu trong hai lần xử lý liên tiếp sẽ thông báo ngay qua loa USB GeneralPlus, không chờ tiến gần. Ví dụ `person`: “Đã tìm thấy người. Xin chào bạn”; `bottle`: “Đã tìm thấy bottle”. Chỉ báo mục tiêu được chọn, không đọc mọi vật trong ảnh. Không báo lặp khi vẫn thấy mục tiêu; mất dấu hơn 10 giây hoặc đổi lớp tìm kiếm sẽ cho phép báo lại. Tạm dừng và lái thủ công không phát thông báo mới. Mục Lời chào hiển thị nội dung hoặc lỗi loa. Không cần chuyển ảnh hay mô hình khuôn mặt; chỉ chạy `run_smart_search.bat`. Cần `espeak` và `pulseaudio-utils` trên robot.

Dashboard hiển thị pin từ `/ros_robot_controller/battery` (`std_msgs/UInt16`, millivolt). Phần trăm là **ước tính tuyến tính** từ 10,0 V đến 12,6 V, không phải số đo dung lượng chính xác; tải động cơ và sụt áp làm giá trị dao động. Nếu không có dữ liệu mới trong 5 giây, dashboard hiện “Chưa có dữ liệu”. Robot dùng pin LiPo 11,1 V; Hiwonder khuyến nghị sạc khi điện áp thấp hơn 10 V.


Khi chưa thấy mục tiêu, robot quay một vòng để camera quan sát rồi khám phá vùng chưa biết có đường đi an toàn. Khi mất dấu, robot ngừng tiến và quét về hướng vị trí cũ trong tối đa 1,5 giây, rồi trở lại khám phá. LiDAR, odometry và camera vẫn có quyền chặn chuyển động nếu dữ liệu thiếu hoặc không có khoảng trống an toàn.

Quét bị kẹt sẽ thử đổi chiều có giới hạn, sau đó lập đường trong vùng đã quan sát. Bộ kiểm tra thân xe có quyền chặn cả quay khi LiDAR không quan sát đủ vùng thân sẽ quét qua; không coi góc quét thiếu là khoảng trống. Bản sửa và hướng dẫn kiểm thử nằm trong [LIDAR_NAVIGATION_FIXES.md](LIDAR_NAVIGATION_FIXES.md).

Kích thước thân, vị trí LiDAR và camera đặt trong `navigation_config.json`. Bản đồ ghép từng tia theo timestamp odometry và giữ bằng chứng có tuổi thọ, nhưng chưa phải SLAM hiệu chỉnh trôi. Kiểm tra khoảng trống cao 60 cm bằng Astra đã có mã, mặc định chưa bật vì chưa có kích thước thân và hiệu chỉnh camera. Khi bật, phiên bản này chỉ xác nhận thể tích cho đi thẳng phía trước; không tự cấp quyền đi vào vùng mù. Xem tài liệu trước khi cấu hình và thử trên phần cứng.

OAK-D hiện mặc định chỉ phát ảnh phụ (`USE_OAKD_FOR_AVOIDANCE = False`), vì góc lắp thấp khiến mặt sàn bị báo là vật cản gần. Robot dùng LiDAR để chặn lệnh di chuyển. Trên dashboard, `Cam FPS` là tốc độ nhận ảnh Astra, còn `AI FPS` là số lần YOLO xử lý mỗi giây; model `yolov5n.onnx` trong dự án có đầu vào cố định 640 × 640 nên giảm kích thước đầu vào trực tiếp sẽ làm lỗi model. Sau khi chép code, server khởi động ở trạng thái tạm dừng; chỉ bấm Tìm kiếm khi vùng quanh robot đã được kiểm tra.


Smart Search: OAK-D is now the primary YOLO camera; Astra is secondary.
Camera FPS and freshness checks follow OAK-D. Streams: `/cam_primary.mjpg`
and `/cam_secondary.mjpg`. Old URLs remain aliases for their stream slots.
Target distance uses the bounding-box estimate, never Astra depth with OAK-D
boxes. LiDAR still gates motion. Run `run_smart_search.bat` to upload/restart.
