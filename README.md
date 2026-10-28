# JetAuto — Find Object

Chương trình chính là `jetauto_find_object.py`: nhận diện mục tiêu bằng YOLO/Astra, né vật cản bằng LiDAR và độ sâu Astra, điều khiển bánh Mecanum và phục vụ dashboard web.

## Cấu trúc

```text
robotic/
├── jetauto_find_object.py   # Chương trình chính và giao diện web
├── obstacle_avoidance.py    # Xử lý vật cản, bản đồ cục bộ và lập lệnh di chuyển
├── models/
│   ├── yolov5n.onnx         # Model được chương trình sử dụng
│   └── coco.names           # Nhãn nhận diện COCO
├── run_robot.bat            # Upload và khởi động từ Windows
├── run_robot.ps1            # Cách chạy tương đương bằng PowerShell
├── start_server.sh          # Kiểm tra/khởi động ROS, cảm biến và chương trình
└── README.md
```

## Chạy từ Windows

Chạy `run_robot.bat`, hoặc chạy `./run_robot.ps1` trong PowerShell nếu máy cho phép chạy script. Cần có `ssh` và `scp`.

Script chép hai file Python, thư mục `models/` và `start_server.sh` vào `/home/jetauto/` trên robot `jetauto@192.168.149.1`, sau đó khởi động chương trình. Mở `http://192.168.149.1:5000`. Nếu cổng đang bận, chương trình thử các cổng tiếp theo và in địa chỉ thực tế trong log.

Robot phải có sẵn ROS, các driver khung gầm/LiDAR/Astra, Python 3, NumPy, OpenCV có `cv2.dnn` và các ROS Python package mà chương trình import. Chương trình ưu tiên ONNX Runtime nếu có; nếu không dùng được thì chuyển sang OpenCV DNN. Hai script upload không cài các thư viện này.

## Chạy trực tiếp trên robot

Sau khi upload, trong môi trường JetAuto:

```bash
bash -l /home/jetauto/start_server.sh
```

Chỉ kiểm tra driver và dữ liệu cảm biến:

```bash
bash -l /home/jetauto/start_server.sh --check-only
```

Chạy chương trình và trả terminal về sau khi khởi động:

```bash
bash -l /home/jetauto/start_server.sh --no-tail
```

Log chương trình: `/tmp/jetauto_ai.log`. Script khởi động đưa chương trình về trạng thái tạm dừng; bấm **Tiếp tục** trên dashboard để cho phép di chuyển.

## Giao diện và cấu hình

HTML/CSS/JavaScript nằm trong `CamHandler._render_html()` của `jetauto_find_object.py`. Không cần thư mục template hoặc file HTML bên ngoài.

Dashboard có camera Astra, radar LiDAR/Astra, chọn mục tiêu, tạm dừng/tiếp tục, dừng khẩn, chạy khô, hiệu chuẩn sàn, test Mecanum và lái tay. Giữ nút hoặc dùng W/S để tiến/lùi, A/D để xoay, Q/E để trượt ngang; Space dừng lệnh lái tay.

Các thông số thân xe, cảm biến, tốc độ và khoảng dừng nằm trong `AvoidanceConfig` của `obstacle_avoidance.py`. Chế độ chạy khô bật/tắt bằng nút trên dashboard; chương trình vẫn tính toán nhưng gửi vận tốc 0 tới động cơ. OAK-D mặc định tắt, khung radar sử dụng dữ liệu LiDAR và Astra.
