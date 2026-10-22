# Mô phỏng trên máy tính

Chạy `run_simulation.bat` trong thư mục này. Cần Python có tkinter và NumPy
(`python -m pip install numpy`). Không cần ROS, model YOLO hoặc kết nối robot.

- Chọn `gap` (hai vật cản), `detour` (vật cản phía trước) hoặc `empty`.
- Nhập độ rộng khe theo cm, bấm **Đặt lại cảnh**, rồi **Chạy**.
- Nhấp bản đồ bên trái để đổi mục tiêu. Thao tác này tạm dừng; bấm Chạy để tiếp tục.
- **Bước 0,1 s** thực hiện một bước thời gian mô phỏng.
- Bên trái là thế giới thật giả lập, lưới 50 cm; bên phải là bản đồ chỉ từ tia LiDAR.
- Đọc Navigation, Monitor và hai vận tốc để phân biệt tìm đường với lệnh được phép chạy.

Mô phỏng import trực tiếp `Navigator`, `Costmap` và `Monitor` của code hiện tại.
Giữ kích thước thân 32,4 × 26 cm, biên 6 cm và vùng mù LiDAR mặc định.
Dùng giá trị mặc định trong Python, không đọc YAML hay tham số đang chạy trên robot.
Thời gian là thời gian giả lập; máy chậm có thể chạy chậm hơn thời gian thực.

Không mô phỏng YOLO, camera depth, sai số odometry, ma sát/trượt bánh, ROS,
độ trễ hoặc lỗi đồng bộ LiDAR–odometry. Vì vậy mô phỏng này kiểm tra thuật toán
hình học và điều khiển, chưa tái hiện lỗi WAIT_SCAN trên phần cứng.

Kiểm tra không mở giao diện:

```powershell
python simulate_navigation.py --headless --scenario gap --gap 0.60
python -m unittest test_simulation
```
