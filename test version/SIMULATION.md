# Mô phỏng trên máy tính

Bản nâng cấp thêm tùy chọn **Bản đồ lâu dài**. Chọn `search`, bật tùy chọn,
bấm **Đặt lại cảnh**, rồi **Chạy** để thử tìm người với mặt bằng lưu sẵn.
Launcher tự tìm Python trong PATH hoặc thư mục cài Python của người dùng.

```powershell
& '.\run_simulation.bat' --scenario search --persistent-map
```

Chạy `python validate_navigation_upgrade.py` để kiểm tra 7 cảnh có đánh giá
va chạm độc lập và lưu kết quả JSON. Hướng dẫn cấu hình mới và giới hạn kiểm thử
nằm trong [NAVIGATION_UPGRADE.md](NAVIGATION_UPGRADE.md).

Chạy `run_simulation.bat` trong thư mục này. Cần Python có tkinter và NumPy
(`python -m pip install numpy`). Không cần ROS, model YOLO hoặc kết nối robot.

- Chọn `gap` (hai vật cản), `detour` (vật cản phía trước), `room` (phòng có tường và
  đồ đạc, mục tiêu nằm sau hộp), `search` (tự động tìm người) hoặc `empty`.
- Cảnh `search`: người (chấm tím) nấp sau tủ; robot xoay quét, đi tới vùng chưa nhìn
  (hình thoi vàng), thấy người thì tiếp cận và đánh dấu đỏ ĐÃ TÌM THẤY. Ô xanh xám ở bản đồ
  phải là vùng camera đã nhìn. Nhấp bản đồ trái để đặt người chỗ khác (tìm lại từ đầu).
  Camera giả lập: ±29°, tầm 3,5 m, bị vật cản che; không mô phỏng lỗi nhận diện YOLO.
- Nhập độ rộng khe theo cm, bấm **Đặt lại cảnh**, rồi **Chạy**.
- Nhấp bản đồ bên trái để đổi mục tiêu. Thao tác này tạm dừng; bấm Chạy để tiếp tục.
  Mặc định là mục tiêu vision (dừng cách vật). Bật **Nhấp = điểm bản đồ** để giống nút
  chọn điểm trên web: điểm chưa quan sát hoặc bị chặn bị từ chối như trên robot.
- Giống `navigation_ros1.py`: tìm đường chạy luồng riêng (có trạng thái PLANNING) và
  trạng thái chặn của Collision Monitor dừng lệnh như hàm `gate()`.
- **Bước 0,1 s** thực hiện một bước thời gian mô phỏng.
- Bên trái là thế giới thật giả lập, lưới 50 cm; bên phải là bản đồ chỉ từ tia LiDAR.
- Đọc Navigation, Monitor và hai vận tốc để phân biệt tìm đường với lệnh được phép chạy.

Mô phỏng import trực tiếp `Navigator`, `Costmap` và `Monitor` của code hiện tại.
Chế độ `search` dùng `search_memory_s` (YAML hiện tại: 120 giây); các cảnh còn lại
dùng `memory_s` (20 giây). Dòng trạng thái hiển thị thời gian nhớ, số ô đã biết
và số ô bị loại do hết hạn/ra ngoài cửa sổ để chẩn đoán hiện tượng mất bản đồ.
Mặc định đọc `navigation_ros1.yaml` và `collision_monitor_ros1.yaml` (giống robot;
cần PyYAML: `python -m pip install pyyaml`). Thêm `--defaults` để dùng giá trị mặc định
trong Python. Tiêu đề cửa sổ ghi cấu hình đang dùng. Robot xoay được (`wz`) trong giả lập.
Thời gian là thời gian giả lập; máy chậm có thể chạy chậm hơn thời gian thực.

Không mô phỏng YOLO, camera depth, sai số odometry, ma sát/trượt bánh, ROS,
độ trễ hoặc lỗi đồng bộ LiDAR–odometry. Vì vậy mô phỏng này kiểm tra thuật toán
hình học và điều khiển, chưa tái hiện lỗi WAIT_SCAN trên phần cứng.

Kiểm tra không mở giao diện:

```powershell
python simulate_navigation.py --headless --scenario gap --gap 0.60
python -m unittest test_simulation
```
