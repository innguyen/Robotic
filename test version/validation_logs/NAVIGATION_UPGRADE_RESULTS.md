# Kết quả kiểm tra bản nâng cấp điều hướng

Phạm vi kiểm tra: source trong `test version`, chạy offline trên Windows.
Không kết nối hoặc phát lệnh tới robot trong lượt triển khai này.

## Kết quả giả lập kín vòng

Chạy `validate_navigation_upgrade.py` với cấu hình YAML của robot. Mỗi bước
tích phân chuyển động là 0,1 giây giả lập. Bộ kiểm tra va chạm độc lập dùng
thân xe có hướng và các hình chữ nhật của thế giới, sau mỗi bước chuyển động.

| Cảnh | Kết quả | Thời gian giả lập | Va chạm ghi nhận |
| --- | --- | ---: | --- |
| Qua khe | ARRIVED | 9,5 s | Không |
| Đi vòng vật cản | ARRIVED | 18,6 s | Không |
| Đi tới điểm phía sau | ARRIVED | 11,2 s | Không |
| Phòng có bản đồ lâu dài | ARRIVED | 28,7 s | Không |
| Đi tới điểm cách 8 m | ARRIVED | 60,7 s | Không |
| Tìm người, bản đồ cục bộ | FOUND | 183,9 s | Không |
| Tìm người, mặt bằng lưu sẵn | FOUND | 76,1 s | Không |

Chi tiết máy đọc được: `navigation_upgrade_simulation.json`.
Hai lượt tìm người sử dụng cùng bản nâng cấp với hai cấu hình bản đồ khác nhau;
đây không phải phép đo so sánh trước/sau trên robot thật.

## Hồi quy và tính tương thích

- Bộ hồi quy toàn dự án: **129 test, 0 lỗi, 0 bỏ qua**, lưu trong
  `navigation_upgrade_tests.txt` và `navigation_upgrade_tests.json`.
- Sau thay đổi cuối của bridge TF thuần message, chạy lại **28 test liên quan**
  (`test_map_adapter_ros1`, `test_navigation_upgrade`, `test_session_recorder`):
  tất cả đạt. Nhóm này có các test trùng với bộ toàn dự án ở trên.
- Dashboard: **19 kiểm tra đạt**.
- Cú pháp Python 3.6 và XML launch đã được kiểm tra; Bash và PowerShell đã
  được kiểm tra cú pháp. Đây không phải chạy node ROS trên Jetson.
- Launcher giả lập Windows đã được chạy ở chế độ headless: cảnh trống ARRIVED.

Các ca hồi quy chính gồm: STUCK không tự chuyển sang xoay, quay không có tiến
triển thì dừng, không hủy đường bởi vật cản đã đi qua, giữ đường khi tính lại
thất bại, cần ba scan trống để xóa vật cản, bản đồ tồn tại sau TTL/reset bộ nhớ,
đi vòng trên bản đồ toàn cục, loại kết quả cũ khi bản đồ đổi, tọa độ map/odom,
dừng khi mất TF hoặc AMCL chưa hội tụ, không báo FOUND khi chỉ nhớ vị trí,
và giữ đúng giới hạn log cùng đoạn đầu chứa bản đồ.

## Giới hạn và bước xác minh tiếp theo

Giả lập dùng LiDAR/odometry lý tưởng và camera hình học; không chạy YOLO/depth,
AMCL hoặc GMapping thật. Nó không tái hiện trượt bánh, nhiễu cảm biến và tải CPU
của Nano. Chờ planner trong giả lập không được tính thành thời gian robot chạy;
do đó kết quả không chứng minh hết WAIT_SCAN hoặc độ trễ phần cứng.

Chưa triển khai các thay đổi mới lên robot, chưa xác minh TF thực tế hoặc tốc độ
xử lý khi chạy đồng thời SLAM/AMCL và YOLO. Chưa phát lại các bag của lượt debug
cũ còn trên robot. Công cụ `replay_navigation.py` đã có nhưng cần môi trường
Python chứa module `rosbag` và dữ liệu tương ứng để đánh giá open loop.

Để thử chế độ cũ của bản đồ, dùng `-MapMode local` (mặc định); hai chế độ mới là
`mapping` và `localization`. Cách mở giả lập, cấu hình ROS, lưu bản đồ và sử dụng
log nằm trong [NAVIGATION_UPGRADE.md](../NAVIGATION_UPGRADE.md).
