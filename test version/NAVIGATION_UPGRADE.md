# Nâng cấp điều hướng và bản đồ lâu dài

Phạm vi là `test version`. Bản nâng cấp được kiểm tra offline trên Windows;
AMCL/GMapping và thời gian xử lý trên Jetson cần kiểm tra thêm trên robot.

## Các thay đổi

- `STUCK` giữ lệnh 0 cho cùng mục tiêu, kể cả khi hướng robot thay đổi. Xoay
  có ngưỡng vào/ra 45°/30°, giới hạn 18 s và dừng nếu góc không tiến triển 3 s.
- Chỉ kiểm tra phần đường còn lại. Giữ đường an toàn khi tìm lại đường thất bại
  hoặc đường mới không cải thiện đủ 15%; thông thường tìm lại mỗi 2 s.
- Giới hạn số ô mở rộng mỗi lần lập đường. Kết quả nền của mục tiêu/bản đồ cũ
  bị loại; dữ liệu mới được kiểm tra lại trước khi phát vận tốc.
- YAML robot yêu cầu 3 scan xác nhận trống để xóa vật cản cục bộ. Phát hiện
  vật cản mới vẫn có hiệu lực ngay. TTL không được làm mới bằng quan sát giả.
- Bám người có kiểm tra tính nhất quán sau khi bù chuyển động robot. Khi chỉ
  nhớ vị trí, robot giảm tốc. `FOUND` cần mục tiêu còn mới, đang thấy và odometry
  xác nhận đã dừng liên tục 0,6 s. `VERIFY_TARGET` là trạng thái đang xác minh.
- Thăm dò chỉ chọn vùng có thể đi tới; lưu vùng camera đã nhìn trong hệ tọa độ
  điều hướng. Có giới hạn tiến triển khi tiếp cận người để tránh xoay vô hạn.

## Ba chế độ bản đồ

| MapMode | Nguồn bản đồ | Hệ tọa độ điều hướng |
| --- | --- | --- |
| `local` (mặc định) | Bản đồ LiDAR cục bộ; nhớ 20 s, tìm kiếm 120 s | `odom` |
| `mapping` | GMapping tạo `/map`, hiệu chỉnh `map → odom` | `map` |
| `localization` | `map_server` đọc YAML/PGM; AMCL định vị với mô hình `omni` | `map` |

Trong hai chế độ mới, bản đồ toàn cục **không hết hạn theo TTL**. Lớp cục bộ
vẫn có thời gian nhớ để xử lý vật cản động; tia trống cục bộ không xóa tường
trong bản đồ đã lưu. Lập đường toàn cục chạy trong worker; điểm trung gian
được chuyển cho bộ điều khiển cục bộ và Collision Monitor đang dùng.

Lập đường toàn cục dùng hình bao tròn; kiểm tra chuyển động cục bộ dùng thân xe
hình chữ nhật và quét quỹ đạo. Khi hình bao tròn từ chối vị trí sát đồ đạc nhưng
thân xe vẫn đủ chỗ, bộ điều khiển thử đường cục bộ theo hình học thực và thử lại
đường toàn cục. Đường toàn cục và cục bộ đều không đi xuyên ô chưa biết.

Robot dừng khi mất/cũ TF định vị; không dùng phép biến đổi đơn vị thay thế.
Adapter đọc trực tiếp cạnh TF động `map → odom` do AMCL/GMapping phát, gồm
kiểm tra timestamp và đồng hồ monotonic. Không yêu cầu module nhị phân `tf`
cho Python 3; nếu dùng nguồn định vị khác phải phát đúng cạnh trực tiếp này.
AMCL cần covariance đạt ngưỡng. Hiệu chỉnh vị trí lớn tạm dừng nhiệm vụ để
tránh dùng mục tiêu/quan sát sai; các hiệu chỉnh nhỏ xóa bộ nhớ cục bộ rồi cập nhật.

Giới hạn tài nguyên: bản đồ đầu vào tối đa 2 triệu ô, bản đồ chấm điểm thăm dò
tối đa 250.000 ô ở độ phân giải điều hướng, và `max_plan_expansions` mặc định
20.000. Cần đo trên Nano trước khi tăng các giới hạn hoặc kích thước bản đồ.

## Thử giả lập trên Windows

Trong thư mục gốc dự án:

```powershell
& '.\test version\run_simulation.bat' --scenario search --persistent-map
```

Hoặc mở giả lập, chọn `search`, bật **Bản đồ lâu dài**, bấm **Đặt lại cảnh**
rồi **Chạy**. Bản đồ trong chế độ này là mặt bằng giả lập đã biết; không dùng
SLAM để suy ra mặt bằng. Người được giữ là vật cản động, không ghi vào mặt bằng.

Chạy kiểm tra tự động từ thư mục `test version`:

```powershell
python -m unittest discover -v
python validate_navigation_upgrade.py
```

`validation_logs/navigation_upgrade_simulation.json` lưu kết quả từng cảnh,
thời gian giả lập, số lần đổi chiều quay, số lần lập/đổi đường và kiểm tra va
chạm độc lập giữa thân xe và đồ đạc. Thời gian thực thi trên Windows không
đại diện cho Jetson. Camera ở đây là mô hình lý tưởng; không chạy YOLO/depth,
ROS, AMCL hoặc GMapping, và không mô phỏng trượt bánh.

## Khởi động ROS trên robot khi cần kiểm tra tiếp

Cần các gói ROS Melodic `gmapping`, `map_server`, `amcl`, `tf`, `rosbag` cùng
driver hiện có. Launcher kiểm tra gói cần dùng và từ chối chạy thêm nguồn `/map`
khi đã có nguồn khác. Các cấu hình không tự cài gói hay thay hệ điều hành.

Cần TF `odom → base_footprint → lidar_frame` (dùng tên thực tế launcher phát hiện).
Giữ phép đo vị trí gắn LiDAR thống nhất giữa TF và `collision_monitor_ros1.yaml`.
Nguồn quét cho định vị được cắt còn vùng quan sát -89,9° tới 119,9°, giữ timestamp
của tia đầu; Collision Monitor tiếp tục dùng scan gốc và vùng mù đã cấu hình.

Tải bản nâng cấp và khởi động chế độ preview tạo bản đồ:

```powershell
& '.\test version\run_test_robot.bat' -Target person -MapMode mapping
```

Preview không phát lệnh tới chassis; muốn tạo bản đồ cần đưa robot quan sát
các hướng/vị trí bằng lượt kiểm tra di chuyển riêng. Khi bản đồ đủ tốt, lưu
trên robot trong terminal ROS (ví dụ đặt tên mới, không ghi đè bản đồ đang dùng):

```bash
mkdir -p /home/jetauto/maps
rosrun map_server map_saver -f /home/jetauto/maps/room_v1 map:=/map
```

Dùng lại bản đồ trên robot:

```powershell
& '.\test version\run_test_robot.bat' -Target person -MapMode localization -RemoteMap /home/jetauto/maps/room_v1.yaml
```

Đặt **2D Pose Estimate** trong RViz, Fixed Frame `map`, rồi chờ định vị ổn định.
Dashboard báo `WAIT_LOCALIZATION`, `LOCALIZATION_UNCERTAIN` hoặc `WAIT_GLOBAL_MAP`
khi chưa đủ dữ liệu. Chế độ live dùng launcher `run_test_robot_live.bat` với
cùng tham số; chỉ dùng khi tiến hành lượt kiểm tra chuyển động trên robot.

## Log trên robot và đánh giá lại

Launcher mặc định ghi bag trực tiếp trên robot, không truyền log qua SSH tới
laptop. Recorder chạy theo vòng đời bộ test: khi supervisor dừng hoặc nhận
HUP do mất kết nối, recorder đóng bag; đây chưa phải dịch vụ chạy nền độc lập.
Giữ khoảng 5 đoạn rosbag, mỗi đoạn khoảng 32 MiB tại
`/home/jetauto/test_version/recordings`; tính cả đoạn `.active` bị gián đoạn.
Đoạn đầu của phiên hiện tại được giữ để có bản đồ/TF tĩnh latched; các vị trí
còn lại giữ dữ liệu gần nhất. Các đoạn của phiên cũ sẽ dần bị thay thế.
Kích thước có thể vượt nhẹ vì một message lớn và buffer của rosbag. Gồm scan,
odom, TF, map, mục tiêu, đường đi, trạng thái và vận tốc. Ctrl+C đóng bag;
bag bị ngắt đột ngột có thể cần `rosbag reindex`. Dùng `-NoRecord` để tắt recorder.

Trên môi trường có module Python `rosbag`, có thể chạy:

```bash
python3 replay_navigation.py recordings/session_....bag --output replay_report.json
```

Nếu đánh giá đoạn sau trong chế độ bản đồ lưu sẵn, truyền thêm
`--bootstrap-map recordings/<đoạn_đầu_cùng_phiên>.bag` để nạp `/map` mà
`map_server` chỉ phát khi recorder bắt đầu đăng ký. Chọn đúng phiên và mặt bằng;
không dùng bản đồ từ lượt thử khác để so sánh. ROSbag Melodic không lặp lại
message latched ở mỗi lần chia đoạn.

Script chỉ đọc bag và tính lại lệnh bằng mã hiện tại, **không tạo ROS publisher**.
Nó so sánh trên các vị trí và nhiệm vụ đã ghi (open loop); kết quả không chứng
minh robot sẽ đi theo quỹ đạo mới an toàn. Ca kín vòng nằm trong giả lập và
cần kiểm tra thêm trên robot. Chưa chạy replay từ các bag còn trên robot ở
lượt debug trước vì phiên kết nối đã được dừng theo yêu cầu.

Tham chiếu cấu hình ROS: [GMapping](https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/src/slam_gmapping.cpp),
[AMCL](https://github.com/ros-planning/navigation/blob/melodic-devel/amcl/src/amcl_node.cpp),
[map_server](https://github.com/ros-planning/navigation/blob/melodic-devel/map_server/src/main.cpp).
