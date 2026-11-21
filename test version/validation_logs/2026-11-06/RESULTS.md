# Kiểm tra trên robot ngày 06/11/2026

Kết nối `jetauto@192.168.149.1`, mạng HW-66151611. Mã cũ được sao lưu trên robot
tại `/home/jetauto/test_version_backup.Qktxzj`; mã mới được tải vào
`/home/jetauto/test_version`. Model YOLO có cùng SHA-256 với bản trong dự án.

## Preview

- Chạy tìm person 6,66 giây (bao gồm bước dừng).
- Không ghi nhận lệnh vận tốc khác 0 trên hai topic chassis; odom dịch chuyển 0 m.
- Đầu ra monitor là `/collision_monitor/cmd_vel_checked`.
- Lưu dữ liệu trong `preview_trial.json`.

## Live có giới hạn thời gian

- Đầu ra monitor là `/jetauto_controller/cmd_vel`.
- Chạy 44,77 giây, kết thúc do node báo `FOUND`.
- Odom ghi nhận khoảng 0,07 m dịch chuyển tịnh tiến; có xoay quét tìm kiếm.
- Ghi nhận 568 lệnh chassis khác 0. Mẫu cuối ghi lệnh điều hướng/chassis bằng 0.
- Khi kết thúc, chương trình kiểm tra gọi pause và gửi thêm chuỗi lệnh 0.
- Sau đó phiên kiểm thử tự tắt; kiểm tra tiến trình không còn ROS Master,
  odom/chassis hoặc các node test chạy trong phiên này.

## Bản đồ và hiệu năng

- Chế độ search dùng thời gian nhớ 120 giây.
- Số ô đã biết tăng từ khoảng 1.000 lên 4.776; reset_count luôn bằng 0.
- expired_cells giữ nguyên 368 trong toàn bộ lượt live; số này đã có trước lúc
  bật tìm kiếm. Không thấy bằng chứng mất ô do TTL trong lượt đang tìm kiếm này.
- Trong 88 mẫu (khoảng 0,5 giây/mẫu), có 8 mẫu WAIT_SCAN. Đây là số mẫu,
  không phải số sự kiện mất scan độc lập.
- map_ms cao nhất 323,1 ms; async_planner_ms cao nhất 2.011,7 ms.
- Tiếp tục cần xử lý/đo độ trễ trên Nano. Một lượt FOUND khi mục tiêu ở gần
  chưa chứng minh robot đi ổn định trên tuyến dài hoặc mọi bố trí vật cản.

## Tệp dữ liệu

- `live_trial.json`: trạng thái, vận tốc, odom và bộ đếm bản đồ của 88 mẫu.
- `camera_during_trial.jpg`: một khung hình trong lúc xoay tìm kiếm; tại thời
  điểm chụp chưa có person được xác nhận.
- Các `.log` là log khởi động và log kiểm thử lấy từ robot.

Thời lượng được đo bằng monotonic clock. Đồng hồ lịch trên robot đang lệch
so với ngày của máy người dùng, nên timestamp trong log robot mang ngày khác.

## Tự chạy lại từ Windows

Tại PowerShell ở thư mục gốc dự án:

```powershell
& '.\test version\run_test_robot.bat' -Target person
```

Lệnh trên là preview. Khi muốn xe di chuyển:

```powershell
& '.\test version\run_test_robot_live.bat' -Target person
```

Giữ cửa sổ launcher mở, vào `http://192.168.149.1:5010`, chọn person rồi bấm
Tự động tìm kiếm. Tạm dừng trên web dừng điều hướng; Ctrl+C trong launcher
dừng bộ test. Có thể chạy stop_test_robot.bat để dừng riêng bộ test.
