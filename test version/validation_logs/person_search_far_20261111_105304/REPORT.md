# Lượt tìm person tại vị trí xa hơn

Ngày thực hiện theo máy người dùng: 11/11/2026. Đồng hồ robot đang ghi
18/09/2026; giữ nguyên tên rosbag theo đồng hồ robot để đối chiếu.

## Kết quả trên robot

- Chế độ `live`, mục tiêu `person`, nguồn bản đồ GMapping.
- Giới hạn chạy 60 giây; xác nhận `PAUSED`, `enabled=false` và lệnh sau
  Collision Monitor bằng 0 sau khoảng 61,26 giây tính từ phản hồi yêu cầu.
- Không đạt `FOUND`. Trong 104 mẫu HTTP có 18 mẫu `SEARCH_SPIN`, 86 mẫu
  `WAIT_SCAN`; không có mẫu xác nhận hoặc nhìn thấy người.
- Chỉ ghi nhận lệnh xoay, tối đa 0,524 rad/s; không có lệnh tịnh tiến trong
  các mẫu HTTP. Không đủ bằng chứng để đánh giá thăm dò và đi đường dài.
- Bản đồ toàn cục vẫn có 102.400 ô; số lần reset bộ nhớ cục bộ bằng 0 suốt
  các mẫu của nhiệm vụ. Không ghi nhận hiện tượng reset bản đồ trong lượt này;
  kích thước lưới không chứng minh độ chính xác hình học của bản đồ.

## Dấu hiệu cần nâng cấp tiếp

| Chỉ số từ telemetry | Trung vị | P95 | Lớn nhất |
| --- | ---: | ---: | ---: |
| Tuổi scan mới nhận theo header | 0,145 s | 0,177 s | 0,183 s |
| Tuổi scan đã xử lý dùng cho điều hướng | 0,598 s | 0,829 s | 0,896 s |
| Cập nhật bản đồ cục bộ | 152,6 ms | 236,1 ms | 330,5 ms |
| Bước lập đường/tính lệnh tìm kiếm | 294,4 ms | 430,7 ms | 524,9 ms |

Collision Monitor có 58 mẫu `STALE_COMMAND`, 35 `IDLE`, 11 `PASS`.
Đây là mẫu trạng thái HTTP, không phải tỷ lệ thời gian chính xác.

Suy luận từ log: chi phí xử lý trong vòng điều hướng là nguyên nhân đáng
nghi chính khiến dữ liệu đã xử lý vượt watchdog 0,5 s và lệnh bị gián đoạn.
Chưa đủ bằng chứng để quy mọi lỗi cho driver LiDAR hoặc cho bản đồ mất TTL.
Cần tối ưu chi phí xử lý và xác minh lại trước khi kết luận tìm người ổn định.
Không nới watchdog để che độ trễ trong lượt này.

## Sửa lỗi đã áp dụng trước lượt hoàn chỉnh

- `start_navigation_ros1.sh`: kiểm tra XML-RPC của node để phân biệt node
  đang chạy với API URI cũ còn lưu trong ROS Master.
- `navigation_ros1.py`: giữ lệnh 0 với `WAIT_SEARCH` nếu yêu cầu search đến
  giữa vòng điều khiển và nhiệm vụ tương ứng chưa được tạo.
- `map_core_ros1.py`: đầu vào pose/target thiếu hoặc không hợp lệ trả về
  `INVALID_GOAL` và lệnh 0, thay vì crash ở `numpy.isfinite`.
- 31 kiểm tra liên quan đã đạt trước khi tải bản sửa lên robot. Bản dự phòng
  của các file sửa được giữ trên robot. Lượt hoàn chỉnh không gặp crash trên.

## Dữ liệu

`telemetry.jsonl`, `summary.json`, `telemetry_analysis.json`, `runner.log`,
`stack.log`, `stop_stack.txt`; manifest rosbag chứa SHA-256 sau khi tải xong.
Không ghi hình camera, nên không thể xác minh độc lập bằng video rằng người
đã nằm trong trường nhìn trong lượt này. Không có phép đo độc lập để khẳng
định không va chạm chỉ từ các trạng thái này.
