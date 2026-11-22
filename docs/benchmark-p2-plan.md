# Kế hoạch P2 — Baseline B0/B1 và metrics T1

Đầu vào là runner P1b.2 đã nghiệm thu trên Windows và protocol t1-0.2. Đây là kế hoạch tổng thể, không phải kết quả nghiên cứu. Linux và các giới hạn trong [P1b.2](benchmark-p1b2-audit.md) vẫn phải được theo dõi.

Cập nhật triển khai: selector B0/B1 và controls được ghi tại [P2.1 audit](benchmark-p21-audit.md); metrics/paired pilot có [kế hoạch](benchmark-p22-plan.md) và [audit P2.2](benchmark-p22-audit.md). Các mục dưới đây giữ vai trò yêu cầu, không tự chứng minh đã hoàn tất.

## 1 Khóa giao diện và điều kiện so sánh

- Tách selector khỏi mission dùng chung. Cả hai nhận cùng public config, model sensing/pose/động học, bounds, Navigator, recovery, coverage adapter, monitor, reporting/evaluator và budget.
- Mỗi policy có state/map/trajectory riêng. Không replay pose của B1 cho B0; không cho selector truy cập fixture, certificate, witness hoặc evaluator.
- Policy ID/config/source fingerprints rõ ràng. Snapshot selector parameters vào manifest và resume identity; thay thuật toán/config phải tạo run mới và certificate mới khi source hash thay đổi.

## 2 Triển khai B0 fixed-order-v2

Theo §5.1 spec: lưới waypoint neo tại start/yaw0, spacing công khai, thứ tự vòng rồi góc, chọn từ observed-free standable domain dùng chung. Unknown được xét lại, waypoint đã giao không được lặp. Không thiết kế waypoint từ GT mục tiêu.

Khi không có waypoint đủ điều kiện, dùng fallback từ miền ứng viên chung với B1, sắp theo thứ tự cố định, không dùng gain/focus/min-gain scoring. Không có ứng viên thì stalled/retry theo dwell đến deadline; vẫn nhận detection mới để approach. Không tự kết luận đã tìm hết môi trường.

Test bắt buộc: thứ tự tính tay, transform frame, unknown xét lại, blacklist/revisit, waypoint không lặp, fallback có thể đi tiếp, stalled tiếp tục sensing, target mới thoát stalled và giữ nguyên monitor.

## 3 Chuẩn hóa B1 từ selector hiện tại

Đối chiếu chính xác `SearchMission.choose_goal()` và MissionAdapter smoke. B1 chính thức phải có phiên bản riêng, giữ gain/distance/focus/min-gain hiện tại nhưng dùng domain/coverage/reporting/lifecycle chung. Không tự coi tên `search-mission-smoke` đã là baseline B1 nghiệm thu.

Test: cùng observation/state cho cùng action; thay GT ẩn không đổi action; bounds/domain và planner giống B0. Chạy lại replay riêng từng policy trước thử nghiệm cố định.

## 4 Positive controls trên dev

Chạy B0 và B1 trên S1 dễ thấy, S2 có điểm nhìn gần, S3 có nhánh đi được, S4 có tuyến vòng. Kiểm tra selector thực sự tạo goal/di chuyển và có khả năng hoàn thành chuỗi task trong cấu hình chung. Lưu goals_selected, fallback_goals, first_stalled_s, stalled_sim_s, unresolved_waypoints và end_reason.

Nếu B0 thất bại do implementation hoặc baseline bị bóp khả năng khám phá/budget, chỉ báo chẩn đoán. Sửa trên dev, tăng policy version rồi chạy lại cả cặp trước khi khóa test. Không bỏ các lượt thất bại hoặc dùng thời gian ngắn của lượt bỏ cuộc làm bằng chứng hiệu quả.

## 5 Hoàn thiện metrics và phép thử tính tay

| Đại lượng                          | Nghiệm thu                                                                                                                |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| ReportSuccess / SafeMissionSuccess | Tách report đúng khỏi tiếp cận/dừng an toàn; mọi positive episode đã bắt đầu thuộc mẫu số, kể cả lỗi thực thi             |
| Negative false alarms              | Mẫu số riêng; oracle không đại diện false-positive rate YOLO                                                              |
| Thời điểm / path                   | Tách raw/confirmed/report/stop, 0 khác null; path từ trajectory GT, không lấy đường ngắn của mission thất bại làm lợi thế |
| Collision / clearance              | Swept footprint độc lập monitor; phương pháp và uncertainty rõ ràng; clearance chưa đo tin cậy giữ null                   |
| Monitor / recovery / stalls        | Đếm đợt, không đếm mỗi tick thành một intervention mới; state transitions và stall duration có đối chứng                  |
| Retry / aggregation                | Primary attempt không bị thay bởi retry thành công; so sánh cặp cùng episode/config, có counts và runtime errors          |
| SPL / GT coverage                  | Chỉ bổ sung khi chứng nhận được reference path tới A và evaluator coverage; chưa có thì null với lý do                    |

Giữ các công thức diagnostic của batch làm regression fixtures, sau đó kiểm tra từ raw events/trajectory bằng ví dụ tính tay độc lập, không chỉ assert lại hàm đang triển khai. Chưa công bố bảng chất lượng policy từ log smoke hiện tại.

## 6 Pilot và cổng đánh giá cố định

Chạy paired dev pilot để kiểm tra fixture traits, controllability của baseline, budget, replay và mức phân biệt giữa hai selector. Chốt corpus/params/test split theo environment và family; rà nguồn sinh tránh near-duplicate. Mọi sample phải có certificate PASS hiện hành.

Chỉ sau pilot và acceptance mới chạy test đã khóa. Báo theo S1–S5, giữ failed/error attempts, số layout/family/episode và paired differences. Nếu suy luận thống kê, bootstrap theo family và giữ cặp episode; số family ít thì chỉ báo counts và từng kết quả. Không dùng seed lặp một mô phỏng xác định như mẫu môi trường độc lập.

Đầu ra P2: hai selector có phiên bản, tests positive controls/replay, metric validation có dữ liệu tính tay, manifest paired runs và báo cáo pilot nêu đầy đủ lỗi/giới hạn. Không chỉnh defaults ROS, YAML hoặc robot thật trong bước này.
