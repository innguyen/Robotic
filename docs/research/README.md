# S3 — Research Blueprint & Unreal Engine 5 Foundation

Ngày bàn giao: **2026-11-25, Asia/Saigon**. Repository `innguyen/Robotic`, nhánh `feat/benchmark`, source nền `671add0137a0ef2f06081bde402e7556e083b15d`.

**S3 hoàn tất ở nhánh không có hạ tầng UE5 đã được xác minh: blueprint nghiên cứu, quyết định nền tảng, hợp đồng dữ liệu và quy trình nghiệm thu. Chưa có vertical slice UE5 chạy thật, benchmark kết quả hoặc novelty đã được chứng minh.** Không thay Webots S0–S2, Godot S0, production hoặc T1.

Đề xuất tập trung vào UGV tìm và báo vị trí nạn nhân dưới suy giảm cảm biến và self-localization. Kết quả cần đo là khả năng báo đúng, an toàn, đúng hạn và biết mức độ tin cậy; photorealism là phương tiện tạo các điều kiện đó. Không mở rộng sang bế/cứu nạn nhân, UAV hay multi-agent trong phiên bản đầu.

| Deliverable | Nội dung |
| --- | --- |
| [Literature / gap matrix](literature-gap-matrix.md) | Đối chiếu nghiên cứu trực tiếp và bằng chứng chống lại các tuyên bố novelty quá rộng |
| [Benchmark proposal](benchmark-proposal.md) | Research gap, ba RQ, thiết kế thực nghiệm và validation JetAuto/Astra |
| [Evaluation protocol](evaluation-protocol.md) | Task, observations/actions, GT boundary, splits, metrics, baselines, thống kê và failure attribution |
| [UE5 platform decision](ue5-platform-decision.md) | UnrealZoo/RescueBench so với Project AirSim, sensor/robot/asset strategy và acceptance gates |
| [Architecture](architecture.md) | UE5 chính, Webots đối chứng nhẹ, isolation và vòng đời; mapping thành phần tái sử dụng |
| [S3 audit](s3-audit.md) | Bằng chứng cục bộ, kiểm tra đã chạy, giới hạn và các hạng mục UNVERIFIED |
| [Scenario contract](contracts/scenario.schema.json) | JSON Schema của **gói authoring riêng tư**, không gửi thẳng cho agent |
| [Scenario mẫu](contracts/scenario.example.json) | Fixture thiết kế, có nhãn `design_only`, không phải scene UE đã tồn tại |
| [Kiểm tra hạ tầng](tools/preflight_ue5.py) | Chỉ đọc máy, ghi JSON mới; không cài đặt, launch engine hoặc điều khiển robot |

Các ngưỡng và số lượng corpus là **đề xuất `sar-s3-draft-0.1`**, chưa đăng ký trước, chưa khóa test. Phải hiệu chuẩn bằng dev data, freeze protocol và evaluator trước khi mở test. Các tiêu chí nghiệm thu trong tài liệu là công việc kỹ thuật tiếp theo, không phải các mốc cần xin xác nhận lại.

Đọc [audit](s3-audit.md) để phân biệt `OBSERVED_LOCAL`, `REPORTED_PRIOR`, `DOCUMENTED_UPSTREAM`, `DESIGN` và `UNVERIFIED`. Nguồn online được truy cập ngày 2026-11-25; tài liệu upstream theo nhánh có thể thay đổi. Không dùng ngày crawl làm ngày xuất bản. Paper chưa xác minh proceedings được ghi là preprint; không suy diễn acceptance từ trang tổng hợp.
