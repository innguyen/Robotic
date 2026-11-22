# P2.2 — Metrics, paired runs và dev pilot

P2.1 cung cấp selector có version và log dev controls; [audit P2.1](benchmark-p21-audit.md) quyết định phần nào đã được nghiệm thu. P2.2 chưa triển khai. Giữ scoring protocol `t1-0.2`, không mở test hoặc công bố bảng B0/B1 từ controls này.

## 1. Khóa cách đo từ raw log

Viết reducer kiểm tra độc lập event/trajectory, đối chiếu result và projection. Mỗi test dùng ví dụ nhỏ có đáp án tính tay: path tiến/ngang/quay; mốc t=0; chưa report là null; report đúng rồi crash; report đúng nhưng dừng/tiếp cận không đạt; report sai cấu trúc trên S5; mẫu số 0; primary thất bại nhưng retry thành công. Không lấy boolean Navigator/monitor làm kết quả chấm hình học.

Bắt buộc nghiệm thu `goals_selected`, `fallback_goals`, thời điểm/tổng stalled và `unresolved_waypoints`, bao gồm episode dừng khi còn goal active và log bị cắt do crash. Giữ riêng các đại lượng không có ở B1; không ép null thành 0. Bổ sung policy ID/version vào CSV xuất cho paired analysis, đối chiếu manifest để tránh ghép nhầm.

## 2. Monitor interventions — yêu cầu bắt buộc trước pilot

Runner hiện phát MONITOR_INTERVENTION khi monitor đổi sang trạng thái ngoài PASS/IDLE/LIMIT; batch đếm các event đó. Như vậy không đếm lại mỗi tick cùng trạng thái, nhưng một chuỗi SLOWDOWN → STOP_ZONE → PREDICTED_COLLISION có thể sinh nhiều event dù không có khoảng hoạt động bình thường ở giữa. Trường hiện tại chưa được nghiệm thu như số đợt chặn liên tục.

Đối chiếu câu “đợt liên tiếp cùng trạng thái chặn” trong §9.2 và chốt rõ hai đại lượng: số đợt theo từng trạng thái và số đợt can thiệp liên tục bất kể chuyển trạng thái. Giữ raw MONITOR_STATE để tái tính. Nếu cách gộp làm đổi metric đã định nghĩa, phải giải thích comparability, version hóa reducer/metric, và xin thống nhất protocol trước khi đổi nghĩa trường cũ; không âm thầm sửa scoring `t1-0.2`.

Test tối thiểu: PASS→STOP→STOP→PASS→STOP; SLOWDOWN→STOP→SLOWDOWN; LIMIT/IDLE; scan stale; APPROACH; đổi trạng thái đúng deadline; crash giữa đợt; episode không có can thiệp. Kiểm tra cả số đợt, thời lượng, mức nghiêm trọng và sự tách biệt với collision evaluator. Cổng này bắt buộc, không dùng con số hiện tại để kết luận policy an toàn hơn.

## 3. Hợp đồng paired runs

Tạo paired manifest khóa danh sách episode, resolved fixture bytes/hash, layout/environment/family/split, seed, protocol, shared config, sensing/động học, evaluator/metric version và identity riêng từng selector. Không ghép theo riêng layout_hash hoặc scenario. Cả hai phải có primary attempt cho mọi episode đã bắt đầu; execution error vẫn trong mẫu số, retry chỉ ở bảng audit riêng.

Từ chối khác shared config/budget/runtime, duplicate episode hoặc thiếu cặp. Resume không được sửa run hoàn tất. Replay chạy riêng mỗi policy, không đòi hai policy có trajectory giống nhau. Gate source identity bằng tests sửa selector parameters/source fingerprint và tests stale certificates.

## 4. Pilot chỉ trên dev

Trước pilot phải xử lý các control FAIL/BLOCKED của P2.1, nếu còn. Dùng nguyên fixtures/control gốc để đối chiếu; bổ sung layout dev có provenance riêng khi cần đa dạng, không thay/xóa những lượt thất bại. Nếu sửa policy, tăng implementation version và chạy lại controls/replay cho cả hai với certificate mới.

Kiểm tra nhiều layout/family, camera offset, rolling map, nhánh chuyển tiếp, fallback, stalled dài, memory mất target và detour. Ghi nhận lỗi implementation/runner và thất bại hợp lệ của thuật toán theo trace; tính khả thi witness không chứng minh policy nhất định thành công. Không kéo dài budget riêng B0, không đưa witness vào policy, không đổi evaluator để controls đạt.

Pilot kiểm tra khả năng đo và ghép cặp; chưa là đánh giá nghiên cứu. Chỉ sau khi metrics, paired identity, controls và replay đạt mới đề xuất khóa corpus/parameters/test. Linux/cross-OS vẫn UNVERIFIED cho tới khi có môi trường Python phù hợp và log thực.

## 5. Đầu ra và điều kiện chuyển bước

- Reducer/paired manifest có version, unit tests tính tay, integration crash/resume và dữ liệu đối chứng.
- Tất cả metric hiện có được đánh dấu VERIFIED hoặc null kèm lý do; SPL/GT coverage/clearance chưa có phương pháp đủ tin cậy không được điền số thay thế.
- Log paired dev pilot giữ mọi primary failure/error và source/fixture identity. Báo cáo không suy rộng số seed của một hình học thành số môi trường độc lập.
- Audit P2.2 ghi lệnh/kết quả thực, PASS/FAIL/BLOCKED/UNVERIFIED, giới hạn và gate trước test đã khóa. Production ROS/camera/launcher/YAML/Collision Monitor giữ nguyên.
