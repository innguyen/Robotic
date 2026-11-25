# P2.3 — Mở rộng corpus và benchmark nghiên cứu

**Trạng thái 2026-11-25: TẠM HOÃN theo ưu tiên của người dùng.** Thực hiện [S0 — Simulator Feasibility](simulator-feasibility.md) trước; chỉ mở lại P2.3 khi có quyết định tiếp tục. Nội dung dưới đây là kế hoạch được giữ lại, chưa phải công việc đang triển khai.

P2.3 chỉ bắt đầu sau cổng [nghiệm thu P2.2 cuối](benchmark-p22-final-audit.md). Pilot dev không chứng minh B1 tốt hơn B0. Giữ protocol t1-0.2; mọi thay đổi về policy, evaluator, sensing hoặc metric phải có version và nêu tác động tới khả năng so sánh.

## 1. Xây corpus có provenance

Lập danh sách layout mới thay vì nhân seed/start/target trên bảy geometry hiện có rồi coi là môi trường độc lập. Mỗi layout có nguồn gốc, giấy phép nếu dùng dữ liệu ngoài, phương pháp dựng/chuyển đổi, phiên bản generator, seed, đơn vị, frame, bounds, footprint, obstacle geometry và lý do gán family. Lưu cả nguồn và bản resolved để tái tạo được hash.

Phủ S1–S5, các mức che khuất/độ hẹp/nhánh/tuyến vòng và negative scenes. Bổ sung trường hợp camera offset, fallback kéo dài, retry sau stalled, mất target quá memory rồi tái phát hiện. Chọn thiết kế trước khi xem outcome của B0/B1; witness chỉ chứng minh tính khả thi theo hợp đồng, không phải đường đi cho policy.

Review provenance và đặc trưng scenario riêng với review outcome. Ghi mã đợt review, bằng chứng và quyết định chấp nhận/từ chối trong hồ sơ corpus. Không xóa case khó sau khi thấy policy thất bại. Case chưa đủ bằng chứng giữ UNVERIFIED hoặc bị loại theo lý do chất lượng dữ liệu đã công bố, không tự PASS.

## 2. Kiểm registry, family và certificate

Chạy toàn corpus trước selection: schema, rect-union-um-v1, environment/layout identity, family và dev/test leakage. Hash chỉ phát hiện geometry tương đương theo canonicalization; cần review thêm layout gần giống, bản xoay/dịch/scale và biến thể cùng nguồn. Các episode cùng environment hoặc family phải cùng split.

Mỗi scenario cần certificate PASS hiện hành với shared config/sensing/budget đã khóa. S3/S4 cần witness/gates thực sự được verifier xác minh. Giữ cả fixture validation, resolved bytes, registry, catalog và nguồn verifier; không sửa certificate để bỏ qua source mismatch. Lưu danh sách case bị loại và lý do.

## 3. Chốt thiết kế đánh giá trước test

Khóa B0/B1 IDs, selector parameters và source fingerprints sau phát triển trên dev. Cùng Navigator/mission/safety/oracle/coverage, cùng fixture và budget; trajectory khác nhau là bình thường. Không tune policy hoặc tăng budget riêng sau khi xem held-out results.

Chọn số family/layout và số episode mỗi scenario theo độ chính xác cần báo và nguồn lực thực chạy; không coi cỡ mẫu của pilot nhỏ là đã đủ. Nếu tính CI hoặc kiểm định, đăng ký trước estimand, phương pháp paired theo episode và bootstrap theo family. Khi số family còn ít, ưu tiên counts, từng cặp và giới hạn suy luận; không tạo CI hẹp giả bằng nhiều seed xác định trên cùng geometry.

Primary endpoint là SafeMissionSuccess trên positive episodes đã bắt đầu; ReportSuccess tách riêng. Giữ mọi primary error trong mẫu số và báo end_reason. Negative false alarms có nhóm/mẫu số riêng. Path/time phải đi kèm outcome và missingness: không coi prefix ngắn của crash hoặc không tìm thấy người là hiệu quả hơn. SPL, oracle coverage và certified clearance vẫn null tới khi có phương pháp được nghiệm thu.

## 4. Khóa test corpus

Tạo release manifest bất biến gồm catalog, family/split, resolved fixture hashes, certificates, shared configuration, policy/evaluator/reducer/runtime identity, execution profile, bảng metrics và quy tắc primary/retry. Review đầy đủ trước mở test; lưu checksum và revision. GT/witness/certificate chỉ ở evaluator-side, không đưa cho selector.

Chạy smoke/dev controls và replay trên môi trường dự kiến, không dùng test để sửa tham số. Trước phiên chạy cần kiểm host liên tục hoạt động, process isolation, dung lượng lưu log và nguồn điện do người dùng quản lý. Docker/Linux phải có kiểm thử thực riêng; chưa có log thì UNVERIFIED, không gộp nền tảng thành cùng điều kiện thực nghiệm.

Nếu phát hiện lỗi dữ liệu hoặc implementation sau khi mở test, dừng diễn giải kết quả, giữ bản lỗi và quyết định có version. Thay hợp đồng hoặc corpus cần release mới và chạy lại đầy đủ phần bị ảnh hưởng; không thay vài dòng kết quả để giữ bảng đẹp.

## 5. Chạy và công bố

Tạo paired manifest mới, chạy toàn bộ episode đã khóa với từng job tuần tự và host-liveness evidence. Lưu journal, attempts/episodes, raw events/trajectory, evaluator result, source/config identity, monitor state entries và continuous episodes riêng. Retry chỉ chẩn đoán, không thay attempt 1. Công bố cả số dự kiến/bắt đầu/hoàn tất/lỗi và null reasons.

Tái tính metrics từ raw, kiểm identity, paired CSV và deterministic replay trước phân tích. Báo theo scenario và family, giữ các cặp thất bại và execution errors. Phân tích trace để phân biệt navigation/search failure hợp lệ, dữ liệu sai và lỗi runner; không đổi scoring sau khi nhìn outcome.

Đầu ra: corpus release/provenance review, thiết kế đánh giá đã khóa, manifests và raw artifacts, kiểm tra độc lập, báo cáo nghiên cứu kèm giới hạn. T1 vẫn là 2D oracle; kết luận về RGB-D/YOLO, địa hình thiên tai thật hay robot thật cần T2/T3 và nghiệm thu riêng.
