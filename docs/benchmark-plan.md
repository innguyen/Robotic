# Kế hoạch khảo sát và xây dựng benchmark cho robot tìm kiếm con người trong môi trường thiên tai

## 1. Mục tiêu và phạm vi đã thống nhất

**Bài toán chính của nhóm:** robot tự di chuyển trong môi trường thiên tai để tìm kiếm con người.

**Ưu tiên ban đầu:** hoàn thiện báo cáo khảo sát phương pháp xây dựng benchmark và đề xuất giao thức đánh giá cho nhóm, trước khi triển khai demo.

Tiến độ hiện tại: harness T1 offline đã qua [P1b.2 trên Windows](benchmark-p1b2-audit.md); hai selector chính thức và dev controls được theo dõi tại [P2.1 audit](benchmark-p21-audit.md). Bước tiếp theo là [P2.2 metrics, paired runs và pilot](benchmark-p22-plan.md). Chưa có đánh giá B0/B1 trên test đã khóa hoặc kết quả robot thật. Các mục bên dưới là kế hoạch tổng thể ban đầu, không phải danh sách đã hoàn thành.

Kế hoạch được tổ chức theo thứ tự công việc, đầu ra và tiêu chí hoàn thành; không đặt lịch hoặc hạn thực hiện.

### 1.1. Điều kiện hiện tại

- Có môi trường ROS1 để triển khai.
- Hiện nhóm chưa thể sử dụng robot JetAuto.
- Có mã nguồn tìm kiếm, điều hướng, collision monitor và mô phỏng offline trong `test version/`.
- Tài liệu bàn giao `ROBOT_AUTONOMOUS_SEARCH_BENCHMARK_CODEX_HANDOFF.md` là đầu vào tham khảo. Các mô tả thuật toán, ngưỡng và phương án thử nghiệm trong đó cần được đối chiếu với source hoặc nguồn nghiên cứu trước khi sử dụng như một kết luận.

### 1.2. Phân biệt mục tiêu nghiên cứu và phạm vi kiểm chứng

| Mức                                                          | Vai trò                                                                                             | Kết luận có thể đưa ra                                |
| ------------------------------------------------------------ | --------------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| Môi trường có đặc trưng thiên tai được mô phỏng có kiểm soát | Đánh giá các thách thức được lựa chọn, như che khuất, lối đi bị chặn và điều kiện cảm biến suy giảm | Năng lực đối với từng đặc trưng thực sự được tái hiện |
| Môi trường hoặc sân thử đại diện sát thực tế hơn             | Kiểm tra khả năng chuyển từ mô phỏng sang thực tế và giới hạn phần cứng                             | Năng lực trong các điều kiện đã thử trực tiếp         |

Phạm vi sân thử cần phù hợp với khả năng phần cứng và tái hiện rõ các đặc trưng thiên tai được lựa chọn. Khả năng vượt đống đổ nát, leo bậc, hoạt động trên nền không ổn định hoặc trong môi trường nguy hiểm phải được đánh giá riêng bằng các thử nghiệm tương ứng.

Phát hiện một người không đồng nghĩa với xác định người đó bị thương hoặc cần cấp cứu. Phân loại người nằm và phát hiện sự kiện ngã là những bài toán mở rộng, chỉ đưa vào đánh giá khi có yêu cầu, dữ liệu và phương pháp tương ứng.

## 2. Sản phẩm cần hoàn thành trước mắt

Tên báo cáo làm việc:

> **Khảo sát phương pháp xây dựng benchmark cho robot tự hành tìm kiếm con người trong môi trường thiên tai và đề xuất giao thức đánh giá cho hệ thống của nhóm.**

| Sản phẩm                        | Nội dung bắt buộc                                                                                               |
| ------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| Bảng khảo sát có bằng chứng     | Phương pháp thực nghiệm của ít nhất năm bài nghiên cứu/tổng quan, kèm các protocol và phương pháp thử liên quan |
| Phân tích tổng hợp              | Điểm chung, khác biệt, hạn chế, điều kiện so sánh công bằng và bài học cho nhóm                                 |
| Protocol đề xuất v0.1           | Task, observation, episode, scenario, ground truth, baseline, success/failure, metrics và quy trình đánh giá    |
| Đối chiếu hệ thống hiện tại     | Thành phần có thể tái sử dụng, giới hạn của simulator và dữ liệu/khả năng còn thiếu                             |
| Lộ trình triển khai sau báo cáo | Các bước xây demo, điều kiện hoàn thành và phụ thuộc cần giải quyết                                             |

Báo cáo cần nối được chuỗi **bằng chứng từ nghiên cứu → bài học về phương pháp → quyết định thiết kế benchmark của nhóm**. Danh sách metric đơn thuần chưa đủ đáp ứng mục tiêu này.

## 3. Câu hỏi dẫn dắt khảo sát

1. Các tác giả định nghĩa nhiệm vụ tìm kiếm và một episode như thế nào?
2. Robot biết gì khi bắt đầu và được nhận những observation nào trong quá trình chạy?
3. Những đặc trưng nào của môi trường thiên tai được tái hiện? Đặc trưng nào bị đơn giản hóa hoặc bỏ qua?
4. Môi trường, vị trí xuất phát, vị trí mục tiêu và độ khó được lựa chọn hoặc sinh ra như thế nào?
5. Ground truth lấy từ đâu? Đầu ra thuật toán có bị dùng làm nhãn chấm điểm cho chính thuật toán không?
6. Khi nào tính là nhìn thấy, xác nhận, định vị, báo cáo đúng và hoàn thành nhiệm vụ?
7. Baseline, ablation và các yếu tố được giữ cố định là gì?
8. Các lượt thất bại, báo sai, timeout, collision và lỗi hệ thống được xử lý thế nào?
9. Số scene, số episode và số lần lặp được lựa chọn, tổng hợp và báo cáo ra sao?
10. Những kết luận nào được dữ liệu hỗ trợ và những kết luận nào chưa thể suy rộng?

## 4. Chọn tài liệu khảo sát

### 4.1. Các bài nghiên cứu/tổng quan chính

| Nguồn                                       | Trọng tâm cần khai thác                                                                 | Bằng chứng cần tìm                                                           |
| ------------------------------------------- | --------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| Into the Robotic Depths / DARPA SubT        | Thiết kế đánh giá hệ thống tìm kiếm trong môi trường khó, tìm–phát hiện–định vị–báo cáo | Thiết kế thử nghiệm, scoring, giới hạn báo cáo, phân biệt Systems và Virtual |
| ZAVIS                                       | Active visual search, đánh giá perception tách khỏi search, baseline và ablation        | Problem formulation; §V-A–V-C; Tables III–IV                                 |
| Goal-Oriented Semantic Exploration — SemExp | ObjectNav, observation, scene split, baseline, ablation và giả định cảm biến            | §4, §4.1, §5.1–5.3                                                           |
| BARN                                        | Sinh môi trường, định lượng độ khó, repetition và fair comparison                       | §III, §IV-A–IV-B                                                             |
| MRPB 1.0                                    | Nhóm scenario, data logging, safety, efficiency và smoothness                           | §III–V                                                                       |

Các công trình ObjectNav và navigation được dùng để kế thừa phương pháp đánh giá từng năng lực. Cần phân tích rõ khoảng cách giữa task của chúng và nhiệm vụ tìm người trong thiên tai.

### 4.2. Nguồn phương pháp và protocol cần đối chiếu

| Nguồn                                                            | Vai trò trong báo cáo                                                                                                             |
| ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| NIST Response Robot Test Methods                                 | Tìm cách chuẩn hóa điều kiện và quy trình thử robot phục vụ ứng phó khẩn cấp; xác định phương pháp nào phù hợp phần cứng của nhóm |
| RoboCupRescue Robot League                                       | Khảo sát cách tổ chức bài thử và đánh giá các năng lực liên quan tới cứu hộ; phân biệt điểm thi đấu với metric nghiên cứu         |
| Habitat ObjectNav và On Evaluation of Embodied Navigation Agents | Bổ sung cách đóng gói episode, observation, STOP, chia scene và định nghĩa metric                                                 |

NIST, luật thi đấu và tài liệu challenge phải được ghi đúng loại nguồn, không tính thành các bài nghiên cứu thực nghiệm để làm đủ số paper. Khi dùng luật hoặc protocol, lưu phiên bản/năm/commit đã đối chiếu.

Rà soát kết quả trích xuất để xác định phần nào sẽ kế thừa, phần nào cần điều chỉnh và phần nào vượt khả năng kiểm chứng hiện tại.

### 4.3. Danh mục nguồn gốc

1. Into the Robotic Depths: <https://doi.org/10.1146/annurev-control-062722-100728>.
2. DARPA SubT, gồm mục Scoring và các tài liệu quy định: <https://www.darpa.mil/research/programs/darpa-subterranean-challenge>.
3. ZAVIS: <https://arxiv.org/abs/2209.08803>; bản HTML: <https://arxiv.org/html/2209.08803v3>.
4. SemExp: <https://arxiv.org/abs/2007.00643>; bản HTML: <https://arxiv.org/html/2007.00643v2>.
5. BARN: <https://arxiv.org/abs/2008.13315>; bản HTML: <https://arxiv.org/html/2008.13315v3>.
6. MRPB 1.0: <https://arxiv.org/abs/2011.00491>; bản HTML: <https://arxiv.org/html/2011.00491v2>.
7. NIST Response Robot Test Methods: <https://www.nist.gov/el/intelligent-systems-division-73500/standard-test-methods-response-robots>.
8. RoboCupRescue Robot League Rules: <https://rrl.robocup.org/rules/>.
9. Habitat ObjectNav 2023: <https://aihabitat.org/challenge/2023/>; protocol: <https://github.com/facebookresearch/habitat-challenge>.
10. On Evaluation of Embodied Navigation Agents: <https://arxiv.org/abs/1807.06757>.

Các liên kết là danh mục làm việc. Chỉ đưa một nhận xét cụ thể vào báo cáo sau khi tìm được bằng chứng trong nguồn tương ứng.

## 5. Quy trình đọc và phân tích tài liệu

### 5.1. Phiếu trích xuất thống nhất

| Trường                | Nội dung cần ghi                                                                                |
| --------------------- | ----------------------------------------------------------------------------------------------- |
| Citation và version   | Tên bài, tác giả, nơi công bố, phiên bản được đọc                                               |
| Research objective    | Câu hỏi nghiên cứu và năng lực được đánh giá                                                    |
| Task                  | Tìm bất kỳ instance, một instance cụ thể hay tất cả mục tiêu; có yêu cầu tiếp cận/báo cáo không |
| Robot và sensors      | Phần cứng, footprint, cảm biến, khả năng chuyển động                                            |
| Observation/action    | Thông tin agent được phép nhận và hành động được phép thực hiện                                 |
| Episode               | Điều kiện khởi tạo, budget, reset và điều kiện kết thúc                                         |
| Environment           | Simulator/dataset/sân thử, số scene, split và đặc trưng môi trường                              |
| Ground truth          | Nguồn GT, frame, đơn vị, độ chính xác và tính độc lập                                           |
| Difficulty            | Biến điều khiển độ khó và cách xác nhận tình huống hợp lệ                                       |
| Success/failure       | Quy tắc chấm, ngưỡng, STOP, timeout, collision và báo sai                                       |
| Baselines/fairness    | Phương pháp so sánh và các yếu tố được giữ cố định                                              |
| Metrics               | Định nghĩa, công thức, đơn vị, mẫu số và trường hợp không áp dụng                               |
| Repetition/statistics | Số episode/lần lặp, cách tổng hợp, độ bất định và đơn vị lấy mẫu                                |
| Ablations             | Yếu tố được thay đổi và mục đích kiểm tra                                                       |
| Reproducibility       | Code, config, dữ liệu, seed, phiên bản và hướng dẫn tái hiện                                    |
| Limitations           | Giả định, yếu tố gây nhiễu, giới hạn kết luận và sim-to-real                                    |
| Evidence              | Section, trang, bảng hoặc hình hỗ trợ nhận xét                                                  |
| Applicability         | Nhóm kế thừa, điều chỉnh hoặc chưa áp dụng nội dung nào; lý do                                  |

Quy tắc ghi chép:

- Không suy đoán số scene, số lượt chạy hoặc ngưỡng mà paper không nêu rõ.
- Phân biệt `chưa xác minh từ bản gốc` với `nguồn không báo cáo`.
- Tách thông tin được paper công bố khỏi nhận xét và đề xuất của nhóm.
- Tách đánh giá định lượng khỏi demonstration định tính.
- Không so sánh trực tiếp success rate của các công trình khác protocol để xếp hạng thuật toán.
- Không xem một ngưỡng từ paper là ngưỡng phù hợp robot nhóm nếu chưa có lập luận hoặc kiểm chứng.

### 5.2. Phân tích tổng hợp bắt buộc

Sau khi điền phiếu, tổng hợp theo các trục sau:

1. **Task và success:** nhìn thấy, xác nhận, tiếp cận, định vị và báo cáo khác nhau thế nào?
2. **Thông tin đầu vào:** có bản đồ trước hay tự khám phá; perception/pose thực hay lý tưởng hóa?
3. **Đặc trưng thiên tai:** những khó khăn nào được kiểm tra, bằng phương pháp nào và ở mức độ nào?
4. **Fair comparison:** cùng scene, mục tiêu, budget, sensing, giới hạn chuyển động và module chung hay không?
5. **Thiết kế thực nghiệm:** số môi trường, điều kiện khởi tạo, biến thể và cách xử lý thất bại.
6. **Giới hạn kết luận:** năng lực nào đã được kiểm chứng, năng lực nào còn thiếu bằng chứng?

Mỗi bài học cần dẫn tới một quyết định thiết kế có thể giải thích cho benchmark của nhóm.

## 6. Cấu trúc báo cáo đề xuất

| Phần                    | Nội dung                                                                 | Đầu ra cần đạt                                        |
| ----------------------- | ------------------------------------------------------------------------ | ----------------------------------------------------- |
| 1. Bài toán và phạm vi  | Robot tìm người trong thiên tai và các đặc trưng môi trường cần đánh giá | Xác định đúng mục tiêu chính và phạm vi kiểm chứng    |
| 2. Khung khảo sát       | Tiêu chí chọn nguồn và phiếu trích xuất                                  | Có phương pháp đọc và so sánh thống nhất              |
| 3. Khảo sát công trình  | Thiết kế thực nghiệm, bảng so sánh và ví dụ tiêu biểu                    | Giải thích được các tác giả đã benchmark như thế nào  |
| 4. Phân tích và bài học | Task, GT, độ khó, baseline, fairness, repetition và limitations          | Nêu được lý do lựa chọn phương pháp đánh giá          |
| 5. Hệ thống hiện tại    | Kiến trúc liên quan, khả năng tái sử dụng và giới hạn                    | Chứng minh tính khả thi mà không phóng đại hiện trạng |
| 6. Protocol đề xuất     | Task, episode, scenario, metrics và quy trình thử                        | Có thiết kế đủ cụ thể để triển khai tiếp              |
| 7. Hạn chế và lộ trình  | Những đặc trưng chưa tái hiện, dữ liệu và phần cứng còn thiếu            | Giới hạn rõ kết luận và hướng mở rộng                 |
| Phụ lục                 | Phiếu đọc, bằng chứng, công thức và mẫu episode                          | Truy vết được từng nhận xét quan trọng                |

Trong báo cáo, trình bày số liệu của paper dưới nhãn kết quả của paper. Chưa có benchmark của nhóm thì chỉ trình bày thiết kế đề xuất, không tạo bảng kết quả giả.

## 7. Thiết kế protocol v0.1 cho nhóm

### 7.1. Câu hỏi đánh giá đề xuất

- Chiến lược tìm kiếm hiện tại có cải thiện tỷ lệ tìm và báo cáo đúng người so với baseline đơn giản không?
- Che khuất, lối đi bị chặn và độ phức tạp bố trí vật cản ảnh hưởng thế nào tới khả năng hoàn thành?
- Hệ thống có hoàn thành an toàn, và thất bại thường xuất hiện ở khâu tìm kiếm, perception hay điều hướng?
- Khi có dữ liệu phù hợp, điều kiện cảm biến suy giảm ảnh hưởng thế nào tới detection và localization?

### 7.2. Tách tầng đánh giá và đặc trưng môi trường

Hai chiều này cần được ghi độc lập: một thí nghiệm dùng cảm biến/robot thật chưa tự động đại diện tốt cho thiên tai; một scene có tên thiên tai cũng chưa bảo đảm đã tái hiện được các thách thức thực tế.

| Tầng đánh giá                     | Input và mục đích                                                                       | Giới hạn                                                              |
| --------------------------------- | --------------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| T1 — Search/navigation simulation | Hình học môi trường, sensing mô phỏng, detector oracle; đánh giá tìm kiếm và điều hướng | Không đo chất lượng YOLO, RGB-D thực hoặc khả năng vượt địa hình thực |
| T2 — Perception offline           | Ảnh/video/RGB-D có GT độc lập; đánh giá detection, confirmation và localization         | Không tự chứng minh robot chọn đường tìm kiếm tốt                     |
| T3 — End-to-end                   | Hệ thống tích hợp trên robot hoặc mô phỏng đã mô tả/kiểm chứng phù hợp                  | Chỉ kết luận trong điều kiện và mức độ trung thực thực sự được thử    |

Mỗi suite cần có bảng đặc trưng môi trường, gồm: đặc trưng, cách tái hiện, tham số điều khiển, GT và giới hạn mô hình. MVP có thể tái hiện che khuất, vật cản tĩnh và đường bị chặn trên mặt phẳng. Ánh sáng yếu, bụi/khói, trượt bánh và nền không bằng phẳng để mở rộng khi có dữ liệu hoặc simulator tương ứng.

### 7.3. Nhiệm vụ và episode

Nhiệm vụ ban đầu: tìm một người trong vùng thử, xác nhận mục tiêu, tiếp cận tới khoảng cách đã quy định, báo cáo vị trí và dừng.

Một episode cần lưu:

- ID, phiên bản schema/protocol, scene/environment ID và split.
- Pose xuất phát, frame và đơn vị; giới hạn không gian nhiệm vụ.
- Hình học vật cản và target GT, hoặc danh sách mục tiêu rỗng.
- Mode perception, sensor geometry, footprint và cấu hình thuật toán đã resolve.
- Budget nhiệm vụ, điều kiện thành công, điều kiện dừng và collision policy.
- Seed cùng mô tả biến nào thực sự được seed điều khiển.
- Config/fixture hash, phiên bản code và metadata đủ để tái hiện.

GT thuộc môi trường và evaluator. Policy chỉ nhận observation được phép, không nhận trước vị trí người hoặc bản đồ GT. Pose GT và odometry estimate cần được phân biệt; nếu đồng nhất trong T1 phải ghi rõ giả định.

### 7.4. Bộ scenario đầu tiên

| ID  | Scenario                                             | Mục đích                                               |
| --- | ---------------------------------------------------- | ------------------------------------------------------ |
| S1  | Người dễ quan sát                                    | Kiểm tra chuỗi phát hiện–xác nhận–tiếp cận–báo cáo     |
| S2  | Người bị vật cản che, có góc quan sát khác           | Kiểm tra khả năng đổi điểm quan sát                    |
| S3  | Người ở khu vực phải khám phá mới thấy               | Đánh giá chiến lược tìm kiếm                           |
| S4  | Người ở khu vực phải đi vòng hoặc qua tuyến thay thế | Đánh giá tìm kiếm kết hợp điều hướng khi đường bị chặn |
| S5  | Không có người                                       | Đánh giá báo sai và hành vi kết thúc tìm kiếm          |

Mỗi nhóm cần nhiều layout và điều kiện khởi tạo. Xác nhận vùng quan sát/tiếp cận khả thi theo footprint, margin và mô hình chuyển động. Trường hợp cố ý không thể hoàn thành phải được gắn nhãn và chấm theo mục đích riêng.

Các layout phát triển và đánh giá cần được tách ở cấp môi trường. Đổi seed hoặc điểm xuất phát trên cùng một layout không biến nó thành môi trường test độc lập.

### 7.5. Baseline và điều kiện so sánh

- **B0:** quét và chọn điểm quan sát theo quy tắc cố định, chỉ dùng thông tin được phép; không thiết kế waypoint từ GT mục tiêu.
- **B1:** chiến lược tìm kiếm hiện tại của `SearchMission`.

Giữ chung scene/episode, sensing, perception, navigation, monitor, giới hạn chuyển động, budget và điều kiện báo cáo. Ưu tiên thay riêng khâu chọn điểm khám phá để làm rõ tác dụng của chiến lược tìm kiếm.

Ablation chỉ bổ sung sau khi baseline comparison hoạt động. Mỗi ablation thay một yếu tố; không vô hiệu hóa safety trên robot thật.

### 7.6. Success, failure và metrics

Cần ghi riêng `raw_detection`, `confirmed_detection`, `target_report` và `mission_completion`. Một target report phải phát sinh từ hệ thống được đánh giá, không được evaluator tự tạo từ GT.

| Metric/trạng thái                    | Quy tắc đề xuất                                                                                     |
| ------------------------------------ | --------------------------------------------------------------------------------------------------- |
| Report success rate                  | Tính trên episode có người; báo cáo đúng class/vị trí và đáp ứng quy tắc nhiệm vụ                   |
| Safe mission success                 | Hoàn thành task, dừng theo quy định và không collision trong episode                                |
| Detection/confirmation/report timing | Đo riêng từng sự kiện từ cùng mốc bắt đầu episode                                                   |
| Path length                          | Từ trajectory GT trong T1; ghi rõ nguồn đo ở các tầng khác                                          |
| Collision và minimum clearance       | Evaluator hình học độc lập với quyết định của monitor; xét footprint và chuyển động giữa các bước   |
| Monitor interventions                | Ghi số đợt, nguyên nhân và độ dài can thiệp; không đồng nhất với collision                          |
| Negative episodes                    | Báo riêng false reports, kết thúc tìm kiếm và timeout; không cộng vào success rate cảnh có người    |
| End reason                           | Phân biệt success, search exhausted, timeout, stuck, collision, planner timeout và lỗi chương trình |

Các ngưỡng và thứ tự ưu tiên khi nhiều điều kiện kết thúc xảy ra cùng bước phải được chốt trước khi chạy bộ đánh giá. Khoảng cách tiếp cận người và ngưỡng sai số vị trí báo cáo là hai đại lượng khác nhau.

Các giới hạn bắt buộc công bố:

- Detector oracle trong cảnh không người không cung cấp bằng chứng về false-positive rate của YOLO thực.
- Hết điểm khám phá không tự chứng minh toàn môi trường không có người.
- Sai số định vị bằng 0 do oracle không chứng minh độ chính xác của camera.
- Coverage do `SearchMission` ước lượng không phải ground-truth coverage.
- Chỉ báo SPL khi định nghĩa và tính được đường tham chiếu phù hợp tới vùng hoàn thành nhiệm vụ; chưa đủ điều kiện thì dùng success rate và path length.

### 7.7. Tổ chức thử nghiệm và báo cáo thống kê

1. Smoke test fixture và runner bằng bộ nhỏ đại diện S1–S5.
2. Pilot trên tập phát triển để kiểm tra tính hợp lệ, khả năng tái lập, chi phí chạy và độ phân biệt giữa methods.
3. Chốt protocol, cấu hình, bộ test và quy mô đánh giá dựa trên pilot; không mặc định con số trong tài liệu bàn giao là cỡ mẫu đã được chứng minh đủ.
4. Chạy cùng episode IDs và điều kiện ngoại sinh cho từng method.
5. Báo số scene, số episode, success/failure counts, end reasons, metric theo scenario và aggregate.
6. Khi tính khoảng tin cậy hoặc bootstrap, xét các episode cùng scene là dữ liệu có phân nhóm; không mặc định tất cả lượt chạy độc lập.

Không bỏ lượt thất bại khỏi báo cáo. Thời gian chỉ tính trên successful runs phải được gắn nhãn rõ, kèm kết quả cho toàn bộ episode và timeout. Giá trị chưa có là `null`, không thay bằng 0.

## 8. Đối chiếu code trước khi triển khai demo

Kiểm tra sơ bộ phục vụ lập kế hoạch đã xác nhận checkout đúng repository `innguyen/Robotic`, nhánh `feat/benchmark`, HEAD `b7025fd182399519c097a5286ffeb64991c02439`. Commit này trùng commit tham chiếu của tài liệu bàn giao, dù tên nhánh khác `detec`. Cần kiểm tra lại HEAD và working tree khi bắt đầu triển khai.

| Thành phần                               | Khả năng tái sử dụng / điểm cần kiểm tra                                                                              |
| ---------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `test version/search_core_ros1.py`       | Mission và lựa chọn điểm khám phá; ghi riêng detection và report vì `found` được đặt sau khi navigation báo `ARRIVED` |
| `test version/navigation_core_ros1.py`   | Navigation core; kiểm soát async planning, reset, cleanup và ảnh hưởng wall-time tới reproducibility                  |
| `test version/collision_monitor_ros1.py` | Monitor dùng chung; giữ nguyên cơ chế bảo vệ và đo collision bằng evaluator riêng                                     |
| `test version/simulate_navigation.py`    | Điểm bắt đầu cho T1; scene đang hard-code, perception lý tưởng hóa, cần bổ sung fixture và instrumentation            |
| `test version/jetauto_find_object.py`    | Nền tảng cho T2; cần tách luồng inference/recorded data phù hợp khỏi ROS runtime                                      |
| YAML và test hiện có                     | Snapshot cấu hình thực sự được dùng; chạy regression trước/sau thay đổi                                               |

Trong phiên kiểm tra sơ bộ, chưa chạy được test vì Python launcher trỏ tới executable không tồn tại. P0 sau đó đã xác định Python 3.10 hoạt động và chạy regression offline; xem [kết quả P0 và giới hạn](benchmark-p0-audit.md). Các mục protocol v0.1 trong kế hoạch là đề xuất ban đầu; hợp đồng T1 hiện hành là [t1-0.2](benchmark-t1-spec.md). Chưa có kết quả benchmark hoặc kiểm thử robot thật.

## 9. Lộ trình triển khai sau báo cáo

| Bước                       | Công việc                                                                         | Điều kiện hoàn thành                                                            |
| -------------------------- | --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| P0 — Audit và regression   | Xác minh checkout, Python/dependencies; đọc source liên quan và chạy test offline | Có báo cáo PASS/FAIL/BLOCKED, lý do và phạm vi chưa kiểm chứng                  |
| P1a — Protocol và fixture  | Cụ thể hóa schema, GT, end reasons và S1–S5                                       | Fixture hợp lệ, có kiểm tra hình học, đơn vị và điều kiện khởi tạo              |
| P1b — Runner headless      | Batch episodes, isolation, reset, timeout, cleanup và log                         | Chạy offline, không cần camera/robot; có manifest và output kiểm tra được       |
| P2 — Baseline và evaluator | B0/B1, metrics, comparison report                                                 | Cùng episode và điều kiện thử; metrics được kiểm tra bằng ví dụ tính trước      |
| Pilot và đánh giá cố định  | Kiểm tra replay trên dev rồi chạy suite test đã khóa                              | Báo cáo gồm cả thành công/thất bại, truy ngược được tới log                     |
| P3 — Perception offline    | Dữ liệu RGB-D có GT, inference thật và phân tích điều kiện cảm biến               | Có split phù hợp, metadata model và kết quả độc lập với T1                      |
| P4 — End-to-end            | Tích hợp và thử trên JetAuto khi nhóm sử dụng được robot                          | Có quy trình thử trực tiếp, GT độc lập và kết quả đúng phạm vi điều kiện đã thử |
| Mở rộng                    | Nhiều người, posture, moving obstacles, địa hình hoặc cảm biến suy giảm           | Mỗi tính năng có dữ liệu/mô hình, GT và tiêu chí chấm tương ứng                 |

### 9.1. Đầu ra kỹ thuật tối thiểu của demo T1

- Suite/fixture machine-readable có schema version.
- Runner offline/headless, budget và watchdog rõ; mỗi episode reset đầy đủ.
- Chế độ benchmark kiểm soát thời điểm nhận kết quả planner, thay vì dựa vào seed để khẳng định deterministic.
- `run_manifest.json`: code/config/fixture versions và hashes, runtime, policy, seed, mode perception/planner và quy ước đơn vị.
- `episodes.csv`, `events.jsonl`, trajectory và `summary.json`/`summary.md`.
- Resume/retry có định danh run/attempt rõ, không nhân đôi episode hoặc che mất lượt lỗi.
- Tests cho schema, metric, positive/negative episodes, timeout, lỗi, replay, resume và ROS/hardware isolation.
- Hướng dẫn tái hiện và ghi rõ các metric chưa hỗ trợ.

Không âm thầm fallback sang Python defaults khi benchmark yêu cầu YAML nhưng không đọc được cấu hình. Ghi riêng simulation time và wall-clock runtime; khi kiểm tra replay không yêu cầu các trường đo wall-clock phải giống nhau.

## 10. Giới hạn thay đổi và vận hành

- Giai đoạn báo cáo chỉ đọc, phân tích và chuẩn bị tài liệu; chưa bao gồm triển khai demo hoặc thay thuật toán.
- Khi triển khai T1, giữ benchmark harness tách khỏi ROS/hardware runtime và bảo toàn CLI/test hiện có khi có thể.
- Không bật live, SSH vào robot, gửi lệnh motor hoặc thay firmware trong quy trình benchmark offline.
- Không bỏ qua collision monitor, watchdog, safety gate hay biến cấu hình thử nghiệm thành mặc định robot thật.
- Thử nghiệm robot thật là bước riêng khi nhóm có phần cứng và người phụ trách trực tiếp; các tham số chưa đo/hiệu chuẩn phải được ghi rõ.
- Dữ liệu người cần có quyền sử dụng; không đưa raw video hoặc thông tin cá nhân vào repository theo mặc định.

## 11. Tiêu chí hoàn thành báo cáo trước mắt

- [ ] Mục tiêu và phạm vi báo cáo tập trung vào tìm con người trong môi trường thiên tai.
- [ ] Có phiếu phân tích ít nhất năm bài nghiên cứu/tổng quan và đối chiếu các protocol liên quan tới cứu hộ.
- [ ] Những nhận xét quan trọng có section/trang/table/figure để kiểm chứng.
- [ ] Có phân tích experimental methodology, không chỉ liệt kê thuật toán hoặc metric.
- [ ] Phân biệt rõ thông tin nguồn, nhận xét của nhóm và thiết kế đề xuất.
- [ ] Có bảng nối bài học từ khảo sát với quyết định thiết kế benchmark.
- [ ] Protocol v0.1 định nghĩa được task, observation, episode, GT, scenario, baseline, success/failure và metrics.
- [ ] Nêu được các đặc trưng thiên tai đã dự kiến tái hiện và phần chưa thể tái hiện.
- [ ] Phân biệt T1, T2, T3 và giới hạn kết luận theo các điều kiện thực sự được kiểm chứng.
- [ ] Đối chiếu code có bằng chứng, không tuyên bố test/demo đã chạy khi chưa thực hiện.
- [ ] Đã rà soát tính nhất quán của thuật ngữ, số liệu và trích dẫn.
- [ ] Có lộ trình demo tiếp theo cùng các phụ thuộc, không gắn lịch hoặc hạn thực hiện.

**Trạng thái của tài liệu này:** kế hoạch làm việc đã thống nhất về định hướng; chưa phải báo cáo khảo sát hoàn chỉnh, protocol đã kiểm chứng hoặc kết quả benchmark của nhóm.
