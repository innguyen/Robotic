# Khảo sát phương pháp xây dựng benchmark cho robot tìm kiếm con người trong môi trường thiên tai

## Tóm tắt

Robot tìm kiếm con người trong môi trường thiên tai cần được đánh giá từ lúc bắt đầu khám phá đến khi báo cáo được vị trí người cần tìm. Chỉ đo độ chính xác nhận diện hoặc quãng đường di chuyển chưa phản ánh đủ nhiệm vụ này. Báo cáo khảo sát sáu công trình và protocol Habitat ObjectNav; từ đó đề xuất benchmark gồm ba tầng: tìm kiếm và điều hướng trong mô phỏng, nhận diện trên dữ liệu đã ghi, và đánh giá toàn hệ thống. Nội dung tập trung vào cách tổ chức thực nghiệm; chưa có kết quả đánh giá nghiên cứu của nhóm.

Trạng thái triển khai T1 được cập nhật riêng trong [P1b.2 audit](benchmark-p1b2-audit.md) và [P2.1 audit](benchmark-p21-audit.md): đã có harness offline và selector có version, cùng tests/dev controls. Các log nghiệm thu này không phải bảng so sánh B0/B1 hoặc bằng chứng hoạt động trong thiên tai thật. Phần khảo sát và thiết kế phương pháp bên dưới giữ nguyên phạm vi đó.

## 1. Bài toán và mục tiêu đánh giá

Nhiệm vụ của robot là tự di chuyển trong khu vực chưa biết đầy đủ để tìm kiếm con người. Những đặc trưng cần xét gồm vật cản che khuất, lối đi bị chặn, nhiều nhánh tìm kiếm, điều kiện quan sát suy giảm và địa hình hạn chế khả năng di chuyển. Mỗi thí nghiệm phải nêu rõ đặc trưng nào được tái hiện và bằng cách nào.

Chuỗi nhiệm vụ được xét gồm: quan sát, chọn nơi tìm tiếp, di chuyển, xác nhận người, xác định vị trí, báo cáo và dừng. Cần phân biệt bốn kết quả: robot đã nhìn thấy người; đã xác nhận mục tiêu; đã báo cáo đúng vị trí; đã hoàn thành nhiệm vụ an toàn. Một robot có thể đạt kết quả đầu nhưng chưa đạt kết quả cuối.

Benchmark được hiểu là bộ tình huống cùng quy tắc chạy và chấm điểm thống nhất. Một **episode** là một lượt thử có điều kiện bắt đầu và kết thúc xác định. **Ground truth (GT)** là thông tin tham chiếu dùng để chấm, như vị trí người và hình học vật cản. Thuật toán không được biết trước GT mục tiêu chỉ vì bộ chấm có thông tin đó.

Ba câu hỏi đánh giá chính là:

1. Chiến lược tìm kiếm hiện tại có cải thiện khả năng tìm và báo cáo đúng người so với một phương pháp đơn giản không?
2. Che khuất và đường bị chặn ảnh hưởng như thế nào đến kết quả, quãng đường và nguyên nhân thất bại?
3. Kết quả thay đổi ra sao khi thay cảm biến lý tưởng bằng nhận diện thực tế và khi ghép các thành phần thành hệ thống hoàn chỉnh?

Phạm vi đầu tiên là người không di chuyển trong episode và có thể được quan sát từ ít nhất một vị trí hợp lệ. Điều này không giới hạn người ở tư thế đứng. Benchmark tìm người chưa chấm phân loại tư thế hoặc phát hiện sự kiện ngã. Tìm người bị vùi kín cần cảm biến và phương pháp đánh giá riêng; nhãn `person` không đủ để kết luận tình trạng chấn thương.

## 2. Phạm vi khảo sát

Sáu bài chính được chọn theo vai trò: phân tích benchmark ở cấp nhiệm vụ, triển khai hệ thống cứu hộ, tìm kiếm bằng thị giác, thiết kế môi trường điều hướng, so sánh cảm biến và xây dựng dữ liệu. Habitat ObjectNav 2023 bổ sung một protocol chuẩn hóa episode. Đây là khảo sát có chọn lọc, không phải tổng quan hệ thống toàn bộ lĩnh vực.

Các nhóm từ khóa dùng để tìm và đối chiếu nguồn gồm `robot search rescue benchmark victim detection evaluation protocol`, `active visual search benchmark success SPL ZAVIS`, `ground robot disaster response dataset evaluation UMA SAR` và `navigation benchmark environment difficulty BARN MRPB`. Ưu tiên bản gốc từ nhà xuất bản, arXiv, website tác giả và trang dataset/challenge chính thức; các bài mới được tìm tiếp qua tài liệu tham khảo. Không đặt bộ lọc năm cứng; sáu bài chính được chọn có năm công bố 2020–2024.

Tiêu chí đưa vào là có mô tả phép thử hoặc phương pháp thu dữ liệu phục vụ đánh giá, có phần áp dụng được cho tìm người hay một thành phần của nhiệm vụ, và truy được bằng chứng về điều kiện thử. Loại khỏi phân tích chính các bài chỉ giới thiệu ý tưởng, nguồn phổ thông thiếu protocol và nghiên cứu không làm rõ được quan hệ với năng lực cần chấm. Luật challenge và phương pháp thử được ghi riêng với paper. Nguồn chưa xác minh đủ được ghi như vậy, không tự chuyển thành kết luận “paper không công bố”.

Mỗi nguồn được đọc theo cùng các câu hỏi: nhiệm vụ là gì; robot được biết gì; tổ chức lượt thử và chấm điểm ra sao; so với phương pháp nào; xử lý thất bại thế nào; và kết luận có giới hạn gì. Bài cung cấp dataset được đánh giá về cách thu dữ liệu, không được xem như đã chứng minh thuật toán tìm kiếm thành công. Trong các phần sau, SR là tỷ lệ thành công; SPL kết hợp thành công với hiệu quả quãng đường; IoU đo mức chồng lắp giữa hai vùng ảnh.

## 3. Phương pháp thực nghiệm trong các công trình

### 3.1. Into the Robotic Depths và DARPA SubT

Chung và cộng sự phân tích kết quả SubT từ dữ liệu ban tổ chức và dữ liệu các đội gửi về. Ngoài điểm nhiệm vụ, bài xét độ phủ và sai lệch bản đồ so với GT. Table 2 phản ánh thành tích tốt nhất đạt được trong 24 lượt mô phỏng mỗi đội, không phải kết quả trung bình. Cách tổng hợp phải được đọc cùng con số. [1, §1, §3.3–3.4](https://doi.org/10.1146/annurev-control-062722-100728).

SubT chấm báo cáo đúng loại artifact và vị trí trong phạm vi 5 m; số báo cáo bị giới hạn để hạn chế báo sai. Artifact có cả hình nộm đại diện người sống sót. Đây là đánh giá tìm–phát hiện–định vị–báo cáo, khác với chỉ nhận diện trong ảnh. [7, mục Scoring](https://www.darpa.mil/research/programs/darpa-subterranean-challenge).

Bài phân biệt khám phá bản đồ với tìm mục tiêu: diện tích đã đi qua chưa bảo đảm các mục tiêu được phát hiện và báo cáo. Nhóm nên đo kết quả nhiệm vụ riêng với coverage, đồng thời chọn lại ngưỡng định vị theo quy mô thử. [1, §4.2, §4.4](https://doi.org/10.1146/annurev-control-062722-100728).

### 3.2. CERBERUS

CERBERUS kết hợp robot chân và robot bay với người giám sát. Bài ghi hai lượt sơ loại và lượt Prize Round. Lượt chấm chính có 60 phút, 40 artifact, tối đa 45 báo cáo; các đội dùng cùng sân. [2, §4.2](https://arxiv.org/html/2207.04914v1).

Timeline phân biệt tự hành, được chỉ dẫn, mất liên lạc và ngừng hoạt động. Bài đối chiếu vị trí báo cáo với GT và phân tích lỗi mắc kẹt, giúp giải thích thất bại bên cạnh điểm tổng. [2, Figure 24, Table 3 và §4.2.3](https://arxiv.org/html/2207.04914v1).

Người giám sát duyệt báo cáo và có thể can thiệp. Kết quả vì thế thuộc chế độ tự hành có giám sát. Điểm thi không thay thế so sánh có kiểm soát giữa hai thuật toán. [2, §3.6 và §4.2.2](https://arxiv.org/html/2207.04914v1).

### 3.3. ZAVIS

ZAVIS tách kiểm tra detector khỏi tìm kiếm. Mô phỏng dùng 60 cảnh iTHOR và 15 cảnh RoboTHOR. Success xảy ra khi ảnh ứng viên chứa mục tiêu GT, với IoU > 0,3 hoặc IoA > 0,5; đi quá 50 m chưa tìm được là thất bại. Tổng episode không được nêu rõ tại §V-B. [3, §V-A–V-B, Table II](https://arxiv.org/html/2209.08803v3).

Tác giả so sánh SR/SPL với CoW và COS-POMDP đã điều chỉnh; ablation xét detector, tri thức đồng xuất hiện và độ bất định. Thử robot thật được trình bày định tính. [3, Tables III–IV, §V-C](https://arxiv.org/html/2209.08803v3).

Nhóm nên kế thừa cách tách thành phần, nhưng bổ sung báo cáo tọa độ và dừng an toàn vào tiêu chí hoàn thành.

### 3.4. BARN

BARN sinh 300 môi trường từ 12 bộ tham số cellular automaton, dùng năm thước đo hình học để mô tả độ khó. [4, §III và §IV-A](https://arxiv.org/html/2008.13315v3).

Trong Gazebo, DWA và E-Band cùng điều khiển Jackal, được đồng bộ giới hạn vận tốc và nhận bản đồ trước. Mỗi planner chạy năm lượt trên mỗi môi trường, tổng cộng 3.000 lượt. Lượt không tới đích chịu penalty 30 giây trong cách tính của bài. Phần thực tế dùng năm môi trường mới, tổng 50 lượt, không cung cấp bản đồ trước. [4, §IV-A.1 và §IV-B](https://arxiv.org/html/2008.13315v3).

Bài học là kiểm soát điều kiện so sánh và tạo môi trường có tham số. BARN đánh giá đi tới đích, chưa đánh giá tìm người. Nhóm cần bổ sung mục tiêu tìm kiếm và không mặc định cấp bản đồ GT cho robot.

### 3.5. So sánh Thermal, Multispectral và RGB trong SAR

Cruz Ulloa và cộng sự dùng robot bốn chân mang ba loại camera. Robot thu ảnh tại điểm người vận hành chỉ định. Có 20 lượt tại sân thử kiểm soát với năm nạn nhân mô phỏng và một lượt qua đường hầm; hai điều kiện có quy mô lặp khác nhau. [5, §3.1–3.2 và §4.1](https://doi.org/10.3390/app14020766).

Bài phân tích điều kiện quan sát và xây điểm tổng có trọng số. Algorithm 2 gọi tỷ lệ số nhận diện đúng trên tổng GT là “detection precision”, nhưng mẫu số này tương ứng cách tính recall. [5, Algorithm 2, §3.3.3](https://doi.org/10.3390/app14020766).

Nhóm nên kế thừa cách phân tầng điều kiện, tự lập luận trọng số và đánh giá riêng khả năng chọn nơi tìm kiếm.

### 3.6. UMA-SAR

UMA-SAR thu bằng xe có người điều khiển trong diễn tập, với RGB, ảnh nhiệt, LiDAR 3D, IMU và GPS. Có bốn chuỗi diễn tập (5,2 km, 77 phút) và ba chuỗi hiện trường trống. [6, Abstract](https://doi.org/10.1177/02783649211004959).

Dữ liệu có định dạng đọc trực tiếp, rosbag, công cụ trích xuất và thông tin calibration. Điều này giúp nghiên cứu offline khi chưa có robot. Tuy nhiên, GPS tham chiếu chuyển động xe không tự cung cấp GT vị trí hoặc bounding box của người. [10, mục Datasets, Software Tools và RGB Camera Calibration](https://www.uma.es/robotics-and-mechatronics/info/124594/sar-datasets/).

Điểm nhóm có thể kế thừa là thu dữ liệu cùng timestamp, lưu calibration và giữ cả điều kiện có hoạt động lẫn hiện trường trống. Dataset là đầu vào để xây phép thử; muốn dùng chấm detector cần kiểm tra, bổ sung annotation và chia tập phù hợp.

### 3.7. Habitat ObjectNav 2023

ObjectNav tìm bất kỳ instance thuộc lớp mục tiêu, không có bản đồ trước, dùng RGB-D và GPS/Compass lý tưởng. HM3D-Semantics v0.2 có 216 scene, chia train/val/test thành 145/36/35. [12, Task và Dataset](https://aihabitat.org/challenge/2023/).

Participation Guidelines ghi mỗi agent được đánh giá trên **1.000 episode**. Đây là số lượt đánh giá được trang challenge nêu, khác với số scene; không suy ra train, val hay minival đều có 1.000 episode. [12, Participation Guidelines](https://aihabitat.org/challenge/2023/).

Success cần gọi STOP trong phạm vi Euclidean 1 m và oracle có thể thấy mục tiêu bằng cách xoay hoặc thay góc camera tại chỗ. Agent không bắt buộc đang nhìn mục tiêu; tiêu chí này chưa chấm detector thực. [12, Evaluation](https://github.com/facebookresearch/habitat-challenge/blob/main/README.md#evaluation).

Nhóm nên kế thừa việc khóa scene split và quy tắc kết thúc. Đây là protocol bổ trợ cho thiết kế episode, không phải bằng chứng robot đã tìm người trong thiên tai.

## 4. Tổng hợp và bài học cho benchmark của nhóm

### 4.1. Thiết kế thực nghiệm

| Nguồn              | Task và chế độ thử                       | Môi trường và đầu vào                  | Ground truth                                  | Quy mô được xác nhận                                |
| ------------------ | ---------------------------------------- | -------------------------------------- | --------------------------------------------- | --------------------------------------------------- |
| SubT [1, 7]        | Tìm và báo artifact; Systems/Virtual     | Địa hình ngầm, cảm biến từng hệ thống  | Artifact và bản đồ tham chiếu của ban tổ chức | Table 2: 24 lượt/đội Virtual                        |
| CERBERUS [2]       | Hệ thống có giám sát; robot thật         | Sân SubT, nhiều nền tảng               | Vị trí artifact do DARPA cung cấp sau sự kiện | Hai sơ loại, một Prize Round                        |
| ZAVIS [3]          | Visual search; mô phỏng và demonstration | iTHOR/RoboTHOR, quan sát thị giác      | Bbox mục tiêu                                 | Số scene ở §3.3; không nêu tổng episode tại §V-B    |
| BARN [4]           | Đi tới đích; mô phỏng/thật               | Jackal; có bản đồ trước ở mô phỏng     | Môi trường, start/goal đã định nghĩa          | 3.000 lượt mô phỏng, 50 lượt thật                   |
| Ba loại camera [5] | Nhận diện theo điểm kiểm tra             | RGB, nhiệt, multispectral              | Nhãn người/vùng ảnh                           | 20 lượt sân thử, một lượt hầm                       |
| UMA-SAR [6, 10]    | Thu dữ liệu; không chấm mission          | Xe có người điều khiển, nhiều cảm biến | GPS tham chiếu xe                             | Bảy chuỗi dữ liệu, không phải bảy episode tìm người |
| ObjectNav [12]     | Navigation tới lớp đối tượng; mô phỏng   | HM3D-Semantics; RGB-D, pose lý tưởng   | Object/scene annotations và oracle visibility | 216 scene; 1.000 episode đánh giá/agent theo [12, Participation Guidelines](https://aihabitat.org/challenge/2023/) |

### 4.2. Chấm điểm và điều kiện so sánh

| Nguồn              | Success và metrics                        | Baseline/đối tượng so sánh               | Ablation                               | Giới hạn khi áp dụng                               |
| ------------------ | ----------------------------------------- | ---------------------------------------- | -------------------------------------- | -------------------------------------------------- |
| SubT [1, 7]        | Báo đúng loại/vị trí; điểm và map metrics | Các hệ thống thi đấu khác nhau           | Không phải thiết kế cô lập từng module | Không quy hết khác biệt cho search policy          |
| CERBERUS [2]       | Artifact score và timeline                | Các lượt triển khai                      | Lượt thi chính không phải ablation     | Có người duyệt báo cáo                             |
| ZAVIS [3]          | Candidate ảnh đúng; SR/SPL                | CoW, COS-POMDP điều chỉnh                | Detector, prior, uncertainty           | Chưa chấm dừng an toàn theo task nhóm              |
| BARN [4]           | Tới đích; thời gian chuẩn hóa             | DWA/E-Band, giữ giới hạn vận tốc         | So sánh baseline, không phải ablation | Không có perception tìm người                      |
| Ba loại camera [5] | Nhận diện và điểm có trọng số             | Ba mode cảm biến, kết hợp mode           | So sánh modality/fusion; chưa xác minh ablation cô lập thành phần | Điểm quan sát do người chỉ định |
| UMA-SAR [6, 10]    | Không áp dụng mission success             | Không áp dụng baseline tìm kiếm          | Không áp dụng                          | Cần annotation/protocol để chấm thuật toán         |
| ObjectNav [12]     | STOP, khoảng cách, visibility; SR/SPL     | Khung chung cho agent; có starter random | Protocol không phải bài ablation       | Không trực tiếp chấm lớp `person` hay detector SAR |

“Không áp dụng” khác với “chưa xác minh”; chỉ ghi “không công bố” khi đã kiểm tra đúng phần nguồn. Các số lượt có mục đích khác nhau, không dùng để so sánh quy mô một cách máy móc.

Baseline comparison đối chiếu các phương pháp hoàn chỉnh trong điều kiện chung. Ablation bỏ hoặc đổi một thành phần của cùng phương pháp để kiểm tra đóng góp của nó. Trong bảng này, ZAVIS có ablation thành phần đã xác minh; so sánh planner của BARN không được tính là ablation. [3, Tables III–IV](https://arxiv.org/html/2209.08803v3), [4, §IV-A](https://arxiv.org/html/2008.13315v3).

Không có một tỷ lệ thành công chung để xếp hạng các nguồn. Task, cảm biến, bản đồ và mức can thiệp của con người khác nhau. Into the Robotic Depths và CERBERUS còn phân tích cùng sự kiện SubT, không phải hai bộ kiểm chứng độc lập.

Từ khảo sát, benchmark của nhóm cần giải quyết bốn việc. Thứ nhất, chấm kết quả tìm người bằng báo cáo có vị trí kiểm chứng được. Thứ hai, giữ GT tách khỏi thuật toán để tránh biết trước mục tiêu. Thứ ba, dùng cùng episode và cấu hình chung giữa các phương pháp. Thứ tư, giữ lại các lượt thất bại để biết hệ thống yếu ở tìm kiếm, nhận diện hay điều hướng.

Các phương pháp thử của NIST bổ sung cách định lượng từng năng lực robot ứng phó khẩn cấp. RoboCupRescue là nguồn tham khảo cho tổ chức bài thử; khi sử dụng phải khóa đúng phiên bản luật. Việc tham khảo hai nguồn này không đồng nghĩa benchmark của nhóm đã đạt một tiêu chuẩn hoặc tuân thủ đầy đủ luật thi đấu. [8](https://www.nist.gov/el/intelligent-systems-division-73500/standard-test-methods-response-robots), [9](https://rrl.robocup.org/rules/).

### 4.3. Từ bằng chứng tới lựa chọn của nhóm

| Quyết định thiết kế                  | Bằng chứng tham khảo                | Điều chỉnh cho nhóm                                             |
| ------------------------------------ | ----------------------------------- | --------------------------------------------------------------- |
| Chấm báo cáo độc lập với coverage    | SubT [1, 7]                         | Tách ReportSuccess và SafeMissionSuccess                        |
| Thử ngưỡng định vị 0,5 m             | SubT có ngưỡng 5 m [7]              | 0,5 m là đề xuất riêng; kiểm tra bằng pilot và độ chính xác GT  |
| Episode có kết thúc do agent yêu cầu | STOP của ObjectNav [12]             | Khóa báo cáo rồi xác nhận dừng, vẫn chấm va chạm trong bước này |
| Chỉ đổi chiến lược chọn nơi tìm      | Cách kiểm soát điều kiện ở BARN [4] | B0/B1 dùng chung sensing, navigation và monitor                 |
| Tách phép thử thành phần             | ZAVIS [3], so sánh camera [5]       | T1 oracle, T2 dữ liệu có nhãn, T3 toàn hệ thống                 |
| Lưu dữ liệu đủ để chấm lại           | UMA-SAR [6, 10]                     | Lưu calibration, timestamp, annotation và log kết quả           |

## 5. Benchmark đề xuất cho robot của nhóm

### 5.1. Ba tầng đánh giá

| Tầng                        | Phép thử                                                | Câu hỏi được trả lời                                               |
| --------------------------- | ------------------------------------------------------- | ------------------------------------------------------------------ |
| T1 — Tìm kiếm và điều hướng | Môi trường hình học, cảm biến mô phỏng, detector oracle | Robot có chọn nơi quan sát và tìm đường tới mục tiêu tốt không?    |
| T2 — Perception offline     | Ảnh/video/RGB-D có annotation độc lập                   | Robot nhận diện và định vị người tốt đến đâu trong từng điều kiện? |
| T3 — Toàn hệ thống          | Robot cùng cảm biến thực và GT đo độc lập               | Các thành phần có phối hợp để hoàn thành nhiệm vụ không?           |

Detector **oracle** ở T1 chỉ trả mục tiêu theo quy tắc quan sát đã định nghĩa, không chạy YOLO. T1 phù hợp kiểm tra chiến lược tìm kiếm, nhưng không đo độ chính xác nhận diện thực. Kết quả cảnh trống với oracle không dùng để kết luận tỷ lệ báo nhầm của YOLO. Trong T2, video cố định cũng không cho biết robot sẽ chọn hướng di chuyển nào.

Trước mắt, T1 tái hiện vật cản, che khuất và đường bị chặn bằng hình học phẳng. Ánh sáng yếu, che khuất từng phần cơ thể và nhiễu RGB-D được xét ở T2 khi có dữ liệu. Nền không ổn định, trượt bánh và khả năng vượt địa hình cần mô hình hoặc thử nghiệm tương ứng. Kết quả được ghi theo đặc trưng đã kiểm tra, không gộp thành tuyên bố chung rằng robot hoạt động được trong mọi môi trường thiên tai.

### 5.2. Quy tắc của một episode

Mỗi lượt bắt đầu với một robot đứng yên tại pose xác định. Robot nhận mục tiêu `person`, sử dụng cảm biến và bản đồ tự xây để tìm kiếm. Phiên bản đầu có tối đa một người không di chuyển. Hai phương pháp cùng biết giới hạn không gian nhiệm vụ; đây không phải bản đồ vật cản. Thuật toán không được nhận trước vị trí người, bản đồ GT hoặc nhãn chấm của episode.

Bộ dữ liệu mô tả episode (fixture) lưu ID, bố trí môi trường (layout), pose ban đầu, hình học vật cản, vị trí người, mô hình cảm biến, giới hạn nhiệm vụ và tập phát triển/test. Evaluator suy ra vùng tiếp cận từ GT/cấu hình và chứng nhận tính khả thi trước khi chạy; không cấp vùng này cho policy. Cấu hình chạy lưu thêm seed, phiên bản code và hash của fixture. Pose hoàn hảo và động học Euler holonomic của T1 phải được công bố như các giả định.

Phép thử bắt đầu tính thời gian nhiệm vụ từ 0. Với T1, dùng bước mô phỏng cố định; thời gian máy chạy được ghi riêng. Bộ chấm (evaluator) quản lý GT, không đưa đối tượng môi trường chứa GT vào giao diện của chiến lược tìm kiếm.

### 5.3. Tình huống và độ khó

| ID  | Tình huống                                        | Nội dung kiểm tra                            |
| --- | ------------------------------------------------- | -------------------------------------------- |
| S1  | Người dễ quan sát                                 | Kiểm tra chuỗi xác nhận, tiếp cận và báo cáo |
| S2  | Người bị vật cản che, có vị trí quan sát thay thế | Khả năng đổi góc nhìn                        |
| S3  | Người ở nhánh phải khám phá mới thấy              | Lựa chọn khu vực tìm kiếm                    |
| S4  | Đường trực tiếp bị chặn, có tuyến đi vòng         | Phối hợp tìm kiếm và điều hướng              |
| S5  | Không có người                                    | Báo sai và cách kết thúc tìm kiếm            |

S1 là tình huống kiểm tra cơ bản, không tự đại diện cho một nhiệm vụ cứu hộ. Với S2, mô phỏng 2D chỉ xác định bị che hoặc thấy theo hình học; tỷ lệ cơ thể nhìn thấy trong ảnh phải được đánh giá bằng dữ liệu ảnh.

Độ khó được mô tả bằng số nhánh, chiều rộng lối đi, đường tới vùng thấy mục tiêu, số điểm cần đổi hướng quan sát và mật độ vật cản. Trước khi chấm, kiểm tra pose không chồng vật cản và tồn tại đường tới vùng hoàn thành hợp lệ theo hình chiếu thân robot (footprint). Tình huống cố ý bất khả thi được đánh dấu thành phép thử riêng.

Nhãn scenario phải được xác minh bằng hình học: S1 thấy từ yaw xuất phát; S2 bị che thật và có điểm nhìn gần đi tới được; S3 có các nhánh tách biệt và phải vượt cửa nhánh mới thấy; S4 mọi đường thẳng tới vùng tiếp cận đều bị chặn nhưng có đường vòng; S5 thật sự không có target. Đường chứng phải khả thi với footprint, động học và budget. Không chứng minh được thì loại trước khi chạy, không tính thành thất bại của thuật toán. Điều kiện và đối chứng cụ thể nằm ở [đặc tả T1, §3.4](benchmark-t1-spec.md#34-xác-minh-s1s5).

Một quy mô khởi đầu có thể là năm nhóm tình huống × bốn layout test mỗi nhóm × năm cấu hình xuất phát/mục tiêu, tức 100 episode cho mỗi phương pháp. Đây là cỡ thử đề xuất, chưa có cơ sở khẳng định đủ sức mạnh thống kê. Pilot dùng layout phát triển riêng để chốt quy mô trước khi khóa bộ test. `environment_id` phải khớp `layout_hash` tính từ bounds và hợp obstacle; cùng hình học đổi tên vẫn bị phát hiện. Các biến thể cùng family luôn nằm cùng split. Hash không tự nhận diện mọi layout gần giống; cần kiểm tra nguồn sinh theo [đặc tả T1, §3.3](benchmark-t1-spec.md#33-nhận-diện-layout-và-chống-rò-split).

### 5.4. Baseline và ablation

Baseline B0 `fixed-order-v2` quét tại chỗ rồi xét lưới cố định neo theo pose xuất phát, theo vòng từ gần ra xa và góc. Chỉ chọn điểm hợp lệ trên bản đồ đã quan sát; điểm chưa biết được xét lại. Nếu lưới chưa có điểm phù hợp, B0 chọn ô khám phá dự phòng theo thứ tự cố định từ miền ứng viên chung với B1, không dùng gain score. Khi hết goal, cả hai giữ trạng thái bế tắc, dừng an toàn và tiếp tục sensing tới budget, không kết thúc sớm như đã tìm hết. Quy tắc được khóa trong [đặc tả T1](benchmark-t1-spec.md#5-baseline-và-phạm-vi-thay-đổi).

B1 dùng chiến lược hiện tại của `SearchMission`. Cả hai dùng cùng detector, quy trình xác nhận và tiếp cận, navigation, collision monitor, giới hạn vận tốc và budget. Mỗi phương pháp chạy cùng danh sách episode. Khác biệt cần đánh giá trước tiên là cách chọn điểm khám phá.

Cả hai có cùng pose ban đầu, mô hình đo pose/quan sát và động học; mỗi policy có trạng thái riêng. Action khác nhau tạo quỹ đạo và observation khác nhau, không ép pose B0 bằng pose B1. Trước khi khóa test, B0 phải qua positive controls S1–S4 trên dev. Nếu B0 không hoạt động do lỗi selector/adapter hoặc budget chưa phù hợp, chỉ báo chẩn đoán, chưa kết luận B1 tốt hơn. Lưu goal, điểm dự phòng và thời gian bế tắc; không bỏ những lượt B0 thất bại.

Sau so sánh chính, có thể làm ablation bộ nhớ vùng đã quan sát hoặc thành phần khoảng cách trong điểm chọn mục tiêu. Mỗi lần chỉ đổi một yếu tố. B0 và các ablation chỉ được xem là đã hỗ trợ khi thực sự có mã và kết quả chạy.

### 5.5. Báo cáo mục tiêu và điều kiện kết thúc

Một báo cáo chính thức chứa nhãn, vị trí trong frame đã quy định, timestamp và tham chiếu tới quan sát xác nhận. Mỗi episode chỉ được nộp một báo cáo. Sau khi nộp, nội dung bị khóa và hệ thống chuyển sang xác nhận dừng; episode chỉ kết thúc sau bước này hoặc khi có timeout/lỗi/va chạm. Cách này giữ được trường hợp báo đúng nhưng dừng không an toàn. Detection trước đó vẫn được lưu riêng, không phải các lần thử báo cáo.

Báo cáo hợp lệ khi class đúng và sai số vị trí không vượt ngưỡng r. Có thể thử r = 0,5 m cho bộ mô phỏng đầu tiên; ngưỡng này là lựa chọn của nhóm, cần chốt bằng pilot và độ chính xác GT. Nó khác với khoảng cách robot phải giữ với người khi tiếp cận.

Hoàn thành nhiệm vụ an toàn còn yêu cầu đến vùng tiếp cận hợp lệ, raw-visible tại pose cuối và dừng trước khi hết budget, không collision. Vùng này xét footprint, khoảng cách tới box người, hướng quan sát và toàn thân nằm trong bounds; bounds là miền nhiệm vụ, không tự sinh tường cảm biến. T1 yêu cầu ba bước 0,1 giây liên tiếp có tốc độ tịnh tiến và quay không vượt 0,01 m/s và 0,01 rad/s; vẫn kiểm tra collision trong khoảng đó. Các giá trị này là quy ước mô phỏng, không phải ngưỡng đã hiệu chuẩn cho robot thật. Chi tiết nằm trong [đặc tả T1](benchmark-t1-spec.md).

| Kết thúc                                                      | Cách chấm                                                                         |
| ------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| Báo cáo đúng, đủ điều kiện tiếp cận và dừng an toàn           | Mission success                                                                   |
| Báo cáo đúng nhưng thiếu điều kiện tiếp cận hoặc dừng an toàn | Report success, mission failure                                                   |
| Báo cáo sai hoặc báo cáo trong cảnh không người               | False report; mission failure                                                     |
| Hết budget trong cảnh có người                               | Mission failure; lưu riêng thời gian bế tắc khi không có goal                     |
| Hết budget trong cảnh không người                            | Ghi timeout/bế tắc; không cộng vào success rate cảnh có người                     |
| Collision                                                     | Mission failure, kể cả nếu cùng bước có báo cáo đúng                              |
| Crash hoặc planner timeout                                    | Lượt lỗi, vẫn nằm trong kết quả của episode đã bắt đầu                            |

Monitor chặn lệnh được ghi như một sự kiện can thiệp; một lần chặn không tự làm nhiệm vụ thất bại. Trạng thái hết tìm kiếm cũng không được diễn giải thành chứng minh toàn môi trường không có người.

### 5.6. Metric và cách tổng hợp

Với N+ episode có người, đặt C_i = 1 khi báo cáo hợp lệ; M_i = 1 khi hoàn thành đầy đủ điều kiện an toàn. Hai tỷ lệ chính là:

$$
\mathrm{ReportSuccess}=\frac{\sum_i C_i}{N_+},\qquad
\mathrm{SafeMissionSuccess}=\frac{\sum_i M_i}{N_+}.
$$

Nếu `N+ = 0`, hai tỷ lệ là `null`/không áp dụng, không phải 0%. Tương tự, tỷ lệ báo sai trên cảnh trống chỉ tính khi có episode không người. Một lượt crash vẫn thuộc mẫu số nếu episode đã bắt đầu.

| Metric bổ sung                           | Cách đo và giới hạn                                                                                                     |
| ---------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| Thời gian phát hiện, xác nhận và báo cáo | Ba mốc riêng từ lúc bắt đầu; chưa có sự kiện thì để trống, không ghi 0                                                  |
| Quãng đường                              | Tổng độ dịch chuyển liên tiếp của pose GT trong T1; không dùng đường ngắn như bằng chứng hiệu quả nếu nhiệm vụ thất bại |
| Collision và clearance                   | Chấm bằng footprint với vật cản/người, xét cả chuyển động giữa hai bước; độc lập với boolean của monitor                |
| False report rate trên cảnh trống        | Số episode không người có báo cáo sai / tổng episode không người                                                        |
| Sai số vị trí                            | Khoảng cách giữa báo cáo và GT trong cùng frame; T1 oracle không phản ánh sai số camera                                 |
| Can thiệp và lỗi                         | Số đợt monitor chặn, recovery, stuck, planner timeout và can thiệp của người                                            |
| Coverage                                 | Tách vùng ước lượng đã nhìn khỏi coverage của evaluator độc lập; không dùng coverage thay cho success                   |

Precision và recall của detector tại T2 dùng định nghĩa TP/(TP+FP) và TP/(TP+FN), cùng quy tắc matching bbox, IoU và confidence được công bố. Annotation cần kiểm tra độc lập; không dùng detection của YOLO làm GT. Tập dữ liệu phải tách theo cảnh hoặc phiên ghi, tránh đưa các frame gần nhau vào cả tập phát triển và test.

Chưa đưa SPL vào bộ metric chính. Muốn dùng SPL, đường tham chiếu phải tới vùng có thể hoàn thành task, xét footprint và vật cản; đường thẳng tới tâm người không đủ làm tham chiếu trong trường hợp đi vòng. [11](https://arxiv.org/abs/1807.06757).

Mỗi bảng kết quả phải có số layout, số episode, số thành công, số timeout và số lượt lỗi. Báo kết quả theo S1–S5 trước khi tổng hợp. Thời gian và quãng đường chỉ trên các lượt thành công phải được gắn nhãn; bổ sung tỷ lệ hoàn thành theo budget để giữ cả các lượt thất bại trong phép so sánh.

Khi ước lượng khoảng tin cậy 95% cho chênh lệch giữa B0 và B1 bằng bootstrap, lấy mẫu lại theo layout family và giữ cặp kết quả cùng episode. Nếu mỗi family chỉ có một layout, đơn vị này trùng layout. Nếu số cụm quá ít để suy luận ổn định, ưu tiên số đếm và từng kết quả. Đổi seed chỉ tạo mẫu mới khi thực sự thay đổi biến đã định nghĩa; lặp nguyên mô phỏng xác định chủ yếu kiểm tra khả năng tái lập.

## 6. Khả năng triển khai trên hệ thống hiện tại

Source được đối chiếu tại commit `b7025fd182399519c097a5286ffeb64991c02439`, trong thư mục `test version/` của repository `innguyen/Robotic`.

| Thành phần                                                       | Hiện trạng liên quan tới benchmark                                                                         |
| ---------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| [SearchMission](../test%20version/search_core_ros1.py)           | Chọn điểm theo vùng chưa quan sát và khoảng cách; `found` được đặt sau khi navigation trả `ARRIVED`        |
| [Navigation core](../test%20version/navigation_core_ros1.py)     | Có costmap cục bộ và planner; cần kiểm soát thời điểm nhận kết quả planner bất đồng bộ khi kiểm tra replay |
| [Simulator](../test%20version/simulate_navigation.py)            | Có headless; xác nhận người bằng quy tắc hình học, trả tọa độ mô phỏng; không chạy detector thực           |
| [Collision monitor](../test%20version/collision_monitor_ros1.py) | Có thể dùng chung giữa B0/B1; vẫn cần bộ chấm collision độc lập                                            |
| [Vision](../test%20version/jetauto_find_object.py)               | Có pipeline YOLO và tracking; cần luồng đọc dữ liệu ghi sẵn cho T2                                         |

Một chi tiết cần thống nhất là tham số `STANDOFF['person'] = 0.55`. Navigator cộng thêm nửa chiều dài thân xe khi tính khoảng dừng tới mục tiêu, rồi xét tolerance. Vì vậy không nên mô tả 0,55 m là khoảng cách từ tâm robot tới người. Quy tắc chấm vùng tiếp cận phải theo đúng hình học và cấu hình đã dùng.

Simulator hiện xác nhận mục tiêu sau hai frame thỏa điều kiện quan sát và nhớ vị trí trong 2,5 giây. Các metric về raw detection, confirmation và report cần được ghi riêng. Coverage trong `SearchMission` là ước lượng từ sensing, không phải GT.

CLI của simulator production trong ảnh chụp source trên dừng sau tối đa 600 bước, tương đương 60 giây mô phỏng. Harness benchmark riêng hiện lấy budget từ fixture và không sửa CLI production. Việc simulator giữ tọa độ người để sinh quan sát là bình thường; rò GT chỉ xảy ra nếu giao diện policy cho phép truy cập dữ liệu chấm.

Các nhận xét trên dựa trên source production tại commit đã nêu. Kết quả regression và các bước triển khai harness được cập nhật theo audit P0–P2.1; chưa có đánh giá B0/B1 trên test đã khóa. Nhóm hiện chưa sử dụng được JetAuto, nên hướng thực nghiệm vẫn là T1, sau đó T2 trên dữ liệu có nhãn.

## 7. Triển khai và khả năng tái lập

Trước hết cần xác nhận môi trường Python, dependencies và regression test hiện có. Tiếp theo tách scene thành fixture, bổ sung evaluator độc lập, bộ chạy không giao diện (headless), B0 và log sự kiện. Pilot dùng để phát hiện fixture sai, metric sai hoặc ảnh hưởng của planner bất đồng bộ trước khi chạy bộ test đã khóa.

Mỗi lần chạy lưu phiên bản code, cấu hình thực sự được nạp, hash fixture, seed, chế độ perception/planner và quy ước đơn vị. Dữ liệu gồm một dòng kết quả mỗi episode, timeline sự kiện và quỹ đạo. Bộ tổng hợp phải tính lại được bảng báo cáo từ log, giữ cả các lượt lỗi và lịch sử chạy lại. Thời gian mô phỏng dùng chấm hành vi; thời gian thực thi phản ánh chi phí tính toán.

[Đặc tả T1 phiên bản 0.2](benchmark-t1-spec.md) chốt fixture, scenario certificates, layout hash/split, frame, B0/B1, matching, thứ tự sự kiện, log và acceptance tests. Validator hình học, single runner và batch/resume đã triển khai theo các giới hạn trong audit. [P2.1](benchmark-p21-audit.md) ghi bằng chứng cho selector/dev controls; [P2.2](benchmark-p22-plan.md) còn phải nghiệm thu metrics và paired pilot trước khi mở test đã khóa.

Khi có robot, T3 cần GT vị trí được đo độc lập, đồng bộ frame và timestamp, calibration cảm biến và quy trình giám sát thử nghiệm. Giữ nguyên collision monitor, watchdog và giới hạn chuyển động. Điều kiện sân thử phải được mô tả đủ để tái hiện, đồng thời phù hợp khả năng phần cứng.

## 8. Kết luận

Các nguồn khảo sát cho thấy benchmark tìm người cần gắn kết quả nhận diện với quyết định tìm kiếm, định vị và báo cáo. Điều kiện thành công, thông tin robot được biết và cách xử lý thất bại quyết định ý nghĩa của số liệu.

Đề xuất cho nhóm là giữ một nhiệm vụ chung nhưng đánh giá theo ba tầng T1–T3, dùng cùng episode giữa các phương pháp và chấm bằng GT độc lập. Bộ thử đầu tiên tập trung vào che khuất và đường bị chặn; các đặc trưng thiên tai khác được bổ sung khi có dữ liệu hoặc mô hình phù hợp. Với cách này, mỗi kết quả có phạm vi giải thích rõ và có thể truy lại từ dữ liệu chạy.

## Tài liệu tham khảo

1. Chung, T. H., Orekhov, V., Maio, A. (2023). [Into the Robotic Depths: Analysis and Insights from the DARPA Subterranean Challenge](https://doi.org/10.1146/annurev-control-062722-100728). _Annual Review of Control, Robotics, and Autonomous Systems_, 6, 477–502.

2. Tranzatto, M. và cộng sự (2024). [Team CERBERUS Wins the DARPA Subterranean Challenge: Technical Overview and Lessons Learned](https://doi.org/10.55417/fr.2024009). _Field Robotics_, 4, 249–312. Các mục và hình được dẫn trong báo cáo theo [bản arXiv v1 năm 2022](https://arxiv.org/html/2207.04914v1); thông tin xuất bản được đối chiếu với [Oxford Research Archive](https://ora.ox.ac.uk/objects/uuid%3Ac203da77-8014-4ad5-924d-042dd2bf9870).

3. Park, J. và cộng sự (2023). [Zero-shot Active Visual Search (ZAVIS): Intelligent Object Search for Robotic Assistants](https://arxiv.org/html/2209.08803v3). _IEEE ICRA_. Bản đối chiếu: arXiv:2209.08803v3.

4. Perille, D., Truong, A., Xiao, X., Stone, P. (2020). [Benchmarking Metric Ground Navigation](https://arxiv.org/html/2008.13315v3). _IEEE SSRR_. Bản đối chiếu: arXiv:2008.13315v3.

5. Cruz Ulloa, C., Orbea, D., del Cerro, J., Barrientos, A. (2024). [Thermal, Multispectral, and RGB Vision Systems Analysis for Victim Detection in SAR Robotics](https://doi.org/10.3390/app14020766). _Applied Sciences_, 14(2), 766.

6. Morales, J., Vázquez-Martín, R., Mandow, A., Morilla-Cabello, D., García-Cerezo, A. (2021). [The UMA-SAR Dataset: Multimodal Data Collection from a Ground Vehicle during Outdoor Disaster Response Training Exercises](https://doi.org/10.1177/02783649211004959). _The International Journal of Robotics Research_, 40(6–7), 835–847.

7. DARPA. [Subterranean Challenge](https://www.darpa.mil/research/programs/darpa-subterranean-challenge), mục Scoring và Resources. Nguồn quy định bổ trợ cho [1].

8. NIST. [Standard Test Methods for Response Robots](https://www.nist.gov/el/intelligent-systems-division-73500/standard-test-methods-response-robots). Nguồn phương pháp thử năng lực robot.

9. RoboCupRescue Robot League. [Rules](https://rrl.robocup.org/rules/). Nguồn tham khảo tổ chức bài thử; báo cáo chưa áp dụng một phiên bản luật để chấm điểm.

10. Universidad de Málaga, Robotics and Mechatronics Group. [UMA-SAR Datasets](https://www.uma.es/robotics-and-mechatronics/info/124594/sar-datasets/). Trang dữ liệu, calibration và công cụ chính thức của [6].

11. Anderson, P. và cộng sự (2018). [On Evaluation of Embodied Navigation Agents](https://arxiv.org/abs/1807.06757). arXiv:1807.06757. Nguồn bổ trợ về task và đánh giá điều hướng.

12. Yadav, K. và cộng sự (2023). [Habitat Navigation Challenge 2023](https://aihabitat.org/challenge/2023/), mục Task, Dataset và Evaluation; [protocol và starter code chính thức](https://github.com/facebookresearch/habitat-challenge/blob/main/README.md). Nguồn protocol bổ trợ, không tính là một bài nghiên cứu thực nghiệm độc lập.
