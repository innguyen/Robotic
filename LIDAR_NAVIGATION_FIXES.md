# Sửa quét LiDAR, né vật cản và kiểm tra chiều cao

Ngày: 27/09/2026. Robot cao toàn bộ **0,50 m**, theo thông tin người dùng.

## Phạm vi và trạng thái thực tế

Đã triển khai mã cho bốn nhóm thay đổi dưới đây. Kiểm thử bằng dữ liệu mô phỏng trên máy phát triển; **chưa triển khai hoặc chạy động cơ robot để xác minh trong lần sửa này**.

Chiều dài/rộng thân xe và vị trí camera chưa được cung cấp. Vì vậy:

- Quét có phục hồi, ghép bản đồ theo timestamp và kiểm tra vùng chuyển động đã được tích hợp.
- Nếu chưa đo thân xe, dùng vùng tròn dự phòng bán kính **0,40 m**, cộng biên ngang **0,08 m**. Đây là cấu hình dự phòng, không phải kích thước robot đã xác minh; có thể chặn nhiều lối đi.
- Kiểm tra chiều cao bằng Astra đã có mã và kiểm thử, nhưng `depth_enabled=false` mặc định. Chỉ bật sau khi đo và hiệu chỉnh; không tự giả định camera nhìn ngang hoặc cao 50 cm.
- Không có cơ chế tự bỏ qua một điểm LiDAR đang báo chắn thân chỉ vì camera nhìn thấy khoảng trống.

## Các tệp thay đổi

| Tệp | Vai trò |
|---|---|
| `navigation_safety.py` | Nội suy odometry, biến đổi tia LiDAR, phục hồi quét, kiểm tra vùng thân xe và thể tích depth |
| `navigation_config.json` | Kích thước, biên an toàn, vị trí cảm biến, thông số phanh |
| `jetauto_smart_search.py` | Tích hợp callback ROS, bản đồ, điều khiển, trạng thái web |
| `templates/smart_dashboard.html` | Hiển thị tiến độ quét, đồng bộ bản đồ, trạng thái hiệu chỉnh |
| `run_smart_search.bat`, `run_robot.bat`, `run_robot.ps1` | Chép module và cấu hình mới cùng chương trình |
| `tests/test_navigation_safety.py` | Kiểm thử logic không cần ROS hoặc động cơ |

## 1. Quét bị kẹt không còn khóa vĩnh viễn

### Trước đây

Robot yêu cầu quay 18°/s, đếm góc dương từ odometry. Sau 30 giây chưa đủ vòng thì vào `BLOCKED` mãi. Thời gian bị lớp an toàn chặn vẫn tính vào 30 giây. Dao động trái–phải cũng có thể làm cộng sai tổng góc.

### Sau sửa

`ScanRecovery` cộng góc có dấu, xử lý đúng điểm chuyển ±180°. Góc quay thực tế, không phải vận tốc yêu cầu, quyết định hoàn tất.

1. Nếu góc thực tế tăng ít hơn 3° trong 4 giây, hoặc một lượt kéo dài quá 40 giây, dừng yêu cầu quay.
2. Chờ 2 giây, đổi chiều quét và thử lại. Tối đa hai lần thử lại.
3. Nếu vẫn không hoàn thành, chuyển sang lập đường trong **vùng đã quan sát**. Không coi phần chưa nhìn thấy là trống.
4. Nếu chưa có đường phù hợp, giữ trạng thái chờ và lập lại đường mỗi 5 giây. Không tự lùi mù hay tự bỏ kiểm tra an toàn.

Các pha chính: `SCAN → RECOVER → SCAN → PLAN → MOVE/WAIT`.

## 2. Đồng bộ LiDAR–odometry và ổn định bản đồ

### Đồng bộ và biến đổi tọa độ

- Lưu lịch sử odometry theo `header.stamp`, tối đa 600 mẫu.
- Nội suy x/y và yaw giữa hai mẫu bao quanh thời điểm đo. Không ngoại suy; khoảng cách giữa hai mẫu không được vượt 0,20 giây.
- Thời điểm tia thứ i: `scan.header.stamp + i * scan.time_increment`.
- Tính vị trí từng tia từ pose tương ứng và cấu hình `lidar_x_m`, `lidar_y_m`, `lidar_yaw_deg`. Vì vậy các tia trong cùng lượt quét không còn bị gán chung một góc mới nhất.
- Nếu odometry chưa đến, giữ tối đa sáu scan trong hàng đợi, chờ tối đa 0,5 giây rồi bỏ scan chưa ghép được.
- Bỏ scan quá cũ, đảo thứ tự hoặc có thông số thời gian/góc không hợp lệ. Dữ liệu LiDAR phục vụ chặn chuyển động vẫn được xử lý độc lập với hàng đợi bản đồ.
- Khi odometry nhảy đột ngột, xóa bản đồ và bộ nhớ vị trí mục tiêu, đặt lệnh vận tốc về 0, bắt đầu lại quét.

`time_increment=0` được chấp nhận nhưng chỉ có thể dùng một pose cho cả lượt đo: không thể bù chuyển động từng tia nếu driver không cung cấp thời gian tia.

### Bản đồ có tích lũy bằng chứng

Giữ bản đồ 12 × 12 m, ô 5 cm, giới hạn tia 4 m. Thay ghi đè đơn giản bằng mức bằng chứng có giới hạn:

- Tia đi qua ô: giảm bằng chứng vật cản 0,7.
- Điểm phản hồi nằm ở ô: tăng 1,5.
- Giới hạn bằng chứng từ -4 đến 4.
- Ô không được quan sát trong 30 giây trở lại **chưa biết**, không tự trở thành lối trống.
- NaN, điểm ngoài khoảng đo hợp lệ và điểm quá gần bị loại; chỉ `+inf` hợp lệ mới biểu thị không có phản hồi trong tầm đo.

Đây vẫn là bản đồ dựa trên odometry, **không phải SLAM có scan matching hoặc hiệu chỉnh vòng lặp**. Bánh trượt, odometry sai hệ số và cấu hình cảm biến sai vẫn làm bản đồ lệch.

## 3. Né theo thân xe và quãng dừng

Hai lớp chọn lệnh AI và xuất lệnh 10 Hz dùng chung `FootprintSafety`. Lớp cuối kiểm tra lại dữ liệu trước khi gửi vận tốc.

- Khi có chiều dài/rộng, dùng hình chữ nhật và vị trí tâm thân so với gốc `base`.
- Khi chưa đo, dùng hình tròn dự phòng. Bộ lập đường dùng bán kính bao thân cộng biên để chọn đường một cách bảo thủ; kiểm tra vận tốc dùng hình thân chi tiết.
- Kiểm tra cả tổ hợp tiến/ngang/quay. Không chỉ kiểm tra độc lập bốn ngưỡng 35 cm.
- Mô phỏng quỹ đạo vận tốc thân xe trong thời gian phản ứng cộng thời gian phanh. Bước dịch chuyển mép thân không quá khoảng 2 cm.
- Khoảng dự báo tăng khi tốc độ hoặc tốc độ góc tăng; dùng `braking_accel_mps2` để tính thời gian phanh.
- Kiểm tra điểm vật cản và độ bao phủ tia trên biên thân sẽ đi qua. Scan thiếu vùng quan sát hoặc chứa dữ liệu không hợp lệ ở vùng cần kiểm tra không được coi là an toàn.
- Scan quá 0,5 giây không cho phát lệnh chuyển động mới.
- Nếu theo mục tiêu và hướng đi bị chặn, chỉ chọn quay trái/phải nếu cùng bộ kiểm tra xác nhận quỹ đạo quay an toàn. Nếu không có động tác hợp lệ thì dừng.

Do yêu cầu kiểm tra vùng bao thân, LiDAR chỉ có một phần góc nhìn có thể khiến lệnh bị chặn. Không còn giả định cứ đủ 90° là có thể quay an toàn cả thân xe.

Các khoảng cách trước/sau/trái/phải vẫn có trong `/status` để chẩn đoán, nhưng không còn là bốn ngưỡng cố định quyết định mọi chuyển động.

## 4. Kiểm tra khoảng trống cao 60 cm bằng Astra

Chiều cao cần trống = `robot_height_m + height_margin_m = 0,50 + 0,10 = 0,60 m`.

`DepthClearance` kiểm tra thể tích phía trước mà thân xe sắp đi vào:

1. Nhận ảnh depth và `CameraInfo` cùng kích thước, cùng `frame_id`, có nội tại hợp lệ.
2. Yêu cầu ảnh đã hiệu chỉnh méo phù hợp với `CameraInfo`; hiện chỉ chấp nhận hệ số méo bằng 0. Không dùng FOV ước lượng để suy ra chiều cao.
3. Đổi mẫu thể tích trong hệ thân sang hệ quang học camera bằng vị trí, góc chúc xuống và góc ngang đã đo.
4. Chiếu mẫu lên ảnh, kiểm tra depth có nằm phía sau thể tích cần trống hay không. Dùng bước mẫu 2 cm và các pixel lân cận.
5. Vật treo nằm hoàn toàn trên 60 cm không tự trở thành điểm chắn thể tích. Mặt bàn thấp, chân bàn, depth thiếu hoặc thể tích nằm ngoài ảnh đều không được cấp quyền tiến.

### Giới hạn cần hiểu rõ

- Phiên bản này chỉ xác nhận **đi thẳng phía trước** bằng depth. Khi bật depth, lùi, ngang hoặc quay ngoài ngưỡng nhỏ bị chặn vì camera trước chưa chứng minh được khoảng trống 3D các hướng đó. Không nên bật rồi kỳ vọng tự đi mọi hướng dưới bàn.
- Camera có vùng mù sát thân hoặc không nhìn hết từ gần sàn tới 60 cm có thể trả `DEPTH VOLUME OUTSIDE VIEW`. Cần thay đổi vị trí/góc camera hoặc bổ sung cảm biến; không hạ yêu cầu dữ liệu để vượt qua.
- Không phát hiện đầy đủ mép cầu thang/hố, kính hoặc mọi vật rất mảnh. Đây là kiểm tra lấy mẫu có biên dự phòng, chưa phải chứng nhận an toàn.
- LiDAR vẫn có quyền chặn. Nếu tia LiDAR thực sự trúng một vật trong mặt phẳng ngang nằm dưới đỉnh robot thì chưa thể gọi đó là vật “nằm hoàn toàn trên đầu”. Phải kiểm tra vị trí lắp và vật cụ thể.
- Bản đồ 2D không bị xóa điểm LiDAR chỉ dựa vào một ảnh depth. Nếu bản đồ đã sai, xử lý đồng bộ/hiệu chỉnh; không dùng chiều cao làm lý do bỏ qua dữ liệu xung đột.

## Cấu hình trước khi kiểm thử phần cứng

### Tự lấy kích thước từ cấu hình ROS

Khi chiều dài/rộng trong JSON còn `null`, chương trình tìm `footprint` và `robot_base_frame` trong các tham số costmap của ROS. Nếu không có, tìm các file `*costmap*.yaml` của package `jetauto_navigation` qua `rospkg`, chỉ nhận khi có một hình thân và hệ tọa độ không mâu thuẫn. Dùng hình chữ nhật bao polygon, không thu nhỏ thân dựa trên khoảng cách vật xung quanh. Giá trị JSON đã điền được ưu tiên.

Nguồn được ghi trong `/status.footprint_source` và log khởi động. `footprint_calibrated` chỉ cho biết đã có kích thước cấu hình; không xác nhận mô hình ROS khớp mọi phần nhô ra sau khi sửa cơ khí. Hệ tọa độ footprint phải trùng hệ thân odometry, hoặc được URDF chứng minh chỉ khác độ cao. Không tìm được cấu hình thì giữ vùng dự phòng và thông báo thiếu kích thước. Việc này sửa nguyên nhân chặn bởi vòng tròn dự phòng quá lớn khi ROS đã có footprint phù hợp, nhưng không bỏ kiểm tra các hướng LiDAR thiếu dữ liệu.

Sửa **bản local** `navigation_config.json`; script chạy sẽ chép nó lên robot.

| Trường | Cách điền |
|---|---|
| `body_length_m`, `body_width_m` | Kích thước lớn nhất, tính cả phần nhô ra; đơn vị mét |
| `body_center_x_m`, `body_center_y_m` | Tâm hình chữ nhật thân so với gốc odometry/base |
| `horizontal_margin_m` | Biên ngang; mặc định 0,08 m |
| `lidar_x_m`, `lidar_y_m` | Tâm LiDAR so với gốc base; x hướng trước, y hướng trái |
| `lidar_yaw_deg` | Góc tia 0 LiDAR so với hướng trước; dương quay trái |
| `lidar_extrinsics_calibrated` | Chỉ đặt `true` sau khi kiểm tra các thông số trên |
| `camera_x_m`, `camera_y_m`, `camera_height_m` | Tâm quang học Astra so với base/sàn |
| `camera_pitch_down_deg` | Dương khi camera chúc xuống |
| `camera_yaw_deg` | Dương khi camera hướng lệch trái; hiện giả định camera không bị nghiêng roll |
| `depth_topic`, `depth_info_topic` | Cặp ảnh depth đã hiệu chỉnh và CameraInfo tương ứng |
| `depth_extrinsics_calibrated` | Chỉ đặt `true` sau khi đo và kiểm tra biến đổi |
| `depth_enabled` | Bật kiểm tra 3D; thiếu hiệu chỉnh sẽ chặn chuyển động thay vì đoán |
| `reaction_time_s`, `braking_accel_mps2` | Mặc định 0,30 s và 0,30 m/s²; cần đo quãng phanh thực tế |

Không điền kích thước ước lượng rồi đánh dấu đã hiệu chỉnh. `fallback_radius_m` chỉ giúp vận hành bảo thủ khi chưa có số đo, không chứng minh đủ khoảng hở cho phần cứng chưa biết kích thước.

## Kiểm thử và triển khai

Chạy tại thư mục dự án:

```powershell
python -m unittest discover -s tests -p test_navigation_safety.py -v
```

Bộ kiểm thử bao gồm góc ±180°, khoảng trống odometry, biến đổi vị trí LiDAR, tia không hợp lệ, phục hồi khi kẹt, chống cộng góc dao động, quỹ đạo góc thân, quãng phanh theo tốc độ, vùng quét bị thiếu, trần cao/thấp, chân bàn, depth lỗi/cũ và tuổi thọ ô bản đồ.

Kết quả trên máy phát triển: **20/20 bài kiểm thử đạt**, bao gồm kiểm thử lớp xuất vận tốc cuối thực sự gửi vận tốc 0 khi depth chặn. Cú pháp dashboard JavaScript và script PowerShell cũng đã kiểm tra.

Đã kiểm tra cú pháp Python 3.6 cho mã chạy trên robot. Các bài kiểm tra này không xác nhận driver, cơ khí, quãng phanh hoặc góc lắp thực tế.

Sau khi kiểm tra cấu hình, chạy `run_smart_search.bat`. Giữ nguyên cổng **5005** và hai lượt nhập mật khẩu; không chuyển lại ảnh/mô hình khuôn mặt. Robot khởi động ở trạng thái tạm dừng.

Trình tự thử phần cứng:

1. Giữ tạm dừng, đọc `/status`: LiDAR, odometry mới; `mapping_status` đồng bộ; kích thước và hiệu chỉnh đúng.
2. Kiểm tra vật chuẩn và hướng scan khi đứng yên, rồi quay chậm có giám sát để kiểm tra chồng vệt.
3. Thử dừng trước vật và góc thân ở tốc độ thấp, xác nhận quãng dừng thực tế.
4. Sau cùng mới bật depth và thử lần lượt: lối trống, thanh thấp dưới 60 cm, mặt bàn trên 60 cm nhưng có chân, vật treo cao với lối dưới trống.

## Trạng thái chẩn đoán

`/status` bổ sung `mapping_status`, `footprint_calibrated`, `depth_clearance`, `depth_clearance_enabled`, `explorer_phase`, `scan_swept_deg`. Dashboard hiển thị chúng dưới phần điều khiển.

| Trạng thái | Ý nghĩa |
|---|---|
| `WAITING FOR STAMPED ODOMETRY` | Chưa có hai pose bao quanh thời điểm scan |
| `SYNCED; LIDAR EXTRINSICS UNCALIBRATED` | Đã ghép thời gian, nhưng vị trí LiDAR còn dùng cấu hình chưa xác minh |
| `RECOVER` | Đang chờ đổi chiều quét; không phải khóa vĩnh viễn |
| `FOOTPRINT COLLISION` | Có điểm trong vùng thân sẽ chiếm |
| `SWEPT AREA UNKNOWN OR BLOCKED` | Vùng quét qua chưa đủ dữ liệu hoặc không đủ khoảng cách |
| `DEPTH NEED CALIBRATION` | Đã bật depth nhưng thiếu kích thước/hiệu chỉnh |
| `DEPTH VOLUME OUTSIDE VIEW` | Camera không thấy đủ thể tích cần đi qua |
| `DEPTH BODY VOLUME BLOCKED` | Có bề mặt trước hoặc trong thể tích cần trống |
| `DEPTH CLEAR TO 60CM HEIGHT` | Kiểm tra thể tích phía trước đạt; vẫn phải qua kiểm tra LiDAR |

## Cập nhật sau video ngày 01-10-2026

- Bán kính thân đo được là 20 cm, cộng biên an toàn 8 cm. Với thân tròn, quay tại chỗ không cần đủ tia LiDAR phía sau; điểm đo trong thân thực 20 cm hoặc bản quét quá cũ vẫn chặn quay.
- Khi dịch chuyển, chỉ yêu cầu dữ liệu LiDAR ở nửa đường bao đang tiến vào vùng mới. Điểm đang nằm trong biên an toàn được phép rời xa nếu khoảng cách tăng liên tục; đi sâu hơn về phía điểm vẫn bị chặn.
- Nếu bản đồ chưa tìm được đường tới vùng chưa biết, robot thử một đoạn tối đa 25 cm theo hướng được LiDAR xác nhận, rồi lập đường lại. Nếu không có hướng an toàn, robot chờ và thử lại. `/status.plan_reason` cho biết vì sao không có đường.
- Với nhiều chai/cốc/vật nhỏ cùng lớp, chọn mục tiêu theo độ tin cậy và vị trí thấp trong ảnh. Khi tiến bị chặn, thử đi ngang tối đa 30 cm theo hướng được kiểm tra an toàn, sau đó quay tìm hướng khác nếu vẫn không tiến được.

Các thay đổi đã qua kiểm thử mô phỏng, chưa được xác minh trên robot sau video này. Cần kiểm tra vị trí tâm LiDAR so với thân và các phản xạ ở gần thân khi thử phần cứng.

### Chống đổi hướng liên tục khi bám mục tiêu

Khi đang bám một chai/người, khung nhận diện chỉ được ghép với vị trí cũ nếu tâm của nó di chuyển trong giới hạn hợp lý. Một khung khác ở xa không được chiếm quyền bám ngay sau một lần YOLO bỏ sót. Góc lái lấy từ tâm khung nhận diện được lọc EMA, dùng vùng chết 24 px để bắt đầu chỉnh và 12 px để ngừng chỉnh; lệnh quay giới hạn 18°/s. Khi đường tiến bị chặn, robot chọn một hướng né và giữ hướng đó tối đa 30 cm hoặc 4 giây, sau đó quay cùng chiều tối đa 45°. Robot chỉ trở lại tiến sau ba lần kiểm tra liên tiếp cho thấy đường tiến an toàn. Nếu cả lách và quay đều bị chặn, bản cũ giữ `BLOCKED` vô thời hạn; bản `2026-10-01-target-retry-4` chờ ngắn rồi thử phía đối diện, và mọi lệnh vẫn phải qua `FootprintSafety`. Trạng thái nằm trong `/status.target_avoidance_phase`, `target_avoidance_direction`, `target_avoidance_attempts` và `motion_block`.

Âm thanh “đã tìm thấy …” được phát sau hai khung nhận diện liên tiếp; nó chỉ xác nhận phát hiện, không xác nhận đã tới gần. Nếu hình ảnh hiện “CHUA CO LOI DI AN TOAN TOI MUC TIEU”, lệnh tiến đã bị kiểm tra footprint/LiDAR chặn. Mục tiêu như ghế cũng phản hồi LiDAR như vật cản, nên bộ an toàn có thể buộc robot dừng hoặc tìm hướng lách; cần dùng `/status.motion_block` và các trường `target_avoidance_*` để xem nguyên nhân cụ thể. Bản retry mới chưa được xác minh chuyển động trên robot thật.
