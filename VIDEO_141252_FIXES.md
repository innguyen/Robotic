# Phân tích video 01-10-2026 14:12:52

Build: `2026-10-01-scan-plan-move-9`, web port 5005.

## Bằng chứng trong video

| Thời điểm xấp xỉ | Quan sát |
| --- | --- |
| 00:15–00:21 | `SWEPT AREA UNKNOWN OR BLOCKED`, camera nhìn ra khoảng sàn trống, robot không tịnh tiến. |
| 00:45–01:15 | `CHO DUONG DI AN TOAN`; bảng trạng thái báo `WAIT`, đã quét khoảng 352°, vị trí robot vẫn (0, 0). |
| 00:15–01:06 | Mục tiêu hiển thị `MOBILEPHONE`; tên lớp thực trong `coco.names` là `cell phone`. |
| 01:29–02:57 | Xen kẽ mất dấu, quét tìm lại và xoay né; mục tiêu ghế/chai có lúc ở gần giữa ảnh nhưng vẫn xoay. |
| Suốt các đoạn quan sát | Astra khoảng 29–30 FPS, YOLO khoảng 4–5 FPS. OAK-D đang chờ tín hiệu. |

Ảnh màn hình không chứa các tia `/scan`. Vì vậy chưa thể khẳng định chính xác lần dừng do tia mất dữ liệu, vật cản thật hay sai vị trí lắp LiDAR. Không coi hình sàn trống là bằng chứng đầy đủ để bỏ kiểm tra va chạm.

## Thay đổi

1. **Tên mục tiêu:** chuẩn hóa `phone`, `mobilephone`, `mobile phone`, `cellphone`, `điện thoại` thành `cell phone`. API từ chối tên lớp không có trong mô hình; web không tiếp tục bật tìm kiếm sau lỗi chọn lớp. Nhãn trên web lấy tên chuẩn từ server.
2. **Vùng LiDAR:** chỉ yêu cầu dữ liệu cho vùng đường bao mới đi vào; vẫn kiểm tra mọi điểm vật cản với thân và biên an toàn. Khắc phục việc kiểm tra vùng đang chiếm chỗ mâu thuẫn với động tác rời xa điểm nằm trong biên an toàn. Khi tia hiện thời bị khuyết, chỉ dùng ô bản đồ được tia LiDAR thật xác nhận là trống trong 2 giây; vật cản đo được xóa bằng chứng trống ngay. Dùng bằng chứng cũ yêu cầu odometry <0,2 s và giới hạn tốc độ tịnh tiến tối đa 0,08 m/s.
3. **Tia mất đơn lẻ:** chỉ bù khoảng mất không quá 1° được kẹp giữa hai tia hợp lệ, lấy khoảng cách nhỏ hơn. Không bù vùng mất rộng hoặc giá trị dương quá gần ngoài giới hạn cảm biến. Điểm vật cản đo được vẫn giữ nguyên trong kiểm tra va chạm. Đây là xấp xỉ không gian ở độ phân giải nhỏ, không phải phép đo bổ sung.
4. **Vừa tiến vừa né:** trước khi xoay né, thử tiến chậm 0,08/0,04 m/s và các lệnh tiến chéo với vận tốc ngang 0,04/0,08 m/s. Mỗi quỹ đạo phải qua `FootprintSafety`; heartbeat tiếp tục kiểm tra trước khi phát vận tốc. Đây là bộ chọn chuyển động cục bộ, chưa phải bộ lập đường vòng toàn cục tới vật thể.
5. **Giữ mục tiêu trong camera:** khi có mục tiêu, quay căn theo sai lệch ảnh thay vì quay theo chiều lách một cách cố định. Mục tiêu đã ở giữa thì không tiếp tục quay né mù. Khi YOLO bỏ sót dưới 0,45 giây, dừng tịnh tiến và chờ; sau đó góc quay tìm lại tỷ lệ theo sai lệch, tối đa 12°/s và có vùng dừng 4°.
6. **Nhận diện chồng lấp:** chạy NMS riêng từng lớp. Hộp person/suitcase không được xóa hộp chair/phone chỉ vì chúng chồng lên nhau. Thay đổi này không đảm bảo mô hình hết nhận nhầm.
7. **Tìm vùng mới:** thêm tiến chậm và tiến chéo vào thăm dò tối đa 25 cm. Chỉ đánh dấu đã thăm khi thực sự tới điểm; điểm bị kẹt chỉ bị loại tạm 15 giây. Lập lại bản đồ không tự đánh dấu điểm chưa tới là đã thăm.
8. **Chẩn đoán:** tách `SWEPT AREA UNKNOWN` và `SWEPT AREA BLOCKED`, kèm góc tia. Hiện nguyên nhân dừng trên ảnh và thêm `navigation_block`, `probe_block`, build lên web/status. Giữ cơ chế thử lại khi `BLOCKED` của bản trước.
9. **Tiến gần mục tiêu hơn:** dùng chiều rộng xấp xỉ theo lớp COCO và FOV 60° để ước lượng khoảng cách khi không có depth. Dừng ghế khoảng 75 cm, người 80 cm, vật nhỏ như điện thoại/chai/cốc 45 cm. Depth Astra đã đăng ký thì ưu tiên số đo depth.
10. **Khoảng hở thân xe:** giảm biên từ 8 xuống 5 cm; giữ bán kính thân đã đo 20 cm, nên vùng an toàn tổng là 25 cm.

## Kiểm tra và giới hạn

- 48 kiểm thử tự động đạt, gồm vật cản thật, vùng LiDAR mất rộng, tia mất đơn lẻ, lùi khỏi biên, quỹ đạo chéo, lớp YOLO chồng lấp, điểm tìm kiếm bị kẹt, ô trống gần hết hạn và heartbeat dừng xe.
- Kiểm tra cú pháp Python 3.6 và JavaScript bằng Node đạt.
- Phát lại chuỗi LiDAR trực tiếp để kiểm tra quyết định đi chậm trước khi chạy trên robot. Thử nghiệm chuyển động build 7 và giới hạn còn lại được ghi ở dưới.
- OAK-D không có hình trong video: chưa xác định nguyên nhân phần cứng/driver; camera này đang chỉ quan sát và không chặn điều hướng LiDAR.
- FPS suy luận và độ chính xác mô hình chưa được đo lại trên JetAuto. Tọa độ mục tiêu suy ra từ kích thước hộp vẫn chỉ là ước lượng khi thiếu depth đã đăng ký.

Biến môi trường `JETAUTO_TARGET` giữ lựa chọn mục tiêu khi khởi động lại. Kiểm tra `/status.build` bằng `2026-10-01-scan-plan-move-8`. Các trường lý do mới phân biệt vùng thiếu phép đo với vật cản đo được.

## Kiểm thử trực tiếp JetAuto (01-10-2026)

Build 7 khởi động được camera Astra (~27–30 FPS), YOLO (~4,6–5,2 FPS), LiDAR 360° và odometry mới; vùng an toàn báo 25 cm tính từ tâm thân. Khi tìm bottle, bộ nhận diện báo thấy vật trong một khung hình rồi mất. Robot quay để quét; tại một số góc LiDAR báo vật nằm sát thân nên bộ kiểm tra dừng quay. Chưa có bằng chứng rằng đã thấy chai thật hoặc đã thử được đoạn tiến tới chai.

Build 8 thực hiện chu trình: quay một vòng để camera tìm vật và tích lũy bản đồ → lập đường tới vùng chưa xem → đi; nếu không có biên bản đồ thì thử một bước ngắn tối đa 25 cm → quét lại ở vị trí mới. Khi quay bị chặn, dùng dữ liệu LiDAR 360° đã thu để lập đường từ phần bản đồ hiện có. Việc lập đường được thử trước khi kiểm tra khả năng tiến theo hướng thân xe đang đứng. Bản đồ giữ ô trống đã đo 2 giây; phát lại 44 bản quét trực tiếp cho phép tiến chậm trong 38 mẫu, so với 18 mẫu khi giữ 0,75 giây. Tia vật cản hiện thời vẫn chặn chuyển động.

Build 8 đã khởi động trên robot ở cổng 5005, mục tiêu bottle, trạng thái tạm dừng. Camera, LiDAR, odometry và AI đều có dữ liệu mới; `/status` báo biên footprint 25 cm và vận tốc lệnh bằng 0. Chưa chạy thử chuyển động build 8 vì tại vị trí hiện tại LiDAR đo 18 cm phía trước, 27 cm bên trái, và ảnh Astra thấy người/ghế sát robot. Cần có khoảng trống thật quanh thân để xác nhận đoạn đi thực tế.

## Live check, 2026-10-01 (build 9)

During the build 8 live run, odometry changed by about 8-10 cm, but intermittent single-frame YOLO bottle detections repeatedly entered `REACQUIRE`. Each return to exploration reset `explorer_phase=SCAN` and `scan_swept_deg=0`, preventing the scan-plan-move cycle from completing. Build 9 requires two nearby detections within 0.7 seconds before leaving exploration and preserves the explorer phase through short target loss.

All 50 unit tests pass; the robot's Python 3.6 interpreter compiles the updated files. Build 9 is deployed on port 5005, with camera, LiDAR, odometry and AI responding. The robot is **paused**, with zero commanded velocity. The battery indicator reads 13-15% (10.3-10.4 V), so live build 9 motion was not tested. After charging and clearing the space, verify a complete scan followed by `PLAN`, `MOVE` or `PROBE` and a change in `pose_x/pose_y`.

## Build 10: measured body shape and lateral escape

The robot identifies itself as JetAutoPro. [Hiwonder lists JetAuto Pro](https://www.hiwonder.com/products/jetauto-pro) as 324 x 260 mm in plan view. The previous 20 cm circular footprint treated a LiDAR return 18 cm ahead as inside the physical body and blocked every translation, even if the side was open. Build 10 uses a 32.4 x 26.0 cm rectangle with the existing 5 cm horizontal safety margin. A return inside the actual rectangle still blocks motion. For a return only in the margin, a slow sideways escape (at most 8 cm/s) is permitted if it does not decrease clearance to any measured obstacle and the full swept path has LiDAR coverage. Forward motion toward the return remains blocked.

This is the manufacturer's stock footprint, not a measurement of this modified robot. Any accessory that projects beyond 32.4 x 26.0 cm requires updating `body_length_m`, `body_width_m`, and the body center before autonomous motion. The 50 cm height is separate and does not establish horizontal clearance. Sensor streams must also be fresh before movement.

Live diagnostics after build 10 deployment: `/status` reported the 32.4 x 26.0 cm body and fresh camera/LiDAR/odometry. A brief active interval showed `TIEN CHAM VA LACH THEO LIDAR`, but no completed lateral displacement was verified. The battery then fell to 8-9% (about 10.22 V), so motion was paused. A stationary raw LiDAR scan had 299 finite valid returns out of 756 samples; the guard rejected forward, left and right translation because the newly entered swept area contained unmeasured bearings (38.4, 137.3 and -137.3 degrees respectively). It allowed in-place turns. Map points on the web may persist from older scans and do not prove a newly swept side corridor is clear. A charged battery and a fresh, sufficiently covered side corridor are needed for a physical lateral-avoidance test.

## Build 11: short side steps and return to the target

The user requested avoiding toward the open side with about 5 cm steps. Tracking now tries a slower safe forward command before selecting a side. LiDAR sector distances order the side candidates; the swept-footprint guard independently checks each side, so a large sector distance alone cannot authorize movement. The chosen side stays latched during avoidance. The previous diagonal-command branch no longer resets an active side step.

Each lateral step uses odometry to stop after approximately 5 cm and rechecks the route to the same target. Speed decreases to 4 cm/s near the end of a step. On the first clear forward observation, lateral motion stops immediately; after a second consistent clear observation, tracking resumes forward. There is no requirement to move 10, 15 or 30 cm before rejoining the target. If the obstacle still blocks the target, another checked 5 cm step can follow, within an overall 60 cm / 8 second avoidance limit and a 2.5 second odometry-progress timeout. Short target loss during a side step allows up to 3 seconds of stationary target reacquisition.

`/status.avoidance_checks` reports forward/left/right availability and the specific guard reason; `strafe_dir` now reflects the motor heartbeat's lateral command instead of the unused OAK-D branch. The dashboard shows whether a side has a path, an obstacle, or insufficient fresh LiDAR coverage. Unknown space and physical collisions remain blocking conditions.

57 automated tests pass, including the actual rectangular-footprint scenario in which a 5 cm lateral shift clears an obstacle edge and forward tracking resumes without extra side travel. Python 3.6 and dashboard JavaScript syntax checks pass. Physical stopping distance can differ from 5 cm due to wheel slip, odometry error and control latency; the heartbeat rechecks the trajectory throughout movement.
