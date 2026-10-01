# Phân tích video 01-10-2026 14:12:52

Build: `2026-10-01-safe-probe-8`, web port 5005.

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
2. **Vùng LiDAR:** chỉ yêu cầu dữ liệu cho vùng đường bao mới đi vào; vẫn kiểm tra mọi điểm vật cản với thân và biên an toàn. Khắc phục việc kiểm tra vùng đang chiếm chỗ mâu thuẫn với động tác rời xa điểm nằm trong biên an toàn. Khi tia hiện thời bị khuyết, chỉ dùng ô bản đồ vừa được tia LiDAR thật xác nhận là trống trong 0,75 giây; stale/occupied cells do not authorize movement. Dùng bằng chứng cũ yêu cầu odometry <0,2 s và giảm tốc tịnh tiến tối đa 0,08 m/s.
3. **Tia mất đơn lẻ:** chỉ bù khoảng mất không quá 1° được kẹp giữa hai tia hợp lệ, lấy khoảng cách nhỏ hơn. Không bù vùng mất rộng hoặc giá trị dương quá gần ngoài giới hạn cảm biến. Điểm vật cản đo được vẫn giữ nguyên trong kiểm tra va chạm. Đây là xấp xỉ không gian ở độ phân giải nhỏ, không phải phép đo bổ sung.
4. **Vừa tiến vừa né:** trước khi xoay né, thử tiến chậm 0,08/0,04 m/s và các lệnh tiến chéo với vận tốc ngang 0,04/0,08 m/s. Mỗi quỹ đạo phải qua `FootprintSafety`; heartbeat tiếp tục kiểm tra trước khi phát vận tốc. Đây là bộ chọn chuyển động cục bộ, chưa phải bộ lập đường vòng toàn cục tới vật thể.
5. **Giữ mục tiêu trong camera:** khi có mục tiêu, quay căn theo sai lệch ảnh thay vì quay theo chiều lách một cách cố định. Mục tiêu đã ở giữa thì không tiếp tục quay né mù. Khi YOLO bỏ sót dưới 0,45 giây, dừng tịnh tiến và chờ; sau đó góc quay tìm lại tỷ lệ theo sai lệch, tối đa 12°/s và có vùng dừng 4°.
6. **Nhận diện chồng lấp:** chạy NMS riêng từng lớp. Hộp person/suitcase không được xóa hộp chair/phone chỉ vì chúng chồng lên nhau. Thay đổi này không đảm bảo mô hình hết nhận nhầm.
7. **Tìm vùng mới:** thêm tiến chậm và tiến chéo vào thăm dò tối đa 25 cm. Chỉ đánh dấu đã thăm khi thực sự tới điểm; điểm bị kẹt chỉ bị loại tạm 15 giây. Lập lại bản đồ không tự đánh dấu điểm chưa tới là đã thăm.
8. **Chẩn đoán:** tách `SWEPT AREA UNKNOWN` và `SWEPT AREA BLOCKED`, kèm góc tia. Hiện nguyên nhân dừng trên ảnh và thêm `navigation_block`, `probe_block`, build lên web/status. Giữ cơ chế thử lại khi `BLOCKED` của bản trước.
9. **Tiến gần mục tiêu hơn:** dùng chiều rộng xấp xỉ theo lớp COCO và FOV 60° để ước lượng khoảng cách khi không có depth. Dừng ghế khoảng 75 cm, người 80 cm, vật nhỏ như điện thoại/chai/cốc 45 cm. Depth Astra đã đăng ký thì ưu tiên số đo depth.
10. **Khoảng hở thân xe:** giảm biên từ 8 xuống 5 cm; giữ bán kính thân đã đo 20 cm, nên vùng an toàn tổng là 25 cm.

## Kiểm tra và giới hạn

- 44 kiểm thử tự động đạt, gồm vật cản thật, vùng LiDAR mất rộng, tia mất đơn lẻ, lùi khỏi biên, quỹ đạo chéo, lớp YOLO chồng lấp, điểm tìm kiếm bị kẹt, ô trống gần hết hạn và heartbeat dừng xe.
- Kiểm tra cú pháp Python 3.6 và JavaScript bằng Node đạt.
- Build 6 đã được chạy trên robot khi tạm dừng; phát lại chuỗi 44 scan trực tiếp cho phép đi thẳng ở tốc độ chậm trong 16 mẫu có đủ ô trống gần đây và chặn 28 mẫu còn thiếu phép đo. Build 7 và ngưỡng dừng gần mới chưa được chạy chuyển động ngoài thực tế.
- OAK-D không có hình trong video: chưa xác định nguyên nhân phần cứng/driver; camera này đang chỉ quan sát và không chặn điều hướng LiDAR.
- FPS suy luận và độ chính xác mô hình chưa được đo lại trên JetAuto. Tọa độ mục tiêu suy ra từ kích thước hộp vẫn chỉ là ước lượng khi thiếu depth đã đăng ký.

Biến môi trường `JETAUTO_TARGET` giữ lựa chọn mục tiêu khi khởi động lại. Kiểm tra `/status.build` bằng `2026-10-01-safe-probe-8`. Các trường lý do mới phân biệt vùng thiếu phép đo với vật cản đo được.

11. **Kh?ng ch? quay ?? v?ng m?i th? ?i:** sau khi qu?t ???c ?t nh?t 90?, b? t?m ki?m th? m?t b??c ti?n an to?n t?i ?a 25 cm ? t?c ?? ch?m. B??c ?i ch? ch?y khi FootprintSafety ch?p thu?n v?i LiDAR/? tr?ng g?n ??y; sau b??c th? robot qu?t l?i t? v? tr? m?i. N?u b? ch?n th? ti?p t?c qu?t ?? t?m h??ng kh?c.

## Th? nghi?m tr?c ti?p JetAuto (01-10-2026)

Build 7 kh?i ??ng ???c camera Astra (~27?30 FPS), YOLO (~4,6?5,2 FPS), LiDAR 360? v? odometry m?i; bi?n footprint b?o 25 cm. Trong th? nghi?m bottle, h?nh camera kh?ng cho th?y chai r? r?ng; b? nh?n di?n b?o target m?t khung h?nh r?i m?t. Robot qu?t v? LiDAR ghi nh?n v?t c?n s?t th?n khi quay, n?n kh?ng c? d? li?u ?? ch?c ?? k?t lu?n chai ???c t?m th?y hay ki?m tra ???c ?o?n ti?p c?n. Build 8 th?m b??c d? an to?n sau 90? ?? gi?m th?i gian ch? xoay. L??t th? build 7 ???c d?ng; build 8 c?n ki?m th? chuy?n ??ng ng?n sau c?p nh?t.
