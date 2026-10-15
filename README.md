# JetAuto tìm kiếm mục tiêu và né vật cản

Hệ thống dùng YOLO + Astra để nhận diện mục tiêu, LiDAR + depth để né vật cản,
và odometry để giới hạn hành vi tìm kiếm. Ứng dụng khởi động ở trạng thái **Pause**;
robot chỉ chạy sau thao tác Resume rõ ràng từ người vận hành.

## Luồng lệnh an toàn

```text
jetauto_find_object.py / jetauto_smart_search.py
               │  /jetauto_ai/cmd_vel_desired + heartbeat
               ▼
     motion_safety_supervisor.py
               │  /jetauto_controller/cmd_vel
               ▼
             chassis
```

Chỉ `motion_safety_supervisor.py` được xuất lệnh lên chassis. Supervisor lập tức
ra lệnh dừng và khóa chuyển động khi heartbeat AI, LiDAR, Astra depth hoặc
odometry bị mất/stale/không hợp lệ. Sau lỗi hoặc sau khi supervisor khởi động
lại, robot không tự chạy tiếp: phải thực hiện **Pause → Resume**.

Tìm kiếm cũng có giới hạn thời gian, bán kính và số chặng. Khi hết giới hạn,
trạng thái chuyển thành `TIM_DUNG` thay vì tiếp tục chạy vô hạn.

## Cài service trên robot

Copy các file thay đổi vào `/home/jetauto/`, sau đó cài hai unit:

```bash
sudo cp /home/jetauto/jetauto-ai.service /etc/systemd/system/
sudo cp /home/jetauto/jetauto-safety.service /etc/systemd/system/
chmod +x /home/jetauto/start_server.sh /home/jetauto/run_motion_safety.sh
sudo systemctl daemon-reload
sudo systemctl enable jetauto-ai.service jetauto-safety.service
sudo systemctl restart jetauto-ai.service jetauto-safety.service
```

Kiểm tra trạng thái và log:

```bash
systemctl status jetauto-ai.service jetauto-safety.service
journalctl -u jetauto-ai.service -u jetauto-safety.service -f
```

`jetauto-safety.service` dùng systemd watchdog. Nếu supervisor treo hoặc thoát,
systemd khởi động lại nhanh; phiên supervisor mới vẫn yêu cầu Pause → Resume.

## Chạy thủ công

`start_server.sh` mặc định tự quản lý supervisor để vẫn dùng được ngoài systemd:

```bash
./start_server.sh jetauto_find_object.py
./start_server.sh jetauto_smart_search.py
./start_server.sh jetauto_find_object.py --check-only
```

Trước khi chạy thủ công, dừng hai service để tránh hai tiến trình cùng tranh quyền
điều khiển: `sudo systemctl stop jetauto-ai.service jetauto-safety.service`.

## Kiểm chứng khi kết nối lại robot

1. Xác nhận app mở ở Pause và `/jetauto_controller/cmd_vel` chỉ có publisher
   `/jetauto_motion_safety`.
2. Resume trong không gian trống, sau đó lần lượt dừng app, kill supervisor,
   che camera depth và ngắt LiDAR; đo quãng đường/thời gian robot dừng.
3. Xác nhận mỗi lỗi đều khóa chuyển động và không tự chạy lại trước Pause → Resume.
4. Chạy thử giới hạn tìm kiếm theo thời gian, bán kính và số chặng.

Các bài kiểm thử logic local:

```bash
python3 -m unittest -v test_obstacle_avoidance.py test_motion_safety_supervisor.py
```
