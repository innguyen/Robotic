# S2 — Webots Core Integration

Tích hợp trực tiếp `SearchMission`, `Navigator`, `Costmap`, `Monitor` trong `test version/`; không import controller tìm kiếm S0, không chạy ROS node. Xem [báo cáo nghiệm thu](../../docs/webots-s2-audit.md).

## Chạy

Yêu cầu: Webots **R2025a**, Python có NumPy. Máy nghiệm thu dùng Python 3.13.15 và NumPy 2.5.4 đã cài. Không `pip install controller`: API đó đi kèm Webots.

Từ repository root:

```powershell
$pythonS2 = 'C:/Users/lemin/AppData/Local/Programs/Python/Python313/python.exe'
& $pythonS2 -m unittest discover -s experiments/webots_s2 -p test_adapter.py -v
& $pythonS2 experiments/webots_s2/run_s2.py
& $pythonS2 experiments/webots_s2/run_s2.py --real --evidence experiments/webots_s2/evidence/my-new-run
```

`--evidence` phải là thư mục mới. Có thể chọn `--scenarios found obstacle` hoặc chỉ `calibration`. Nếu Webots cài chỗ khác, truyền `--webots '.../webots.exe'`. Runner giữ project tạm trong `.runs/`, cùng raw logs, world thực chạy, configs và SHA256 trong evidence; không ghi đè S0/S1 hoặc phiên Webots đang mở.

Để xem tương tác, mở [world S2](worlds/s2_search.wbt) bằng Webots và chạy. Controller dùng thuật toán chính thức, tự quay tìm rồi tiếp cận; giữ đứng khi FOUND. Dùng Python command có NumPy trong Preferences. Reset/reload world để chạy lại; không lưu end pose vào canonical world. `run_s2.py` phát hiện canonical world khác generator; chỉ dùng `--write-world` khi chủ động tái tạo fixture.

## Model và giới hạn

- Model **omni lý tưởng bằng body velocity** có rigid body/collision; nhận đủ `(vx,vy,wz)`. Không có mô phỏng roller, traction, encoder hoặc động lực học Mecanum JetAuto.
- Pose là `supervisor_ideal_pose` của chính robot. Target chỉ đến từ camera recognition oracle `s0_person`, xác nhận hai frame mới. Observer riêng đọc scene ground truth để assertion, không cấp target cho policy.
- Scan 192 rays, 360°, một layer; camera 160×120; control/sensor 96 ms, physics 32 ms. LiDAR có sai lệch âm tại seam cylindrical đã ghi trong audit.
- Mặc định planner **đồng bộ**, là cấu hình nghiệm thu tái lập. `--async-planning` là chế độ thử: worker được đóng đúng, ba nhiệm vụ đã FOUND, nhưng thời gian lặp đã FAIL ngưỡng 0.3 s; không dùng chế độ này để yêu cầu replay thời gian giống nhau.
- Ca `absent` kiểm không false FOUND trong 100 s và dừng ở deadline, không chứng minh đã tìm hết mọi nơi.

Mọi lệnh chuyển động đi qua actuator limits và Collision Monitor. Startup/reset/shutdown có đường fail-safe chỉ ghi vận tốc 0 khi chưa có observation hợp lệ. Không nối actuator của robot thật.

## File chính

| File | Trách nhiệm |
| --- | --- |
| [adapter.py](adapter.py) | Frame, scan, timestamps, core lifecycle, mission và safety pipeline |
| [runtime.py](runtime.py) | Webots device sampling, nguồn pose, áp full holonomic Twist |
| [scenarios.json](scenarios.json), [fixtures.py](fixtures.py) | Scenario nhỏ, seed và world generator |
| [s2_core.py](controllers/s2_core/s2_core.py) | Entry point nhiệm vụ thật |
| [s2_audit.py](controllers/s2_audit/s2_audit.py) | Observer độc lập: contacts, footprint SAT, hold, fault, reset/reload |
| [s2_calibrate.py](controllers/s2_calibrate/s2_calibrate.py) | Scan/point-cloud, camera frames, actuator axes, STOP_ZONE, tắt LiDAR thật |
| [test_adapter.py](test_adapter.py) | Unit và integration test với core thật; sensor tổng hợp trong unit test |

YOLO, nâng đồ họa, production ROS và benchmark T1 nằm ngoài S2.
