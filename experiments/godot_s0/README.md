# Godot S0 — Game Engine Simulator Feasibility

Project độc lập; không import ROS, Webots, SearchMission/Navigator hay YOLO. Xem [báo cáo nghiệm thu](../../docs/godot-s0-feasibility.md).

## Mở project

Trong Godot Project Manager chọn **Import** → `experiments/godot_s0/project.godot` → Run (F6/F5 theo scene/project). Project này dùng **Compatibility**, không phải project Forward+ ở `C:/Projects/robotic-godot-benchmark` đã có trên máy. Không cần asset hoặc plugin bên ngoài.

Scene hiển thị phòng, robot xanh, mannequin capsule đỏ, tường/hộp và ảnh camera. Robot đứng chờ Python; đây là chế độ lockstep. Chạy trực tiếp:

```powershell
& C:\Godot\Godot_v4.7.2-stable_win64.exe --path C:\Projects\Robotic\experiments\godot_s0
```

Bridge mặc định bind **127.0.0.1:17650**, chỉ một client, không expose LAN. Đổi port bằng `-- --port=17651`.

## Python điều khiển

Python 3.10+ với standard library, không cần `pip install`. Trên máy nghiệm thu dùng `C:\msys64\ucrt64\bin\python.exe` (3.12.11); launcher Python 3.13 trỏ tới installation không còn tồn tại.

Từ thư mục này, sau khi project đang chạy:

```python
from bridge import GodotBridge, red_detection

sim = GodotBridge()
try:
    obs = sim.reset("clear", seed=7)
    obs = sim.step(vx=0.0, vy=0.0, wz=0.6, ticks=6)
    print(obs["simulation_timestamp"], obs["pose"], obs["scan"]["ranges"])
    print(red_detection(obs["camera"]))
finally:
    sim.close()
```

`clear`: người nằm ngoài FOV ban đầu; `occluded`: người sau tường; `obstacle`: cùng tường chặn đường cho thử collision; `absent`: không có người trong phòng; `calibration`: phòng trống để đối chiếu analytic ray ranges. Fixtures cố định; seed ghi identity, **không** tạo randomized worlds.

## Chạy audit tự động

Runner tự mở Godot thật bằng subprocess, chạy assertions, đóng đúng process đã mở. Thư mục output phải chưa tồn tại để bảo toàn evidence cũ:

```powershell
& C:\msys64\ucrt64\bin\python.exe experiments/godot_s0/run_s0.py --godot C:\Godot\Godot_v4.7.2-stable_win64.exe --out experiments/godot_s0/evidence/my-rendered-run
& C:\msys64\ucrt64\bin\python.exe experiments/godot_s0/run_s0.py --godot C:\Godot\Godot_v4.7.2-stable_win64.exe --headless --out experiments/godot_s0/evidence/my-headless-run
& C:\msys64\ucrt64\bin\python.exe -m unittest discover -s experiments/godot_s0 -p test_bridge.py -v
```

Headless có physics/rays/bridge, **không có RGB** với dummy renderer: `camera.valid=false`, `data=null`; visibility/mission phải UNVERIFIED. Dùng cửa sổ có đồ họa để nghiệm thu camera; không lấy headless FPS làm kết quả GPU. Một audit đồ họa mất khoảng một phút trong fixture nhỏ; không chạy đồng thời hai audit nếu muốn diễn giải performance sample.

## Wire protocol v1

UTF-8 NDJSON qua TCP, một request → một response, newline framing. Request ≤64 KiB; client kiểm response ID, clocks, schema sensor cơ bản, finite pose/ranges và kích thước RGB. Client không thread-safe. Các command:

```json
{"id":1,"op":"reset","scenario":"clear","seed":7}
{"id":2,"op":"observe"}
{"id":3,"op":"step","epoch":1,"velocity":[0.3,0.1,0.2],"ticks":6}
{"id":4,"op":"quit"}
```

Success: `{"ok":true,"id":3,"observation":{...}}`. Error: `{"ok":false,"id":3,"error":"stale_epoch","epoch":2}`. Sau reset dùng epoch mới cho step. Client `GodotBridge` tự cập nhật epoch. Giới hạn `vx ±0.6 m/s`, `vy ±0.4 m/s`, `wz ±1 rad/s`, `ticks` nguyên 1–600. Lệnh không hợp lệ bị từ chối trước khi chuyển động; command hết hạn sau số tick đã yêu cầu. Disconnect hủy phần command còn lại. Đây không phải safety system cho robot thật.

Observation có `epoch`, `tick`, `simulation_timestamp=tick/60`, self `pose`, `scan`, `camera`, diagnostics. Pose world ENU (+X đông, +Y bắc, +Z lên); command base FLU (+X tiến, +Y trái, yaw CCW). Godot world `(x,y,z)=(east,up,-north)`. Pose là ground truth của **robot**, không phải encoder/SLAM.

Base origin tại tâm chassis, world height 0,11 m. Scan: 192 ray, một vòng 360°, tâm bin đầu `-π+π/192`, increment `2π/192`, max 6 m, ray origin base `(0.12,0,0.14)` m, tức world height 0,25 m; `null` = no return, không dùng JSON Infinity. Range chưa có noise hoặc mô hình intensity/material.

Camera: RGB8 160×120, horizontal FOV 1.012290966 rad, origin base `(0.12,0,0.44)` m, tức world height 0,55 m, nhìn +X. `data` là base64 của 57.600 byte RGB row-major, không phải PNG/JPEG. `stamp` cùng sim tick với pose/scan: robot được giữ đứng yên khi chờ `frame_post_draw` và readback. Ảnh PNG trong evidence được encode lossless từ RGB này. Không có target coordinates, object recognition ID hay visibility oracle trong observation.

Logical simulation clock chỉ tiến khi `step`; physics engine vẫn tick khi chờ TCP. Chỉ phù hợp với scene S0 gồm static bodies và robot được điều khiển thủ công. Nếu bổ sung rigid bodies/animation thì phải thiết kế lại stepping/pause toàn thế giới. `engine_physics_frames` cho thấy sự khác biệt này. Không dùng wall clock Python cho timeout mission.

## Evidence và giới hạn

`results.json` chứa assertions/metrics; `observations.jsonl` giữ đầy đủ pose, scan, stamps, request và camera SHA256 (bỏ payload base64 để gọn); các frame quan trọng được lưu PNG. `mission-states.json`, `performance-samples.json`, stdout/stderr, engine log, CLI/PID/exit codes, source snapshots và SHA256 trước/sau nằm cùng output. Performance được đánh dấu **MEASURED**, không dùng kết quả đó để tự đánh dấu performance acceptance PASS.

Body 0.4×0.4 m dùng `CharacterBody3D.move_and_collide`, lý tưởng trên mặt phẳng: collision translation có thật; không có wheel/motor/slip/inertia/braking dynamics, gravity hay sweep khi xoay. Camera màu chỉ là detector fixture; người/cảnh khác cần perception mới. Quỹ đạo reveal trong test occlusion là scripted, không phải planner né tường. Quyết định nền tảng và các hạng mục UNVERIFIED nằm trong báo cáo.
