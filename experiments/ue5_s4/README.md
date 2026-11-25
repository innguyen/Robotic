# S4 UE5 static SAR slice

Một backend thực: **UE 5.6.1 / D3D11 / PythonScriptPlugin**, scene primitive tự dựng, robot omni kinematic với swept collision. Đây là **diagnostic fixture**, không phải benchmark photorealistic hoặc JetAuto có dynamics. [Audit S4](../../docs/research/s4-audit.md) ghi kết quả và giới hạn.

## Chạy

Không cần cài thêm packages cho evaluator hoặc hai policy `random`, `sweep`. `core` dùng NumPy trong Python hiện có. Chạy từ repository root với output directory mới:

```powershell
& C:/Users/lemin/AppData/Local/Programs/Python/Python310/python.exe -m unittest discover -s tests/sar_benchmark -v
& C:/Users/lemin/AppData/Local/Programs/Python/Python310/python.exe -m sar_benchmark.runner `
  --editor 'C:/Program Files/Epic Games/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe' `
  --out experiments/ue5_s4/runs/my-pilot --suite pilot --ticks 30 --seeds 2 --methods random sweep core
& C:/Users/lemin/AppData/Local/Programs/Python/Python310/python.exe -m sar_benchmark.audit_run `
  experiments/ue5_s4/runs/my-pilot --out experiments/ue5_s4/runs/my-pilot-audit.json
```

`--suite smoke` chạy một clean case/method. `pilot` chạy clean/P/N/joint, occluded, absent, multi và sensor-black. `--ticks 30` là **3 s host task time**, không phải 300 s S3. Tăng `--ticks` tối đa 3000 khi cần; scene chưa được nghiệm thu cho main scientific mode. Không suy throughput từ thông số editor FPS.

Thêm `--calibrate` để kiểm vx/vy/yaw/zero, 10 resets và forced wall contact trong **engineering fixture riêng**. Lượt này gọi trusted backend trực tiếp để kiểm engine sweep; không gộp forced contact vào policy scores. Raw data và `calibration.json` vẫn được giữ đầy đủ.

Runner tự launch/đóng **process do nó tạo**, không sửa project `A:/RoboticUE5Test`, không kết nối ROS/SSH/robot. UE tự ghi AppData/cache thông thường. Project nguồn không cần `.umap` hay downloadable assets: `editor_server.py` dựng scene và tạo material; `editor_host/Content` được sinh lại và ignored. Không chạy đồng thời nhiều runner vào cùng project authoring này.

## Dữ liệu và ranh giới

- Agent chỉ nhận JSON qua stdin: RGB8, depth Z mét/`null`, range scan/status, synthetic odometry, calibration, timestamps, action limits. Không có scenario, seed của fault, target pose, mask, actor IDs, reward hoặc evaluator feedback.
- Python policy là process riêng; **cùng OS user và filesystem**, chưa có sandbox chống agent đối kháng. Listener đặc quyền chỉ bind loopback, dùng token riêng không chuyển cho worker. Không mở engine/raw host port cho agent. Tất cả run được gắn `scientific_main_eligible=false`.
- `public/observations.jsonl.gz`: payload đã phát thực, lossless. `private/host.jsonl.gz`: clean captures, GT/visibility và actual motion; các ledger, scenario và commands nằm private. `source/`: snapshot mã thực thi. `manifest.json`, chain journal, engine log và `inventory.json` giữ provenance.
- `engine-log.txt` là bản sao nguyên byte của log UE; `.log` và `__pycache__` không thuộc inventory portable ở runner cuối. Các lượt thử cũ lưu nguyên để audit, có thể chứa inventory local-only.
- Hai camera native cùng FOV/transform ở scene tĩnh, RGB 80×60 và depth RGBA16F Z/100. Range scan 72 collision rays là measurement lý tưởng, không phải LiDAR G4 đã hiệu chuẩn. Không có IMU, wheel encoder hoặc dynamics bánh xe.
- Sau reset/move, host chờ các editor frames rồi capture. Time ns trong packet là **transaction task clock**; `capture_wall_ns` riêng ở private raw log. Không nhận global physics lockstep. Geometry/victims tĩnh; thay scene động phải có clock/sensor barrier khác và profile mới.
- Detector chạy pilot là **red connected-components**, ảnh thật của primitive surrogate. Không sử dụng GT để nhận diện. `perception.py` có adapter ONNX person và được kiểm inference, nhưng checkpoint provenance và mannequin/person scene chưa đủ để dùng làm main baseline.

## Code tái sử dụng

`CoreSearchPolicy` import `SearchMission`, `Navigator`, rolling costmap và cấu hình footprint từ `test version/`; target estimate đến từ RGB/depth public. Không import S2 oracle recognition. `footprint_separation` của S2 dùng riêng cho assertion hình học. Common safety gateway áp lên cả ba policy. `OnnxPersonDetector` port phần preprocessing/YOLOv5 decoding từ production, không import module có side effects.

RescueBench/UnrealZoo/Project AirSim đã được audit trước khi quyết định; không vendor scene binary hoặc claim đã chạy các framework đó. Native UnrealCV build thử bị blocker; release này giữ **một host UE native**, không để nhiều integration candidate chưa dùng trong source. Xem lý do và commit trong audit.
