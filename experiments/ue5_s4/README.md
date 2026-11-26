# Native UE SAR benchmark

Một backend UE 5.6.1 / D3D11 / PythonScriptPlugin. Primitive diagnostic và human SAR dev dùng chung runtime/evaluator. Human profile có indoor/outdoor nguyên bản, damaged walls, debris, occluders, người đứng/ngồi/nằm, UGV visual và Mecanum kinematics. Người còn stylized, sensors synthetic, robot swept kinematic; chưa phải photorealistic corpus hay physical JetAuto. [Audit và evidence](../../docs/research/s4-audit.md).

## Chuẩn bị và chạy

Python cần NumPy/OpenCV có DNN ONNX và EXR codec. Evaluator cơ bản chỉ cần stdlib. Dùng packages hiện có, không upgrade production dependencies. UE cần PythonScriptPlugin và EditorScriptingUtilities theo project đã có.

```powershell
$sarPython = 'C:/Users/lemin/AppData/Local/Programs/Python/Python310/python.exe'
& $sarPython experiments/ue5_s4/prepare_assets.py
& $sarPython experiments/ue5_s4/bake_humans.py
& $sarPython -m unittest discover -s tests/sar_benchmark -v
& $sarPython -m sar_benchmark.runner `
  --editor 'C:/Program Files/Epic Games/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe' `
  --out experiments/ue5_s4/runs/my-human-pilot --suite sar-pilot `
  --environments indoor outdoor --cases clean occluded absent multi obstacles blocked `
  --methods frontier core verify --ticks 60 --action-dt .5 --calibrate
& $sarPython -m sar_benchmark.audit_run experiments/ue5_s4/runs/my-human-pilot `
  --out experiments/ue5_s4/runs/my-human-pilot-audit.json
```

Downloader có byte caps và expected SHA256: CesiumMan khoảng 0,46 MB, official ONNX khoảng 3,98 MB cùng license notices. Không tải environment packs hoặc thay production checkpoint. [Attribution](assets/README.md) đi cùng source/derived OBJ. Imported assets/materials dùng tên theo content hash; generated uasset/shader caches ignored.

sar-pilot thêm clean/P/N/P+N ghép cặp bên cạnh cases đã chọn. Mặc định một seed, standing; --postures standing seated prone mở cả ba tư thế. sar-smoke chỉ chạy cases chọn. --size 640x480 --scan-rays 720 cấu hình độ phân giải thiết kế; --depth-model ideal|gaussian|structured chọn response synthetic. Indoor/outdoor hiện chung một layout lineage. blocked là auxiliary infeasible navigation challenge, báo riêng khỏi feasible positive corpus.

Lệnh trên là **30 s dev task tại 2 Hz**, khác S3 300 s/10 Hz. Trên graphics worker đủ tài nguyên, lệnh sau yêu cầu cấu hình dài; phải đo throughput/watchdogs và nghiệm thu trước khi dùng kết quả nghiên cứu:

```powershell
& $sarPython -m sar_benchmark.runner `
  --editor 'C:/Program Files/Epic Games/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe' `
  --out experiments/ue5_s4/runs/worker-human-dev --suite sar-pilot `
  --methods frontier core verify --postures standing seated prone --seeds 3 `
  --size 640x480 --scan-rays 720 --ticks 3000 --action-dt .1 --render-quality standard --rhi d3d12 `
  --admissibility --calibrate
```

--admissibility chạy trước policies: conservative circular-footprint 2D connectivity và hai native captures/viewpoint, không dùng detector để chọn scene. Rejected probes giữ private; positive case không đạt dừng run, blocked giữ auxiliary. Chưa là wheel-dynamics reachability certificate. --calibrate chạy trusted engineering suite riêng; forced contacts diagnostic không gộp policy scores.

Render preset preview giữ scalability thấp cho Iris Xe; standard bật scalability level 2 và FXAA/shadows, vẫn không phải photorealistic corpus hoặc Lumen acceptance. RHI mặc định D3D11; D3D12 là cấu hình worker chưa được nghiệm thu trên máy này. Actual CVars và requested RHI được lưu riêng; đối chiếu RHI thật trong engine log.

--overview lưu góc nhìn robot/scene riêng trong private, phục vụ QA; không chuyển cho policy và không dùng làm scoring evidence.

## Runtime và dữ liệu

Output directory phải mới. Runner chỉ launch/đóng process nó tạo; không chạm ROS/SSH/robot hoặc project UE khác. Không chạy đồng thời hai runner vào cùng authoring project. Editor render cap 10 FPS giảm tranh CPU với inference; cap này không phải sensor frequency/physics clock.

- public/observations.jsonl.gz: RGB8, depth Z mét/null, range/status, estimated pose, K/extrinsic và task timestamps. Không target coordinates, masks, victim count hoặc correctness ACK.
- private/host.jsonl.gz: lossless native PNG/EXR, isolated depth masks, GT anchors, actual pose, scan và software capture interval/availability. Visibility dùng giao rendered silhouette và full-scene Z, không dùng detector boxes làm GT.
- Private scenarios, ledgers, commands, worker exits; source snapshot chỉ giữ scripts/config nguồn. Không snapshot Saved/CrashReportClient/shader settings.
- Manifest, SHA256 inventory, chain journal và engine-log.txt giữ provenance. capture-tmp là scratch; bytes PNG/EXR tương ứng đã trong private raw log.

Scene tĩnh trong inference/capture; integer task time tăng .1/.5 s mỗi transaction. Hai camera cùng pose/K; RGBA16F depth cm đổi sang optical Z mét. Raw timestamps là software readback intervals, chưa là exposure time từng sensor. Chưa có per-ray time, IMU/encoders, empirical Astra/G4 response hoặc global engine lockstep.

Robot có .4 m square root collider, swept translation từng .01 s và rotated chassis. Mecanum inverse/forward/saturation là kinematics. Wheels/mast là visual children; chưa có roller contacts, inertia/friction/slip/traction/braking. Human safety dự báo circle sweep ở các increments .01 s qua trọn command interval và cho phép retreat zero-yaw khi clearance tăng. Diagnostic giữ gate cũ. Đây là bảo vệ từ planar measured rays trên proxy footprint, chưa chứng nhận physical JetAuto.

Loopback RPC có token không chuyển cho worker, response decompress cap. Worker vẫn cùng Windows user/filesystem; base replicate seed dùng chung cho algorithm và fault streams có domain separation. Chưa có OS isolation/seed secrecy. scientific_main_eligible luôn false.

## Thuật toán và sensitivity

Frontier chỉ nhận scan/estimated pose; core dùng actual SearchMission/Navigator/Costmap. Ba methods dùng cùng official YOLOv5n v7.0, letterbox/NMS, median torso depth, transforms, report budget và safety. verify thêm translated-view evidence/stability, không GT feedback. Covariance là sample dispersion có synthetic floor, chưa calibrated uncertainty. Random/sweep là sanity, red detector chỉ ở diagnostic profile.

Replay sau cố định RGB, boxes và estimated poses, thay native depth bằng ba synthetic profiles. Đây là secondary sensor-only sensitivity, không thay closed-loop RQ3; Gaussian chưa RMSE matched và chưa có Astra data:

```powershell
& $sarPython -m sar_benchmark.sensor_study experiments/ue5_s4/runs/my-human-pilot `
  --model experiments/ue5_s4/assets/source/yolov5n-v7.onnx `
  --out experiments/ue5_s4/runs/my-human-sensor-sensitivity.json
```

RescueBench/UnrealZoo/Project AirSim đã được xem xét; không vendor scene binaries hoặc claim chạy upstream runtime. Historical evidence giữ nguyên. Run dài/nặng ở runs/ được ignored; không commit weights/caches hay files quá giới hạn dịch vụ. Artifact index phải chỉ rõ raw local paths/hashes và phần evidence thực sự mang theo repository.

## Replay và source freeze

Copy source snapshot sang workspace mới để replay; không chạy UE trực tiếp trong evidence archive. Khôi phục asset/model bytes và pose manifest đúng hashes của run; dùng committed OBJ bytes khi cần giữ EOL/hash, không suy bitwise cross-platform từ việc rebake. Dùng output directory mới, các runner arguments/private scenario configs đã lưu, rồi chạy independent audit. Partial archive cần original local raw files theo archive-provenance.json; không tự xem partial metadata là portable raw dataset.

Manifest mới ghi runner_arguments và private/scheduled-scenarios.json trước admission để giữ cả rejected inputs. Các lượt cũ thiếu fields này vẫn có per-episode configs/source snapshots; giới hạn metadata được giữ nguyên.
