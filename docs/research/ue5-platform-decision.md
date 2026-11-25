# UE5 platform decision và realism strategy

**ADR S3-001, 2026-11-25 — DESIGN, chưa nghiệm thu runtime.** Chọn **Project AirSim v1.1.0 + UE5.8** làm candidate nền Robotics chính; tái sử dụng có chọn lọc ý tưởng/API/baseline wrappers của UnrealZoo/RescueBench, và chỉ dùng scene assets có quyền + source/cook phù hợp. Quyết định này dựa trên nhu cầu sensor/control extensibility, không phải benchmark FPS đã đo. Không cài đặt hoặc tải các bộ scene lớn trong S3.

## 1. So sánh dựa trên phiên bản hiện hành

Không đồng nhất Microsoft AirSim cũ, AirSim-Colosseum, Project AirSim và UnrealZoo. ESARBench dùng Colosseum; điều đó không chứng minh API/cấu hình của nó chạy nguyên trên Project AirSim. Tên `car` trong AirSim cũ cũng không chứng minh Mecanum ở Project AirSim mới.

| Tiêu chí / trọng số quyết định | UnrealZoo / RescueBench | Project AirSim | Hệ quả cho dự án |
| --- | --- | --- | --- |
| Sensor/clock extensibility — **cao nhất** | UnrealCV/Python/Gym; RGB-D, v3.1 có LiDAR. MuJoCo examples không chứng minh global lockstep cho mọi actor | Sensor config, client/server và physics extension; có camera/LiDAR/IMU, clock API. Native vehicle engine clock có giới hạn pause | Cả hai cần bridge riêng bảo vệ GT và kiểm sensor stamps; ưu tiên nền robotics APIs của Project AirSim |
| UGV/control — **cao nhất** | Có vehicles/robots và Chaos/MuJoCo examples; chưa xác minh JetAuto/Mecanum API hoặc controller tương thích core | v1.1.0 có native `AWheeledVehiclePawn`, Rover/SimpleDrive throttle/steering/brake; custom controllers/actuators | UGV **có hỗ trợ**, nhưng SUV Ackermann không phải JetAuto omni. Main holonomic adapter vẫn phải xây và kiểm |
| Scenes/victims — cao | Lợi thế scene collection, agent/victim interactions và SAR baseline tooling | Blocks/city/landscape packages, không có sẵn SAR victim protocol của nhóm | Dùng một scene có quyền trước. Không cần carry/drop/return của RescueBench |
| Sensor realism — cao | Có raw modalities, chưa thấy calibration Astra cụ thể | Camera response config; LiDAR nhiều kiểu nhưng docs gọi sensor lý tưởng | Cả hai chưa cung cấp empirical Astra/JetAuto validation cho dự án này |
| Python / extension — cao | Gym trên UnrealCV; engine queries có quyền mạnh | Python/C++/ROS2; robot/sensor/core C++ + Unreal plugin | Python agent dùng facade allowlist, server giữ API đặc quyền; ROS2 bridge không bắt buộc cho vòng offline |
| Reproducibility — cao | Branch/binary/UE versions khác nhau, update tính năng tùy package | Source/tag/client/binaries khác nhịp, native Chaos stepping riêng | Freeze source, plugin, binary, engine và renderer thành một compatibility tuple |
| License/release — gate | Code Apache-2.0; scene Marketplace và baseline phụ thuộc quyền riêng | Code MIT; UE, SUV/scene và third-party notices riêng | Không bên nào cho giấy phép phát hành mọi asset nhờ license code |
| Công sức của dự án — vừa | Nhanh cho navigation/victim demo; empirical sensor và custom UGV vẫn tốn công | Nhanh cho Robotics sensor/config/control smoke; authoring SAR và omni robot cần làm | Chọn nền robotics, không fork toàn bộ RescueBench task để sửa dần thành bài khác |

Đây là rubric ưu tiên định tính, không phải chấm điểm thực nghiệm hoặc bằng chứng Project AirSim “chân thực nhất”. Nếu candidate không vượt clock/UGV/sensor gates ở §6, giữ contract/evaluator, thử UnrealZoo/UnrealCV với cùng gates; không âm thầm đổi task để hợp API có sẵn.

## 2. Evidence ledger và compatibility

| Nguồn đã kiểm | Điều được chứng minh từ tài liệu/source | Chưa chứng minh |
| --- | --- | --- |
| [Project AirSim v1.1.0 release](https://github.com/iamaisim/ProjectAirSim/releases/tag/v1.1.0) | Native wheeled vehicles và UE5.8 được release notes mô tả; Python client 1.1.0. Trang release có asset links nhưng phần mô tả cũng còn dòng packages pending verification | Binaries tương ứng đã chạy, từng asset chứa đủ features, checksum download có khớp; không nhận PASS từ release note |
| [Native wheeled vehicles](https://iamaisim.github.io/ProjectAirSim/wheeled_vehicle.html), [tagged example](https://github.com/iamaisim/ProjectAirSim/blob/v1.1.0/client/python/example_user_scripts/hello_wheeled_vehicle_simpledrive.py) | Dùng `Rover`, `move_on_path_async`, `set_rover_controls`, `wait_until_stopped_async`; native Chaos path và direct-force fallback phải phân biệt | Omni/Mecanum, low-speed braking, contacts và fixed stepping trên máy đích |
| [Scene clock docs](https://iamaisim.github.io/ProjectAirSim/config_scene.html) | Steppable clock có pause; engine-driven host/Chaos owns pacing và không hỗ trợ SimClock pause/resume | Manual-step drone example bảo đảm toàn bộ UGV/Chaos scene dừng/step đúng |
| [Camera capture docs](https://iamaisim.github.io/ProjectAirSim/sensors/camera_capture_settings.html) | RGB, planar Z depth, perspective range, segmentation, normals; imaging settings/exposure | Structured-light Astra measurement process, transparent material response và RGB/depth exact-time sync |
| [LiDAR docs](https://iamaisim.github.io/ProjectAirSim/sensors/lidar.html) | Mesh-trace và GPU/depth scan types; mesh trace không mô phỏng physical intensity | G4 beam/time/material response hoặc scattering trong khói; intensity render không phải calibrated reflectance |
| [UnrealZoo current](https://github.com/UnrealZoo/unrealzoo-gym), [v3.1 changelog](https://github.com/UnrealZoo/unrealzoo-gym/blob/v3.1/doc/CHANGELOG_v3.1.md) | Branch/binary v3.1 dùng UE5.7, trước đó v3.0 UE5.6; v3.1 có XYZI và camera/physics tooling. Changelog cảnh báo Unreal world cm khác LiDAR m | Binary v3.1 nạp vào Project AirSim UE5.8; LiDAR response calibrated hoặc Mecanum control |
| [RescueBench interface paper D.2](https://arxiv.org/html/2606.01848v1) | UE5.6/UnrealCV; forward/yaw và interaction gates; input segmentation trong protocol paper | Cùng GT boundary với benchmark main của nhóm |

Candidate tuple để thử: `ProjectAirSim tag v1.1.0`, `client 1.1.0`, `UE 5.8 exact patch/build TBD`, compatible plugin built from same source, Python 3.11 env riêng, Windows 11 hoặc Ubuntu 22.04 được upstream mô tả. [Supported platform docs](https://iamaisim.github.io/ProjectAirSim/). Tag/branch chưa resolve thành full commit và package chưa tải nên đây **chưa phải lockfile**. Release hiển thị commit prefix `be18449`; không dùng prefix này thay hash đầy đủ trong run manifest.

UnrealZoo tuple thử độc lập khi cần: branch v3.1 + matching UE5.7 package + UnrealCV client tương thích; RescueBench UE5.6 là tuple thứ ba. Không nạp cooked UE5.7 `.pak` vào UE5.8 rồi mặc định hợp lệ. Reuse scene qua **source assets có quyền**, migrate/cook lại, test colliders/materials và hash. Nếu chỉ có binary hoặc thiếu source/license, dùng nó như external reference environment với backend riêng hoặc bỏ scene đó. Không cần hai framework cùng tồn tại trong một UE project để bắt đầu.

## 3. Scenes và victim assets

Authoring order đã chốt theo mục tiêu khoa học: một calibration room → một room/corridor có partial occlusion → kiểm bốn P/N cells → mới mở rộng families. Scene đầu có sàn, hai material planes, một tường opaque có cửa rộng, một hộp thấp, một victim ngồi/nằm và một start ngoài FOV. Full render images lấy từ camera sensor, không từ cinematic viewport. Scene nhỏ đủ chẩn đoán depth/frame/report trước khi dùng scan photorealistic.

Ưu tiên asset tự dựng/photogrammetry có quyền hoặc CC0/CC-BY với attribution rõ ràng. Có thể dùng scan building interiors/industrial props và HDRI/PBR assets nếu quyền phân phối phù hợp. Ghi metric scale, topology, LOD/Nanite settings, collision mesh, texture provenance, source site và render config. Mảnh rubble nhỏ không được chỉ có visual mà không có collision; collision quá thô phải nằm trong QC. 3DGS đẹp có thể thiếu geometry/material/visibility masks phù hợp nên không là collision hoặc metric depth GT tự động.

Victim asset ban đầu là mannequin có mesh/rig/torso anchor và collision capsule đơn giản kiểm được. Sau gate, thêm licensed human meshes có standing/seated/prone poses, clothing/body-shape/skin-tone diversity và held-out identities; không dùng dấu hiệu màu cố định. Không cần gory injuries. Chọn poses không xuyên sàn/props, masks không gồm bóng, anchor giữ nhất quán theo skeleton. Collider không lấn tường và không biến tissue dynamics thành claim. Fully occluded target không vào main denominator.

Visual quality profile cố định cho evaluation: resolution, anti-aliasing, tonemapping, exposure history, Lumen/Nanite/shadows, texture quality và streaming. “Low/High” phải có resolved console/config values. Warmup shader/streaming/temporal histories trước capture; reset history hoặc ghi residual. Không đổi Lumen giữa methods; nếu hardware tier phải giảm rendering thì thành track khác. Chất lượng ảnh và detection được đánh giá theo strata, không chỉ preference survey. Không tạo ảnh AI để thay sensor evidence.

## 4. Sensors và robot

**Camera RGB-D:** main design 640×480, 10 Hz; depth native/resampling và RGB alignment có K/distortion/extrinsic riêng. Không lấy RGB-depth cùng FOV mặc định vì Astra RGB/depth lens khác nhau. [Orbbec Astra](https://www.orbbec.com/products/structured-light-camera/astra-series/) mô tả structured light; phần mềm mô phỏng chỉ dùng raw planar depth làm clean geometry layer. Bổ sung response layer có range-dependent bias, quantization, edge flying pixels/holes, temporal correlation, exposure/noise, latency và invalidity; các tham số chưa đo phải ghi synthetic. Không dùng depth visualization 8-bit thay metric float. Kiểm plane/corner ở nhiều range và unprojection→projection roundtrip, kể cả pixel ngoại vi.

**LiDAR:** khởi đầu ray/mesh trace một layer hợp với G4, không dùng camera Z-buffer seam của Webots làm chuẩn thật. 720 bins/10 Hz/8 m là profile thiết kế, cần đối chiếu device settings thực. Ghi beam direction/time, sensor origin, valid/no-return, motion distortion và deskew source. Noise/material dropout/intensity phải có evidence nếu gọi realistic. Fog volumetric trong RGB không tự giảm range, tạo backscatter hoặc mô phỏng IR; khi chưa có optics model, label **synthetic visibility stress**, không label physical smoke sensor simulation.

**Robot:** ba profile tách biệt:

| Profile | Mục đích và phần cần làm | Claim được phép |
| --- | --- | --- |
| `platform_rover_smoke_v1` | Native SUV/Rover từ Project AirSim, throttle/steering/brake; xác minh server–Python–camera/depth/reset trước | Foundation UGV API smoke, **không** JetAuto hoặc main benchmark |
| `ugv_omni_proxy_v1` | Footprint/sensor mast kiểu JetAuto với body velocity/acceleration model, collision-respecting implementation riêng. Không teleport trong mission | Main search/perception conclusions có điều kiện theo ideal omni; không traction/Mecanum transfer |
| `jetauto_mecanum_physical_v1` | Đo wheel radius/track/wheelbase/roller orientation, mass/inertia, friction/slip/encoder/actuator response; custom controller/dynamics hoặc kiểm giải pháp upstream phù hợp | Chỉ claim physical fidelity sau validation thật; hiện UNVERIFIED |

Không ép wheel robot vi sai/Ackermann nhận holonomic command rồi bỏ vy. `Navigator` hiện có thể xuất vy; phải chọn proxy phù hợp hoặc xây local controller nonholonomic và re-evaluate safety trên command thực. Engine Chaos hoặc asset mang tên “robot” không là bằng chứng Mecanum đúng. Các kích thước body/extrinsics trong production là đầu vào đo lại, khác body 0,4 m của S2.

## 5. License và phân phối

Code licenses đã đọc: [UnrealZoo Apache-2.0](https://raw.githubusercontent.com/UnrealZoo/unrealzoo-gym/v3.1/LICENSE), [RescueBench Apache-2.0](https://raw.githubusercontent.com/UnrealZoo/RescueBench/main/LICENSE), [Project AirSim MIT](https://github.com/iamaisim/ProjectAirSim/blob/main/LICENSE). Chúng không cấp quyền chung cho Marketplace scenes, SUV pack, motion clips, pretrained weights hoặc dữ liệu participant. Kiểm `NOTICE` và third-party licenses cho đúng commit. Không tự relicensing cả dependency graph bằng license code dự án.

UE là proprietary engine, chịu [UE EULA](https://www.unrealengine.com/eula/unreal), có các ngoại lệ seat cụ thể cho educational/noncommercial use nhưng không tự miễn mọi điều kiện phân phối. [Fab Standard License](https://www.fab.com/eula) phân biệt incorporated project với standalone asset; cần ghi quyền source/cooked binary/rendered dataset/training riêng cho từng asset. **Cập nhật ngày 29-10-2026** của Epic đã bỏ một số hạn chế AI/MetaHuman khi chấp nhận EULA mới; không lặp lại lệnh cấm MetaHuman cũ như thể còn áp dụng cho mọi người. Fab NoAI và custom/legacy agreements vẫn cần kiểm theo asset/account. [Thông báo chính thức](https://www.unrealengine.com/ue-devs-can-now-choose-to-train-ai-models-on-their-projects).

Asset manifest bắt buộc: source URL/author/version/hash, license text/date, attribution, quyền AI training/testing, quyền distribute source/cooked/rendered frames, restrictions/NoAI, acquisition account/team rights và reviewer. Nếu không rõ → exclude khỏi public core corpus, không phát hành file. Một benchmark có test scene chỉ trong proprietary binary phải công bố hạn chế tái lập; ưu tiên một core scene pack có nguồn có thể dựng lại.

## 6. Hạ tầng và vertical slice acceptance

[Epic requirements](https://dev.epicgames.com/documentation/unreal-engine/hardware-and-software-specifications-for-unreal-engine) khuyến nghị 32 GB RAM, graphics RAM≥8 GB; đây không phải cam kết FPS của SAR. Dự án đề xuất workstation **32–64 GB RAM, dedicated GPU 12–16 GB VRAM có Vulkan/DX12/SM6 phù hợp, SSD còn trống 200–300 GB** cho source/build/cache/scenes; số storage/VRAM này là dự phòng kỹ thuật, phải đo scene thật. Nếu inference chung GPU cần đo VRAM contention; có thể tách inference máy khác nhưng phải log network latency. Không mua/cấp máy trong S3.

Headless/offscreen phải giữ GPU rendering và sensor readback. `NullRHI` hoặc runtime không rendering không thể nghiệm thu RGB-D. [Project AirSim Runtime docs](https://iamaisim.github.io/ProjectAirSim/) nêu không có rendered cameras/LiDAR/world mesh collisions như Unreal host. H200/MIG có CUDA không chứng minh Vulkan/DX sensor rendering; [NVIDIA](https://docs.nvidia.com/datacenter/tesla/mig-user-guide/620/deployment-considerations.html) giới hạn graphics của MIG, H200 không thuộc các profile +gfx được liệt kê. Không tự đổi mode server trường.

**Trạng thái máy S3:** chưa phát hiện UE Editor qua PATH/env/common locations; GPU live qua CIM bị access denied; evidence Godot cùng ngày báo Iris Xe/8 GB, xem [preflight](evidence/preflight.json) và [audit](s3-audit.md). Không có graphics worker UE được cấp/kiểm. Vì vậy không launch giả, không dùng screenshot upstream làm runtime evidence. Các bước sau đều **UNVERIFIED**, là recipe thi công trên worker phù hợp:

| Gate | Hành động trên worker | Evidence và điều kiện đạt |
| --- | --- | --- |
| G0 compatibility | Resolve full source commit v1.1.0, engine exact build, plugin/client 1.1.0; build hoặc tải một Blocks package kiểm SHA256. Ghi license và actual GPU/RHI | Fresh env import, source/build log, binary hashes, camera không chạy NullRHI; tag alone chưa đủ |
| G1 small scene | Blocks + native Rover/SUV + một camera RGB và planar depth; victim mannequin tự có quyền, tường và hộp | Sensor frames lossless, dims/K/depth plane có kiểm analytic, actual robot spawned có contact; không cinematic-only |
| G2 Python/control | Chạy tagged `hello_wheeled_vehicle_simpledrive.py` trên sim riêng; sau đó bounded slow commands và brake | Desired/actual motion, timestamps, local client/server logs; không kết nối hardware. Không dùng tốc độ 4 m/s của upstream example làm default robot gần người |
| G3 clock/reset | Probe cả pause/step và native engine-driven mode; moving clock actor kiểm toàn scene, sensor timestamps và cached-frame flush | ≥10 reset/reload traces; epoch/barrier đúng, actuator zero, physics/renderer đều tiến đúng ticks. Nếu engine-driven không pause: implement host tick barrier hoặc chỉ nhận deployment timing smoke, chưa nhận main lockstep |
| G4 minimal evaluator | Independent GT lane chấm một report thật, duplicate, absent, occluded/reveal và false-position; telemetry không lộ poses/IDs GT | Raw public/private packet captures, offline recompute, matched results/false reports, evaluator source hash; oracle/camera fixture detector label rõ |
| G5 main robot/perception | Thêm collision-respecting omni proxy + RGB detector/depth localizer, common safety; test vx/vy/yaw, faults, return buffers/reset | Main profile acceptance, không lấy Rover smoke làm main PASS; physical Mecanum vẫn gate riêng |
| G6 performance/repeat | 30-min soak small scene, cold/warm launches, offscreen rendered sensor test, command-trace repeat và một worker thứ hai | RTF/p95/VRAM/RAM, warnings/crashes, tolerances và payloads; không lấy spec/vendor FPS làm kết quả của nhóm |

G0–G4 tạo **vertical slice thật**; G5 cần trước khi chạy main research protocol. Sau gate có thể đặt source ở `experiments/ue5_s3/` với pinned integration, runner và tests; S3 hiện chỉ thêm docs/contracts/tools, không tạo `.uproject` rỗng để gây ấn tượng đã tích hợp.

Tất cả acceptance logs phải là run mới không ghi đè, có process IDs do runner sở hữu, exit codes và source identities. Lỗi Clock/Chaos là blocker kỹ thuật của candidate; không đổi timestamp logical như Godot S0 để che world vẫn chạy. Chỉ triển khai scenes rộng hơn khi bốn condition pairs và report evaluator đã vượt gates.
