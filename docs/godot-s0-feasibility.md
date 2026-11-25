# Godot S0 — Game Engine Simulator Feasibility

Ngày nghiệm thu: **2026-11-25**, Asia/Saigon. Repository `innguyen/Robotic`, nhánh `feat/benchmark`, HEAD đầu việc `2a81052895dc1ace48628903e4f3183f5ea78cb8`.

**GO cho feasibility prototype Godot trên laptop hiện tại; NO-GO cho chuyển nền tảng chính sang Godot ở thời điểm này. Khuyến nghị giữ Webots làm simulator Robotics chính.** Godot đã thực sự chạy Compatibility trên Intel Iris Xe, cung cấp ảnh RGB/raycast qua Python, reset và hoàn thành nhiệm vụ tìm–phát hiện–tiếp cận–dừng tối thiểu. Tuy nhiên, phần Robotics phải tự xây còn đáng kể; S0 chưa có bằng chứng để thay thế integration và **7/7 scenario PASS của Webots S2**.

Đây là kết luận về phạm vi đã kiểm, không phải đánh giá Godot không phù hợp Robotics nói chung. Không tích hợp YOLO, không port SearchMission/Navigator, không sửa production ROS, T1 hoặc Webots S0–S2. Các thay đổi Webots có sẵn khi bắt đầu vẫn được giữ nguyên.

## 1. Artifact và môi trường thực thi

Project độc lập tại [experiments/godot_s0](../experiments/godot_s0/), với [hướng dẫn chạy và wire protocol](../experiments/godot_s0/README.md).

| Thành phần | Implementation |
| --- | --- |
| Project/entry scene | [project.godot](../experiments/godot_s0/project.godot), [main.tscn](../experiments/godot_s0/main.tscn) |
| Environment, robot, sensors, TCP server | [simulator.gd](../experiments/godot_s0/simulator.gd) |
| Python bridge và detector màu tối thiểu | [bridge.py](../experiments/godot_s0/bridge.py) |
| Runner, observer/assertions, metrics | [run_s0.py](../experiments/godot_s0/run_s0.py) |
| Protocol/detection regression tests | [test_bridge.py](../experiments/godot_s0/test_bridge.py) |

Đã dùng Godot **4.7.2.stable.official.ed1daf0bf**, binary `C:\Godot\Godot_v4.7.2-stable_win64.exe`, renderer `gl_compatibility`, Windows 11 build 26300. [Engine stdout thực](../experiments/godot_s0/evidence/run-05-final-rendered/stdout.txt) ghi **OpenGL 3.3.0, Intel Iris Xe, driver 31.0.101.5186**. Python **3.12.11 x64**, `C:\msys64\ucrt64\bin\python.exe`, chỉ standard library. Launcher Python 3.13 trên máy đang trỏ tới `C:\Python313\python.exe` không còn tồn tại; S0 không sửa installation này.

RAM vật lý OS báo **8.215.101.440 byte (~7,65 GiB)**, phù hợp cấu hình RAM 8 GB. Khi bắt đầu lượt rendered cuối, RAM available khoảng **947 MiB**; không dọn app hoặc dựng môi trường máy trống. Xem [invocation/hardware/source identities](../experiments/godot_s0/evidence/run-05-final-rendered/invocation.json).

Project được tạo trong repo, không sử dụng hoặc sửa project Forward+ đã có ở `C:\Projects\robotic-godot-benchmark`. Không cài thêm package, asset hay GDExtension. Godot hỗ trợ Compatibility dùng OpenGL cho phần cứng desktop yếu; việc chọn renderer này dựa trên cấu hình laptop và đã xác nhận qua runtime. [Godot: Overview of renderers](https://docs.godotengine.org/en/stable/tutorials/rendering/renderers.html).

## 2. Mô hình simulator

### Environment và robot

Phòng 10×10 m, sàn, bốn tường ngoài, một mannequin capsule đỏ và một tường chắn tùy fixture. Tất cả dùng primitive mesh/collider, material unshaded, không texture, shadow, GI, particle hay asset nặng. Cửa sổ overview 960×640 và preview camera 320×240; sensor gốc 160×120.

Robot xanh có footprint 0,4×0,4 m, cao 0,2 m; tâm chassis cao 0,11 m. `CharacterBody3D` nhận `(vx,vy,wz)` trong base FLU, có cả dịch ngang. `move_and_collide` kiểm translation sweep với static colliders; khi bị chặn chỉ đi tới điểm contact, không teleport qua tường. Yaw và displacement tích phân theo `dt=1/60 s`, yaw được cập nhật trước dịch chuyển.

**Đây là ideal kinematic omni body.** Không có motor, roller Mecanum, mass/inertia, gravity, traction/slip, wheel encoder, braking curve hoặc lực tương tác với người. Không sweep collider khi xoay: rotation gần góc/tường và dynamic-body interactions **UNVERIFIED**. Dừng zero ở S0 không chứng minh phanh vật lý. Godot có RigidBody3D và physics engine sẵn, nhưng phải tự thiết kế robot actuator/dynamics nếu muốn dùng chúng cho Robotics. [Godot: Physics introduction](https://docs.godotengine.org/en/stable/tutorials/physics/physics_introduction.html).

### Frames và sensors

World ENU; base FLU: +X tiến, +Y trái, +Z lên, yaw dương CCW, origin tại tâm chassis cao 0,11 m. Mapping Godot `(X,Y,Z)=(east,up,-north)`. Self pose là transform lý tưởng của robot từ engine, **không phải SLAM/odometry thực**.

| Sensor | Dữ liệu thực có trong S0 | Phần tự phát triển / giới hạn |
| --- | --- | --- |
| Camera RGB | Camera3D → SubViewport → chờ `frame_post_draw` → RGB8; 160×120, horizontal FOV 1.012290966 rad; base origin `(0.12,0,0.44)` m, world height 0,55 m | Tự readback, đóng gói base64, gắn stamp, kiểm trạng thái; không có camera noise/exposure model hay recognition Robotics |
| Range scan | PhysicsDirectSpaceState3D raycasts vào collider: 192 bin, 360°, max 6 m; base origin `(0.12,0,0.14)` m, world height 0,25 m | Tự tạo scan angle/order, range semantics, timestamp/extrinsic; không phải native LiDAR device có lịch sample/noise/intensity |
| Pose | Engine ideal self pose, world ENU | Tự mapping frame/protocol; không có odometry error hoặc sensor fusion |

`angle_min=-π+π/192`, increment `2π/192`, không có hai tia trùng ±π. Ray không hit trong 6 m trả JSON `null`; không dùng Infinity không chuẩn JSON. Self collider bị loại khỏi ray query. Noise tắt; analytic calibration có 144 tia hữu hạn, 48 no-return do đường chéo phòng dài hơn 6 m.

Godot cung cấp camera rendering, viewport texture và ray query. Lịch sensor Robotics và Python transport là code S0. Ray query thực hiện trong `_physics_process`, theo yêu cầu truy cập physics space của engine. GPU → CPU readback và RGB base64 có chi phí; không coi đó là native zero-copy sensor stream. [Godot: Ray-casting](https://docs.godotengine.org/en/stable/tutorials/physics/ray-casting.html), [ViewportTexture](https://docs.godotengine.org/en/stable/classes/class_viewporttexture.html).

### Bridge và thời gian

TCP server bind **127.0.0.1**, một client, NDJSON v1. Request `reset`, `observe`, `step`, `quit`; response có request ID, epoch, logical simulation timestamp, pose, scan, RGB và diagnostics. `step` nhận vận tốc và số tick nguyên 1–600, giới hạn `|vx|≤0,6`, `|vy|≤0,4`, `|wz|≤1`. Payload sai/epoch cũ bị từ chối trước khi chuyển động; command chỉ tồn tại đúng số tick yêu cầu. Disconnect hủy phần command còn lại. Python client kiểm ID, đồng bộ stamps, finite pose/ranges và độ dài RGB.

`simulation_timestamp=tick/60`; reset tăng epoch và đưa tick/pose/command/collision count về trạng thái fixture. Scene không random; seed 7 là identity của fixture, không phải bằng chứng randomization reproducibility.

**Lockstep ở đây là logical stepping tự xây cho fixture tĩnh.** Engine physics vẫn tick khi chờ TCP; S0 không gọi move robot và không tăng logical tick trong thời gian chờ. `engine_physics_frames` trong log thể hiện engine tiếp tục chạy. Nếu thêm rigid bodies tự do, animation hoặc sensor theo engine time thì phải thiết kế lại pause/stepping toàn thế giới. Chưa có API tương đương Webots `robot.step()` cho toàn bộ thế giới động trong implementation này.

Sau tick cuối của request, robot được giữ đứng yên, scan/pose được lấy trong physics callback rồi RGB lấy sau render cùng pose đó; các sensor cùng logical stamp. Đây là snapshot đồng bộ của fixture, không phải timestamp thiết bị phần cứng độc lập. Trong headless dummy renderer, camera trả `valid=false`, `data=null`, không tạo ảnh giả.

## 3. Kiểm thử và bằng chứng

**Lượt nghiệm thu đồ họa cuối: 7/7 nhóm chức năng PASS**, thêm một performance sample **MEASURED**, exit Godot/runner đều 0. [Results](../experiments/godot_s0/evidence/run-05-final-rendered/results.json), [raw observations](../experiments/godot_s0/evidence/run-05-final-rendered/observations.jsonl), [invocation](../experiments/godot_s0/evidence/run-05-final-rendered/invocation.json).

| Nhóm kiểm | Assertion và kết quả thực |
| --- | --- |
| Clock/reset | Idle 0,25 s không tăng logical clock/pose; 60 step ticks = 1 s; reset trong cùng process xóa pose/yaw/clock/collision, tăng epoch; initial scan và RGB phục hồi |
| Calibration | So cả 192 rays với analytic room geometry, gồm no-return: max range error **5,58e-7 m** trên 144 tia hữu hạn; 8 actuator trials vx/vy/wz/kết hợp tại yaw 0 và π/2: max XY error **3,01e-6 m**; tolerance 2 mm, yaw 1e-4 rad |
| Collision/zero hold | Chạy thẳng vào tường x=1,5 m: chassis dừng ở **x=1,239861 m**, 117 tick báo collision, không xuyên qua; zero command giữ thêm 1 s, drift **0 m** |
| Repeat | Ba reset/trace giống nhau, tick/stamps trùng; max pose delta **3,87e-7**, max scan delta **1,43e-6 m**, trong tolerance 1e-5; RGB bytes giống nhau ở các checkpoint |
| Protocol/reconnect | Epoch cũ, overspeed, fractional ticks, sai vector length đều bị reject; tick/pose không đổi; đóng/mở TCP sau bounded command không làm robot tiếp tục trôi |
| Visibility/absent | Người sau tường ban đầu có **0 detection** trong RGB; dịch ngang scripted 2 m rồi xoay −0,6 rad thấy **1.134 red pixels**; không va chạm; fixture absent không false detection |
| Minimal mission | SEARCH → DETECT → APPROACH → STOP; **64 policy steps**, **6,4 s simulation**, base–person **1,05095 m**, không collision; zero giữ thêm 1 s drift **0 m** |

Mission dùng **red-pixel detector từ ảnh RGB thực**, hai observation liên tiếp để xác nhận, bearing từ camera intrinsics và range ray gần bearing đó. Policy không nhận target world pose/recognition oracle. Chỉ observer tính khoảng cách đến fixture person sau STOP để nghiệm thu. Camera/red capsule là fixture có chủ đích, không phải detector người dùng ngoài thực tế. Thao tác reveal sau tường là scripted để kiểm sensor occlusion, **không phải autonomous obstacle avoidance** hoặc bài SEARCH_EXPLORE của S2.

Frame thực: [bắt đầu mission](../experiments/godot_s0/evidence/run-05-final-rendered/mission-start.png), [dừng trước người](../experiments/godot_s0/evidence/run-05-final-rendered/mission-stop.png), [bị tường che](../experiments/godot_s0/evidence/run-05-final-rendered/occluded-before.png), [sau reveal](../experiments/godot_s0/evidence/run-05-final-rendered/occluded-after.png), [không có người](../experiments/godot_s0/evidence/run-05-final-rendered/absent.png). Đây là PNG encode lossless từ sensor RGB, không phải ảnh dựng minh họa.

**5/5 unit tests PASS**, kiểm socket response bị phân mảnh, request epoch, sai response ID, scan invalid/freshness, kích thước RGB và bearing detector. [Output](../experiments/godot_s0/evidence/run-05-final-rendered/unit-tests.txt). `compileall` các file Python thành công; engine đã parse và chạy GDScript thật trong hai chế độ.

Lượt [headless cuối](../experiments/godot_s0/evidence/run-06-final-headless/results.json) chạy lại clock/reset, calibration, collision, repeat và protocol; camera-dependent checks phải UNVERIFIED do dummy renderer. Kết quả camera/mission nghiệm thu lấy từ lượt rendered, không từ headless.

### Tính toàn vẹn evidence

Mỗi lượt cuối lưu CLI, PID, Python/platform/RAM, engine exit, runner exit, SHA256 source trước–sau và bản source thực đã chạy trong `source/`. Có SHA256 trước–sau của tracked Webots S0/S2, production/T1 trong `test version/` và các root Python/YAML/shell files; so sánh trên **working-tree bytes**, giữ cả chỉnh sửa đã có từ trước.

Raw JSONL giữ mọi accepted observation, command/tick và camera SHA256; bỏ base64 image payload để giảm dung lượng, giữ PNG ở các checkpoint. Không tuyên bố lưu toàn bộ video RGB. Repeat assertions đã so trực tiếp RGB bytes trong runtime. `mission-states.json` và `performance-samples.json` lưu state sequence và từng mẫu đo.

[Post-run verifier](../experiments/godot_s0/evidence/verify_final.py) đối chiếu lại source snapshots/current files, protected files, raw clocks/scan/command bounds và PNG→RGB SHA256; kết quả tại [verification.json](../experiments/godot_s0/evidence/verification.json).

Rà soát cuối **PASS** trên **362 accepted observations**, **5 PNG sensor frames**, **6 source files** và **472 protected tracked files**. Cả hai lượt cuối có Godot exit 0, runner exit 0, source/protected SHA256 trước–sau khớp; `compileall` và kiểm diff không phát hiện lỗi. Các untracked Webots GUI artifacts đã có trước vẫn giữ nguyên.

Các lượt chẩn đoán được giữ nguyên: `run-01-headless` bị ngắt trước khi chốt invocation và có lỗi assertion test trừ hai giá trị no-return `None`; không dùng làm nghiệm thu. `run-02-rendered` có mission PASS nhưng visibility FAIL: lateral 1,2 m chưa đủ thoát đường che của tường. Sửa quỹ đạo test thành 2 m rồi chạy lại; không nới detector hoặc cho target truth vào policy. Lượt cuối mới là cơ sở kết luận.

`run-03-final-rendered` / `run-04-final-headless` là các lượt trung gian đã PASS trước rà soát metadata cuối: sensor heights từng được ghi nhầm thành base extrinsics. Sửa metadata về offset tương đối với tâm chassis, bổ sung assertion world height rồi chạy lại toàn suite thành **run-05 / run-06**. Geometry, ảnh và policy không thay đổi; bằng chứng cũ được giữ để truy vết.

## 4. Hiệu năng: số đo và phạm vi

[Performance sample rendered](../experiments/godot_s0/evidence/run-05-final-rendered/performance-samples.json): 100 request, mỗi request 6 ticks = 0,1 s, scene tĩnh/robot zero, scan + RGB readback + base64/JSON + Python validation/logging, engine cap 60 FPS.

| Chỉ số | Số đo |
| --- | ---: |
| Tổng simulation / wall time sample | **10,0 / 11,705 s** |
| RTF end-to-end | **0,854** |
| Request median / p95 | **116,55 / 122,20 ms** |
| Engine-reported FPS trong sample | **60–60** |
| Peak working set process Godot đến cuối sample | **255.193.088 byte ≈243,4 MiB** |

Đây là số đo thật trên Iris Xe, **không phải performance acceptance PASS**. FPS là counter của engine dưới cap, không phải p95 GPU frame time. RTF bao gồm ít nhất thời gian chờ logical ticks và render/transport; chưa chứng minh giới hạn GPU ở 0,854× real time. Working set chỉ thuộc process runtime Godot, không gồm Project Manager, Python, OS hay integrated GPU shared memory.

Runtime stderr có lỗi không tạo được shader cache `user://` và đọc Windows root certificate store trong môi trường thực thi giới hạn quyền. Bridge local không dùng TLS; camera/physics tests vẫn hoàn thành và exit 0. **Không coi log là sạch**, không loại thời gian lỗi/cache ra để làm đẹp số đo. Chưa đo cold/warm startup hoặc hiệu năng với cache hoạt động bình thường.

Webots S2 không có FPS/RAM/RTF đối chứng cùng workload trong [audit S2](webots-s2-audit.md); thời gian chạy suite hoặc mission duration không thể dùng thay thế. Vì vậy **UNVERIFIED: Godot nhanh/nhẹ hơn Webots**. Không chạy lại hoặc sửa S2 chỉ để tạo số đối chứng ngoài phạm vi yêu cầu.

## 5. So sánh với Webots S2

Đối chiếu với implementation và kết quả trong [Webots S2 audit](webots-s2-audit.md), không coi hai suite là cùng độ khó.

| Khía cạnh | Godot hỗ trợ sẵn | Godot S0 đã tự xây / còn thiếu | Webots S2 hiện có và ý nghĩa lựa chọn |
| --- | --- | --- | --- |
| Python | TCPServer/StreamPeerTCP và JSON; scripting chính của project là GDScript | Toàn bộ wire protocol, client, lifecycle/epoch, validation, sensor serialization. Không dùng native Python controller API | Python controller/Supervisor API sẵn; adapter đã chạy core SearchMission/Navigator/Monitor thực. Webots giảm công tích hợp Robotics |
| Sensors | Camera3D, SubViewport, physics ray queries | RGB readback/stamps và 192-ray scanner tự viết; chưa noise/intensity/dropout/device scheduling. Pose ideal | Camera, Recognition, LiDAR devices sẵn; S2 đã calibrate angles/extrinsics và disable/re-enable. Recognition vẫn oracle; LiDAR có seam bias công khai, không hoàn hảo |
| Physics robot | CharacterBody3D, RigidBody3D, colliders, solver | S0 chỉ kinematic; tự actuator, wheel/Mecanum/contact/encoder nếu cần. Rotation sweep chưa kiểm | S2 ideal omni rigid body, engine contact physics; chưa Mecanum mechanics. Cả hai đều chưa chứng minh sim-to-real, Webots có khung robot/device chuyên biệt |
| Thời gian | Fixed physics TPS, engine process callbacks | Logical tick/time/step và sensor barrier tự viết; engine world vẫn tick trong idle; dynamic global stepping chưa có | `robot.step(96)`/simulation time và thiết bị sampling sẵn; S2 freshness/reset/core time đã kiểm. Webots phù hợp control-loop hiện tại hơn |
| Reset/repeat | Có scene lifecycle/transform APIs | Tự restore fixture, epoch và state; 3 traces PASS trên máy này; chưa reload/restart equivalence hoặc dynamic state reset | S2 load/Supervisor reset/worldReload đã kiểm 3 episode; synchronous repeat PASS, async timing FAIL được ghi rõ |
| Environment | Scene editor, primitive/mesh, material, camera và game/render tooling | S0 dựng phòng nhẹ, kiểm full wall occlusion và reveal; richer SAR scenes phải thêm assets/collider semantics/perception | S2 có scene rõ/occluded/obstacle/absent; geometry primitive. Godot có tiềm năng authoring/game visuals, chưa chứng minh chất lượng cảm biến hay realism tốt hơn |
| Nhiệm vụ | Không có Robotics search stack đặc thù trong engine | Demo RGB search/turn/approach/stop; reveal scripted; không planner/costmap/Monitor production | S2 chạy core thật, autonomous detour/exploration và faults; 7/7 PASS. Chuyển nền tảng cần tái nghiệm thu lớp này |
| Hiệu năng laptop | Compatibility/OpenGL khả dụng thực trên Iris Xe | Sample 60 FPS cap, ~243 MiB runtime peak, RTF end-to-end 0,854; soak/scene lớn chưa đo | S2 chạy thật trên laptop nhưng thiếu matched performance data; chưa có cơ sở nói nền tảng nào nhanh/nhẹ hơn |
| Công sức | Generic engine building blocks giúp dựng scene nhanh | S0 đã cần GDScript server/sensors/stepping + Python client/observer/runner/tests; các robotics guarantees là trách nhiệm dự án | S2 đã có adapter/core integration, calibration và audit. Chi phí migration và duy trì hai backend hiện chưa được bù bởi lợi ích đã chứng minh |

Godot cung cấp TCP server primitives, không tự cung cấp contract observation Robotics/Python bridge đã dùng ở đây. [Godot TCPServer](https://docs.godotengine.org/en/stable/classes/class_tcpserver.html), [StreamPeerTCP](https://docs.godotengine.org/en/stable/classes/class_streampeertcp.html). Physics Godot cũng không đảm bảo deterministic replay giữa máy/backend/driver; kết quả repeat chỉ có giá trị với fixture và cấu hình đã ghi. [Godot Physics introduction](https://docs.godotengine.org/en/stable/tutorials/physics/physics_introduction.html).

## 6. Quyết định GO/NO-GO và phần còn UNVERIFIED

**GO: feasibility kỹ thuật của Godot S0 đạt.** Laptop có thể chạy project nhẹ bằng Compatibility; Python điều khiển full vx/vy/wz, nhận RGB/ranges/pose/time; reset, trace repeat, tường che RGB, collision translation và minimal search mission đều có bằng chứng thực.

**NO-GO: chuyển simulator Robotics chính từ Webots sang Godot ngay sau S0.** Tiêu chí quyết định là khả năng tiếp tục benchmark/control integration đã có, mức bảo đảm thời gian/cảm biến/vật lý và chi phí xây thêm; chưa có lợi ích hiệu năng hoặc môi trường được đo đủ để bù phần phải tự phát triển. **Giữ Webots S2 làm nền tảng chính**, giữ Godot S0 như experiment độc lập cho nhu cầu game/scene authoring hoặc perception visuals về sau.

| Hạng mục | Trạng thái |
| --- | --- |
| Chạy thật trên Windows 11 / Iris Xe / RAM 8 GB, Compatibility | **VERIFIED** trong lượt rendered |
| Bridge, snapshot sensors, reset/repeat cùng process, minimal RGB mission | **PASS** trong fixtures S0 đã công bố |
| Autonomous detour/exploration, core SearchMission/Navigator/Monitor trong Godot | **UNVERIFIED**, chủ ý ngoài phạm vi S0 |
| YOLO, robust human detection, partial/transparent/complex mesh occlusion tổng quát | **UNVERIFIED**; chỉ fixture đỏ và opaque wall đã kiểm |
| Determinism giữa máy/driver hoặc reset toàn bộ dynamic world | **UNVERIFIED** |
| Wheel dynamics, traction, encoder/SLAM, phanh và sim-to-real | **UNVERIFIED** |
| Long-running FPS/RAM/thermal, scene lớn, nhiều robot/camera, GPU memory | **UNVERIFIED** |
| Godot tốt hơn Webots về tốc độ/RAM/realism hoặc đủ thay S2/T1 | **UNVERIFIED**; không suy diễn từ sample S0 |

Nếu sau này cân nhắc đổi nền tảng, cần một workload đối chứng giống nhau và kiểm toàn-world stepping/dynamic reset cùng contract sensors trước khi tái tích hợp core. Đây là điều kiện kỹ thuật cho quyết định sau, không phải phần đã làm hoặc yêu cầu xác nhận để hoàn tất S0 này.
