# S2 — Webots Core Integration

Ngày nghiệm thu: **2026-11-25**, Asia/Saigon. Repository `innguyen/Robotic`, nhánh `feat/benchmark`, HEAD đầu việc `1f6251a6cbe7d30e5aee204a341b32c0a274dab7`. Đầu vào: [audit S1](webots-s1-audit.md) và source core hiện hành.

**Đạt điều kiện hoàn thành S2 cho cấu hình planner đồng bộ, model omni lý tưởng, recognition oracle.** Webots R2025a thực trên laptop đã chạy `SearchMission → Navigator/Costmap → Collision Monitor → actuator` để tìm–tiếp cận người, kể cả vòng vật cản và tìm người sau tường. Bộ nghiệm thu cuối **7/7 scenario PASS**. Source ROS production, evaluator và benchmark T1 không đổi.

Không dùng controller S0 làm policy. Không tích hợp YOLO, nâng đồ họa lớn, Godot hoặc mở rộng benchmark corpus. Kết quả này không chứng minh động lực học Mecanum, odometry thật hay perception từ ảnh.

## 1. Implementation và ranh giới

Integration nằm trong [experiments/webots_s2](../experiments/webots_s2/), có [hướng dẫn chạy](../experiments/webots_s2/README.md).

| Thành phần | Implementation |
| --- | --- |
| Adapter | [adapter.py](../experiments/webots_s2/adapter.py): observation, frame transforms, scan conversion, freshness/epoch gate, core lifecycle và pipeline điều khiển |
| Webots I/O | [runtime.py](../experiments/webots_s2/runtime.py): camera/LiDAR, ideal self pose, timestamps, body velocity và telemetry |
| Policy entry | [s2_core.py](../experiments/webots_s2/controllers/s2_core/s2_core.py): tạo Webots controller, chạy adapter, fail-safe shutdown |
| Fixtures | [scenarios.json](../experiments/webots_s2/scenarios.json), [fixtures.py](../experiments/webots_s2/fixtures.py), [world tương tác](../experiments/webots_s2/worlds/s2_search.wbt) |
| Independent assertions | [s2_audit.py](../experiments/webots_s2/controllers/s2_audit/s2_audit.py), [geometry.py](../experiments/webots_s2/geometry.py): contacts, SAT footprint, standoff, dừng vật lý, fault và repeat |
| Calibration | [s2_calibrate.py](../experiments/webots_s2/controllers/s2_calibrate/s2_calibrate.py): độc lập với mission, dùng test commands qua cùng Monitor/limits |
| Tests/runner | [test_adapter.py](../experiments/webots_s2/test_adapter.py), [run_s2.py](../experiments/webots_s2/run_s2.py) |

Adapter import trực tiếp ba file hiện hành trong `test version/`: [search_core_ros1.py](../test%20version/search_core_ros1.py), [navigation_core_ros1.py](../test%20version/navigation_core_ros1.py), [collision_monitor_ros1.py](../test%20version/collision_monitor_ros1.py). Không copy/reimplement state machine hoặc planner, không gọi ROS `main()`. `CORE_START` ghi module/class và **đường dẫn file thật đã import**; `invocation.json` ghi SHA256 của core và code integration. Có đếm riêng số lần gọi `SearchMission.step`, `Navigator.step` thật và Monitor.

Fixture tái sử dụng hình mannequin primitive từ world S0; đây chỉ là geometry, không tái sử dụng controller demo. Runner tạo project tạm riêng và giữ nguyên S0/S1. Không commit/push trong S2.

## 2. Robot, actuator và safety

### Model đã chọn

**Ideal holonomic velocity body**: Robot có rigid body mass 2 kg, bounding/visual Box **0.4 × 0.4 × 0.2 m**, contact với floor có friction 0. Body nhận đủ `(vx,vy,wz)` và Webots giải contact physics. Không teleport pose khi chạy mission. Calibration có teleport để bố trí phép đo, được tách thành controller riêng.

Model này đại diện **khả năng điều khiển omni**, không phải chassis JetAuto Mecanum đã mô hình hóa cơ khí. Không có motor/roller, slip, encoder hoặc wheel inverse kinematics để tuyên bố sim-to-real. Việc giữ đủ `vy` đã kiểm bằng dịch ngang thuần và lệnh phối hợp, không âm thầm bỏ lateral command.

`WorldInfo.physicsDisableTime=1000000` giữ rigid body thức qua dwell/zero command trong phạm vi episode ≤180 s. Lượt đầu cho thấy model bị đứng sau dwell với auto-sleep mặc định; tăng thời gian sleep giải quyết trên máy này. Đây là cấu hình của actuator lý tưởng, không phải sửa thuật toán production. Quy tắc set velocity dùng world frame theo [Supervisor R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/supervisor.md); cơ chế idle/sleep theo [WorldInfo R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/worldinfo.md).

### Command pipeline

```text
Observation hợp lệ
 → scan_points → Monitor.set_scan
 → Costmap.update → Mission.update_coverage → grid snapshot
 → SearchMission.step (tự gọi Navigator.step khi cần)
 → actuator_limit(raw), common scaling giữ tỷ lệ vx/vy/wz
 → Monitor.set_command(limited) → Monitor.evaluate
 → actuator_limit(checked); nếu đổi Twist thì Monitor đánh giá lại
 → xoay linear velocity base → world → Robot.setVelocity
```

Limits: forward **0.18 m/s**, backward **0.06 m/s**, lateral **0.12 m/s**, yaw **π/6 rad/s**. Footprint cấu hình Monitor/Costmap là 0.4 × 0.4 m; scan extrinsic x=0.12, y=yaw=0; không mang body size, offset 0.096 m hay blind sectors JetAuto sang fixture này.

Mỗi `CONTROL` ghi `raw`, `limited`, `checked`, `applied`, reason và Monitor state. Scan/pose/camera invalid hoặc timestamp lỗi không step policy; adapter invalidate scan rồi đưa zero qua Monitor và limits. Khi bootstrap/reset/shutdown chưa có observation, I/O có **fail-safe chỉ phát zero**; không có đường phát vận tốc khác zero bỏ qua Monitor. Lỗi bất ngờ ghi traceback và dừng simulator, không tiếp tục chạy lệnh cũ.

Calibration kiểm `STOP_ZONE` trên scan render thật: đề xuất `[0.1,0,0]`, checked/applied `[0,0,0]`, drift **0 m**. Scene tránh hộp có Monitor `SLOWDOWN`; scene occluded có cả `APPROACH`/`SLOWDOWN`. Không gọi toàn bộ behavior này là kiểm chuẩn phanh hoặc contact dynamics của JetAuto thật.

## 3. Frames, sensors, clock và target

### Tọa độ và nguồn dữ liệu

World ENU, base FLU (+X tiến, +Y trái, +Z lên), yaw dương CCW. Odom origin là initial self pose của episode:

```text
pO_robot = Rz(-yaw0) * (pW_robot - pW0)
yawO = wrap(yawW - yaw0)
pO_target = pO_robot(t_camera)
           + Rz(yawO(t_camera)) * [x_camera + 0.12, y_camera]
```

Camera extrinsic `(0.12,0,0.55)` m và LiDAR `(0.12,0,0.15)` m, rotation mặc định. Robot pose lấy từ **chính robot** qua Supervisor; roll/pitch >5° bị invalidate. Đây là `supervisor_ideal_pose`, không gọi là SLAM/encoder odometry.

`Observation` không có trường target world/ground truth. Runtime lọc `getRecognitionObjects()` có model `s0_person`, chỉ đưa ID và camera-relative vector vào adapter; xác nhận **hai frame mới liên tiếp cùng ID**. Mất recognition trả target `None`; SearchMission tự giữ last sighting/revisit bằng state của nó. Không giữ timestamp target giả.

Các file policy/runtime không gọi `getFromDef` để lấy người/tường. Observer được phép đọc người toàn cục để assertion sau FOUND; calibration đọc marker để đo transform. Hai đường này không đưa ground truth vào policy. Policy config chỉ có log path, deadline, epoch, fault schedule và planner mode; target/obstacle truth chỉ nằm trong config của observer.

Recognition là oracle, không phải detector ảnh. Object được Webots trả về có thể còn nhìn một phần dù tâm nằm ngoài image FOV; adapter không áp thêm centre-only FOV gate gây mất target sai. Kiểm hợp lệ vector hữu hạn/phía trước; visibility/occlusion/range do node Recognition của fixture thực hiện. [Camera R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/camera.md), [Recognition R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/recognition.md).

### LiDAR và calibration

S2 dùng **192 rays**, 1 layer, 360°, fixed/cylindrical, min/max 0.05–6 m, near 0.02 m, noise 0. Camera giữ 160×120, horizontal FOV 1.012290966 rad, recognition range 3.5 m, `occlusion 1`. Physics 32 ms; control, camera, recognition và LiDAR **96 ms**.

Theo công thức fixed-head trong [WbLidar.cpp R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/src/webots/nodes/WbLidar.cpp), raw ray `i` có góc `FOV/2-(i+0.5)*FOV/N`. Adapter đảo range array để core nhận:

```text
angle_min = -π + π/192 = -3.125230191852346 rad
angle_increment = 2π/192 = 0.032724923474893676 rad
```

Không dùng `2π/(N-1)` hoặc tạo hai tia trùng ±π. Extrinsic chỉ được core cộng một lần. `+Inf` là no-return; NaN/-Inf/zero/negative/out-of-range hữu hạn bị fail closed, không biến thành free space. Bình thường không bật point cloud.

Calibration trên máy thật:

| Kiểm | Kết quả |
| --- | --- |
| So góc với point cloud Webots | 192/192 points; max sai số **3.98e-8 rad** |
| Analytic wall ranges, bốn hướng chính | Max sai số **5.03e-5 m** |
| Rays ngoài vùng seam một angular step | Max sai số **5.03e-5 m** |
| Toàn vòng, gồm seam | Max sai số tuyệt đối **0.118601 m**, hướng **đánh giá range ngắn hơn** |
| Overestimate range lớn nhất | **5.03e-5 m** |
| Camera → base → odom | Marker tại yaw 0, +π/2, -π/2, π; sai số <1e-14 m trên fixture lý tưởng |
| Actuator | +vx, +vy, +wz và lệnh phối hợp tại yaw 0, π/2; max position error ~**0.000664 m**, yaw error <**2.6e-5 rad**, zero dừng |

**Giới hạn LiDAR đã quan sát:** 180 rays có hai `+Inf` bất thường ở scene phòng kín; đổi 192 rays loại bỏ hai hole trong calibration, nhưng còn bias âm gần seam cylindrical/cube-face. Guard calibration yêu cầu tất cả rays hữu hạn ở phòng kín, error ngoài seam/cardinal ≤5 mm, overestimate ≤1 cm và bias toàn vòng ≤15 cm. Không gọi đây là range accuracy đồng nhất toàn vòng và không sửa scan bằng scene ground truth. Bias âm có thể tạo obstacle giả lớn hơn và ảnh hưởng route; scene khác vẫn cần kiểm lại. [LiDAR rendering/API R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/lidar.md).

Marker calibration là cube nhỏ đặt ngang cao camera để tách phép đo transform khỏi hình học mannequin. Với mannequin gần camera, recognition position có thể khác tâm node do hình học/clipping. Core đặt `standoff=.55 m`, nhưng Navigator tính thêm nửa body length 0.2 m và goal tolerance 0.1 m theo source hiện hành. Khoảng cách **base → tâm người ground truth** khi dừng thực tế khoảng 1.009–1.036 m ở các ca PASS, không phải 0.55 m tuyệt đối. Không hiệu chỉnh target bằng ground truth để ép số đo đẹp hơn.

### Đồng hồ và freshness

Tất cả timeout, coverage, map aging, mission và Monitor dùng `robot.getTime()`/simulation time. Wall clock chỉ ghi invocation duration, không dùng làm timeout policy hay metric RTF. Core planner có phép đo thời gian thực thi riêng; không sửa global clock production.

Fixture sampling đồng bộ: một `step(96)` cho một sensor tick. Timestamp chỉ tăng khi device và recognition còn enable đúng period; nếu bị disable thì giữ buffer/stamp cũ. Kiểm stamp hữu hạn, không tương lai/lùi, age ≤0.5 s, pose–scan–camera cùng stamp; không gắn pose hiện tại lên buffer cũ. Cả ba buffer giữ nguyên sẽ dừng `NO_NEW_SENSOR_SAMPLE`, không xác nhận target lần hai. Mọi camera fault cũng đóng gate thay vì coi là “không có người”.

Sáu fault của scenario `faults` là **injection dữ liệu/metadata sau khi đọc sensor thật**, có nhãn trong từng log: invalid/stale scan, camera, pose. Ngoài ra calibration **thực sự gọi `lidar.disable()`**: adapter dừng ngay vì không đồng bộ stamp, sau đó `STALE_OR_FUTURE_SCAN`; re-enable phục hồi IDLE. Không tuyên bố có timestamp phần cứng độc lập cho camera Webots: freshness dựa trên lịch sample synchronous và trạng thái device. Device render âm thầm đóng băng nhưng vẫn báo enable: **UNVERIFIED**.

## 4. Reset, reload và workers

`CoreAdapter.new_episode` gọi `Navigator.close()` (executor shutdown `wait=True`) trước khi tạo Navigator/Costmap, SearchMission và Monitor mới; xóa map, obstacle points, coverage, last target/confirmation, timestamps, path, progress và futures. `Navigator.reset()` riêng không đủ cho episode mới.

Runtime phát hiện sim clock lùi, phát zero, gọi `new_episode`, re-enable sensor và bỏ bước warmup trước khi nhận frame mới. Không restart core controller trong phép thử API reset, nên thực sự kiểm reset **trong cùng tiến trình**; observer được restart riêng để kiểm fixture ban đầu. GUI reset/reload tạo process mới cũng tạo core mới. Epoch lọc telemetry `customData` cũ; không lấy FOUND cũ nghiệm thu episode mới.

[Repeat mặc định](../experiments/webots_s2/evidence/run-07-final/repeat/result.json):

| Lượt | Episode start sim time | FOUND sim time | Elapsed |
| --- | ---: | ---: | ---: |
| Initial load | 0 | 17.664 | **17.664 s** |
| Supervisor reset, core không restart | 0.096 | 17.760 | **17.664 s** |
| worldReload | 0 | 17.664 | **17.664 s** |

Clock có thể được nhận biết ở tick tiếp theo sau Supervisor reset; log nêu đúng 0.096 s, không giả là controller đã xử lý ở t=0. Physical fixture ban đầu được observer kiểm trước khi chạy lại. Cả ba stop pose XY giống nhau trên máy này, hold 1.056 s. Ngưỡng repeat đã dùng: stop XY chênh ≤2 cm, elapsed chênh ≤0.3 s. Đây không phải đảm bảo bitwise replay trên driver/máy khác.

**Planner mặc định đồng bộ** để tái lập lượt thử nhỏ. Không chờ async worker rồi báo hiệu năng simulation tốt hơn; không có tuyên bố benchmark tốc độ.

Kiểm thêm [async thật](../experiments/webots_s2/evidence/run-06-async/repeat/result.json): ba episode đều FOUND, worker trước reset được thu hồi (`EPISODE_RESET.closed_workers=1`); unit test cũng tạo worker thật rồi xác nhận shutdown/join. Tuy nhiên elapsed là **15.840, 17.472, 17.568 s**, vượt ngưỡng repeat 0.3 s: **FAIL tái lập thời gian cho chế độ async**, không phải PASS cả suite. Stop pose lệch khoảng 1.15 mm. `--async-planning` giữ là chế độ thử, không là cấu hình nghiệm thu; chưa bảo đảm lịch hoàn tất future độc lập wall scheduling. Không nới threshold để đổi kết quả FAIL.

## 5. Kiểm thử thực nghiệm cuối

Môi trường: Webots **R2025a**, Windows laptop hiện có, Python **3.13.15 x64**, NumPy **2.5.4**. Runner chạy `--batch --minimize --no-rendering --mode=fast`, vẫn render camera/LiDAR. Không đo lại RAM/FPS. Mỗi scenario có invocation, fixture thực chạy, config, SHA256 source/core trước–sau và exit code.

**Bộ cuối:** [run-07-final/summary.json](../experiments/webots_s2/evidence/run-07-final/summary.json), tất cả exit 0 và `SUITE_PASS` từ controller/observer.

| Scenario | Bằng chứng / kết quả |
| --- | --- |
| [found](../experiments/webots_s2/evidence/run-07-final/found/result.json) | SPIN → APPROACH → FOUND **17.664 s**; 184 policy steps, **128 Navigator calls**; Monitor PASS/IDLE; base–person ~1.036 m |
| [occluded](../experiments/webots_s2/evidence/run-07-final/occluded/result.json) | Tường opaque 0.1×1.4×2.2 m che người ban đầu; policy SPIN/GOTO/SEARCH_EXPLORE, detection đầu **65.760 s**, FOUND **78.912 s**; 551 Navigator calls; lệch ngang khỏi đường trực tiếp ~0.985 m |
| [obstacle](../experiments/webots_s2/evidence/run-07-final/obstacle/result.json) | Hộp thấp 0.5×0.7×0.4 m chặn đường LiDAR, camera vẫn thấy người phía trên; FOUND **30.624 s**, detour ~0.629 m, **vy đạt 0.12 m/s**, Monitor SLOWDOWN; min SAT separation ~0.0603 m |
| [absent](../experiments/webots_s2/evidence/run-07-final/absent/result.json) | Không target/false FOUND trong **100.032 s**, có SPIN/GOTO/SEARCH_EXPLORE và chuyển động; deadline dừng, hold đạt; **không** báo SEARCH_COMPLETE hoặc suy ra đã tìm hết mọi nơi |
| [faults](../experiments/webots_s2/evidence/run-07-final/faults/result.json) | Cả 6 invalid/stale fault đều zero qua gate/Monitor, **150 kiểm dừng vật lý**; sau phục hồi FOUND **23.424 s** |
| [repeat](../experiments/webots_s2/evidence/run-07-final/repeat/result.json) | 3 episode load/reset/reload, elapsed 17.664 s và stop pose tái lập |
| [calibration](../experiments/webots_s2/evidence/run-07-final/calibration/calibration.jsonl) | Scan/point-cloud, camera transforms bốn yaw, 8 actuator trials, STOP_ZONE thật, LiDAR disable/re-enable thật đều đạt các ngưỡng công khai ở trên |

Observer kiểm contacts **mỗi physics tick 32 ms**, loại contact floor thấp hơn 3 cm; không có non-floor contact được chấp nhận. Đồng thời kiểm footprint robot quay bằng **SAT trên bốn trục**, không dùng world AABB bao robot để kết luận va chạm giả ở góc. Cả hai phép kiểm đều đạt ở các scene nghiệm thu. Không mở rộng kết luận sang vật cản rất thấp dưới ngưỡng contact hoặc mesh chưa kiểm.

FOUND/terminal phải có applied zero và giữ vị trí ≥1 s; hold drift cho phép ≤5 mm, vận tốc XY/yaw cuối ≤0.001 theo đơn vị tương ứng. Với fault, observer còn kiểm velocity sau khi zero đã được áp dụng. Terminal deadline ở absent là budget stop riêng của harness/adapter, không giả làm quyết định “không có người” của SearchMission.

### Raw logs và fixture identities

Trong từng thư mục scenario:

- `control.jsonl`: toàn bộ chu kỳ core, targets từ recognition, reason/state, ba tầng command, stamps/epoch, counters và module paths.
- `audit.jsonl`: assertions, episode PASS, reset/reload requests hoặc traceback lỗi; calibration có `calibration.jsonl` riêng.
- `fixture.wbt`, `scenario.json`, `policy-config.json`: scene và inputs chính xác; policy config không chứa vị trí người.
- `invocation.json`: CLI, Python, seed qua fixture, model, fixture SHA256, code/core SHA256 trước–sau, PID, exit code, wall duration; duration không phải RTF/FPS.

Ví dụ [found control](../experiments/webots_s2/evidence/run-07-final/found/control.jsonl), [found invocation](../experiments/webots_s2/evidence/run-07-final/found/invocation.json), [occluded control](../experiments/webots_s2/evidence/run-07-final/occluded/control.jsonl), [fault audit](../experiments/webots_s2/evidence/run-07-final/faults/audit.jsonl), [repeat control](../experiments/webots_s2/evidence/run-07-final/repeat/control.jsonl).

Windows launcher tiếp tục cho stdout/stderr redirect rỗng trong phiên này. JSONL ghi trực tiếp từ controller là bằng chứng chính; không dựng console giả, không lấy redirect rỗng làm chứng minh không warning. Thu đầy đủ mọi engine warning qua CLI vẫn **UNVERIFIED** như S1.

## 6. Tests và lịch sử lỗi

**14/14 unit/core integration tests PASS**, [output thật](../experiments/webots_s2/evidence/run-07-final/unit-tests.txt). Bao gồm frame bốn yaw, pixel-center scan, actuator scaling/full vy, SAT regression, invalid/stale/future/epoch gate, held buffer, confirmation hai frame, oracle sát mép FOV, reset state, async worker thật được join, và vòng sensor tổng hợp đưa **core thật** tới FOUND. Sensor trong unit loop là tổng hợp, không được tính là bằng chứng Webots.

**4/4 regression tests S1 PASS**, [output](../experiments/webots_s2/evidence/run-07-final/s1-regression-tests.txt). `compileall` cho S2 thành công. Các kiểm cuối về diff, link và identity được ghi trong đợt rà soát; không chạy T1/evaluator vì không đổi chúng.

[Rà soát độc lập sau chạy](../experiments/webots_s2/evidence/run-07-final/verification.json) đã đối chiếu **3.251 CONTROL records**: actuator bounds, checked/applied, zero khi fault/terminal, target phải có recognition và clock tăng trong từng epoch. SHA256 của source hiện hành, bản controller thực chạy và fixture khớp bằng chứng; không có lỗi kiểm tra. Raw fixtures lịch sử giữ nguyên byte, gồm cả whitespace của các lần đầu, để không phá hash. Rà soát source/doc và liên kết tách khỏi việc định dạng lại bằng chứng.

Các lần chạy trước được giữ nguyên:

| Lần | Trạng thái và chẩn đoán |
| --- | --- |
| [run-01](../experiments/webots_s2/evidence/run-01/) | Suite FAIL: calibration có +Inf ở 180 rays; model không quay sau dwell; centre-only FOV gate làm mất target sai. Không dùng absent PASS của lượt này để khẳng định search chuyển động. |
| [run-02](../experiments/webots_s2/evidence/run-02/) | found PASS sau khi giữ body thức và bỏ FOV gate sai; calibration FAIL do seam holes; obstacle observer FAIL vì world AABB bao robot quay gây collision giả. |
| [run-03](../experiments/webots_s2/evidence/run-03/) | Sáu scene mission PASS với 192 rays và SAT; calibration FAIL vì yêu cầu accuracy 4 cm trên mọi ray, raw scan chỉ ra bias âm cục bộ ~11.86 cm ở seam. |
| [run-04-calibration](../experiments/webots_s2/evidence/run-04-calibration/) | Calibration PASS sau khi tách/giới hạn bias seam, đối chiếu point cloud và thêm STOP/native disable checks. Không sửa range bằng ground truth. |
| [run-05-final](../experiments/webots_s2/evidence/run-05-final/) | **7/7 PASS**, planner đồng bộ; sau lượt này chỉ bỏ whitespace trống trong world generator để chuẩn bị bundle cuối. |
| [run-06-async](../experiments/webots_s2/evidence/run-06-async/) | Mission/lifecycle đạt, **FAIL repeat timing**; đã nêu số đo và giới hạn tại mục 4. |
| [run-07-final](../experiments/webots_s2/evidence/run-07-final/) | Bộ nghiệm thu cuối sau chỉnh whitespace, cùng thuật toán/model; các số liệu chính và link bằng chứng trong báo cáo trỏ tới lượt này. |

Lỗi unit loop đầu tiên là raycaster tổng hợp chưa áp maxRange 6 m, khiến adapter đúng khi từ chối các range lớn hơn 6; sửa fixture test cho đúng semantics. Không hạ gate invalid hoặc sửa core để làm test PASS.

## 7. Phạm vi đã đạt và UNVERIFIED

| Nội dung | Trạng thái |
| --- | --- |
| SearchMission/Navigator/Costmap/Monitor thật điều khiển Webots | **PASS**, có module paths, hash, counters và raw logs |
| Holonomic vx/vy/wz, limits, transform, scan angles, oracle occlusion, obstacle detour | **PASS trong fixtures đã ghi**; bias seam LiDAR và oracle centroid đã nêu |
| Invalid/stale sensor, native LiDAR disable, dừng vật lý và reset/reload | **PASS** trong protocol S2 |
| Planner worker được thu hồi khi new episode | **PASS** qua unit worker thật và async Webots reset log |
| Async replay timing ≤0.3 s | **FAIL**; synchronous là cấu hình nghiệm thu |
| Chứng minh không có người trên toàn bộ mọi vùng | **UNVERIFIED**; absent chỉ kiểm bounded search/no false FOUND/deadline stop |
| Mecanum roller/traction, động lực học/phanh/odometry thật, sim-to-real | **UNVERIFIED**; model hiện là ideal omni body |
| YOLO/RGB perception, multi-person association, transparent/partial mesh occlusion tổng quát | **UNVERIFIED**, ngoài S2 |
| Render sensor âm thầm freeze khi API vẫn báo enabled | **UNVERIFIED** |
| GUI Reset bấm tay trong S2, GUI FPS/RAM dài hạn, scene lớn | **UNVERIFIED**; API reset/reload đã chạy, không đo lại hiệu năng |
| Thu đủ mọi engine warning qua Windows CLI | **UNVERIFIED**, JSONL controller vẫn đầy đủ |

S2 đã đưa thuật toán chính thức vào vòng điều khiển Webots có kiểm thử thực. Bước sau có thể chọn robot Mecanum cơ khí hoặc pose/perception thực hơn theo mục tiêu mới; không cần làm lại search policy demo để tiếp tục thử nghiệm.
