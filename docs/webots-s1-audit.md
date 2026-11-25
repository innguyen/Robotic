# S1 — Webots PoC Hardening

Ngày thực hiện: **2026-11-25**, múi giờ Asia/Saigon. Repository `innguyen/Robotic`, nhánh `feat/benchmark`, HEAD khi bắt đầu `9f43cb8494a5fc58eb72bbc653865504f1bd1583`.

**Kết quả: PASS trong phạm vi fixture nhỏ và oracle recognition.** Đã chạy Webots **R2025a thực trên laptop Windows hiện tại**, qua load → reset kèm restart controller → reload; cả ba lượt tìm–tiếp cận–dừng thành công. Ca tường che kín có đối chứng mở/đóng đạt 0/12 → 12/12 → 0/12 mẫu nhận ra người. Đây là cơ sở chuẩn bị nối thuật toán; **tích hợp SearchMission/Navigator/Collision Monitor và perception bằng ảnh vẫn UNVERIFIED**.

## 1. Đầu vào và phạm vi

Đã đọc [báo cáo S0](simulator-feasibility.md), [world S0](../experiments/webots_s0/worlds/s0_search.wbt), [controller S0](../experiments/webots_s0/controllers/s0_smoke/s0_smoke.py), [console lượt S0 đã lưu](../experiments/webots_s0/evidence/run-02-console.txt), cùng các interface lõi sau:

- [SearchMission và SearchConfig](../test%20version/search_core_ros1.py): `update_coverage(...)`, `step(pose, target, standoff, now, dt)`.
- [Navigator/Costmap](../test%20version/navigation_core_ros1.py): `costmap.update(...)`, `snapshot(...)`, `step(...)`, `reset()`, `close()`.
- [Monitor/scan_points](../test%20version/collision_monitor_ros1.py): `set_scan`, `invalidate_scan`, `set_command`, `evaluate`.
- [Luồng ROS hiện hành](../test%20version/navigation_ros1.py), cấu hình tương ứng và [simulation 2D](../test%20version/simulate_navigation.py): đối chiếu gate, standoff và đồng bộ pose/scan.

Thay đổi S1 chỉ ở `experiments/webots_s0/`, báo cáo này và liên kết trạng thái S1 trong tài liệu S0. **Không sửa source production, benchmark T1 hay evaluator; không commit/push.** File `.s0_search.wbproj` đã có thay đổi của người dùng trước S1, được giữ nguyên. Không đo lại RAM/FPS theo quyết định S0; không cài Blender, model AI hoặc cảnh nặng.

## 2. Lỗi initial pose và cách khắc phục

World ở HEAD vẫn lưu robot tại `(-0.0887217372, -1.0083094133, 0.0994054107)`, yaw khoảng `0.007632` rad. Nó còn lưu góc hai bánh, caster và hidden velocities. Vì thế reload bắt đầu ngay ở đích, có thể FOUND sau 0.192 s. Tài liệu S0 đã mô tả một lần sửa, nhưng file hiện hành đã quay lại trạng thái cuối; tài liệu không thay thế kiểm fixture trên đĩa.

Đã khôi phục:

| Thành phần | Trạng thái canonical |
| --- | --- |
| Robot `DEF S0_ROBOT` | `translation -2 -1 0.1`, `rotation 0 0 1 -π/2` |
| Bánh trái/phải | Tâm `(0, ±0.18, 0)`, quay cylinder quanh X `π/2`, joint angle mặc định 0 |
| Caster | `(±0.16, 0, -0.07)`, rotation/joint angle mặc định |
| Vận tốc khởi đầu | Bỏ mọi `linearVelocity`/`angularVelocity` đã lưu; mặc định bằng 0 |
| WorldInfo | Ghi tường minh `basicTimeStep 32`, `coordinateSystem "ENU"`, `randomSeed 0` |
| Cảm biến | Ghi tường minh tên device, recognition `occlusion 1`, LiDAR fixed/cylindrical/noise 0 |

Giữ sàn, tường, người, hộp và viewpoint đã có. Bản end pose lịch sử vẫn ở [saved-pose-before-repeat-fix.wbt](../experiments/webots_s0/evidence/saved-pose-before-repeat-fix.wbt), SHA256 `7d9e024841edaf20bc8f585de9dab927d75ad07010877e24b4efdcc12dde8357` (đã kiểm lại).

**Reset có hai nghĩa khác nhau:** nút Reset GUI restart controller; API `Supervisor.simulationReset()` giữ controller đang chạy. Harness S1 gọi reset rồi `restartController()` cho cả smoke controller và observer. Trên R2025a, hai controller bắt đầu lại tại 0.032 s, nhưng pose, orientation và velocity vẫn đúng initial state. `worldReload()` bắt đầu tại 0 s. Hành vi restart được đối chiếu với [tài liệu Supervisor R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/supervisor.md).

`customData` có thể còn telemetry FOUND từ epoch trước qua Supervisor reset. S1 thêm `sim_s` và `epoch_started_sim_s`; observer bỏ dữ liệu khác epoch, timestamp ở tương lai hoặc mẫu trùng. Không dùng thông báo FOUND cũ để nghiệm thu lượt mới.

## 3. Guard và công cụ chạy lại

[s1_fixture.py](../experiments/webots_s0/s1_fixture.py) kiểm các scalar field của fixture native: vị trí/kích thước collision geometry, robot initial pose, wheel/caster, joint state, velocity, camera và LiDAR. Đây là guard cho cấu trúc nhỏ hiện tại, **không phải parser VRML tổng quát**. Webots thực mới là kiểm load/sensor/physics.

[test_s1_fixture.py](../experiments/webots_s0/test_s1_fixture.py) có 4 test với các mutation hồi quy: end pose, yaw cuối, joint position/velocity đã lưu, thay kích thước sàn/camera/rays/extrinsic và tắt occlusion hoặc bỏ collision của tường đều phải bị phát hiện. Static contract:

| Tham số | Giá trị |
| --- | --- |
| Physics / sensor-control | 32 ms / 96 ms |
| Camera | 160 × 120 BGRA, 76,800 bytes; horizontal FOV 1.012290966 rad (~58°) |
| Camera extrinsic trong base | `(0.12, 0, 0.55)` m, rotation mặc định |
| Camera clip / oracle maxRange | 0.02–6 m / 3.5 m |
| LiDAR | 180 rays × 1 layer, 2π horizontal FOV, vertical FOV 0.01 rad |
| LiDAR extrinsic / clip / range | `(0.12, 0, 0.15)` m; near 0.02 m; 0.05–6 m |
| LiDAR mode | fixed, cylindrical, noise 0, không bật point cloud |
| Vi sai | radius 0.10 m, track 0.36 m |
| Smoke stop / timeout | camera planar range ≤ 0.8 m / 90 s simulation time |

Đã chạy **4/4 test PASS**; output thật ở [static-tests.txt](../experiments/webots_s0/evidence/s1-2026-11-25-05/static-tests.txt). `compileall` cho script/controller S1 và `git diff --check` đều thành công. Đã kiểm thêm whitespace của file mới, 45 liên kết nội bộ và SHA256 controller/world hiện hành khớp đúng mã đã chạy nghiệm thu. Không chạy bộ benchmark T1 hoặc test production vì không thay những phần đó.

[run_s1.py](../experiments/webots_s0/run_s1.py) kiểm static trước khi chạy, pin R2025a, tạo project riêng trong `.s1-runs/` (Git ignore), copy controller/world rồi thêm observer. Nó ghi invocation, version, SHA256 world/controller, exit code, logs, kết quả và ảnh. Mỗi lượt yêu cầu thư mục evidence mới; không ghi đè kết quả trước. World canonical có hash trước/sau bằng nhau. Timeout chỉ dừng cây tiến trình được harness tạo.

Chạy từ repository root bằng Python hiện có. Trong phiên này `py` trỏ sai interpreter nên đã dùng đường dẫn Python 3.13.15 hoạt động; không cần cài lại Python:

```powershell
$pythonS1 = 'C:/Users/lemin/AppData/Local/Programs/Python/Python313/python.exe'
& $pythonS1 -m unittest discover -s experiments/webots_s0 -p test_s1_fixture.py -v
& $pythonS1 experiments/webots_s0/run_s1.py
& $pythonS1 experiments/webots_s0/run_s1.py --real --evidence experiments/webots_s0/evidence/s1-new-run
```

Nếu chủ động thay canonical fixture, cập nhật guard có lý do và tạo lại ca occlusion bằng `run_s1.py --write-occlusion`. Ca [s1_occlusion.wbt](../experiments/webots_s0/worlds/s1_occlusion.wbt) được sinh từ baseline để giữ cùng robot/sensor; test phát hiện hai world lệch nhau. Có thể mở trực tiếp world này trong Webots để xem test; không truyền evidence args thì nó giữ chạy tại pha che kín sau khi test xong.

Khi GUI đang giữ scene cũ: pause, reload từ đĩa và không lưu end pose đè lên canonical. Chạy static check sau thao tác Save để phát hiện việc lưu nhầm. `.wbproj` chỉ chứa bố trí giao diện, không phải nguồn initial pose.

## 4. Kết quả thực chạy và bằng chứng

Bộ chạy nghiệm thu ở [evidence/s1-2026-11-25-05](../experiments/webots_s0/evidence/s1-2026-11-25-05/), khoảng **12:39 ngày 2026-11-25 Asia/Saigon**. Webots R2025a, Python 3.13.15 x64. Chạy `--batch --minimize --no-rendering --mode=fast`; main view không render, camera/LiDAR vẫn render. Đây là phép thử chức năng, **không dùng wall duration làm RTF/FPS hoặc benchmark hiệu năng**. Ý nghĩa CLI và sensor rendering theo [CLI R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/guide/starting-webots.md) và [LiDAR R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/lidar.md).

### 4.1 Load / reset / reload

Logs gốc: [repeat-smoke.jsonl](../experiments/webots_s0/evidence/s1-2026-11-25-05/repeat-smoke.jsonl), [repeat-supervisor.jsonl](../experiments/webots_s0/evidence/s1-2026-11-25-05/repeat-supervisor.jsonl). [Kết quả pose/hold](../experiments/webots_s0/evidence/s1-2026-11-25-05/repeat-result.json), [invocation + hashes controller](../experiments/webots_s0/evidence/s1-2026-11-25-05/repeat-invocation.json).

| Lượt | Initial sim time | SEARCH_SPIN | APPROACH | FOUND từ smoke | Supervisor thấy FOUND |
| --- | ---: | ---: | ---: | ---: | ---: |
| Load | 0 | 0.096 | 3.456 | 20.544 | 20.640 |
| Reset + restart | 0.032 | 0.128 | 3.488 | 20.576 | 20.672 |
| Reload | 0 | 0.096 | 3.456 | 20.544 | 20.640 |

Đơn vị thời gian là giây mô phỏng. Thời gian từ lúc controller khởi đầu đến FOUND đều **20.544 s**, camera planar range tại FOUND **0.7924920181 m**. Observer có thể chậm một mẫu vì telemetry được gửi qua bước simulation; không lấy timestamp observer thay timestamp detection.

Cả ba lượt đều có initial position `[-2,-1,0.1]`, yaw `-π/2`, sáu thành phần velocity bằng 0. Stop pose đều khoảng `[-0.0887217335,-1.0083086147,0.0994054107]` m. Sau FOUND, command bằng 0, hold **2.016 s**, dịch chuyển trong hold **0.000059858 m**; max trị tuyệt đối của sáu thành phần velocity cuối là `4.90e-9` (linear m/s, angular rad/s). Vì vậy kiểm cả lệnh dừng lẫn trạng thái vật lý, không chỉ tên state.

Ngưỡng harness: initial pose/yaw/velocity sai số ≤ `1e-8`, hold drift ≤ 5 mm, từng thành phần velocity ≤ 0.001, stop pose giữa lượt ≤ 1 cm, chênh thời gian FOUND ≤ hai mẫu (0.192 s). Kết quả đồng nhất pose trên máy này không bảo đảm bitwise determinism trên phiên bản/driver/máy khác. `valid_samples` là số telemetry mới observer đã đọc, không phải số đo throughput sensor.

### 4.2 Occlusion thật

Robot cố định tại `(-2,-1,0.1)`, yaw 0, hướng về người `(1,-1,0.9)`. Tường opaque có Shape Box và `boundingObject` cùng kích thước **0.1 × 2 × 2.2 m**, tâm `(-0.5,-1,1.1)`. Tường che toàn người, không chỉ đặt collision vô hình. Người luôn tồn tại trong FOV và trong oracle maxRange; camera-frame ground truth khoảng `(2.88,0,0.251)` m.

Controller [s1_occlusion.py](../experiments/webots_s0/controllers/s1_occlusion/s1_occlusion.py) giữ hai motor bằng 0, di chuyển tường ngang sang y=2 để mở tầm nhìn, rồi đặt lại. Mỗi pha flush 3 chu kỳ sensor rồi kiểm 12 mẫu mới:

| Pha | Người được oracle nhận ra | LiDAR phía trước | Kết quả |
| --- | ---: | ---: | --- |
| blocked | 0/12 | ~1.33023 m (tường) | PASS |
| open | 12/12 | ~2.80048 m | PASS |
| reblocked | 0/12 | ~1.33023 m (tường) | PASS |

Logs gốc [occlusion-controller.jsonl](../experiments/webots_s0/evidence/s1-2026-11-25-05/occlusion-controller.jsonl); [invocation](../experiments/webots_s0/evidence/s1-2026-11-25-05/occlusion-invocation.json). Đã xem ảnh camera thật lưu bởi Webots: [blocked](../experiments/webots_s0/evidence/s1-2026-11-25-05/occlusion-blocked.png), [open](../experiments/webots_s0/evidence/s1-2026-11-25-05/occlusion-open.png), [reblocked](../experiments/webots_s0/evidence/s1-2026-11-25-05/occlusion-reblocked.png). Ảnh blocked/reblocked tối đen do camera nhìn sát mặt tường trong lighting tối giản; pha open thấy silhouette mannequin. Không chỉnh sửa ảnh hoặc dùng ảnh tổng hợp để thay bằng chứng.

Kết luận giới hạn: **recognition không nhìn xuyên bức tường opaque của fixture này**. `occlusion 1` xét tâm vật thể; không chứng minh mọi trường hợp che một phần, vật trong suốt hoặc mọi mesh. Mức 2 xét biên và tốn hơn; chưa chạy: **UNVERIFIED**. Recognition vẫn là oracle dù đã kiểm occlusion. [Recognition R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/recognition.md).

### 4.3 Chất lượng log và lịch sử chẩn đoán

Trên Windows launcher trong phiên này, file redirect stdout/stderr rỗng ngay cả khi controller chạy. Không dùng file rỗng làm bằng chứng không lỗi. JSONL do controller ghi trực tiếp, append từng event, là raw log chính; return code 0 và các assertion PASS là bằng chứng thực thi. Chưa xác minh được thu đầy đủ mọi warning của engine qua launcher: **UNVERIFIED**.

Giữ các lần thử trước, không gắn nhãn PASS cả suite:

| Thư mục evidence | Kết quả / nguyên nhân |
| --- | --- |
| [s1-2026-11-25](../experiments/webots_s0/evidence/s1-2026-11-25/) | Trial đầu hoàn tất; Supervisor reset không restart observer/controller; dừng đúng cây tiến trình harness. Console redirect rỗng; chỉ còn result JSON, không dựng lại console. |
| [s1-2026-11-25-02](../experiments/webots_s0/evidence/s1-2026-11-25-02/) | Exit 1, trial đầu xong, redirect rỗng nên không đủ raw trace để chốt nguyên nhân riêng của lần này. |
| [s1-2026-11-25-03](../experiments/webots_s0/evidence/s1-2026-11-25-03/) | JSONL trực tiếp chỉ ra controller sau restart bắt đầu tại 0.032 s; assertion cũ yêu cầu đúng 0 s. Pose/vận tốc initial đúng. |
| [s1-2026-11-25-04](../experiments/webots_s0/evidence/s1-2026-11-25-04/) | Observer đọc FOUND cũ qua customData, dẫn tới assertion drift sai; đã thêm stamp/epoch và bỏ mẫu trùng. Smoke thực chạy lại tới FOUND. |
| [s1-2026-11-25-05](../experiments/webots_s0/evidence/s1-2026-11-25-05/) | Suite hoàn chỉnh PASS, exit 0 cho repeat và occlusion. |

[fixture-hashes.json](../experiments/webots_s0/evidence/s1-2026-11-25-05/fixture-hashes.json) xác nhận world trước/sau không bị sửa bởi test. Không suy đoán lỗi GPU từ các lỗi harness trên.

## 5. Ba nguồn thông tin cần tách rõ

| Nguồn | Dữ liệu / quyền truy cập | Ý nghĩa |
| --- | --- | --- |
| Oracle recognition | `camera.getRecognitionObjects()`, model `s0_person`, relative position/size, bị lọc bởi FOV/range/occlusion | Nhãn và hình học do simulator cung cấp; đang dùng để smoke tìm–tiếp cận–dừng. Không phải detector học từ RGB. |
| Simulation ground truth | Supervisor đọc pose robot/người/tường, velocity, scene state kể cả sau tường | Dùng để assertion/log, hoặc robot ideal pose có khai báo. Không cấp vị trí người ẩn cho SearchMission. |
| Perception bằng ảnh | Camera BGRA → RGB/BGR → detector đã xác định model/threshold; kết hợp depth/range có kiểm association để ra vị trí | Cần triển khai và kiểm riêng. Render ảnh hoạt động không chứng minh YOLO, depth association hay ảnh camera robot thật hoạt động: **UNVERIFIED**. |

Trong ca occlusion, ground truth đọc người sau tường chỉ để chứng minh người vẫn tồn tại trong range/FOV; oracle vẫn trả rỗng. Smoke controller không import Supervisor và không đọc pose người toàn cục. Trong ca repeat, observer dùng pose ground truth để kiểm, không gửi target cho policy. API Camera/recognition được mô tả ở [Camera R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/camera.md).

## 6. Thiết kế simulation adapter — chưa triển khai

Phần này là hợp đồng cho bước tích hợp sau S1, **toàn bộ đường chạy qua core còn UNVERIFIED trong Webots**. Đặt adapter sau này trong `experiments/`; dùng interface core hiện hành, không nối motor/ROS robot thật. Không dùng evaluator T1 để kiểm PoC này.

### 6.1 Observation và ranh giới dữ liệu

Mỗi `Observation` bất biến gồm `epoch`, `now`, `dt`; pose `(x,y,yaw)` + `stamp` + `source`; scan ranges + angle metadata + range limits + `stamp` + validity; camera bytes + stamp; target odom hoặc `None` + stamp + `source`. Nguồn target phải là `webots_recognition_oracle` hoặc `rgb_detector`, nguồn pose là `supervisor_ideal_pose` hoặc `wheel_odometry`; không trộn ngầm. Ground truth người nằm trong kênh assertion riêng, không phải input mission.

### 6.2 Frame và transforms

Dùng mét/radian/giây, world `W` ENU, base `B`: +X tiến, +Y trái, +Z lên, yaw dương CCW. Odom `O` đặt tại initial robot pose, không dùng trực tiếp world XY làm odom. Với initial yaw `ψ0` và position `pW0`:

```text
pO = Rz(-ψ0) (pW - pW0)
yawO = wrap(yawW - ψ0)
T_OB(t) = inverse(T_WB(0)) T_WB(t)
pO_target = T_OB(t_camera) T_BC pC_target
T_BC: translation (0.12,0,0.55), rotation I
T_BL: translation (0.12,0,0.15), rotation I
```

Với baseline yaw `-π/2`, target phía đông world nằm về phía trái trong base ban đầu; đây là ca phát hiện sai dấu transform. Webots camera của fixture dùng +X forward/+Y left/+Z up; không áp công thức ROS optical frame lên vector recognition này. Nếu detector/depth dùng optical frame +X right/+Y down/+Z forward, đổi `pC = (z_opt, -x_opt, -y_opt)` trước extrinsic. Recognition position có thể lệch tâm Solid vài mm do ước lượng hình học; không buộc nó bằng node ground truth tuyệt đối.

Ideal pose giai đoạn đầu: Supervisor đọc **robot** position/orientation, yaw từ `atan2(R[3], R[0])` với ma trận row-major. Stamp tại sensor tick, dùng transform đầy đủ khi robot nghiêng; từ chối projection 2D ngoài ngưỡng roll/pitch đã cấu hình. Không lấy target world của Supervisor. Sau đó mới thay pose bằng GPS/IMU/encoder hoặc wheel odometry; cần thêm device/kiểm noise và drift. Không gọi ideal pose là SLAM/odometry đã kiểm chứng.

### 6.3 LiDAR

Lấy bản sao range image sau `step`, một layer. Webots trả từ trái qua phải; core nhận `angle_min + i*angle_increment`. Adapter phải cung cấp góc trong **sensor frame**, vì `scan_points` và `Costmap.update` đã cộng `cc.scan_yaw` và dịch `cc.scan_x/scan_y`. Không transform scan sang base rồi lại đưa extrinsic vào core lần nữa.

Chọn chuẩn output tăng góc CCW: đảo thứ tự rays và tạo metadata tương ứng. **Chưa chốt/kiểm số đo góc tia đầu/điểm lấy mẫu ở biên 360°**; không mặc định `2π/(N-1)` gây trùng ±π. Gate tích hợp bắt buộc: đọc quy tắc pixel của R2025a rồi đối chiếu các hộp hẹp đặt trước/trái/phải/sau và một mốc lệch góc, thử cả yaw robot khác 0. Có thể dùng point cloud tạm để calibration, tắt khi chạy bình thường. Chỉ đưa metadata vào core khi sai số góc/range đạt ngưỡng đã ghi. S1 mới kiểm front rays và occlusion, chưa đủ chứng minh scan transform toàn vòng.

Giữ `+Inf` là no-return hợp lệ; NaN, -Inf, zero/negative hoặc giá trị ngoài range hữu hạn là unknown/invalid, không biến thành không gian trống. `scan_points` đã kiểm usable fraction; adapter bắt exception, `invalidate_scan(reason)` và zero actuator. Không giả timestamp mới khi đọc lại cùng buffer. Có thể dùng timeout source mặc định 0.5 s, command 0.4 s và odom 0.5 s theo core; log rõ cấu hình, tất cả theo sim clock.

Config dành cho fixture: `scan_x=.12`, `scan_y=0`, `scan_yaw=0`; không dùng mù YAML JetAuto `.096`. Footprint bảo thủ khoảng **0.40 × 0.40 m** bao cả bánh, không dùng riêng thân 0.40 × 0.28 m hoặc body 0.324 × 0.260 mặc định. `blind_sectors_deg=[]` chỉ được chấp nhận sau ca kiểm không tự che toàn vòng; mask blind robot thật không chuyển nguyên vào simulator. Giữ các margin/limit an toàn có cấu hình; chưa kiểm braking dynamics nên không quảng bá đây là Collision Monitor đã nghiệm thu.

### 6.4 Đồng hồ, reset và freshness

`now = robot.getTime()`; control/sensor 96 ms, `dt` từ chênh sim time trong cùng epoch. `perf_counter/monotonic` chỉ đo chi phí thực thi. Trong core, planner worker dùng `time.monotonic` cho thời gian tính toán; các timeout/state được truyền `now`, không cần sửa production để thay global clock.

Pause không làm scan/target già thêm theo wall time; không phát lệnh robot thật từ simulation. Clock lùi/reset hoặc reload tạo epoch mới: zero motors, `nav.close()` để thu worker cũ rồi tạo lại Navigator, Costmap, Mission, Monitor; xóa coverage, history, target confirmation, futures và watchdog. `nav.reset()` riêng không xóa toàn costmap/mission. Không chỉ reset đồng hồ rồi giữ FOUND/map cũ. API Supervisor reset cần restart controller hoặc thực hiện đầy đủ reset nội bộ; reset physics riêng không đủ.

Mỗi stamp phải hữu hạn, không ở tương lai, tăng trong epoch. Ghép scan/target với pose cùng stamp; nếu khác chu kỳ dùng pose history và tolerance được cấu hình (`pose_sync_tolerance` hiện mặc định 0.15 s), không gắn pose mới vào detection cũ. Target stale/mất → `None` để mission xử lý lost/revisit; pose/scan stale → gate đóng và zero. Mất camera trong khi vẫn chạy tìm kiếm phải có trạng thái lỗi rõ; không coi camera hỏng là “không có người”.

### 6.5 Thứ tự gọi core và safety

```text
Webots.step → observation snapshot + frame/stamp/validity checks
  → scan_points → Monitor.set_scan(scan_stamp)
  → Costmap.update(scan, matched_pose, scan_stamp)
  → Mission.update_coverage(scan, matched_pose)
  → nav.grid = Costmap.snapshot(pose, now)
  → gate pose/scan/camera/monitor/epoch
  → Mission.step(pose, fresh_target_or_None, standoff, now, dt)
  → feasibility conversion for robot model
  → Monitor.set_command(feasible_command, now) → evaluate(now)
  → final actuator mapping/saturation → actual applied command
  → log raw / feasible / checked / applied + reason + epoch
```

`SearchMission` tự gọi Navigator; adapter không gọi `nav.step` thêm lần thứ hai. Bootstrap Monitor bằng scan mới và zero command, cho phép IDLE trước khi mission chạy. Tham chiếu `MONITOR_OK` trong `navigation_ros1.py`; WAIT_COMMAND/STALE_COMMAND chỉ cho phép tính đề xuất mới, không phát lại lệnh cũ. Không có scan hợp lệ hoặc lỗi conversion thì không chạy mission. Gate phải kiểm lại trước apply nếu epoch/input đổi trong lúc planning. Monitor luôn có thể đưa lệnh về 0; map/coverage update không được hồi sinh input lỗi. Nếu actuator thay đổi tỷ lệ Twist sau Monitor thì phải đánh giá lại command thực trước apply.

Standoff người chính thức hiện là **0.55 m từ base tới target trong odom** theo `navigation_ros1.py`, khác smoke **0.8 m từ camera tới tâm recognition**. Không sao chép threshold smoke rồi gọi tương đương. `update_coverage` dùng LiDAR để ước lượng vùng camera đã nhìn, không phải ground truth visibility theo pixel.

### 6.6 Vi sai so với JetAuto Mecanum

Navigator hiện xuất đủ `(vx,vy,wz)` holonomic. UGV S0 chỉ chạy được `(vx,0,wz)`:

```text
omega_left  = (vx - wz * track/2) / radius
omega_right = (vx + wz * track/2) / radius
vx_applied  = radius * (omega_right + omega_left) / 2
wz_applied  = radius * (omega_right - omega_left) / track
```

Motor wheel units rad/s. Giới hạn cả cặp bằng cùng hệ số theo `getMaxVelocity`/giới hạn cấu hình để giữ curvature; không clamp từng bánh làm đổi đường đi. Hướng motor +, dấu yaw và scale đã phù hợp với smoke quay/tiến; kiểm chuẩn vận tốc độc lập vẫn **UNVERIFIED**.

Không âm thầm bỏ `vy`: điều đó đổi đường đã planner/Monitor xét. Đường tích hợp đầu tiên ưu tiên model omni/Mecanum có sẵn đã kiểm đủ 3 DOF; hoặc model kinematic omni được ghi rõ không kiểm dynamics/traction. Nếu giữ vi sai, adapter mặc định **từ chối `abs(vy)>epsilon` và dừng với UNSUPPORTED_LATERAL**. Một bộ nonholonomic follower theo waypoint với rotate-then-translate cần thiết kế/kiểm riêng, nằm trước Monitor, đồng thời chứng minh route/footprint vẫn khả thi; chưa có trong S1.

Với model Mecanum X lý tưởng, nếu chọn wheel order FL,FR,RL,RR và quy ước roller tương ứng, mapping tham khảo là:

```text
omega = (1/r) * [ vx - vy - (lx+ly)wz,
                  vx + vy + (lx+ly)wz,
                  vx + vy - (lx+ly)wz,
                  vx - vy + (lx+ly)wz ]
```

Đây là thiết kế hình học, **không khẳng định là wheel order/dấu/radius của JetAuto hoặc model chưa chọn**. Cần đo/import đúng thông số, kiểm lệnh riêng +vx, +vy, +wz, inverse mapping và saturation; sau đó mới tái sử dụng Twist của Navigator. Không so chất lượng navigation Mecanum với UGV S0 như cùng actuator.

### 6.7 Gate trước tích hợp chính thức

| Kiểm bắt buộc tiếp theo | Hiện tại |
| --- | --- |
| SearchMission/Navigator/Monitor chạy thật trong Webots; không bypass Monitor | UNVERIFIED |
| Calibration scan 4 hướng, pixel angle, extrinsic; target transform tại nhiều yaw | UNVERIFIED |
| Pose/scan/target stale, NaN/-Inf, stamp tương lai, clock rollback, worker qua epoch | UNVERIFIED |
| Command vx/vy/wz trên model đúng, lateral reject cho vi sai; saturation giữ command an toàn | UNVERIFIED |
| Collision Monitor STOP/SLOWDOWN/invalid source trên scan Webots; braking/calibration vật lý | UNVERIFIED |
| GUI Reset/Reload bấm tay trong S1 | UNVERIFIED; lần này dùng API có log; S0 có thao tác người dùng |
| Detector RGB, depth association, camera robot thật, sim-to-real | UNVERIFIED |
| FPS/RAM dài hạn, scene lớn, nhiều robot | UNVERIFIED; không đo trong S1 |

## 7. Đề xuất nâng hình ảnh với RAM thấp — chưa triển khai

Mục tiêu bước sau là **một scene phòng nhỏ**, giữ nguyên fixture sensor/physics để so sánh chức năng trước/sau. Toàn bộ ngân sách dưới đây là đề xuất kỹ thuật, **UNVERIFIED trên laptop**, không phải cam kết hiệu năng. Snapshot RAM Available 876 MB ở S0 tiếp tục là lý do giữ scene nhỏ; không mở Blender và Webots cùng lúc khi authoring trên máy này.

1. **Blender làm offline asset authoring.** Dựng vài prop low-poly (bàn, hộp, cửa, mannequin); dùng mét, apply scale/rotation, kiểm trục +Z up và kích thước trước export. Decimate có kiểm silhouette, loại mặt khuất; bake chi tiết từ high-poly sang normal/base-color của low-poly thay vì giữ subdivision/displacement. Nguồn: [Blender Decimate](https://docs.blender.org/manual/en/5.0/modeling/modifiers/generate/decimate.html), [Blender texture baking](https://docs.blender.org/manual/en/3.6/render/cycles/baking.html). Không cần chạy high-poly/bake cùng simulator.
2. **Import geometry đơn giản, material khai báo trong Webots.** Export OBJ rồi dùng `Mesh` URL tương đối, kiểm normals/winding và scale. Không giả định Blender Principled shader hoặc glTF material tự chuyển đầy đủ sang Webots. Webots R2025a Mesh hỗ trợ OBJ/DAE/STL; boundingObject dùng Box/Cylinder hoặc proxy đơn giản riêng. Giữ visual silhouette của vật cản chính khớp collision để LiDAR/camera và contact không kể hai câu chuyện khác nhau. [Mesh R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/mesh.md).
3. **PBR nhỏ trước.** Dùng `PBRAppearance` cho sàn/tường/prop với `metalness 0`, roughness khoảng 0.65–0.9 cho vật liệu phi kim; một baseColorMap tileable hoặc atlas, tùy nhu cầu thêm normalMap/roughnessMap. Dùng DEF/USE cho vật liệu/geometry lặp lại. Không mặc định channel-packed ORM hoạt động: PBRAppearance có map riêng, cần kiểm channel/import. Một DirectionalLight và ánh sáng ambient vừa đủ trước khi thêm hiệu ứng; chưa cần bloom, realtime AO, motion blur, nhiều shadow lights hoặc HDRI lớn. [PBRAppearance R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/pbrappearance.md).

| Hạng mục | Ngân sách khởi đầu đề xuất |
| --- | --- |
| Visual geometry | Khoảng ≤ 30k triangles cho cả scene nhỏ; props 0.5–2k, mannequin ≤ 5k |
| Material/texture | 2–4 vật liệu dùng lại; atlas 512², tối đa một bộ 1024² cho focal asset |
| Texture residency | Mục tiêu phần tăng thêm ≤ 32 MiB decoded texture + mipmaps; không dùng kích thước PNG trên đĩa làm RAM |
| Ánh sáng | Một directional, ambient vừa đủ; tắt hiệu ứng hậu kỳ tốn kém lúc đầu |
| Sensor | Giữ 160×120, 180 rays, 96 ms; chỉ thay visual assets từng bước |

Ước tính bảo thủ RGBA8: `width × height × 4` bytes cho level gốc; chuỗi mip đầy đủ khoảng `×4/3`. Một texture 1024² khoảng 5.33 MiB có mipmaps; ba map cùng kích thước khoảng 16 MiB, chưa tính CPU copies, render targets và driver overhead. Một bộ ba 512² khoảng 4 MiB. Vì Iris Xe chia sẻ RAM hệ thống, không coi đây là toàn bộ RAM scene. Không hứa PBR tự làm scene nhẹ hơn.

Sau mỗi thay đổi visual, chạy lại S1, đối chiếu ảnh camera, occlusion, front ranges và baseline stop; pin license/nguồn/hash từng asset. Khi người dùng quyết định mở rộng mới đo RAM/RTF/FPS theo protocol phù hợp; nếu vượt ngân sách thì giảm texture/mesh/material trước. **S1 chưa tải asset, chưa dựng scene Blender/PBR/texture và chưa nghiệm thu ngân sách này.**

## 8. Kết luận và bước tiếp theo

PoC đã có fixture canonical được guard, kiểm reset/reload thực, ca occlusion có đối chứng và raw logs trực tiếp từ controller. **Sẵn sàng bắt đầu triển khai adapter có kiểm soát**, chưa phải tích hợp thuật toán chính thức hoàn tất. Bước tiếp theo nên chốt model đủ động học hoặc lateral-reject contract, kiểm frame/scan/clock/Monitor bằng các ca ở mục 6.7, rồi mới nối SearchMission. Đồ họa và detector ảnh là các nhánh nâng cấp riêng sau hợp đồng sensor/control hoạt động.
