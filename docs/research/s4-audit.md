# S4 audit — SAR Benchmark Foundation & MVP

Bàn giao: **2026-11-26, Asia/Saigon**, repository `innguyen/Robotic`, nhánh `feat/benchmark`. **Foundation Python và vertical slice UE thật đã triển khai. Main high-fidelity research benchmark chưa được nghiệm thu.** Không dùng kết quả fixture để khẳng định perception robustness của người thật, physical JetAuto/Mecanum, novelty hay kết quả RQ có ý nghĩa thống kê.

## 1. Quyết định nền tảng

**Chọn một host tối giản UE 5.6.1 native, qua PythonScriptPlugin đã cài, D3D11.** Đây là fallback có bằng chứng thực thi, thay candidate Project AirSim/UE5.8 ở S3. Không đổi tài liệu S3 thành báo cáo implementation. Không tải Rescue environment ~8 GB, UnrealZoo ~70 GB, engine khác hoặc model mới.

Đã ưu tiên đọc paper RescueBench arXiv:2606.01848v1 cùng source chính thức RescueBench/UnrealZoo, sau đó đối chiếu Project AirSim. Phân biệt framework Python, plugin C++, cooked environment và quyền source/assets:

| Nền tảng / source đã khóa | Kết luận cho S4 trên máy hiện tại |
| --- | --- |
| [RescueBench](https://github.com/UnrealZoo/RescueBench/tree/6da2490b45301e6e44ae49df6fa2918f5c3701a9) | Gần SAR nhất; có baseline adapters/task tooling, nhưng nhiệm vụ bốn giai đoạn khác search/report. Public runner có đường đưa GT/env tới agent. Binary scene không phải source UE có thể tùy ý sửa sensor hoặc phân phối. Không dùng nguyên runner làm main của nhóm |
| [UnrealZoo v3.0](https://github.com/UnrealZoo/unrealzoo-gym/tree/6ea6aa87eb108d35eb48d953ab96a5201d111062) / [v3.1](https://github.com/UnrealZoo/unrealzoo-gym/tree/01f56d4180c8dba2f5ca65430a61cb59e76b2ba0) | v3.0 phù hợp UE5.6 hơn v3.1/UE5.7. Có RGB-D, robotics/control/scene tooling; v3.1 có LiDAR. UnrealCV+ phát hành qua môi trường không đồng nghĩa mọi tính năng server mở source; không mặc định có JetAuto hoặc global stepping |
| [UnrealCV source](https://github.com/unrealcv/unrealcv/tree/67d466a246c7bacffab5392312b0746191fa5c10) | Đã tải **sparse source**, không scene packages. Editor build dừng ở thiếu NetFxSDK/SwarmInterface. Runtime spike chưa build hoàn tất: integration header include trong spike không tìm thấy. Đây không phải bằng chứng UnrealCV không hỗ trợ UE5.6. Không giữ spike C++ chưa dùng trong deliverable |
| [Project AirSim v1.1.0](https://github.com/iamaisim/ProjectAirSim/tree/be18449c8305641a6bf0d6061f8fd7f405371354) | Source có Rover/SimpleDrive, Python/sensor extension. Native wheeled vehicle không tương đương Mecanum. Tuple release ở S3 cần UE5.8; máy chỉ xác minh UE5.6.1. Không cài engine hay binary khác để thử. Engine-driven vehicle clock không được coi mặc định pause/step như drone |

Nguồn paper: [RescueBench full text và D.2](https://arxiv.org/html/2606.01848v1). [Source ledger](evidence/s4/upstream-inspection.json), [build Editor](evidence/s4/build-02.txt), [build runtime spike](evidence/s4/build-game-01.txt). Quyết định fallback nhằm có pipeline đo đúng với hạ tầng được phép; không xếp hạng tổng quát chất lượng ba framework.

Code RescueBench/UnrealZoo được mô tả Apache-2.0, Project AirSim MIT; kiểm code license không cấp quyền tất cả scenes/checkpoints. S4 không vendor assets của chúng. Engine meshes dùng từ bản UE cục bộ theo quyền UE hiện có, chỉ phát hành recipe/script; không phát hành lại engine mesh nguồn. Vật liệu màu và scene layout tự dựng để diagnostic, không là human asset corpus có license đã kiểm.

## 2. Audit ground-truth leakage

Đường dữ liệu cụ thể đã kiểm:

1. [`state_machine_wrapper.prepare_info`](https://github.com/UnrealZoo/RescueBench/blob/6da2490b45301e6e44ae49df6fa2918f5c3701a9/benchmark/wrappers/state_machine_wrapper.py) thêm `info['target_pose']` từ target của task state machine.
2. [`episode_runner.run_episode`](https://github.com/UnrealZoo/RescueBench/blob/6da2490b45301e6e44ae49df6fa2918f5c3701a9/benchmark/core/episode_runner.py) gọi `agent.prepare_episode(task_context)` trước reset; context chứa injured/stretcher/agent/ambulance poses. Sau đó `prepare_step_inputs(env, obs, info)` nhận cả object environment và enriched info.
3. [`agent_base`](https://github.com/UnrealZoo/RescueBench/blob/6da2490b45301e6e44ae49df6fa2918f5c3701a9/benchmark/agents/agent_base.py) mặc định chuyển nguyên obs/info tới `act`. [`SolutionAgent`](https://github.com/UnrealZoo/RescueBench/blob/6da2490b45301e6e44ae49df6fa2918f5c3701a9/benchmark/agents/solution_agent.py) có thể chuyển info vào reset/predicts của solution.
4. [`SGNavRescueAgent.prepare_step_inputs`](https://github.com/UnrealZoo/RescueBench/blob/6da2490b45301e6e44ae49df6fa2918f5c3701a9/benchmark/agents/sgnav_rescue_agent.py) bind env rồi đọc `obj_poses[protagonist_id]` làm self pose. Đây là privileged self localization; không tự suy rằng mọi baseline dùng injured GT để navigation.
5. UnrealCV có actor poses, object masks, reflection/Blueprint calls và console commands. Chỉ bỏ field `target_pose` chưa đóng các đường đặc quyền khác. Task state machine cũng dùng GT proximity để trợ giúp carry/drop; không mang trigger đó vào scoring search/report.

S4 không gọi các hooks trên. Agent chỉ nhận allowlisted JSON observations; không nhận env/client/evaluator object, `info`, reward, masks, victim count, fault seed, scenario identity hoặc đường dẫn GT. Tests đặt canary `target_pose` ở mọi object public và kiểm reject. Report ACK chỉ nhận/slot, không matching/success.

**Giới hạn security:** process riêng cùng Windows user có thể đọc filesystem/network nếu là mã đối kháng. Chưa có OS sandbox/test server isolation; tất cả run gắn `scientific_main_eligible=false`. Đây là API/data-flow separation đã test cho local honest policies, chưa là chứng minh hostile-agent isolation. Không có main-track result nào được cấp từ profile này.

## 3. Kiến trúc và implementation

- [`sar_benchmark/`](../../sar_benchmark/README.md): scenario/public observation/action/report validation, lifecycle/reset/epoch, common safety, blind reporter, independent evaluator, manifests/run store/runner, statistics và offline audit.
- [`experiments/ue5_s4/`](../../experiments/ue5_s4/README.md): một native UE host, scene primitive có rubble/occluder/victim surrogate, swept kinematic UGV, RGB/depth và collision-ray scan. Không thay project UE của người dùng trên ổ A.
- Report tối đa 5 slots cho mọi K=0..3; malformed slots/confidence không tạo loophole né FP, duplicate ID bị penalty, duplicate victim không tăng TP. Evidence yêu cầu hai delivered captures đúng epoch/time, đủ visible pixels/IoU, XY≤1 m và |Δz|≤.75 m.
- Matcher DP tối đa cardinality → tối thiểu squared XY error → stable tie-break; exhaustive matcher và một reference scorer tính eligibility/counts/time credit độc lập. Prefix matching giữ đúng time credit khi assignment cuối thay đổi.
- Metrics: SLR/recall/precision, FP, duplicate/invalid reports, absent FAR/specificity, completeness, time-integrated recall, restricted first-report time, path, XY median/p90, Brier và optional covariance XY NLL/coverage trên matched inliers. Không có covariance output của fixture policies thì ghi unavailable, không bịa uncertainty.
- `statistics.rq1` kiểm đủ bốn paired cells, average seeds trong family rồi family-weighted interaction. Một family không có cluster CI. Không dùng seeds làm independent scene families.
- Run directories tạo exclusive, không overwrite/resume chọn best; source snapshot, config/model/profile hashes, separate public/private raw gzip JSONL, immutable reports, command chain và file inventories. Agent exception giữ outcome, infrastructure failure chặn complete result; final runner kiểm exit code của engine.

Tái sử dụng thực từ Robotics: `SearchMission`, `Navigator`/costmap, footprint config của core qua `CoreSearchPolicy`; SAT geometry S2 để assertion; port preprocessing/YOLOv5 decode từ production sang optional ROS-free detector. Không tái sử dụng oracle recognition hoặc target poses S2. Không có binary/assets/baseline weights của RescueBench, UnrealZoo hay Project AirSim được chạy/tái phân phối; reuse ở đây là thiết kế boundary và audit interface, không claim tích hợp upstream runtime.

## 4. Bằng chứng thực thi và verification

[Preflight S4](evidence/s4-preflight.json) xác nhận engine cục bộ; [depth probe](evidence/s4/editor-probe-01.json) đọc 4.800 samples từ UE 5.6.1. Center depth **259,75 cm**, so với mặt phẳng trước box **260 cm**. Đây là sanity ở một range, không calibration Astra. Raw UE log xác nhận Iris Xe, D3D11/SM5; không dùng NullRHI.

[Engine/asset identity](evidence/s4/engine-asset-identity.json) lưu build/changelist, SHA256 editor, PythonScriptPlugin, D3D11 module và mesh cục bộ; checkpoint ONNX có sẵn cũng được hash và ghi chưa xác minh provenance/license. Không copy những engine files đó vào repository.

[Runtime smoke 04](evidence/s4/runtime-04/results.json) có hai episode: random và sweep đều TP=1, FP=0; sweep XY error khoảng **0,151 m**. Các lần trước được giữ nguyên: startup callback thiếu keep-alive ở runtime-01; report bị mép ảnh/thiếu evidence ở runtime-02/03. Đã sửa policy confirmation và barrier capture, không nới evaluator để làm report đó đúng.

[Pilot 01](evidence/s4/pilot-01/results.json) ghi đủ **32 episodes, 960 public packets**, 2 methods ×2 seeds ×8 cases. [Offline audit](evidence/s4/pilot-01-offline-audit.json) đối chiếu independent scorer, source snapshots, hashes, public schema: không disagreement. **Engine exit code 3221225477 sau khi đã ghi log “Exiting”**; không xem lượt này là clean lifecycle PASS. Giữ nguyên evidence gốc và sửa runner để exit code khác 0 không được cấp complete status. Lượt tiếp theo và calibration được ghi bổ sung ở phần nghiệm thu cuối.

**Lượt nghiệm thu cuối:** [pilot-02](evidence/s4/pilot-02/results.json) hoàn tất **48 episodes / 1.440 public packets**, 3 methods ×2 seeds ×8 cases; UE exit **0**, wall duration **442,27 s**. Có **26 TP, 0 FP, 0 unsafe episodes** trong fixture; các miss vẫn ở denominator. [Offline audit cuối](evidence/s4/pilot-02-offline-audit.json) PASS inventory/source snapshots/public allowlist/reference scorer cho cả 48 episodes. Đây là pilot theo source snapshot đã lưu, không phải main-track score.

| Nominal condition, mean SLR qua 2 seeds | Random | Sweep | Core SearchMission/Navigator |
| --- | --- | --- | --- |
| Clean | 0,50 | 1,00 | 1,00 |
| Occluded | 0,50 | 1,00 | 1,00 |
| Multi-victim | 0,75 | 1,00 | 1,00 |

Absent và sensor-black đều không phát report ở cả ba methods. P và P+N đều ở floor 0 của marker detector; N-only giữ clean SLR. **RQ1 I=0 cho cả ba methods, không CI vì chỉ có 1 family**. Không có bằng chứng statistical robustness hoặc ranking tổng quát từ bảng này.

Adversarial replay trên real delivered evidence cho **TP=1, FP=2** khi thêm một duplicate và một false-location report. Đối chứng dịch report coordinates `.7 m` theo từng factor cho bốn recall cells `[1,1,1,0]`, **I=−1**: chứng minh evaluator/contrast đo được threshold interaction. Đây chỉ là **offline report perturbation**, không phải sensor intervention trên UE hoặc kết quả khoa học RQ1.

[Calibration cuối](evidence/s4/calibration-01/calibration.json) trên native engine transforms đã **PASS 11 checks**: forward .03 m, left .02 m, yaw-left .06 rad trong một transaction; zero ở cả ba trục; 10 reset epochs riêng và pose/scan/RGB hashes lặp đúng trên cùng máy; swept collision chặn wall tại x≈**0,348997 m**, có CONTACT; common gate trả zero gần wall. [Lượt calibration](evidence/s4/calibration-01/results.json) UE exit **0**, kèm 3 short smoke episodes và source snapshot mới. Collision forcing là privileged engineering probe riêng, không phải policy run hoặc safety-certified result. Không suy bitwise determinism cross-machine từ 10 resets này.

Kiểm tra đã chạy:

| Check | Kết quả đã quan sát |
| --- | --- |
| [Unit/integration/adversarial S4 cuối](evidence/s4/unit-final-03.txt) | **46 PASS**, gồm 500 random matcher graphs +150 randomized independent eligibility/metric comparisons, strict JSON, covariance, extreme finite/overflow, frames, resets, safety, report caps, real worker process, core adapter, source-snapshot filter và ONNX inference black image bằng model có sẵn |
| [T1 regression](evidence/s4/t1-regression.txt) | **143 PASS**, offline |
| [Godot protocol regression](evidence/s4/godot-regression.txt) | **5 PASS**, không chạy Godot runtime lại |
| [Webots S2 regression](evidence/s4/webots-regression.txt) | **13 PASS, 1 FAIL**: canonical world khác generated fixture. Blob local và HEAD cùng `317ab24ecbbd26ce9a25ce20644899727e4de3b0`; lỗi nhất quán đã có trong baseline, không sửa world/core trong S4 |
| [Preservation và artifact QA cuối](evidence/s4/verification-final.json) | **5.196 protected tracked files không đổi so HEAD**, 77 local links hợp lệ; không nhận preserved hash là simulator runtime test |

Lần thử regression bằng MSYS Python thiếu NumPy cũng được giữ tại [attempt log](evidence/s4/webots-regression-attempt.txt). Sau đó dùng Python310 với NumPy/OpenCV đã cài; không cài/upgrade production dependencies.

## 5. Fidelity và giới hạn

Profile triển khai là `ue5_static_proxy_rgbd_scan_v1`, khác S3 main contract: RGB/depth **80×60**, 72 ideal engine collision rays, synthetic odometry từ actual self-motion increments, không có wheel encoder/IMU. Camera cùng FOV/extrinsic; base frame ở floor projection, collider center cao .2 m. Depth Z dùng readback RGBA16F, đổi cm→m đúng một lần; có quantization. Native scene/rendering không đồng nghĩa empirical sensor response.

Host chờ editor frames sau reset/motion. Task time là integer **100 ms/transaction**, robot dịch chuyển bằng 10 swept kinematic increments; scene tĩnh trong inference/capture. Engine render loop tiếp tục, **không global Chaos lockstep**, không dynamics bánh xe và không “physics 100 Hz đã xác minh”. Scan và private visibility lấy engine collision rays; primitive collision/visual geometry mới chỉ ở fixture đơn giản.

Hai baseline nhẹ là seeded random và systematic sweep, dùng cùng detector/depth/report/safety. Core baseline dùng actual Robotics algorithm. Detector fixture là red connected-components từ ảnh thật; không phải person detection. Optional ONNX adapter đã chạy inference, nhưng chưa có human mesh/checkpoint provenance/pose diversity/calibration để gọi human-perception main baseline. Stress P là post-render synthetic darkening/depth bias+holes; N là synthetic pose increment drift; không gọi là khói, Astra/G4 hoặc calibrated wheel slip.

Một layout family, hai seeds, short 3 s task budget, primitive seated surrogate, không train/validation/test corpus, không held-out identities/real-robot data/second machine/soak. P pilot có floor effect; RQ1 interaction descriptive bằng 0 không chứng minh không có interaction trên SAR thực. Offline report-coordinate perturbation chỉ kiểm độ nhạy evaluator, không thay thí nghiệm can thiệp sensor thật.

## 6. Mức độ hoàn thành

**Đã có implementation có thể chạy và chấm đúng ở phạm vi diagnostic MVP. Chưa đạt trọn main high-fidelity benchmark S3.** Foundation contracts/evaluator, real UE RGB-D/control/reset và fair local baselines có raw evidence. Mọi PASS trong tài liệu gắn đúng phạm vi; không tuyên bố hoàn thành secure evaluation, photorealistic corpus, physical Mecanum hoặc kết luận nghiên cứu.

## 7. Chuyển sang S5

1. Hoàn thiện native runtime integration trên toolchain có NetFxSDK hoặc worker tương thích; chỉ giữ một backend được nghiệm thu. Với UnrealCV/UnrealZoo, chặn privileged endpoint bằng OS/network isolation; freeze source/plugin/binary/assets tuple, không chỉ tag.
2. Thay marker surrogate bằng licensed human meshes, torso anchors/visible masks, seated/prone/standing và occlusion strata; kiểm source/cooked/rendered/training rights từng asset. Chạy common ONNX detector và các baseline mạnh B1/B2/B3 cùng inputs/budget.
3. Nghiệm thu full clock, real sensor capture stamps, 640×480 RGB-D/720 scan nếu dùng S3 profile, IMU/encoders, sync/depth planes/multi-range/extrinsics; fit synthetic response từ Astra/G4 rồi held-out validation.
4. Dynamics Mecanum riêng: wheel/roller geometry, inertia/friction/slip/braking so với dữ liệu thật. Giữ ideal proxy là track riêng.
5. Dev tuning/power theo scene families, đóng băng evaluator/thresholds/hypotheses, mở rộng asset lineage splits và independent graphics worker; run longer missions/soak và failure adjudication. Chưa có đủ family thì tiếp tục báo descriptive, không CI giả theo frame/seeds.

Không commit/push/publish/deploy. Rollback: bỏ riêng các thư mục source/evidence S4 mới và phần liên kết S4 trong README; không cần chỉnh T1/Webots/Godot/production. Source cache/build products không thuộc deliverable.
