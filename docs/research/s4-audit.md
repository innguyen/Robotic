# S4 audit — High-Fidelity SAR Benchmark Implementation

Cập nhật 2026-11-26, Asia/Saigon; repository innguyen/Robotic, feat/benchmark, tiếp tục từ 2b60605. **Đã triển khai human SAR dev trên native UE; chưa nghiệm thu đầy đủ high-fidelity main benchmark S3.** Không coi số test PASS hoặc primitive pilot là bằng chứng photorealism, physical JetAuto, sensor fidelity hay generalization.

## Nền tảng và phạm vi

Giữ một backend UE 5.6.1/D3D11/PythonScriptPlugin có thể chạy trên máy hiện tại. Không thay engine, không tải environment pack lớn, không sửa production ROS/T1 hoặc historical accepted evidence. [Runtime và lệnh chạy](../../experiments/ue5_s4/README.md), [package](../../sar_benchmark/README.md).

S4 foundation đã xem xét source chính thức [RescueBench](https://github.com/UnrealZoo/RescueBench/tree/6da2490b45301e6e44ae49df6fa2918f5c3701a9), [UnrealZoo v3.0](https://github.com/UnrealZoo/unrealzoo-gym/tree/6ea6aa87eb108d35eb48d953ab96a5201d111062), [UnrealCV](https://github.com/unrealcv/unrealcv/tree/67d466a246c7bacffab5392312b0746191fa5c10), [Project AirSim v1.1.0](https://github.com/iamaisim/ProjectAirSim/tree/be18449c8305641a6bf0d6061f8fd7f405371354). Rescue binary scene khoảng 8 GB và UnrealZoo khoảng 70 GB không được tải; quyền framework source không mặc định cấp quyền sửa/phân phối mọi cooked asset. Native UnrealCV Editor build thiếu NetFxSDK/SwarmInterface; runtime spike chưa build hoàn tất, không suy rằng upstream không hỗ trợ UE5.6. AirSim tuple của S3 cần engine khác và rover không tự là Mecanum. [Source inspection](evidence/s4/upstream-inspection.json), [Editor build](evidence/s4/build-02.txt), [runtime spike](evidence/s4/build-game-01.txt) giữ nguyên.

Không vendor upstream runtime/binary scenes. Native fallback phát triển tiếp implementation hiện có, không dựng backend thứ hai. Engine primitive meshes lấy từ UE cục bộ theo quyền UE hiện có, chỉ phát hành recipe, không phân phối engine assets nguồn.

## Scene và human perception

Human profile dùng original indoor/outdoor rescue layouts: tường gạch/bê tông bị đứt, low walls, column, ceiling beam, debris, partition/occlusion và đường bị chặn. Vật liệu lit PBR với procedural noise/roughness, nguồn sáng movable và low-light condition; không dùng red surrogate trong human experiments. Hai môi trường còn chung một geometry lineage, không được tính là hai independent scene families.

[Cesium Man attribution và model license](../../experiments/ue5_s4/assets/README.md): human mesh CC BY 4.0 từ pinned Khronos source, offline skeletal pose baking sang standing/seated/prone, chuẩn hóa 1,75 m, torso-origin metadata và replacement skin/clothing. Texture logo nguồn không dùng khi render. Chỉ một identity stylized; chưa phải photorealistic humans, injury corpus hoặc held-out appearance evaluation. OBJ/manifest có hashes, import/material cache names phụ thuộc content; [asset integrity](evidence/s4-human/asset-integrity.txt).

Detector thực là official YOLOv5n ONNX v7.0, pinned source GPL-3.0, SHA256 04f0e55c26f58d17145b36045780fe1250d5bd2187543e11568e5141d05b3262. Checkpoint khoảng 3,98 MB được khôi phục bằng downloader có byte cap/hash, ignored, không ghi đè production checkpoint. Letterbox 640/NMS, COCO person, RGB-D median torso patch và estimated pose/extrinsic cho localization. Candidate threshold .4, evaluator/report operating threshold .5; không nới scoring để buộc model đúng.

[Pose render run](evidence/s4-human/pose-renders/results.json) có sáu tổ hợp indoor/outdoor × standing/seated/prone, 90 public captures, UE exit 0; [offline audit](evidence/s4-human/pose-renders-audit.json) PASS. Lượt ngắn này có 0 TP/0 FP, được giữ như evidence miss chứ không coi là detector nghiệm thu. [Standing](evidence/s4-human/frames/poses-epoch-1.png), [seated](evidence/s4-human/frames/poses-epoch-2.png), [prone nhìn dọc thân](evidence/s4-human/frames/poses-epoch-3.png); [image index](evidence/s4-human/frames/index.json) chỉ đúng source/epoch. Góc nhìn prone dọc thân foreshortened rõ; generator hiện dùng side orientation cho prone. Không tự suy chất lượng anatomy hoặc detection coverage chỉ từ mesh tồn tại.

## Robot, sensor và clock

UGV có chassis .4 m square swept collider, wheel/mast visual children; Mecanum X-arrangement inverse/forward kinematics, common wheel-speed saturation và twist acceleration/clearance gate. Human gate dự báo circle sweep qua trọn TTL với .01 s increments, cho phép zero-yaw retreat tăng clearance; đây là planar measured-ray protection, không 3D collision certificate. Source hiện đặt wheel radius .05 m, half-length .13 m, half-width .17 m; geometry là giả định proxy, chưa đo JetAuto. Chassis dịch chuyển bằng .01 s swept increments, rotation được kiểm assertion geometry. Chưa có roller contacts, wheel dynamics, inertia/friction/slip/traction/braking; không dùng nhãn physics realism.

Human profile hỗ trợ 160×120/320×240/640×480 RGB-D và 360/720 rays. Local pilot dùng 320×240, 360 rays, .5 s transactions (2 Hz). Native RGB PNG sRGB và RGBA16F depth EXR optical Z cm→m, K/extrinsic cố định; private visibility dùng isolated rendered depth so với full-scene Z, không dùng boxes của detector làm GT. Scan là ideal complex collision rays, không phải G4 response model. N là synthetic body-frame self-motion increments tích phân trong estimated heading: translation gain 3%, yaw gain 10% cùng yaw bias 2°/m và .01 rad/s ở stress; không phải encoder/IMU đo thật.

Depth có ideal, range-dependent Gaussian và structured bias/edge/range holes/1 mm quantization. Structured nominal bias .006z, Gaussian sigma .002+.002z², severity tăng với P; đây là assumptions công khai, chưa fit Astra. P cũng giảm rendered directional lighting và gain RGB .75; không gọi là khói hoặc IR scattering. RNG frame-seeded, domain-separated theo sensor/model/tick; chưa phải per-pixel counter stream của S3. Không dùng victim coordinates để inject lỗi.

Public capture/available time là transaction task clock; raw private records giữ monotonic software capture start/end/availability thực. Scene tĩnh trong inference/readback, render loop vẫn chạy, không global Chaos lockstep. Chưa có hardware exposure/per-ray time, measured sensor delays hoặc deployment timing mode. Editor FPS cap 10 giúp giảm cạnh tranh CPU, không phải sensor clock.

[Native calibration](evidence/s4-human/native-calibration/calibration.json), [run](evidence/s4-human/native-calibration/results.json), [audit](evidence/s4-human/native-calibration-audit.json): UE exit 0, 15 checks PASS. Plane optical-Z ở .6/1/2/4 m, center và off-axis: max errors .0003125/.000625/.00125/.0025 m. Mười reset epochs khác nhau với repeat pose/scan/RGB cùng máy; rendered visible human 800 pixels; vx/vy/yaw và zero đúng ideal transforms. Đây là renderer/transport calibration, chưa empirical sensor calibration hoặc cross-machine determinism. Các chỉnh sửa ground alignment/wheel dimensions/prone yaw sau pilot có source riêng; mỗi run phải được hiểu theo source snapshot của nó.

## Policies, scenarios và experiments

Ba baseline dùng cùng ONNX weights, preprocessing/depth localizer, observations, report slots, action budget và common safety:

- frontier: persistent free/occupied grid, inflated reachable-frontier selection theo gain/travel, visual sweeps, actual Navigator/Costmap.
- core: actual SearchMission/Navigator/Costmap từ Robotics, thay S2 oracle bằng detections ước lượng.
- verify: frontier + temporal multi-target association, translated viewpoint ≥.2 m, ≥3 observations và position stability. Hai-frame report evidence vẫn theo evaluator chung. Covariance là sample dispersion có synthetic floor, chưa calibrated uncertainty.

Random/sweep giữ sanity diagnostic. ReportTracker không chốt/dedup candidate có mean confidence dưới .5; localization chặn invalid/stale estimated pose. Không dùng GT để merge predictions, chọn viewpoint hoặc báo all-found. Absent không có success oracle.

Dev scenarios có clean, occluded, absent, multi-victim, obstacles, blocked, P-only/N-only/P+N. blocked là auxiliary infeasible navigation challenge; không gộp vào feasible positive corpus để báo recall. Seed replicas không tạo independent families. --admissibility đã triển khai trusted pre-policy conservative circular-footprint grid connectivity và native two-capture visibility; không dùng detector performance để chọn scene. Đây chưa là certificate dynamics hay corpus held-out.

[Pilot trước sửa reset](evidence/s4-human/failed-pilot-summary.json) đã chạy đủ 54 episodes / 3.240 public captures, UE exit 0. [Independent audit](evidence/s4-human/failed-pilot-audit.json) xác nhận data integrity/source/metrics cho 54 ledgers nhưng **run acceptance FAIL** vì requested calibration không đạt RGB repeat. Không cấp COMPLETE hoặc dùng lượt này làm nghiệm thu. Raw local được giữ tại experiments/ue5_s4/runs/sar-dev-complete; [archive index](evidence/s4-human/failed-pilot/archive-provenance.json) đánh dấu partial rõ ràng.

[Đối chiếu hai reset](evidence/s4-human/reset-rgb-difference.json) đo max 1 RGB level, 1.198 pixels thay đổi. Source UE có frame-dependent LDR quantization dither; override 32-bit dither precision riêng lẻ chưa đủ sửa lỗi. Giữ directional light actors/registration IDs cố định qua resets đã khắc phục repeat trong các lượt tiếp theo, phù hợp giả thuyết thứ tự/lifecycle lighting ảnh hưởng floating-point accumulation. Đây là kết luận từ can thiệp và phép đo, chưa phải GPU RenderDoc trace chứng minh từng phép cộng. Output vẫn RGB8; không nới tolerance hay scoring threshold.

[Reset sau sửa](evidence/s4-human/reset-stable/calibration.json) PASS 15 checks, [audit](evidence/s4-human/reset-stable-audit.json) PASS. [Full-resolution run](evidence/s4-human/profile-640/results.json) chạy sáu tổ hợp indoor/outdoor × ba poses, 24 public packets, 59 native captures gồm admission/calibration; 640×480 và 720 rays, UE exit 0. [Independent audit](evidence/s4-human/profile-640-audit.json) PASS cả 24 capture-to-policy timing records, asset manifests, source snapshots, metrics và requested calibration. Mười resets RGB hash khớp chính xác; clock implementations đều GetTickCount64 với resolution .015625 s, không nhầm ns field với độ chính xác exposure.

[Standing 640](evidence/s4-human/frames/native-indoor-standing-640.png), [seated 640](evidence/s4-human/frames/native-indoor-seated-640.png), [prone side view 640](evidence/s4-human/frames/native-indoor-prone-640.png); [native image index](evidence/s4-human/frames/native-640-index.json) chứa epoch/source/hash và visible masks summary. Ảnh là bytes PNG gốc ở policy tick 0, không chỉnh sửa hoặc lấy từ privileged admission viewpoint. Visible pixels lần lượt 3.250 / 2.687 / 2.789. Seated là baked static pose; appearance realism và body support/contact chưa được nghiệm thu.

Matrix mới tại experiments/ue5_s4/runs/sar-pilot-frozen đang chạy 54 episodes trên source snapshot đã sửa reset, có pre-policy admission và 30 s task budget tại 2 Hz. Active-search detour budget 90 s và standard raster QA được chạy riêng. Kết quả cuối sẽ chỉ được ghi sau clean exit, calibration và independent audit; không dùng scheduled count làm completed count.

[sensor_study](../../sar_benchmark/sensor_study.py) triển khai secondary fixed-trajectory replay: cùng RGB/detector boxes/estimated pose, thay ba depth profiles. Chỉ là synthetic sensor sensitivity; Gaussian chưa matched RMSE và chưa có Astra validation, không thay closed-loop RQ3. summarize_run giữ abstention/absent và auxiliary failures, descriptive RQ1/RQ2; không significance/generalization/fixed-FAR claim trên dev corpus này.

## Correctness, security và storage

Giữ evaluator max-cardinality → min squared-XY cost → stable ties; exhaustive matcher và reference scorer tính eligibility/counts/time credit độc lập. Report cap 5, malformed/duplicate penalties, delivered evidence đúng epoch/time, pixel/IoU/XY/z thresholds, unsafe gating, time-integrated recall và matched-inlier uncertainty giữ nguyên. .5 s action clock có regression riêng; report evidence separation vẫn .1 s.

Offline audit kiểm file inventory, journal chain/head, clean engine exit, source snapshot, worker exit, public schema, delivered RGB hash với ledger, report evidence thực sự đã phát, monotonic software readback intervals và asset manifests khi run có ghi. Backward metadata thiếu ở earlier dev runs được ghi NOT_RECORDED, không bịa đã kiểm.

Agent nhận public measurements qua stdin; private engine token/endpoints, GT anchors, masks, ledgers và scoring không forward. Allowlist reject nested canaries; ACK chỉ slot/received. Worker cùng OS user/filesystem/network nên chưa chặn agent đối kháng. Algorithm hiện dùng cùng base replicate seed với faults nhưng domain-separated RNG, không có seed secrecy. All runs scientific_main_eligible=false. Upstream RescueBench hooks target_pose/task_context/env bindings không được reuse để cấp GT cho policies.

Run directories exclusive, không overwrite/resume chọn best. Watchdog mới bao phủ serialization/stdin write và response; child không đọc 2 MB input được terminate đúng hạn trong regression. Private QA export failure là INFRA_ERROR, không đổ lỗi policy hoặc cấp COMPLETE. Source snapshots chỉ scripts/config cần thiết, không Saved/CrashReportClient settings. Native PNG/EXR lossless trong private raw; scratch capture-tmp không thuộc portable inventory. archive_run tạo archive mới, giữ original inventory/provenance; raw quá lớn giữ local ignored paths kèm hashes và đánh dấu PARTIAL_LOCAL_RAW_INDEX, không gọi partial archive là portable audited run. Không commit weights/shader/engine caches. Các failed/aborted attempts được giữ và phân loại, không gộp vào completed pilot.

## Regression và historical evidence

Canonical Webots S2 world đã regenerate đúng generator found fixture; regression mới 14/14 PASS. Historical worlds/results/evidence snapshots không chỉnh. S4 foundation 48 episodes vẫn nguyên tại [pilot-02](evidence/s4/pilot-02/results.json), [audit](evidence/s4/pilot-02-offline-audit.json); đây là 80×60 red-primitive diagnostic, không phải human results. Pilot-01 vẫn ghi engine exit 3221225477 và không được xem clean lifecycle PASS. [Diagnostic calibration](evidence/s4/calibration-01/calibration.json) giữ 11 checks và forced collision/safety probes riêng.

| Regression trên implementation mới | Kết quả thực chạy |
| --- | --- |
| [S4 regression mới](evidence/s4-human/unit-final-io-watchdog.txt) | 59 PASS; gồm schema/privacy, independent scoring, tracking/translated-view, threshold dedup, Mecanum, stale pose, bounded compression bomb và directional safety/retreat và watchdog cho blocked stdin |
| [T1 đầy đủ](evidence/s4-human/t1-regression.txt) | 239 PASS, offline; discovery rộng hơn bộ 143 tests của evidence cũ |
| [Godot](evidence/s4-human/godot-regression.txt) | 5 PASS, protocol tests; không chạy lại Godot runtime |
| [Webots S2](evidence/s4-human/webots-regression.txt) | 14 PASS sau regenerate canonical world; không sửa historical evidence |
| [Preservation/syntax/link QA](evidence/s4-human/preservation-01.json) | 5.859 protected tracked files không đổi; đây không phải simulator test, sẽ kiểm lại cuối bàn giao |

Preservation QA bảo vệ production, T1, S3 và tracked historical evidence; chỉ whitelist implementation S4, audit/tools và đúng canonical Webots world được yêu cầu. [Runtime environment capture](evidence/s4-human/runtime-environment.json) ghi Python/OpenCV/NumPy và binary hash quan sát trong phiên; không giả rằng earlier manifests đã attest package versions tại thời điểm launch.

## Đối chiếu S3 và chuyển S5

| Hạng mục | Đã triển khai/kiểm được | Còn thiếu để nghiệm thu main S3 |
| --- | --- | --- |
| Runtime/evaluator | Native UE thật, RGB-D/control/reset, private/public lanes, independent scoring và artifacts | OS/network isolation, soak/cross-machine, full dynamic clock/contact proof |
| Human perception | Licensed human mesh, ba poses, RGB ONNX detector/localizer, native masks | Photorealistic/identity diversity, held-out image corpus/AP, texture/scene realism acceptance |
| Environments | Indoor/outdoor rescue dev strata, occlusion/debris/navigation failures | 48 lineage-disjoint families/splits, frozen admissibility and exposure strata |
| Sensors | Native 640×480/720-ray đã kiểm ở short static suite; depth-plane/transform/reset và synthetic response | 300 s/10 Hz soak/timing, exposure/per-ray clocks, IMU/encoders, empirical Astra/G4 sessions và held-out fits |
| Embodiment | Visual UGV, swept proxy, Mecanum kinematics/safety | Measured JetAuto geometry, roller/contact dynamics, braking/slip/real trajectory validation |
| Methods/experiments | Frontier/core/active verification, paired P/N and synthetic replay plumbing | 300 s/10 Hz accepted missions, strong semantic baseline, empirical RQ3, power/confirmatory study |

**Chưa đủ điều kiện chuyển sang S5 confirmatory scientific experiments.** Có thể dùng implementation cho S5 engineering/integration trên graphics worker, nhưng không đổi tên dev pilot thành main result. Blockers cụ thể: Iris Xe/RAM 8 GB không đủ cơ sở nghiệm thu photorealistic 300 s/10 Hz throughput; chưa được phép tải scene packs lớn; chưa có diverse licensed human/environment corpus, Astra/G4/encoder calibration sessions hoặc OS sandbox. Không tìm thấy real sensor recordings trong các source liên quan đã kiểm; simulator calibration logs S2 không thay real calibration.

[Worker command](../../experiments/ue5_s4/README.md) đã chuẩn bị cho 640×480/720 rays/3000 ticks với pre-policy admission và calibration. Trước confirmatory S5 phải khóa licensed asset lineage/splits trên graphics workstation, đo full sensor/clock/stop/contacts, fit và validate real sensor models, nghiệm thu physical track riêng và cô lập agent. Không thay tài liệu S3 để khớp các giới hạn hiện tại; không tự nhận rank/significance từ một layout lineage.

Không commit/push/publish/deploy. Rollback code S4 độc lập; canonical Webots world có thể regenerate bằng generator. Không cần thay production/T1 hoặc historical accepted results.
