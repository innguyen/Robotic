# Evaluation protocol — `sar-s3-draft-0.1`

**DESIGN, chưa triển khai evaluator UE5, chưa khóa corpus, chưa có kết quả benchmark.** Các số dưới đây là default đề xuất để kiểm trên dev rồi freeze. Thay task, matching, sensor profile hoặc success threshold sau freeze phải tăng protocol version và chạy lại toàn bộ methods bị ảnh hưởng.

## 1. Task và episode

Một robot UGV tìm **0–3 victim tĩnh** trong vùng một tầng đi được, phát hiện từ sensor và báo tọa độ. Không biết K, tọa độ hay vị trí viewpoint từ đầu. Maximum **5 reports/episode**, cố định cả absent và multi-victim để không lộ K. Được dừng tìm sớm (`finish`) nhưng không có “all found” oracle feedback. `finish` biểu thị agent đã chọn kết thúc, không tự chứng minh không còn nạn nhân.

Budget **300 s simulation time**, action period **0,1 s**, physics target **0,01 s** (10 substeps/action) cho profile lý tưởng. Engine/backend phải chứng minh được đúng clock và contact sampling; không gắn nhãn lockstep chỉ vì có API `step`. Deadline dùng integer ticks, không tích lũy float. Một report tại đúng deadline chỉ được nhận nếu observation và request đã được chấp nhận trước khi deadline đóng, rồi được ghi cùng terminal tick; không gọi policy mới tại deadline. `finish`, timeout, safety abort, agent crash đều có terminal record và zero command.

Observation đầu tiên chỉ phát sau reset, sensor warmup và barrier đúng epoch. Warmup không được tạo đường đi/mapping cho agent; thời gian warmup ghi riêng. Evaluation budget bắt đầu tại observation đầu hợp lệ. Mọi frame gửi agent trước deadline đều được giữ để offline verification. Ground truth được lấy mỗi physics tick; report GT dùng capture time đúng epoch và ground-truth target anchor, không dùng estimated pose của agent làm GT.

Không require tiếp cận/chạm victim cho primary task; có thể report từ xa nếu localization và evidence đúng. Task phụ **safe approach** chỉ chấm trên subset có approach pose đi được, tách bảng: footprint-to-victim collider distance trong [0,5;1,0] m, không contact, tốc độ tuyến tính ≤0,01 m/s và yaw ≤0,01 rad/s liên tục 1 s. Đây là convention simulator, **không phải khoảng cách an toàn real**. Không dùng FOUND của state machine thay independent scoring.

## 2. Agent observations

Main track **RGBD-L-ODOM**. Profile **RGBD-L-POSE** với self pose lý tưởng chỉ là diagnostic track. Camera/robot/sensor profile phải giống giữa baselines; không đổi camera resolution/FOV theo method rồi so như cùng input. Mọi resize/inference preprocessing nằm trong agent và được ghi.

| Trường | Hợp đồng main track |
| --- | --- |
| Header | Opaque episode ID, `epoch`, `tick`, `sim_time_ns`, packet sequence, protocol/profile IDs. Không gửi generator seed, difficulty label hoặc scene-family metadata |
| RGB | RGB8 640×480, 10 Hz, intrinsics K/distortion, exposure metadata nếu có thật; `capture_start_ns`, `capture_end_ns`, `available_ns`, `frame_id`, valid flag và payload hash |
| Depth | Optical-axis Z float32 mét, 640×480 sau registration đã định nghĩa; validity bitmap riêng. Raw simulated clean depth và ideal masks chỉ ở trusted channel. Invalid không tự biến thành free space |
| Camera geometry | Tách native RGB/depth intrinsics và `T_base_rgb_optical`, `T_base_depth_optical`; registered depth dùng RGB ray grid và đơn vị Z đã công bố. FOV lấy từ calibration/profile, không đoán camera S2 tương đương Astra |
| LiDAR | Planar profile 720 bins/360°/10 Hz, 0,05–8 m **giá trị thiết kế**, extrinsic và per-ray time offset. No-return `null` kèm status `no_return`; `invalid`/`dropped` có mask riêng; binary transport có thể dùng +Inf nhưng JSON logs phải chuẩn |
| Proprioception | Wheel odometry estimate `T_episode_base_est`, measured/estimated twist và covariance, IMU angular velocity/acceleration ở rate công bố. Không tự gắn covariance có vẻ chính xác lên pose GT |
| Status | Sensor freshness/validity, limits và elapsed budget; không có target ID, visibility GT, nearest-victim distance, map coverage oracle, navmesh, evaluator success reward hoặc semantic segmentation |

Asynchronous sensor packets giữ timestamp capture/delivery riêng. Main synchronous profile cung cấp pose interpolated estimate tại camera/scan capture; nếu pose bracket không đủ thì flag invalid, không lấy pose mới ghép ảnh cũ. Max age **0,3 s** cho scan/odom; max depth–RGB skew **0,03 s**, camera age **0,5 s** là defaults cần dev validation. Fault injection có thể cố tình vi phạm: stop/freshness decision thuộc safety/agent, evaluator vẫn giữ fault record và đánh giá outcome. Transport không retimestamp buffer giữ nguyên thành dữ liệu mới.

Missing packet do lỗi infrastructure phải phân biệt intentional dropout: trusted fault schedule + packet sequence + engine log. Không tự đánh dấu mọi thiếu dữ liệu là benchmark failure của agent hoặc ngược lại.

## 3. Actions và reports

Main embodiment `ugv_omni_proxy_v1`: base FLU twist `(vx,vy,wz)` m/s, m/s, rad/s trong đúng một action interval, có epoch và sequence. Limits đề xuất vx ∈[−0,1;0,3], |vy|≤0,2, |wz|≤0,6. Gia tốc tuyến tính cap 0,5 m/s² và yaw cap 1 rad/s² là **actuator response model**, không kết quả hiệu chuẩn JetAuto. Cap áp trên vector, ghi desired/limited/safety_checked/applied; Monitor phải kiểm **lệnh sau mọi transform/limits**. Nếu native Ackermann SUV chỉ có throttle/steering/brake, nó thuộc platform smoke profile riêng; không cắt vy để đưa main baseline vào đó.

Mỗi command hết hạn ở cuối interval. Disconnect/invalid epoch/invalid finite values/timeout → zero và terminal hoặc sensor-safe gate theo lỗi; command bất hợp lệ không được làm tiến world một phần. Reset hoàn tất mới nhận command mới. Policy không có teleport, spawn, sensor reposition hoặc actor-query API; camera cố định ở main track. `finish` → zero và giữ 1 s để kiểm stop, không dùng hold time để thêm report.

Report schema logic (wire implementation còn UNVERIFIED):

```json
{
  "report_id": "r1", "epoch": 7, "tick": 120,
  "label": "victim_person", "position_episode_m": [2.2, -1.1, 0.65],
  "position_covariance_m2": [[0.1,0,0],[0,0.1,0],[0,0,0.2]],
  "confidence": 0.85,
  "evidence": [
    {"rgb_sequence": 118, "bbox_xyxy_px": [180,130,270,340]},
    {"rgb_sequence": 120, "bbox_xyxy_px": [182,131,273,342]}
  ]
}
```

Tọa độ là **torso anchor 3D** của victim, định nghĩa nhất quán theo skeletal/mesh metadata; main localization score dùng XY và |z error|≤0,75 m (để tránh report nhầm độ cao). Không chấm tâm toàn mesh thay torso rồi đổi định nghĩa theo tư thế. Agent biết cách định nghĩa anchor, không biết anchor GT. Covariance phải finite, symmetric positive definite; absent covariance bị ghi `uncertainty_unavailable`, không làm report đúng thành sai nhưng không đủ điều kiện uncertainty table. Confidence ∈[0,1], số hữu hạn; cấm schema fields lạ/GT IDs/path tùy ý.

Hai evidence frames **đã gửi tới agent** cùng epoch, cách nhau ≥0,1 s, không tương lai, frame mới nhất ≤2 s trước report. Bbox hợp lệ trong ảnh gốc; evaluator xác định cùng một victim bằng IoU ≥0,25 với visible-mask bounding box ở cả hai frame, mỗi frame có ≥25 victim pixels. Đây là evidence eligibility, **không phải thuật toán nhận dạng cấp cho agent**. Full occlusion, poster và absent không được cấp eligibility giả. Kiểm sensitivity với pixel/IoU thresholds trên dev để tránh loại oan target nhỏ; đóng băng trước test. Agent có thể report nhầm; evaluator chỉ trả ACK đã nhận, không trả matched/GT error trong episode.

Report không sửa/xóa sau ACK. Duplicate report IDs bị reject và log; report mới trùng victim vẫn dùng một report budget và có thể là FP. Report vi phạm schema/evidence dùng một slot trong 5 slots và được tính FP nếu confidence vượt operating threshold; thiếu/invalid confidence được xử lý như confidence=1 cho penalty, không cho malformed payload né FP. Oversized/unparseable protocol message làm terminal `AGENT_PROTOCOL_ERROR`, score=0 và log violation. Quá 5 slots → reject, log protocol violation; không cho thêm cơ hội. `finish(no_more_targets)` không được tính thành một localization report hoặc negative GT proof.

## 4. Scenario schema và authoring

[JSON Schema](contracts/scenario.schema.json) và [example](contracts/scenario.example.json) mô tả **trusted authoring package** gồm public run profile và private scene/GT/interventions. `design_only=true` cấm dùng làm benchmark result; example paths/asset IDs là logical identifiers chưa có binary. Schema structural không tự chứng minh reachability, observability, sensor realism, license, split disjointness hoặc clock behavior.

| Phần | Trường bắt buộc / validation ngữ nghĩa |
| --- | --- |
| Identity | Schema/protocol version; opaque ID; scene family, source-site ID, split; content hashes khi freeze; sinh episode không dựa vào model outcome |
| Robot/sensors | Body profile, actuation model, extrinsics, sensor response profile; source calibration session; explicitly `ideal`/`synthetic`/`empirical` |
| Scene | Metric scale, frame, traversability region, starts, collision mesh/material variants, lighting state; geometry/navmesh hash private |
| Victims/GT | K, asset identity, pose, canonical torso anchor, masks/visibility/accessible viewpoints và distance margins; không phát agent |
| Conditions | Pair group, P/N severity, random streams và mechanism parameters, event timing, rendered vs injected vs empirical provenance |
| Reproducibility | Engine/build/plugin/package hashes, source/weights/container IDs, seed streams, rendering settings, dataset/license manifest |

Generation acceptance chạy **trước policy evaluation**: initial penetration không có; profile footprint đi được từ start tới ít nhất một viewpoint/victim theo inflated collision geometry; viewpoint nhìn ≥25 pixels trong ≥2 frames và không cần đi xuyên victim; K=0 không vô tình spawn victim; mọi GT anchor/mask khớp actual spawned actor sau settle. Dùng frozen grid of candidate viewpoints cho admissibility, không lấy trajectory của baseline tốt để chọn scene. Giữ rejected cases + lý do để phát hiện sampling bias.

## 5. Difficulty và interventions

Không gộp mọi yếu tố thành một scalar “hard”. Dataset card ghi các trục liên tục và bins sau dev pilot; so sánh theo factor và interaction.

| Trục | Mức đề xuất | Cách áp và giới hạn |
| --- | --- | --- |
| Search geometry | Viewpoint geodesic 3–10 / 10–20 / >20 m; corridor width/footprint, clutter density, dead ends | Private GT measurements, không gửi agent; map size không đồng nghĩa difficulty |
| Victim appearance | Standing/seated/prone; lighting contrast; visible fraction >0,6 / 0,25–0,6 / 0,05–0,25 | Chạy segmentation offline để phân tầng; target phải đạt admissibility |
| RGB | Nominal / low-light / glare, exposure và motion blur | Rendered intervention có settings/hash; post-render noise phải ghi synthetic. Không coi RGB fog là IR/LiDAR scattering |
| Depth | Ideal; synthetic Gaussian; bias+holes theo distance/edge/material, temporal dropout | Gaussian σ(z) và hole probability fit hoặc default công khai; chưa fit thì `synthetic`, không gọi Astra calibrated |
| LiDAR | Noiseless; range bias/noise; missing sectors/dropout; latency | Separate ablation; noise không mặc định mô phỏng smoke/intensity. Per-ray clock + no-return semantics phải giữ |
| Self-localization N | Ideal reference; drift 1% / 3% distance, yaw bias 0,5° / 2° per m | Synthetic default, pre-integrated noisy encoder/IMU increments; không lấy path-dependent GT target để inject error |
| Actuation extension | Command lag, saturation, slip/friction zones | Khác N; cần physics/model validation. Không nhập vào core RQ1 rồi nhận là cùng measurement intervention |
| Integrity/timing | Stale/freeze/reorder, disconnect, engine reset | Fault suite tách scientific sensor stress với integration failures; keep capture/delivery time |

Core RQ1 `P=0` nominal RGB + empirical nominal depth nếu có; `P=1` frozen low-light setting + mild depth profile; `N=0` zero-drift estimate, `N=1` mild drift. “Clean” nghĩa nominal profile, không luôn là perfect sensor. Ideal camera/pose là diagnostics riêng. Severity 2 giữ cho generalization stress test. Trước empirical calibration, tất cả P levels là synthetic và RQ3 không được nhận real validation.

Paired clean/P/N/joint giữ **scene, victim, start, algorithm seed**; stream P và N độc lập. RNG phân theo `(pair_id,stream_id,sensor_id,sim_tick,ray_or_pixel)` để không phụ thuộc số lần gọi inference. Policy divergent paths có thể đổi exposure: đây là closed-loop **total effect** của intervention; không gọi same random seed là same observed images. Recorded fixed-trajectory replay là secondary test để tách sensor-only effects, không thay main closed-loop outcomes.

## 6. Splits và chống leakage

Plan 48 families =24 train/8 validation/16 test; 10 base missions/family, 2 absent và 8 positive, positive K được balance giữa 1/2/3. Mỗi base có 3 algorithm/fault replicate seeds ×9 P/N cells. Corpus chưa tạo; số thực chỉ được ghi sau manifest freeze. Phân bổ scene types/pose/range theo strata trước sampling; không cân bằng theo performance.

Family grouping theo **source site/building/layout lineage**; cùng geometry reskin, crop, procedural seed descendants, mirrored floorplan hoặc cùng scan không qua splits. Victim identities/clothing texture sets cũng group disjoint cho OOD appearance; generic wall/floor materials có thể shared nhưng phải công bố. `source_site_id` và duplicate/geometry hash audit quyết định group, không chỉ dựa vào string IDs tự đặt.

Train có nominal + mild single faults; validation có combinations đủ chọn threshold; test gồm unseen families/identities và held-out severity/joint profile parameters. Báo riêng **T-layout** (unseen family, điều kiện đã thấy) và **T-compound** (unseen family + severity/composition); không gộp rồi gọi một loại OOD. Mỗi family phải có đủ cặp condition để estimation không nhầm scene type với corruption.

Test server giữ GT, generator seeds và private actor queries. Agent chỉ đọc public observations trên separate process/container user/network; không mount scene manifests/GT/logs, không direct UnrealCV/Project AirSim privileged endpoint. Test output chỉ aggregate sau freeze; log số submissions, weights/prompt/version. LLM hosted model version/cost/latency/retention phải công bố; nếu provider thay đổi không pin được thì reproducibility limitation. Không dùng test để tune prompts hoặc thresholds, kể cả xem từng fail video.

## 7. Independent evaluator và metrics

Sau episode, evaluator dựng graph reports–victims. Candidate edge khi evidence đúng cùng victim, XY error ≤**1,0 m**, |z error|≤0,75 m, confidence ≥τ, không vi phạm format/epoch/deadline. Ghép **maximum-cardinality, sau đó minimum total squared XY error**, ties theo report sequence rồi private stable victim order; cần lexicographic implementation thay Hungarian cost sai penalty khiến mất cardinality. Mỗi victim/report dùng tối đa một lần. Duplicate/eligible nhưng unmatched report =FP. K≤3 và R≤5 nên exhaustive reference matcher có thể dùng để kiểm implementation tối ưu. Không dùng GT để merge predictions trước chấm.

τ chọn một lần trên validation cho mỗi method theo cùng quy tắc: tối đa safe recall, ràng buộc FP/episode≤0,1 và absent false-alarm rate≤0,05; ties chọn τ cao hơn. Nếu không có τ hữu ích đạt constraint, ghi infeasible/abstaining cùng recall=0, không bỏ method. Freeze τ; test báo cả actual FP rate và CI. Đây là operating constraint ước lượng, không bảo đảm threshold có exact false alarm rate trên test.

Đặt `K_e` là số victim hợp lệ, `TP_e(t)` là số matched reports có timestamp≤t, `FP_e` là unmatched slots vượt τ, `safe_e` là không contact/out-of-bounds/tip-over tới terminal. Safety abort giữ prefix, không bị loại khỏi denominator.

| Metric | Định nghĩa / cách tránh diễn giải sai |
| --- | --- |
| **Primary: safe localization recall (SLR)** | Positive episode: `safe_e * TP_e(T)/K_e`. Macro average trong family rồi trung bình families; absent không có recall và không quy thành 1. Báo recall ungated + unsafe rate cạnh SLR để lộ trade-off |
| Localization precision / false reports | Micro `ΣTP/(ΣTP+ΣFP)`; no reports → precision `NA`, recall 0. FP/episode tính cả invalid reports vượt τ. Absent FAR = fraction K=0 có ≥1 report vượt τ. Báo duplicate rate và per-pose recall |
| Mission completeness | Với K>0: tất cả K matched, zero FP, safe. Với K=0: zero FP, safe và `finish` trước budget; báo riêng **absent specificity**, không tuyên bố coverage proof |
| **Time-integrated recall** | `(1/T)∫[0,T]TP_e(t)/K_e dt`; reports được timestamp credit, sau early finish giữ TP cuối tới T, unsafe episode bị gate về 0 ở safe variant. Timeout/miss ở cuối cho phần còn lại 0; không báo mean time chỉ trên successful runs |
| Time / path | First correct report và restricted time `min(t_first,T)`, miss=T; all-found time censored/report survival curve. Path từ GT base movement; stopped agent có path=0 nhưng recall không tăng |
| Localization errors | XY median/p90 trên TP, kèm matched count và recall; supplementary nearest-prediction/all-target miss-capped error. Threshold sensitivity 0,5/1/2 m, không chọn threshold đẹp nhất trên test |
| Detection | AP/recall trên fixed held-out image trajectories với visible masks/boxes, stratified pose/range/occlusion; online detection opportunities theo agent path chỉ diagnostic, không cùng distribution giữa methods |
| Confidence/uncertainty | Report confidence Brier và reliability diagram với label matched=1, unmatched=0 sau deterministic matching; không report → không được “calibration tốt”. XY covariance coverage ở 50/90/95%, NLL trên evidence-associated inliers kèm outlier rate; chỉ score covariance không đủ đánh giá missing victims |
| Safety | Any prohibited contact, victim contact, min clearance (nếu certified geometry), out-of-bounds, stuck duration, safe stop time. Floor contact bị loại theo allowlist cố định; không loại low obstacles bằng ngưỡng tùy tiện. Collision tick counts gộp thành events theo actor pair, cooldown 0,2 s |
| Robustness | Per-cell SLR, `Δ=Ycondition−Ynominal`, RQ1 interaction I, 3×3 table. Relative ratio chỉ khi nominal>0 và báo denominator; rank Kendall τ kèm bootstrap uncertainty, không ép thứ hạng khi CI overlap |
| Compute / cost | Sim duration, wall duration, RTF, observation/inference/action p50/p95, timeout/drop rate, GPU/RAM peak, hosted token/cost nếu có. Không lấy engine FPS hoặc mission sim seconds làm throughput |

Không làm một weighted “rescue score” che safety/false alarms. Không có SPL primary: nhiều targets, absent và report-from-distance không có một shortest path goal đơn. Single-victim safe-approach appendix có thể dùng SPL nếu shortest **feasible** path tới valid approach set được kiểm độc lập với footprint/dynamics; NavMesh humanoid không đủ để gọi optimal UGV path.

## 8. Baselines và công bằng

| ID | Cấu hình dự kiến | Vai trò / điều kiện |
| --- | --- | --- |
| B0 | Seeded random turns/short goals + common detector/depth localizer + same safety | Sanity/lower reference; không coi là baseline nghiên cứu đủ mạnh |
| B1 | Frontier information-gain/distance planner + common person detector + robust depth estimator, two-frame confirmation | Classical active search; track/report memory và dedup ước lượng, không GT |
| B2 | B1 + confidence/covariance-aware multi-view verification và revisit | Method probe RQ2; ablate từng module; cùng detector/safety/action limits |
| B3 | Core `SearchMission/Navigator/Costmap/Monitor` hiện có + new image detector/localizer/report adapter | Tận dụng S2 nhưng thay oracle target bằng estimated detections; current core giả định holonomic, chỉ chạy profile phù hợp |
| B4 | Semantic frontier/value-map navigation (VLFM-style) + cùng report protocol | Strong semantic baseline; implementation/checkpoint/license phải xác minh. Adaptation khác bản gốc cần ghi rõ |
| D-P/D-N/D-NAV | Visible GT perception / ideal self pose / privileged navigation, thay từng module | Diagnostic only, bảng riêng; oracle perception chỉ khi target thấy được theo visibility mask, không cho target sau tường |

B1/B2/B3 share detector weights, inputs, common depth localizer trước verification, robot/safety và inference resource budget. Fixed policy steps không miễn phí cho VLM: **main deterministic scientific mode** đóng băng simulation khi agent tính, có wall watchdog **30 s/action** và **30 min/episode**; báo latency như limitation. **Deployment timing mode** chạy world liên tục hoặc delay replay từ measured inference distribution, commands có TTL, safety worker độc lập; budget 300 simulated seconds và wall hardware context. Timing modes không chung leaderboard; dynamic-scene lockstep phải được nghiệm thu trước. [RTNav](https://arxiv.org/abs/2608.26496) là tiền lệ trực tiếp cho rủi ro inference latency.

Tất cả versions/weights/prompts, hyperparameter budget và training compute ghi manifest. Không dùng trợ giúp GT proximity triggers của RescueBench trong main; không cấp segmentation như input mặc định upstream. Nếu VLM dùng input/compute khác, báo separate resource tier và ablation, không nhận search policy là nguyên nhân duy nhất.

## 9. Statistical analysis

**Đơn vị độc lập chính là scene family**, không phải frame, victim, corruption cell hoặc seed. Average seeds và base missions theo strata trong mỗi family trước primary test; giữ paired method/condition vectors nguyên vẹn. Primary effect cho RQ1 là mean family interaction I, RQ2 là paired B2−B1 joint-cell SLR, RQ3 là difference-in-differences giữa sensor models. Báo estimate bằng percentage points +95% CI.

Cluster bootstrap **10.000 draws** resample families, giữ nguyên methods/conditions/seeds trong cluster. Sensitivity hierarchical bootstrap family→base mission, seed average; không resample methods độc lập làm mất pair. Paired sign-flip/randomization test trên family differences chỉ khi exchangeability/symmetry assumption hợp lý; 16 families có thể enumerate 2^16 signs. Nếu assumptions không hợp lý, dùng cluster CI/descriptive và ghi rõ giới hạn. Binary mission completeness McNemar chỉ là exploratory khi bỏ qua clustering; không dùng như primary p-value.

Ba primary RQ contrasts dùng **Holm correction**, familywise α=0,05; secondary strata/ablations exploratory, công bố toàn bộ comparisons. Interaction fixed scale đăng ký trước; optional mixed-effects model (family/base random intercept) là sensitivity, không tìm model cho p nhỏ nhất. Report prevalence và family weights; không gộp absent vào positive recall hoặc weight scene nhiều targets quá mức.

Power analysis trên dev pilot bằng resampling **whole families** và simulation effect sizes 2/5/10 pp với observed ICC/paired variance. Chọn số family để đạt power mục tiêu≥0,8 với Holm-adjusted tests; 16 test families là budget ban đầu, chưa được chứng minh đủ. Freeze sample size trước test; không optional stopping theo significance. Nếu compute hạn chế, hạ số RQ confirmatory hoặc tăng CI uncertainty công khai.

Mọi scheduled run thuộc intention-to-evaluate. Agent exception/OOM/invalid actions giữ outcome 0 với failure reason; intentional faults vẫn thuộc denominator. Verified engine/transport infrastructure crash có `INFRA_ERROR` và raw evidence: retry nguyên pair block với cùng input, lưu cả attempts; công bố completion/error rate và worst-case bounds (missing=0 vs1). Nếu missing block không tái chạy được, primary table ghi incomplete, không âm thầm complete-case. Không chọn attempt tốt nhất. Broken reset/GT integrity → quarantine block, ngừng claim scientific result cho block đó.

## 10. Failure attribution và reproducibility

Attribution ban đầu là multi-label timeline: `NOT_EXPOSED`, `VISIBLE_MISSED`, `DEPTH_INVALID`, `POSE_DRIFT`, `ASSOCIATION_DUPLICATE`, `FALSE_REPORT`, `LOCALIZATION_ERROR`, `PLANNER_STUCK`, `MONITOR_BLOCKED`, `CONTACT`, `TIMEOUT`, `AGENT_ERROR`, `INFRA_ERROR`. “Not exposed” khác detector miss: evaluator có mask/time/range, không chỉ biết vị trí target. Cần visibility opportunities trước khi gọi lỗi perception; không có opportunity do route → search/navigation candidate.

Không gán causal blame chỉ từ last state. Re-run cùng block bằng interventions thay **một module**: visible GT detections giữ estimated pose; ideal self pose giữ noisy images; privileged waypoints giữ perception; oracle depth giữ RGB. Kiểm 2×2 oracle perception/pose khi cần interaction. Counterfactual policies có đường đi khác, nên kết luận là module substitution effect của pipeline này, không causal effect duy nhất của một subsystem. Fixed-trajectory replay bổ trợ để tách lỗi đo. Human audit mù method trên một sample cố định, hai người adjudicate và ghi disagreement; không chỉ chọn video đẹp.

Thêm negative-control chạy **RGB/depth bị che hoàn toàn**, và label-shuffled/incorrect evidence reports trong evaluator tests để phát hiện reliance on private metadata hoặc task priors. Source-grounding sanity có tiền lệ ở [VisualNeedle](https://arxiv.org/abs/2605.26380v2) và [JRDB-AVR](https://arxiv.org/html/2609.35032v2); không coi thiết kế evidence report tự thân là mới. Sensor-black control không kỳ vọng score tuyệt đối 0 nếu odometry/LiDAR và prior đã đủ đoán, nhưng mọi credited report vẫn phải vượt private evidence gate; divergence cần audit.

Mỗi run directory mới ghi: resolved private/public config hashes, engine/plugin source commit + packaged SHA256, RHI/GPU driver/OS, sensor calibration ID, model hashes, seeds theo streams, initial settled state hash, reset epoch, full desired/checked/applied commands, delivered sensor packets + binary lossless RGB/depth/scan, GT/events ở riêng evaluator, evaluator version và result. Raw observations có thể rất lớn: pilot đo storage; scientific test phải giữ đủ payload để offline re-evaluation, không chỉ camera hash. Public release có thể cung cấp subset có quyền và byte manifest; không hứa full reproducibility nếu thiếu binary/assets.

Resume theo immutable run contract, kernel-owned lock và append journal; attempts không ghi đè. Journal hash phát hiện hỏng file, không tự là security boundary. Test ground truth không nằm trong agent namespace; source/scripts/containers release kèm notice và documented license, secrets bị loại. Publish train/dev và test server recipe sau embargo phù hợp. Chạy repeat load/reset/reload ≥10 lần một action trace: clock/epoch chính xác; pose/scan/RGB tolerance đo trên pinned machine; cross-machine comparisons dùng predeclared numeric tolerances, không hứa bitwise GPU/Chaos determinism.

Acceptance evaluator trước science: 0/1/3 victims, absent FP, two reports/one victim, crossed assignment, equal costs, threshold equality, z mismatch, unsupported frame, fake/old/future evidence, same buffered frame hai lần, deadline edge, invalid covariance, safety abort, held/frozen buffers, reset mid-flight, interrupted attempt và privileged field leakage. Reference matcher và metric recomputation phải agree; runtime UE5 và các tests này hiện **UNVERIFIED**.
