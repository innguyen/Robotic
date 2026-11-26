# SAR benchmark — sar-s4-mvp-0.1

ROS-free contracts, policies, native UE integration và independent evaluator. Production ROS và benchmark T1 không bị sửa.

| Module | Trách nhiệm |
| --- | --- |
| contracts, scenario_contract, episode | Public/private allowlist, timestamps, epoch/reset, immutable report slots, blind ACK, lifecycle |
| safety, embodiment | Common clearance/freshness/acceleration gate; Mecanum kinematics/saturation |
| perception, policies, tracking | YOLOv5 ONNX, letterbox/NMS, RGB-D torso estimate, association, dedup, translated-view verification |
| core_policy, frontier_policy | Actual SearchMission/Navigator/Costmap reuse; classical frontiers; matched verification arm |
| ue_backend, capture, sensor_response, transport | Native PNG/EXR, private depth-tested visibility, synthetic sensors, bounded compressed RPC |
| evaluator, reference, audit_run | Multi-target matching, independent eligibility/metrics, inventory/chain/delivery hashes, adversarial replay |
| admissibility, calibration | Trusted native visibility/2D connectivity; depth planes, transforms, zero, reset repeatability |
| runner, artifacts, statistics, sensor_study | Source snapshots, paired experiments, family-level analysis, secondary fixed-trajectory sensitivity |
| asset_scenarios, visual_qa, runtime_metrics | Hai asset-based dev layouts; native/robot/depth/mask image provenance; private UE memory samples và measured engine-frame rate |

Positive SLR=safe*TP/K; absent recall=null. FP gồm malformed slots và duplicate reports vượt threshold; max 5 slots không phụ thuộc K. Matching XY≤1 m, |Δz|≤.75 m; hai delivered captures cùng epoch, cách ≥.1 s, mới nhất ≤2 s, mỗi frame ≥25 visible pixels và IoU≥.25. Dev evaluator threshold .5, detector candidate threshold .4; không chọn threshold trên test.

| Profile | Thực hiện | Giới hạn |
| --- | --- | --- |
| ue5_static_proxy_rgbd_scan_v1 | 80×60, 72 rays, red-component detector, .1 s transactions | Primitive diagnostic/regression |
| ue5_sar_human_rgbd_scan_v1 | Licensed stylized humans, lit rescue layouts, ONNX-person/RGB-D; configurable 160×120/320×240/640×480 và 360/720 rays | Static kinematic transactions; synthetic sensors; chưa là S3 main |

Action interval .1 hoặc .5 s được ghi vào observation/ledger và evaluator; không đổi nhãn 2 Hz thành 10 Hz. Public time là transaction clock; private records giữ software capture start/end/availability monotonic time. Chưa có hardware exposure timestamps hoặc global Chaos lockstep. Cấu hình được hỗ trợ chưa đồng nghĩa đã nghiệm thu runtime.

`--scene-set assets` chọn CC0 structural/prop meshes, texture PBR và MakeHuman anatomical poses; mặc định `legacy` để giữ fixtures. Runner freeze source và toàn bộ asset-input bytes trước khi UE import. Audit kiểm asset bytes, source/journal/inventory, stored metrics so với independent recomputation và số completed episodes so với schedule khi có ghi; scheduled count hoặc calibration FAIL không cấp PASS. `visual_qa` phân biệt initial/last delivered robot frames, matched report evidence và privileged calibration views; các ảnh QA không được dùng để tăng score.

Revision 2 dùng camera HDR sky và một cubemap cố định cho ambient light. Các snapshot ghi hash riêng của mesh người mặc trang phục cho ba tư thế. `episode_video RUN EPISODE --out NEW_DIRECTORY` xuất video theo đồng hồ tác vụ từ RGB đã giao cho agent và kiểm từng hash với ledger.

Agent chỉ nhận measurements và algorithm seed. Local worker cùng OS user có thể đọc filesystem/network nếu cố ý; chưa có hostile-agent sandbox. Base replicate seed hiện cũng dùng cho algorithm và fault streams với domain separation; đây không phải seed secrecy. Tất cả run scientific_main_eligible=false.

Xem [cách chạy UE](../experiments/ue5_s4/README.md), [asset/license manifest](../experiments/ue5_s4/assets/README.md) và [audit S4](../docs/research/s4-audit.md).
