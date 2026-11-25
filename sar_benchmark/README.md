# SAR benchmark foundation — `sar-s4-mvp-0.1`

ROS-free; evaluator/contract tests chỉ dùng Python standard library. Module engine và policy không được import bởi evaluator.

| Module | Trách nhiệm |
| --- | --- |
| `contracts`, `scenario_contract` | Strict JSON, fields allowlist, finite/types, covariance SPD, transforms/frames/timestamps, private authoring contract |
| `episode`, `safety` | Reset/epoch, report slots, blind ACK, command TTL, actuator caps/acceleration/freshness/clearance, terminal zero |
| `evaluator` | Visible evidence, multi-victim DP assignment tối đa cardinality rồi tối thiểu squared XY cost, FP/duplicate/absent, time/safety/localization/confidence metrics |
| `reference`, `audit_run` | Eligibility và exhaustive assignment viết độc lập, offline recomputation, artifact checks, real-trace adversarial replay |
| `policies`, `core_policy`, `perception` | Hai fixture baselines và adapter core Robotics; optional YOLOv5 ONNX detector |
| `ue_backend`, `transport`, `runner` | Native UE integration duy nhất, private response layer, public projection, process worker, reproducible experiments |
| `artifacts`, `statistics` | Source/config hashes, exclusive attempts, chain journal, lossless logs, family-weighted paired RQ1 contrast |

Primary metrics giữ quy ước S3: positive SLR=`safe*TP/K`, absent recall=`null`; FP tính cả malformed slots, duplicate/new IDs vẫn chiếm slot; max 5 reports không phụ thuộc K; match XY≤1 m, |Δz|≤.75 m. Evidence phải là hai delivered frames cùng epoch, capture khác nhau cách ≥.1 s, không tương lai, frame cuối không cũ hơn 2 s, mỗi frame ≥25 visible pixels/IoU≥.25. Agent chỉ nhận ACK đã nhận, không nhận correctness.

S4 profile tách khỏi contract thiết kế S3: ảnh 80×60, 72 ideal collision rays, synthetic GT-increment odometry, scene static, short pilot budget. S3 640×480/720-ray/IMU, calibrated sensors, secure main evaluation và physical Mecanum chưa được nghiệm thu. Không đổi S3 draft để làm như đã đáp ứng nó.

Hướng dẫn chạy và raw evidence được dẫn từ [UE slice](../experiments/ue5_s4/README.md) và [audit](../docs/research/s4-audit.md).
