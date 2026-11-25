# Đề xuất benchmark UGV Victim Search under Degraded Sensing

**Trạng thái: DESIGN, `sar-s3-draft-0.1`, 2026-11-25.** Tên làm việc, không phải benchmark đã phát hành. Kết luận S3: đầu tư vào phép đo và khả năng phân biệt nguyên nhân lỗi trước khi mở rộng scene. UE5 là backend chính đề xuất; Webots S2 là nền đối chứng nhẹ được bảo toàn.

## 1. Đóng góp có thể bảo vệ

Mục tiêu là kiểm tra **khả năng tìm, xác nhận và báo vị trí nạn nhân đáng tin cậy khi cảm biến và self-localization đồng thời suy giảm**. Robot chỉ tìm/báo; không đánh giá extraction, manipulation, return/handoff, chẩn đoán y khoa hoặc xác định sống/chết. Victim là người tĩnh theo định nghĩa nhiệm vụ, có bằng chứng ảnh nhìn được một phần từ vùng UGV đi được. Không coi hoàn toàn chôn vùi là RGB-detectable.

[Literature matrix](literature-gap-matrix.md) chỉ ra các thành phần đã có tiền lệ: SAR đa giai đoạn, weather, coordinate reports, robustness, recovery và latency. Đóng góp đề xuất là tập điều kiện đánh giá và bằng chứng thực nghiệm về **tương tác lỗi**, false reports và localization confidence; chưa khẳng định đầu tiên. Thành công không phụ thuộc một thuật toán của nhóm thắng tất cả baselines. Một kết quả âm có thể hữu ích nếu chỉ ra giả định ideal depth/ideal pose làm đảo ranking có kiểm chứng và giới hạn rõ ràng.

Ba kết quả tối thiểu trước paper: evaluator có regression suite chống false positives/GT leak; corpus split theo scene provenance có stress pairs; ít nhất một kết luận RQ có CI và failure evidence, được lặp ở một máy khác. Không gọi bộ ảnh đẹp hoặc video scripted là benchmark nghiên cứu.

## 2. Research questions và phép bác bỏ

Quy tắc chung: đóng băng threshold, baselines và hypotheses trên dev/validation trước khi mở test. Hiệu ứng có ý nghĩa thực tiễn được đề xuất là **5 percentage points** safe localization recall; mức này cần power analysis, không phải đảm bảo phát hiện được.

| RQ | Giả thuyết / biến được can thiệp | Thí nghiệm và endpoint đăng ký trước | Khi nào không được kết luận như kỳ vọng |
| --- | --- | --- | --- |
| **RQ1 — Compound degradation**: suy giảm cảm biến và odometry có tạo lỗi báo vị trí vượt tổng hai ảnh hưởng riêng? | P: RGB-D perception stress; N: self-localization drift, độc lập với ground-truth motion. Đo `I=Y11-Y10-Y01+Y00`, Y là safe localization recall | Factorial 2×2 core trên cùng scene/start/victim/seed; mở rộng 3×3 severity. Primary: I và 95% cluster CI; secondary: ranking, FP, localization error | CI chứa 0 hoặc effect <5 pp: không nhận có super-additive loss. Nếu chỉ vài scene chi phối, báo heterogeneity. Không gọi additive-scale interaction là nguyên nhân sinh học hay kết luận phổ quát |
| **RQ2 — Active verification**: chủ động đổi viewpoint/xác nhận nhiều góc nhìn có tăng số report đáng tin cậy dưới cùng budget? | Baseline B1 đối với B2 chỉ khác verification; pose, detector, planner, model weights và thời gian chung | Primary: paired Δsafe recall ở joint stress với threshold chọn trên validation theo false-report constraint. Secondary: FP/episode, time-integrated recall, reliability curve. Ablate multi-view, covariance gate, revisit riêng | Nếu chỉ giảm FP bằng cách không report, recall/time sẽ lộ; nếu lợi ích mất khi cùng compute/budget, không nhận cải thiện hiệu quả search. Nếu FP test vượt constraint, không dùng cụm “tại mức false alarm cố định” |
| **RQ3 — Sensor-model dependence**: ideal/Gaussian depth có dự báo đúng thứ hạng khi dùng mô hình Astra fit từ dữ liệu? | Giữ scene/robot và seed, thay depth ideal → Gaussian có matched RMSE → structured invalidity/bias fit từ Astra. LiDAR faults được giữ cố định hoặc ablate riêng | Primary: thay đổi paired B2−B1 safe recall; secondary: rank correlation kèm uncertainty, error-vs-range/edge, calibration. Validation nhỏ từ Astra thật, tách ngày/session | Chưa có real calibration thì chỉ gọi synthetic stress sensitivity. Không có rank reversal cũng là kết quả. Nếu real chỉ có mannequin/phòng sạch, không suy rộng sang sập đổ/khói hoặc field deployment |

Tại RQ1, `P` là can thiệp đã định nghĩa trước, không trộn tất cả nguyên nhân thành một nhãn khó. Main factorial dùng low-light + depth holes theo profile xác định; chạy lại từng thành phần để attribution. `N` là measurement drift; friction/slip là suite cơ học phụ để tránh lẫn sai perception với thay đổi transition dynamics. Với Y càng cao càng tốt, `I<0` là mất hiệu năng lớn hơn cộng trên **thang chênh lệch**; báo cả bốn cell, tránh chỉ báo I.

## 3. Phạm vi corpus và nhiệm vụ

Một UGV có camera RGB-D trước, LiDAR phẳng, IMU và wheel odometry. Agent nhận mục tiêu lớp `victim_person`, vùng hoạt động suy ra qua khám phá; không nhận số victim, coordinates, navmesh, map đầy đủ, masks hay oracle recognition. Victim count 0–3; 20% base episodes absent; không chứa non-victim pedestrians ở v0 để tránh nhãn “người nào cần cứu” không quan sát được. Distractors gồm poster/TV/mannequin không phải victim chỉ khi definition/appearance hợp lý và được annotator độc lập kiểm; không tạo cạm bẫy nhãn tùy tiện.

Ba môi trường authoring ưu tiên: phòng/kho công nghiệp, hành lang có vách ngăn/rubble nhẹ, courtyard có bóng gắt và clutter. Giữ mặt phẳng đi được cho omni proxy trong main track; slope/rubble có traction là extension có dynamics riêng. Không dựng cả thành phố hoặc nhiều tầng trước khi xác nhận evaluator và RQ có độ nhạy.

Phải có victim đứng/ngồi/nằm, partial occlusion, khoảng nhìn thấp, các kích thước apparent khác nhau; có inaccessible target nhưng có viewpoint hợp lệ. Phân loại “khó thấy nhưng nhìn được” với “bất khả quan sát”; trường hợp bất khả quan sát chỉ vào challenge appendix, không nằm trong mẫu số main recall.

Corpus mục tiêu là **48 scene families**, train/validation/test = **24/8/16**. Một family bao gồm toàn bộ biến thể từ cùng site/building/source layout, kể cả reskin/procedural siblings. Mỗi family có 10 base missions cho thiết kế benchmark; test có 160 base missions ×3 run seeds ×9 P/N conditions = **4.320 episodes/method**. Đây là ngân sách dự kiến, chưa có 48 scene. Power phụ thuộc 16 independent test families, không phải 4.320 samples độc lập.

Năm methods ×4.320×300 s có upper bound **1.800 simulation-hours**, chưa gồm training, diagnostic interventions và hardware overhead. Cần đo RTF của scene/sensors thật trước khi lập lịch GPU; không hứa nhanh hơn real time. Có thể dùng fractional factorial phụ cho factor screening trên dev; confirmatory 2×2 phải giữ đủ ghép cặp trên test. Không giảm số cell sau khi nhìn kết quả.

## 4. Baseline và method contribution

Main comparison: seeded random exploration (sanity), frontier+RGB detector, cùng frontier+active verification, current core SearchMission/Navigator/Monitor với detector ảnh thật, và semantic frontier/VLFM-style baseline. Tất cả cần một embodiment/action adapter đã nghiệm thu chung. Baseline thứ năm được gọi “style/adaptation” khi chưa chạy implementation gốc; không mượn tên published model để gắn kết quả tự viết. Cần thêm uncertainty-free heuristic tốt để không chỉ đánh bại baseline yếu.

Nguồn triển khai semantic baseline đã xác định là [VLFM official repository](https://github.com/rai-opensource/vlfm/blob/main/README.md); dependency/model stack của bản gốc khác env UE5. Giữ môi trường tách biệt, không nâng/hạ production packages để ép chạy. Chưa cài/chạy hoặc xác minh mọi checkpoint license trong S3.

Tái sử dụng YOLOv5n ONNX có sẵn làm detector khởi điểm; phải hash checkpoint và xác minh source/training/license của **file thực**, không suy từ tên. Person detector COCO không mặc định giỏi người nằm/che khuất. Huấn luyện hoặc fine-tune chỉ trên train; threshold/calibration trên validation. Mọi method dùng cùng checkpoint khi nghiên cứu search/verification; khác detector là experiment riêng.

Oracle perception, oracle self pose và oracle navigation chỉ là **diagnostic arms**. Không trộn với bảng main. Policy không được gọi UE navmesh shortest-path API. Frozen full-map planner reference được label privileged; không gọi là guaranteed upper bound nếu planner có thể fail.

## 5. Ba loại realism và cách chứng minh

| Loại | Cần có | Không đủ để kết luận | Phép kiểm |
| --- | --- | --- | --- |
| **Visual realism** | PBR scale, lighting/exposure, shadows, believable clutter và victim pose/material | UE5/Lumen, screenshot đẹp, human preference đơn lẻ | Blind scene QC; match camera response/range/occlusion với real samples, report histogram và task strata; tránh target nổi bật bởi màu/lighting cố định |
| **Sensor realism** | Intrinsics/extrinsics, metric depth, invalid masks, bias, per-beam timing, latency/dropout, calibration nguồn gốc | Z-buffer + Gaussian noise hoặc RGB fog = physics của IR/LiDAR trong khói | Planes ở nhiều range/incidence/material; error và missingness curves; timing logs; held-out session validation |
| **Physical realism** | Footprint/contact, actuator limits, wheel geometry, slip, mass/inertia và braking nếu claim transfer | Render robot Mecanum, direct body velocity, Chaos chạy | Axis/turn/braking/contact tests trên ground phù hợp; so quỹ đạo real; main omni proxy luôn ghi ideal actuation |

Asset và camera/LiDAR implementation được đặc tả trong [platform decision](ue5-platform-decision.md). Những khác biệt giữa UE5 và Webots là một yếu tố của thí nghiệm, không là nhiễu có thể bỏ qua bằng cùng seed.

## 6. Validation JetAuto/Astra an toàn và có thể lặp

**Chưa thu hoặc chạy robot thật trong S3.** Dữ liệu này phục vụ kiểm sensor/behavior trong phòng có kiểm soát, không chứng nhận cứu hộ. Source hiện hành có Astra, LiDAR G4 và chassis Mecanum; các offset trong `obstacle_avoidance.py` là cấu hình phần mềm, phải đo lại trên thiết bị. Không tự kết nối IP/SSH, khởi động ROS hoặc gửi `cmd_vel` trong nghiên cứu tài liệu.

1. **Stationary capture trước:** wheels disabled/robot cố định an toàn; lưu RGB, raw depth/validity, `camera_info`, scan, transforms và timestamps ROS lẫn monotonic. Thước/calibration board và planes matte ở 0,6/1/2/3/4 m, các góc 0/30/60°, ít nhất 3 sessions/ngày/lighting nếu khả thi; 100 frames/station chỉ là repeated measurements. Đọc exact Astra model/firmware/USB mode, không thay calibration bằng FOV catalog. Astra là structured-light và được hãng định hướng indoor; không dùng fog/thermal claim cho output lý tưởng. [Orbbec specs](https://www.orbbec.com/products/structured-light-camera/astra-series/).
2. **Fit vs validation tách session:** fit depth bias-vs-z, robust variance, edge holes và temporal autocorrelation trên session A/B; giữ C để kiểm prediction intervals/error curves. LiDAR so walls/poles ở range an toàn; báo angular resolution, blind sectors, `Inf`/invalid và scan period. Bootstrap theo station/session, không theo từng pixel. Hạn chế extrapolation xa/ngoài trời.
3. **Perception replay:** RGB-D có mannequin và props nhẹ, các tư thế/occlusion giống strata; annotator độc lập ghi boxes/landmarks và uncertainty. Dataset train/calibration/test tách session, subject/asset identity; không dùng frame liền kề vào hai split. Chạy detector offline, không phát motion. So tỷ lệ miss/false alarm/localization chứ không chỉ visually similar.
4. **Motion validation nhỏ:** chỉ sau local safety check và người vận hành được phép; khu vực kín, nền phẳng khô, không cầu thang, không vật nặng rơi. Khởi đầu ở vận tốc giới hạn **≤0,1 m/s**, người cầm physical stop, watchdog độc lập, vùng cấm tiếp xúc; khoảng dừng được quyết định từ measured stopping distance + uncertainty, không dùng standoff mô phỏng làm bảo đảm an toàn. Chạy vx/vy/yaw rồi search mannequin; không đặt người nằm trước xe. Chọn 3 layouts ×3 conditions không nguy hiểm ×3 repeats =27 trials như **pilot**, không đủ tuyên bố tổng quát. Randomize order, ghi interventions, battery và failures.
5. **GT real và kết luận:** dùng markers/overhead camera hoặc đo mốc có uncertainty, không cấp markers target cho agent. So localization error với uncertainty của GT; chỉ claim transfer nếu khoảng tin cậy/tác vụ phù hợp. Báo gap giữa sim/real cùng pipeline, không gộp vào một leaderboard hoặc đổi threshold để che gap.

Ưu tiên mannequin/nhân vật tổng hợp. Nếu dùng tình nguyện viên: consent có mục đích, quyền rút lui và giới hạn công bố; đánh giá ethics/IRB theo trường trước khi thu; không thu nạn nhân thật, không suy tình trạng sức khỏe. Làm mờ người ngoài cuộc/biển số, giới hạn quyền truy cập, retention được phê duyệt; chỉ phát hành dữ liệu có quyền. Xin quyền địa điểm và đo đạc; tuân thủ hướng dẫn laser/IR của thiết bị, không tăng công suất/nhìn sát emitter hoặc tạo khói/bụi/lửa, ánh sáng chớp, mặt sàn trơn để “tái tạo disaster”.

## 7. Điều kiện để đầu tư authoring quy mô lớn

Một phòng nguồn có quyền rõ ràng phải tạo được bốn cặp clean/P/N/P+N; baseline ảnh thật có thể phát hiện và cũng có thể fail với nguyên nhân xác định; evaluator phân biệt duplicate/false/localization-error report; fault injection không làm rò GT. Sau đó mới freeze protocol và sinh corpus theo splits. Nếu toàn bộ methods floor/ceiling, điều chỉnh dev factors và version trước test. Nếu power không đủ, tăng **scene families** hoặc thu hẹp RQ; không bù bằng nhiều seed cùng phòng rồi gọi độc lập.

Các tài liệu còn lại là thiết kế có thể thi công; chưa có bằng chứng hỗ trợ hypothesis nào. Tính mới của kết luận, ranking, calibrated Astra model, physical Mecanum và transfer đều **UNVERIFIED**.
