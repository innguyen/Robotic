# S3 audit — Research Blueprint & UE5 Foundation

**Bàn giao ngày 2026-11-25, Asia/Saigon**, `innguyen/Robotic`, branch `feat/benchmark`; HEAD nền `671add0137a0ef2f06081bde402e7556e083b15d`.

**Hoàn tất S3 theo nhánh được yêu cầu khi chưa có hạ tầng UE5 phù hợp:** đã khảo sát, thiết kế research protocol/architecture, chọn framework candidate, tạo scenario contract và preflight/QA có thể chạy lại. **UE5 vertical slice, kết quả RQ, novelty và sim-to-real đều UNVERIFIED.** Không có benchmark scores hoặc ảnh UE được tạo để làm bằng chứng giả.

## 1. Quy ước bằng chứng

| Nhãn | Ý nghĩa |
| --- | --- |
| `OBSERVED_LOCAL` | Thực sự đọc/chạy trong S3, có output/file tương ứng |
| `REPORTED_PRIOR` | Đọc audit/raw evidence của Webots/Godot cũ, không chạy lại simulation trong S3 |
| `DOCUMENTED_UPSTREAM` | Paper/docs/source upstream được đọc online; không tương đương đã chạy trong repository |
| `DESIGN` | Quyết định/protocol/threshold/recipe đề xuất, chưa experimental validation |
| `UNVERIFIED` | Chưa có đủ evidence, không suy PASS từ code, specs hay screenshot của bên khác |

## 2. Hoàn thành theo yêu cầu

| Yêu cầu | Deliverable / kết quả |
| --- | --- |
| Đọc Webots S0–S2, Godot S0, kiến trúc và T1 | [Reuse map](architecture.md); đọc audits/READMEs, adapter/runtime/core/evaluator/journal/paired/schema interfaces và production sensor config. S2 oracle/ideal omni, async repeat FAIL, Godot logical-only stepping đều được giữ rõ |
| Khảo sát bốn hệ thống và related works | [Literature matrix](literature-gap-matrix.md), hơn mười đối chứng trực tiếp; tách paper version, repo version và status preprint; chỉ dùng primary sources |
| Gap và 2–3 research questions | [Proposal](benchmark-proposal.md): compound perception×odom effects, active verification, sensor-model dependence; có null outcomes, minimum useful effect và novelty withdrawal conditions |
| Protocol đầy đủ | [Protocol](evaluation-protocol.md): task/API, evidence reports, independent GT matching, absent/duplicate/false alarms, splits, baselines, cluster statistics, attribution, reproducibility và compute budget |
| Photorealism/sensors/robot | [Platform decision](ue5-platform-decision.md): visual/sensor/physical realism tách riêng; RGB-D/LiDAR response, victim assets, ideal omni vs physical Mecanum |
| UnrealZoo/RescueBench vs Project AirSim | Candidate Project AirSim v1.1.0 + UE5.8; current UnrealZoo v3.1/UE5.7 có LiDAR được công nhận; clock, licenses, source-vs-cooked và compatibility gates ghi riêng |
| UE5 chính + Webots nhẹ | [Architecture](architecture.md): common task boundary, backend-specific physics/sensor manifests; không pool scores hoặc coi hai simulator tương đương |
| Vertical slice nếu hạ tầng cho phép | Preflight không tìm thấy engine trong phạm vi kiểm; chưa có verified graphics worker. G0–G6 recipe và acceptance evidence đã thiết kế, runtime **chưa chạy** |
| JetAuto/Astra thực tế | [Proposal §6](benchmark-proposal.md): stationary calibration → offline replay → guarded low-speed pilot, consent/asset/data/site permissions và giới hạn claim |
| Không phá các mốc cũ/T1 | Working tree ban đầu sạch; snapshot SHA256 **5.196 tracked files** trước thay đổi. Mọi file thêm mới nằm trong `docs/research/`; xác minh byte identities bằng QA |

Đây là một lần bàn giao S3. Các gates tiếp theo là acceptance criteria để thi công tự chủ khi được cấp worker, không phải chuỗi xin xác nhận từng milestone. Không có thay đổi external services, tải asset lớn, cài framework, commit, push hoặc deploy.

## 3. Hạ tầng thực tế

Chạy [preflight script](tools/preflight_ue5.py), output tại [preflight.json](evidence/preflight.json), lúc **19:02:48 ngày 25-11-2026** theo Asia/Saigon:

| Probe | Kết quả / phạm vi |
| --- | --- |
| OS / Python | `OBSERVED_LOCAL`: Windows 11 build 26300; Python 3.12.11, `C:/msys64/ucrt64/bin/python.exe` |
| RAM | `OBSERVED_LOCAL` qua GlobalMemoryStatusEx: **8.215.101.440 bytes (~7,65 GiB)** physical; available **952.971.264 bytes (~909 MiB)** tại snapshot. Không phải UE benchmark hoặc soak test |
| UE PATH/env | `UnrealEditor`, `UnrealEditor-Cmd` không tìm thấy trên PATH; `UE_ROOT`, `UE_ENGINE_ROOT`, `UNREAL_ENGINE` không được đặt |
| Common install paths | Không tìm thấy `C:/Program Files/Epic Games`, `C:/Epic Games`, `C:/UnrealEngine`, `D:/Epic Games`, LauncherInstalled.dat; không quét mọi ổ đĩa. Có thể có custom engine path chưa được biết |
| Repo UE project | `git ls-files -- '*.uproject'` không có output; đây là tracked-only probe, không khẳng định toàn máy không có UE project |
| GPU live | CIM query trả **Access denied**; `nvidia-smi` không có trên PATH. Không suy ra máy không có GPU hoặc chắc chắn không render được |
| GPU historical | `REPORTED_PRIOR`: [Godot rendered stdout](../../experiments/godot_s0/evidence/run-05-final-rendered/stdout.txt) ghi Intel Iris Xe/driver 31.0.101.5186; invocation cùng ngày có RAM khớp. Không thay nó bằng “GPU measured S3” |
| Graphics/RHI/sensor runtime | **UNVERIFIED_NOT_RUN**; không có DX12/Vulkan test, engine launch, RGB/depth readback, actual UGV/control/reset/evaluator UE run |

Các truy vấn CIM CPU/RAM/GPU đầu phiên cũng bị access denied; RAM sau đó lấy bằng API read-only khác. Không yêu cầu elevation hoặc sửa driver/MIG để tiếp tục công việc tài liệu. Không có auto-review rejection cho một action đã yêu cầu escalation; đây là lỗi quyền đọc API trong sandbox, khác quy trình approval review.

Kết luận giới hạn: **chưa có engine + graphics infrastructure được xác minh**, đủ để chọn nhánh blueprint/UNVERIFIED được user cho phép; không nói UE5 tuyệt đối không chạy trên Iris Xe. Hardware khuyến nghị là nguồn/cấu hình dự kiến, không số đo tốc độ của máy này.

## 4. Research/source audit

Online primary-source inspection thực hiện trong S3, gồm paper full-text HTML của RescueBench/ESARBench/SimWorld-Robotics/NavTrust; conference/source pages của UnrealZoo/RobustNav; SubT/NIST/IEEE official pages; recent ObjectNav/SAR preprints; Project AirSim release/vehicle/clock/sensor docs và actual tagged example source. Search strings, selection criteria và giới hạn nằm trong matrix.

[Source inspection ledger](evidence/upstream-inspection.json) ghi URL và Git blob SHA của năm file quan trọng. Không tải/vendoring full code hoặc environment packages. Repository branches/docs có thể thay đổi; tag/source tuple và license asset còn phải freeze trước runtime. Các URL thất bại lúc dò đường dẫn đã được thay bằng canonical sources; không lấy 404 làm bằng chứng feature không tồn tại.

Các sửa nhận định quan trọng từ evidence: UnrealZoo v3.1 **có LiDAR**; Project AirSim release mới **có native wheeled UGV**; engine-driven clock **không mặc định pause/step** như steppable drone clock; RescueBench có stage diagnostics và GT-assisted interaction path cần audit; Epic có thay đổi AI/MetaHuman EULA ngày 29-10-2026. Không giữ assumption từ phiên bản cũ để làm lựa chọn có vẻ vượt trội.

## 5. Kiểm tra đã thực hiện

`OBSERVED_LOCAL`:

- `Test-Json` xác nhận scenario example hợp JSON Schema; [9/9 schema cases PASS](evidence/schema-validation.json), gồm unknown fields, GT field trong public profile, quá 3 victim, version sai, unsafe ID, empirical thiếu calibration, runtime chưa freeze và severity ngoài range. Đây là **schema checks, không simulation tests**.
- Python artifact verifier đã **PASS**: kiểm semantics/strict JSON của contracts, local Markdown links, code fences, compile Python tool source và SHA256 **5.196 tracked files** nền. [Lượt QA đầu](evidence/verification.json) kiểm 7 Markdown documents, 50 local links, 2 contract JSON files và không có lỗi. Sau bổ sung related work/rá soát, lượt cuối lưu `evidence/verification-final.json`; output phân biệt artifact QA với runtime acceptance.
- `git diff --check` cho tracked diff và kiểm whitespace riêng file mới; rà soát final status để chỉ có `docs/research/` được thêm. Không commit/push.

Không chạy lại Webots/Godot runtime, không claim các test suite của S1/S2/T1/production đã PASS lại trong S3. Vì không sửa các phần đó, kiểm byte identities phù hợp hơn việc launch simulators đang dùng GPU/RAM hạn chế. Kết quả PASS lịch sử chỉ dẫn từ audit tương ứng. Không có test perception YOLO, UE5 hoặc robot thật được ngụy trang thành kiểm syntax/schema.

Lệnh tái kiểm (dùng output path **mới**, không ghi đè evidence):

```powershell
& C:/msys64/ucrt64/bin/python.exe docs/research/tools/preflight_ue5.py --out docs/research/evidence/preflight-new.json
& ./docs/research/tools/check_scenario_schema.ps1 -OutFile docs/research/evidence/schema-new.json
& C:/msys64/ucrt64/bin/python.exe docs/research/tools/verify_s3.py --out docs/research/evidence/verification-new.json
git diff --check
```

Artifact verifier không là full JSON Schema validator, full Markdown renderer hoặc simulator checker; structural schema check là lệnh PowerShell riêng. Schema chỉ kiểm authoring package. Multi-target matching, covariance scoring, full sensor runtime schema, split generator và physics admissibility vẫn chưa có implementation.

## 6. Outstanding / UNVERIFIED và acceptance evidence cần có

| Nội dung | Tình trạng | Điều kiện để đổi trạng thái |
| --- | --- | --- |
| UE5 actual vertical slice | **UNVERIFIED** | G0–G4 ở platform decision, có scene/robot/RGB/depth/Python/reset/evaluator raw evidence và package/source identities |
| Main omni UGV + image perception | **UNVERIFIED** | G5 axis/contact/safety/reset + detector/depth localizer thật; không lấy native SUV example thay thế |
| Astra/G4 sensor realism | **UNVERIFIED** | Real calibration và held-out error/missingness/timing curves; synthetic Gaussian/holes chưa đủ |
| Physical JetAuto Mecanum | **UNVERIFIED** | Wheel/roller dynamics/encoder/braking/slip validation và measured GT uncertainty |
| Novel evaluation / gap exclusivity | **UNVERIFIED** | Rà forward/backward citations + source implementation của nearest methods, giữ claim hẹp; không đảm bảo acceptance top-tier |
| RQ1–3 effect, ranking, statistical power | **UNVERIFIED** | Dev pilot power, frozen corpus/evaluator/baselines, blind test có cluster CI và đủ paired blocks |
| Corpus 48 families / safe sensor profiles | **DESIGN** | Actual assets có license/source lineage, scene QC, observable/reachable GT và duplicate/split audit |
| Reproducibility cross-machine | **UNVERIFIED** | Pinned tuple + reset/trace repeat và independent graphics worker; không hứa bitwise GPU/Chaos |
| Real robot / ethics / asset release rights | **UNVERIFIED** | Site/operator/ethics consent nếu cần, accepted applicable licenses và guarded trials; S3 chưa thực hiện external action nào |

Rủi ro chính là chi phí clock/omni integration và thiếu sensor calibration, không phải số lượng cảnh. Phương án dự phòng giữ nguyên research contract, dùng UnrealZoo/UnrealCV candidate qua cùng gates hoặc tiếp tục Webots diagnostics có nhãn. Không hạ chuẩn dữ liệu/GT để tạo video sớm. Thay đổi S3 cô lập trong thư mục mới; rollback không cần sửa bất kỳ baseline nào.
