# Nghiệm thu P2.2 — Metrics, paired runs và dev pilot

Nền commit `72d5d44`, nhánh `feat/benchmark`, protocol **t1-0.2**. Phạm vi là kiểm tra pipeline metrics/paired identity và dev pilot trên Windows. Không mở test nghiên cứu, không chạy ROS/SSH/camera/robot, không đổi YAML hoặc source production.

**P2.2 chưa đạt nghiệm thu tổng thể.** Implementation, metrics, paired identity, CSV và monitor checks PASS trên Windows. Pilot đã chạy đủ 16 primary + 16 replay, thêm một retry chẩn đoán, nhưng strict replay attempt 1 chỉ PASS 15/16: B1/sample-s5 bị WALL_TIMEOUT. Audit giữ FAIL; retry không thay kết quả gốc. Linux/cross-OS UNVERIFIED, bước khóa test nghiên cứu còn BLOCKED.

## Source và hợp đồng

| File | Vai trò và bằng chứng |
| --- | --- |
| [metrics.py](../test%20version/benchmark/metrics.py) | Reducer `t1-metrics-1`, monitor extension `monitor-episodes-1`; golden tests và kiểm raw archive |
| [paired.py](../test%20version/benchmark/paired.py) | Khóa paired identity, bounded job, đọc journal không sửa, giữ primary/retry, CSV và replay |
| [test_benchmark_metrics.py](../test%20version/test_benchmark_metrics.py) | 23 golden tests có đáp án tính tay |
| [test_benchmark_paired.py](../test%20version/test_benchmark_paired.py) | 9 integration tests: CRASH/retry, reservation trước manifest, global lock, tamper/missing pair và CSV |
| [p22-pilot.py](checks/p22-pilot.py) | Xây/khóa input provenance, chạy job nhỏ và tổng hợp audit; không tune theo outcome |
| [p22-evidence-check.cjs](checks/p22-evidence-check.cjs) | Kiểm tra độc lập hash/journal, path/monitor/stall/goal, paired CSV, mẫu số và replay |
| [p22-catalog.json](../test%20version/benchmark/fixtures/p22-catalog.json), [p22-witnesses.json](../test%20version/benchmark/fixtures/p22-witnesses.json), [compact](../test%20version/benchmark/fixtures/p22-compact.json), [offset](../test%20version/benchmark/fixtures/p22-offset.json), [lower](../test%20version/benchmark/fixtures/p22-lower.json) | Input có provenance, registry và certificate; không sửa fixture P2.1 |
| [.gitattributes](../.gitattributes), [spec §9.3](benchmark-t1-spec.md#93-reducer-và-paired-dev-pilot-p22), [README](../test%20version/benchmark/README.md), [P2.2 plan](benchmark-p22-plan.md), [P2 plan](benchmark-p2-plan.md), [plan](benchmark-plan.md), [report](benchmark-report.md), audit này | Các file chỉnh sửa: giữ nguyên byte evidence/new input, giải thích metric/version/CLI/giới hạn và cập nhật liên kết trạng thái |
| [p22-audit-logs/](p22-audit-logs/) | Tests, certificate, paired manifests/bindings, journal/event/trajectory/result, bảng và replay thực |

Không đổi selector hoặc policy version 2 từ P2.1. Thêm module làm verifier fingerprint thay đổi nên pilot dùng certificate/run mới. Source reducer/paired/evaluator/runtime được khóa trước rollout; không sửa chúng trong lúc pilot đang chạy.

Reducer không dùng bảng CSV tổng hợp làm input chấm. Nó tái dựng first report từ confirmed observations đã nhận, áp dụng matching/memory/deadline, tính path từ start và trajectory, kiểm Euler, swept collision/bounds, stop hold và A. Result/evaluator event/projection được dùng đối chiếu, không ghi đè. Tolerance `1e-10` chỉ dùng consistency/Euler serialization; không cộng vào ngưỡng report, stop, A hoặc collision.

ReportSuccess/SafeMissionSuccess chỉ dùng primary positive đã bắt đầu, kể cả execution error. Negative false alarms có mẫu số riêng, tính cả report sai cấu trúc. Retry giữ ở attempts và không cứu primary. Mẫu số 0 là null, không phải 0; t=0 là mốc hợp lệ. Complete path gồm cả VERIFY_STOP; path dở dang là null và giữ observed prefix riêng. SPL/GT coverage/clearance chưa chứng nhận giữ null kèm lý do.

Monitor: trường protocol `monitor_interventions` giữ số lần vào từng state can thiệp ngoài PASS/IDLE/LIMIT. Chuyển giữa các state can thiệp tạo state entry mới, không phải đợt liên tục mới. Extension đặt tên riêng `monitor_continuous_episodes`, active state changes, duration/per-state counts và nhãn none/limited/stopped. Đếm duration bằng interval thực thi × dt, không wall-time. Raw MONITOR_STATE và markers được đối chiếu. Mất prefix/counter telemetry không được tự suy thành 0; các giá trị chưa đầy đủ là UNVERIFIED/null.

Paired manifest schema `t1-paired-2` khóa resolved bytes/hash, shared/config/budget, environment/family/split/seed, sensing, động học, planner, evaluator/metrics source, policy identity và execution profile. Một slot là một batch/episode với isolation, journal và resume P1b.2. Profile single-job-v1 dùng OS lock trước cả validation, chỉ cho một job/audit trong paired run; không sửa mission budget/watchdog. Reader kiểm hash-chain/FINISH và không sửa raw journal. Thiếu/sai cặp, đổi source/config/runtime hoặc sửa result đã chốt phải bị từ chối. Bảng paired dùng attempt 1; all-attempts giữ cả primary/replay và mọi retry/error.

Reservation có journal START nhưng chưa có attempt manifest được giữ là error sau batch recovery. Khi không có execution evidence, reducer chỉ dùng reservation identity đã kiểm chứng và để physics/telemetry UNVERIFIED/null, không tạo header giả. Replay chưa hề chạy policy là UNVERIFIED. Có raw execution nhưng mất manifest thì BLOCKED; không suy diễn identity.

## Corpus và pilot

Hồ sơ profile hiện hành: [pilot-serial/](p22-audit-logs/pilot-serial/). [pilot/](p22-audit-logs/pilot/) chỉ là preflight chưa rollout. [pilot-verified/](p22-audit-logs/pilot-verified/) là pilot schema 1 đã đủ 32 rollout nhưng FAIL replay, được giữ nguyên. Certificate đủ cả chín episode (một test chỉ validate), rollout chỉ tám dev episode theo [provenance](p22-audit-logs/pilot-serial/provenance.json): **7 layout, 6 family khai báo**, không phải tám môi trường độc lập; cần review provenance ngoài tác giả trước nghiên cứu.

- Giữ năm S1–S5 P2.1 làm regression anchors. Không dùng chúng kết luận B1 tốt hơn B0.
- `pilot-offset-s1`: layout hai box mới, camera x=0.12 m/y=0.08 m/yaw=0.08 rad, cùng cấu hình cho hai policy.
- `pilot-compact-s5`: bounds nhỏ thuộc cùng family empty-layout; không có goal đạt min-distance chung, dùng kiểm stalled/sensing tới deadline.
- `pilot-lower-s3`: cùng environment/family S3 cũ, start/target variant ở nhánh dưới; giữ budget chung 600 s và witness nhánh còn lại. Không cấp witness hoặc gate cho policy.

Không thêm budget riêng hoặc sửa grading để giúp controls/pilot đạt. `primary` và `replay` là hai lần chạy riêng từng policy, không ép B0/B1 chung trajectory. Bảng paired dev là đối chứng pipeline, không phải bảng chất lượng nghiên cứu.

## Lệnh đã chạy và kết quả

Python thực chạy: `C:/Users/lemin/AppData/Local/Programs/Python/Python310/python.exe` (3.10.11), Windows, NumPy 2.2.6, PyYAML 6.0.3. Lệnh từ gốc repo trừ nơi ghi rõ:

```powershell
python -B -m unittest discover -s "test version" -p "test_benchmark*.py" -v
python -B docs/checks/p22-pilot.py prepare --output docs/p22-audit-logs/pilot-serial
python -B docs/checks/p22-pilot.py run --output docs/p22-audit-logs/pilot-serial --limit 2
python -B docs/checks/p22-pilot.py audit --output docs/p22-audit-logs/pilot-serial
node docs/checks/p22-evidence-check.cjs
```

Các slot có thể gọi `python -B -m benchmark.paired run --output docs/p22-audit-logs/pilot-serial/paired --policy POLICY --replica primary|replay --episode-id ID` từ `test version/`. Chạy tuần tự, không tests/pilot khác đồng thời. Output mỗi lệnh phải khác nhau hoặc resume đúng manifest. Không chạy hai driver tự chọn cùng pending job.

Audit và checker đã thực chạy, cùng trả **exit 1** vì strict replay FAIL; CSV/identity/monitor đã kiểm đúng không biến lượt lỗi thành PASS. Bảng [paired-results.csv](p22-audit-logs/pilot-serial/paired/analysis/paired-results.csv) có 8 hàng; [episodes.csv](p22-audit-logs/pilot-serial/paired/analysis/episodes.csv) chỉ 16 primary attempt 1, [attempts.csv](p22-audit-logs/pilot-serial/paired/analysis/attempts.csv) giữ các retry của replica primary và [all-attempts.csv](p22-audit-logs/pilot-serial/paired/analysis/all-attempts.csv) có 33 hàng gồm replay/retry/error. Log từng slot có batch journal, attempt manifest, events, trajectory và evaluator result. Không sửa raw archive để tái hiện trên đường dẫn khác.

Lệnh tái hiện production regression từ `test version/`:

```powershell
python -B -m unittest test_collision_monitor_ros1 test_launcher_ros1 test_navigation_ros1 test_search_ros1 test_simulation test_vision_tracking -v
```

| Kiểm tra | Trạng thái/bằng chứng |
| --- | --- |
| Golden metrics | **PASS 23 tests**, [metric-verified-tests.txt](p22-audit-logs/metric-verified-tests.txt) |
| Toàn bộ benchmark | **PASS 132 tests**, không skip; [benchmark-serial-tests.txt](p22-audit-logs/benchmark-serial-tests.txt). Bao gồm 23 metrics + 9 paired, không cộng hai lần. Dòng YAML error là negative CLI test. Lượt 130 là schema 1 trước bổ sung khóa toàn pair. |
| Primary CRASH/INTERRUPTED và retry SUCCESS | **PASS integration**, primary vẫn lỗi, report/safe rates không lấy retry; raw [integration-serial/](p22-audit-logs/integration-serial/) |
| Concurrency/identity profile | **PASS**, global lock chặn job thứ hai trước tạo binding; đổi max_active_jobs bị từ chối, kể cả tự rehash manifest; [profile-identity-test.txt](p22-audit-logs/profile-identity-test.txt) |
| Production regression | **PASS 96 tests**, [regression.txt](p22-audit-logs/regression.txt) |
| P2.1 raw log regression | **PASS** trên các primary P2.1, [p21-raw-regression.json](p22-audit-logs/p21-raw-regression.json); không là kết quả nghiên cứu |
| Full corpus certificates | **PASS 9**, [prepare-serial.txt](p22-audit-logs/prepare-serial.txt) |
| Raw metric reductions | **PASS consistency 33/33**, [audit.json](p22-audit-logs/pilot-serial/paired/analysis/audit.json). Một trajectory lỗi là partial/UNVERIFIED, full path/counters null; không phải 33 mission thành công. |
| Paired identity / CSV / giữ primary | **PASS**, 8 cặp và 16 primary; [independent-check.json](p22-audit-logs/pilot-serial/independent-check.json), [CLI](p22-audit-logs/independent-check-cli.txt). Không mất/nhân đôi episode, retry vẫn ở bảng riêng. |
| Monitor count từ raw pilot | **PASS**, checker độc lập tái tính 114 state entries và 114 đợt liên tục trên 33 attempts, cả prefix lỗi. Các ví dụ golden có đổi active state kiểm tra trường hợp hai số khác nhau; không suy ra pilot đã phát mọi state. |
| Dev pilot thực chạy | **PASS execution coverage**, đủ 32 slot + 1 retry. 16 primary không có execution error; S1–S4/offset/nhánh dưới hoàn thành, S5 kết thúc mission TIMEOUT. Không dùng làm success rate nghiên cứu. |
| Strict replay attempt 1 | **FAIL 1/16**, 15 cặp còn lại PASS đầy đủ trajectory/events/result. [Audit CLI](p22-audit-logs/pilot-serial-audit.txt) và checker độc lập cùng giữ FAIL. |
| Retry B1/S5 chẩn đoán | **PASS semantic comparison**, [diagnostic-replay.json](p22-audit-logs/pilot-serial/diagnostic-replay.json). Batch CLI vẫn exit 1/execution_errors=1 vì attempt 1 còn lỗi; không cứu gate. |
| Contract/docs regression | **PASS 24**, [contract-final.txt](p22-audit-logs/contract-final.txt) |
| Liên kết và source mới | **PASS**, local links/anchors trên 7 tài liệu, whitespace của 6 source/test mới và git diff --check; [document-source-check.json](p22-audit-logs/document-source-check.json) |
| Production unchanged | **PASS source check**, [final-source-check.json](p22-audit-logs/final-source-check.json) và [policy identities](p22-audit-logs/source-state.json); không chỉnh bất kỳ source/YAML production hoặc selector cũ |
| Linux/cross-OS | **UNVERIFIED**, chưa thực chạy |
| Khóa test nghiên cứu / nghiệm thu P2.2 tổng thể | **BLOCKED / chưa đạt**, cần giải quyết gate replay và review corpus theo điều kiện bên dưới |

## Đọc trace pilot hiện hành

[Trace review](p22-audit-logs/pilot-serial/trace-review.json) giữ đường dẫn raw attempt cho từng kết luận. Không có positive mission thất bại ở primary; bốn negative primary kết thúc TIMEOUT đúng budget, không report sai. Các goal failure giữa mission vẫn có thật: hai goal ở S2 cho mỗi policy, một ở S4/B1 và bốn ở nhánh dưới/B1. Đây là outcome navigation được giữ, không phải tự động CRASH của runner.

Compact S5 bắt đầu stalled ở t=26 s, tiếp tục sensing tới deadline t=60 s (34 s stalled). Nhánh dưới có stalled ở cả hai policy rồi tiếp tục khám phá; B0 dùng fallback. B1 có 11 sensing steps không raw-visible từ t=418.0 đến 419.0 s sau confirmation t=416 s, mission vẫn FOLLOW rồi hoàn tất. Đây là khoảng mất raw ngắn khi còn memory; chưa chứng minh pilot đã đi qua mọi nhánh LAST_SEEN/memory expiry. Các chi tiết này là kiểm tra phối hợp pipeline, không xếp hạng B0/B1.

## Thất bại và giới hạn

Pilot schema 1 có đủ 16 primary và 16 replay. Primary không có execution error; một replay B0/pilot-lower-s3 bị WALL_TIMEOUT ở t=242.1 s mô phỏng, wall≈301.4 s. Primary cùng case đã SUCCESS_REPORT ở t=313.9 s; đây là lỗi thực thi/watchdog, không phải mission TIMEOUT. [Kiểm tra prefix](p22-audit-logs/replay-wall-timeout.json) xác nhận trajectory replay khớp byte với prefix của primary và workers đã được thu hồi. [Audit schema 1](p22-audit-logs/pilot-verified/paired/analysis/audit.json) ghi FAIL đúng, không lấy retry hoặc xóa lỗi để PASS. Source schema 1 lưu tại [source-snapshot/](p22-audit-logs/pilot-verified/source-snapshot/).

Nguồn gây chậm cụ thể ngoài ứng dụng chưa được profile đầy đủ; không khẳng định chỉ CPU contention. Tuy nhiên chạy nhiều job/proof đồng thời là nguồn tải mà harness kiểm soát được. Schema 2 thêm admission lock và audit exclusion, chạy lại **toàn bộ** corpus trong paired run mới. [Comparability check](p22-audit-logs/profile-comparability.json) xác nhận inputs, shared configs/budgets, policy identities và metric identity không đổi; chỉ paired.py đổi. Không tăng watchdog, không chỉnh scoring. Hai hồ sơ không được trộn thành mẫu thống kê hoặc bảng nghiên cứu.

Một wrapper PowerShell của pilot đầu bị sai cú pháp redirect console sau khi sáu job đã hoàn tất. [Ghi nhận](p22-audit-logs/wrapper-issue.json) tách lỗi wrapper khỏi execution result; raw journal/result vẫn nguyên vẹn, các attempt không bị chạy lại. Wrapper đã sửa và kiểm bằng các nhóm replay sau đó.

Profile tuần tự schema 2 có replay B1/sample-s5 kết thúc WALL_TIMEOUT ở t=38.4 s mô phỏng, wall=2055.39 s; primary cùng case kết thúc mission TIMEOUT đúng t=180 s, wall=103.86 s. [Kiểm tra prefix](p22-audit-logs/pilot-serial/replay-wall-timeout.json) xác nhận trajectory khớp byte với prefix primary và workers_reaped=true. [Windows Kernel-Power](p22-audit-logs/pilot-serial/host-power-events.json) ghi máy vào Modern Standby trong lượt này và thoát lúc 16:57:21, ngay trước khi ghi result lúc 16:57:22. Đây là bằng chứng host bị gián đoạn, không phải sai khác thuật toán đã quan sát được. Metadata file chỉ hỗ trợ chẩn đoán, không dùng chấm/replay; chưa có scheduler trace đầy đủ để quy mọi overhead cho standby. Admission lock không bảo đảm host luôn hoạt động. Không tăng watchdog, không xóa/ghi đè attempt lỗi, không chuyển nó thành mission timeout hợp lệ. Tất cả slot còn lại đã hoàn tất. Retry attempt 2 chạy tới t=180 s và có semantic hash giống primary nhưng chỉ là bằng chứng chẩn đoán, không đổi strict gate của attempt 1.

Ranh giới policy là cooperative process isolation, chưa phải filesystem sandbox chống code đối địch. Pair manifest/hash-chain phát hiện thay đổi ngoài ý muốn, không phải chữ ký chống sửa đồng loạt. Pair initialization chưa nghiệm thu mất điện; thiếu manifest/checksum phải BLOCKED, không tự repair. Counter B0 có work limit 1 triệu candidates; vượt thì UNVERIFIED, không xấp xỉ để PASS. Input/output giả định một người dùng workspace kiểm soát, không chống symlink race đối địch.

Corpus hand-authored nhỏ, 2D, oracle/noiseless và budget xác định; không đại diện thiên tai thật hoặc false-positive YOLO. Family gần giống cần review provenance ngoài hash. Raw test export có catalog path tới temporary directory đã cleanup: dùng để audit, không sửa manifest để resume; tái hiện bằng test source. Pilot dùng input tracked và paths hiện hành để kiểm lại end-to-end.

## Điều kiện trước khi khóa test nghiên cứu

1. Reducer/paired identity/monitor counters và replay phải có audit PASS hiện hành; chốt nghĩa field/version và chính sách null/primary/retry trước khi chạy test.
2. Không còn lỗi implementation/runner chưa giải thích. Thất bại thuật toán hợp lệ vẫn được giữ; không bắt buộc mọi policy tìm được mọi scene.
3. Dựng corpus lớn hơn với provenance được review, tách dev/test theo environment và family, kiểm near-duplicates, đủ đặc trưng S1–S5 và certificate PASS theo sensor/footprint/budget đã khóa.
4. Khóa manifest, policy IDs/params/source, evaluator/reducer/runtime, fixture resolved bytes và common configs. Không dùng test để tune policy hoặc đổi grading/budget sau khi xem kết quả.
5. Chốt trước bảng paired, mẫu số lỗi và kế hoạch thống kê theo family. Replica/seed trên cùng geometry không phải mẫu môi trường mới. Chưa có reference path/coverage đáng tin thì SPL/GT coverage vẫn null.
6. Linux/cross-OS chỉ được công bố trong phạm vi đã có log thật; T3 cần hiệu chuẩn/hardware evaluation riêng, không suy từ pilot Windows.

Việc tiếp theo trước nghiệm thu: khóa điều kiện host hoạt động liên tục, ghi nhận host liveness để phân biệt OS suspend/scheduling gap với worker hang và xác minh lại toàn bộ paired manifest ở output mới. Nếu đổi harness/profile thì version hóa; giữ cả hai hồ sơ FAIL hiện tại. Không đổi scoring hoặc watchdog để che lỗi, không chọn riêng các lượt xanh làm bằng chứng nghiệm thu. Chưa mở held-out test và chưa đề xuất kết luận B1 tốt hơn B0.
