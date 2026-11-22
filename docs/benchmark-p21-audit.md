# Nghiệm thu P2.1 — Hai search selector chính thức

Nền commit `4db37ff`, nhánh `feat/benchmark`, protocol **t1-0.2**. Phạm vi là implementation, compatibility và dev controls; không phải đánh giá nghiên cứu hoặc benchmark comparison. Không chạy ROS, SSH, camera, chassis hay launcher.

**P2.1 PASS trong phạm vi implementation và dev controls trên Windows.** Có 20 rollout hoàn tất trong hồ sơ nghiệm thu, replay riêng từng policy và kiểm tra độc lập. Các lượt bị ngắt được giữ riêng, không coi là hoàn tất. Linux/cross-OS UNVERIFIED; metrics/paired pilot còn cổng P2.2 bên dưới. Đây không phải kết luận B1 tốt hơn B0.

## Contract và phần dùng chung

Đã rà §5.1–5.2, giới hạn P1b.2 và kế hoạch P2. Không đổi quy tắc matching, A/F, budget, Euler, oracle, monitor, stop check, end_reason hoặc primary/retry. Các câu trạng thái cũ “chưa có verifier/runner” trong spec được cập nhật, không thay scoring.

`GoalSelector.select(CandidateDomain) -> Selection | None` là giao diện chọn goal. Selector nhận `SelectionState` bất biến gồm pose/time, estimated coverage, blacklist, spun_at và focus; grid là costmap từ LiDAR của Navigator. Không truyền GT/target thật, witness, certificate, evaluator hoặc mission/nav instance vào selector.

`SelectorMission` giữ logic spin/dwell, `_goto`, lost-target, approach và reporting từ SearchMission; cả B0/B1 dùng cùng adapter này. Navigator async + next-tick barrier, Collision Monitor, camera oracle, coverage extrinsics, bounds và lifecycle/evaluator giữ nguyên. Policy được spawn riêng, không nhận kết quả GT matching. Ranh giới này dành cho code hợp tác, không phải filesystem sandbox chống policy chủ động đọc ổ đĩa.

- **B0**: ID `b0-fixed-order-v2`, algorithm `fixed-order-v2`, implementation version `2`. Lưới neo ở pose sensing đầu, xoay theo yaw đầu, bỏ gốc; thứ tự vòng Chebyshev/góc/kx/ky. Mỗi lần chọn xét lại mọi index, không xóa unknown/occupied/out-of-window. Index đã giao không giao lại dù failed. Fallback sắp các ô của miền chung theo frame đầu, không dùng gain/min-gain/focus.
- **B1**: ID `b1-gain-distance-focus-v1`, implementation version `2`. Giữ nguyên integral gain, min-gain, score `gain/(distance+.5)`, focus divisor và strict row-major tie của production. Test đối chiếu 24 trạng thái gồm coverage, blacklist, revisit, focus còn/hết hạn và ngưỡng gain, cộng trường hợp `now=None` theo API production. Không sửa source production.

Miền chung version `observed-standable-v1`: raw-free/allowed, erosion một ô, gần unknown/chưa covered (dilation hai ô), footprint ở yaw hiện tại, bounds và blacklist/revisit/min-distance. Không sửa `grid.allowed` của planner. So với smoke P1b.2, không lặp việc loại một goal footprint không hợp lệ rồi erosion lại. Đây là khác biệt adapter được version hóa cho cả hai official policy; không coi log `search-mission-smoke` cũ là log B1 chính thức.

Không có ứng viên thì cả hai giữ STALLED, command zero qua monitor, vẫn sensing tới deadline và retry theo `spin_dwell_s`. Target confirmed hợp lệ được xử lý qua approach chung; việc tạm bỏ target unreachable vẫn theo logic core. Không phát SEARCH_EXHAUSTED.

## Provenance, logging và comparability

Single manifest có `policy_identity`; batch identity khóa ID/version, selector parameters, shared search parameters và source hashes (selectors/adapter/coverage/SearchMission/Navigator/monitor). Resume từ chối khác identity; replay kiểm tra identity cùng trajectory, semantic events và result. Cập nhật source làm bundle cũ stale; không sửa certificate/manifest để đi vòng kiểm tra. Default CLI vẫn là smoke cho tương thích.

Event bổ sung: SELECTOR_DECISION chứa point/kind/index và danh sách skip reasons; WAYPOINT_OUTCOME ghi reached/failed/preempted; SELECTOR_STATUS lưu counters/state. Waypoint active ở deadline/error chưa có outcome reached được giữ unresolved. Không tự coi helper test là một rollout.

B0 `unresolved_waypoints` = tổng index lưới trừ index có outcome reached, gồm chưa giao/failed/preempted/active; fallback không thêm index lưới. B1 không có lưới nên null. `goals_selected` đếm goal khám phá, không đếm approach/LAST_SEEN. `fallback_goals` là phần được giao từ fallback. Stalled duration được dựng từ state mission theo simulated time, không lấy số tick có reason monitor làm số giây stalled. Recovery giữ các status đã nhận; phần chưa biết không tự điền 0.

Implementation `2` sửa hai lỗi do regression tests tái hiện: B1 không được áp dụng focus khi `now=None`, và worker bootstrap chưa trả status thì counters phải là null trong result/CSV. Selector events được gắn source policy. [Log trước sửa](p21-audit-logs/edge-before-fix.txt) có hai ERROR; [16 tests sau sửa](p21-audit-logs/selector-tests.txt) đã PASS. Không đổi trọng số hoặc hành vi với clock mission hữu hạn; vẫn tạo certificate/run mới do source identity và log semantics thay đổi. Nếu crash sau một status, counters chỉ là snapshot cuối đã nhận, với trajectory_complete=false; không coi đó là tổng đã xác minh của cả mission.

Thêm policy identity vào semantic replay làm hash replay mới khác định dạng fingerprint lịch sử P1b.2; không sửa log/hash cũ. Muốn so controls phải dùng source/certificate hiện hành, cùng fixture/shared config và runtime. Không đổi protocol scoring.

## File mới và chỉnh sửa

| File | Thay đổi và kiểm tra |
| --- | --- |
| [selectors.py](../test%20version/benchmark/selectors.py) | Interface/state, common domain, B0/B1 và reducer chẩn đoán; selector/compatibility tests và dev controls |
| [policy.py](../test%20version/benchmark/policy.py) | Adapter chung, source/parameter identity và dispatch official ID; spawn/isolation, mission và runner tests |
| [runner.py](../test%20version/benchmark/runner.py) | Public spacing, policy manifest và selector log/result; single acceptance và controls |
| [batch.py](../test%20version/benchmark/batch.py) | Khóa policy identity, projection/recovery chẩn đoán; batch/resume tests |
| [replay.py](../test%20version/benchmark/replay.py) | So identity, phát hiện đổi version ngay cả khi trajectory giống; replay tests và controls lặp |
| [test_benchmark_selectors.py](../test%20version/test_benchmark_selectors.py) | 16 tests mới, gồm thực chạy policy/simulator/evaluator, bootstrap hang và hai hidden worlds cho cùng sensing |
| [p21-dev-controls.py](checks/p21-dev-controls.py) | Full-corpus certificate gate, rollout chỉ dev, lặp/replay và resume từng policy |
| [p21-evidence-check.cjs](checks/p21-evidence-check.cjs) | Kiểm tra độc lập source/artifact hashes, journal, attempts, replay và dev-only |
| [.gitattributes](../.gitattributes) | Giữ nguyên byte archive P2.1 khi Git xử lý newline; kiểm tra hash bằng checker |
| [spec](benchmark-t1-spec.md), [README](../test%20version/benchmark/README.md), [report](benchmark-report.md), [plan](benchmark-plan.md), [P2 plan](benchmark-p2-plan.md), audit này và [P2.2 plan](benchmark-p22-plan.md) | Trạng thái hiện tại, lệnh tái hiện, version/giới hạn và cổng metrics; contract tests/link review |
| [p21-audit-logs/](p21-audit-logs/) | Test logs, certificates, event/trajectory/result/manifest/journal và controls/replay thực chạy |

## Lệnh và kết quả thực

Chạy từ gốc repository bằng Python 3.10.11, NumPy 2.2.6, PyYAML 6.0.3 trên Windows; `python` dưới đây là `C:/Users/lemin/AppData/Local/Programs/Python/Python310/python.exe` trong phiên kiểm tra.

Script dev đầy đủ dùng output mới. Hai lệnh `run-3` và `--finalize-existing` là cách hoàn tất hồ sơ sau khi phiên tool dài bị ngắt trong lần này, không phải bước cần chạy thêm sau một script đầy đủ đã hoàn tất bình thường.

```powershell
python -B -m unittest discover -s "test version" -p test_benchmark_selectors.py -v
python -B -m unittest discover -s "test version" -p "test_benchmark*.py" -v
python -B -m unittest discover -s "test version" -p "test_*ros1.py" -v
python -B -m unittest discover -s "test version" -p test_simulation.py -v
python -B -m unittest discover -s "test version" -p test_vision_tracking.py -v
node --test docs/checks/benchmark-contract.test.cjs
node "test version/test_navigation_status.js"
python -B docs/checks/p21-dev-controls.py --output docs/p21-audit-logs/dev-final
python -B "test version/benchmark_batch.py" --catalog "test version/benchmark/fixtures/catalog.json" --bundle docs/p21-audit-logs/dev-final/certificates --output docs/p21-audit-logs/dev-final/b1-gain-distance-focus-v1/run-3 --policy b1-gain-distance-focus-v1 --episode-id sample-s1 --episode-id sample-s2 --episode-id sample-s3 --episode-id sample-s4 --episode-id sample-s5
python -B docs/checks/p21-dev-controls.py --output docs/p21-audit-logs/dev-final --finalize-existing --b1-repeat run-3
node docs/checks/p21-evidence-check.cjs
```

| Hạng mục | Trạng thái và bằng chứng |
| --- | --- |
| Selector/unit/compatibility/spawn integration | **PASS 16 tests**, [selector-tests.txt](p21-audit-logs/selector-tests.txt) |
| Toàn bộ benchmark acceptance/regression | Implementation 2 **PASS 100 tests**, không skip; [benchmark-final-tests.txt](p21-audit-logs/benchmark-final-tests.txt). Bao gồm 16 test mới, không cộng hai lần. Dòng malformed YAML là negative CLI test trả exit 2. Lượt cũ 98 tests chỉ là lịch sử trước hai đối chứng biên. |
| Production core/launcher/search | **PASS 80 tests**, [regression.txt](p21-audit-logs/regression.txt) |
| Production simulation / vision | **PASS 8 + 8 tests**, [simulation.txt](p21-audit-logs/simulation.txt), [vision.txt](p21-audit-logs/vision.txt); tổng production 96 |
| Contract / dashboard | **PASS 24 / 13**, [contract.txt](p21-audit-logs/contract.txt), [dashboard.txt](p21-audit-logs/dashboard.txt) |
| Dev controls S1–S5 | **PASS** cho cả hai policy; [controls.json](p21-audit-logs/dev-final/controls.json), [dev-final-cli.txt](p21-audit-logs/dev-final-cli.txt), [B1 repeat](p21-audit-logs/b1-repeat-cli.txt) |
| Replay | **PASS** strict cả năm episode mỗi policy; trajectory, semantic events/result và policy identity khớp; [finalize-cli.txt](p21-audit-logs/finalize-cli.txt) |
| Resume | **PASS**, giữ nguyên 26 file mỗi policy (25 file attempt + journal), không thêm attempt; [controls.json](p21-audit-logs/dev-final/controls.json) |
| Kiểm tra độc lập | **PASS** source/certificate/artifact hashes, journal/attempts, counters, dev-only, di chuyển S2–S4, vượt gate S3, S5 sensing tới deadline và replay; [independent-check.json](p21-audit-logs/independent-check.json) |
| Từ chối batch dở | **PASS** negative probe: không có result thì finalize từ chối; [finalize-incomplete-negative.txt](p21-audit-logs/finalize-incomplete-negative.txt) |
| Production diff / tài liệu | **PASS** production diff rỗng tại [source-state.json](p21-audit-logs/source-state.json); [document-links.json](p21-audit-logs/document-links.json) kiểm link; git diff --check ngoài raw archive không có lỗi |
| Linux / cross-OS | **UNVERIFIED**, không có lần chạy Linux trong P2.1 |
| Monitor episode metric | **BLOCKED cho mục đích pilot**, cần nghiệm thu định nghĩa/reducer ở P2.2; raw monitor log vẫn được giữ |

## Dev controls và giới hạn

Catalog mẫu có dev S1–S5 và một fixture test dùng kiểm tra registry. Script xác minh toàn catalog nhưng chỉ rollout dev; không dùng fixture test để chỉnh selector. Không đổi hình học, budget hoặc shared YAML để làm controls đạt. S1 có thể đi thẳng vào approach mà không cần goal khám phá; S2/S3/S4 phải kiểm tra goal và chuyển động từ trace.

B0 implementation 2: S1–S4 có SUCCESS_REPORT và S5 TIMEOUT không có report; replay dùng `run-1`/`run-2`, đều PASS. S2/S3 có fallback thực sự; S4 có cả goal cố định và fallback. B0 không dừng sớm vì hết goal; kiểm tra S5 có đầy đủ 1.800 bước sensing tới budget.

B1 implementation 2: S1–S4 có SUCCESS_REPORT và S5 TIMEOUT không có report; replay dùng `run-1`/`run-3`, đều PASS. S2–S4 có chọn goal và dịch chuyển thật. Với cả hai, S3 có đoạn trajectory cắt gate, không chỉ có witness hoặc helper test. Chứng nhận/evaluator độc lập xác nhận chuỗi report–tiếp cận–dừng.

Các rollout hoàn tất dùng để nghiệm thu không có execution error hoặc thất bại nhiệm vụ dương tính cần phân loại thêm. Hai lỗi biên trước sửa là **lỗi implementation đã khắc phục**, còn hai lượt thiếu result bên dưới là **gián đoạn thực thi, nguyên nhân UNVERIFIED**, không phải bằng chứng thất bại thuật toán. Không suy ra tỷ lệ nghiên cứu, tốc độ hoặc độ an toàn tương đối từ controls; các rates diagnostic sẵn có trong summary.json không được dùng làm bảng B0/B1. Những lần chạy này không phải các môi trường độc lập mới.

[Lượt dev đầu](p21-audit-logs/dev/) dùng implementation 1: B0 xong hai lượt và replay/resume; B1 dừng giữa S2, không có result cuối. Phiên tool cũ không còn tồn tại và không còn process Python khi kiểm tra; nguyên nhân ngắt **UNVERIFIED**, không gán thành thất bại thuật toán. Giữ nguyên raw log và [ghi nhận gián đoạn](p21-audit-logs/dev-interruption.json). Hồ sơ nghiệm thu hiện hành phải lấy ở [dev-final/](p21-audit-logs/dev-final/), không trộn source/certificate của hai lượt.

Trong implementation 2, phiên tool dài cũng kết thúc trước khi B1 `run-2` hoàn tất; S5 chỉ có log dở, không có result. [Ghi nhận](p21-audit-logs/dev-final-interruption.json) giữ nguyên lượt này. B1 được chạy một batch mới `run-3` bằng CLI riêng ([log](p21-audit-logs/b1-repeat-cli.txt)); không overwrite hoặc sửa primary attempt ở run-2. Script tổng hợp chỉ nhận batch đã đủ result, có `--finalize-existing --b1-repeat run-3`. [Đối chứng batch thiếu](p21-audit-logs/finalize-incomplete-negative.txt) đã bị từ chối bằng ValueError; không biến gián đoạn thành PASS. Các lượt dở dang vẫn trong archive, không dùng làm tỷ lệ nghiên cứu.

Đã rà monitor counting: event hiện phát khi đổi sang trạng thái ngoài PASS/IDLE/LIMIT, không lặp từng tick cùng state. Tuy nhiên nhiều trạng thái can thiệp chuyển tiếp không có khoảng bình thường ở giữa vẫn có thể được đếm nhiều lần. Không đổi monitor hoặc metric cũ trong P2.1. [P2.2 plan](benchmark-p22-plan.md) đặt test burst/state transitions và chốt nghĩa metric là điều kiện bắt buộc trước paired pilot.

Các giới hạn còn lại: một layout dev cho mỗi scenario chưa chứng minh coverage tổng quát; B0 không bảo đảm đầy đủ; lưới rất dày/miền rất lớn chưa được nghiệm thu về chi phí; planner/costmap vẫn dựa sensing và có thể đưa quỹ đạo ra bounds (evaluator phải chấm thất bại, không clip). Oracle không đo sai số camera hoặc false-positive detector thật. Linux, power-loss, robot thật và policy đối địch ngoài phạm vi đã kiểm chứng. SPL/GT coverage/clearance nghiên cứu chưa có nghiệm thu mới. Chưa khóa test, chưa có paired pilot hay bảng B0/B1.

Bước tiếp theo: [P2.2](benchmark-p22-plan.md) nghiệm thu reducer metrics bằng ví dụ tính tay, chốt monitor bursts, khóa paired manifest và chạy dev pilot. Cổng này chưa PASS chỉ vì P2.1 đã có controls/replay.
