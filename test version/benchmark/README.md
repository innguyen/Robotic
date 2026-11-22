# T1 offline — P2.2 metrics và paired dev pilot

Final re-acceptance: [audit](../../docs/benchmark-p22-final-audit.md), [P2.3 plan](../../docs/benchmark-p23-plan.md). Wrapper opt-in `benchmark.stabilization` version `stabilized-paired-1` chạy trên paired schema 2 mà không sửa runner/policy/scoring. `host-liveness-1` ghi sidecar clock/CPU/progress, không vào semantic events và không nới watchdog. Không tự đổi thiết lập nguồn điện; giữ máy hoạt động liên tục trong phiên chạy.

Sau khi benchmark tests và production regression đã kết thúc, tạo output mới bằng `docs/checks/p22-pilot.py prepare --output docs/p22-final-audit-logs/pilot`. Từ `test version/`, chạy `python -B -m benchmark.stabilization init --output docs/p22-final-audit-logs/pilot/paired`, rồi `run` cho từng policy/replica/episode với `--policy`, `--replica`, `--episode-id`. Chạy tuần tự đủ 32 slot; wrapper từ chối slot đã có và không hỗ trợ thay attempt 1 bằng retry. Kết thúc bằng `audit` và checker `node docs/checks/p22-final-evidence-check.cjs` từ gốc repo; xem audit để lấy lệnh/bằng chứng môi trường và Kernel-Power. Các hồ sơ FAIL trước vẫn nguyên trạng.

Protocol: `t1-0.2`, verifier `p1a2-1`. Đã có validator, evaluator, single runner, batch/resume và hai selector chính thức có version. Chưa có đánh giá test hoặc bảng so sánh B0/B1. Không khởi động ROS, camera, chassis hay launcher. Xem [đặc tả](../../docs/benchmark-t1-spec.md), [P1b.2](../../docs/benchmark-p1b2-audit.md) và [P2.1 audit](../../docs/benchmark-p21-audit.md) để biết phạm vi thực kiểm.

Metrics và paired pilot hiện hành được ghi tại [P2.2 audit](../../docs/benchmark-p22-audit.md). Các bảng dev dùng kiểm tra pipeline, không phải kết luận policy nào tốt hơn.

Python 3.10+ và PyYAML cho validator; runner cần thêm NumPy và Git để ghi provenance. Môi trường đã kiểm tra dùng Python 3.10.11, PyYAML 6.0.3, NumPy 2.2.6. Runner không cần OpenCV hoặc ROS. Các regression production vẫn dùng dependencies cũ. Golden tests chạy chéo cần Node.js.

Từ gốc repository, chạy test:

```powershell
python -B -m unittest discover -s "test version" -p "test_benchmark*.py" -v
```

Để gọi CLI, chuyển vào `test version/`. Đường suite và witness vẫn tính từ **gốc repository**, không theo cwd:

```powershell
cd "test version"
python -B -m benchmark --suite "test version/benchmark/fixtures/samples.json" --witnesses "test version/benchmark/fixtures/witnesses.json"
python -B -m benchmark --catalog "test version/benchmark/fixtures/catalog.json" --output "benchmark-results/validation-p1a2"
```

`--suite` là chế độ chẩn đoán, không được dùng `--output`. Để xuất artifact phải dùng `--catalog`: catalog khai báo mọi suite dev/test, witness file và provenance của mọi family. Kiểm tra toàn bộ registry trước mọi chứng nhận; không có filter để né kiểm tra. Catalog mẫu chỉ là corpus kiểm thử validator, không phải bộ test đã khóa. Các nhãn `reviewed` trong đó là ghi nhận tác giả đã đối chiếu hình học mẫu; corpus thực cần người quản lý dữ liệu rà provenance. Tool không thể phát hiện file chưa được khai báo trong catalog.

Exit 0 chỉ khi mọi certificate PASS; exit 2 nếu cấu trúc sai hoặc có FAIL/UNVERIFIED. Artifact có thể chứa certificate UNVERIFIED để audit: có artifact không đồng nghĩa được chạy benchmark. Không đưa witnesses cho policy. Bỏ `--witnesses` ở chế độ chẩn đoán sẽ khiến các episode dương tính UNVERIFIED, không tự suy đoán đường đi.

API:

- `schema.load_json()` từ chối duplicate key/NaN. `validate_corpus()` phải chạy trước chứng nhận; trả config đã resolve cùng digest YAML/source. Nó chỉ xác minh cấu trúc/config/layout/start, **không** xác nhận suite đủ điều kiện rollout.
- `layout.canonical_layout()/layout_hash()/registry()` quản lý hình học tĩnh, ID/family/split và physical duplicate. Vectors lưu trong `fixtures/layout-golden.json`, được tạo từ JS reference P0 và đối chiếu lại khi test.
- `geometry` dùng polygon lồi cho footprint có hướng, SAT intersection, khoảng cách Euclidean, ray/AABB và swept footprint. `swept()` nhận yaw cuối chưa wrap; nội suy tâm tuyến tính. Cận dịch chuyển là nửa chiều dài đoạn cộng bán kính thân nhân nửa độ quay, làm tròn ra ngoài; test PASS còn có guard 32 ULP theo scale tọa độ. Đây là điều kiện chứng nhận chặt hơn, không nới epsilon/bounds của evaluator. Giới hạn 20 tầng/4096 nút: chưa chứng minh được thì UNVERIFIED. Bounds đóng, không tự sinh tường raycast.
- `oracle.visible()` áp dụng extrinsics/FOV/range/occlusion. `Oracle.step()` nhận tick liên tiếp từ 0; trả `(raw_visible, immutable_observation_or_none)`. Không trả GT ID, obstacle hoặc evaluator. Chỉ Observation được phép đi qua ranh giới policy trong runner tương lai; không truyền instance Oracle hay episode dictionary.
- `scenarios.certify()` nhận **episode/shared/config đã validate** và witness riêng evaluator. Trả certificate có fixture/config/layout/evidence hashes, verified traits và lý do. `resolved_episode_bytes()` tạo chính byte artifact; không serialize lại khi resume. SearchConfig chỉ trong benchmark được resolve camera FOV/range theo shared camera; defaults production giữ nguyên.
- `witness.check_path()` dịch chuyển rồi quay ở từng waypoint, tạo và kiểm tra lệnh từng tick theo Euler, caps và swept footprint. Chênh lệch yaw waypoint là **unwrapped**: từ 0 tới 4π là hai vòng, không tự chọn góc ngắn. Khi tích phân, pose yaw vẫn wrap như protocol; swept kiểm tra góc chưa wrap trong interval. Chứng nhận thêm hold confirmation ở start S1, confirmation ở endpoint, interval report và hold dừng; cả lịch nằm trong budget nguyên tick.
- `proofs` dùng Fraction trên tọa độ binary64 cho ray và phân rã topology. Chứng minh âm dùng miền phủ lớn hơn F/A; chứng minh dương luôn cần witness với footprint thật. Bóng che của một obstacle lồi từ một điểm là lồi: bốn góc miền đều bị chặn bởi cùng obstacle thì toàn miền bị chặn. Phép chứng nhận dùng obstacle inset 32 ULP, không nhận tia chỉ tiếp xúc biên làm bằng chứng. Camera offset được bao bằng AABB của đĩa quay quanh thân, làm tròn ra ngoài.
- `artifacts` đọc catalog, dựng bundle và xuất atomically bằng staging cùng parent rồi rename. Manifest giữ hash mọi artifact, catalog và source của verifier. Output tồn tại chỉ được đọc lại nếu **mọi byte/file** khớp kết quả recompute; thay input/config/witness/verifier hoặc sửa/xóa/thêm file bị từ chối, không ghi đè.

Phạm vi chứng nhận hiện tại (đều có đối chứng):

- S1: thấy ở start, witness tới A. S2: toàn miền camera origin ở start bị che, có yaw hợp lệ thỏa range/FOV, witness thấy trước ngưỡng đường 0,75 m. Camera offset quá lớn/không nằm trong bóng che chứng minh được thì UNVERIFIED.
- S3: gate song song trục, phân rã rectangle theo mọi cạnh obstacle/gate (không phải raster lấy mẫu). Point-free space là overapproximation của F; vùng chứa start không thể thấy target theo proof bóng che. Đóng gate phải tách nhánh, mở riêng từng gate phải nối đúng nhánh. Main witness tới A và witness tới vùng đứng diện tích dương trong mỗi nhánh còn lại phải qua swept-footprint. Gate chéo, hơn 4096 cells hoặc chưa đủ witness: UNVERIFIED.
- S4: mọi tâm thuộc A nằm trong AABB phủ đĩa khoảng cách tiếp cận; chứng minh toàn AABB bị obstacle chặn từ start, đồng thời có witness đi vòng tới A. Chỉ chặn đường tới tâm người không đủ. Ngoài điều kiện đủ này: UNVERIFIED.
- S5: không có target, start có lân cận pose trống diện tích dương.
- Mỗi pha dùng lệnh không vượt một phần tư speed cap; lịch witness là chứng minh tồn tại theo động học Euler, **không** phải rollout hay thời gian hoàn thành B0/B1. Không mô phỏng động lực học, acceleration controller hoặc monitor trên witness. Giới hạn 100.000 tick/witness và budget; vượt khả năng chứng minh thì UNVERIFIED.
- Các proof âm trên tọa độ rectangle dùng số hữu tỷ; trig, khoảng cách và footprint vẫn binary64 với guard bảo thủ, chưa phải interval arithmetic toàn bộ. Không khẳng định an toàn robot thật. Family gần giống cần provenance/manual review; catalog mẫu chứa dev và test để kiểm tra leakage, không phải đánh giá held-out.

Bundle gồm `manifest.json` và mỗi episode có `resolved_episode.json`, `evidence.json`, `fixture_validation.json`. File certificate chứa lịch witness theo tick để tái kiểm tra. Staging không phải output hợp lệ. Lỗi Python trước rename được cleanup; nếu process bị kill có thể để lại staging, nhưng không công bố bundle dở. Chưa chứng minh durability khi mất điện; journal/resume của runner được mô tả ở phần P1b.2 bên dưới. Giả định output trong workspace do một người dùng kiểm soát; không bảo vệ race symlink từ tiến trình đối địch.

P1b.1 dùng `spawn` tách policy khỏi simulator/evaluator. Policy chỉ nhận config công khai và SensorFrame bất biến, không nhận GT/fixture/witness/certificate. Đây là ranh giới dữ liệu cho code hợp tác, chưa phải filesystem/security sandbox chống policy chủ động đọc dữ liệu trên ổ đĩa.

Coverage trong runner đã có adapter dùng camera pose/extrinsics và chỉ đọc ô free trên costmap quan sát được; không raycast GT cho policy. Nó dừng ở unknown/occupied, là estimated coverage. Core production không sửa; mọi policy runner dùng chung bước coverage này.

## Chạy một episode P1b.1

Từ thư mục `test version/`, tạo certificate theo đúng source hiện hành rồi chạy **một** episode:

```powershell
python -B -m benchmark --catalog "test version/benchmark/fixtures/catalog.json" --output "benchmark-results/p1b1-certificates"
python -B benchmark_run.py --catalog "test version/benchmark/fixtures/catalog.json" --bundle "benchmark-results/p1b1-certificates" --episode-id sample-s1 --output "benchmark-results/p1b1-single"
```

Từ gốc repository, entry point tương đương là `python -B "test version/benchmark_run.py" ...`. Mọi path input/output tính từ gốc repo. Output runner phải chưa tồn tại. Bundle cũ sẽ bị từ chối nếu source/config/evidence khác; thêm module benchmark cũng đổi verifier hash. Không chỉnh sửa certificate để vượt kiểm tra; tạo bundle mới.

Default `--policy search-mission-smoke` dùng Navigator async, SearchMission và Collision Monitor hiện tại qua adapter benchmark. Đây là smoke integration, chưa là B1 dùng so sánh. Các policy `test-*` chỉ để acceptance; không phải baseline. API `run_single(..., fault=...)` có actuator fault phục vụ test evaluator độc lập, ghi rõ trong manifest, không có flag actuator fault trên CLI.

Runner tạo `run_manifest.json`, `events.jsonl`, `trajectory.csv`, `result.json`. Manifest lưu config/source/bundle hashes, git SHA/dirty, seed, LiDAR model và planner barrier mode. Event có run/episode/attempt/sequence/tick; log vị trí quan sát và report riêng. Result phân biệt report_correct, mission_success, end_reason và trajectory_complete. Dữ liệu không hữu hạn bị sanitize, không ghi JSON NaN/Infinity.

Supervisor sở hữu hai process riêng (simulator và policy), chỉ wall watchdog dùng monotonic time. Clock nhiệm vụ là tick×0,1 s. Planner future được chờ có timeout, kết quả chỉ áp dụng ở lần core kiểm tra tiếp theo; cleanup bị treo vẫn được terminate/kill và join bằng supervisor. Sau report đầu, ngừng gọi policy, yêu cầu 0 qua monitor, tiếp tục tích phân/geometry/stop check, không sửa pose để chấm thành công.

Single-episode API giữ định dạng P1b.1. Batch/journal/resume được bổ sung trong P1b.2 bên dưới. Các trường không biết khi phục hồi giữ null/trajectory_complete=false; chưa nghiệm thu positive controls B0/B1.

Acceptance tests có thể lưu từng run để audit (chọn đường dẫn export mới):

```powershell
$env:T1_ACCEPTANCE_EXPORT = 'docs/local-acceptance-export'
python -B -m unittest discover -s "test version" -p test_benchmark_runner.py -v
```

Lệnh test trên chạy từ gốc repository; bỏ biến môi trường để dùng temporary output và cleanup. Không đưa các log/GT artifact này vào policy.

## Batch, journal và resume — P1b.2

Từ `test version/`, tạo bundle theo source hiện hành, rồi chạy batch tuần tự:

```powershell
python -B -m benchmark --catalog "test version/benchmark/fixtures/catalog.json" --output "benchmark-results/p1b2-certificates"
python -B benchmark_batch.py --catalog "test version/benchmark/fixtures/catalog.json" --bundle "benchmark-results/p1b2-certificates" --output "benchmark-results/p1b2-run" --episode-id sample-s2 --episode-id sample-s3 --episode-id sample-s4
```

Chạy lại cùng lệnh, thêm `--resume` để tiếp tục. Giữ nguyên selection, policy, config, corpus và runtime. `--retry-errors` chỉ đi với resume và chỉ tạo attempt mới cho lỗi thực thi; không retry TIMEOUT/COLLISION/false report của nhiệm vụ. Attempt 1 luôn là primary. Có thể lọc `--seed N` (không đổi seed), `--max-episodes N` hoặc nhiều `--episode-id`; mọi fixture trong catalog vẫn phải được kiểm tra và PASS **trước** lọc. Các selector P2.1 dùng cùng luồng batch này.

Manifest khóa fingerprint toàn corpus/config/source/evidence, runtime, selection và policies. Chỉ một supervisor giữ OS lock; process chết sẽ giải phóng lock tự động. Journal hash-chain fsync START trước khi chạy; FINISH chỉ ghi sau atomic result. Khi resume:

- FINISH đã chốt: kiểm tra hash và giữ nguyên mọi file attempt.
- Có atomic result nhưng thiếu FINISH: nhận result sau kiểm tra identity, không chạy lại/ghi đè.
- Chưa có result: chốt CRASH/INTERRUPTED từ log nhận được, giữ report đúng trước sự cố, mission_success=false. Không tự động chạy lại episode đó.
- Tail journal bị cắt: lưu byte hỏng riêng và truncate đúng tail; hỏng record hoàn chỉnh/hash-chain thì từ chối.

Policy/simulator có lifeline riêng: parent mất thì watcher kết thúc worker kể cả planner thread bị treo. PID chỉ dùng để xác minh worker cũ đã chết, không kill PID tùy ý trên resume. Đây vẫn không phải sandbox chống code đối địch đọc file.

Batch tạo `attempts.csv` (mọi attempt), `episodes.csv` (primary), `events.jsonl`, `trajectory.csv`, `summary.json`, `summary.md`. Các bảng là projection từ journal/result, tạo lại atomically, không append trùng khi resume. Journal là nguồn tiến độ chính trong khi episode còn chạy; CSV/summary cập nhật tại checkpoint hoàn tất/phục hồi. Recovery giữ raw log; global events đánh dấu worker end chưa commit và thêm recovery error/terminal riêng.

`summary.json` có mẫu số và các công thức t1-0.2, gắn `purpose=runner_acceptance_not_research`. Giá trị đang chạy/chưa biết hoặc mẫu số 0 là null. `summary.md` chỉ trình bày số đếm và end reasons. Các số diagnostic này không phải success rate nghiên cứu. SPL/GT coverage chưa hỗ trợ, không thay bằng số giả.

Replay dùng trajectory, toàn semantic events và result/end_reason; chỉ bỏ run_id, attempt_id, wall-time và trạng thái thu hồi process. Ví dụ từ `test version/`:

```powershell
python -B -m benchmark.replay "../benchmark-results/run-a/attempts/sample-s2/attempt-0001" "../benchmark-results/run-b/attempts/sample-s2/attempt-0001"
```

Khác CLI suite/output, đường replay đọc theo cwd. Không dùng tolerance để che khác biệt trajectory. Xem [nghiệm thu P1b.2](../../docs/benchmark-p1b2-audit.md) để biết phạm vi Windows/Linux thực chạy.

Test batch có thể xuất hồ sơ crash/resume (từ gốc repo, chọn đường export mới):

```powershell
$env:T1_BATCH_EXPORT = 'docs/local-batch-acceptance'
python -B -m unittest discover -s "test version" -p test_benchmark_batch.py -v
```

Giới hạn: chưa chứng minh durability khi mất điện/phần cứng lưu trữ lỗi; không sửa code/config lúc run hoạt động. Projection lỗi IO sẽ dừng batch, giữ reservation để resume sau khi xử lý IO. Source/runtime thay đổi làm resume bị từ chối; không chỉnh manifest để bỏ kiểm tra. Chưa triển khai so sánh baseline hay thống kê nghiên cứu.

## Hai selector chính thức — P2.1

- `b0-fixed-order-v2`, implementation version `2`: lưới neo tại pose đầu từ sensing, thứ tự vòng/góc cố định; unknown/out-of-window được xét lại; index đã giao không được giao lại. Khi hết waypoint đủ điều kiện, fallback sắp các ô chung trong frame ban đầu, không dùng gain, min-gain hoặc focus.
- `b1-gain-distance-focus-v1`, implementation version `2`: score, min-gain, focus và strict row-major tie từ production `SearchMission.choose_goal()`. Tests đối chiếu trực tiếp source production trên cùng miền hợp lệ, kể cả `now=None`.

`selectors.SelectionState` chỉ chứa pose/time/estimated coverage/blacklist/spun_at/focus công khai. `CandidateDomain` nhận costmap từ sensing, bounds và kích thước thân; không nhận fixture, GT, witness, certificate, evaluator hay Navigator. `GoalSelector.select(domain)` trả `Selection | None`; mission giữ spin/dwell, navigation, tiếp cận và reporting chung. `SelectorMission` dùng chung cho cả B0/B1, giữ stalled/retry tới deadline và vẫn nhận target xác nhận mới.

Miền `observed-standable-v1` áp dụng §5.2 trước scoring, dùng copy mask, không sửa grid của planner. So với `search-mission-smoke`, nó lọc footprint trước scoring và không lặp thao tác loại điểm rồi erosion lại. Đây là khác biệt adapter đã công bố; smoke không phải bản B1 chính thức. Source/config thay đổi đòi hỏi certificate và run mới. Mọi policy vẫn chạy Navigator async, barrier, monitor, oracle và evaluator hiện có.

Manifest lưu `policy_identity`: ID/version, algorithm/domain version, selector parameters, shared search parameters và source hashes. Batch khóa các trường này khi resume; replay so cả identity, semantic events và trajectory. Thay source/parameters không được resume vào run cũ hoặc tuyên bố replay tương đương. Default CLI vẫn là smoke để giữ tương thích; chọn official policy bằng `--policy`.

Ví dụ từ gốc repo, chỉ chọn dev S2 (output mới):

```powershell
python -B "test version/benchmark_run.py" --catalog "test version/benchmark/fixtures/catalog.json" --bundle "docs/p21-audit-logs/dev-final/certificates" --episode-id sample-s2 --output "benchmark-results/p21-b0-s2" --policy b0-fixed-order-v2
python -B docs/checks/p21-dev-controls.py --output "benchmark-results/p21-dev-new"
python -B -m unittest discover -s "test version" -p test_benchmark_selectors.py -v
```

Lệnh đầu cần bundle đúng source hiện hành, có thể tạo mới bằng CLI validator; không dùng archive lịch sử khi source đã đổi. Script dev kiểm tra **toàn catalog**, chỉ rollout dev S1–S5, chạy hai lần riêng từng policy rồi kiểm tra replay/resume. Đây là control nghiệm thu, không chạy held-out test hoặc xuất bảng so sánh thuật toán.

Nếu phiên tool chạy dài bị ngắt, có thể chia thành các batch CLI riêng với output mới. Script có `--finalize-existing --b1-repeat run-3` để kiểm tra các batch hoàn tất trong hồ sơ P2.1; nó từ chối batch thiếu result, không tự chạy bù hoặc sửa log dở. `controls.json` ghi rõ hai thư mục thực sự được dùng cho mỗi policy; xem audit để phân biệt lượt hoàn tất và lượt gián đoạn.

Log thêm `SELECTOR_DECISION` (point/kind/index và lý do skip), `WAYPOINT_OUTCOME` (reached/failed/preempted), `SELECTOR_STATUS`. Waypoint còn active khi episode chấm dứt được giữ là unresolved, không tự ghi reached. B0 `unresolved_waypoints` là tổng index lưới chưa có outcome reached, gồm chưa giao, failed, preempted và đang chạy; fallback không thêm index lưới. B1 không có lưới, trường này là null. `goals_selected` chỉ đếm goal khám phá của selector; LAST_SEEN/approach không tăng bộ đếm này. Stalled duration lấy từ state mission theo simulated time, kể cả khi monitor thay reason hiển thị. Kết quả kiểm tra metrics theo đợt monitor được ghi ở [audit P2.2](../../docs/benchmark-p22-audit.md).

## Metrics và paired runs — P2.2

`metrics.py` là reducer evaluator-side `t1-metrics-1`: đọc raw events/trajectory và resolved bytes đã bind với certificate, tái kiểm report, Euler, swept geometry, stop hold và vùng tiếp cận rồi đối chiếu result. Nó không lấy `found` hoặc cờ monitor làm kết quả chấm. Sai khác ghi FAIL; reducer không sửa kết quả cũ. Legacy batch projections vẫn giữ định dạng cũ; output reducer nằm riêng trong `paired/analysis/`.

`monitor_interventions` giữ số lần vào từng state ngoài PASS/IDLE/LIMIT. Extension `monitor-episodes-1` thêm `monitor_continuous_episodes`, số đổi state trong can thiệp, duration theo dt và per-state durations. Đổi SLOWDOWN→STOP_ZONE→SLOWDOWN là ba state entries nhưng một đợt liên tục. LIMIT là speed cap thông thường, không thuộc can thiệp theo hợp đồng hiện có. Nhãn none/limited/stopped chỉ mô tả monitor, không đo rủi ro vật lý.

Khi execution error/recovery làm thiếu trajectory, tổng path/monitor/goal/stall chưa xác minh là null; `*_observed` giữ prefix đã nhận. Không sửa raw tail khi đọc, không coi đoạn ngắn của một mission lỗi là lợi thế. SPL/GT coverage/min-clearance chưa được chứng nhận giữ null kèm lý do. B1 unresolved_waypoints là không áp dụng. Report đúng đã nhận trước crash vẫn được giữ, nhưng safe mission false.

`paired.py` khóa manifest `t1-paired-2` trước rollout: toàn corpus, resolved/shared/config/budget, sensing/Euler/planner/evaluator/metric sources, runtime và identity từng policy. Profile single-job-v1 dùng OS lock để chặn job/audit đồng thời trước cả proof recomputation; không sửa watchdog hoặc mission budget. Mỗi slot là một batch của một episode, dùng process isolation/journal/resume cũ. Không truyền manifest/reducer cho policy. Audit từ chối thiếu cặp, sai bindings, sửa completed result hoặc hỏng journal; reader không tự truncate/repair. Bảng paired/episodes chỉ dùng attempt 1, attempts giữ primary-run retries, all-attempts giữ thêm toàn bộ replay/error.

Từ gốc repo, chọn output mới:

```powershell
python -B docs/checks/p22-pilot.py prepare --output benchmark-results/p22-dev
python -B docs/checks/p22-pilot.py run --output benchmark-results/p22-dev --limit 2
# Lặp lệnh run tới ROLLOUTS_COMPLETE; không chạy hai driver tự chọn job cùng lúc.
python -B docs/checks/p22-pilot.py audit --output benchmark-results/p22-dev
node docs/checks/p22-evidence-check.cjs benchmark-results/p22-dev
```

`prepare` tạo/kiểm tra fixtures P2.2 có provenance; input đã tồn tại phải khớp byte, không ghi đè. Toàn catalog có dev/test đều qua validator trước chọn dev. Ba biến thể thêm là camera offset trên layout mới, S5 bounds nhỏ cùng family với empty layout, và S3 target ở nhánh kia trên cùng environment/family cũ. Chúng không tạo thêm family độc lập chỉ vì đổi start/target/seed.

Có thể chạy slot độc lập từ `test version/` để giới hạn thời gian một phiên tool:

```powershell
python -B -m benchmark.paired run --output benchmark-results/p22-dev/paired --policy b0-fixed-order-v2 --replica primary --episode-id sample-s1
```

Chạy tuần tự, không chạy tests hoặc pilot khác đồng thời trong phiên nghiệm thu. Profile khóa một slot/audit trong cùng paired run; không kiểm soát toàn bộ ứng dụng ngoài hoặc paired run khác. Output đã tồn tại cần `--resume`; thêm `--retry-errors` khi chủ động chạy retry, vẫn không thay primary. Nếu journal bị cắt, dùng batch resume qua lệnh job này, không sửa byte bằng tay. Manifest/config/source đổi thì cần run mới. Pair initialization chưa có chứng minh durability khi mất điện; manifest/checksum không đầy đủ phải bị từ chối.

Nếu START đã được journal nhưng chưa tạo attempt manifest, batch resume giữ primary CRASH/INTERRUPTED. Paired reducer dùng reservation identity đã khóa, giữ physics/telemetry null và gắn identity_basis rõ ràng; không viết manifest giả. Nếu có raw execution mà thiếu manifest thì BLOCKED. Reservation chưa chạy policy không được coi là replay PASS. Xem audit P2.2 về pilot schema 1 đã FAIL do một WALL_TIMEOUT và pilot schema 2 mới; không ghi đè hoặc trộn hai hồ sơ.

Reducer độc lập cũng dùng được cho archive đã bind với bundle tương ứng, không yêu cầu source verifier hiện hành trùng archive. Đây là phân tích lại snapshot, không phải quyền rollout bằng certificate cũ. Ví dụ từ `test version/`:

```powershell
python -B -m benchmark.metrics --attempt docs/p21-audit-logs/dev-final/b0-fixed-order-v2/run-1/attempts/sample-s1/attempt-0001 --bundle docs/p21-audit-logs/dev-final/certificates --output benchmark-results/p21-s1-reduced.json
```

Source/version reducer được ghi trong output mới; không overwrite. Paired pilot thì luôn yêu cầu certificate/source/runtime hiện hành. Counter B0 có work limit một triệu lattice candidates: vượt thì UNVERIFIED/BLOCKED, không ước lượng cardinality hoặc đổi scoring. Linux/cross-OS chỉ được ghi PASS sau khi thực chạy.
