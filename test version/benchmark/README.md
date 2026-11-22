# T1 offline — P1b.2

Protocol: `t1-0.2`, verifier `p1a2-1`. Đã có validator, evaluator và single-episode headless runner P1b.1 và batch/journal/resume P1b.2; chưa có baseline B0/B1 chính thức hoặc bảng so sánh. Không khởi động ROS, camera, chassis hay launcher. Xem [đặc tả](../../docs/benchmark-t1-spec.md), [P1a.1](../../docs/benchmark-p1a1-audit.md), [P1a.2](../../docs/benchmark-p1a2-audit.md) và [nghiệm thu P1b.1](../../docs/benchmark-p1b1-audit.md).

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

Chạy lại cùng lệnh, thêm `--resume` để tiếp tục. Giữ nguyên selection, policy, config, corpus và runtime. `--retry-errors` chỉ đi với resume và chỉ tạo attempt mới cho lỗi thực thi; không retry TIMEOUT/COLLISION/false report của nhiệm vụ. Attempt 1 luôn là primary. Có thể lọc `--seed N` (không đổi seed), `--max-episodes N` hoặc nhiều `--episode-id`; mọi fixture trong catalog vẫn phải được kiểm tra và PASS **trước** lọc. Không có policy B0/B1 chính thức.

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
