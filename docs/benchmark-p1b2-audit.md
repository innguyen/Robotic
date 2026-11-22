# Nghiệm thu P1b.2 — Batch, Reproducibility, Journal và Resume

## Phạm vi và trạng thái

Nền commit `85d2eff`, nhánh `feat/benchmark`, protocol **t1-0.2**. Đã đọc [spec](benchmark-t1-spec.md), [P1b.1](benchmark-p1b1-audit.md) và source package benchmark. Giữ nguyên quy tắc chấm và các file production ROS/Navigator/SearchMission/Collision Monitor/vision/YAML/launcher.

**P1b.2 PASS trong phạm vi Windows đã thực chạy; Linux UNVERIFIED.** Batch nhiều episode, resume sau sự cố, worker lifeline và replay đã có log thực. Không triển khai B0/B1 chính thức hoặc công bố success rate nghiên cứu. Các tỷ lệ trong summary.json chỉ phục vụ kiểm tra công thức t1-0.2 và gắn `purpose=runner_acceptance_not_research`; summary.md chỉ trình bày counts/end reasons.

## Source và kiến trúc

| File mới/chỉnh sửa                                                   | Vai trò                                                                                         |
| -------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| [batch.py](../test%20version/benchmark/batch.py)                     | Full-corpus gate, selection, batch identity, attempt reservation, recovery/retry và projections |
| [journal.py](../test%20version/benchmark/journal.py)                 | Hash-chain journal, fsync, tail repair, OS lock và atomic replace có retry giới hạn             |
| [lifecycle.py](../test%20version/benchmark/lifecycle.py)             | Parent lifeline và kiểm tra PID còn sống bằng API không gửi tín hiệu trên Windows               |
| [replay.py](../test%20version/benchmark/replay.py)                   | So sánh strict semantic events, trajectory và result/end_reason                                 |
| [runner.py](../test%20version/benchmark/runner.py)                   | Tách verified corpus khỏi single execution; attempt/run IDs; lifeline và critical-event fsync   |
| [policy.py](../test%20version/benchmark/policy.py)                   | Watch parent qua kênh control riêng, không thêm GT vào policy args                              |
| [coverage.py](../test%20version/benchmark/coverage.py)               | Vectorize costmap ray lookup/early-stop, có test đối chiếu toàn cell set với định nghĩa scalar  |
| [benchmark_batch.py](../test%20version/benchmark_batch.py)           | CLI batch, resume/retry và filters                                                              |
| [test_benchmark_batch.py](../test%20version/test_benchmark_batch.py) | 17 tests batch, kill thật, recovery, retry, locks, replay và projections                        |
| [p1b2-evidence-check.cjs](checks/p1b2-evidence-check.cjs)            | Kiểm tra độc lập source/hash, journal-chain, log uniqueness, replay và kill evidence            |
| [README.md](../test%20version/benchmark/README.md)                   | Hướng dẫn chạy, dữ liệu output và giới hạn                                                      |

Luồng batch: verify toàn catalog và mọi certificate PASS → lọc episode → khóa manifest/runtime/source identity → OS lock → journal START → single runner với hai worker spawn → atomic result → journal FINISH → tạo projections. Mỗi episode reset các core và sensor như P1b.1, không chia sẻ state mission giữa các lượt.

Policy chỉ nhận public config và SensorFrame/Observation bất biến. Catalog/GT/witness/certificate/journal thuộc evaluator/supervisor. Lifeline là kênh control EOF, không truyền dữ liệu môi trường. Đây là ranh giới cho code hợp tác, không phải filesystem sandbox chống policy đối địch.

## Journal, crash và resume

Journal dùng sequence, previous checksum và SHA-256 record. START được fsync trước chạy. FINISH tham chiếu hash của result/manifest/events/trajectory và recovery/worker records có sẵn. Không lặp reservation hoặc FINISH. Một supervisor giữ OS file lock; kill process tự nhả lock.

Manifest khởi tạo trong staging rồi rename cả directory; resume không thấy một output đã công bố mà thiếu manifest do ghi nửa chừng. File projections/result dùng temp → fsync → replace; Windows transient sharing violation được retry tối đa 2 giây, không xóa file đích. IO lỗi kéo dài dừng batch và giữ reservation để phục hồi.

| Trạng thái khi resume                                | Xử lý                                                                                      |
| ---------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| FINISH và hashes hợp lệ                              | Giữ nguyên file attempt; bỏ qua episode, không chạy trùng                                  |
| Atomic result có sẵn, FINISH chưa ghi                | Kiểm tra run/episode/attempt identity, nhận lại result, không overwrite                    |
| START nhưng chưa có result                           | Chốt CRASH hoặc INTERRUPTED từ journal/log; giữ report đúng đã nhận; mission_success=false |
| Journal tail không có newline đầy đủ                 | Lưu tail ra file riêng rồi truncate đúng vị trí cuối hợp lệ                                |
| Record đầy đủ nhưng sai chain/hash                   | Từ chối; không tự chữa corruption thành PASS                                               |
| Source/config/evidence/runtime/policy/selection khác | Từ chối resume; cần run mới                                                                |
| Worker cũ còn sống                                   | Chờ lifeline có giới hạn rồi từ chối nếu chưa xác nhận chết; không kill PID tùy ý          |

Recovery đọc confirmed observations và report nhận được, có thể chấm lại report nếu chưa nhận REPORT_EVALUATED. Thời gian/path partial lấy từ các record đã ghi; phần chưa biết là null, trajectory_complete=false. Raw events/trajectory được giữ nguyên. Global projection đổi terminal chưa commit thành UNCOMMITTED_EPISODE_END và thêm recovery error/end rõ ràng, không giả là mission thành công.

`--resume` không tự chạy lại attempt lỗi. `--retry-errors` chỉ dùng cùng resume và tạo attempt mới cho CRASH/PLANNER_TIMEOUT/WALL_TIMEOUT/INTERRUPTED/INVALID_STATE, không retry lỗi nhiệm vụ TIMEOUT/COLLISION/false report. Attempt 1 luôn là primary; retry thành công không thay primary thất bại. Test đã chạy đúng trường hợp retry thành công sau crash sau report.

## File output

- `run_manifest.json`: protocol, SHA/dirty, registry environment/hash/family/split, config hashes, runtime và identity bất biến.
- `journal.jsonl`: tiến độ canonical, START/FINISH/INTERRUPT có chain; hash-chain phát hiện corruption, không phải chữ ký chống người sửa có chủ đích.
- `attempts/<episode>/attempt-XXXX/`: log/result single episode, worker PID metadata, recovery marker khi có.
- `attempts.csv`: mọi attempt; `episodes.csv`: primary của mỗi episode đã bắt đầu.
- `events.jsonl`, `trajectory.csv`: projections có run/episode/attempt identity; không append trùng khi resume.
- `summary.json`, `summary.md`: theo scenario/tổng, counts, mẫu số, runtime errors và end reasons; rates diagnostic có null khi mẫu số 0/chưa chốt. SPL/GT coverage chưa hỗ trợ giữ null với lý do.

Journal là nguồn tiến độ khi worker đang chạy; các projection cập nhật ở checkpoint hoàn tất/phục hồi. Mỗi file được replace atomically, chưa có transaction đa file cho người đọc đồng thời. Sau IO lỗi, resume tái tạo projections; chỉ dùng kết quả sau checkpoint hợp lệ, đối chiếu summary.journal_head với journal.

## Lệnh tái hiện

Từ `test version/`, chọn output mới:

```powershell
python -B -m benchmark --catalog "test version/benchmark/fixtures/catalog.json" --output "benchmark-results/p1b2-certificates"
python -B benchmark_batch.py --catalog "test version/benchmark/fixtures/catalog.json" --bundle "benchmark-results/p1b2-certificates" --output "benchmark-results/p1b2-run" --episode-id sample-s2 --episode-id sample-s3 --episode-id sample-s4
```

Tiếp tục bằng cùng lệnh thêm `--resume`; retry lỗi thực thi thêm `--resume --retry-errors`. Full corpus luôn được xác minh trước filter `--episode-id`, `--seed` (lọc, không override), `--max-episodes` (prefix sau lọc). Batch exit 0 khi chỉ có kết quả nhiệm vụ, exit 1 nếu có lỗi thực thi primary, exit 130 khi interrupt, exit 2 khi input/resume/IO không hợp lệ.

Từ gốc repo:

```powershell
python -B -m unittest discover -s "test version" -p test_benchmark_batch.py -v
node docs/checks/p1b2-evidence-check.cjs
```

Test có thể lưu hồ sơ bằng biến `T1_BATCH_EXPORT` tới output mới; fixtures acceptance ngắn sinh từ test source, không phải corpus đánh giá. Artifact lưu sau test được di chuyển khỏi đường temporary gốc; muốn tái hiện test phải chạy test source, không sửa manifest để resume các archive đã di chuyển.

## Bằng chứng thực chạy

| Kiểm tra                        | Kết quả                                                                             | Bằng chứng                                                                                                                    |
| ------------------------------- | ----------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| Batch/journal/resume acceptance | PASS 17 tests, không skip                                                           | [batch-tests.txt](p1b2-audit-logs/batch-tests.txt)                                                                            |
| P1a + P1b.1                     | PASS 67 tests, không skip                                                           | [prior-acceptance.txt](p1b2-audit-logs/prior-acceptance.txt)                                                                  |
| Production regression           | PASS 96 tests, không skip                                                           | [regression.txt](p1b2-audit-logs/regression.txt)                                                                              |
| Contract / dashboard            | PASS 24 tests / 13 checks                                                           | [contract.txt](p1b2-audit-logs/contract.txt), [dashboard.txt](p1b2-audit-logs/dashboard.txt)                                  |
| Batch integration S2/S3/S4      | Hoàn tất ba episode với search-mission-smoke, mỗi result SUCCESS_REPORT             | [verified-batch-cli.txt](p1b2-audit-logs/verified-batch-cli.txt), [attempts.csv](p1b2-audit-logs/verified-batch/attempts.csv) |
| Deterministic replay Windows    | PASS strict trajectory/semantic events/result cả S2/S3/S4                           | [replay.json](p1b2-audit-logs/replay.json)                                                                                    |
| Resume batch hoàn tất           | Không đổi 15 file attempt hoặc byte journal; không thêm episode/attempt             | [resume-check.json](p1b2-audit-logs/resume-check.json)                                                                        |
| Kill supervisor thật            | Ba checkpoint; gồm worker đã bootstrap, cả hai worker không còn sống sau kill       | [kill-evidence.json](p1b2-audit-logs/acceptance-verified/kill-evidence.json)                                                  |
| Kiểm tra độc lập bằng Node      | PASS source/artifact hash, chain, counts, event uniqueness, replay và kill evidence | [independent-check.json](p1b2-audit-logs/independent-check.json)                                                              |
| Windows                         | Đã thực chạy Python 3.10.11, NumPy 2.2.6, PyYAML 6.0.3                              | [environment.json](p1b2-audit-logs/environment.json)                                                                          |
| Linux                           | **UNVERIFIED**: WSL docker-desktop không có Python; Docker daemon không sẵn         | [environment.json](p1b2-audit-logs/environment.json)                                                                          |

Lượt Windows ban đầu gặp PermissionError khi thay trajectory.csv; giữ [log lỗi](p1b2-audit-logs/batch-tests-windows-lock.txt), đã sửa bằng bounded retry và có regression test tiêm transient lock. [Log đầu](p1b2-audit-logs/batch-tests-initial.txt) còn có bootstrap traceback do kill trước worker ready; test cuối đã đổi sang checkpoint running để chứng minh lifeline sau bootstrap, không dùng lượt đầu làm bằng chứng đó. Dòng YAML error trong log test cuối là đối chứng CLI trả exit 2 trước khi tạo output, không phải test failure. [Source-state](p1b2-audit-logs/source-state.txt) ghi nhận production giữ nguyên.

Hồ sơ chính là [verified-batch/summary.md](p1b2-audit-logs/verified-batch/summary.md), [verified-repeat/summary.md](p1b2-audit-logs/verified-repeat/summary.md), [verified-certificates/manifest.json](p1b2-audit-logs/verified-certificates/manifest.json) và [acceptance-verified/](p1b2-audit-logs/acceptance-verified/). Các thư mục integration/integration-replay/certificates/acceptance trước đó chỉ là hồ sơ phát triển. S2/S3/S4 là integration của search/navigation, không phải ước lượng năng lực tổng quát của policy; nhiệm vụ thất bại với end_reason hợp lệ sẽ được phân biệt với lỗi runner, không ép mọi scenario thành công để test qua.

## Giới hạn và cổng P2

- Linux/cross-OS replay **UNVERIFIED**; Docker runtime **BLOCKED trong môi trường hiện có**, không tự cài hay khởi động dịch vụ để tuyên bố PASS.
- Chưa kiểm chứng mất điện hoặc lỗi lưu trữ/kernel; fsync/rename không thay thế thử nghiệm power-loss. Windows file lock kéo dài có thể chặn projection; journal cho phép resume sau khi xử lý nguyên nhân IO.
- Interrupt được kiểm bằng KeyboardInterrupt tại checkpoint và kill process thật; chưa thử mọi biến thể console signal. Projections đang dùng bộ nhớ và tái dựng tuần tự, chưa nghiệm thu corpus rất lớn.
- Không sửa source/config trong run đang hoạt động. Fingerprints khóa lúc bắt đầu và xác minh lại trên resume; đây không phải sandbox chống tiến trình đối địch sửa filesystem.
- Chưa có B0/B1 chính thức, test suite nghiên cứu đã khóa, thống kê nghiên cứu, GT coverage hoặc SPL. Chưa chạy ROS/Linux robot/JetAuto thật.
- Replay đã xác nhận trên các fixture và runtime Windows nêu trên; không suy rộng cho mọi corpus, nhiễu hoặc phiên bản NumPy/Python khác. Rates diagnostic không dùng làm kết quả nghiên cứu.

Đề xuất bước tiếp theo là [kế hoạch P2](benchmark-p2-plan.md): chuẩn hóa interface selector, hiện thực B0 fixed-order-v2 và B1 có version, positive controls dev, metric tests tính tay rồi pilot paired trước khi khóa test. Không mở comparison chính thức chỉ vì runner smoke đã chạy thành công.
