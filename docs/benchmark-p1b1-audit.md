# Nghiệm thu P1b.1 — Single-Episode Headless Runner

## Phạm vi và kết luận

Triển khai trên `feat/benchmark`, nền commit `3c8ca5fdcb0b5eaf3fd83e648c32b5827a37626b`, theo [protocol t1-0.2](benchmark-t1-spec.md), [P1a.2](benchmark-p1a2-audit.md) và source hiện có. Không thay quy tắc chấm hoặc defaults production. Không chạy ROS, SSH, camera hay robot thật.

**P1b.1 PASS trong phạm vi single-episode đã kiểm thử.** Đã chạy episode `sample-s1` qua simulator → oracle → SearchMission/Navigator async → Collision Monitor → evaluator → logging. Kết thúc thực tế `SUCCESS_REPORT`; có di chuyển, có planner barrier, có xác nhận dừng. Đây là smoke integration, không phải B0/B1 comparison hay bằng chứng hiệu năng cứu hộ.

Bundle P1a.2 hash toàn bộ file Python trong package benchmark. Thêm runner làm bundle cũ stale dù protocol không đổi. Runner giữ kiểm tra fail-closed, tái tạo certificate bằng cùng verifier và cùng fixture; không sửa nhãn PASS hoặc bỏ hash check. Bundle dùng trong lượt cuối là [verified-bundle/manifest.json](p1b1-audit-logs/verified-bundle/manifest.json).

## Source và kiến trúc

| File mới/chỉnh sửa | Vai trò |
| --- | --- |
| [benchmark_run.py](../test%20version/benchmark_run.py) | Entry point chạy đúng một episode, guarded main |
| [runner.py](../test%20version/benchmark/runner.py) | Gate certificate, supervisor, state machine simulator, clock, evaluator integration, log/result và CLI |
| [policy.py](../test%20version/benchmark/policy.py) | Spawned policy worker, SensorFrame frozen, MissionAdapter, planner barrier; policy smoke/test, không B0/B1 |
| [simulator.py](../test%20version/benchmark/simulator.py) | World evaluator-owned, LiDAR/AABB gồm box người; bounds không tự sinh tường |
| [coverage.py](../test%20version/benchmark/coverage.py) | Estimated camera coverage từ costmap, có extrinsics/FOV/range, dừng tại unknown/occupied |
| [evaluator.py](../test%20version/benchmark/evaluator.py) | Matching/validity/freshness/frame, vùng A, swept collision/bounds và kết thúc báo cáo |
| [test_benchmark_runner.py](../test%20version/test_benchmark_runner.py) | 16 test methods, gồm acceptance chạy process thật, helper tests và certificate rejection |
| [p1b1-evidence-check.cjs](checks/p1b1-evidence-check.cjs) | Kiểm chứng độc lập source/hash/log/Euler của các artifact nghiệm thu |
| [README.md](../test%20version/benchmark/README.md) | Cập nhật dependencies, lệnh validation/runner, API, giới hạn P1b.1 |

Không sửa Navigator, SearchMission, vision, Collision Monitor, YAML hoặc launcher production. [Bằng chứng source-state](p1b1-audit-logs/source-state.txt).

Supervisor sở hữu **hai process spawn riêng**: simulator/evaluator có GT; policy có Navigator/SearchMission nhưng chỉ nhận public config và SensorFrame. Frame chứa tick, pose noiseless, tuple scan, immutable Observation và trạng thái monitor; không có episode dictionary, GT obstacles/targets, witness hoặc certificate. Report matching chỉ được ghi ở evaluator/log, không phản hồi cho policy. Sau report đầu, policy không còn được gọi.

Spawn tránh kế thừa heap GT qua fork. Đây là ranh giới truyền dữ liệu cho code hợp tác, chưa là sandbox chống policy cố ý đọc file hoặc import dữ liệu từ repository. Các artifact evaluator có GT không được đưa vào API policy.

## Hành vi đã hiện thực

1. Load và validate **toàn corpus catalog** trước chọn episode. Recompute bundle bằng source/config/evidence hiện hành, đối chiếu mọi byte/file; kiểm tra certificate episode là PASS. Missing/tampered/stale/UNVERIFIED bị từ chối trước tạo output hoặc worker. Không có filter né registry.
2. Mỗi episode tạo Navigator async, SearchMission adapter, costmap, oracle và monitor mới. Adapter chỉ đổi estimated coverage, miền goal trong bounds và cách xử lý hết goal thành stalled/retry theo dwell; production không sửa. Tên `search-mission-smoke` không đại diện B1 chính thức đã nghiệm thu.
3. Clock nhiệm vụ là `step_index * 0.1`, không cộng dồn dt hoặc dùng wall-clock. Cảm biến trước policy, barrier trước monitor, Euler trước independent geometry, stop counter trước terminal/deadline. LiDAR 720 beam, range 0,05–6 m, extrinsics/blind sectors từ config; mô hình được ghi manifest.
4. Coverage dùng camera origin/yaw thật trên bản đồ **đã quan sát**, không dùng GT raycast. Unknown cắt ray; không coi coverage này là GT coverage. Tất cả policy worker đi qua cùng cập nhật sensing/coverage.
5. `SEARCHING → REPORT_SUBMITTED → VERIFY_STOP → FINISHED`. Runner gắn lúc nhận, đóng băng report, gửi yêu cầu 0 qua monitor và vẫn tích phân/kiểm tra hình học. Không teleport pose hoặc ép vận tốc log về 0. Ba interval liên tiếp đủ chậm mới verified; vi phạm reset counter. Hold tính vào budget.
6. Evaluator kiểm tra structure, frame, class, confirmed observation ID/freshness và Euclidean tolerance, không cộng geometry epsilon vào r. Kiểm tra A gồm footprint, heading, standoff, clearance và visibility thật ở pose cuối. Collision/bounds kiểm tra cả interval và độc lập monitor; uncertainty hình học bị xử lý bảo thủ, có trạng thái kiểm tra trong event.
7. Collision ưu tiên trước success, bounds trước stop result; nonfinite state không được coi an toàn. Stop hoàn tất đúng deadline được nhận; deadline trước đủ hold là TIMEOUT. Giữ report_correct khi crash/collision/timeout sau report đúng; mission_success chỉ khi SUCCESS_REPORT.
8. Policy chờ future có `planner_wall_timeout_s`; core nhận kết quả ở lần kiểm tra tick tiếp theo. Planner exception là CRASH, future treo là PLANNER_TIMEOUT. Parent áp `episode_wall_timeout_s`, sở hữu/terminate/kill/join cả hai worker, kể cả khi `Navigator.close()` chờ executor treo.

Wall-clock chỉ phục vụ watchdog/chi phí thực thi. Không có noise ngẫu nhiên trong model này; chưa tuyên bố replay toàn suite deterministic trước khi kiểm chứng thêm ở P1b.2.

## Lệnh chạy một episode

Python thực dùng `C:\Users\lemin\AppData\Local\Programs\Python\Python310\python.exe` (3.10.11), NumPy 2.2.6, PyYAML 6.0.3 và Git. Không cài dependency mới. Từ `test version/`:

```powershell
python -B -m benchmark --catalog "test version/benchmark/fixtures/catalog.json" --output "benchmark-results/p1b1-certificates"
python -B benchmark_run.py --catalog "test version/benchmark/fixtures/catalog.json" --bundle "benchmark-results/p1b1-certificates" --episode-id sample-s1 --output "benchmark-results/p1b1-single"
```

Đường dẫn luôn relative gốc repo, không theo cwd; chọn output chưa tồn tại. Runner exit 0 khi ghi kết quả mission bình thường, kể cả TIMEOUT/COLLISION/false report; exit 1 khi lỗi thực thi, exit 2 khi input/certificate/output không hợp lệ; interrupt trả 130. Không có batch/resume hoặc ghi đè run cũ.

## Kết quả thực chạy và log

| Kiểm tra | Kết quả thực tế | Bằng chứng |
| --- | --- | --- |
| Acceptance/unit runner | PASS 16 test methods, không skip; có 20 episode chạy simulator thật | [acceptance.txt](p1b1-audit-logs/acceptance.txt), [tóm tắt kiểm tra độc lập](p1b1-audit-logs/independent-check.json) |
| Unit P1a | PASS 51 tests, không skip | [validation-unit.txt](p1b1-audit-logs/validation-unit.txt) |
| Regression production | PASS 96 tests, không skip | [regression.txt](p1b1-audit-logs/regression.txt) |
| Contract / dashboard | PASS 24 tests / 13 checks | [contract.txt](p1b1-audit-logs/contract.txt), [dashboard.txt](p1b1-audit-logs/dashboard.txt) |
| CLI default mission thật | SUCCESS_REPORT, report_correct=true, stop_verified=true; worker đã thu hồi | [cli-current.txt](p1b1-audit-logs/cli-current.txt), [result.json](p1b1-audit-logs/verified-episode/result.json) |
| Kiểm tra log độc lập bằng Node | PASS event sequence/time, end_reason và worker cleanup; Euler trajectory; source/certificate hash hiện hành | [independent-check.json](p1b1-audit-logs/independent-check.json) |

Lượt CLI cuối bắt đầu ở pose fixture với mục tiêu cách 2 m; log có 81 interval, 76 interval tịnh tiến và 13 lần barrier planner. Các con số này xác minh vòng lặp/di chuyển/planner thực sự đã chạy, **không dùng để so sánh chất lượng policy**. Oracle cho vị trí chính xác theo thiết kế; position_error=0 không phải kết quả camera thật.

Log episode cuối:

- [run_manifest.json](p1b1-audit-logs/verified-episode/run_manifest.json): config/source/bundle hashes, SHA/dirty, seed, policy và sensor/planner mode.
- [events.jsonl](p1b1-audit-logs/verified-episode/events.jsonl): observation, mission/goal, report, matching, stop và terminal có sequence/timestamp.
- [trajectory.csv](p1b1-audit-logs/verified-episode/trajectory.csv): pose GT cuối interval, command requested/monitor/executed, trạng thái monitor và runner.
- [result.json](p1b1-audit-logs/verified-episode/result.json): kết quả một episode, report, thời điểm sự kiện, tình trạng trajectory và worker cleanup.

[acceptance-current/](p1b1-audit-logs/acceptance-current/) giữ manifest/events/trajectory/result của từng ca; bảng `cases` trong independent-check.json nối thư mục run với policy/fault/end_reason. Fixture acceptance ngắn được sinh xác định trong test source và cleanup sau test, không phải suite đánh giá. Các thư mục `acceptance-runs/`, `acceptance-final/`, `s1-smoke/`, `single-episode/` và các bundle trước đó là hồ sơ phát triển. Dùng `acceptance-current/`, `verified-episode/`, `verified-bundle/` cho nghiệm thu source cuối.

## Đối chứng và phạm vi PASS

| Trường hợp | Bằng chứng/điều kiện kiểm tra |
| --- | --- |
| Có người, mission thật | SearchMission/Navigator/Monitor chạy, báo đúng, đủ A/stop, SUCCESS_REPORT |
| Không người, mission thật | Không phát report; TIMEOUT; không cộng vào thành công dương tính |
| Có người nhưng test policy đứng yên | TIMEOUT dù đã có confirmed observation |
| Invalid/false report, cảnh trống báo sai | INVALID_REPORT/FALSE_REPORT; không sửa bằng report sau; giữ false-report count |
| Sai vùng tiếp cận | Report đúng nhưng APPROACH_NOT_SATISFIED |
| Matching biên | Đúng bằng r nhận, vượt r loại; thiếu observation ID không hợp lệ |
| Camera offset/coverage | Mission có offset thành công; coverage khác đúng origin/yaw, không vượt ô unknown/occupied |
| Collision và bounds | Fault actuator sau monitor được ghi requested/monitor/executed riêng; geometry phát hiện COLLISION/OUT_OF_BOUNDS |
| Đúng report cùng interval collision | report_correct=true, mission_success=false; collision không bị success ghi đè |
| Dừng/deadline | Pulse chuyển động reset counter; drift liên tục STOP_NOT_VERIFIED; hold đúng deadline nhận, thiếu hold TIMEOUT |
| Crash sau report | Report đúng còn trong result, mission thất bại, trajectory_complete=false |
| Planner exception/treo | CRASH / PLANNER_TIMEOUT; future/executor bị chặn vẫn thu hồi process |
| Policy treo | WALL_TIMEOUT từ supervisor; clock nhiệm vụ không chạy theo thời gian thực |
| Certificate sai/stale | Gate từ chối tamper hoặc evidence đổi trước khi chạy |

Fault actuator là phép thử evaluator trong harness: monitor vẫn được gọi và log, sau đó model actuator cố ý thực thi khác command đã kiểm tra. Các run này vi phạm giả định actuator danh định, không phải rollout T1 hợp lệ để đánh giá policy hoặc bằng chứng monitor production không an toàn. Fault không có trên CLI; chỉ API test dùng, và luôn ghi trong manifest.

Không có test FAIL/BLOCKED ở lượt cuối. Log đầu có ResourceWarning do test mở CSV thiếu context manager; đã sửa test và chạy lại, log cuối không còn warning. Không có thay đổi thuật toán production để làm test qua.

Kiểm tra độc lập có thể chạy lại từ gốc repo bằng `node docs/checks/p1b1-evidence-check.cjs`. Một test CLI cố ý phát sinh `malformed test YAML` và xác nhận exit 2; dòng error đó trong raw test log không phải test failure.

## Giới hạn và P1b.2

- Chưa có full journal, resume/retry, attempts.csv/episodes.csv hay summary thống kê. Parent flush log và lưu result temp→replace; supervisor bị kill/mất điện vẫn có thể để output dở, chưa có khả năng khôi phục tự động. `trajectory_complete=false` hoặc null giữ phần chưa biết.
- Chưa có baseline B0/B1 chính thức, positive controls so sánh, batch hoặc bộ test đã khóa. Policy smoke chỉ kiểm tra tích hợp; test policies không có giá trị xếp hạng.
- Process isolation bảo vệ ranh giới dữ liệu, không chống code đối địch đọc file; chưa áp filesystem sandbox. Worker lỗi trước integration có thể có trajectory.csv rỗng.
- Coverage là ước lượng từ observed costmap, không phải visibility GT hay mô hình RGB thực; geometry vẫn có giới hạn số học/P1a.2. Collision UNKNOWN được xử lý bảo thủ và ghi rõ. Chưa mô phỏng dynamics, trượt bánh hoặc phần cứng.
- Chưa chạy replay ngữ nghĩa toàn suite hoặc test supervisor bị kill thật; negative watchdog tests là policy/planner treo có kiểm soát. Chưa kiểm thử ROS/Ubuntu/JetAuto trực tiếp.

P1b.2 tiếp theo: hoàn thiện schema log/journal và khôi phục output dở; kiểm tra replay với cùng fixture/config/seed và bỏ riêng wall-time/run IDs; mở rộng fault/cleanup tới supervisor/IO; nghiệm thu lifecycle và public policy interface trước batch. Chỉ sau runner acceptance mới xây B0/B1 chính thức và positive controls dev; chưa mở benchmark comparison từ kết quả P1b.1.
