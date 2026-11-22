# Nghiệm thu P1a.1 — Schema, layout, geometry và oracle

## Phạm vi

Thực hiện ngày 22/11/2026, nhánh `feat/benchmark`, nền commit `48d67cdcb8bc97fbd2bb18607f021212fc0244e2`. Đã đối chiếu P0 audit, spec t1-0.2, report và plan. Không đổi quy tắc chấm hoặc mặc định production. Đây là kiểm thử validator/fixture, không phải benchmark hoặc so sánh B0/B1.

Spec yêu cầu SearchConfig đã resolve nhưng schema hiện không có đường YAML search riêng. Triển khai snapshot defaults bằng AST từ source hiện có và hash source, không import runtime hoặc âm thầm thay thế YAML bị thiếu. Các giá trị YAML được nạp đầy đủ, gồm metadata ROS nhưng không dùng để khởi động ROS. Thay đổi cách cấu hình SearchConfig trong tương lai cần rà hợp đồng riêng.

Không phát hiện mâu thuẫn cần thay protocol. Các trường hợp chưa có chứng minh liên tục, đặc biệt S3/S4 và S2 camera offset, giữ UNVERIFIED theo spec. Plan v0.1/P0 là lịch sử; không xem trạng thái “chưa có verifier” trong đó như kết quả của phiên này.

## File và kiến trúc

Mã mới nằm dưới `test version/benchmark/`, không sửa module production:

| File | Vai trò |
| --- | --- |
| [__init__.py](../test%20version/benchmark/__init__.py) | Package offline |
| [schema.py](../test%20version/benchmark/schema.py) | JSON/YAML strict, types/fields/ID/frame/budget/config/path, start và target geometry |
| [layout.py](../test%20version/benchmark/layout.py) | rect-union-um-v1, hash, registry toàn corpus và duplicate |
| [geometry.py](../test%20version/benchmark/geometry.py) | Oriented footprint, SAT, clearance, bounds, ray và sweep có cận bảo thủ |
| [oracle.py](../test%20version/benchmark/oracle.py) | Extrinsics, FOV/range, occlusion, confirmation/memory, frozen Observation |
| [scenarios.py](../test%20version/benchmark/scenarios.py) | F/A và chứng nhận scenario bằng witness; interface UNVERIFIED S3/S4 |
| [__main__.py](../test%20version/benchmark/__main__.py) | CLI validation-only, nhiều suite trước mọi chứng nhận |
| [samples.json](../test%20version/benchmark/fixtures/samples.json) | Ba fixture dev S1/S2/S5 |
| [witnesses.json](../test%20version/benchmark/fixtures/witnesses.json) | Đường chứng evaluator, không cấp policy |
| [layout-golden.json](../test%20version/benchmark/fixtures/layout-golden.json) | Sáu golden vectors tạo từ JS reference P0, gồm gần giới hạn safe integer |
| [test_benchmark.py](../test%20version/test_benchmark.py) | Unit/negative/cross-language tests |
| [README.md](../test%20version/benchmark/README.md) | API, dependencies, lệnh chạy và giới hạn |

Luồng dữ liệu: JSON → schema/config → registry toàn corpus → geometry/oracle → certificate. Chỉ frozen Observation được thiết kế để truyền tới policy; chưa có policy hoặc runner để truy cập GT. Các file log nghiệm thu nằm trong `docs/p1a1-audit-logs/`, liên kết bên dưới.

## Kiểm thử thực chạy

Các lệnh đầy đủ và hướng dẫn Python không ROS ở [README](../test%20version/benchmark/README.md). Python dùng executable 3.10.11 đã xác minh ở P0; không cài dependency mới.

```powershell
python -B -m unittest discover -s "test version" -p test_benchmark.py -v
python -B -m unittest discover -s "test version" -p "test_*ros1.py" -v
node --test docs/checks/benchmark-contract.test.cjs
node "test version/test_navigation_status.js"
```

Hai lệnh sau chạy từ `test version/`:

```powershell
python -B -m unittest -v test_simulation test_vision_tracking
python -B -m benchmark --suite "test version/benchmark/fixtures/samples.json" --witnesses "test version/benchmark/fixtures/witnesses.json"
```

| Kiểm tra | Kết quả | Bằng chứng |
| --- | --- | --- |
| Unit P1a.1 | PASS 30 test, không skip, exit 0 | [unit.txt](p1a1-audit-logs/unit.txt) |
| Regression core/search/navigation/monitor/launcher | PASS 80 test, không skip, exit 0 | [regression-core.txt](p1a1-audit-logs/regression-core.txt) |
| Regression simulation/vision | PASS 16 test, không skip, exit 0 | [regression-simulation-vision.txt](p1a1-audit-logs/regression-simulation-vision.txt) |
| Contract P0 | PASS 24 test, exit 0 | [contract.txt](p1a1-audit-logs/contract.txt) |
| Dashboard | PASS 13 checks, exit 0 | [dashboard.txt](p1a1-audit-logs/dashboard.txt) |
| CLI ba fixture có witness | PASS S1/S2/S5, exit 0 | [certificates.json](p1a1-audit-logs/certificates.json) |
| CLI thiếu witness | S1/S2 UNVERIFIED, S5 PASS; exit 2 đúng yêu cầu | [unverified.json](p1a1-audit-logs/unverified.json) |
| Production diff và tài liệu mới | Git diff production exit 0; mọi local link mới tồn tại | [source-state.txt](p1a1-audit-logs/source-state.txt) |

Golden test so cả canonical bytes và SHA Python với vectors đã lưu, đồng thời gọi JS reference hiện tại để kiểm tra chéo. Negative tests gồm bool giả số, key lạ/trùng, NaN, path traversal/symlink, budget/frame/ID sai, target/start va chạm, split/family/alias/duplicate, occlusion và memory biên. Sweep có đối chứng vật cản mỏng giữa endpoint, quay quét góc và giới hạn depth trả UNVERIFIED.

## Trạng thái yêu cầu và giới hạn

| Yêu cầu | Trạng thái và phạm vi |
| --- | --- |
| Schema/config/path/layout registry | PASS trong tests đã chạy; caller phải đưa đủ corpus vào validate_corpus, không có catalog tự khám phá mọi suite trên ổ đĩa |
| Hash tương thích JS | PASS sáu vectors và kiểm tra chéo Node; không phải chứng minh mọi đầu vào binary64 |
| Geometry/oracle/F/A | PASS các test đã nêu; binary64, chưa có interval arithmetic cho dữ liệu suy biến |
| S1/S2/S5 | PASS ba mẫu và đối chứng; witness yaw cố định, S2 camera centered; không phải mọi fixture cùng nhãn đều được hỗ trợ |
| S3/S4 | PASS interface/negative tests; chứng nhận scenario **UNVERIFIED**, không có PASS giả |
| Policy GT boundary | PASS frozen DTO và kiểm tra không import runtime; cách ly policy process chưa thuộc P1a.1 |
| Production regression | PASS 96 test; không kết luận phần cứng/ROS thật được kiểm chứng |
| B0/B1, runner, performance | Chưa triển khai, ngoài phạm vi; không có metric hiệu năng |

Không có blocker môi trường sau khi chọn Python đúng. Các phần UNVERIFIED là giới hạn đã chủ động giữ lại, không được dùng chạy suite đánh giá. Family gần giống và provenance vẫn cần rà thủ công; hash chỉ bảo vệ hình học theo phép chuẩn hóa đã quy định. Certificate gồm hashes của input/config; artifact byte đã có API nhưng journal/resume/persist nguyên artifact chưa được triển khai. Không dùng certificate cũ sau khi đổi input.

**P1a.1 hoàn thành trong phạm vi đã kiểm thử ở trên.** Không có test FAIL/BLOCKED; S3/S4, S2 offset và witness có quay vẫn UNVERIFIED theo thiết kế. Các regression chỉ chứng minh không phát hiện hồi quy trên máy này. Source/YAML/launcher production giữ nguyên; chưa chứng nhận toàn bộ suite T1 để rollout. Không tạo commit hoặc push trong lượt triển khai này.

## P1a.2 tiếp theo

1. Bổ sung witness có quay và lịch điều khiển từng tick, kiểm tra Euler/caps/deadline bằng đối chứng tính tay; snapshot SearchConfig gắn camera coverage với shared camera trước khi có policy.
2. Xây chứng nhận liên tục S3 (gate/components) và S4 (mọi đường thẳng tới A); không dùng raster/planner failure làm bằng chứng âm. Mở rộng all-yaw S2 offset bằng bao miền camera origin.
3. Củng cố sai số số học và geometry suy biến bằng đối chứng/property tests; nghiệm thu registry với corpus dev/test độc lập và provenance family.
4. Persist resolved_episode.json + fixture_validation.json atomically, kiểm tra certificate stale, catalog toàn corpus và mã lỗi CLI. Chưa triển khai rollout khi các chứng nhận này chưa đủ.
5. Chạy lại regression tương ứng; chỉ sau đó xem xét runner P1b, GT/process isolation và replay. B0/B1 comparison vẫn là giai đoạn riêng.
