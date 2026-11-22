# Nghiệm thu P1a.2 — Chứng nhận liên tục và artifact

## Phạm vi và contract

Thực hiện trên `feat/benchmark`, tiếp nối [P1a.1](benchmark-p1a1-audit.md), ngày 22/11/2026. Protocol vẫn là **t1-0.2**, verifier mới là **p1a2-1**. Không đổi success/matching, threshold, bounds, đồng hồ, observation hoặc tiêu chí S1–S5. Không sửa code/config production, không chạy policy B0/B1, ROS, camera, SSH hoặc robot.

SearchConfig trong snapshot benchmark lấy camera FOV/range từ `shared.camera`, để coverage dùng đúng mô hình quan sát. Đây là resolve cấu hình cho harness, không sửa defaults của SearchMission. Verifier/source/evidence mới được ghi hash; certificate P1a.1 không được tự coi tương đương hoặc tái sử dụng trong bundle mới.

## File và kiến trúc

| File                                                                                | Thay đổi                                                                                          |
| ----------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| [witness.py](../test%20version/benchmark/witness.py)                                | Mới: Euler từng tick, command caps, swept footprint; dịch chuyển rồi quay, yaw waypoint unwrapped |
| [proofs.py](../test%20version/benchmark/proofs.py)                                  | Mới: exact rational ray, convex shadow, camera-origin enclosure, S3 decomposition, S4 barrier     |
| [artifacts.py](../test%20version/benchmark/artifacts.py)                            | Mới: catalog/provenance, manifest, stale/tamper check, publish bundle atomically                  |
| [scenarios.py](../test%20version/benchmark/scenarios.py)                            | Tích hợp chứng nhận S2 offset, S3/S4, confirmation và lịch witness/report/hold theo budget        |
| [schema.py](../test%20version/benchmark/schema.py)                                  | Resolve SearchConfig camera trong namespace benchmark                                             |
| [geometry.py](../test%20version/benchmark/geometry.py)                              | Làm tròn cận sweep ra ngoài và guard 32 ULP; không nới ngưỡng collision/bounds                    |
| [**main**.py](../test%20version/benchmark/__main__.py)                              | CLI `--catalog/--output`; chẩn đoán `--suite` không được xuất artifact                            |
| [test_benchmark.py](../test%20version/test_benchmark.py)                            | Cập nhật kỳ vọng witness có quay; tách budget ngắn khỏi test va chạm                              |
| [test_benchmark_p1a2.py](../test%20version/test_benchmark_p1a2.py)                  | Mới: 21 tests với đối chứng/property/atomic/stale                                                 |
| [advanced.json](../test%20version/benchmark/fixtures/advanced.json)                 | Mẫu S3/S4 có hình học, gate và budget riêng                                                       |
| [independent-test.json](../test%20version/benchmark/fixtures/independent-test.json) | Layout ba obstacle bất đối xứng độc lập để kiểm tra split                                         |
| [corpus-witnesses.json](../test%20version/benchmark/fixtures/corpus-witnesses.json) | Main/alternate branch paths chỉ dành evaluator                                                    |
| [catalog.json](../test%20version/benchmark/fixtures/catalog.json)                   | Corpus ba suite, sáu episode và provenance family                                                 |
| [README.md](../test%20version/benchmark/README.md)                                  | Hướng dẫn/API/giới hạn hiện hành                                                                  |

Luồng: catalog → validate mọi suite/config/registry → continuous negative proof + positive tick witness → certificate → bundle staging → rename → revalidation bằng recompute. Bundle chứa GT và witness, chỉ dành evaluator; không cấp bundle, Oracle instance hoặc certificate cho policy.

## Cơ sở chứng nhận và giới hạn

**Witness:** mỗi waypoint được dịch chuyển ở yaw hiện tại rồi quay theo chênh lệch yaw unwrapped. Command mỗi tick được tích phân đúng Euler của spec; swept check dùng yaw chưa wrap trong interval. Có test tiến/ngang, qua ±π và hai vòng quay không bị rút ngắn. Thêm confirmation đứng yên ở start S1, confirmation tại endpoint, interval report và hold dừng; tổng số interval so với budget bằng số nguyên. Đây là đường chứng tồn tại, không phải hành vi SearchMission hoặc số liệu thời gian B0/B1.

**S2:** toàn bộ camera origin khi thân quay nằm trong AABB của đĩa offset. Nếu toàn AABB nằm trong bóng che một obstacle, mọi yaw bị che. Một yaw thật có footprint hợp lệ/range/FOV được dùng làm chứng minh dương. Việc lấy mẫu yaw chỉ tìm chứng dương, không được dùng để kết luận mọi yaw không thấy. Có test offset dương được chứng nhận và offset lớn không đủ bằng chứng.

**S3:** chia các rectangle theo mọi cạnh obstacle/gate bằng số hữu tỷ, không raster lấy mẫu. Robot-centre point-free space là miền lớn hơn F. Chứng minh vùng liên thông chứa start trong miền lớn đó bị che toàn bộ là đủ để loại visibility trong F. Đóng gate phải tạo các nhánh khác nhau; mở riêng từng gate phải nối đúng một nhánh với vùng chung. Sau đó main witness tới A và witness tới miền đứng mở trong mọi nhánh khác đều phải qua kiểm tra footprint thật. Đây là chứng nhận đủ cho mẫu hình học được hỗ trợ, không phải tính toàn bộ configuration space của rectangle quay.

**S4:** mọi tâm thuộc A nằm trong AABB phủ đĩa bán kính standoff đã cộng nửa thân/tolerance. Chứng minh toàn AABB nằm sau obstacle đối với start đủ để chặn mọi đường thẳng tới A. Witness đi vòng tới A được kiểm tra độc lập. Có đối chứng chỉ chặn đường tới tâm người: không được PASS.

Shadow của obstacle lồi từ một điểm là lồi: tổ hợp lồi của các điểm dạng `t*b`, `t>=1`, vẫn có dạng đó với b trong obstacle. Vì vậy kiểm tra bốn góc miền chữ nhật bằng exact Fraction đủ cho toàn miền. Obstacle dùng để chứng nhận được inset 32 ULP để từ chối chứng cứ chỉ tiếp xúc biên; camera bounds/cận sweep làm tròn ra ngoài. Đây là điều kiện chứng nhận chặt hơn, không thay đổi phép chấm visibility hoặc geometry epsilon.

**UNVERIFIED còn lại:** gate chéo, phân rã quá 4096 cells, witness quá 100.000 tick, shadow không phủ được toàn miền, hoặc sweep không chứng minh được clearance trong giới hạn subdivision. Trig/SAT/clearance vẫn binary64 có guard, chưa phải interval arithmetic toàn bộ. Chưa kiểm chứng động lực học, acceleration controller, monitor trên witness hay an toàn phần cứng.

## Artifact và corpus

CLI catalog kiểm tra toàn bộ dev/test trước mọi chứng nhận; không hỗ trợ filter bỏ qua registry. Family provenance yêu cầu đủ key, nguồn/ghi chú và `reviewed=true`; không tự chứng minh các family gần giống độc lập. Dữ liệu mẫu do tác giả đối chiếu hình học để kiểm tra validator, **không phải bộ held-out test dùng đánh giá thuật toán**. Corpus thực vẫn cần rà nguồn sinh bằng con người và khóa riêng.

Bundle mỗi episode có `resolved_episode.json`, `evidence.json`, `fixture_validation.json`; manifest giữ hash mọi file, source verifier và catalog. Input/config/witness/verifier đổi, certificate bị sửa, thiếu/thừa file đều làm output cũ bị từ chối. Không sửa bundle đã tồn tại. Một bundle có certificate UNVERIFIED vẫn có thể được lưu để audit nhưng CLI trả exit 2.

Publish dùng staging cùng parent, fsync file rồi rename cả directory. Test tiêm lỗi trước rename xác nhận không có output dở. Chưa có kiểm thử kill process thật hoặc mất điện; orphan staging có thể còn sau kill và không phải bundle hợp lệ. Chưa có journal/resume runner. Giả định workspace không bị tiến trình đối địch đổi symlink đồng thời.

## Lệnh và bằng chứng thực chạy

Từ gốc repository:

```powershell
python -B -m unittest discover -s "test version" -p "test_benchmark*.py" -v
node --test docs/checks/benchmark-contract.test.cjs
node "test version/test_navigation_status.js"
```

Từ `test version/`:

```powershell
python -B -m unittest -v test_search_ros1 test_simulation test_navigation_ros1 test_collision_monitor_ros1 test_vision_tracking test_launcher_ros1
python -B -m benchmark --catalog "test version/benchmark/fixtures/catalog.json" --output "docs/p1a2-audit-logs/validation-final"
```

Python thực dùng executable 3.10.11 đã xác minh từ P0; không cài dependency mới. Lệnh test chéo hash cần Node. Các đường suite/catalog/output đều relative từ gốc repository, dù gọi CLI từ `test version/`.

| Kiểm tra              | Kết quả                                         | Bằng chứng                                       |
| --------------------- | ----------------------------------------------- | ------------------------------------------------ |
| Unit P1a.1 + P1a.2    | PASS 51 tests, không skip                       | [unit.txt](p1a2-audit-logs/unit.txt)             |
| Production regression | PASS 96 tests, không skip, 196,970 giây; exit 0 | [regression.txt](p1a2-audit-logs/regression.txt) |
| CLI/catalog/bundle    | PASS 7 ca CLI: publish/recheck exit 0; các ca từ chối exit 2      | [cli.json](p1a2-audit-logs/cli.json)             |
| Contract P0           | PASS 24 tests; exit 0 | [contract.txt](p1a2-audit-logs/contract.txt)     |
| Dashboard             | PASS 13 checks; exit 0 | [dashboard.txt](p1a2-audit-logs/dashboard.txt)   |
| Bundle kiểm tra độc lập bằng Node | PASS hash 19 file, source 10 module; sáu certificate và chuỗi tick liên tục | [bundle-check.json](p1a2-audit-logs/bundle-check.json) |
| Production diff | Không đổi tracked production, git diff exit 0 | [source-state.txt](p1a2-audit-logs/source-state.txt) |

Lượt unit đầu có hai FAIL do test data: test collision giữ budget 0,1 giây của ca trước; obstacle test quay nằm ngoài bán kính quét. Đã sửa dữ liệu, chạy lại thành công; giữ [log ban đầu](p1a2-audit-logs/unit-initial.txt). Không xóa lượt lỗi khỏi hồ sơ.

Bundle hiện hành là [validation-final/manifest.json](p1a2-audit-logs/validation-final/manifest.json). Bundle `validation-bundle/` và `cli-initial.json` giữ làm bằng chứng lịch sử: source verifier đã đổi nên CLI từ chối tái sử dụng bundle đó. Không dùng bundle cũ để nghiệm thu. Lượt cuối có thêm test yaw khởi tạo ngoài [-π, π] để kiểm tra cả lịch đứng yên wrap đúng Euler và pose liên tục giữa các interval.

## Cổng tiếp theo

**P1a.2 hoàn thành trong phạm vi chứng nhận bảo thủ đã kiểm thử.** Không có test FAIL/BLOCKED ở lượt cuối. S2 offset, S3 và S4 đã có mẫu dương được chứng nhận và đối chứng bị loại; điều này không bảo đảm verifier chứng nhận được mọi layout hợp lệ. Trường hợp chưa đủ bằng chứng vẫn UNVERIFIED, không được rollout.

Chưa bắt đầu P1b hoặc so sánh B0/B1. Có thể bắt đầu xây P1b trên fixture PASS với source/config/evidence hash hiện hành; fixture UNVERIFIED phải bị từ chối. Chưa khóa suite đánh giá, chưa có kết quả benchmark. Không tạo commit/push trong lượt này.

P1b cần: worker cách ly episode, DTO observation không chứa GT, planner barrier/timeouts/cleanup, journal/resume và replay; evaluator vẫn ở phía sở hữu GT. Sau runner acceptance mới đến positive controls của B0/B1 và pilot. Không được dùng witness ticks, số unit tests hoặc sáu certificate PASS làm metric tìm người.

Riêng coverage: SearchConfig chỉ chứa FOV/range, chưa chứa camera extrinsics. Trước rollout với camera offset, P1b phải có adapter coverage dùng camera pose thay vì mặc định tâm thân xe cho cả hai policy. Việc resolve FOV/range hiện tại không tự xác minh coverage của production với offset; production được giữ nguyên trong lượt này.
