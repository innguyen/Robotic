# P0 audit trước triển khai benchmark T1

## 1 Phạm vi và bằng chứng

Audit thực hiện ngày 21/11/2026 trên nhánh `feat/benchmark`, commit `b7025fd182399519c097a5286ffeb64991c02439`. Phần theo dõi bởi Git không có thay đổi khi bắt đầu; `docs/` là dữ liệu chưa được Git theo dõi. Không sửa source/YAML/launcher production, không chạy ROS, camera, SSH hoặc điều khiển robot.

Hợp đồng hiện tại là [T1 t1-0.2](benchmark-t1-spec.md), đối chiếu với [báo cáo phương pháp](benchmark-report.md). Đây là audit tài liệu và regression của code hiện có, **không phải kết quả benchmark T1**. Chưa có suite được chứng nhận, runner, rollout B0/B1 hoặc pilot.

Môi trường đã xác minh: Windows, Python 3.10.11, NumPy 2.2.6, PyYAML 6.0.3, OpenCV 4.14.0; Node.js 24.14.1. [Log runtime](p0-audit-logs/runtime.txt). Python launcher trong sandbox trỏ tới đường dẫn không tồn tại; chạy được bằng executable Python 3.10 thực tế ngoài sandbox, không cài thêm dependency.

## 2 Thay đổi hợp đồng và kiểm tra đi kèm

| Thay đổi                                                               | File liên quan                                              | Kiểm tra P0                                                                                                                                     | Giới hạn / nghiệm thu còn lại                                                                         |
| ---------------------------------------------------------------------- | ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| S1–S5 có predicate, đối chứng và chứng nhận PASS/FAIL/UNVERIFIED       | Spec §3.4, report §5.3                                      | PASS tĩnh: đủ năm dòng điều kiện/đối chứng, gate metadata và quy tắc loại UNVERIFIED                                                            | Chưa có verifier liên tục; toàn bộ đối chứng hình học phải được hiện thực ở P1a                       |
| Hash hợp obstacle, registry environment/family/split                   | Spec §3.3; [mã hash tham chiếu](checks/layout-contract.cjs) | PASS thực thi: known-answer SHA-256; reorder, rename, duplicate, partition, translation; thay bounds/obstacle; mismatch, alias và split leakage | Chưa kiểm tra registry suite thật; family cần provenance; hash không tự bắt mọi layout gần giống      |
| B0 có điểm dự phòng; hết goal thành stalled tới budget ở cả hai policy | Spec §5; report §5.4                                        | PASS tĩnh: fallback, stalled, diagnostics và positive controls có trong hợp đồng; 7 test SearchMission cũ PASS riêng                            | Chưa có B0 adapter/rollout. Test SearchMission cũ không chứng minh B0 mới khả thi                     |
| Cùng mô hình pose/quan sát/động học, quỹ đạo độc lập                   | Spec §2, §5.2; report §5.4                                  | PASS tĩnh: công thức Euler, swept yaw không wrap và action khác sinh pose/scan khác                                                             | Chưa có test tích phân/isolation/replay của runner                                                    |
| Bounds đóng, vùng A/F, visibility cuối và quy ước đường dẫn            | Spec §3.1, §6.3; report §5.5                                | PASS: link/anchor, JSON examples, resolve đường YAML ví dụ; kiểm tra tĩnh quy tắc A                                                             | Chưa có CLI validator chặn traversal/symlink; chưa kiểm tra swept geometry/bounds/clearance thực thi  |
| Tách baseline comparison/ablation; thêm 1.000 episode Habitat          | Report §3.7, §4.1–4.2                                       | PASS tĩnh và đối chiếu trang chính thức [Participation Guidelines](https://aihabitat.org/challenge/2023/) ghi 1.000 episode/agent               | Không suy ra số episode train/val/minival; không tái xác minh toàn bộ bibliography trong lượt sửa này |
| Cập nhật ghi chú blocker Python cũ và dẫn tới protocol hiện hành       | Plan §8                                                     | PASS kiểm tra link tới audit/spec                                                                                                               | Nội dung v0.1 của plan vẫn là lịch sử đề xuất; không dùng thay đặc tả t1-0.2                          |

Các kiểm tra P0 ở bảng trên nằm trong [benchmark-contract.test.cjs](checks/benchmark-contract.test.cjs); [log 24 test](p0-audit-logs/contract-checks.txt). Kiểm tra tĩnh chỉ bảo vệ nội dung hợp đồng khỏi thiếu hoặc mâu thuẫn đã biết. Nó không biến yêu cầu ghi trong tài liệu thành hành vi đã triển khai. Test hash/registry thực thi thuật toán tham chiếu; không phải schema validator đầy đủ và không kiểm tra GT isolation, feasibility hoặc scenario.

## 3 Đối chiếu source

| Thành phần                                                                                                               | Bằng chứng hiện có                                                                                                                                                     | Điều phải làm trong harness                                                                                                                 |
| ------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| [SearchMission](../test%20version/search_core_ros1.py)                                                                   | `choose_goal()` dùng gain/(distance+0.5), min-gain, blacklist/revisit và focus; `_next_goal()` hiện trả SEARCH_COMPLETE khi không có goal; `found` chỉ đặt sau ARRIVED | Adapter riêng cho B0, miền ứng viên và stalled của cả hai policy; không sửa default core                                                    |
| [Simulator](../test%20version/simulate_navigation.py)                                                                    | Time bắt đầu 10 s; dt 0,1 s; CLI giới hạn 600 bước; camera visibility xấp xỉ từ ray LiDAR, confirm hai frame/memory 2,5 s                                              | Đồng hồ/budget từ fixture; oracle ray đúng extrinsics; observation/report events riêng; không dùng simulator hiện tại làm evaluator độc lập |
| [Navigator](../test%20version/navigation_core_ros1.py)                                                                   | Async future; simulator chờ tối đa 0,2 s mỗi bước. `reset()` tăng generation; `close()` chờ executor bằng shutdown(wait=True)                                          | Barrier rõ ràng, process timeout/cleanup; reset không thay thế cách ly worker; chưa có replay chứng minh deterministic                      |
| [Navigation ROS adapter](../test%20version/navigation_ros1.py) và [monitor](../test%20version/collision_monitor_ros1.py) | ROS được import trong main; pose/source/command watchdog và monitor gate có logic riêng                                                                                | Reuse core offline sau kiểm tra import; giữ nguyên ROS/default/preview/live/watchdog; evaluator hình học nằm ngoài monitor                  |
| [Vision](../test%20version/jetauto_find_object.py)                                                                       | Các test tracking chạy được với OpenCV hiện có                                                                                                                         | Không coi tracking test là đánh giá detector, camera hoặc dữ liệu SAR                                                                       |

Standoff `person = 0.55` được Navigator cộng thêm nửa chiều dài thân xe; arrival còn xét goal tolerance và hướng. Vì vậy A dùng cấu hình thực, không mô tả 0,55 m là khoảng cách giữa hai tâm. Bounds và target box phải được chấm bằng hình học độc lập cả giữa hai tick.

## 4 Lệnh và kết quả kiểm thử

Chạy từ gốc repository:

```powershell
node --test docs/checks/benchmark-contract.test.cjs
node "test version/test_navigation_status.js"
```

Chạy Python từ `test version/`, dùng executable đã xác minh (máy khác chọn Python có đủ dependencies):

```powershell
& 'C:\Users\lemin\AppData\Local\Programs\Python\Python310\python.exe' -B -m unittest -v test_search_ros1 test_simulation test_navigation_ros1 test_collision_monitor_ros1 test_vision_tracking test_launcher_ros1
```

| Bộ kiểm tra                  | Kết quả thực tế                                                    | Bằng chứng                                        |
| ---------------------------- | ------------------------------------------------------------------ | ------------------------------------------------- |
| P0 contract/reference        | PASS, 24/24 test; exit 0                                           | [Log thô](p0-audit-logs/contract-checks.txt)      |
| Python regression sáu module | PASS, 96/96 test trong 256,383 s; exit 0; không failure/error/skip | [Log thô](p0-audit-logs/python-regression.txt)    |
| Dashboard diagnostics        | PASS, 13 checks; exit 0                                            | [Log thô](p0-audit-logs/dashboard-regression.txt) |

Phân bổ 96 test Python: search 7, simulation 8, navigation 43, collision monitor 26, vision tracking 8, launcher/web control 4. Log giữ từng tên test; không có kết quả benchmark được trộn vào các số này. Kiểm tra `git diff --exit-code HEAD -- . ':(exclude)docs'` sau test trả exit 0; working tree vẫn chỉ có `?? docs/`.

Launcher tests dùng fake commands/temporary files và HTTP loopback cho web control; không gọi robot. Các test simulator là regression của mô hình sẵn có, không phải fixture T1 mới. Không dùng số test hoặc kết quả ARRIVED của simulator làm success rate tìm người.

## 5 Cổng P1 và phần chưa xác minh

**P0 PASS trong phạm vi tài liệu, hash tham chiếu và regression offline. Cổng bắt đầu P1a đã mở; chưa triển khai P1 trong lượt này.** Không còn mâu thuẫn đã biết giữa report/spec ở sáu mục được yêu cầu. Trạng thái này cho phép xây validator/fixture, không cho phép coi suite hoặc benchmark đã nghiệm thu.

P1a cần hiện thực schema, registry toàn cục, F/A, swept geometry, oracle và chứng nhận S1–S5 với đối chứng. Chỉ sau đó mới tạo fixture hợp lệ và tiến tới runner. Positive controls của B0, worker timeout/cleanup, replay, resume/journal, metric và GT isolation vẫn là yêu cầu chưa kiểm chứng; không được coi là đã PASS từ P0. Chưa khóa suite test hoặc có cơ sở so sánh B0/B1. Kiểm thử ROS/Ubuntu, calibration, camera và JetAuto chưa thực hiện.
