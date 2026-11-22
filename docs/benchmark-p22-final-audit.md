# P2.2 Stabilization và Final Re-Acceptance

Nhánh feat/benchmark; nền commit ef10650. Protocol t1-0.2, policy implementation version 2 và reducer t1-metrics-1 giữ nguyên. Hồ sơ [P2.2 trước](benchmark-p22-audit.md) và mọi FAIL/retry cũ được giữ nguyên.

**P2.2 PASS trong phạm vi final re-acceptance trên Windows.** Đã thực chạy đủ 16 primary + 16 replay attempt 1 trong paired run mới, strict replay PASS 16/16, không execution error, host continuity/metrics/identity/monitor và checker độc lập đều PASS. Đây là nghiệm thu pipeline dev, không phải benchmark nghiên cứu. Linux và planner/scheduler causal tracing vẫn UNVERIFIED; các FAIL cũ không bị thay thế.

## 1. Phân tích replay B1/S5 cũ

[Raw diagnosis](p22-audit-logs/pilot-serial/replay-wall-timeout.json) ghi primary mission TIMEOUT ở t=180 s, wall=103.86 s; replay WALL_TIMEOUT ở t=38.4 s, wall=2055.39 s, trajectory là prefix byte-exact của primary. Worker đã được thu hồi. Retry hoàn tất khớp semantic hash của primary, nhưng không thay attempt 1.

[Windows Kernel-Power](p22-audit-logs/pilot-serial/host-power-events.json) ghi Modern Standby trong cửa sổ attempt: manifest tạo 16:23:06; vào standby lúc 16:23:19 và 16:24:49; thoát lần cuối 16:57:21, result ghi 16:57:22. Đây là bằng chứng host bị gián đoạn và phù hợp với watchdog sau khi host chạy lại; không chứng minh mọi overhead là standby hoặc loại trừ mọi scheduler delay.

Python 3.10.11 trên host báo monotonic implementation GetTickCount64(), không adjustable, resolution 0.015625 s. [Microsoft](https://learn.microsoft.com/en-us/windows/win32/sysinfo/windows-time) xác nhận counter này gồm sleep/hibernation; watchdog cũ dùng time.monotonic() so với episode_wall_timeout_s, không trừ khoảng ngủ. Simulated ticks chỉ tiến khi simulator xử lý Euler dt=0.1, nên không được lấy wall gap làm thời gian nhiệm vụ hoặc bù tick.

Hồ sơ cũ có SENSE/trajectory/progress và PLANNER_BARRIER sau barrier; không có heartbeat độc lập trước/trong planner wait, không có clock/CPU sampler trong lúc gián đoạn. Vì vậy nguyên nhân worker/planner hang hoặc CPU starvation ngoài đoạn trajectory khớp là UNVERIFIED. Không diễn giải workers_reaped như bằng chứng planner luôn hoạt động.

[Đối chiếu raw](p22-final-audit-logs/historical-failure-analysis.json) xác nhận 384 interval đã thực thi (step cuối 383, t=38.4), 385 SENSE events, 20 barrier hoàn tất; barrier cuối ở t=37.9. SENSE step 384 đã được nhận trước ba terminal events do supervisor ghi. Như vậy tick 384 chưa có trajectory commit; không đủ log để biết đang tính sensing, chờ policy hay chờ planner trong tick đó. Không có PLANNER_TIMEOUT trong trace này để quy lỗi riêng cho planner.

## 2. Phần bổ sung và version

| File | Vai trò |
| --- | --- |
| [host_liveness.py](../test%20version/benchmark/host_liveness.py) | host-liveness-1: sidecar định kỳ 0.5 s, monotonic/UTC/biased boot/unbiased active clock, CPU từng PID, tick/event progress; bounded tail reader không sửa raw |
| [stabilization.py](../test%20version/benchmark/stabilization.py) | stabilized-paired-1: khóa admission trước validation, ràng buộc profile/source/pair hash, chỉ slot mới attempt 1, lưu sidecar hash và journal head, audit fail closed |
| [test_benchmark_host_liveness.py](../test%20version/test_benchmark_host_liveness.py) | 11 tests mới: clock gaps, CPU observations, torn tail, observer error, lock/tamper/no-overwrite và semantic compatibility với runner không instrument |
| [p22-final-evidence-check.cjs](checks/p22-final-evidence-check.cjs) | Chạy checker paired/metric/monitor cũ, kiểm độc lập host samples/identity/32 slots và input/policy/metric comparability |
| [p22-host-power.ps1](checks/p22-host-power.ps1) | kernel-power-window-1: truy vấn chỉ đọc Kernel-Power trên toàn cửa sổ host samples, không thay cấu hình nguồn |
| [.gitattributes](../.gitattributes) | Bảo toàn byte evidence mới |
| [spec §9.4](benchmark-t1-spec.md#94-host-liveness-sidecar-cho-final-re-acceptance), [README](../test%20version/benchmark/README.md), audit này và [P2.3 plan](benchmark-p23-plan.md) | Các tài liệu mới/chỉnh sửa; giữ nguyên audit FAIL cũ |

Không sửa paired.py, metrics.py, runner.py, batch.py, policy.py, selectors.py hoặc source/YAML production. Wrapper dùng đúng admission lock và _job_locked hiện hành; sidecar chỉ ở harness, không vào SensorFrame, scoring hay semantic events. Thêm module làm verifier fingerprint thay đổi nên tạo certificate và output mới. Mission budget và watchdog giữ nguyên, không reset/pause deadline.

QueryUnbiasedInterruptTime chỉ đếm thời gian working state theo [Microsoft](https://learn.microsoft.com/en-us/windows/win32/api/realtimeapiset/nf-realtimeapiset-queryunbiasedinterrupttime). Diagnostics so chênh boot/active clock >2 s và sampling gap >5 s; đây là ngưỡng diagnostics có version, không phải tolerance chấm task. UTC adjustment chỉ là ghi nhận, không kéo dài watchdog. Host sampler cùng process với supervisor nên cũng có thể bị scheduler/GIL/I/O trì hoãn.

CPU_ACTIVE_NO_TICK có thể là tính toán lâu hoặc spinning; CPU_QUIET_OR_UNAVAILABLE_NO_TICK có thể là blocking, starvation hoặc process đã thoát. Không tự kết luận deadlock từ CPU thấp. Muốn phân biệt chắc chắn cần phase channel policy-received/planner-submitted/started/completed và scheduler/ETW trace có version; bước này chỉ lưu CPU/progress quan sát được, không nhận là đã có heartbeat planner riêng. Lỗi quan sát/thiếu clock/gap vượt ngưỡng giữ UNVERIFIED, không tự PASS.

## 3. Môi trường và lệnh

Người dùng xác nhận giữ máy cắm nguồn, mở nắp và không Sleep/Hibernate trong phiên nghiệm thu. Không chạy powercfg để sửa cấu hình, không dùng keep-awake API, không khởi động Docker/WSL/ROS/camera/SSH/robot. Nếu cần thay nguồn điện, người dùng tự vào Settings → System → Power & battery → Screen, sleep & hibernate timeouts, tạm đặt Sleep khi cắm nguồn thành Never và khôi phục sau phiên.

Docker API không có daemon đang phục vụ; WSL chỉ có docker-desktop ở trạng thái Stopped. [Docker probe](p22-final-audit-logs/docker-probe.txt), [WSL probe](p22-final-audit-logs/wsl-probe.txt), [kiểm tra lại sau pilot](p22-final-audit-logs/linux-final-probe.json) cho cùng kết quả. Linux compatibility **UNVERIFIED**, không khởi động môi trường chỉ để gắn PASS.

```powershell
# Từ gốc repository, dùng Python 3.10.11 đã cài
python -B -m unittest discover -s "test version" -p "test_benchmark*.py" -v
python -B docs/checks/p22-pilot.py prepare --output docs/p22-final-audit-logs/pilot
# Từ test version/, sau khi tests/regression đã dừng
python -B -m benchmark.stabilization init --output docs/p22-final-audit-logs/pilot/paired
python -B -m benchmark.stabilization run --output docs/p22-final-audit-logs/pilot/paired --policy POLICY --replica primary --episode-id ID
# Chạy đủ cả hai policy × tám episode × primary/replay, từng job tuần tự
python -B -m benchmark.stabilization audit --output docs/p22-final-audit-logs/pilot/paired
# Từ gốc repository
& ./docs/checks/p22-host-power.ps1
node docs/checks/p22-final-evidence-check.cjs
```

Output mới không dùng --resume/--retry-errors; wrapper từ chối slot đã tồn tại. Nếu phiên mới FAIL, giữ trọn hồ sơ và không thay một lượt bằng retry. Không chạy tests hoặc pilot khác đồng thời trong phiên nghiệm thu.

Đã chạy tests trước pilot, rồi tám nhóm lệnh tuần tự primary-01 đến primary-04 và replay-01 đến replay-04, mỗi nhóm bốn job. [Logs](p22-final-audit-logs/) giữ console và toàn bộ raw artifacts. Production regression chạy từ `test version/` bằng `python -B -m unittest test_collision_monitor_ros1 test_launcher_ros1 test_navigation_ros1 test_search_ros1 test_simulation test_vision_tracking -v`.

## 4. Kết quả thực

| Cổng | Trạng thái |
| --- | --- |
| Benchmark regression | **PASS 143**, 132 cũ + 11 mới; [log](p22-final-audit-logs/benchmark-tests.txt), 207.743 s. Các dòng YAML error là negative CLI tests. |
| Production regression | **PASS 96**, [log](p22-final-audit-logs/production-regression.txt), 240.786 s |
| Toàn corpus trước selection | **PASS 9 certificates**, chỉ rollout 8 dev episode, held-out chỉ validate; [prepare](p22-final-audit-logs/prepare.txt), [provenance](p22-final-audit-logs/pilot/provenance.json) |
| Paired run mới 16 primary + 16 replay | **PASS**, đủ 32 attempts, toàn bộ attempt 1, không retry và không execution error; [acceptance summary](p22-final-audit-logs/acceptance-summary.json) |
| Raw metrics/paired identity | **PASS 32 reductions**, [paired audit](p22-final-audit-logs/pilot/paired/analysis/audit.json), [CLI exit 0](p22-final-audit-logs/final-audit-cli.txt) |
| Monitor checks | **PASS**, checker độc lập tái tính 114 state entries và 114 continuous episodes cùng duration từ raw. Golden tests vẫn kiểm trường hợp hai cách đếm khác nhau; không đổi nghĩa legacy field. |
| Strict replay 16/16 | **PASS**, trajectory, semantic events và result; [independent paired check](p22-final-audit-logs/pilot/independent-check.json) |
| Host continuity | **PASS trên 32 job**, 3.291 samples; max observer gap 1.969 s (<5), max biased-minus-active gap ≈0.0711 s (<2); [final independent check](p22-final-audit-logs/pilot/final-independent-check.json) |
| Kernel-Power | **PASS query**, đủ cửa sổ của 32 sample files, không event 42/107/506/507; [evidence](p22-final-audit-logs/pilot/host-power-events.json), [CLI](p22-final-audit-logs/host-power-query.txt) |
| Host bindings/audit | **PASS**, source/profile/pair/sample hashes, journal heads và final ticks được đối chiếu; [stabilization audit](p22-final-audit-logs/pilot/paired/analysis/stabilization-audit.json) |
| Independent checker | **PASS exit 0**, p22-final-evidence-1, gồm paired/CSV/monitor/replay và host; [CLI](p22-final-audit-logs/independent-check-cli.txt) |
| Comparability và bảo toàn source | **PASS**, inputs/shared configs/budgets, policy/metric identities không đổi; [source preservation](p22-final-audit-logs/source-preservation.json). Không sửa source/YAML production hoặc hồ sơ FAIL cũ. |
| Contract regression | **PASS 24**, [log](p22-final-audit-logs/contract-tests.txt) |
| Tài liệu/source QA | **PASS**, local links/anchors, code fences, whitespace source mới và git diff --check; [evidence](p22-final-audit-logs/document-source-check.json) |
| Linux | UNVERIFIED |
| Planner heartbeat / scheduler causal tracing | UNVERIFIED: sidecar không có phase channel riêng hoặc ETW trace; không gán nguyên nhân từ CPU thấp |
| Nghiệm thu P2.2 | **PASS trên Windows trong phạm vi pilot pipeline** |
| Mở benchmark nghiên cứu | **BLOCKED trước P2.3**: cần corpus/provenance/family review và khóa thiết kế/test riêng |

Đầu ra paired có [8 hàng](p22-final-audit-logs/pilot/paired/analysis/paired-results.csv), [16 primary rows](p22-final-audit-logs/pilot/paired/analysis/episodes.csv) và [32 all-attempt rows](p22-final-audit-logs/pilot/paired/analysis/all-attempts.csv), không lấy retry cũ để điền vào run mới. Sidecar nằm ở `pilot/paired/host/<replica>/<policy>/<episode>/`, mỗi job có samples.jsonl và result.json. Snapshot môi trường/profile trong [stabilization manifest](p22-final-audit-logs/pilot/paired/stabilization_manifest.json); fixture/algorithm/runtime bindings trong [paired manifest](p22-final-audit-logs/pilot/paired/paired_manifest.json).

## 5. Giới hạn và bước sau

Sidecar là quan sát cooperative, không phải sandbox chống policy đối địch hay chữ ký chống sửa đồng loạt. Bounded sampling không chứng minh host chưa từng gián đoạn ở mọi thang thời gian; cần đối chiếu thêm Kernel-Power trong cửa sổ chạy. Planner phase/scheduler causality vẫn UNVERIFIED nếu thiếu trace, không dùng diagnostics để đổi task result.

Corpus vẫn là tám dev episode trên bảy layout/sáu family khai báo, hand-authored 2D oracle. Không công bố superiority B0/B1 hoặc suy ra khả năng tìm nạn nhân trong thiên tai thật. Điều kiện khóa test, review provenance, family split, thống kê và benchmark nghiên cứu nằm trong [kế hoạch P2.3](benchmark-p23-plan.md); chỉ chuyển bước sau nghiệm thu thực.
