# Đặc tả benchmark T1 cho robot tìm kiếm con người

## 1 Phạm vi và phiên bản

Phiên bản protocol và schema là `t1-0.2`. Tài liệu này cụ thể hóa [benchmark đề xuất trong báo cáo](benchmark-report.md). Validator, oracle, single runner và batch/resume đã triển khai trong phạm vi ghi tại [P1b.2 audit](benchmark-p1b2-audit.md). Hai selector chính thức và dev controls được theo dõi tại [P2.1 audit](benchmark-p21-audit.md). Các bảng yêu cầu bên dưới không tự chứng minh kiểm tra đã PASS; xem audit và log tương ứng. Chưa có đánh giá B0/B1 trên test đã khóa.

T1 đánh giá tìm kiếm và điều hướng trong hình học phẳng, với tối đa một mục tiêu `person` không di chuyển. Người được biểu diễn bằng một hình chữ nhật có vị trí xác định, không có nhãn tư thế. Perception là oracle hình học, không chạy YOLO hoặc mô phỏng người bị vùi kín.

Mọi thay đổi ở đây chỉ áp dụng cho harness offline. Giữ nguyên cấu hình và hành vi mặc định của runtime ROS, preview/live, collision monitor và watchdog. Các ngưỡng dưới đây là quy ước mô phỏng, chưa phải thông số an toàn đã hiệu chuẩn cho robot thật. Khi đổi quy tắc chấm sau khi khóa suite, tăng phiên bản protocol.

## 2 Frame và đồng hồ

| Đại lượng                | Quy ước                                                               |
| ------------------------ | --------------------------------------------------------------------- |
| Frame thế giới           | `benchmark_map`; gốc và hình học do fixture định nghĩa                |
| Trục và đơn vị           | x/y theo mét; yaw theo radian, dương ngược chiều kim đồng hồ quanh +z |
| Frame thân robot         | +x tiến, +y trái; `vx`, `vy` theo m/s, `wz` theo rad/s                |
| Pose cung cấp cho policy | Trong `benchmark_map`, trùng pose GT theo giả định noiseless của T1   |
| Thời gian nhiệm vụ       | Bắt đầu từ 0, bước `dt_s = 0.1`; không dùng đồng hồ ROS               |
| Thời gian thực thi       | Đồng hồ monotonic, chỉ dùng đo chi phí và watchdog                    |

Adapter truyền pose cùng hệ tọa độ vào core vốn dùng tên `odom`; không kết nối TF/ROS. Mốc thời gian nội bộ 10 giây của simulator cũ phải được chuẩn hóa trước khi ghi sự kiện benchmark.

Động học T1 là mô hình holonomic tích phân Euler như simulator hiện tại. Với command đã qua monitor ở bước k: `x_next = x + (cos(yaw)*vx - sin(yaw)*vy)*dt`, `y_next = y + (sin(yaw)*vx + cos(yaw)*vy)*dt`, `yaw_next = wrap(yaw + wz*dt)`. Để chấm swept geometry, tâm nội suy tuyến tính giữa hai endpoint và yaw đi theo góc tăng `wz*dt` chưa wrap, tránh nhảy qua ±pi. Đây là quy ước rời rạc chung cho B0/B1, chưa mô phỏng trượt bánh hoặc động lực học phần cứng. Không được tích phân một policy bằng cách khác.

`time_limit_s` là bội nguyên dương của `dt_s`. Dùng chỉ số bước để xét deadline, không cộng số thực liên tiếp. Không gọi policy khi thời gian đã tới deadline. Một kết quả xác nhận dừng hoàn tất đúng deadline vẫn hợp lệ.

## 3 Suite và fixture

### 3.1 Cấu trúc dữ liệu

Suite dùng JSON UTF-8. Đường dẫn trong JSON/CLI dùng `/`, tương đối với gốc repository; cấm đường dẫn tuyệt đối, `..`, và symlink thoát khỏi repository sau khi resolve. Markdown dùng link tương đối với file đang đọc; khoảng trắng trong đích link ghi `%20`. Đây là hai quy ước đường dẫn khác nhau. Validator từ chối trường không biết, số không hữu hạn, ID trùng và phiên bản không hỗ trợ. Suite/episode/environment/family ID dùng mẫu `[A-Za-z0-9][A-Za-z0-9_-]{0,79}`, không dùng dấu phân cách đường dẫn.

Ví dụ dữ liệu đầu vào cho một S1. Hash đã được kiểm tra bằng mã tham chiếu P0; hình học, budget và scenario vẫn phải qua validator P1 trước khi được coi là fixture hợp lệ:

```json
{
  "schema_version": "t1-0.2",
  "suite_id": "pilot-dev-v1",
  "shared": {
    "navigation_yaml": "test version/navigation_ros1.yaml",
    "monitor_yaml": "test version/collision_monitor_ros1.yaml",
    "person_standoff_m": 0.55,
    "dt_s": 0.1,
    "camera": {
      "x_m": 0.0,
      "y_m": 0.0,
      "yaw_rad": 0.0,
      "fov_deg": 58.0,
      "range_m": 3.5,
      "confirm_frames": 2,
      "memory_s": 2.5
    },
    "report_tolerance_m": 0.5,
    "stop_check": {
      "linear_speed_m_s": 0.01,
      "angular_speed_rad_s": 0.01,
      "hold_steps": 3,
      "timeout_s": 2.0
    },
    "b0_spacing_m": 0.75,
    "scenario_checks": { "s2_max_view_path_m": 0.75 },
    "geometry_epsilon_m": 0.001,
    "planner_wall_timeout_s": 10.0,
    "episode_wall_timeout_s": 300.0
  },
  "episodes": [
    {
      "episode_id": "S1-DEV-001",
      "environment_id": "layout-dev-01",
      "layout_family_id": "family-dev-01",
      "layout_hash": "eb085274e87be582ce14b082a7479a0c7e3b97f1ec7f428f278e3597806f56fb",
      "scenario_id": "S1",
      "split": "dev",
      "seed": 42,
      "frame_id": "benchmark_map",
      "bounds_m": [-1.0, -2.0, 4.0, 2.0],
      "start_pose": { "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0 },
      "time_limit_s": 180.0,
      "obstacles": [{ "id": "box-01", "bounds_m": [0.8, 0.8, 1.2, 1.2] }],
      "targets": [
        {
          "id": "gt-person-01",
          "label": "person",
          "x_m": 2.0,
          "y_m": 0.0,
          "size_m": [0.3, 0.3]
        }
      ]
    }
  ]
}
```

`bounds_m` có thứ tự `[xmin, ymin, xmax, ymax]`, với xmin < xmax và ymin < ymax. Đây là miền nhiệm vụ đóng được công bố cho B0/B1, không tiết lộ bản đồ vật cản. Toàn bộ footprint, kể cả khi quay và giữa hai tick, phải nằm trong miền này. Chạm biên bounds được phép nếu không có tường; vượt biên là `OUT_OF_BOUNDS`, không cắt pose về biên. Geometry epsilon không được dùng để cho phép vượt bounds. Bounds không tự sinh tường LiDAR; tường vật lý phải khai báo trong `obstacles`. Ray cảm biến không bị cắt bởi biên nhiệm vụ và vẫn có thể thấy obstacle được khai báo bên ngoài bounds.

Obstacle là box song song trục. Target có box tâm tại `(x_m, y_m)`, kích thước dương theo x/y; box này cũng là vật cản vật lý. `targets` có 0 hoặc 1 phần tử; S5 bắt buộc rỗng, S1–S4 bắt buộc có một người. GT ID không xuất hiện trong observation hoặc báo cáo do policy tạo.

`seed` là số nguyên không âm, lưu nguồn gốc biến thể. Fixture đã chứa tọa độ cuối cùng: runner không tự đặt lại người/vật cản dựa trên kết quả method. Nếu sinh fixture bằng ngẫu nhiên, thực hiện trước và lưu dữ liệu đã sinh. Chạy lại cùng hình học với seed khác không được tính là một mẫu môi trường mới.

### 3.2 Kiểm tra trước khi chạy

- Kiểm tra kiểu, trường bắt buộc, đơn vị, bounds, kích thước, ID và budget.
- Pose ban đầu hợp lệ, footprint không chạm vật cản/người và không ra ngoài bounds; áp dụng cùng geometry epsilon như bộ chấm. Target không chồng obstacle hoặc vượt bounds.
- Mỗi episode dương tính có ít nhất một vùng vừa quan sát được vừa tiếp cận hợp lệ, nối được từ start theo footprint. Kiểm tra này dùng GT offline, chỉ trả kết quả validation cho runner, không đưa đường tham chiếu cho policy. Bộ kiểm tra chưa chứng nhận được tính khả thi thì từ chối episode, không âm thầm cho qua.
- Kiểm tra toàn bộ registry dev/test theo §3.3, trước khi áp dụng bộ lọc CLI. Không chỉ kiểm tra các episode được chọn chạy.
- Chứng nhận đặc trưng scenario theo §3.4. Tên S1–S5 và metadata do người tạo fixture khai không đủ làm bằng chứng.
- Đọc đầy đủ YAML, `SearchConfig` và mọi cấu hình dùng chung. Các ngưỡng phải dương, `hold_steps` nguyên dương, stop-check timeout là bội dt và không ngắn hơn cửa sổ hold. Thiếu file/dependency phải báo lỗi; không fallback sang Python defaults.
- Xác nhận hash của cấu hình đã nạp và fixture. Sau khi bắt đầu run, không sửa suite/config; muốn đổi phải tạo run mới.

Suite không hợp lệ bị từ chối trước mọi episode. Báo rõ vị trí lỗi; không tính lỗi fixture thành thất bại của thuật toán.

### 3.3 Nhận diện layout và chống rò split

`environment_id` nhận diện một layout tĩnh; `layout_family_id` nhóm các layout cùng nguồn sinh, gồm bản xoay, phản chiếu hoặc sửa nhẹ. Mỗi episode bắt buộc có hai ID này và `layout_hash` SHA-256 chữ thường 64 ký tự. Start, người, seed, scenario và budget không thuộc hình học layout. `fixture_hash` là SHA-256 của byte UTF-8 artifact `resolved_episode.json` đã chứa đầy đủ shared/config và episode. Lưu chính artifact đó, dùng chung cho cả cặp; không serialize lại bằng cách khác khi resume. Không dùng `layout_hash` thay thế fingerprint đầy đủ này.

Thuật toán hash `rect-union-um-v1`:

1. Bounds và các cạnh obstacle phải biểu diễn chính xác tới micromét, chuyển thành số nguyên an toàn binary64 bằng phép nhân 10^6 và kiểm tra chuyển ngược đúng số gốc. Từ chối độ chính xác vượt mức này, không làm tròn âm thầm. Các hiệu tọa độ cũng phải là số nguyên an toàn.
2. Tịnh tiến hình học về gốc `(xmin, ymin)`. Giữ chiều rộng/cao bounds và **hợp hình học** của mọi obstacle, kể cả phần ngoài bounds; bỏ ID, thứ tự và box lặp. Không đưa target vào hợp tĩnh.
3. Chia theo các cạnh x của box. Trong mỗi slab, hợp các đoạn y chồng hoặc chạm nhau; sắp tăng. Bỏ slab rỗng; gộp slab kề nhau có cùng danh sách đoạn y. Như vậy chia một tường thành nhiều box không đổi hash.
4. Serialize UTF-8, không whitespace, theo đúng thứ tự khóa `bounds_um`, `slabs`, `version`. Mỗi slab là `[xlo,xhi,[[ylo,yhi],...]]`; version là `rect-union-um-v1`. Lấy SHA-256 của byte JSON này.

Chuỗi chuẩn của ví dụ §3.1 là `{"bounds_um":[5000000,4000000],"slabs":[[1800000,2200000,[[2800000,3200000]]]],"version":"rect-union-um-v1"}`. [Mã tham chiếu P0](checks/layout-contract.cjs) và [test](checks/benchmark-contract.test.cjs) kiểm tra quy tắc này; chúng chưa kiểm tra tính khả thi của scene.

Các bất biến bắt buộc trên registry hợp nhất của tất cả suite:

- Một `environment_id` chỉ có một hash, một family và một split; thay obstacle/bounds thì cấp environment ID mới.
- Một hash chỉ có một `environment_id` và một split. Từ chối đổi tên cùng hình học để vượt split, kể cả sau tịnh tiến hoặc chia lại box.
- Một family chỉ thuộc dev **hoặc** test. Hash không tự phát hiện xoay, phản chiếu hoặc layout gần giống; nguồn sinh/family phải được kiểm tra bằng provenance, không chỉ tin tên ID.
- Thay start/target/seed không tạo layout mới. Fixture vật lý trùng nhau chỉ được ghi một episode đánh giá, không nhân mẫu bằng đổi ID, seed hoặc nhãn scenario.
- Tính lại và so sánh hash khai báo; sai hash thì từ chối. Lưu registry cùng suite/config hashes vào manifest. Resume và ghép cặp B0/B1 phải khớp cả registry lẫn fixture fingerprint.

### 3.4 Xác minh S1–S5

Gọi `F` là tập pose có footprint hợp lệ theo §6.3 và `A` là vùng tiếp cận ở mục đó. `V` là tập pose trong F có oracle **raw-visible** theo §4; `V_xy` là vị trí tâm có ít nhất một yaw hợp lệ thuộc V. Xét camera extrinsics thực sự đã khai báo, không thay bằng tia từ tâm thân xe khi có offset.

Mọi episode có người phải có đường chứng từ start tới A. Đường là chuỗi đoạn dịch chuyển/quay được xác minh swept footprint, thỏa mô hình động học/speed caps và đủ budget cho sensing, confirmation, report, hold dừng. Đó chỉ là chứng minh tồn tại, không yêu cầu B0/B1 đi theo đường này. Cận dưới thời gian vượt budget thì loại; chỉ có đường hình học nhưng chưa chứng minh được budget thì `UNVERIFIED` và cũng loại.

| Scenario | Điều kiện bắt buộc ngoài tính khả thi chung | Đối chứng phải bị từ chối |
| --- | --- | --- |
| S1 | Start với yaw đã khai báo thuộc V; có thể đứng yên đủ `confirm_frames` mà vẫn hợp lệ và còn budget tới A | Người chỉ thấy sau khi xoay, ngoài range hoặc bị che ở yaw xuất phát |
| S2 | Start_xy không thuộc V_xy **do obstacle che ray**, dù mục tiêu nằm trong range ở một yaw hợp lệ. Có đường hợp lệ dài không quá `s2_max_view_path_m` tới V; mặc định 0,75 m | Chỉ quay lưng/ngoài FOV; chỉ ngoài range; điểm nhìn thay thế nằm sau khe robot không qua được |
| S3 | Có ít nhất hai nhánh được chứng nhận bằng các gate tại cửa nhánh; start nằm ở vùng chung. Đóng các gate bằng rào ảo thì vùng nối với start không chứa V. Có đường thật vượt gate nhánh mục tiêu tới V và A; nhánh còn lại có vùng đứng quan sát hợp lệ, tách biệt nhánh mục tiêu | Không có nhánh thứ hai; vẫn thấy người từ vùng chung; gate vẽ tùy ý giữa vùng trống mà không chia được vùng |
| S4 | Start không thuộc A. Đoạn thẳng từ start_xy tới **mọi vị trí tâm thuộc A** bị chặn, trong khi có đường đi vòng hợp lệ tới A | Chỉ đường tới tâm người bị chặn nhưng vẫn đi thẳng tới một phần A; hoặc không có đường vòng khả thi |
| S5 | `targets` rỗng; start có một lân cận pose hợp lệ với diện tích vị trí dương trong bounds | Còn target ẩn trong fixture; start bị kẹp không có vùng đứng hợp lệ |

Gate chỉ là đoạn cắt dùng xác minh topology, không phải tường thật và không xuất hiện trong sensing/policy. Fixture S3 có trường `scenario_evidence.gates`, ít nhất hai phần tử `{id, segment_m: [[x1,y1],[x2,y2]]}` hữu hạn, hai đầu khác nhau. Khi đóng gate, cấm tâm robot cắt đoạn đó ở mọi yaw. Xét liên thông trong F: ít nhất có vùng chung chứa start và hai thành phần nhánh khác nhau; mở từng gate phải nối nhánh tương ứng với vùng chung bằng đường thật. Validator tự kiểm tra mỗi nhánh có miền đứng diện tích dương, không tin chứng nhận fixture khai sẵn hoặc đồ thị điểm bỏ qua footprint. Các scenario khác không có trường này. Đặc trưng có thể chồng nhau; nhãn chính được khóa trước khi chạy, không đổi nhãn sau khi thấy kết quả, và lưu toàn bộ traits đã xác minh.

Không dùng thất bại của planner hoặc raster thô để kết luận “không có đường/không thấy”. Chứng minh dương dùng đường swept-footprint thực. Chứng minh âm S2/S3/S4 phải bao phủ miền liên tục bằng hình học chính xác hoặc bao bảo thủ có sai số khai báo. Có thể dùng đĩa nội tiếp thân robot để chứng minh một đoạn chắc chắn bị chặn bất kể yaw; không dùng đĩa ngoại tiếp để tuyên bố robot không thể qua. Chưa chứng minh được điều kiện phổ quát thì trả `UNVERIFIED`, không `PASS`.

Validator lưu `fixture_validation.json`: phiên bản verifier, fixture/config hashes, traits, witness path/time, gate components, phương pháp/sai số và `PASS|FAIL|UNVERIFIED` kèm lý do. Chứng nhận bị vô hiệu khi đổi hình học, sensor, footprint hoặc budget; nằm riêng ở evaluator, không truyền cho policy. Các test dương/đối chứng và giới hạn của verifier hiện có được ghi tại [P1a.2 audit](benchmark-p1a2-audit.md); chưa chứng minh được thì vẫn phải trả UNVERIFIED.

## 4 Observation và oracle

Policy chỉ nhận pose, thời gian, bounds nhiệm vụ, scan, costmap từ sensing và observation của detector. Không truyền `Simulation`, `person`, `targets`, obstacle GT hoặc evaluator cho policy. Thuộc tính GT trong simulator không phải lỗi; việc cấp tham chiếu đó cho policy mới tạo rủi ro rò dữ liệu.

Oracle dùng camera có extrinsics/FOV/range đã khai báo. Các giá trị camera trong ví dụ là giả định 2D, không phải calibration Astra. Xét ray từ camera tới tâm target: tâm phải nằm trong FOV và range; không có obstacle khác chắn đoạn ray. Bỏ chính box target khi kiểm tra ray này. Mô hình chỉ xét visibility của tâm, không mô phỏng phần trăm cơ thể nhìn thấy.

Mỗi bước thấy được tạo raw detection. Sau `confirm_frames` bước thấy liên tiếp, phát confirmed observation và cập nhật `last_confirmed_seen_s`. Khi mất visibility, giữ estimate tới hết `memory_s` kể từ mốc này; raw detection không liên tiếp chưa tạo xác nhận mới. Oracle trả vị trí chính xác theo thiết kế; vì vậy không dùng sai số 0 để đánh giá camera.

Observation gồm `observation_id`, `label`, `x_m`, `y_m`, `frame_id`, `last_confirmed_seen_s`, `visible` và `confirmed`. ID do sensing sinh theo thứ tự, không lấy GT target ID. Policy không được thay đổi bản ghi sensing. Cấu hình camera/confirmation là dùng chung, không lấy riêng từ policy B0 hay B1.

## 5 Baseline và phạm vi thay đổi

### 5.1 Quy tắc B0

1. Quét tại start bằng logic spin/dwell dùng chung của `SearchMission`.
2. Sinh lưới cố định với bước `b0_spacing_m`, neo tại start và xoay theo yaw ban đầu. Điểm `(kx, ky)` có vị trí thế giới `start_xy + R(yaw0) * spacing * (kx, ky)`, với kx/ky nguyên. Giữ những điểm nằm trong bounds nhiệm vụ, bỏ `(0, 0)`. Chỉ dùng bounds công khai để giới hạn lưới.
3. Sắp xếp theo bộ khóa `(max(abs(kx), abs(ky)), angle, kx, ky)`, với `angle = atan2(ky, kx)` chuẩn hóa vào `[0, 2*pi)`. Thứ tự này cố định từ vòng gần ra xa, trong vòng ngược chiều kim đồng hồ từ trục tiến ban đầu.
4. Mỗi lần chọn goal, duyệt lại danh sách theo thứ tự trên. Điểm phải chưa được chọn, ô đã biết trống, thuộc miền ứng viên chung của §5.2 và thỏa blacklist/revisit/min-distance chung. Mọi kiểm tra dựa trên bản đồ robot, không dùng GT.
5. Điểm chưa biết, bị chặn hoặc ngoài rolling costmap được tạm bỏ qua, không xóa. Điểm đã giao chỉ chọn một lần, kể cả tiếp cận thất bại; lưu outcome, index và lý do bỏ qua. Giới hạn này phải được kiểm tra qua positive controls bên dưới.
6. Nếu không có waypoint đủ điều kiện, dùng **điểm dự phòng từ sensing**: lấy cùng tập ô ứng viên của B1, nhưng không dùng gain score, min-gain hoặc focus weighting. Sắp ô theo `(max(abs(u),abs(v)), angle, world_cell_x, world_cell_y)`, với `(u,v) = R(-yaw0)*(point-start_xy)/b0_spacing_m`; chọn ô hợp lệ đầu tiên. `angle` chuẩn hóa như bước 3. Không cấp bản đồ/đích GT cho bước này.
7. Khi tới điểm, dùng spin/dwell chung. Không có ứng viên không được tự ghi “đã tìm hết”. Adapter benchmark của **cả B0/B1** giữ trạng thái `POLICY_STALLED`, yêu cầu dừng qua monitor và tiếp tục sensing tới deadline. Có confirmed target mới/hợp lệ thì vào approach ngay; nếu chưa có, thử chọn lại mỗi `spin_dwell_s` thời gian mô phỏng, không quay bù chỉ cho một policy. Hết budget là `TIMEOUT`, lưu mốc stalled đầu và tổng thời gian stalled. Quy tắc này thay riêng cách xử lý hết goal trong harness; không sửa `SearchMission` mặc định. `SEARCH_EXHAUSTED` dành cho mở rộng có chứng nhận hoàn tất riêng, không phát trong cặp T1 này.

Đây là B0 `fixed-order-v2`, một baseline có thứ tự cố định và dự phòng, không phải baseline bảo đảm đầy đủ. Trước khi khóa test, phải chạy positive controls trên dev: S1 dễ thấy, S2 có điểm nhìn gần, S3 có nhánh đi được và S4 có đường vòng. Kiểm tra baseline thực sự chọn goal, di chuyển khi cần, qua được vùng chuyển tiếp và có thể hoàn thành từng loại bằng cấu hình chung. Chưa đạt control do lỗi chọn goal/adapter/budget thì kết quả chỉ là chẩn đoán, chưa dùng kết luận B1 tốt hơn. Không xóa episode B0 thất bại; sửa trên dev, tăng policy version rồi chạy lại cả cặp và khóa trước test.

Luôn báo `goals_selected`, `fallback_goals`, `first_stalled_s`, `stalled_sim_s`, `unresolved_waypoints`, path và confirmed/report timing. Trình bày tỷ lệ hoàn thành theo cùng budget; đường ngắn/thời gian ngắn của B0 thất bại không phải lợi thế. Test đơn vị selector không thay thế positive controls bằng rollout; trạng thái controls thực chạy được ghi tại [P2.1 audit](benchmark-p21-audit.md).

### 5.2 B1 và phần dùng chung

B1 giữ score của `SearchMission.choose_goal()`; chỉ adapter benchmark xử lý hết goal theo §5.1. Miền ứng viên chung lấy từ costmap đã quan sát: ô raw-free, `grid.allowed`, nằm một ô bên trong vùng allowed, gần ô unknown hoặc chưa được camera quan sát (dilation hai ô như core), `pose_free` tại yaw hiện tại, footprint nằm trong bounds, và thỏa blacklist/revisit/min-distance chung. B1 còn dùng min-gain/focus/score hiện tại; B0 dùng thứ tự cố định. Mỗi policy có bản đồ/trạng thái riêng sinh từ đường đi của nó. Planner dựa trên sensing; đi ra bounds là thất bại, không cắt/sửa pose.

| Được khác                                        | Phải giống nhau                                                          |
| ------------------------------------------------ | ------------------------------------------------------------------------ |
| Bộ chọn goal khám phá và trạng thái riêng của nó | Fixture, pose xuất phát, mô hình quan sát/pose và động học, dt và budget |
| Policy ID và cấu hình riêng đã công bố           | Navigator/planner, recovery, spin/dwell, approach, monitor và speed caps |
| Thứ tự chọn điểm do policy quyết định            | Reporting adapter, vùng tiếp cận, stop check và evaluator                |

Snapshot `NavConfig`, monitor config, `SearchConfig`, standoff, chế độ planner và lớp kiểm tra chung. Không thay planner giữa B0/B1. Việc giữ trạng thái GT ở evaluator không cho phép B0 dùng nó để lọc waypoint.

“Cùng pose” chỉ đúng với điều kiện khởi tạo và cách đo pose noiseless. B0/B1 **không dùng chung quỹ đạo** hoặc chuỗi observation: action khác làm pose, scan, coverage và detection về sau khác nhau. Không replay pose của B1 cho B0. Chỉ test tính độc lập GT mới cấp cùng observation/trạng thái cho hai lần gọi để kiểm tra cùng action.

## 6 Báo cáo và xác nhận dừng

### 6.1 Nội dung và matching

Reporting adapter lấy ý định báo cáo từ mission, không lấy tọa độ GT để tự tạo báo cáo. Với core hiện tại, lần đầu `found` được đặt có thể kích hoạt ý định này. Adapter dùng class và observation ID của sensing; đóng băng nội dung ngay khi nộp.

```json
{
  "report_id": "report-0001",
  "label": "person",
  "x_m": 2.0,
  "y_m": 0.0,
  "frame_id": "benchmark_map",
  "observation_id": "obs-0012"
}
```

Runner gắn `received_sim_s` và `step_index`; không tin timestamp do policy tự khai để chấm deadline. Báo cáo hợp lệ về dữ liệu phải có đủ trường, giá trị hữu hạn, frame đúng, tham chiếu confirmed observation cùng episode và tuổi không vượt `camera.memory_s` tại lúc nhận. Class phải khớp observation; việc khớp GT được xét riêng ở bước matching.

Trong episode một người, match khi class đúng và khoảng cách Euclidean 2D từ báo cáo tới target GT không vượt `report_tolerance_m`. Không match nếu thiếu người, sai class hoặc vượt ngưỡng. Target identity chỉ được ghi vào kết quả evaluator. So sánh `distance <= report_tolerance_m` bằng số thực binary64, không cộng geometry epsilon vào ngưỡng vị trí. Test giá trị biên dùng tọa độ biểu diễn chính xác như khoảng cách 0,5 m trên một trục.

Chỉ báo cáo đầu tiên được chấm. Các báo cáo sau bị log là duplicate và không tạo cơ hội sửa tọa độ; adapter bình thường chỉ phát một lần. Báo cáo sai cấu trúc hoặc tham chiếu quan sát không hợp lệ được ghi `INVALID_REPORT`, không được chọn lại một báo cáo khác.

### 6.2 Trạng thái sau khi nộp

`SEARCHING -> REPORT_SUBMITTED -> VERIFY_STOP -> FINISHED`.

Ngay khi nộp, khóa report, ngừng gọi policy chọn đường và yêu cầu vận tốc 0 qua luồng monitor mô phỏng dùng chung. Không ép pose hoặc vận tốc thực thi về 0 trong dữ liệu để làm đẹp phép chấm. Không trả kết quả GT matching cho policy trước khi kết thúc.

Xác nhận dừng khi có `hold_steps` interval liên tiếp sau lúc nộp, mỗi interval thỏa `hypot(vx_checked, vy_checked) <= linear_speed_m_s` và `abs(wz_checked) <= angular_speed_rad_s`. Counter reset khi vi phạm. Mặc định là ba interval, tương đương 0,3 giây mô phỏng. Giữ kiểm tra collision và bounds trong cả quá trình.

Budget nhiệm vụ không dừng lại trong `VERIFY_STOP`. Hết `stop_check.timeout_s` mà chưa xác nhận thì `STOP_NOT_VERIFIED`; hết budget trước đó thì `TIMEOUT`. Trong T1 đây chỉ là kiểm tra vận tốc động học, không chứng minh tính năng phanh của phần cứng.

### 6.3 Vùng tiếp cận và an toàn

Vùng tiếp cận `A` là tập pose `(x,y,yaw)` thỏa đồng thời các điều kiện dưới đây, được tính từ GT/cấu hình chung; không phải một goal point và không lấy boolean `ARRIVED` làm bằng chứng duy nhất. Tại cuối interval xác nhận dừng, cần thuộc A:

- Khoảng cách tâm robot–target không vượt `body_length/2 + person_standoff_m + goal_tolerance`.
- Sai lệch hướng tới target không vượt `align_tolerance_deg`.
- Khoảng cách Euclidean giữa footprint chữ nhật có hướng và box người ít nhất bằng `stop_margin`, đồng thời lớn hơn `geometry_epsilon_m`; footprint chứa trong bounds đóng và cách mọi obstacle lớn hơn geometry epsilon.
- Tâm người raw-visible theo camera thực ở pose cuối (§4), không chỉ còn estimate trong memory. Góc hướng dùng sai số yaw chuẩn hóa vào `[-pi, pi]`; khi tâm trùng người thì loại bởi điều kiện footprint.

Các ngưỡng khoảng cách/hướng dùng `<=`, clearance dùng `>= stop_margin` và `> geometry_epsilon_m`; không tự thêm epsilon vào standoff hoặc report tolerance. Định nghĩa F ở §3.4 gồm footprint trong bounds đóng, cách obstacle và target hơn geometry epsilon, chưa yêu cầu standoff/hướng/visibility. Đường chứng phải nằm trong F; đích của nó phải ở A. `person_standoff_m = 0.55` không phải khoảng cách tâm robot–người; với body length 0,324 m, phần nửa thân xe là 0,162 m trước khi cộng tolerance.

Evaluator hình học phải độc lập với các cờ của monitor/costmap. Collision xét cả footprint quay và chuyển động giữa các pose. Cần phép kiểm tra liên tục hoặc subdivision có bao bảo thủ cho phần chuyển động chưa lấy mẫu, không chỉ kiểm tra hai đầu đoạn. Quy ước tiếp xúc trong `geometry_epsilon_m = 0.001` m là collision; kiểm tra chưa loại trừ được tiếp xúc cũng xử lý bảo thủ và log. Clearance phải ghi phương pháp và độ chính xác; nếu chưa đo tin cậy thì để `null`.

## 7 Tick và điều kiện kết thúc

Tại bước k, đồng hồ là `t_k = k * dt`. Thứ tự cố định:

1. Sinh scan và observation từ pose hiện tại; cập nhật costmap/coverage từ sensing.
2. Nếu còn `SEARCHING`, chạy mission; nhận command và tối đa một report. Nếu có report, ghi lúc nhận là `t_k`, chuyển sang yêu cầu dừng.
3. Hoàn tất barrier planner của bước này theo §8; lọc command bằng gate/monitor dùng chung.
4. Tích phân tới `t_(k+1)`, ghi command yêu cầu/thực thi và trajectory; kiểm tra swept collision, bounds và trạng thái hữu hạn.
5. Cập nhật stop counter rồi xử lý kết thúc. Chỉ chuyển sang bước kế tiếp khi chưa kết thúc và `t_(k+1) < time_limit_s`.

Ưu tiên cùng interval: collision, ra bounds, invalid state, rồi lỗi runtime/planner. Nếu trạng thái số không hữu hạn khiến không thể kiểm tra hình học, kết thúc `INVALID_STATE` ngay, không coi là không va chạm. Những sự kiện này không bị success ghi đè. Khi stop check hoàn tất trước hoặc đúng deadline, xét theo thứ tự report hợp lệ về dữ liệu, match, rồi vùng tiếp cận; nếu chưa hoàn tất mà đã tới deadline thì timeout. Nếu chưa tới deadline nhưng hết stop-check timeout thì `STOP_NOT_VERIFIED`.

| `end_reason`                                              | Điều kiện                                                                            |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `SUCCESS_REPORT`                                          | Report hợp lệ/match, vùng tiếp cận đạt, stop verified và không có collision trước đó |
| `FALSE_REPORT`                                            | Stop verified nhưng report không match GT                                            |
| `INVALID_REPORT`                                          | Stop verified nhưng cấu trúc/frame/observation của report không hợp lệ               |
| `APPROACH_NOT_SATISFIED`                                  | Report đúng, dừng được nhưng chưa đạt vùng tiếp cận                                  |
| `STOP_NOT_VERIFIED`                                       | Hết thời gian xác nhận dừng trước deadline nhiệm vụ                                  |
| `TIMEOUT`                                                 | Đến deadline nhiệm vụ mà chưa đủ điều kiện kết thúc khác                             |
| `SEARCH_EXHAUSTED`                                        | Dự phòng cho protocol có chứng nhận hoàn tất; B0/B1 T1 hiện không phát lý do này |
| `COLLISION`, `OUT_OF_BOUNDS`                              | Vi phạm hình học/giới hạn nhiệm vụ                                                   |
| `INVALID_STATE`                                           | Pose, command hoặc trạng thái số không hợp lệ                                        |
| `PLANNER_TIMEOUT`, `WALL_TIMEOUT`, `CRASH`, `INTERRUPTED` | Lỗi thực thi, không đồng nhất với thất bại tìm đường thông thường                    |

Core có thể trả `SEARCH_COMPLETE`, nhưng adapter T1 đổi thành `POLICY_STALLED` như §5.1; tới deadline vẫn là `TIMEOUT`. Report matching được lưu riêng, nên một report đúng trước crash/collision hoặc timeout khi chờ dừng vẫn có thể có `report_correct = true`, nhưng `mission_success = false`.

## 8 Planner và cách ly episode

Dùng core planner hiện tại ở chế độ async có barrier benchmark. Nếu có future được tạo trong bước k, chờ nó hoàn tất trước khi tiến thời gian mô phỏng; core chỉ nhận kết quả ở lần kiểm tra của bước tiếp theo như luồng hiện tại. Không bỏ qua barrier theo tốc độ máy. Điều này khác mô phỏng độ trễ xử lý thật và phải được ghi trong manifest.

Quá `planner_wall_timeout_s` thì kết thúc `PLANNER_TIMEOUT`. Chạy mỗi episode trong worker process riêng; parent áp dụng `episode_wall_timeout_s`, đóng/thu hồi worker và ghi kết quả lỗi nếu worker treo, kể cả lúc cleanup executor. Sau mỗi episode, reset Navigator, costmap, SearchMission, monitor, detector, planner và RNG; đóng future/executor trong `finally` khi có thể.

Trong suite này môi trường và oracle không ngẫu nhiên sau khi fixture đã khóa. Nếu sau này thêm nhiễu, dùng luồng RNG riêng cho từng nguồn, có chỉ số thời gian cố định để số lần policy gọi hàm không làm đổi nhiễu giữa B0/B1. Cùng seed chưa đủ để tuyên bố deterministic: phải kiểm tra trajectory và chuỗi sự kiện thực chạy. Các trường wall-time được loại khỏi so sánh replay.

Runner không import `rospy`, mở camera, khởi động ROS node, gọi launcher/SSH hoặc kết nối chassis. Các module core có hậu tố `ros1` không bị cấm chỉ vì tên; cần kiểm tra tác dụng thực tế khi import.

## 9 Log và metric

### 9.1 File đầu ra

| File                         | Nội dung                                                                                                                                 |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `run_manifest.json`          | Protocol, SHA, dirty flag/digest source, suite/config hashes, registry environment/hash/family/split, cấu hình, runtime, policy/planner mode |
| `episodes.csv`               | Một dòng cho attempt chính của mỗi episode đã bắt đầu                                                                                    |
| `events.jsonl`               | Timeline có định danh attempt, gồm cả attempt lỗi/chạy lại                                                                               |
| `trajectory.csv`             | Pose GT từng interval cùng command yêu cầu/thực thi và monitor state                                                                     |
| `attempts.csv`               | Tất cả attempt, phục vụ rà soát retry                                                                                                    |
| `summary.json`, `summary.md` | Kết quả theo scenario và tổng thể, có mẫu số và end reasons                                                                              |

Parent ghi nhận thứ tự sự kiện từ worker và giữ journal. Mỗi attempt có kết quả riêng, chỉ đánh dấu hoàn tất sau khi lưu atomically. CSV/summary được tạo lại từ kết quả đó để resume không nhân đôi. Nếu crash trước kết quả cuối, parent chốt attempt lỗi từ journal đã nhận; các phần chưa biết là `null`, không điền 0.

`episodes.csv`/`attempts.csv` giữ `environment_id`, `layout_hash`, `layout_family_id`, `fixture_hash`, split và các chẩn đoán B0 ở §5.1. Mỗi JSONL record có `schema_version`, `run_id`, `episode_id`, `attempt_id`, `event_seq`, `step_index`, `sim_time_s`, `event_type`, `source`, `payload`. `event_seq` tăng trong attempt; thứ tự này phân xử các sự kiện cùng timestamp. JSON không có NaN/Infinity. Nếu đầu ra policy chứa số không hữu hạn, log tên trường và lỗi với giá trị đã làm sạch; không ghi nguyên giá trị đó thành JSON không chuẩn.

```json
{
  "schema_version": "t1-0.2",
  "run_id": "pilot-current-001",
  "episode_id": "S1-DEV-001",
  "attempt_id": 1,
  "event_seq": 18,
  "step_index": 12,
  "sim_time_s": 1.2,
  "event_type": "TARGET_REPORT",
  "source": "reporting_adapter",
  "payload": {
    "report_id": "report-0001",
    "label": "person",
    "x_m": 2.0,
    "y_m": 0.0,
    "frame_id": "benchmark_map",
    "observation_id": "obs-0012"
  }
}
```

Event types tối thiểu: `EPISODE_START`, `RAW_DETECTION`, `CONFIRMED_DETECTION`, `MISSION_STATE`, `GOAL_SELECTED`, `GOAL_SKIPPED`, `POLICY_STALLED`, `TARGET_REPORT`, `REPORT_EVALUATED`, `DUPLICATE_REPORT`, `STOP_VERIFIED`, `MONITOR_INTERVENTION`, `RECOVERY`, `COLLISION`, `ERROR`, `EPISODE_END`. GT match chỉ nằm trong log evaluator, không phản hồi vào policy. Payload có schema theo event type; lỗi ghi loại lỗi và thông báo cần thiết, không dump môi trường hoặc dữ liệu nhạy cảm.

### 9.2 Quy tắc tính

- `report_correct`: báo cáo đầu đúng cấu trúc/quan sát, trong budget, đúng class và trong ngưỡng vị trí.
- `mission_success`: `report_correct` và `end_reason == SUCCESS_REPORT`.
- `search_found`: đã có confirmed observation, độc lập với hai cờ trên.
- `n_false_reports`: số báo cáo chính thức đầu không hợp lệ hoặc không match; duplicate ghi riêng. Cảnh trống có bất kỳ báo cáo đầu nào đều có false alarm, kể cả sai cấu trúc.
- ReportSuccess/SafeMissionSuccess chỉ tính trên episode có người đã bắt đầu. Crash trước report cho hai giá trị false; crash sau report đúng giữ report_correct nhưng mission_success false.
- Nhóm không có episode có người: hai success rates là `null`. Nhóm không có episode trống: negative false-alarm rate là `null`. Khi mẫu số dương mà không có thành công thì tỷ lệ là 0.
- `path_length_m` là tổng dịch chuyển GT; `elapsed_sim_s` tính cả VERIFY_STOP. Lưu `elapsed_wall_s` riêng. Dữ liệu dở dang có cờ `trajectory_complete = false`.
- Thời gian raw/confirmed/report/stop là các trường riêng. Chưa có sự kiện là `null`; sự kiện ở t = 0 là 0 hợp lệ.
- `position_error_m` ghi cho report có tọa độ hữu hạn trong đúng frame kể cả vượt ngưỡng; không có GT, sai frame hoặc tọa độ không hợp lệ thì `null`. Gắn `perception_mode = oracle`.
- Đếm monitor intervention theo đợt liên tiếp cùng trạng thái chặn, không cộng mỗi tick thành một lần mới. Collision dừng episode nên có tối đa một collision event được chấm trong attempt.
- SPL và oracle coverage chưa được hỗ trợ thì ghi `null` cùng lý do, không tạo số thay thế. Estimated coverage ghi đúng tên.

So sánh B0/B1 chỉ ghép cùng episode/fixture hash, environment/layout hash/family/split, cấu hình chung, seed và protocol. Hash giống nhau không đủ nếu start/target khác. Thống kê theo layout dùng environment đã kiểm tra registry; các layout cùng family phải được giữ cùng cụm khi bootstrap. Báo riêng số episode dự kiến, đã bắt đầu, hoàn tất và lỗi. Thống kê chính dùng attempt đầu; retry không tự thay thất bại bằng lượt tốt hơn. Nếu thử cấu hình mới, tạo run mới và công bố nó.

## 10 CLI dự kiến và resume

CLI hiện có dùng `benchmark_batch.py --catalog ... --bundle ... --output ... --policy b0-fixed-order-v2|b1-gain-distance-focus-v1`; bundle được tạo trước bằng `python -B -m benchmark --catalog ... --output ...` từ `test version/`. Xem [README](../test%20version/benchmark/README.md) cho lệnh đầy đủ. Tên CLI dự kiến bên dưới chưa phải alias hiện có. Thêm selector không thay scoring protocol t1-0.2; policy identity/config/source được khóa riêng trong manifest và được kiểm tra khi resume/replay.

Entry point dự kiến là `test version/benchmark_search.py`, đặt cùng cấp module core để giữ import rõ ràng. Các lệnh dưới đây là hợp đồng cần triển khai, chưa phải lệnh hiện có:

```text
python "test version/benchmark_search.py" --suite "test version/benchmark/scenarios/pilot.json" --validate-only
python "test version/benchmark_search.py" --suite "test version/benchmark/scenarios/pilot.json" --policy baseline --output "benchmark-results/pilot-b0" --headless
python "test version/benchmark_search.py" --suite "test version/benchmark/scenarios/pilot.json" --policy current --output "benchmark-results/pilot-b1" --headless
```

`--policy baseline|current` chọn B0/B1. Runner luôn offline/headless, kể cả không ghi `--headless`. `--max-episodes N` lấy prefix theo thứ tự suite, N nguyên dương; `--episode-id ID` chọn lượt cần tái hiện. `--seed N` chỉ lọc seed đã có trong fixture, không đặt lại hình học hoặc sinh thêm episode; các bộ lọc kết hợp theo AND, không có kết quả thì báo lỗi.

`--validate-only` không chạy policy và không tạo kết quả benchmark. `--resume` yêu cầu manifest/config/source fingerprints khớp, bỏ qua attempt đã chốt, và khôi phục log dở dang thành attempt lỗi trước khi tiếp tục. `--retry-errors` chỉ đi cùng resume, tạo attempt mới cho lỗi thực thi; không thay attempt chính hoặc tự chạy lại các timeout/collision/false report của nhiệm vụ.

Chỉ ghi dưới `--output`. Nếu output đã chứa run khác, từ chối ghi đè. Không sửa YAML production hay commit dữ liệu sinh ra. Mã thoát: 0 khi các lượt được chọn đã chạy và ghi kết quả bình thường, kể cả mission failure; 1 khi có lỗi thực thi/journal; 2 khi cấu hình/đầu vào/resume không hợp lệ; 130 khi bị ngắt. Process tiếp tục episode sau một worker lỗi nếu còn điều kiện ghi kết quả.

## 11 Acceptance tests trước pilot

Các test dưới đây là yêu cầu nghiệm thu, chưa phải kết quả đã chạy:

| Nhóm      | Trường hợp bắt buộc                                                                                                                                              |
| --------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Schema    | ID trùng, NaN, box đảo cạnh, số mục tiêu sai, pose va chạm, đường dẫn thoát repo và budget không chia hết dt bị từ chối; gọi CLI từ cwd khác vẫn resolve đúng |
| Layout    | Sai hash; cùng ID khác hình học; cùng hình học đổi ID; dev/test chung hash/family bị từ chối; reorder/ID/chia box/tịnh tiến không đổi hash; đổi bounds/obstacle đổi hash; fixture vật lý trùng dù đổi seed/ID không nhân mẫu |
| Scenario  | Mỗi S1–S5 có fixture dương và đối chứng ở §3.4; gate giả, đường qua khe hẹp, detour tới tâm nhưng không chặn A, chứng nhận stale/UNVERIFIED đều bị từ chối |
| Hình học  | Vật cản mỏng giữa hai endpoint; góc thân quét vào vật khi quay; target là vật cản; bounds và tolerance tiếp xúc                                                  |
| Oracle    | Ngoài FOV/range; bị che; hai frame liên tiếp; mất dấu và hết memory; ID không chứa GT identity                                                                   |
| Isolation | Cùng observation và trạng thái policy cho cùng action dù thay GT ẩn; cấm truy cập simulator/evaluator; không import ROS hoặc mở camera                           |
| B0        | Thứ tự lưới/frame; unknown xét lại; waypoint đã giao không lặp; không có waypoint nhưng có frontier thì chọn dự phòng; stalled vẫn nhận target và retry theo dwell, hết budget là timeout ở cả hai; positive controls S1–S4 |
| Pose      | Cùng start/model; Euler tiến/ngang/quay và qua ±pi có kết quả tính tay; action khác tạo quỹ đạo/scan khác, không ép bằng nhau; F/A kiểm tra biên bounds, clearance, hướng, visibility và swept path |
| Matching  | Đúng bằng r được nhận; vượt r bị loại; sai class/frame/obs ID và stale bị loại; S5 không thể match; duplicate không sửa report đầu                               |
| Dừng      | Một interval còn chuyển động reset counter; ba interval đạt thì verified; timeout khi đang chờ dừng giữ ReportSuccess nhưng MissionSuccess false nếu report đúng |
| Ưu tiên   | Report đúng cùng interval collision vẫn MissionSuccess false; hoàn thành stop đúng deadline được nhận; exhausted ở deadline là timeout                           |
| Lỗi       | Planner treo, worker crash và cleanup treo đều chốt được kết quả; report đúng trước crash không bị mất khỏi journal                                              |
| Metrics   | Mẫu số 0 trả null, mẫu số dương với 0 success trả 0; thời điểm 0 khác null; đếm đợt monitor và tính path theo ví dụ tính tay                                     |
| Resume    | Gián đoạn ghi, CSV tái tạo, attempt ID và retry không nhân đôi hoặc thay attempt chính; manifest khác bị từ chối                                                 |
| Replay    | Cùng fixture/config tạo cùng trajectory và semantic events; bỏ riêng wall-time khỏi phép so sánh; chạy regression core hiện có                                   |

P0 chỉ mở cổng **bắt đầu P1a** khi tài liệu nhất quán và regression core không có lỗi chưa giải thích. P1a phải hiện thực schema/registry/verifier cùng các đối chứng; chưa chứng nhận S1–S5 thì chưa khóa suite, chưa chuyển sang rollout đánh giá. Sau acceptance tests runner và pilot mới khóa test và so sánh thuật toán. Việc hoàn thành đặc tả không thay thế xác minh triển khai.
