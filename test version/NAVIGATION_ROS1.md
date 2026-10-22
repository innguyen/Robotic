# Local navigation for JetAuto on ROS 1 Melodic

This implements the planner/controller/costmap separation used by Nav2, with a
small original Python implementation for the existing Ubuntu 18.04 robot.
It is not Nav2 itself or a source-compatible port. No upstream code is copied.

Architecture reference:
https://docs.nav2.org/rolling/getting_started/navigation_concepts/navigation_servers/

```text
Astra RGB/depth -> jetauto_find_object.py -> /vision/target (odom position + class)
LiDAR + odometry -> navigation_ros1.py (costmap -> A* -> Mecanum controller)
                   -> /collision_monitor/cmd_vel_raw
LiDAR -> collision_monitor_ros1.py
                   -> /collision_monitor/cmd_vel_checked [preview default]
                   -> /jetauto_controller/cmd_vel        [explicit live mode]
```

## Implemented behavior

- Vision confirms a COCO target and publishes an atomic JSON message on
  `/vision/target`: `valid`, `stamp`, `frame_id`, `label`, `x`, `y`, `source`.
  Position is in `odom`; odometry near the RGB timestamp anchors the target.
  Publication never refreshes the original observation timestamp while remembering.
  Missing distance or matched odometry means no navigable target is published.
- Scan ray tracing marks observed free cells, occupied endpoints and unknown cells.
  A 4.86 m rolling window uses 6 cm cells and 4 s observation memory in odom.
  LiDAR is matched to odometry within 0.15 s. Scan/odom stamps and frame names are
  checked. An odometry jump clears geometry, discards the goal and pauses navigation.
- The displayed obstacle buffer is 0.06 m (one 6 cm cell). Chassis clearance is
  checked separately using the fixed-heading rectangular footprint plus that buffer
  against measured LiDAR hits, replacing the former 0.36 m circular approximation.
  A* avoids footprint collisions, unknown cell centers and diagonal
  corner cutting; replanning runs at up to 2 Hz, or earlier if the path is blocked.
- The approach point respects class standoff (person 0.55 m, chair 0.40 m,
  default 0.35 m from front extent). A multi-goal A* searches all reachable points
  around the complete standoff ring, rather than a few predetermined angles.
  If the ring is not reachable yet, it can use an observed frontier that makes
  progress as a temporary observation point; unknown cells remain blocked.
- A 10 Hz fixed-heading holonomic controller produces forward/sideways/diagonal
  velocities. It checks the lookahead segment and limits acceleration and speed.
  The Collision Monitor independently filters every proposed command.
- Missing/stale odometry, scan, target or monitor state causes zero commands.
  No progress for 8 s causes `STUCK`; a changed goal or pause/re-enable resets it.
- `/jetauto_navigation/costmap` and `/jetauto_navigation/path` are standard
  `nav_msgs/OccupancyGrid` and `nav_msgs/Path` for RViz. State is JSON on
  `/jetauto_navigation/state`. The navigator starts paused.

## Files and requirements

For the combined Windows launcher, double-click `run_test_robot.bat`. It performs
the upload, driver check, frame detection and starts all three nodes in preview.
Use Start/Pause on the camera page; no separate enable-service terminal is needed.
See README.md for optional live-mode and target arguments. The manual instructions
below remain available for diagnosis or running individual components.

Copy all test-version Python files, both YAML files and the shell launcher into
`/home/jetauto/test_version/`. Use the existing ROS 1 Python 3 environment; the pair
needs NumPy, rospy, rosgraph, geometry_msgs, sensor_msgs, nav_msgs, std_msgs and
std_srvs. Vision additionally needs OpenCV DNN, optionally ONNX Runtime.

From Windows PowerShell at the repository root:

```powershell
ssh jetauto@192.168.149.1 "mkdir -p /home/jetauto/test_version"
$testFiles = Get-ChildItem 'test version' -File | Where-Object { $_.Extension -in '.py', '.yaml', '.sh' }
scp $testFiles.FullName jetauto@192.168.149.1:/home/jetauto/test_version/
```

No OS upgrade or ROS 2 installation is needed. Do not use the original
`start_server.sh`: it starts the old application and can bypass the new filter.
Start the existing chassis/odom/Astra/LiDAR drivers separately using the robot's
existing environment. This launcher does not start, restart or kill vendor drivers.

## Check frames before running

On the robot, after sourcing its ROS environment:

```bash
rostopic echo -n 1 /scan/header
rostopic echo -n 1 /odom/header
rostopic echo -n 1 /odom/child_frame_id
```

`navigation_ros1.yaml` defaults to `odom` and `base_footprint`. Set them to the
actual names if different. Vision uses the same `~odom_frame`/`~base_frame` private
parameters. Confirm the measured LiDAR mount in the monitor YAML (x = 0.096 m,
y = 0, yaw = 0); frame-name matching alone cannot confirm calibration. Fixed planar
geometry is used, not arbitrary TF transforms.

## Preview first: outputs do not connect to motors

Replace `YOUR_SCAN_FRAME` with `/scan/header/frame_id`:

```bash
cd /home/jetauto/test_version
sed -i 's/\r$//' start_navigation_ros1.sh
bash start_navigation_ros1.sh YOUR_SCAN_FRAME preview
```

In a second robot terminal:

```bash
python3 /home/jetauto/test_version/jetauto_find_object.py bottle --models-dir /home/jetauto/models --port 5010
```

Open `http://192.168.149.1:5010` to select a target. With drivers publishing valid
data and a confirmed target, enable generation of **preview** velocity commands:

```bash
rosservice call /jetauto_navigation/enable "data: true"
```

Inspect (separate terminals, or interrupt each command before running the next):

```bash
rostopic echo /vision/target
rostopic echo /jetauto_navigation/state
rostopic echo /jetauto_collision_monitor/state
rostopic echo /collision_monitor/cmd_vel_checked
```

Pause immediately with:

```bash
rosservice call /jetauto_navigation/enable "data: false"
```

Check obstacle detours in the published path, arrival distance, target disappearance,
and stale sensor behavior before connecting the chassis. Since preview does not
move the real robot, continued nonzero requests can trigger `STUCK`; pause/re-enable
to restart a preview. Any old motion application must be stopped separately: a
preview node does not stop another controller.

## Live integration

Stop the preview pair with Ctrl+C. Stop other movement applications while keeping
sensor/chassis drivers running. The launcher checks for existing publishers on
both `/cmd_vel` and `/jetauto_controller/cmd_vel`, and refuses live startup if found.

```bash
bash /home/jetauto/test_version/start_navigation_ros1.sh YOUR_SCAN_FRAME live
```

It still starts paused. Verify topic connections, then use the same enable service
when ready to test at low speeds. In live mode only the Collision Monitor may
publish to the chassis topic; `/cmd_vel` must have no bypass publisher. The launcher
checks only at startup; do not start vendor apps/teleop that bypass it later.

## Current limitations

### Update after reviewing robotic-1.mp4

The recording shows brief `FOLLOW_PARTIAL`, interrupted by `WAIT_TARGET`,
`SCAN_NEEDS_MATCHED_ODOMETRY` and `STALE_ODOM_STAMP`; it is already in live mode.
The adapter now queues only the newest scan and performs ray tracing/planning
outside the callback lock. It rechecks source age and pause state before publishing
each command, and can retry a scan awaiting near-time odometry without refreshing
the scan's observation age. Ray-cell insertion is deduplicated with NumPy.
Neither the source watchdog nor the collision filter was disabled or relaxed.

For chairs, the vision threshold is 0.40 with three consistent observations and
bearing/range jump checks; chair memory remains 5 seconds (navigator target
freshness still 1.2 seconds). Sub-threshold detections are orange. The page shows
confidence versus threshold and hit count. Navigation status additionally reports
`map_ms`, `plan_ms`, `odom_age_s`, `scan_age_s` and `scan_sync_s` for diagnosing
actual Nano load and source timing. Offline tests do not establish that this
resolves the real robot's transport latency; re-run and inspect these measurements.

### Approach planning and map display update

The old selector could stop after checking the direct approach and only six
alternatives within +/-60 degrees. It also did not try other endpoints if a free
direct endpoint was disconnected. Multi-goal A* now searches the complete ring
in reachable known space and can select a temporary observed frontier when the
ring is occluded. `NO_REACHABLE_APPROACH` means neither was found under the current
map, chassis footprint and blind-direction constraints. It does not prove that
the physical room has no possible route after rotating or obtaining more data.

The web map now separates 6 cm inflation (yellow) from robot-center cells excluded
by rectangular footprint checks (dark red, value 98). A white square shows the
chosen approach/observation point, while the green dot remains the detected object.
Navigation status includes `planning.strategy`, `planning.reachable_cells` and
`planning.approach_point`. A visual gap is not sufficient if the chassis does not fit.

Path starts use the actual robot position rather than the center of its grid cell,
avoiding artificial tiny backward commands into blind sectors. Regression tests
cover disconnected direct goals, far-side approaches beyond 60 degrees, observed
frontiers, sub-cell goals and a complete ray-traced detour through the monitor.

### Corrected footprint precision

The first rectangular mask still added sqrt(2) * 0.06 m = 8.49 cm to each half
extent as blanket grid compensation. That was excessively restrictive in real
aisles. The costmap now retains measured LiDAR hit coordinates and computes exact
fixed-heading footprint membership at candidate node centers. Continuous pose and
segment checks use the real robot coordinates, avoiding snap-to-cell-center errors.
There is no fixed 8.49 cm addition. With the configured 32.4 x 26 cm body and 6 cm
margin, footprint half extents are 22.2 cm forward/back and 19 cm sideways.

Constant-heading translation uses an analytic swept rectangle in both the planner
and Collision Monitor; rotating motion keeps its sampled conservative check. Grid
search is still discretized at 6 cm and can reject almost-fitting passages without
an appropriate node. The red cells remain forbidden *robot-center* positions,
not the measured dimensions of obstacles. Regression cases include a 50 cm aisle
that must remain traversable and a 32 cm aisle that cannot fit the body plus margin.

### Scope

- Local navigation only. No SLAM, AMCL, global map, ROS action server, behavior tree
  framework, Nav2 plugins, MPPI or automatic exploration of unseen spaces.
- LiDAR obstacles only. Astra is used for target range, not yet as an obstacle or
  cliff costmap source. Do not treat this version as equivalent to the original
  app's depth-based low-obstacle/cliff protection.
- Fixed-heading control deliberately requests no rotation. Known rear blind
  sectors also restrict A* edges and velocity requests. It can stop without a path
  even where a fully observed robot could rotate or reverse. It does not spin
  blindly when a target disappears; it stops after target expiry.
- Unknown **center cells** are blocked, but observed-center checks are not proof
  that every part of the footprint is visible. Inflation protects against measured
  obstacles, not invisible obstacles. Depth and more complete coverage remain work.
- No-return +Inf rays are considered clear up to the configured ray range. Invalid
  rays remain unknown. Transparent surfaces can be missed by LiDAR.
- Odometry drift remains; history is short-term and assumes static obstacles.
  Motion is planar and scan deskew is not implemented. Target positions depend on
  depth/box accuracy and timing. Standoff is approximate, using body front extent.
- Live driving, measured braking, camera/odom extrinsics, ROS Python dependencies
  and Jetson Nano CPU/memory must be verified on hardware. A killed process cannot
  enforce a stop: an independent driver watchdog is still needed.

## Offline checks

```bash
cd 'test version'
python3 -m unittest -v test_collision_monitor_ros1.py test_navigation_ros1.py
```

Tests cover obstacle detours, unknown and inflated cells, blocked diagonals,
replanning, standoff arrival, progress checks, frame/timestamp validation, target
export and a simulated Mecanum path passed through the monitor. These are offline
checks, not evidence of a successful run on the real robot.
# Cập nhật: tìm đường nền và chọn điểm trên bản đồ

Node ROS dùng một worker tìm đường riêng. Vòng nhận scan, dựng bản đồ, kiểm tra
đường với bản đồ mới và xuất vận tốc không chờ A* hoàn tất. Khi chưa có đường,
trạng thái là `PLANNING` và lệnh bằng 0. Một kết quả thuộc mục tiêu cũ hoặc lần
chạy trước khi tạm dừng không được sử dụng. Các giới hạn tuổi cảm biến giữ nguyên.
`plan_ms` đo bước điều khiển; `async_planner_ms` đo lần tìm đường nền đã hoàn tất.

Trên UI: **Chọn điểm trên bản đồ → nhấp vùng trống → Đi tới điểm đã chọn**.
Nút Đi tới điểm gửi yêu cầu bắt đầu điều hướng ngay sau khi node chấp nhận.
Không cần YOLO xác nhận mục tiêu trong chế độ này. Điểm đến dùng hệ tọa độ odom
của bản đồ cục bộ, sai số đến đích theo `goal_tolerance` (mặc định 10 cm), không
áp dụng khoảng dừng theo loại vật thể. Chỉ chọn được ô trống trên bản đồ mới;
node kiểm tra lại điểm và dữ liệu cảm biến trước khi cho chạy.

**Tạm dừng** dừng cả hai chế độ. **Theo mục tiêu camera** chuyển về camera và
tạm dừng; bấm Bắt đầu để chạy tiếp. Reset/jump odometry hủy mục tiêu và dừng.
Chế độ preview vẫn chỉ gửi lệnh vào topic kiểm thử, không tới chassis.
Đây vẫn là bản đồ cục bộ theo odometry, không phải SLAM hay bản đồ toàn cục.

