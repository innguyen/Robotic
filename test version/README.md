# JetAuto vision test

## Start everything from Windows

Double-click **`run_test_robot.bat`** in this directory. It uploads the test code,
models and driver-check helper, verifies sensors, stops the old AI application,
detects scan/odom frames and runs vision + navigation + Collision Monitor together.
Enter the robot SSH password when requested. It does not install dependencies.

Open **http://192.168.149.1:5010**, select a target and use **Bắt đầu / Tạm dừng**.
Default mode is **preview**: commands go to a test topic, not the motors.
For the explicit live shortcut, use **`run_test_robot_live.bat`**. The page shows
the active mode and why navigation is waiting or blocked. It also displays the
local obstacle grid and path next to the camera. This grid is not SLAM.
The old controller remains stopped after the launcher exits. Keep the terminal
open; Ctrl+C shuts down the test nodes while retaining sensor drivers.
Re-running the launcher gracefully replaces a previous test supervisor only after
verifying its script path and startup-lock ownership. Unidentified lock holders
are reported without terminating them; deleting a lock file is not a recovery step.

Optional PowerShell commands from the repository root:

```powershell
& '.\test version\run_test_robot.ps1' -Target 'cell phone'
& '.\test version\run_test_robot.ps1' -Mode live -Target bottle
```

`live` connects the filter to the chassis, but navigation still starts paused until
you press **Bắt đầu**. Validate preview first; live behavior has not been checked on
the real robot. Existing bypass publishers cause live startup to fail.

SSH target is `jetauto@192.168.149.1`. Driver setup uses the bundled original
`start_server.sh --check-only`. The launcher expects existing ROS 1/Python 3 camera
dependencies and permission for the same driver checks as the original launcher.

A separate ROS 1 velocity filter is now available in `collision_monitor_ros1.py`.
See [COLLISION_MONITOR_ROS1.md](COLLISION_MONITOR_ROS1.md) for configuration,
preview testing and chassis integration. Vision does not publish velocity commands;
its web controls call the separate navigation node's start/pause service.

The local planner/controller is in `navigation_ros1.py` and `navigation_core_ros1.py`.
See [NAVIGATION_ROS1.md](NAVIGATION_ROS1.md) for the complete vision -> navigation ->
monitor setup. Vision now publishes `/vision/target` when confirmed and anchored
to matched odometry, but still never publishes motion commands itself.

`jetauto_find_object.py` is a standalone ROS 1 vision test derived from the main application.

It retains Astra RGB/depth input, YOLOv5n, COCO class selection, confidence filtering,
target confirmation and memory, RGB/depth timestamp matching, distance estimation,
and a camera/status web page. `/odom` is optional and read-only for target tracking.
Camera mounting geometry is fixed to the measured defaults; floor calibration is not included.

It does not import `obstacle_avoidance.py`, subscribe to LiDAR, construct a local
obstacle grid, plan motion, publish velocity commands, or offer manual driving controls.
Running this file does not stop another controller that is already running.

## Run on the robot

Use the existing ROS 1 environment with Astra drivers already running, plus NumPy,
OpenCV with DNN, and optionally ONNX Runtime. Python 3.6 or newer is supported.

When the repository structure is preserved:

```bash
python3 'test version/jetauto_find_object.py' bottle --port 5010
```

When the file is uploaded separately to `/home/jetauto/test_version/`:

```bash
python3 /home/jetauto/test_version/jetauto_find_object.py bottle --models-dir /home/jetauto/models --port 5010
```

Open `http://192.168.149.1:5010`. Change the COCO target class on the page.
The existing main `start_server.sh` still launches the original motion application;
it is not a launcher for this test file.

## Validation

Local checks covered Python 3.6 syntax, a synthetic detection/JPEG cycle, target
confirmation and expiration, bounding-box distance fallback, HTTP page/status/target
selection, and the absence of motion publishers/endpoints. ROS drivers, live camera
inference, and Jetson performance still require validation on the robot.
