# Collision Monitor for ROS 1 Melodic

This is an original, small implementation inspired by the stop/slowdown/approach
architecture of Nav2 Collision Monitor. It is not copied Nav2 source, a full Nav2
port, a path planner, or a certified safety controller.

Reference: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/

## What is implemented

- Independent ROS 1 node, Python 3.6+, NumPy, standard ROS messages; no ROS 2.
- Rectangular footprint, fixed stop and slowdown zones, speed caps.
- Forward/lateral/diagonal/yaw Twist support, with proportional scaling of all axes.
- Predicted swept rectangle over reaction plus conservative braking time.
- Wall-clock watchdog publishes zero continuously on stale commands or sensor data.
- Rejected scan frames, stale/non-increasing scan stamps, invalid scans and NaN commands fail closed.
- Status JSON on `/jetauto_collision_monitor/state`.
- Default output is a preview topic, not the motor topic.

Defaults match the local JetAuto configuration: 0.324 x 0.260 m body; 0.06 m stop
margin and 0.25 m slowdown margin measured from the body; slowdown ratio 0.35;
0.25 s reaction time; 0.35 m/s^2 deceleration; 20 Hz output. Angular units are rad/s.

Known rear LiDAR blind sectors block reverse motion and rotation by default.
This is deliberately more restrictive than the original application's remembered
occupancy grid. The monitor has no remembered obstacle map or second sensor to
justify motion there. `block_blind_motion: false` removes this restriction, but does
not make those areas observed. Keep it enabled until a suitable sensor/coverage
strategy is implemented and tested.

## Preview on the robot

Copy `collision_monitor_ros1.py` and `collision_monitor_ros1.yaml` into
`/home/jetauto/test_version/`. Use the same Python/ROS environment as the main app.
Start existing sensor drivers separately; this node does not launch or stop them.

In a robot terminal:

```bash
source /opt/ros/melodic/setup.bash
rostopic echo -n 1 /scan/header
```

Set `scan_frame` in the YAML to that header's `frame_id`. Confirm the scan's fixed
mount position (`scan_x`, `scan_y`, `scan_yaw`) against the robot. The default
position is 9.6 cm forward, yaw 0. This node does not use TF and only supports a
fixed, planar LiDAR transform. Merely matching the frame name does not validate
the mount geometry.

```bash
rosparam load /home/jetauto/test_version/collision_monitor_ros1.yaml /jetauto_collision_monitor
python3 /home/jetauto/test_version/collision_monitor_ros1.py
```

In another robot terminal, inspect output/state:

```bash
rostopic echo /collision_monitor/cmd_vel_checked
rostopic echo /jetauto_collision_monitor/state
```

With no commands it sends zeros. For a preview-only test, publish a low requested
velocity to the raw topic (the YAML output must still be the checked preview topic):

```bash
rostopic pub -r 10 /collision_monitor/cmd_vel_raw geometry_msgs/Twist '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Check stop/slowdown with an obstacle, then interrupt the test publisher and verify
zero output within the command timeout plus one output period. Sensor loss should
also produce zero. An existing controller can still move the robot independently;
preview mode is not a stop mechanism for other running programs.

## Connect to the chassis after preview validation

```text
Controller -> /collision_monitor/cmd_vel_raw
  -> this node -> /jetauto_controller/cmd_vel -> chassis
```

Set `output_topic` to `/jetauto_controller/cmd_vel` in YAML and restart the node.
Every other controller, including teleoperation/vendor apps, must stop publishing
directly to the chassis. The driver also listens to `/cmd_vel`, so check both:

```bash
rostopic info /jetauto_controller/cmd_vel
rostopic info /cmd_vel
```

The only publisher to the chassis must be this filter; commands on `/cmd_vel`
would bypass it. Do not run two monitor instances on the same chassis output.

The original main app can be routed through the filter by a ROS topic remapping
argument placed AFTER its target argument:

```bash
python3 /home/jetauto/jetauto_find_object.py person /jetauto_controller/cmd_vel:=/collision_monitor/cmd_vel_raw
```

That main app still contains its original obstacle planner and movement controls.
Do not use `start_server.sh` for this setup: it restarts the original application
without this remapping. The vision-only file in `test version` produces no velocity
requests at all; a planner/controller must be connected separately.

## Limits and verification

Only LiDAR is implemented. No Astra depth, floor/cliff detection, TF motion
compensation, odometry memory, global planning, or autonomous target following.
The valid-ray fraction is a coarse scan health check, not proof that the requested
path is observed. Obstacles outside the scan plane and unknown ray gaps can be
missed. Sweep prediction assumes stationary obstacles and measured deceleration.
Set reaction time to cover sensor/transport/processing latency in real operation.
If the monitor process or computer fails, the driver must independently time out
commands; this software cannot stop the chassis after its own process is killed.

Offline tests (from this directory):

```bash
python3 -m unittest -v test_collision_monitor_ros1.py
```

Tests exercise geometry, diagonal velocity limits, blind directions, watchdogs,
scan conversion and invalid inputs. ROS transport, sensor mounting, real braking
distance and Jetson performance require hardware validation before live use.
