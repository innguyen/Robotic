#!/bin/bash
# Run the ROS 1 navigation pair. Sensor drivers and vision are separate.
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCAN_FRAME="${1:-}"
MODE="${2:-preview}"
if [ -z "$SCAN_FRAME" ] || { [ "$MODE" != preview ] && [ "$MODE" != live ]; }; then
    echo 'Usage: bash start_navigation_ros1.sh <scan-frame-id> [preview|live]'
    exit 1
fi
source /opt/ros/melodic/setup.bash
set -u
python3 - "$MODE" <<'PY'
import sys
import rosgraph
master = rosgraph.Master('/navigation_startup_check')
publishers = master.getSystemState()[0]
if sys.argv[1] == 'live':
    bypass = [(topic, nodes) for topic, nodes in publishers
              if topic in ('/cmd_vel', '/jetauto_controller/cmd_vel') and nodes]
    if bypass:
        raise SystemExit('Stop existing chassis publishers before live mode: ' + str(bypass))
for node in ('/jetauto_navigation', '/jetauto_collision_monitor'):
    try:
        master.lookupNode(node)
    except rosgraph.MasterError:
        continue
    raise SystemExit('Already running: ' + node)
PY
rosparam load "$TASK_DIR/collision_monitor_ros1.yaml" /jetauto_collision_monitor
rosparam load "$TASK_DIR/navigation_ros1.yaml" /jetauto_navigation
rosparam set /jetauto_collision_monitor/scan_frame "$SCAN_FRAME"
if [ -n "${JETAUTO_ODOM_FRAME:-}" ]; then
    rosparam set /jetauto_navigation/odom_frame "$JETAUTO_ODOM_FRAME"
fi
if [ -n "${JETAUTO_BASE_FRAME:-}" ]; then
    rosparam set /jetauto_navigation/base_frame "$JETAUTO_BASE_FRAME"
fi
if [ "$MODE" = live ]; then
    rosparam set /jetauto_collision_monitor/output_topic /jetauto_controller/cmd_vel
fi
MONITOR_PID=""
NAV_PID=""
cleanup() {
    # Stop producer first, then keep the monitor alive briefly to forward zeros.
    [ -z "$NAV_PID" ] || kill -TERM "$NAV_PID" 2>/dev/null || true
    [ -z "$NAV_PID" ] || wait "$NAV_PID" 2>/dev/null || true
    [ -z "$MONITOR_PID" ] || kill -TERM "$MONITOR_PID" 2>/dev/null || true
    [ -z "$MONITOR_PID" ] || wait "$MONITOR_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
python3 "$TASK_DIR/collision_monitor_ros1.py" &
MONITOR_PID=$!
python3 "$TASK_DIR/navigation_ros1.py" &
NAV_PID=$!
echo "Navigation pair started in $MODE mode; paused until /jetauto_navigation/enable."
# If either process exits, terminate the other instead of leaving it unattended.
wait -n "$MONITOR_PID" "$NAV_PID"
