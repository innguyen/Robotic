#!/bin/bash
# Foreground supervisor for sensors + vision + navigation + collision filter.
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-preview}"
TARGET="${2:-bottle}"
PORT="${3:-5010}"
case "$MODE" in preview|live) ;; *) echo 'Mode must be preview or live'; exit 1 ;; esac
[[ "$TARGET" =~ ^[a-z][a-z\ ]{0,39}$ ]] || { echo 'Invalid target class'; exit 1; }
[[ "$PORT" =~ ^[0-9]+$ ]] && [ "$PORT" -ge 1024 ] && [ "$PORT" -le 65535 ] || { echo 'Invalid port'; exit 1; }
exec 8>/tmp/jetauto_test_stack.lock
if ! flock -n 8; then
    echo 'A previous test stack holds the startup lock. Checking its identity...'
    python3 "$TASK_DIR/launcher_lock_ros1.py" /tmp/jetauto_test_stack.lock "$TASK_DIR/start_all_ros1.sh" "$$" 8>&-
    ACQUIRED=false
    for attempt in 1 2 3 4 5 6 7 8 9 10; do
        if flock -n 8; then ACQUIRED=true; break; fi
        sleep 1
    done
    if [ "$ACQUIRED" != true ]; then
        echo 'Previous test stack has not released its lock. No drivers were killed.'
        echo 'Inspect with: fuser -v /tmp/jetauto_test_stack.lock'
        exit 1
    fi
fi
echo '[1/5] Checking ROS, chassis and camera drivers; pausing old controller...'
[ -f "$TASK_DIR/start_server.sh" ] || { echo 'Missing bundled start_server.sh'; exit 1; }
# Driver children must not retain our supervisor lock.
bash -l "$TASK_DIR/start_server.sh" --check-only 8>&-
source /opt/ros/melodic/setup.bash
for workspace in /home/jetauto/jetauto_ws/devel/setup.bash /home/jetauto/ros_ws/devel/setup.bash /home/jetauto/catkin_ws/devel/setup.bash; do
    [ ! -f "$workspace" ] || source "$workspace"
done
export ROS_MASTER_URI=http://192.168.149.1:11311
export ROS_IP=192.168.149.1 ROS_HOSTNAME=192.168.149.1
echo '[2/5] Stopping old AI application (sensor drivers stay running)...'
APP_PATTERN='^([^ ]*/)?python[23]?([.][0-9]+)?[ ]+(-u[ ]+)?([^ ]*/)?jetauto_(find_object|smart_search)[.]py([ ]|$)'
pkill -TERM -f "$APP_PATTERN" 2>/dev/null || true
for attempt in 1 2 3 4 5; do
    pgrep -f "$APP_PATTERN" >/dev/null || break
    sleep 1
done
if pgrep -f "$APP_PATTERN" >/dev/null; then
    echo 'Old AI application did not stop; startup cancelled.'
    exit 1
fi

echo '[3/5] Preparing Python environment and detecting sensor frames...'
CV2_DIR=''
for candidate in /home/jetauto/.virtualenvs/mediapipe/lib/python3.6/site-packages/cv2 /home/jetauto/.virtualenvs/mediapipe/lib/python3.6/site-packages /usr/local/lib/python3.6/dist-packages; do
    if [ -d "$candidate" ] && PYTHONPATH="$candidate:${PYTHONPATH:-}" python3 -c 'import cv2; assert hasattr(cv2, "dnn")' >/dev/null 2>&1; then
        CV2_DIR="$candidate"
        break
    fi
done
export PYTHONPATH="/home/jetauto/.local/lib/python3.6/site-packages:$CV2_DIR:${PYTHONPATH:-}"
python3 -c 'import rospy, rosgraph, cv2, numpy; from std_srvs.srv import SetBool; from sensor_msgs.msg import LaserScan; assert hasattr(cv2, "dnn")'
FRAME_INFO=$(timeout 20 python3 - <<'PY'
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
rospy.init_node('test_stack_frame_check', anonymous=True)
scan = rospy.wait_for_message('/scan', LaserScan, timeout=8)
odom = rospy.wait_for_message('/odom', Odometry, timeout=8)
frames = [scan.header.frame_id, odom.header.frame_id, odom.child_frame_id]
if any(not frame or any(c.isspace() for c in frame) for frame in frames):
    raise RuntimeError('Missing or invalid sensor frame name')
print(' '.join(frames))
PY
)
read -r SCAN_FRAME JETAUTO_ODOM_FRAME JETAUTO_BASE_FRAME <<< "$FRAME_INFO"
export JETAUTO_ODOM_FRAME JETAUTO_BASE_FRAME
echo "Frames: scan=$SCAN_FRAME, odom=$JETAUTO_ODOM_FRAME, base=$JETAUTO_BASE_FRAME"
python3 - "$TASK_DIR/models" "$TARGET" "$PORT" <<'PY'
import socket, sys
from pathlib import Path
directory = Path(sys.argv[1])
if not (directory / 'yolov5n.onnx').is_file():
    raise SystemExit('Missing yolov5n.onnx')
if sys.argv[2] not in (directory / 'coco.names').read_text().splitlines():
    raise SystemExit('Unknown COCO target class')
with socket.socket() as probe:
    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    probe.bind(('0.0.0.0', int(sys.argv[3])))
PY

PAIR_PID=''
VISION_PID=''
cleanup() {
    # Close controller/filter before perception; each child handles graceful stop.
    [ -z "$PAIR_PID" ] || kill -TERM "$PAIR_PID" 2>/dev/null || true
    [ -z "$PAIR_PID" ] || wait "$PAIR_PID" 2>/dev/null || true
    [ -z "$VISION_PID" ] || kill -TERM "$VISION_PID" 2>/dev/null || true
    [ -z "$VISION_PID" ] || wait "$VISION_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
echo '[4/5] Starting navigation and Collision Monitor...'
bash "$TASK_DIR/start_navigation_ros1.sh" "$SCAN_FRAME" "$MODE" 8>&- &
PAIR_PID=$!
echo '[5/5] Starting YOLO and web camera...'
python3 -u "$TASK_DIR/jetauto_find_object.py" "$TARGET" --models-dir "$TASK_DIR/models" --port "$PORT" \
    "_odom_frame:=$JETAUTO_ODOM_FRAME" "_base_frame:=$JETAUTO_BASE_FRAME" 8>&- &
VISION_PID=$!
echo "Camera: http://192.168.149.1:$PORT"
echo "Mode: $MODE. Navigation starts PAUSED. Use Start/Pause on the camera page."
echo 'Keep this terminal open. Ctrl+C stops all test nodes; sensor drivers remain running.'
wait -n "$PAIR_PID" "$VISION_PID"
