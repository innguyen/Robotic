#!/bin/bash
# Verify live sensor data and launch each driver once.
set -o pipefail
APP_SCRIPT="${1:-jetauto_find_object.py}"
MODE="${2:-run}"
case "$APP_SCRIPT" in jetauto_find_object.py|jetauto_smart_search.py) ;; *) exit 1 ;; esac
case "$MODE" in run|--check-only|--no-tail) ;; *) exit 1 ;; esac
exec 9>/tmp/jetauto_start_server.lock
flock -n 9 || { echo '[ERROR] Another startup is in progress.'; exit 1; }

for setup in /opt/ros/melodic/setup.bash /opt/ros/noetic/setup.bash; do
    [ -f "$setup" ] && source "$setup" && break
done
for ws in /home/jetauto/jetauto_ws/devel/setup.bash /home/jetauto/ros_ws/devel/setup.bash /home/jetauto/catkin_ws/devel/setup.bash; do
    [ -f "$ws" ] && source "$ws"
done
[ -f /home/jetauto/jetauto_ws/.typerc ] && source /home/jetauto/jetauto_ws/.typerc
export ROS_MASTER_URI=http://192.168.149.1:11311
export ROS_IP=192.168.149.1 ROS_HOSTNAME=192.168.149.1
fail() { echo "[ERROR] $*"; exit 1; }
node_alive() {
    # rosnode ping can exit zero even for an unknown node. Check master lookup.
    timeout 5 python -c "import rosgraph; rosgraph.Master('/jetauto_startup_check').lookupNode('$1')" >/dev/null 2>&1
}
message_ready() { timeout 6 rostopic echo -n 1 "$1/header" >/dev/null 2>&1; }
wait_message() {
    local topic="$1" log="$2" attempt
    for attempt in 1 2 3; do message_ready "$topic" && return 0; done
    tail -30 "$log" 2>/dev/null
    fail "No live messages on $topic; app startup cancelled."
}
start_launch() {
    local log="$1"; shift
    # Children must not inherit the startup lock.
    nohup roslaunch "$@" >"$log" 2>&1 < /dev/null 9>&- &
}
pause_app() {
    local port
    for port in 5000 5001 5002 5005; do
        curl -s --max-time 2 -X POST "http://127.0.0.1:$port/pause" >/dev/null 2>&1 || true
    done
}
pause_app
sudo -n systemctl stop start_app_node.service || fail 'Cannot stop vendor app service.'

echo '>> [1/4] ROS Master'
if ! timeout 5 rostopic list >/dev/null 2>&1; then
    # Never kill every roslaunch process when the master is slow.
    nohup roscore >/tmp/roscore.log 2>&1 < /dev/null 9>&- &
    MASTER_READY=false
    for attempt in 1 2 3 4 5; do
        sleep 1
        if timeout 3 rostopic list >/dev/null 2>&1; then MASTER_READY=true; break; fi
    done
    [ "$MASTER_READY" = true ] || fail 'ROS Master unavailable; see /tmp/roscore.log'
fi

echo '>> [2/4] LiDAR'
if ! message_ready /scan; then
    node_alive /ydlidar_lidar_g4_publisher && fail 'LiDAR node alive but /scan stalled.'
    start_launch /tmp/lidar.log jetauto_peripherals lidar.launch
    wait_message /scan /tmp/lidar.log
fi

echo '>> [3/4] Chassis and odometry'
if ! node_alive /jetauto_odom_publisher; then
    node_alive /ros_robot_controller && fail 'Partial chassis stack detected; inspect before restart.'
    # bringup.launch also starts LiDAR, vendor apps and cameras. Use only chassis.
    start_launch /tmp/bringup.log jetauto_controller jetauto_controller.launch
fi
wait_message /odom /tmp/bringup.log
node_alive /ros_robot_controller || fail 'Motor controller unavailable.'
timeout 5 rostopic info /jetauto_controller/cmd_vel | grep -q '/jetauto_odom_publisher' \
    || fail 'Velocity topic has no chassis subscriber.'
# rostopic pub -1 sleeps after publishing and can time out on a busy Jetson.
# Wait for the subscriber and send a short sequence of zero commands instead.
timeout 15 python - <<'PY' || fail 'Cannot send stop command.'
import time
import rospy
from geometry_msgs.msg import Twist
rospy.init_node('jetauto_startup_stop', anonymous=True)
pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)
deadline = time.time() + 5
while pub.get_num_connections() == 0 and time.time() < deadline:
    time.sleep(0.1)
if pub.get_num_connections() == 0:
    raise RuntimeError('No chassis subscriber connected')
for attempt in range(5):
    pub.publish(Twist())
    time.sleep(0.1)
PY

echo '>> [4/4] Astra camera'
if ! message_ready /depth_cam/rgb/image_raw; then
    node_alive /depth_cam && fail 'Camera node alive but RGB data stalled.'
    start_launch /tmp/depth_cam.log jetauto_peripherals depth_cam.launch
    wait_message /depth_cam/rgb/image_raw /tmp/depth_cam.log
fi
# Verify again after all launches, detecting any driver shutdown during startup.
wait_message /scan /tmp/lidar.log
wait_message /odom /tmp/bringup.log
if [ "$MODE" = --check-only ]; then
    echo 'Stationary checks passed. Existing app remains paused.'
    exit 0
fi

SYSTEM_CV2_DIR=""
for candidate in /home/jetauto/.virtualenvs/mediapipe/lib/python3.6/site-packages/cv2 /home/jetauto/.virtualenvs/mediapipe/lib/python3.6/site-packages /usr/local/lib/python3.6/dist-packages; do
    if [ -d "$candidate" ] && PYTHONPATH="$candidate" python3 -c "import cv2; exit(0 if hasattr(cv2, 'dnn') else 1)" 2>/dev/null; then
        SYSTEM_CV2_DIR="$candidate"; break
    fi
done
export PY3_PYTHONPATH="/home/jetauto/.local/lib/python3.6/site-packages:$SYSTEM_CV2_DIR:$PYTHONPATH"
[ -f "/home/jetauto/$APP_SCRIPT" ] || fail "Missing app: $APP_SCRIPT"
# Match only app Python processes, allow graceful motor stop, never kill sensors.
APP_PATTERN='^([^ ]*/)?python[23]?([.][0-9]+)?[ ]+(-u[ ]+)?([^ ]*/)?jetauto_(find_object|smart_search)[.]py([ ]|$)'
pkill -TERM -f "$APP_PATTERN" 2>/dev/null || true
for attempt in 1 2 3 4 5; do
    pgrep -f "$APP_PATTERN" >/dev/null || break
    sleep 1
done
pgrep -f "$APP_PATTERN" >/dev/null && fail 'Old AI app did not stop cleanly.'
LOG_FILE=/tmp/jetauto_ai.log
nohup env PYTHONPATH="$PY3_PYTHONPATH" python3 -u "/home/jetauto/$APP_SCRIPT" >"$LOG_FILE" 2>&1 < /dev/null 9>&- &
AI_PID=$!
sleep 3
kill -0 "$AI_PID" 2>/dev/null || { tail -30 "$LOG_FILE"; fail 'AI app exited.'; }
pause_app
echo "Drivers verified; AI app started (PID $AI_PID). See $LOG_FILE."
flock -u 9
exec 9>&-
[ "$MODE" = --no-tail ] || tail -f "$LOG_FILE"
