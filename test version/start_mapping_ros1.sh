#!/bin/bash
# Called by the foreground test supervisor after sensor drivers are ready.
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROFILE="${1:?mapping or localization}"
MAP_FILE="${2:-}"
case "$PROFILE" in mapping|localization) ;; *) echo 'Invalid map profile'; exit 1 ;; esac
source /opt/ros/melodic/setup.bash
set -u
for package in tf map_server "$([ "$PROFILE" = mapping ] && echo gmapping || echo amcl)"; do
    rospack find "$package" >/dev/null || { echo "Missing ROS package: $package"; exit 1; }
done
if [ "$PROFILE" = localization ] && [ ! -f "$MAP_FILE" ]; then
    echo "Saved map YAML not found: $MAP_FILE"; exit 1
fi
# Refuse a second map/localization authority. Existing vendor SLAM must be stopped first.
python3 - <<'PY'
import rosgraph
publishers = rosgraph.Master('/test_mapping_preflight').getSystemState()[0]
if any(topic == '/map' and nodes for topic, nodes in publishers):
    raise SystemExit('An existing /map publisher is running; stop it before starting this map profile.')
PY
CROP_PID=''
LAUNCH_PID=''
cleanup() {
    [ -z "$LAUNCH_PID" ] || kill -INT "$LAUNCH_PID" 2>/dev/null || true
    [ -z "$CROP_PID" ] || kill -TERM "$CROP_PID" 2>/dev/null || true
    [ -z "$LAUNCH_PID" ] || wait "$LAUNCH_PID" 2>/dev/null || true
    [ -z "$CROP_PID" ] || wait "$CROP_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
python3 "$TASK_DIR/mapping_scan_ros1.py" &
CROP_PID=$!
roslaunch "$TASK_DIR/mapping_ros1.launch" "profile:=$PROFILE" "map_file:=$MAP_FILE" \
    "odom_frame:=${JETAUTO_ODOM_FRAME:-odom}" "base_frame:=${JETAUTO_BASE_FRAME:-base_footprint}" &
LAUNCH_PID=$!
wait -n "$CROP_PID" "$LAUNCH_PID"
