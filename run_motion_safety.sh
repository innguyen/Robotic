#!/bin/bash
# Entrypoint owned by jetauto-safety.service. Keep this process in the foreground
# so systemd can restart it and monitor its watchdog directly.
set -e

for setup in /opt/ros/melodic/setup.bash /opt/ros/noetic/setup.bash; do
    if [ -f "$setup" ]; then
        source "$setup"
        break
    fi
done
for ws in /home/jetauto/jetauto_ws/devel/setup.bash /home/jetauto/ros_ws/devel/setup.bash /home/jetauto/catkin_ws/devel/setup.bash; do
    [ -f "$ws" ] && source "$ws"
done
[ -f /home/jetauto/jetauto_ws/.typerc ] && source /home/jetauto/jetauto_ws/.typerc

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.149.1:11311}"
export ROS_IP="${ROS_IP:-192.168.149.1}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-$ROS_IP}"

exec python3 -u /home/jetauto/motion_safety_supervisor.py
