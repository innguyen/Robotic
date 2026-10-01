#!/bin/bash
# Script khoi dong he thong JetAuto (LiDAR + Dual Camera + AI YOLO Server)

# ================================================================
# BUOC 0: Cau hinh moi truong ROS va Python
# ================================================================
for setup in /opt/ros/melodic/setup.bash /opt/ros/noetic/setup.bash; do
    [ -f "$setup" ] && source "$setup" && break
done
for ws in /home/jetauto/jetauto_ws/devel/setup.bash \
           /home/jetauto/ros_ws/devel/setup.bash \
           /home/jetauto/catkin_ws/devel/setup.bash; do
    [ -f "$ws" ] && source "$ws"
done

[ -f /home/jetauto/.bashrc ] && source /home/jetauto/.bashrc 2>/dev/null || true
[ -f /home/jetauto/jetauto_ws/.jetautorc ] && source /home/jetauto/jetauto_ws/.jetautorc 2>/dev/null || true
[ -f /home/jetauto/jetauto_ws/.typerc ] && source /home/jetauto/jetauto_ws/.typerc 2>/dev/null || true
export DEPTH_CAMERA_TYPE=${DEPTH_CAMERA_TYPE:-AstraProPlus}
export LIDAR_TYPE=${LIDAR_TYPE:-G4}
export ROS_MASTER_URI=http://192.168.149.1:11311
export ROS_HOSTNAME=192.168.149.1
export ROS_IP=192.168.149.1

# Tim thu vien OpenCV va DepthAI cho Python 3
SYSTEM_CV2_DIR=""
for CV2_CANDIDATE in \
    /home/jetauto/.virtualenvs/mediapipe/lib/python3.6/site-packages/cv2 \
    /home/jetauto/.virtualenvs/mediapipe/lib/python3.6/site-packages \
    /usr/local/lib/python3.6/dist-packages; do
    if [ -d "$CV2_CANDIDATE" ]; then
        if PYTHONPATH="$CV2_CANDIDATE" python3 -c "import cv2; exit(0 if hasattr(cv2, 'dnn') else 1)" 2>/dev/null; then
            SYSTEM_CV2_DIR="$CV2_CANDIDATE"
            break
        fi
    fi
done
export PY3_PYTHONPATH="/home/jetauto/.local/lib/python3.6/site-packages:$SYSTEM_CV2_DIR:$PYTHONPATH"

echo "========================================"
echo "   HE THONG JETAUTO AI VISION SERVER    "
echo "========================================"

# A ROS master can retain a topic name after its camera node has died.  Test
# for an actual publisher, rather than only testing `rostopic list`, before
# deciding that a device is already running.
topic_has_publisher() {
    rostopic info "$1" 2>/dev/null | awk '
        /^Publishers:/{publishers=1; next}
        /^Subscribers:/{publishers=0}
        publishers && /^[[:space:]]*\*/ {found=1}
        END {exit !found}'
}

scan_has_data() {
    topic_has_publisher /scan && timeout 4 rostopic echo -n 1 /scan >/dev/null 2>&1
}

# Giai phong conflict tu app mac dinh cua Hiwonder va cap quyen thiet bi
sudo systemctl stop start_app_node.service 2>/dev/null || true
pkill -9 -f "app_node" 2>/dev/null || true
sudo chmod 666 /dev/ttyTHS* /dev/ttyUSB* /dev/video* /dev/rrc /dev/ttyACM* 2>/dev/null || true

# Cap quyen USB cho OAK-D Myriad X (03e7)
if [ ! -f /etc/udev/rules.d/80-movidius.rules ]; then
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null 2>&1 || true
    sudo udevadm control --reload-rules 2>/dev/null || true
    sudo udevadm trigger 2>/dev/null || true
fi

# ================================================================
# BUOC 1: Kiem tra ROS Master (roscore)
# ================================================================
echo ">> [1/4] Kiem tra ROS Master (roscore)..."
if ! rostopic list > /dev/null 2>&1; then
    echo "   Don dep roscore/rosmaster cu bi treo..."
    killall -9 roscore rosmaster roslaunch 2>/dev/null || true
    sleep 2
    echo "   Dang khoi dong roscore moi..."
    nohup roscore > /tmp/roscore.log 2>&1 &
    sleep 5
else
    echo "   roscore da hoat dong!"
fi

# ================================================================
# BUOC 2: Kiem tra va khoi dong LiDAR G4
# ================================================================
echo ">> [2/4] Kiem tra va khoi dong LiDAR G4..."
if scan_has_data; then
    echo "   LiDAR G4 dang phat ban quet /scan!"
else
    echo "   Dang khoi dong LiDAR G4..."
    pkill -9 -f "ydlidar" 2>/dev/null || true
    pkill -9 -f "rplidar" 2>/dev/null || true
    sleep 1
    nohup roslaunch jetauto_peripherals lidar.launch > /tmp/lidar.log 2>&1 &
    
    LIDAR_STARTED=false
    for i in 1 2 3 4 5; do
        sleep 1
        if scan_has_data; then
            LIDAR_STARTED=true
            echo "   LiDAR G4 da khoi dong thanh cong (co du lieu /scan)!"
            break
        fi
    done
    if [ "$LIDAR_STARTED" = false ]; then
        echo "   [CANH BAO] LiDAR khong phat du lieu /scan; robot se khong tu chay! Log /tmp/lidar.log:"
        cat /tmp/lidar.log 2>/dev/null | tail -10
    fi
fi

# ================================================================
# ================================================================
# BUOC 2.5: Kiem tra va khoi dong Bo dieu khien Dong co Khung gam (jetauto_controller)
# ================================================================
echo ">> [2.5/5] Kiem tra va khoi dong Bo dieu khien Dong co Khung gam..."
if rostopic list 2>/dev/null | grep -q "/jetauto_controller/cmd_vel\|/cmd_vel" && pgrep -f "odom_publisher" > /dev/null 2>&1; then
    echo "   Bo dieu khien khung gam (jetauto_controller) dang hoat dong san!"
else
    echo "   Dang khoi dong jetauto_controller jetauto_controller.launch..."
    pkill -9 -f "jetauto_controller.launch" 2>/dev/null || true
    pkill -9 -f "odom_publisher" 2>/dev/null || true
    pkill -9 -f "ros_robot_controller" 2>/dev/null || true
    sleep 1
    nohup roslaunch jetauto_controller jetauto_controller.launch > /tmp/controller.log 2>&1 &
    
    CHASSIS_STARTED=false
    for i in 1 2 3 4 5 6 7 8 9 10; do
        sleep 1
        if rostopic list 2>/dev/null | grep -q "/jetauto_controller/cmd_vel\|/cmd_vel"; then
            CHASSIS_STARTED=true
            echo "   >> Bo dieu khien khung gam da khoi dong thanh cong! (Log: /tmp/controller.log)"
            break
        fi
    done
    if [ "$CHASSIS_STARTED" = false ]; then
        echo "   [CANH BAO] Chassis Controller chua len hoan toan! Kiem tra log /tmp/controller.log:"
        cat /tmp/controller.log 2>/dev/null | tail -15
    fi
fi

# BUOC 3: Kiem tra va khoi dong Camera Astra Pro Plus & OAK-D
# ================================================================
echo ">> [3/5] Kiem tra va khoi dong Camera Astra Pro Plus..."

# Khoi dong Camera Astra bang depth_cam.launch cua JetAuto
if topic_has_publisher /depth_cam/rgb/image_raw || topic_has_publisher /camera/rgb/image_raw; then
    echo "   Camera Astra dang hoat dong san (topic RGB da co)!"
else
    echo "   Dang giai phong va khoi dong driver camera Astra..."
    export DEPTH_CAMERA_TYPE=AstraProPlus
    pkill -9 -f "depth_cam" 2>/dev/null || true
    pkill -9 -f "orbbec" 2>/dev/null || true
    pkill -9 -f "usb_cam" 2>/dev/null || true
    pkill -9 -f "astra_camera" 2>/dev/null || true
    sleep 2
    
    nohup roslaunch jetauto_peripherals depth_cam.launch > /tmp/depth_cam.log 2>&1 &
    
    ASTRA_STARTED=false
    for i in 1 2 3 4 5 6 7 8 9 10; do
        sleep 1
        if rostopic list 2>/dev/null | grep -q "/depth_cam/rgb/image_raw\|/camera/rgb/image_raw"; then
            ASTRA_STARTED=true
            echo "   >> Camera Astra da khoi dong thanh cong!"
            break
        fi
    done
    if [ "$ASTRA_STARTED" = false ]; then
        echo "   [CANH BAO] Astra chua len topic RGB! Log /tmp/depth_cam.log:"
        cat /tmp/depth_cam.log 2>/dev/null | tail -15
    fi
fi

# OAK-D se duoc ket noi truc tiep bang DepthAI ben trong jetauto_find_object.py
if lsusb 2>/dev/null | grep -q "03e7"; then
    echo "   [OAK-D] Phat hien phan cung Luxonis OAK-D qua USB (DepthAI ColorCamera)!"
fi

# ================================================================
# BUOC 4: Setup Python 3 va chay AI YOLO Server
APP_SCRIPT="${1:-jetauto_find_object.py}"
if [ "$APP_SCRIPT" = "jetauto_smart_search.py" ]; then
    if ! command -v espeak-ng >/dev/null 2>&1 && ! command -v espeak >/dev/null 2>&1; then
        echo "[LOA] Thieu giong noi offline. Cai mot lan: sudo apt-get install espeak"
    fi
fi
echo ">> [4/5] Khoi chay AI Server ($APP_SCRIPT)..."

# Tell an existing server to stop the chassis before replacing it.  SIGTERM
# lets rospy run the shutdown callback that publishes a zero Twist.
if command -v curl >/dev/null 2>&1; then
    for port in 5005; do
        curl --silent --max-time 1 -X POST "http://127.0.0.1:$port/pause" >/dev/null 2>&1 || true
    done
fi
pkill -TERM -f "python3.*jetauto_find_object" 2>/dev/null || true
pkill -TERM -f "python3.*jetauto_smart_search" 2>/dev/null || true
sleep 2

LOG_FILE="/tmp/jetauto_ai.log"
nohup env PYTHONPATH="$PY3_PYTHONPATH" \
    python3 -u "/home/jetauto/$APP_SCRIPT" \
    > "$LOG_FILE" 2>&1 &

AI_PID=$!
echo "   AI Server da duoc bat (PID: $AI_PID)"
sleep 2

# Kiem tra xem process con song khong
if kill -0 $AI_PID 2>/dev/null; then
    echo ""
    echo "======================================================="
    echo "  HE THONG DANG CHAY THANH CONG!"
    echo "  (Nhan Ctrl+C de thoat xem log - Server van chay ngam)"
    echo "======================================================="
    tail -f "$LOG_FILE"
else
    echo "[LOI] AI Server khoi dong that bai! Noi dung log:"
    cat "$LOG_FILE" 2>/dev/null | tail -30
fi
