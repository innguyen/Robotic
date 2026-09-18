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

# Giai phong conflict tu app mac dinh cua Hiwonder va cap quyen thiet bi
sudo systemctl stop start_app_node.service 2>/dev/null || true
pkill -9 -f "app_node" 2>/dev/null || true
sudo chmod 666 /dev/ttyTHS* /dev/ttyUSB* /dev/video* 2>/dev/null || true

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
if rostopic list 2>/dev/null | grep -q "/scan" && pgrep -f "ydlidar|rplidar|lidar" > /dev/null 2>&1; then
    echo "   LiDAR G4 dang hoat dong tot (topic /scan)!"
else
    echo "   Dang khoi dong LiDAR G4..."
    pkill -9 -f "ydlidar" 2>/dev/null || true
    pkill -9 -f "rplidar" 2>/dev/null || true
    sleep 1
    nohup roslaunch jetauto_peripherals lidar.launch > /tmp/lidar.log 2>&1 &
    
    LIDAR_STARTED=false
    for i in 1 2 3 4 5; do
        sleep 1
        if rostopic list 2>/dev/null | grep -q "/scan"; then
            LIDAR_STARTED=true
            echo "   LiDAR G4 da khoi dong thanh cong (topic /scan)!"
            break
        fi
    done
    if [ "$LIDAR_STARTED" = false ]; then
        echo "   [CANH BAO] LiDAR chua tao topic /scan! Log /tmp/lidar.log:"
        cat /tmp/lidar.log 2>/dev/null | tail -10
    fi
fi

# ================================================================
# BUOC 2.5: Kiem tra va khoi dong Dong co / Chassis Controller
# ================================================================
echo ">> [2.5/5] Kiem tra va khoi dong Dong co (Chassis Controller)..."
if pgrep -f "chassis|jetauto_controller|driver_node" > /dev/null 2>&1; then
    echo "   Dong co / Chassis Controller dang hoat dong san!"
else
    echo "   Dang tim va khoi dong Controller Dong co..."
    CHASSIS_STARTED=false
    for CANDIDATE in \
        "jetauto_controller jetauto_controller.launch" \
        "jetauto_controller controller.launch" \
        "jetauto_bringup bringup.launch" \
        "jetauto_bringup jetauto_bringup.launch" \
        "jetauto_bringup robot_bringup.launch" \
        "jetauto_bringup robot.launch" \
        "jetauto_driver driver.launch" \
        "jetauto_driver chassis_driver.launch"; do
        PKG=$(echo $CANDIDATE | cut -d' ' -f1)
        LAUNCH=$(echo $CANDIDATE | cut -d' ' -f2)
        PKG_DIR=$(rospack find "$PKG" 2>/dev/null)
        if [ -n "$PKG_DIR" ]; then
            LAUNCH_PATH=$(find "$PKG_DIR" -name "$LAUNCH" 2>/dev/null | head -1)
            if [ -n "$LAUNCH_PATH" ]; then
                echo "   Phat hien launch file: $LAUNCH_PATH"
                echo "   Dang khoi dong $PKG $LAUNCH..."
                nohup roslaunch $PKG $LAUNCH > /tmp/chassis.log 2>&1 &
                sleep 3
                if pgrep -f "chassis|jetauto_controller|driver" > /dev/null 2>&1; then
                    CHASSIS_STARTED=true
                    echo "   >> Controller Dong co da bat thanh cong!"
                    break
                fi
            fi
        fi
    done
    if [ "$CHASSIS_STARTED" = false ]; then
        FOUND_LAUNCH=$(find /home/jetauto/jetauto_ws/src/ /home/jetauto/catkin_ws/src/ /home/jetauto/ -name "*bringup*.launch" -o -name "*chassis*.launch" -o -name "*controller*.launch" 2>/dev/null | grep -v "depth\|lidar\|arm\|view" | head -1)
        if [ -n "$FOUND_LAUNCH" ]; then
            echo "   Phat hien launch file chassis tu dong: $FOUND_LAUNCH"
            nohup roslaunch "$FOUND_LAUNCH" > /tmp/chassis.log 2>&1 &
            sleep 3
            CHASSIS_STARTED=true
        fi
    fi
    if [ "$CHASSIS_STARTED" = true ]; then
        echo "   Controller Dong co da duoc khoi dong! (Log: /tmp/chassis.log)"
    else
        echo "   [CANH BAO] Chua khoi dong duoc Chassis Controller! Log /tmp/chassis.log:"
        cat /tmp/chassis.log 2>/dev/null | tail -15
    fi
fi

# ================================================================
# BUOC 3: Kiem tra va khoi dong Camera Astra Pro Plus & OAK-D
# ================================================================
echo ">> [3/5] Kiem tra va khoi dong Camera Astra Pro Plus..."

# Khoi dong Camera Astra bang depth_cam.launch cua JetAuto
if rostopic list 2>/dev/null | grep -q "/depth_cam/rgb/image_raw\|/camera/rgb/image_raw"; then
    echo "   Camera Astra dang hoat dong san (topic RGB da co)!"
else
    echo "   Dang giai phong va khoi dong driver camera Astra..."
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
# ================================================================
echo ">> [4/5] Khoi chay AI YOLO Server..."

# Giai phong cong 5000-5005 de server luon nhan cong 5000
fuser -k 5000/tcp 5001/tcp 5002/tcp 2>/dev/null || true
pkill -9 -f "python3 -u /home/jetauto/jetauto_find_object.py" 2>/dev/null || true
pkill -9 -f "jetauto_find_object.py" 2>/dev/null || true
sleep 1

LOG_FILE="/tmp/jetauto_ai.log"
nohup env PYTHONPATH="$PY3_PYTHONPATH" \
    python3 -u /home/jetauto/jetauto_find_object.py \
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
