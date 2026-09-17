#!/bin/bash
# Script khoi dong tu dong cho JetAuto

# ================================================================
# BUOC 0: Source ROS voi moi truong sach (Python 2 - cho roslaunch)
# QUAN TRONG: Phai source va start camera TRUOC KHI them Python 3 path
# ================================================================
for setup in /opt/ros/melodic/setup.bash /opt/ros/noetic/setup.bash; do
    [ -f "$setup" ] && source "$setup" && break
done
for ws in /home/jetauto/jetauto_ws/devel/setup.bash \
           /home/jetauto/ros_ws/devel/setup.bash \
           /home/jetauto/catkin_ws/devel/setup.bash; do
    [ -f "$ws" ] && source "$ws"
done
export ROS_MASTER_URI=http://192.168.149.1:11311
export ROS_HOSTNAME=192.168.149.1
export ROS_IP=192.168.149.1

echo "========================================"
echo "   HE THONG JETAUTO AI VISION SERVER    "
echo "========================================"

# ================================================================
# BUOC 1: Kiem tra ROS Master (roscore)
# ================================================================
echo ">> [1/3] Kiem tra ROS Master (roscore)..."
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
# BUOC 2: Kiem tra va khoi dong Camera Astra Pro Plus
# ================================================================
echo ">> [2/3] Kiem tra va khoi dong Camera Astra..."

# Neu camera dang hoat dong san va publish topic roi thi giu nguyen
if rostopic list 2>/dev/null | grep -q "/depth_cam/rgb/image_raw"; then
    echo "   Camera Astra dang hoat dong san (den cam dang sang)!"
else
    echo "   Dang giai phong va khoi dong camera Astra..."
    pkill -9 -f "depth_cam" 2>/dev/null || true
    pkill -9 -f "orbbec" 2>/dev/null || true
    pkill -9 -f "usb_cam" 2>/dev/null || true
    sleep 2
    
    nohup roslaunch jetauto_peripherals depth_cam.launch > /tmp/depth_cam.log 2>&1 &
    
    # Cho camera sang den va publish topic (toi da 8 giay)
    CAM_STARTED=false
    for i in 1 2 3 4 5 6 7 8; do
        sleep 1
        if rostopic list 2>/dev/null | grep -q "/depth_cam/rgb/image_raw"; then
            CAM_STARTED=true
            echo "   Camera Astra da khoi dong thanh cong (den cam da sang)!"
            break
        fi
    done
    
    if [ "$CAM_STARTED" = false ]; then
        echo "   [CANH BAO] Astra chua khoi dong duoc! Log /tmp/depth_cam.log:"
        cat /tmp/depth_cam.log 2>/dev/null | tail -15
    fi
fi

# ================================================================
# BUOC 3: Setup Python 3 env va chay AI Server
# Chi them Python 3 path SAU KHI da khoi dong xong camera
# ================================================================
echo ">> [3/3] Khoi chay AI YOLO Server..."

# Tim cv2 Python 3 co dnn (mediapipe virtualenv)
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

pkill -f "python3 /home/jetauto/jetauto_find_object.py" 2>/dev/null || true
sleep 1

LOG_FILE="/tmp/jetauto_ai.log"
nohup env PYTHONPATH="$SYSTEM_CV2_DIR:$PYTHONPATH" \
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

