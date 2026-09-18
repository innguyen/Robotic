#!/bin/bash
# ================================================================
# JetAuto AI Vision & Motor Control Server
# Tu dong cau hinh va khoi dong he thong theo chuan Hiwonder JetAuto
# ================================================================

# Dung dich vu nen start_app_node (tranh xung dot tranh chap camera va cong COM)
echo hiwonder | sudo -S systemctl stop start_app_node.service 2>/dev/null || true

# Source moi truong ROS chuan cua robot
if [ -f "/home/jetauto/jetauto_ws/src/jetauto_bringup/scripts/source_env.bash" ]; then
    source "/home/jetauto/jetauto_ws/src/jetauto_bringup/scripts/source_env.bash"
else
    for setup in /opt/ros/melodic/setup.bash /opt/ros/noetic/setup.bash; do
        [ -f "$setup" ] && source "$setup" && break
    done
    [ -f "/home/jetauto/jetauto_ws/devel/setup.bash" ] && source "/home/jetauto/jetauto_ws/devel/setup.bash"
fi

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
    killall -9 roscore rosmaster 2>/dev/null || true
    sleep 1
    echo "   Dang khoi dong roscore moi..."
    nohup roscore > /tmp/roscore.log 2>&1 &
    sleep 4
else
    echo "   roscore da hoat dong!"
fi

# ================================================================
# BUOC 1.5: Kiem tra va khoi dong Driver Dong Co Khung Gam (/dev/rrc)
# ================================================================
echo ">> [1.5/3] Kiem tra driver khung gam (dong co banh xe)..."
CHASSIS_RUNNING=false

if rostopic info /jetauto_controller/cmd_vel 2>/dev/null | grep -q "Subscribers:.*[a-zA-Z]" || \
   rostopic info /cmd_vel 2>/dev/null | grep -q "Subscribers:.*[a-zA-Z]"; then
    CHASSIS_RUNNING=true
    echo "   Driver dong co (jetauto_controller) da hoat dong san!"
else
    # Kiem tra xem phan cung bo dieu khien STM32 (/dev/rrc) da duoc cam va cap nguon chua
    if [ -e "/dev/rrc" ] || [ -e "/dev/ttyACM0" ]; then
        echo "   Phat hien bo mach STM32 (/dev/rrc)! Dang khoi dong jetauto_controller..."
        nohup roslaunch jetauto_controller jetauto_controller.launch > /tmp/chassis.log 2>&1 &
        sleep 4
        if rostopic info /jetauto_controller/cmd_vel 2>/dev/null | grep -q "Subscribers:.*[a-zA-Z]" || \
           rostopic info /cmd_vel 2>/dev/null | grep -q "Subscribers:.*[a-zA-Z]"; then
            CHASSIS_RUNNING=true
            echo "   Driver dong co da san sang hoat dong!"
        fi
    else
        echo "   -------------------------------------------------------"
        echo "   [CANH BAO PHAN CUNG DONG CO]"
        echo "   Chua phat hien bo mach dieu khien banh xe (/dev/rrc)."
        echo "   -> Hay kiem tra 2 dieu sau tren xe:"
        echo "      1. Cong tac nguon khung gam (pin) da BAT chua?"
        echo "      2. Day USB noi giua bo STM32 (duoi xe) va Jetson Nano da cam chat chua?"
        echo "   -------------------------------------------------------"
    fi
fi

# ================================================================
# BUOC 2: Kiem tra va khoi dong Camera Astra Pro Plus
# ================================================================
echo ">> [2/3] Kiem tra va khoi dong Camera Astra..."

# Kiem tra xem topic camera da co san chua
if rostopic list 2>/dev/null | grep -q "/depth_cam/rgb/image_raw"; then
    echo "   Camera Astra dang hoat dong san (den cam dang sang)!"
else
    echo "   Dang don dep va khoi dong camera Astra..."
    killall -9 orbbec_camera_node 2>/dev/null || true
    pkill -9 -f "depth_cam" 2>/dev/null || true
    sleep 2
    
    nohup roslaunch jetauto_peripherals depth_cam.launch > /tmp/depth_cam.log 2>&1 &
    
    echo "   Dang cho camera Astra khoi dong (cho toi da 15 giay)..."
    CAM_STARTED=false
    for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
        sleep 1
        if rostopic list 2>/dev/null | grep -q "/depth_cam/rgb/image_raw"; then
            CAM_STARTED=true
            echo "   Camera Astra da khoi dong thanh cong (den cam da sang)!"
            break
        fi
    done
    
    if [ "$CAM_STARTED" = false ]; then
        echo "   [CANH BAO] Astra chua publish topic! Kiem tra /tmp/depth_cam.log:"
        cat /tmp/depth_cam.log 2>/dev/null | tail -15
    fi
fi

# ================================================================
# BUOC 3: Setup Python 3 env va chay AI Server
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

# Them duong dan jetauto_driver va sdk vao PYTHONPATH
SDK_DIRS="/home/jetauto/jetauto_ws/src/jetauto_driver/jetauto_sdk/src:/home/jetauto/jetauto_ws/src/jetauto_driver/ros_robot_controller/src"

pkill -9 -f "jetauto_find_object.py" 2>/dev/null || true
sleep 1

cleanup() {
    echo ""
    echo "======================================================="
    echo ">> [JetAuto] DANG DUNG HE THONG VA DUNG XE AN TOAN..."
    echo "======================================================="
    pkill -9 -f "jetauto_find_object.py" 2>/dev/null || true
    echo ">> [JetAuto] Da thoat hoan toan!"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo ""
echo "======================================================="
echo "  AI SERVER DANG CHAY TRUC TIEP!"
echo "  (Nhan Ctrl+C de DUNG HOAN TOAN ca server va dong co)"
echo "======================================================="

PYTHONPATH="$SDK_DIRS:$SYSTEM_CV2_DIR:$PYTHONPATH" python3 -u /home/jetauto/jetauto_find_object.py
