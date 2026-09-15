#!/bin/bash
# Script khoi dong PuppyPi AI YOLO Server ben trong Docker

# 1. Tim Docker container chua ROS Noetic cua PuppyPi
CID=""
for id in $(sudo docker ps -q); do
    if sudo docker exec $id test -d /opt/ros/noetic 2>/dev/null; then
        CID=$id
        break
    fi
done

if [ -z "$CID" ]; then
    CID=$(sudo docker ps -q | head -n 1)
fi

echo ">> Dang ket noi vao Docker Container: $CID"

# 2. Copy code va giao dien vao container
sudo docker cp /home/pi/app_yolo_server_v2.py $CID:/home/ubuntu/app_yolo_server_v2.py
sudo docker cp /home/pi/templates $CID:/home/ubuntu/
if [ -f /home/pi/yolo11n.onnx ]; then
    sudo docker cp /home/pi/yolo11n.onnx $CID:/home/ubuntu/yolo11n.onnx
fi

# 3. Nạp workspace PuppyPi va chay chuong trinh
sudo docker exec -it -w /home/ubuntu $CID bash -c '
source /opt/ros/noetic/setup.bash 2>/dev/null
for s in $(find /home -name "setup.bash" 2>/dev/null); do
    source "$s" 2>/dev/null
done
pkill -f "python3 app_yolo_server" 2>/dev/null || true
python3 app_yolo_server_v2.py
'
