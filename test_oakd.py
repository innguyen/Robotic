#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
Script doc lap (STANDALONE) de kiem tra camera Luxonis OAK-D (RGB + Depth).
KHONG CAN ROS - Chay duoc truc tiep tren ca Windows va Robot Linux.

Cach chay tren Windows:
    .\.venv\Scripts\python.exe test_oakd.py
    hoac: python test_oakd.py (sau khi da pip install depthai opencv-python)

Cach chay tren Robot JetAuto (Linux):
    python3 test_oakd.py
"""

import sys
import time
import numpy as np

print("=" * 60)
print("  KIEM TRA DRIVER VA KET NOI CAMERA LUXONIS OAK-D")
print("=" * 60)

# 1. Kiem tra thu vien depthai
try:
    import depthai as dai
    print(f"[OK] Da import thanh cong depthai (phien ban: {dai.__version__})")
except ImportError:
    print("[LOI] Chua cai dat thu vien 'depthai'!")
    print("\nHUONG DAN CAI DAT:")
    if sys.platform.startswith('win'):
        print("  Tren Windows, go lenh:")
        print("    pip install depthai opencv-python")
        print("  Hoac su dung python trong .venv da co san:")
        print(r"    .\.venv\Scripts\python.exe test_oakd.py")
    else:
        print("  Tren Robot / Ubuntu / Linux, go lenh:")
        print("    python3 -m pip install depthai opencv-python")
        print("  Neu gap loi thieu udev rules tren Linux:")
        print("    echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"' | sudo tee /etc/udev/rules.d/80-movidius.rules")
        print("    sudo udevadm control --reload-rules && sudo udevadm trigger")
    sys.exit(1)

# 2. Kiem tra opencv
try:
    import cv2
    print(f"[OK] Da import thanh cong OpenCV (phien ban: {cv2.__version__})")
except ImportError:
    print("[LOI] Chua cai dat 'opencv-python'! Vui long chay: pip install opencv-python")
    sys.exit(1)

# 3. Quet thiet bi OAK-D
print("\n[>>] Dang quet thiet bi OAK-D qua USB...")
devices = dai.Device.getAllAvailableDevices()

if not devices:
    print("\n[!] KHONG TIM THAY CAMERA OAK-D NAO DUOC CAM VAO MAY!")
    print("Vui long kiem tra cac van de sau:")
    print(" 1. Ban da cam day Type-C cua OAK-D vao cong USB chua?")
    print(" 2. Chu y: Nen cam vao cong USB 3.0 (cong mau XANH DUONG). OAK-D can nguon va bang thong cao.")
    print(" 3. Thu doi day cap Type-C khac (nhieu day chi la day sac, khong co duong data).")
    if not sys.platform.startswith('win'):
        print(" 4. Tren Linux/Robot: Can udev rules de cap quyen truy cap USB:")
        print("    echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"' | sudo tee /etc/udev/rules.d/80-movidius.rules")
        print("    sudo udevadm control --reload-rules && sudo udevadm trigger")
    sys.exit(0)

print(f"[OK] Phat hien {len(devices)} thiet bi OAK-D:")
for i, dev in enumerate(devices):
    print(f"  [{i+1}] MxId: {dev.getMxId()} | State: {dev.state} | Protocol: {dev.protocol.name}")

# 4. Khoi tao Pipeline OAK-D de stream hinh anh
print("\n[>>] Dang khoi tao Pipeline Camera (RGB + Stereo Depth)...")
pipeline = dai.Pipeline()

# Cam RGB
cam_rgb = pipeline.createColorCamera()
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
xout_rgb = pipeline.createXLinkOut()
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

# Cam Stereo Depth (neu la OAK-D co 2 mat kinh stereo)
has_stereo = False
try:
    mono_left = pipeline.createMonoCamera()
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    mono_right = pipeline.createMonoCamera()
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo = pipeline.createStereoDepth()
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    xout_depth = pipeline.createXLinkOut()
    xout_depth.setStreamName("depth")
    stereo.disparity.link(xout_depth.input)
    has_stereo = True
except Exception as e:
    print(f" [!] Bo qua Stereo Depth (loai OAK-1 hoac loi setup stereo): {e}")

try:
    print("[>>] Dang ket noi va bat camera OAK-D...")
    with dai.Device(pipeline) as device:
        usb_speed = device.getUsbSpeed()
        print(f"[OK] Camera OAK-D da duoc ket noi thanh cong! Toc do USB: {usb_speed.name}")
        if "HIGH" in usb_speed.name:
            print("  [CHU Y] Camera dang chay o toc do USB 2.0 (High Speed). Khuyen khich cam cong USB 3.0 de dat hieu nang cao nhat.")
        elif "SUPER" in usb_speed.name:
            print("  [TUYET VOI] Camera dang chay o toc do USB 3.0 (SuperSpeed)!")

        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False) if has_stereo else None

        print("\n=======================================================")
        print("  DANG STREAM HINH ANH TU OAK-D!")
        print("  Nhan phim 'q' tren cua so video de THOAT.")
        print("=======================================================")

        prev_time = time.time()
        fps = 0.0

        while True:
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                cur_time = time.time()
                fps = 0.9 * fps + 0.1 * (1.0 / max(cur_time - prev_time, 0.001))
                prev_time = cur_time

                cv2.putText(frame, f"OAK-D RGB | FPS: {fps:.1f}", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(frame, f"USB: {usb_speed.name}", (15, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, "Nhan 'q' de thoat", (15, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                cv2.imshow("OAK-D - RGB Stream", frame)

            if q_depth is not None:
                in_depth = q_depth.tryGet()
                if in_depth is not None:
                    depth_frame = in_depth.getFrame()
                    # Chuyen disparity sang anh mau color map de nhin chieu sau
                    disp_norm = (depth_frame * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
                    disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
                    cv2.imshow("OAK-D - Depth Stream", disp_color)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

        cv2.destroyAllWindows()
        print("\nDa tat camera OAK-D an toan.")

except Exception as e:
    print(f"\n[LOI KHOI DONG OAK-D]: {e}")
