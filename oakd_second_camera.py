#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS node for Luxonis OAK-D camera (RGB Stream).

Publishes:
  - /oakd/rgb/image_raw  (sensor_msgs/Image, bgr8)
"""
from __future__ import print_function

import sys
import time
import numpy as np

import rospy
from sensor_msgs.msg import Image

try:
    import depthai as dai
except ImportError:
    dai = None

def frame_to_ros_img(frame, frame_id='oakd_rgb', encoding='bgr8'):
    """Chuyen numpy array sang sensor_msgs/Image truc tiep, khong can cv_bridge."""
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg


class OAKDSecondCameraNode(object):
    def __init__(self):
        if dai is None:
            rospy.logerr('[OAK-D] Chua cai dat depthai! Vui long chay: python3 -m pip install depthai')
            raise RuntimeError('depthai not installed')

        self.device = None
        self.rgb_queue = None

        # Khoi tao thiet bi truoc
        self._init_device()

        # Dang ky Topic sau khi thiet bi da san sang 100%
        self.rgb_pub = rospy.Publisher('/oakd/rgb/image_raw', Image, queue_size=1)
        rospy.loginfo('>> [OAK-D] Node khoi tao thanh cong va san sang stream!')

    def _create_rgb_pipeline(self):
        pipeline = dai.Pipeline()

        # Cam RGB 1080P preview 640x480, 30 FPS
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)

        xout_rgb = pipeline.createXLinkOut()
        xout_rgb.setStreamName('rgb')
        cam_rgb.preview.link(xout_rgb.input)

        return pipeline

    def _init_device(self):
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            print('[OAK-D] ERROR: Khong tim thay thiet bi OAK-D nao tren USB!', flush=True)
            rospy.logerr('[OAK-D] KHONG TIM THAY THIET BI OAK-D NAO!')
            raise RuntimeError('No OAK-D found')

        print('[OAK-D] Tim thay {} thiet bi OAK-D. Dang khoi tao...'.format(len(devices)), flush=True)
        rospy.loginfo('[OAK-D] Tim thay %d thiet bi OAK-D', len(devices))

        pipeline = self._create_rgb_pipeline()

        # 1. Thu ket noi che do Auto USB
        connected = False
        try:
            print('[OAK-D] Dang nap firmware vao OAK-D (che do Auto Speed)...', flush=True)
            self.device = dai.Device(pipeline)
            connected = True
            print('[OAK-D] KET NOI THANH CONG che do Auto Speed!', flush=True)
        except Exception as err:
            print('[OAK-D] Che do Auto Speed gap su co: {}.'.format(err), flush=True)
            print('[OAK-D] Thu lai voi che do tuong thich USB 2.0 (HIGH Speed)...', flush=True)
            time.sleep(2.5) # Doi USB bus reset
            try:
                self.device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)
                connected = True
                print('[OAK-D] KET NOI THANH CONG che do USB 2.0 High Speed!', flush=True)
            except Exception as err2:
                print('[OAK-D] LOI: Ket noi che do USB 2.0 cung that bai: {}'.format(err2), flush=True)
                raise err2

        usb_speed = self.device.getUsbSpeed()
        print('[OAK-D] Camera dang hoat dong o toc do USB: {}'.format(usb_speed.name), flush=True)
        rospy.loginfo('[OAK-D] Toc do USB: %s', usb_speed.name)

        self.rgb_queue = self.device.getOutputQueue(name='rgb', maxSize=4, blocking=False)

    def publish_loop(self):
        rate = rospy.Rate(30)
        first_frame_logged = False

        while not rospy.is_shutdown():
            if self.rgb_queue is not None:
                try:
                    in_rgb = self.rgb_queue.tryGet()
                    if in_rgb is not None:
                        frame = in_rgb.getCvFrame()
                        if frame is not None:
                            msg = frame_to_ros_img(frame, frame_id='oakd_rgb', encoding='bgr8')
                            self.rgb_pub.publish(msg)
                            if not first_frame_logged:
                                print('>> [OAK-D] Frame dau tien da duoc publish thanh cong len /oakd/rgb/image_raw!', flush=True)
                                rospy.loginfo('>> [OAK-D] Frame dau tien da duoc publish thanh cong len /oakd/rgb/image_raw!')
                                first_frame_logged = True
                except Exception as exc:
                    rospy.logwarn_throttle(5.0, 'RGB publish failed: %s', str(exc))

            rate.sleep()


def main():
    rospy.init_node('oakd_second_camera', anonymous=True)
    rospy.loginfo('Bat dau OAK-D ROS Node...')
    try:
        node = OAKDSecondCameraNode()
        node.publish_loop()
    except Exception as e:
        print('[OAK-D] LOI KHOI DONG: {}'.format(e), file=sys.stderr, flush=True)
        rospy.logerr('[OAK-D] Loi khoi dong: %s', str(e))
        import traceback
        traceback.print_exc()
        sys.stderr.flush()
        sys.exit(1)
    finally:
        rospy.loginfo('[OAK-D] Da tat OAK-D ROS Node an toan.')


if __name__ == '__main__':
    main()
