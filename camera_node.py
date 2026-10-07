#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
NODE ROS CAMERA OAK-D S2 CHUẨN XÁC & ĐỘC LẬP (CAMERA_NODE.PY)
Được phát triển riêng cho đồ án JetBot SLAM (CS532.R11)
Thay thế hoàn toàn depthai_examples C++ bên ngoài, giải quyết triệt để lỗi:
  - "Couldn't open stream"
  - "No such file or directory: *.launch"
  - Lỗi xung đột thư viện cv_bridge trên Python 3 Jetson Nano.

Các Topics được xuất bản chuẩn xác (Publishers):
  1. /stereo_inertial_publisher/color/image (sensor_msgs/Image, BGR8)
  2. /stereo_inertial_publisher/stereo/depth (sensor_msgs/Image, 16UC1 mm)
  3. /yolov4_publisher/color/image (Bí danh tương thích)
  4. /yolov4_publisher/stereo/depth (Bí danh tương thích)
  5. /obstacle_distance (std_msgs/Float32, mét)
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import numpy as np
import cv2

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32, String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

try:
    import depthai as dai
    HAS_DEPTHAI = True
except ImportError:
    HAS_DEPTHAI = False


def create_oak_pipeline():
    """Tạo Pipeline DepthAI chuẩn tối ưu cho OAK-D S2 trên Jetson Nano"""
    if not HAS_DEPTHAI:
        return None, False

    pipeline = dai.Pipeline()
    is_v3 = not hasattr(dai.node, 'XLinkOut')

    if is_v3:
        # DepthAI v3 API
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)

        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)

        mono_r = pipeline.create(dai.node.MonoCamera)
        mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(stereo.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)

        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)
        return pipeline, is_v3
    else:
        # DepthAI v2 API (Chuẩn Jetson Nano / Ubuntu 18.04 Melodic)
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)

        mono_r = pipeline.create(dai.node.MonoCamera)
        mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)

        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline, is_v3


def build_image_msg(img_np, encoding, frame_id="oak-d_frame"):
    """Chuyển đổi ma trận Numpy thành sensor_msgs/Image thuần túy không cần cv_bridge"""
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.height = int(img_np.shape[0])
    msg.width = int(img_np.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    if len(img_np.shape) == 3:
        msg.step = int(msg.width * img_np.shape[2] * img_np.itemsize)
    else:
        msg.step = int(msg.width * img_np.itemsize)
    msg.data = img_np.tobytes()
    return msg


def main():
    print("\n" + "═" * 74)
    print("📷 KHỞI ĐỘNG NODE CAMERA OAK-D S2 THUẦN TÚY (CAMERA_NODE.PY)")
    print("📦 PHIÊN BẢN (VERSION) : v2.4.0-SAFETY-DUAL-ROS")
    print("🎯 MỤC TIÊU            : Cung cấp luồng ảnh RGB & Stereo Depth ổn định cho ROS")
    print("═" * 74)

    if not HAS_ROS:
        print("❌ [LỖI] Không tìm thấy thư viện rospy. Hãy chạy qua 'roslaunch' hoặc 'roscore'.")
        sys.exit(1)

    if not HAS_DEPTHAI:
        print("❌ [LỖI] Không tìm thấy thư viện depthai. Hãy cài đặt: pip3 install depthai")
        sys.exit(1)

    rospy.init_node('oak_camera_publisher', anonymous=False)

    # Publishers chuẩn cho đồ án
    pub_rgb_stereo = rospy.Publisher('/stereo_inertial_publisher/color/image', Image, queue_size=1)
    pub_depth_stereo = rospy.Publisher('/stereo_inertial_publisher/stereo/depth', Image, queue_size=1)
    pub_rgb_yolo = rospy.Publisher('/yolov4_publisher/color/image', Image, queue_size=1)
    pub_depth_yolo = rospy.Publisher('/yolov4_publisher/stereo/depth', Image, queue_size=1)
    pub_obs_dist = rospy.Publisher('/obstacle_distance', Float32, queue_size=1)

    print("🚀 [OAK-D] Đang kết nối thiết bị OAK-D S2 qua cổng USB...")
    pipeline, is_v3 = create_oak_pipeline()

    try:
        device = dai.Device(pipeline)
        q_rgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
        q_depth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
        print("✅ [OAK-D] Đã kết nối thành công Camera OAK-D S2! Bắt đầu phát luồng Topics 30 FPS...")
        print("📡 Topics đang phát:")
        print("   ├─ /stereo_inertial_publisher/color/image")
        print("   ├─ /stereo_inertial_publisher/stereo/depth")
        print("   └─ /obstacle_distance")
    except Exception as e:
        print(f"❌ [OAK-D] Khởi tạo thiết bị thất bại: {e}")
        print("💡 [HƯỚNG DẪN] Kiểm tra cáp USB và chạy: sudo udevadm control --reload-rules && sudo udevadm trigger")
        sys.exit(1)

    rate = rospy.Rate(35) # Đảm bảo đạt 30 FPS ổn định
    last_clearance_m = 99.0

    while not rospy.is_shutdown():
        in_rgb = q_rgb.tryGet()
        in_depth = q_depth.tryGet()

        if in_rgb is not None:
            rgb_frame = in_rgb.getCvFrame() if hasattr(in_rgb, 'getCvFrame') else in_rgb.getFrame()
            msg_rgb = build_image_msg(rgb_frame, "bgr8")
            pub_rgb_stereo.publish(msg_rgb)
            pub_rgb_yolo.publish(msg_rgb)

        if in_depth is not None:
            depth_frame = in_depth.getFrame() if hasattr(in_depth, 'getFrame') else in_depth.getCvFrame()
            msg_depth = build_image_msg(depth_frame, "16UC1")
            pub_depth_stereo.publish(msg_depth)
            pub_depth_yolo.publish(msg_depth)

            # Tính cự ly an toàn tức thì từ Depth
            h, w = depth_frame.shape[:2]
            roi = depth_frame[int(h * 0.15):int(h * 0.60), int(w * 0.25):int(w * 0.75)]
            valid = roi[(roi >= 50) & (roi <= 3500)]
            if len(valid) >= 20:
                last_clearance_m = round(float(np.percentile(valid, 5)) / 1000.0, 2)
            elif last_clearance_m <= 0.55:
                last_clearance_m = 0.20 # Giữ chốt điểm mù
            else:
                last_clearance_m = 99.0

            pub_obs_dist.publish(Float32(data=last_clearance_m))

        rate.sleep()

    device.close()
    print("🛑 [OAK-D] Đã đóng camera an toàn.")


if __name__ == '__main__':
    main()
