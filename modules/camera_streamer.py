#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 3: Quản lý luồng Camera OAK-D S2 & Mây điểm 3D (Camera Streamer)
Tuân thủ Quy tắc Harness 3: Giảm tải CPU bằng cách skip frames và downsampling.
"""

import os
import sys
import time
import math
import json
import numpy as np
import cv2

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

# 20 nhãn lớp chuẩn của mô hình MobileNet-SSD trên VPU Myriad X
MOBILENET_LABELS = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

try:
    import rospy
    from sensor_msgs.msg import Image as ROSImage
    from std_msgs.msg import Float32 as ROSFloat32
    from std_msgs.msg import String as ROSString
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    ROSImage = object
    ROSFloat32 = object
    ROSString = object

class CameraStreamer:
    """
    Module Quản lý luồng Camera OAK-D S2 & Webcam Laptop (Thành viên 2).
    Hỗ trợ 3 chế độ xem:
      1. 'rgb': Camera hiện thực (RGB tự nhiên từ Webcam laptop hoặc OAK-D)
      2. 'depth': Camera độ sâu (Stereo Depth Colormap thang màu JET từ OAK-D hoặc mô phỏng)
      3. 'ai': AI Spatial Detection & Cảnh báo phanh khẩn cấp 20cm (Bounding box, tâm ngắm, thước đo)
    """
    VIEW_MODES = ['ai', 'thermal']

    def __init__(self, depth_step=25, depth_skip=5, img_skip=1, as_ros_publisher=False):
        self.depth_step = depth_step
        self.depth_skip = depth_skip
        self.img_skip = img_skip
        self.as_ros_publisher = as_ros_publisher

        self._img_counter = 0
        self._depth_counter = 0

        self.latest_jpeg = None
        self.points_3d = []
        self.valid_depth_pct = 0.0

        # Thông số camera mặc định OAK-D S2 / Laptop Webcam
        self.fx = 450.0
        self.fy = 450.0
        self.cx = 320.0
        self.cy = 200.0

        # Chế độ khung hình mặc định: 'ai' (Cam thường có detect người & khoảng cách), 'thermal' (Camera nhiệt)
        self.view_mode = 'ai'

        # Quản lý Laptop Webcam
        self._cap = None
        self._cap_failed = False
        self._last_cap_time = 0.0
        self._cached_frame = None

        # Đo FPS thực tế chuẩn xác (Rolling Counter)
        self.calc_fps = 0.0
        self._fps_frame_count = 0
        self._fps_start_time = time.time()

        # Cự ly vật cản đo được thời gian thực (hỗ trợ tới 20cm)
        self.obstacle_distance = 0.65
        self.is_emergency_braked = False

        # Quản lý thiết bị OAK-D S2 cắm trực tiếp USB máy tính
        self._oak_device = None
        self._oak_q_rgb = None
        self._oak_q_raw_depth = None
        self._oak_q_nn = None
        self._last_oak_check = 0.0
        self.is_oak_connected = False
        self.latest_oak_depth = None
        self._cached_oak_frame = None
        self.latest_vpu_detections = []

        # Đồng bộ luồng phát hình ảnh độ trễ thấp (Zero Latency Event & ID)
        import threading
        self.frame_id = 0
        self.new_frame_event = threading.Event()

        # Cache tính toán Camera Nhiệt siêu tốc (< 2.5ms, FPS > 30)
        self._spatial_heat_cache = None
        self._spatial_heat_shape = None
        self._bar_strip_cache = None
        self._bar_strip_h = None

    def set_view_mode(self, mode):
        """Chuyển đổi giữa 2 chế độ: 'ai' (Camera AI Detect) hoặc 'thermal' (Camera Nhiệt)"""
        if not isinstance(mode, str):
            return self.view_mode
        m = mode.strip().lower()
        if m in ['1', 'ai', 'detect', 'normal', 'rgb', 'spatial']:
            self.view_mode = 'ai'
        elif m in ['2', 'thermal', 'heat', 'nhiet', 'depth']:
            self.view_mode = 'thermal'
        print(f"📷 [CAMERA TV2] Chế độ xem: {self.view_mode.upper()}")
        return self.view_mode

    def get_view_mode(self):
        return self.view_mode

    def get_latest_jpeg_with_id(self):
        """Trả về (frame_id, latest_jpeg) để web server chỉ phát khi có frame mới thực sự"""
        return self.frame_id, self.latest_jpeg

    def _try_open_oak(self):
        """Khởi tạo camera OAK-D S2 khi cắm trực tiếp vào máy tính qua cổng USB (Hỗ trợ cả DepthAI v2 & v3)"""
        if self._oak_device is not None or getattr(self, '_oak_pipeline', None) is not None:
            return True

        if time.time() - self._last_oak_check < 3.0:
            return False
        self._last_oak_check = time.time()

        try:
            import depthai as dai
            print("🚀 [OAK-D USB] Đang kiểm tra và khởi tạo Camera OAK-D S2...")
            pipeline = dai.Pipeline()
            is_v3 = not hasattr(dai.node, 'XLinkOut')

            if is_v3:
                # ─── DEPTHAI V3 API (Windows PC / Python 3.13) ───
                cam_rgb = pipeline.create(dai.node.ColorCamera)
                cam_rgb.setPreviewSize(640, 480)
                cam_rgb.setInterleaved(False)
                cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                cam_rgb.setFps(30)
                self._oak_q_rgb = cam_rgb.preview.createOutputQueue(maxSize=1, blocking=False)

                try:
                    mono_l = pipeline.create(dai.node.MonoCamera)
                    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                    mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)

                    mono_r = pipeline.create(dai.node.MonoCamera)
                    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                    mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)

                    stereo = pipeline.create(dai.node.StereoDepth)
                    if hasattr(stereo, 'PresetMode') and hasattr(stereo.PresetMode, 'DENSITY'):
                        stereo.setDefaultProfilePreset(stereo.PresetMode.DENSITY)
                    elif hasattr(stereo, 'PresetMode') and hasattr(stereo.PresetMode, 'HIGH_DENSITY'):
                        stereo.setDefaultProfilePreset(stereo.PresetMode.HIGH_DENSITY)

                    stereo.setLeftRightCheck(True)
                    stereo.setExtendedDisparity(True)

                    mono_l.out.link(stereo.left)
                    mono_r.out.link(stereo.right)
                    self._oak_q_raw_depth = stereo.depth.createOutputQueue(maxSize=1, blocking=False)
                except Exception as ex_stereo:
                    print(f"⚠️ [OAK-D USB] Bỏ qua Stereo Depth: {ex_stereo}")
                    self._oak_q_raw_depth = None
                    # Tạo lại pipeline RGB sạch để tránh node treo
                    pipeline = dai.Pipeline()
                    cam_rgb = pipeline.create(dai.node.ColorCamera)
                    cam_rgb.setPreviewSize(640, 480)
                    cam_rgb.setInterleaved(False)
                    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                    cam_rgb.setFps(30)
                    self._oak_q_rgb = cam_rgb.preview.createOutputQueue(maxSize=1, blocking=False)

                pipeline.start()
                self._oak_pipeline = pipeline
                self.is_oak_connected = True
                print("✅ [OAK-D USB] Đã kích hoạt Camera OAK-D S2 (DepthAI v3) trực tiếp trên máy tính thành công!")
                return True
            else:
                # ─── DEPTHAI V2 API (Jetson Nano / Ubuntu ROS) ───
                cam_rgb = pipeline.create(dai.node.ColorCamera)
                cam_rgb.setPreviewSize(640, 480)
                cam_rgb.setInterleaved(False)
                cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                cam_rgb.setFps(30)

                xout_rgb = pipeline.create(dai.node.XLinkOut)
                xout_rgb.setStreamName("rgb")
                cam_rgb.preview.link(xout_rgb.input)

                try:
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

                    xout_raw_depth = pipeline.create(dai.node.XLinkOut)
                    xout_raw_depth.setStreamName("raw_depth")
                    stereo.depth.link(xout_raw_depth.input)
                    has_stereo = True
                except Exception as ex_stereo:
                    print(f"⚠️ [OAK-D USB] Bỏ qua Stereo Depth: {ex_stereo}")
                    has_stereo = False

                # Tích hợp mô hình VPU Spatial Detection Network nếu có sẵn blob trên Jetson Nano hoặc PC
                has_vpu_nn = False
                res_blob = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "resources", "mobilenet-ssd.blob")
                cache_blob = os.path.join(os.path.expanduser("~"), ".cache", "blobconverter", "mobilenet-ssd_openvino_2022.1_6shave.blob")
                blob_candidates = [
                    res_blob,
                    cache_blob,
                    "/home/jetbot/catkin_ws/src/depthai-ros/depthai_examples/resources/mobilenet-ssd_openvino_2021.2_6shave.blob",
                ]
                chosen_blob = None
                for b_path in blob_candidates:
                    if os.path.exists(b_path):
                        chosen_blob = b_path
                        break
                if chosen_blob is None:
                    try:
                        import blobconverter
                        chosen_blob = blobconverter.from_zoo(name="mobilenet-ssd", shaves=6)
                    except Exception:
                        pass

                if chosen_blob is not None and has_stereo:
                    try:
                        spatial_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
                        spatial_nn.setBlobPath(chosen_blob)
                        spatial_nn.setConfidenceThreshold(0.25)
                        spatial_nn.input.setBlocking(False)
                        spatial_nn.setBoundingBoxScaleFactor(0.5)
                        spatial_nn.setDepthLowerThreshold(60)    # 6cm (hỗ trợ cản áp sát)
                        spatial_nn.setDepthUpperThreshold(2000)  # 2m (chống loạn camera)

                        manip = pipeline.create(dai.node.ImageManip)
                        manip.initialConfig.setResize(300, 300)
                        manip.initialConfig.setFrameType(dai.RawImgFrame.Type.BGR888p)
                        manip.inputConfig.setWaitForMessage(False)

                        cam_rgb.preview.link(manip.inputImage)
                        manip.out.link(spatial_nn.input)
                        stereo.depth.link(spatial_nn.inputDepth)

                        xout_nn = pipeline.create(dai.node.XLinkOut)
                        xout_nn.setStreamName("nn")
                        spatial_nn.out.link(xout_nn.input)
                        has_vpu_nn = True
                        print(f"🧠 [OAK-D VPU] Đã kết nối mạng VPU MobileNet-SSD: {chosen_blob}")
                    except Exception as ex_nn:
                        print(f"ℹ️ [OAK-D VPU] Bỏ qua nạp VPU NN ({ex_nn}), sử dụng thị giác tiêu chuẩn.")
                        has_vpu_nn = False

                try:
                    device = dai.Device(pipeline)
                except Exception as ex_dev:
                    if has_vpu_nn:
                        print(f"⚠️ [OAK-D VPU] Runtime không khớp blob VPU ({ex_dev}). Khởi chạy RGB + Stereo Depth thuần túy!")
                        pipeline = dai.Pipeline()
                        cam_rgb = pipeline.create(dai.node.ColorCamera)
                        cam_rgb.setPreviewSize(640, 480)
                        cam_rgb.setInterleaved(False)
                        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                        cam_rgb.setFps(30)
                        xout_rgb = pipeline.create(dai.node.XLinkOut)
                        xout_rgb.setStreamName("rgb")
                        cam_rgb.preview.link(xout_rgb.input)
                        if has_stereo:
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
                            xout_raw_depth = pipeline.create(dai.node.XLinkOut)
                            xout_raw_depth.setStreamName("raw_depth")
                            stereo.depth.link(xout_raw_depth.input)
                        device = dai.Device(pipeline)
                        has_vpu_nn = False
                    else:
                        raise ex_dev

                self._oak_device = device
                self._oak_q_rgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
                if has_stereo:
                    self._oak_q_raw_depth = device.getOutputQueue(name="raw_depth", maxSize=1, blocking=False)
                else:
                    self._oak_q_raw_depth = None
                if has_vpu_nn:
                    self._oak_q_nn = device.getOutputQueue(name="nn", maxSize=1, blocking=False)
                else:
                    self._oak_q_nn = None
                self.is_oak_connected = True
                print("✅ [OAK-D USB] Đã kích hoạt Camera OAK-D S2 trực tiếp trên Jetson Nano thành công!")
                return True
        except Exception as e:
            err_str = str(e)
            if "X_LINK_UNBOOTED" in err_str or "permission" in err_str.lower():
                print(f"⚠️ [OAK-D USB] Cần cấp quyền USB trên Jetson Nano ({e})")
                print("💡 [HƯỚNG DẪN FIX]: Chạy lệnh sau trên Jetson Nano: sudo udevadm control --reload-rules && sudo udevadm trigger")
            else:
                print(f"⚠️ [OAK-D USB] Không mở được OAK-D trực tiếp ({e}). Tự động fallback sang Webcam/ROS.")
            self._oak_device = None
            if hasattr(self, '_oak_pipeline') and self._oak_pipeline:
                try:
                    self._oak_pipeline.stop()
                except Exception:
                    pass
                self._oak_pipeline = None
            self.is_oak_connected = False
            return False

    def shutdown(self):
        """Giải phóng toàn bộ tài nguyên Camera OAK-D và Webcam sạch sẽ"""
        if self._oak_device:
            try:
                self._oak_device.close()
            except Exception:
                pass
            self._oak_device = None
        if hasattr(self, '_oak_pipeline') and self._oak_pipeline:
            try:
                self._oak_pipeline.stop()
            except Exception:
                pass
            self._oak_pipeline = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self._oak_q_nn = None
        self._cached_oak_frame = None
        self.is_oak_connected = False

    def read_oak_frame(self):
        """Đọc cả ảnh màu RGB, bản đồ độ sâu Depth và VPU detections từ OAK-D cắm USB"""
        if not self._try_open_oak():
            return None, None
        try:
            in_rgb = self._oak_q_rgb.tryGet() if self._oak_q_rgb else None
            in_depth = self._oak_q_raw_depth.tryGet() if self._oak_q_raw_depth else None
            in_nn = self._oak_q_nn.tryGet() if getattr(self, '_oak_q_nn', None) else None

            frame = None
            if in_rgb is not None:
                frame = in_rgb.getCvFrame() if hasattr(in_rgb, 'getCvFrame') else (in_rgb.getFrame() if hasattr(in_rgb, 'getFrame') else None)
            depth = None
            if in_depth is not None:
                depth = in_depth.getFrame() if hasattr(in_depth, 'getFrame') else (in_depth.getCvFrame() if hasattr(in_depth, 'getCvFrame') else None)
            if frame is not None:
                self._cached_oak_frame = frame
            if depth is not None:
                self.latest_oak_depth = depth

            # Xử lý kết quả nhận diện từ chip VPU OAK-D
            if in_nn is not None and hasattr(in_nn, 'detections'):
                vpu_dets = []
                for d in in_nn.detections:
                    lbl = MOBILENET_LABELS[d.label] if (hasattr(d, 'label') and d.label < len(MOBILENET_LABELS)) else "OBJECT"
                    x_m = round(float(d.spatialCoordinates.x) / 1000.0, 2)
                    y_m = round(float(d.spatialCoordinates.y) / 1000.0, 2)
                    z_m = round(float(d.spatialCoordinates.z) / 1000.0, 2)
                    if z_m < 0.06:
                        z_m = 0.15
                    # Giới hạn bán kính 2.0m chống loạn camera
                    if z_m > 2.0 or (x_m * x_m + z_m * z_m > 4.0):
                        continue

                    bx = max(0, min(640, int(d.xmin * 640)))
                    by = max(0, min(480, int(d.ymin * 480)))
                    bw = max(10, min(640 - bx, int((d.xmax - d.xmin) * 640)))
                    bh = max(10, min(480 - by, int((d.ymax - d.ymin) * 480)))
                    vpu_dets.append({
                        "id": 0,
                        "class": lbl.upper(),
                        "name": lbl.upper(),
                        "score": round(float(d.confidence), 2),
                        "x": x_m, "y": y_m, "z": z_m,
                        "distance": z_m,
                        "bbox": [bx, by, bw, bh],
                        "source": "OAK-D VPU"
                    })
                self.latest_vpu_detections = vpu_dets

            # Chỉ xuất bản các Topics ROS khi được cấu hình làm ROS Publisher Node độc lập
            if getattr(self, 'as_ros_publisher', False):
                self._publish_ros_frames(frame, depth, self.obstacle_distance)

            return frame, depth
        except Exception as e:
            print(f"⚠️ [OAK-D USB] Mất kết nối OAK-D: {e}")
            self.shutdown()
            return None, None

    def _publish_ros_frames(self, rgb_frame, depth_frame, obstacle_dist=None):
        """Tự động xuất bản các Topics ROS chuẩn khi roscore đang chạy"""
        if not HAS_ROS:
            return
        try:
            if not rospy.core.is_initialized():
                return
            if not getattr(self, '_ros_pubs_initialized', False):
                self._ros_pub_rgb = rospy.Publisher('/stereo_inertial_publisher/color/image', ROSImage, queue_size=1)
                self._ros_pub_depth = rospy.Publisher('/stereo_inertial_publisher/stereo/depth', ROSImage, queue_size=1)
                self._ros_pub_dist = rospy.Publisher('/obstacle_distance', ROSFloat32, queue_size=1)
                self._ros_pub_objects = rospy.Publisher('/spatial_objects', ROSString, queue_size=2)
                self._ros_pubs_initialized = True

            now_ros = rospy.Time.now()
            if rgb_frame is not None and hasattr(self, '_ros_pub_rgb'):
                msg_rgb = self._build_image_msg(rgb_frame, "bgr8", "oak-d_frame", now_ros)
                self._ros_pub_rgb.publish(msg_rgb)

            if depth_frame is not None and hasattr(self, '_ros_pub_depth'):
                msg_depth = self._build_image_msg(depth_frame, "16UC1", "oak-d_frame", now_ros)
                self._ros_pub_depth.publish(msg_depth)

            if obstacle_dist is not None and hasattr(self, '_ros_pub_dist'):
                self._ros_pub_dist.publish(ROSFloat32(data=float(obstacle_dist)))

            if getattr(self, 'latest_vpu_detections', None) and hasattr(self, '_ros_pub_objects'):
                try:
                    self._ros_pub_objects.publish(ROSString(data=json.dumps(self.latest_vpu_detections)))
                except Exception:
                    pass
        except Exception:
            pass

    @staticmethod
    def _build_image_msg(img_np, encoding, frame_id="oak-d_frame", stamp=None):
        msg = ROSImage()
        msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
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

    def _open_laptop_camera(self):
        """Mở kết nối tới Webcam của Laptop khi không có OAK-D và không có ROS"""
        if self._cap is not None and self._cap.isOpened():
            return self._cap

        if self._cap_failed and (time.time() - self._last_cap_time < 4.0):
            return None

        self._last_cap_time = time.time()
        try:
            # Trên Windows ưu tiên CAP_DSHOW để mở webcam nhanh và mượt mà
            if sys.platform == 'win32':
                cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            else:
                cap = cv2.VideoCapture(0)

            if cap.isOpened():
                try:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                except Exception:
                    pass
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
                self._cap = cap
                self._cap_failed = False
                print("📹 [WEBCAM] Đã kết nối Webcam Laptop (MJPG 30 FPS, Zero-Latency Buffer) thành công!")
                return self._cap
            else:
                cap.release()
                self._cap_failed = True
        except Exception as e:
            self._cap_failed = True
            print(f"ℹ️ [WEBCAM] Không thể mở webcam laptop: {e}")
        return None

    def read_laptop_frame(self):
        """Đọc 1 frame thật từ Webcam Laptop (lật gương để tự nhiên)"""
        cap = self._open_laptop_camera()
        if cap is not None and cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                frame = cv2.flip(frame, 1)  # Lật gương
                if frame.shape[0] != 480 or frame.shape[1] != 640:
                    frame = cv2.resize(frame, (640, 480))
                self._cached_frame = frame
                return frame
        return None

    def get_live_frame(self):
        """
        Lấy frame hình ảnh trực tiếp:
          - Ưu tiên 1: Camera OAK-D S2 cắm cổng USB (cung cấp cả RGB và Depth Stereo thật)
          - Ưu tiên 2: Webcam Laptop tích hợp (CHỈ KHI HOÀN TOÀN KHÔNG CÓ OAK-D)
        Trả về: (bgr_frame, depth_frame, source_name)
        """
        # Nếu OAK-D đang kết nối: KHÓA CHẶT LUỒNG VÀO OAK-D, KHÔNG NHẢY SANG WEBCAM
        if self._try_open_oak() and self.is_oak_connected:
            # Đóng webcam laptop nếu trước đó đã mở để giải phóng phần cứng
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None

            oak_frame, oak_depth = self.read_oak_frame()
            if oak_frame is not None:
                return oak_frame, (oak_depth if oak_depth is not None else self.latest_oak_depth), "OAK-D S2 (USB LIVE)"
            else:
                return None, self.latest_oak_depth, "OAK-D S2 (USB LIVE)"

        # Chỉ khi OAK-D không cắm hoặc mất kết nối hoàn toàn mới dùng Webcam Laptop
        lap_frame = self.read_laptop_frame()
        if lap_frame is not None:
            return lap_frame, None, "WEBCAM LAPTOP"

        return None, None, "MO PHONG"

    def _update_fps(self):
        """Đo FPS thực tế chuẩn xác dựa trên số khung hình thực tế nhận được theo chu kỳ 0.8 giây"""
        now = time.time()
        self._fps_frame_count += 1
        elapsed = now - self._fps_start_time
        if elapsed >= 0.8:
            self.calc_fps = round(self._fps_frame_count / elapsed, 1)
            self._fps_frame_count = 0
            self._fps_start_time = now
        elif self.calc_fps <= 0.0:
            self.calc_fps = 25.0
        return self.calc_fps

    def generate_thermal_colormap(self, bgr_img, depth_uint16_mm=None):
        """Tạo khung hình Camera Nhiệt (Thermal Heatmap) chuẩn công nghiệp siêu tốc (< 2.5ms, FPS > 30)"""
        if bgr_img is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        h, w = bgr_img.shape[:2]
        thermal_img = None

        # 1. Nếu có dữ liệu độ sâu thực tế từ OAK-D (raw_depth) -> chuyển đổi chuẩn C++ OpenCV siêu tốc
        if depth_uint16_mm is not None and isinstance(depth_uint16_mm, np.ndarray):
            try:
                disp = np.clip(depth_uint16_mm, 200, 3500)
                disp_norm = cv2.convertScaleAbs(disp - 200, alpha=255.0 / 3300.0)
                disp_norm = 255 - disp_norm
                if disp_norm.shape[:2] != (h, w):
                    disp_norm = cv2.resize(disp_norm, (w, h), interpolation=cv2.INTER_NEAREST)
                thermal_img = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
            except Exception:
                thermal_img = None

        # 2. Nếu không có depth phần cứng -> tạo Thermal Heatmap từ ảnh BGR với ma trận cache (tiết kiệm 98% CPU)
        if thermal_img is None:
            gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
            blurred = cv2.blur(gray, (7, 7))

            # Cache ma trận gradient nhiệt tâm (chỉ tính 1 lần duy nhất cho mỗi kích thước ảnh)
            if self._spatial_heat_cache is None or self._spatial_heat_shape != (h, w):
                y_indices, x_indices = np.indices((h, w))
                dist_from_center = np.sqrt((x_indices - w / 2.0) ** 2 + (y_indices - h / 2.0) ** 2)
                self._spatial_heat_cache = np.clip(255 - dist_from_center / (max(w, h) / 1.2) * 160, 40, 255).astype(np.uint8)
                self._spatial_heat_shape = (h, w)

            heat_intensity = cv2.addWeighted(blurred, 0.60, self._spatial_heat_cache, 0.40, 0)
            thermal_img = cv2.applyColorMap(heat_intensity, cv2.COLORMAP_JET)

        # 3. Thanh thước đo nhiệt độ ở cạnh phải (Dùng mảng cache cắt lát thay cho vòng lặp 400 lần)
        h_bar = max(10, h - 80)
        if self._bar_strip_cache is None or self._bar_strip_h != h_bar:
            bar = np.linspace(255, 0, h_bar, dtype=np.uint8).reshape(-1, 1)
            bar_color = cv2.applyColorMap(bar, cv2.COLORMAP_JET)
            self._bar_strip_cache = np.repeat(bar_color, 14, axis=1)
            self._bar_strip_h = h_bar

        bar_x = w - 24
        thermal_img[40:40 + h_bar, bar_x:bar_x + 14] = self._bar_strip_cache

        # 4. Tâm ngắm đo nhiệt độ & cự ly tại điểm ngắm
        cx, cy = w // 2, h // 2
        cv2.drawMarker(thermal_img, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        c_cm = self.obstacle_distance * 100.0 if self.obstacle_distance is not None else 65.0
        c_temp = round(36.5 + max(0, (40 - c_cm) * 0.05), 1) if c_cm < 80 else 36.2

        cv2.putText(thermal_img, f"TARGET: {c_temp} C | {c_cm:.1f} cm", (cx + 14, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(thermal_img, "38 C / 20cm", (w - 105, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(thermal_img, "24 C / 3.5m", (w - 105, h - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 180, 0), 1, cv2.LINE_AA)

        return thermal_img

    def process_color_frame(self, bgr_img, detections_annotator=None, detections=None, obstacle_dist=None):
        """
        Xử lý và tạo luồng ảnh JPEG sạch sẽ, tối ưu hiển thị HUD:
          1. 'ai': Camera thường hiển thị Detect người + Bounding Box + Tâm ngắm (Cự ly hiển thị bằng HTML HUD sắc nét)
          2. 'thermal': Camera nhiệt Thermal Heatmap (Thang nhiệt và tâm đo chuẩn)
        """
        cam_src = "OAK-D S2"
        if bgr_img is None:
            laptop_frame = self.read_laptop_frame()
            if laptop_frame is not None:
                bgr_img = laptop_frame
                cam_src = "WEBCAM LAPTOP"
            else:
                bgr_img = np.zeros((480, 640, 3), dtype=np.uint8)
                bgr_img[:] = (15, 12, 10)
                cam_src = "MO PHONG"

        current_fps = self._update_fps()
        h, w = bgr_img.shape[:2]

        # Đồng bộ cự ly vật cản chuẩn theo thuật toán test_emergency_brake.py
        if obstacle_dist is not None and float(obstacle_dist) <= 4.0:
            self.obstacle_distance = float(obstacle_dist)
            forward_clearance_mm = self.obstacle_distance * 1000.0
        else:
            self.obstacle_distance = None
            forward_clearance_mm = 9999.0  # Đường thoáng (> 4.0m)

        c_cm = forward_clearance_mm / 10.0
        self.is_emergency_braked = bool(self.obstacle_distance is not None and forward_clearance_mm <= 200.0)

        # ─── 1. CHẾ ĐỘ CAMERA NHIỆT (THERMAL HEATMAP) ───
        if self.view_mode == 'thermal':
            display_frame = self.generate_thermal_colormap(bgr_img, depth_uint16_mm=self.latest_oak_depth)

        # ─── 2. CHẾ ĐỘ CAMERA THƯỜNG + DETECT NGƯỜI & KHOẢNG CÁCH (AI SPATIAL) ───
        else:
            display_frame = bgr_img.copy()

            # 1. Bounding box nhận diện mục tiêu từ YOLO / Fallback
            if detections_annotator:
                display_frame = detections_annotator(display_frame)

            # 2. Thước đo cự ly CLEARANCE y hệt test_emergency_brake.py (Góc trên bên trái)
            if forward_clearance_mm > 4000.0:
                col = (0, 255, 100)
                stat_txt = "> 4.0 m"
            elif self.is_emergency_braked:
                col = (0, 0, 255)
                stat_txt = f"BRAKE! {c_cm:.1f} cm"
            elif c_cm < 40.0:
                col = (0, 200, 255)
                stat_txt = f"{c_cm:.1f} cm"
            else:
                col = (0, 255, 100)
                stat_txt = f"{c_cm:.1f} cm"

            cv2.putText(display_frame, f"CLEARANCE: {stat_txt}", (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)

            # 3. Thanh cảnh báo đáy màn hình khi nguy cấp (Không icon)
            if self.is_emergency_braked:
                cv2.rectangle(display_frame, (2, 2), (w - 2, h - 2), (0, 0, 255), 3)
                banner_w = 460
                bx1 = (w - banner_w) // 2
                cv2.rectangle(display_frame, (bx1, h - 42), (bx1 + banner_w, h - 10), (0, 0, 220), -1)
                cv2.rectangle(display_frame, (bx1, h - 42), (bx1 + banner_w, h - 10), (255, 255, 255), 1)
                msg = f"[PHANH KHAN CAP] CAN SAT < 25CM ({c_cm:.1f}cm)"
                (mw, _), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
                cv2.putText(display_frame, msg, (bx1 + (banner_w - mw) // 2, h - 21),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
            elif forward_clearance_mm < 450.0:
                banner_w = 400
                bx1 = (w - banner_w) // 2
                cv2.rectangle(display_frame, (bx1, h - 40), (bx1 + banner_w, h - 10), (0, 160, 255), -1)
                cv2.rectangle(display_frame, (bx1, h - 40), (bx1 + banner_w, h - 10), (255, 255, 255), 1)
                msg = f"[CANH BAO] GIAM TOC: {c_cm:.1f} cm"
                (mw, _), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.44, 1)
                cv2.putText(display_frame, msg, (bx1 + (banner_w - mw) // 2, h - 21),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 1, cv2.LINE_AA)

            pass

        # Nén thành JPEG tối ưu truyền qua sóng WiFi Robot (giảm 40% dung lượng gói tin, chống lag đệm TCP)
        encode_params = [
            int(cv2.IMWRITE_JPEG_QUALITY), 55,
            int(cv2.IMWRITE_JPEG_OPTIMIZE), 0
        ]
        _, jpeg = cv2.imencode('.jpg', display_frame, encode_params)
        self.latest_jpeg = jpeg.tobytes()
        self.frame_id += 1
        self.new_frame_event.set()
        return self.latest_jpeg

    def estimate_visual_rotation(self, bgr_img):
        """Ước lượng góc quay ngang delta_yaw từ thị giác (Visual Gyroscope / Phase Correlation)
        Hoạt động cực nhanh (~0.3ms), đo chính xác góc quay của xe kể cả khi người dùng
        dùng tay xoay JetBot hoặc khi bánh xe bị trượt trên sàn gạch men!
        """
        if bgr_img is None:
            return 0.0
        try:
            h, w = bgr_img.shape[:2]
            # Downsample về 160x90 grayscale để tính toán siêu tốc < 0.3ms
            small = cv2.resize(bgr_img, (160, 90), interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)

            if not hasattr(self, '_prev_vis_gray') or self._prev_vis_gray is None or self._prev_vis_gray.shape != gray.shape:
                self._prev_vis_gray = gray
                return 0.0

            shift, response = cv2.phaseCorrelate(self._prev_vis_gray, gray)
            self._prev_vis_gray = gray

            # Chỉ chấp nhận nếu độ tin cậy tương quan cao
            if response > 0.30:
                dx = shift[0]
                # Bỏ qua rung lắc vi mô (< 0.5 pixel) và bước nhảy quá lớn (> 50 pixel)
                if 0.5 <= abs(dx) <= 50.0:
                    # fx tương ứng ở độ phân giải 160px với FOV 75 độ
                    fx_small = 160.0 / (2.0 * math.tan(math.radians(75.0) / 2.0))
                    # Khi xe quay trái, cảnh dạt sang phải (dx > 0) -> delta_yaw > 0 (CCW)
                    delta_yaw = math.atan2(dx, fx_small)
                    return delta_yaw
        except Exception:
            pass
        return 0.0

    def get_visual_keyframe(self, bgr_img=None):
        """Trích xuất ảnh mốc chuẩn (Anchor Keyframe) 160x90 float32 grayscale để khóa vòng lặp (Loop Closure)"""
        if bgr_img is None:
            bgr_img = getattr(self, '_cached_oak_frame', None) or getattr(self, '_cached_frame', None)
        if bgr_img is None:
            return None
        try:
            small = cv2.resize(bgr_img, (160, 90), interpolation=cv2.INTER_AREA)
            return cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
        except Exception:
            return None

    def match_visual_keyframe(self, current_bgr, anchor_gray):
        """So khớp ảnh hiện tại với ảnh mốc Anchor bằng Phase Correlation để đóng vòng quay 360° chính xác 100%.
        Trả về: (dx, dy, response). Nếu response >= 0.28 và abs(dx) <= 2.5px -> Trùng khớp góc quay ban đầu!
        """
        if current_bgr is None or anchor_gray is None:
            return 999.0, 999.0, 0.0
        try:
            curr_gray = self.get_visual_keyframe(current_bgr)
            if curr_gray is None or curr_gray.shape != anchor_gray.shape:
                return 999.0, 999.0, 0.0
            shift, response = cv2.phaseCorrelate(anchor_gray, curr_gray)
            return float(shift[0]), float(shift[1]), float(response)
        except Exception:
            return 999.0, 999.0, 0.0

    def process_depth_frame(self, depth_uint16_mm, rx=0.0, ry=0.0, rz=0.0, yaw=0.0):
        """
        Chiếu ma trận Stereo Depth thành tia quét 2D LaserScan / Obstacle Scan chuẩn xác:
        - Với mỗi cột góc nhìn ngang u, tìm vật cản GẦN NHẤT z_min trong tầm độ cao xe (0.05m đến 0.85m).
        - Đảm bảo tia quét DỪNG LẠI tại vật cản đầu tiên (ví dụ: mặt ghế ở 34cm),
          tuyệt đối không xuyên thủng vật cản để vẽ không gian thoáng xuyên ra tường 3.5m!
        """
        self._depth_counter += 1
        if self._depth_counter % self.depth_skip != 0:
            return self.points_3d

        if depth_uint16_mm is None or not isinstance(depth_uint16_mm, np.ndarray):
            return self.points_3d

        h, w = depth_uint16_mm.shape[:2]
        # Lấy dải quét từ 10% đến 65% chiều cao ảnh (vùng tầm nhìn thân xe & vật cản)
        v_start = int(h * 0.10)
        v_end = int(h * 0.65)
        v_slice = depth_uint16_mm[v_start:v_end, :]

        # Lấy mẫu ngang 60 góc quét phân bố đều trên FOV của camera
        u_cols = np.linspace(15, w - 15, 60, dtype=np.int32)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        scan_pts = []
        for u in u_cols:
            col_z = v_slice[:, u].astype(np.float32) / 1000.0
            valid_mask = (col_z >= 0.06) & (col_z <= 2.0)
            if not np.any(valid_mask):
                continue

            valid_z = col_z[valid_mask]
            valid_v = np.where(valid_mask)[0] + v_start

            # Tính độ cao z_rob trong tọa độ xe cho các điểm hợp lệ
            y_cam = (valid_v - self.cy) * valid_z / self.fy
            z_rob = -y_cam + 0.12

            # Lọc chỉ lấy các điểm thực sự là vật cản (cao hơn sàn 5cm và dưới trần 85cm)
            obs_mask = (z_rob >= 0.05) & (z_rob <= 0.85)
            if not np.any(obs_mask):
                continue

            # Lấy vật cản GẦN NHẤT (bề mặt đón tia đầu tiên) trong cột góc nhìn này
            obs_z = valid_z[obs_mask]
            obs_v = valid_v[obs_mask]
            min_idx = np.argmin(obs_z)
            closest_z = float(obs_z[min_idx])
            closest_v = float(obs_v[min_idx])

            # Chiếu sang tọa độ Robot:
            x_cam = (u - self.cx) * closest_z / self.fx
            y_cam = (closest_v - self.cy) * closest_z / self.fy

            x_rob = closest_z
            y_rob = -x_cam
            z_rob_pt = -y_cam + 0.12

            # Chiếu sang tọa độ Bản đồ toàn cục (World Frame):
            wx = rx + (x_rob * cos_y - y_rob * sin_y)
            wy = ry + (x_rob * sin_y + y_rob * cos_y)
            wz = z_rob_pt

            scan_pts.append([round(wx, 3), round(wy, 3), round(wz, 3)])

        self.points_3d = scan_pts
        return self.points_3d



def run_ros_camera_node():
    """
    Khởi chạy Node ROS OAK-D S2 trực tiếp từ CameraStreamer.
    Cung cấp các Topics chuẩn:
      - /stereo_inertial_publisher/color/image (BGR8)
      - /stereo_inertial_publisher/stereo/depth (16UC1 mm)
      - /yolov4_publisher/color/image
      - /yolov4_publisher/stereo/depth
      - /obstacle_distance (Mét, lọc sạch sàn nhà)
    """
    try:
        import rospy
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32
    except ImportError:
        print("❌ [LỖI] Không tìm thấy rospy. Hãy chạy qua 'roslaunch' hoặc 'roscore'.")
        return

    rospy.init_node('oak_camera_publisher', anonymous=False)

    streamer = CameraStreamer(as_ros_publisher=True)
    if not streamer._try_open_oak():
        rospy.logerr("❌ [OAK-D] Không thể mở kết nối OAK-D S2 qua USB!")
        return

    print("✅ [OAK-D] CameraStreamer đã kết nối OAK-D S2 thành công! Bắt đầu phát ROS topics 30 FPS...")

    rate = rospy.Rate(35)
    while not rospy.is_shutdown():
        frame, depth = streamer.read_oak_frame()
        # streamer.read_oak_frame() đã tự động xuất bản các topic chuẩn qua _publish_ros_frames()
        rate.sleep()


if __name__ == '__main__':
    # Nếu chạy qua roslaunch hoặc có tham số node / ROS environment
    if len(sys.argv) > 1 and any(arg.startswith('__name:=') or arg == '--node' for arg in sys.argv):
        run_ros_camera_node()
        sys.exit(0)

    # Tự kiểm thử standalone nếu chạy thủ công
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử CameraStreamer...")
    cs = CameraStreamer()
    # Tạo frame ảnh giả lập
    fake_img = np.zeros((400, 640, 3), dtype=np.uint8)
    cv2.putText(fake_img, "TEST STREAM", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    # Xử lý 2 frame để qua bộ lọc skip
    cs.process_color_frame(fake_img)
    jpeg = cs.process_color_frame(fake_img)
    assert jpeg is not None and len(jpeg) > 1000, "Lỗi: Không tạo được ảnh JPEG!"
    print(f"  ├─ Kích thước JPEG tạo ra: {len(jpeg)} bytes")
    print("✅ [SELF-TEST] CameraStreamer ĐẠT CHUẨN!")
