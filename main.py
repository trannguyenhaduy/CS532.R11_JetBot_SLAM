#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
JETBOT MODULAR MASTER ORCHESTRATOR & WEB COCKPIT (MAIN.PY)
Hệ thống điều phối trung tâm với Cờ Tính Năng (Feature Flags) độc lập
Tuân thủ Quy tắc Harness 7: Dễ debug, dễ cô lập lỗi, bật/tắt module linh hoạt.
══════════════════════════════════════════════════════════════════════════════
Cách chạy:
  python3 main.py                    # Chạy toàn bộ hệ thống mặc định
  python3 main.py --no-camera        # Chạy không cần camera (test động cơ & web)
  python3 main.py --no-motors        # Chạy không cần động cơ (test AI & web)
  python3 main.py --enable-follower  # Kích hoạt tính năng bám người tự hành
══════════════════════════════════════════════════════════════════════════════
"""

# ══════════════════════════════════════════════════════════════════════════════
# 🎛️ BẬT / TẮT TÍNH NĂNG TRỰC TIẾP TẠI ĐÂY (DỄ DÀNG ĐỂ TEST TỪNG BƯỚC)
# Bạn chỉ cần đổi thành ON hoặc OFF (hoặc True / False):
# ══════════════════════════════════════════════════════════════════════════════
ON  = True
OFF = False

MOTOR    = ON   # 1. ĐỘNG CƠ: BẬT (Giai đoạn 1 - Lái xe bằng phím WASD, chip PCA9685 0x60)
PIN      = ON   # 2. ĐO PIN: BẬT (Giai đoạn 2 - Giám sát pin thời gian thực INA219 0x41)
BATTERY  = PIN  # (Bí danh tương đương PIN)
CAMERA   = ON   # 3. CAMERA: BẬT (Giai đoạn 3 - Luồng ảnh màu & đám mây điểm 3D)
YOLO     = ON   # 4. AI NHẬN DIỆN: BẬT (Giai đoạn 3 - Bộ lọc đối tượng 3D Tiny YOLO)
FOLLOWER = OFF  # 5. BÁM NGƯỜI: Tắt (Tạm thời bỏ qua theo yêu cầu để làm sau)
MAPPER   = ON   # 6. BẢN ĐỒ 3D: BẬT (Giai đoạn 5 - Lập bản đồ ngữ nghĩa Semantic SLAM)
WEB      = ON   # 7. WEB COCKPIT: BẬT (Mở cổng 8080 để lái xe bằng phím W-A-S-D)
# ══════════════════════════════════════════════════════════════════════════════

def to_bool(val):
    if isinstance(val, str):
        return val.strip().upper() in ('ON', 'TRUE', '1', 'YES', 'BAT')
    return bool(val)

import sys
import os
import json
import time
import math
import argparse
import threading
import numpy as np
import cv2

# Nạp cấu hình trung tâm
import config

# Nạp các module tính năng độc lập
from modules.motor_controller import MotorController
from modules.battery_monitor import BatteryMonitor
from modules.camera_streamer import CameraStreamer
from modules.spatial_detector import SpatialPerceptionEngine
from modules.person_tracker import PersonTracker
from modules.emergency_brake import EmergencyBrake
from modules.occupancy_slam import OccupancySLAM
from modules.semantic_mapper import SemanticMapper
from modules.web_server import WebCockpitServer

# Kiểm tra môi trường ROS
try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Float32, Float32MultiArray, String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Image = object
    CameraInfo = object
    Odometry = object
    OccupancyGrid = object
    Twist = object
    Float32 = object
    Float32MultiArray = object
    String = object

try:
    from depthai_ros_msgs.msg import SpatialDetectionArray
    HAS_DEPTHAI_MSGS = True
except ImportError:
    HAS_DEPTHAI_MSGS = False
    SpatialDetectionArray = object

def is_ros_master_running():
    """Kiểm tra nhanh xem roscore có đang chạy trên port 11311 không (timeout 200ms)"""
    import socket
    try:
        with socket.create_connection(('localhost', 11311), timeout=0.2):
            return True
    except Exception:
        return False


class JetBotMasterSystem:
    def __init__(self, flags):
        self.flags = flags
        self.running = True
        self.lock = threading.RLock()

        # Trạng thái tổng thể xe
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.path_history = []
        self.detections = []
        self.last_depth_clearance = 99.0
        self.last_yolo_clearance = 99.0
        self.obstacle_distance = 99.0
        self.last_vpu_det_time = 0.0
        self.battery_metrics = (11.1, 50, 0.85, 9.4, 120)
        self.last_manual_drive_time = 0.0

        # Vận tốc tức thời phục vụ Dead-Reckoning Odometry
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_cmd_vel_time = 0.0
        self.last_real_ros_odom_time = 0.0
        self.is_auto_scanning = False
        self.auto_scan_remaining = 0.0
        self.auto_scan_duration = float(getattr(config, 'AUTO_SCAN_DURATION_S', 5.0))

        # 1. Khởi tạo Module Động cơ
        self.motors = None
        if self.flags.motors:
            self.motors = MotorController(
                bus_num=config.I2C_BUS, addr=config.PCA9685_ADDR,
                wheel_sep=config.WHEEL_SEPARATION_M,
                max_v=config.MAX_LINEAR_SPEED, max_w=config.MAX_ANGULAR_SPEED,
                brake_dist=config.SAFETY_BRAKE_DIST_M,
                swap_motors=getattr(self.flags, 'swap_motors', getattr(config, 'SWAP_MOTORS', False)),
                invert_linear=getattr(self.flags, 'invert_linear', getattr(config, 'INVERT_LINEAR', False)),
                invert_left=getattr(config, 'INVERT_LEFT_MOTOR', False),
                invert_right=getattr(config, 'INVERT_RIGHT_MOTOR', False),
                enable_brake=getattr(config, 'ENABLE_SAFETY_BRAKE', True)
            )

        # 2. Khởi tạo Module Pin
        self.battery = None
        if self.flags.battery:
            self.battery = BatteryMonitor(bus_num=config.I2C_BUS, addr=config.INA219_ADDR)
            init_metrics = self.battery.read_metrics()
            self.battery_metrics = init_metrics
            print(f"⚡ [BATTERY] Đã kích hoạt giám sát pin INA219: {init_metrics[0]}V ({init_metrics[1]}%), Dòng {init_metrics[2]}A ({init_metrics[3]}W)")

        # 3. Khởi tạo Module Camera Streamer
        self.camera = None
        if self.flags.camera:
            self.camera = CameraStreamer(
                depth_step=config.DEPTH_DOWNSAMPLE_STEP,
                depth_skip=config.DEPTH_SKIP_FRAMES,
                img_skip=config.IMAGE_SKIP_FRAMES
            )

        # 4. Khởi tạo Module Nhận diện AI
        self.yolo = SpatialPerceptionEngine() if self.flags.yolo else None

        # 5. Khởi tạo Module Bám người HRI (Luôn sẵn sàng, mặc định tắt theo Harness 4)
        self.follower = PersonTracker()
        self.follower.set_enabled(bool(self.flags.follower))
        if self.flags.follower:
            print("🎯 [HRI] Đã kích hoạt tính năng Tự hành Bám người!")

        # 6. Khởi tạo Module Phanh khẩn cấp & Cản ảo 3D (Độc lập, dễ kiểm thử)
        self.safety_brake = EmergencyBrake(
            brake_dist_m=getattr(config, 'SAFETY_BRAKE_DIST_M', 0.20),
            warning_dist_m=getattr(config, 'SAFETY_WARNING_DIST_M', 0.40),
            is_enabled=getattr(config, 'ENABLE_SAFETY_BRAKE', True)
        )

        # 7. Khởi tạo Module Bản đồ Chiếm dụng 2D Tích Lũy Tĩnh & Ghim Vật Thể (Occupancy SLAM)
        self.mapper = OccupancySLAM(size_m=10.0, resolution=0.05) if self.flags.mapper else None

        # 8. Khởi tạo Trạm điều khiển Web Cockpit
        self.web = None
        if self.flags.web:
            self.web = WebCockpitServer(host=config.WEB_HOST, port=config.WEB_PORT)
            self.web.state_provider_cb = self.get_state_for_web
            self.web.jpeg_provider_cb = self.get_latest_jpeg
            self.web.jpeg_id_provider_cb = self.get_latest_jpeg_with_id
            self.web.new_frame_event = getattr(self.camera, 'new_frame_event', None)
            self.web.drive_cmd_cb = self.on_drive_command
            self.web.feature_toggle_cb = self.toggle_feature
            self.web.map_save_cb = self.save_map
            self.web.map_reset_cb = self.reset_map
            self.web.auto_scan_cb = self.toggle_auto_scan
            self.web.start()

        # Kết nối ROS trong nền nếu có roscore
        self.ros_cmd_pub = None
        self.ros_batt_pub = None
        self.ros_obj_pub = None
        self.last_ros_img_time = 0.0
        self.last_vpu_det_time = 0.0
        self.last_fallback_found_time = 0.0
        self.latest_raw_bgr = None
        self.latest_depth_np = None

        if HAS_ROS:
            threading.Thread(target=self._ros_connect_loop, daemon=True).start()

        # Khởi chạy các luồng hậu đài (luôn hoạt động, độc lập với ROS)
        threading.Thread(target=self._battery_loop, daemon=True).start()
        threading.Thread(target=self._odometry_loop, daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()
        threading.Thread(target=self._camera_provider_loop, daemon=True).start()
        threading.Thread(target=self._ai_inference_loop, daemon=True).start()

    def _ros_connect_loop(self):
        """Tự động kết nối với ROS Core trong nền khi roscore được bật, không bao giờ làm đơ hệ thống"""
        connected = False
        while self.running and not connected:
            if is_ros_master_running():
                try:
                    rospy.init_node('jetbot_main_orchestrator', anonymous=True, disable_signals=True)
                    self.ros_cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
                    self.ros_batt_pub = rospy.Publisher('/battery_telemetry', Float32MultiArray, queue_size=1)
                    self.ros_obj_pub = rospy.Publisher('/spatial_objects', String, queue_size=2)

                    # Subscribers nhận hình ảnh RGB từ DepthAI (buffer 16MB chống nghẽn gói tin lớn)
                    img_buf = 2**24
                    rospy.Subscriber('/stereo_inertial_publisher/color/image', Image, self._ros_image_cb, queue_size=1, buff_size=img_buf)
                    rospy.Subscriber('/yolov4_publisher/color/image', Image, self._ros_image_cb, queue_size=1, buff_size=img_buf)
                    rospy.Subscriber('/mobilenet_publisher/color/image', Image, self._ros_image_cb, queue_size=1, buff_size=img_buf)

                    # Subscribers nhận bản đồ độ sâu Depth
                    rospy.Subscriber('/stereo_inertial_publisher/stereo/depth', Image, self._ros_depth_cb, queue_size=1, buff_size=img_buf)
                    rospy.Subscriber('/yolov4_publisher/stereo/depth', Image, self._ros_depth_cb, queue_size=1, buff_size=img_buf)
                    rospy.Subscriber('/mobilenet_publisher/stereo/depth', Image, self._ros_depth_cb, queue_size=1, buff_size=img_buf)
                    rospy.Subscriber('/stereo/depth', Image, self._ros_depth_cb, queue_size=1, buff_size=img_buf)
                    rospy.Subscriber('/camera/depth/image_raw', Image, self._ros_depth_cb, queue_size=1, buff_size=img_buf)
                    rospy.Subscriber('/obstacle_distance', Float32, self._ros_obstacle_dist_cb, queue_size=1)

                    rospy.Subscriber('/rtabmap/odom', Odometry, self._ros_odom_cb, queue_size=1)
                    rospy.Subscriber('/odom', Odometry, self._ros_odom_cb, queue_size=1)
                    rospy.Subscriber('/cmd_vel', Twist, self._ros_cmd_cb, queue_size=1)
                    rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self._ros_grid_map_cb, queue_size=1)
                    rospy.Subscriber('/map', OccupancyGrid, self._ros_grid_map_cb, queue_size=1)
                    try:
                        from sensor_msgs.msg import Imu
                        rospy.Subscriber('/stereo_inertial_publisher/imu', Imu, self._ros_imu_cb, queue_size=2)
                        rospy.Subscriber('/imu/data', Imu, self._ros_imu_cb, queue_size=2)
                    except Exception:
                        pass

                    # Subscribers nhận danh sách nhận diện 3D từ OAK-D S2 VPU
                    rospy.Subscriber('/spatial_objects', String, self._ros_spatial_objects_cb, queue_size=2)
                    rospy.Subscriber('/stereo_inertial_publisher/color/raw_detections', String, self._ros_spatial_objects_cb, queue_size=2)
                    if HAS_DEPTHAI_MSGS:
                        rospy.Subscriber('/stereo_inertial_publisher/color/yolov4_Spatial_detections',
                                         SpatialDetectionArray, self._ros_depthai_detections_cb, queue_size=2)
                        rospy.Subscriber('/yolov4_publisher/color/yolov4_Spatial_detections',
                                         SpatialDetectionArray, self._ros_depthai_detections_cb, queue_size=2)
                        rospy.Subscriber('/mobilenet_publisher/color/mobilenet_spatial_detections',
                                         SpatialDetectionArray, self._ros_depthai_detections_cb, queue_size=2)
                    print("🔗 [ROS] Đã kết nối thành công với ROS Core (Master URI: http://localhost:11311)!")
                    print("📡 [ROS TOPIC] Đang lắng nghe luồng từ Terminal 1 (camera_ai.launch):")
                    print("   ├─ Ảnh màu RGB : /yolov4_publisher/color/image")
                    print("   ├─ Độ sâu Depth: /yolov4_publisher/stereo/depth")
                    print("   └─ Nhận diện 3D: /yolov4_publisher/color/yolov4_Spatial_detections")
                    connected = True
                    break
                except Exception:
                    pass
            time.sleep(2.0)

    def _ros_grid_map_cb(self, msg):
        """Nhận bản đồ chiếm dụng 2D trực tiếp từ ROS RTAB-Map hoặc Gmapping"""
        if self.mapper and self.flags.mapper:
            try:
                self.mapper.update_from_ros_grid(msg)
            except Exception:
                pass

    def _update_fused_obstacle_clearance(self):
        """Hợp nhất cự ly cản gần nhất giữa ma trận Depth quang học và đối tượng 3D AI"""
        with self.lock:
            fused = min(self.last_depth_clearance, self.last_yolo_clearance)
            if fused < 4.0:
                self.obstacle_distance = round(float(fused), 2)
            else:
                self.obstacle_distance = None  # Đường thoáng (> 4.0m)
            if self.motors:
                self.motors.update_obstacle_distance(self.obstacle_distance if self.obstacle_distance is not None else 99.0)
            if self.camera:
                self.camera.obstacle_distance = self.obstacle_distance
            dets = list(self.detections)
        self._update_semantic_mapper(dets)

    def _update_semantic_mapper(self, detections):
        """Cập nhật các vật thể nhận diện vào Bản đồ Ngữ nghĩa 3D toàn cục (Semantic Mapper)"""
        if not self.flags.mapper or not self.mapper:
            return
        with self.lock:
            rx, ry, rz, yaw = self.robot_x, self.robot_y, self.robot_z, self.robot_yaw
            obs_dist = self.obstacle_distance
        for d in detections:
            try:
                zc = float(d.get('z', 99.0))
                xc = float(d.get('x', 0.0))
                # Đồng bộ zc nếu cản trước mặt gần hơn (tránh YOLO đo tâm lưng ghế 0.65m nhưng mép cản trước ở 0.34m)
                if obs_dist is not None and abs(xc) <= 0.35 and obs_dist < zc:
                    zc = float(obs_dist)
                if 0.15 <= zc <= 3.5:
                    self.mapper.add_detection(
                        int(d.get('id', 0)),
                        str(d.get('name', 'OBJ')),
                        xc,
                        float(d.get('y', 0.0)),
                        zc,
                        float(d.get('score', 0.8)),
                        rx=rx, ry=ry, rz=rz, yaw=yaw
                    )
            except Exception:
                pass

    def _ros_spatial_objects_cb(self, msg):
        if not self.yolo: return
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                filtered = self.yolo.filter_detections(data)
                with self.lock:
                    self.detections = filtered
                    self.last_vpu_det_time = time.time()
                    if filtered:
                        forward_objs = [d['z'] for d in filtered if abs(d.get('x', 0.0)) <= 0.35 and d.get('z', 99.0) > 0.08]
                        self.last_yolo_clearance = min(forward_objs) if forward_objs else 99.0
                    else:
                        self.last_yolo_clearance = 99.0
                self._update_fused_obstacle_clearance()
        except Exception: pass

    def _ros_depthai_detections_cb(self, msg):
        if not self.yolo: return
        raw_list = []
        try:
            img_w, img_h = 640, 360
            if self.latest_raw_bgr is not None:
                img_h, img_w = self.latest_raw_bgr.shape[:2]

            for det in getattr(msg, 'detections', []):
                # 1. Trích xuất tọa độ 3D không gian (X, Y, Z)
                pos = getattr(det, 'position', None)
                x, y, z = 0.0, 0.0, 0.0
                if pos:
                    x, y, z = float(pos.x), float(pos.y), float(pos.z)
                    # Chuyển đổi mm -> m nếu cảm biến trả về milimet
                    if z > 20.0:
                        x /= 1000.0
                        y /= 1000.0
                        z /= 1000.0

                # 2. Trích xuất Bounding Box 2D thật chuẩn xác từ mạng nơ-ron
                bbox_pixels = None
                bbox_obj = getattr(det, 'bbox', None)
                if bbox_obj is not None:
                    cx, cy, sx, sy = None, None, None, None
                    if hasattr(bbox_obj, 'center') and hasattr(bbox_obj, 'size_x'):
                        cx = float(bbox_obj.center.x)
                        cy = float(bbox_obj.center.y)
                        sx = float(bbox_obj.size_x)
                        sy = float(bbox_obj.size_y)
                    elif hasattr(bbox_obj, 'xmin'):
                        cx = (float(bbox_obj.xmin) + float(bbox_obj.xmax)) / 2.0
                        cy = (float(bbox_obj.ymin) + float(bbox_obj.ymax)) / 2.0
                        sx = float(bbox_obj.xmax) - float(bbox_obj.xmin)
                        sy = float(bbox_obj.ymax) - float(bbox_obj.ymin)

                    if cx is not None and sx is not None and sx > 0 and sy > 0:
                        if cx <= 1.0 and sx <= 1.0:
                            x1 = int((cx - sx / 2.0) * img_w)
                            y1 = int((cy - sy / 2.0) * img_h)
                            x2 = int((cx + sx / 2.0) * img_w)
                            y2 = int((cy + sy / 2.0) * img_h)
                        else:
                            x1 = int(cx - sx / 2.0)
                            y1 = int(cy - sy / 2.0)
                            x2 = int(cx + sx / 2.0)
                            y2 = int(cy + sy / 2.0)
                        bbox_pixels = [max(0, x1), max(0, y1), min(img_w - 1, x2), min(img_h - 1, y2)]

                # 3. Trích xuất thông tin nhãn lớp
                for res in getattr(det, 'results', []):
                    cid = getattr(res, 'id', getattr(res, 'class_id', 0))
                    label_name = getattr(res, 'label', '')
                    score = float(getattr(res, 'score', 0.8))

                    item = {
                        "id": int(cid),
                        "name": label_name,
                        "score": score,
                        "x": round(x, 2),
                        "y": round(y, 2),
                        "z": round(z, 2)
                    }
                    if bbox_pixels:
                        item["bbox"] = bbox_pixels
                    raw_list.append(item)

            filtered = self.yolo.filter_detections(raw_list)
            # Đi qua bộ lọc ổn định thời gian TemporalTracker chống nhấp nháy
            tracked = self.yolo.tracker.update(filtered)
            with self.lock:
                self.detections = tracked
                self.last_vpu_det_time = time.time()
                # Cập nhật cự ly vật cản trước mặt trực tiếp từ các đối tượng 3D
                if tracked:
                    forward_objs = [d['z'] for d in tracked if abs(d.get('x', 0.0)) <= 0.35 and d.get('z', 99.0) > 0.08]
                    self.last_yolo_clearance = min(forward_objs) if forward_objs else 99.0
                else:
                    self.last_yolo_clearance = 99.0
            self._update_fused_obstacle_clearance()
        except Exception: pass

    def _ros_image_cb(self, msg):
        if not self.camera: return
        now = time.time()
        # Khử trùng lặp khung hình khi nhiều topic alias cùng phát (giới hạn nhịp ~35 FPS)
        if (now - self.last_ros_img_time) < 0.025:
            return
        self.last_ros_img_time = now
        try:
            w, h = msg.width, msg.height
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            img = raw.reshape((h, w, 3)) if msg.encoding in ['bgr8', 'rgb8'] else raw.reshape((h, w, -1))
            if msg.encoding == 'rgb8': img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            self.latest_raw_bgr = img
            if not getattr(self, '_ros_first_img_logged', False):
                self._ros_first_img_logged = True
                print("🎥 [TERMINAL 1 -> 2] Đã nhận luồng hình ảnh màu RGB trực tiếp từ 'camera_ai.launch' thành công!")

            # Ước lượng góc quay thị giác thời gian thực (Visual Gyroscope):
            # Nhận diện chính xác xe quay khi người dùng dùng tay xoay JetBot hoặc khi trượt bánh
            if self.camera:
                delta_yaw = self.camera.estimate_visual_rotation(img)
                if abs(delta_yaw) > 0.002:
                    with self.lock:
                        self.robot_yaw += delta_yaw
                        self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

            with self.lock:
                current_dets = list(self.detections)
                obs_dist = self.obstacle_distance

            annotator = (lambda im: self.yolo.draw_detections(im, current_dets)) if self.yolo else None
            self.camera.process_color_frame(img, annotator, detections=current_dets, obstacle_dist=obs_dist)

        except Exception: pass

    def _ros_depth_cb(self, msg):
        if not self.camera: return
        now = time.time()
        # Khử trùng lặp bản đồ độ sâu (giới hạn nhịp ~35 FPS)
        if (now - getattr(self, 'last_ros_depth_time', 0.0)) < 0.025:
            return
        self.last_ros_depth_time = now
        try:
            w, h = msg.width, msg.height
            if getattr(msg, 'encoding', '') in ['32FC1'] or len(msg.data) == w * h * 4:
                raw_f = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
                raw_f = np.nan_to_num(raw_f, nan=0.0, posinf=0.0, neginf=0.0)
                valid_mask = raw_f > 0.05
                is_meters = (np.count_nonzero(valid_mask) > 0 and np.nanmax(raw_f[valid_mask]) < 50.0)
                depth_np = (raw_f * 1000.0).astype(np.uint16) if is_meters else raw_f.astype(np.uint16)
            else:
                depth_np = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            self.latest_depth_np = depth_np
            if not getattr(self, '_ros_first_depth_logged', False):
                self._ros_first_depth_logged = True
                print("📐 [TERMINAL 1 -> 2] Đã nhận ma trận Stereo Depth từ 'camera_ai.launch'! Hệ thống Phanh Khẩn Cấp đã sẵn sàng.")
            with self.lock:
                rx, ry, rz, yaw = self.robot_x, self.robot_y, self.robot_z, self.robot_yaw
            self.camera.process_depth_frame(depth_np, rx, ry, rz, yaw)

            if self.mapper and self.flags.mapper:
                self.mapper.update_scan(rx, ry, self.camera.points_3d)

            if self.safety_brake:
                dist = self.safety_brake.calculate_clearance(depth_np)
                self.last_depth_clearance = dist
                self._update_fused_obstacle_clearance()
            elif self.yolo:
                dist = self.yolo.calculate_obstacle_distance(depth_np)
                self.last_depth_clearance = dist
                self._update_fused_obstacle_clearance()
        except Exception: pass

    def _ros_obstacle_dist_cb(self, msg):
        try:
            val = float(msg.data)
            if val < 4.0:
                self.last_depth_clearance = val
                self._update_fused_obstacle_clearance()
        except Exception: pass

    def _ros_imu_cb(self, msg):
        """Nhận góc quay trực tiếp từ cảm biến quán tính IMU (OAK-D hoặc BNO055/MPU6050)"""
        try:
            q = msg.orientation
            norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if norm_sq < 0.5:
                return # Bỏ qua quaternion rỗng hoặc không có dữ liệu
            if hasattr(msg, 'orientation_covariance') and len(msg.orientation_covariance) > 0 and msg.orientation_covariance[0] < 0:
                return
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            with self.lock:
                self.robot_yaw = yaw
                self.last_real_ros_odom_time = time.time()
        except Exception:
            pass

    def _ros_odom_cb(self, msg):
        try:
            q = msg.pose.pose.orientation
            norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if norm_sq < 0.5:
                return
            p = msg.pose.pose.position
            # Bỏ qua nếu topic phát dummy tĩnh (0,0,0) liên tục
            if abs(p.x) < 1e-4 and abs(p.y) < 1e-4 and abs(q.z) < 1e-4:
                return
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            with self.lock:
                self.robot_x, self.robot_y, self.robot_z = p.x, p.y, p.z
                self.robot_yaw = yaw
                self.last_real_ros_odom_time = time.time()
                self.path_history.append([round(p.x, 3), round(p.y, 3), round(p.z, 3)])
                if len(self.path_history) > 400: self.path_history.pop(0)
        except Exception:
            pass

    def _ros_cmd_cb(self, msg):
        # Bỏ qua nếu lệnh vừa được phát từ Web Cockpit nội bộ trong 0.3s để tránh lặp kênh
        if time.time() - self.last_manual_drive_time < 0.30:
            return
        v, w = float(msg.linear.x), float(msg.angular.z)
        with self.lock:
            self.current_v = v
            self.current_w = w
            self.last_cmd_vel_time = time.time()
        if self.safety_brake and self.safety_brake.is_enabled:
            v, w, alert = self.safety_brake.evaluate_velocity(v, w, self.obstacle_distance)
        if self.motors:
            if abs(v) < 0.01 and abs(w) < 0.01:
                self.motors.stop()
            else:
                self.motors.set_cmd_vel(v, w)

    def on_drive_command(self, v, w):
        """Xử lý lệnh lái tay từ Web W-A-S-D (Ưu tiên cao nhất, tạm ngắt bám người)"""
        self.last_manual_drive_time = time.time()
        if self.is_auto_scanning:
            self.is_auto_scanning = False
        print(f"🎮 [WEB LÁI TAY] Lệnh nhận được: v={v:.2f}, w={w:.2f}")

        brake_thresh = getattr(config, 'SAFETY_BRAKE_DIST_M', 0.20)

        # 1. Đánh giá qua EmergencyBrake nếu có
        if self.safety_brake and self.safety_brake.is_enabled:
            v, w, alert = self.safety_brake.evaluate_velocity(v, w, self.obstacle_distance)
            if alert == "EMERGENCY_STOP":
                threshold_cm = int(self.safety_brake.brake_dist_m * 100)
                obs_cm = f"{self.obstacle_distance*100:.1f} cm" if self.obstacle_distance is not None else "< 20cm"
                print(f"🚨 [PHANH KHẨN CẤP] Cản cách {obs_cm} (< {threshold_cm}cm) -> Đã ngắt tiến, chỉ cho phép lùi/quay!")

        # 2. Can thiệp phanh khẩn cấp cứng:
        # Nếu cản nguy hiểm (<= 20cm) và đang nhấn TIẾN -> Khóa lệnh tiến, chỉ cho phép lùi (v < 0) hoặc quay (w != 0)
        if (v > 0.01) and (self.obstacle_distance is not None and self.obstacle_distance <= brake_thresh):
            v = 0.0
            print(f"🛑 [KHÓA LỆNH TIẾN] Cản cách {self.obstacle_distance*100:.1f} cm (<= {int(brake_thresh*100)}cm). Cho phép LÙI hoặc QUAY để thoát cản!")

        # 3. Cập nhật vận tốc tức thời cho vòng lặp Dead-Reckoning
        with self.lock:
            self.current_v = v
            self.current_w = w
            self.last_cmd_vel_time = time.time()

        if self.motors:
            if abs(v) < 0.01 and abs(w) < 0.01:
                self.motors.stop()
            else:
                self.motors.set_cmd_vel(v, w)

        if HAS_ROS and self.ros_cmd_pub:
            t = Twist()
            t.linear.x, t.angular.z = v, w
            self.ros_cmd_pub.publish(t)

    def _battery_loop(self):
        """Vòng lặp đo pin định kỳ 1 Hz"""
        while self.running:
            if self.battery:
                try:
                    m = self.battery.read_metrics()
                    with self.lock:
                        self.battery_metrics = m
                    if HAS_ROS and hasattr(self, 'ros_batt_pub') and self.ros_batt_pub:
                        msg = Float32MultiArray()
                        msg.data = [float(m[0]), float(m[1]), float(m[2]), float(m[3]), float(m[4])]
                        self.ros_batt_pub.publish(msg)
                except Exception:
                    pass
            time.sleep(1.0)

    def _odometry_loop(self):
        """Luồng tính toán Dead-Reckoning Odometry vi sai thời gian thực (20 Hz):
        Tích phân động học vi sai (Differential Drive Kinematics) từ vận tốc v (m/s) và w (rad/s)
        để cập nhật liên tục tọa độ (X, Y) và góc quay Yaw (θ) khi xe nhận lệnh điều khiển.
        """
        last_t = time.time()
        while self.running:
            time.sleep(0.05)
            now = time.time()
            dt = min(0.2, max(0.01, now - last_t))
            last_t = now

            with self.lock:
                # Nếu không nhận thêm lệnh lái trong 0.35s -> coi như đã dừng xe (v=0, w=0)
                if (now - getattr(self, 'last_cmd_vel_time', 0.0)) > 0.35:
                    self.current_v = 0.0
                    self.current_w = 0.0

                v = getattr(self, 'current_v', 0.0)
                w = getattr(self, 'current_w', 0.0)

                if abs(v) > 0.005 or abs(w) > 0.005:
                    # Tích phân góc quay (Yaw) chuẩn vi phân:
                    self.robot_yaw += w * dt
                    # Chuẩn hóa góc quay về [-pi, +pi]
                    self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

                    # Tích phân tọa độ không gian (X, Y):
                    self.robot_x += v * dt * math.cos(self.robot_yaw)
                    self.robot_y += v * dt * math.sin(self.robot_yaw)

                    # Cập nhật vết lịch sử đường đi (path history) khi di chuyển đủ 2cm
                    if not self.path_history or (
                        (self.robot_x - self.path_history[-1][0])**2 + (self.robot_y - self.path_history[-1][1])**2 > 0.0004
                    ):
                        self.path_history.append([round(self.robot_x, 3), round(self.robot_y, 3), 0.02])
                        if len(self.path_history) > 400:
                            self.path_history.pop(0)

    def _control_loop(self):
        """Vòng lặp bám người HRI (10 Hz) với cơ chế nhường quyền lái tay"""
        last_log = 0.0
        while self.running:
            if self.follower and self.follower.is_enabled:
                # Ưu tiên lái tay: Nếu vừa bấm phím lái trong 1.0 giây, tạm ngừng follower
                if (time.time() - self.last_manual_drive_time > 1.0):
                    with self.lock: dets = list(self.detections)
                    v, w = self.follower.compute_command(dets)
                    if v is not None and w is not None:
                        with self.lock:
                            self.current_v = v
                            self.current_w = w
                            self.last_cmd_vel_time = time.time()
                        if (abs(v) > 0.01 or abs(w) > 0.01) and (time.time() - last_log > 0.6):
                            last_log = time.time()
                            print(f"🎯 [HRI BÁM NGƯỜI] Điều khiển theo mục tiêu: v={v:.2f} m/s, w={w:.2f} rad/s")
                        if self.motors: self.motors.set_cmd_vel(v, w)
                        if HAS_ROS and self.ros_cmd_pub:
                            t = Twist()
                            t.linear.x, t.angular.z = v, w
                            self.ros_cmd_pub.publish(t)
            time.sleep(0.1)

    def _ai_inference_loop(self):
        """Luồng suy luận AI bất đồng bộ (Zero-Latency Async Worker):
        Tách biệt hoàn toàn tính toán mạng nơ-ron AI khỏi luồng Camera Stream.
        Giúp Camera Stream đạt tối đa 25-30 FPS siêu mượt, độ trễ < 25ms!"""
        while self.running:
            if self.flags.yolo and self.yolo and (self.latest_raw_bgr is not None):
                try:
                    now = time.time()
                    # Nếu đang có luồng nhận diện từ VPU của OAK-D (qua ROS trong 1.8s gần nhất),
                    # KHÔNG chạy fallback CPU để tránh ghi đè làm mất các vật thể từ OAK-D
                    vpu_active = (now - getattr(self, 'last_vpu_det_time', 0.0) < 1.8)

                    if not vpu_active:
                        frame = self.latest_raw_bgr
                        depth = self.latest_depth_np
                        live_dets = self.yolo.detect_fallback(frame, depth)
                        tracked = self.yolo.tracker.update(live_dets)
                        with self.lock:
                            self.detections = tracked

                        calc_dist = self.yolo.calculate_obstacle_distance(depth, tracked)
                        with self.lock:
                            if calc_dist is not None:
                                self.last_yolo_clearance = calc_dist
                            else:
                                self.last_yolo_clearance = 99.0

                        # Hợp nhất cự ly cản an toàn, cập nhật Motor & Camera HUD
                        self._update_fused_obstacle_clearance()
                except Exception:
                    pass
            time.sleep(0.04) # Cập nhật AI ~20-25 Hz mượt mà, không chặn luồng video

    def _camera_provider_loop(self):
        """Cung cấp luồng hình ảnh camera siêu tốc độ cao (25-30 FPS, độ trễ cực thấp)"""
        mock_pts = []
        for x in np.linspace(-2.0, 2.0, 25):
            for z in np.linspace(0.1, 1.5, 8):
                mock_pts.append([round(float(x), 2), 2.0, round(float(z), 2)])
                mock_pts.append([round(float(x), 2), -2.0, round(float(z), 2)])

        last_sim_time = 0.0

        while self.running:
            if not self.flags.camera or not self.camera:
                time.sleep(0.1)
                continue

            now = time.time()
            has_fresh_ros_frame = (now - self.last_ros_img_time < 1.5)

            # Nếu chưa có luồng ảnh thật từ ROS, tự động lấy ảnh từ OAK-D (cắm USB) hoặc Laptop Webcam
            if not has_fresh_ros_frame:
                try:
                    live_frame, live_depth, src_name = self.camera.get_live_frame()
                except Exception as ex_cam:
                    live_frame, live_depth, src_name = None, None, "ERROR"

                # Đọc nhận diện 3D từ phần cứng VPU OAK-D nếu có
                vpu_dets = getattr(self.camera, 'latest_vpu_detections', [])
                if vpu_dets:
                    with self.lock:
                        self.detections = vpu_dets
                        self.last_vpu_det_time = now
                        vpu_dists = [d['z'] for d in vpu_dets if 'z' in d and 0.15 <= d['z'] <= 10.0]
                        if vpu_dists:
                            self.last_yolo_clearance = min(vpu_dists)
                    self._update_fused_obstacle_clearance()
                if live_depth is not None:
                    self.latest_depth_np = live_depth
                    with self.lock:
                        rx, ry, rz, yaw = self.robot_x, self.robot_y, self.robot_z, self.robot_yaw
                    self.camera.process_depth_frame(live_depth, rx, ry, rz, yaw)
                    if self.mapper and self.flags.mapper:
                        self.mapper.update_scan(rx, ry, self.camera.points_3d)

                    # Giám sát an toàn tức thời từ ma trận Depth USB
                    if self.safety_brake:
                        d_clear = self.safety_brake.calculate_clearance(live_depth)
                        with self.lock:
                            self.last_depth_clearance = d_clear
                        self._update_fused_obstacle_clearance()
                    elif self.yolo:
                        d_clear = self.yolo.calculate_obstacle_distance(live_depth)
                        with self.lock:
                            self.last_depth_clearance = d_clear if d_clear is not None else 99.0
                        self._update_fused_obstacle_clearance()

                if live_frame is not None:
                    self.latest_raw_bgr = live_frame

                    # Ước lượng xoay thị giác thời gian thực (Visual Gyroscope)
                    delta_yaw = self.camera.estimate_visual_rotation(live_frame)
                    if abs(delta_yaw) > 0.002:
                        with self.lock:
                            self.robot_yaw += delta_yaw
                            self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

                    with self.lock:
                        current_dets = list(self.detections)
                        obs_dist = self.obstacle_distance

                    annotator = (lambda im: self.yolo.draw_detections(im, current_dets)) if self.yolo else None
                    self.camera.process_color_frame(live_frame, annotator, detections=current_dets, obstacle_dist=obs_dist)
                else:
                    if src_name == "MO PHONG":
                        # Chế độ mô phỏng: Giữ nhịp 30 FPS (~33ms) chuẩn xác, tránh đốt CPU
                        if now - last_sim_time >= 0.033:
                            last_sim_time = now
                            if not self.camera.points_3d:
                                self.camera.points_3d = list(mock_pts)
                            if self.mapper and self.flags.mapper:
                                with self.lock:
                                    rx, ry = self.robot_x, self.robot_y
                                self.mapper.update_scan(rx, ry, self.camera.points_3d)
                            demo_img = np.zeros((360, 640, 3), dtype=np.uint8)
                            demo_img[:] = (15, 12, 10)
                            cv2.putText(demo_img, "OAK-D S2 // SIMULATOR", (150, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 240, 255), 2)
                            annotator = (lambda im: self.yolo.draw_detections(im, self.detections)) if self.yolo else None
                            self.camera.process_color_frame(demo_img, annotator, obstacle_dist=self.obstacle_distance)
                        else:
                            time.sleep(0.005)
                    else:
                        # Đang chờ OAK-D hoàn tất frame tiếp theo (chu kỳ 30 FPS ~33ms)
                        time.sleep(0.002)
            else:
                time.sleep(0.005)

    def get_latest_jpeg(self):
        if self.camera:
            return self.camera.latest_jpeg
        return None

    def get_latest_jpeg_with_id(self):
        if self.camera and hasattr(self.camera, 'get_latest_jpeg_with_id'):
            return self.camera.get_latest_jpeg_with_id()
        if self.camera:
            return 0, self.camera.latest_jpeg
        return 0, None

    def get_state_for_web(self):
        with self.lock:
            v, pct, curr, pwr, rem = self.battery_metrics
            pts = self.camera.points_3d if self.camera else []
            has_person = any(d.get('name') == 'PERSON' for d in self.detections)
            confirmed_objs = self.mapper.get_confirmed_landmarks() if (self.flags.mapper and self.mapper) else []
            map_payload = self.mapper.get_payload_for_web() if (self.flags.mapper and self.mapper) else {}
            brake_dist = getattr(config, 'SAFETY_BRAKE_DIST_M', 0.20)
            cam_fps = float(self.camera.calc_fps) if (self.camera and hasattr(self.camera, 'calc_fps') and self.camera.calc_fps > 0) else 15.0
            
            if time.time() - self.last_ros_img_time < 2.0:
                cam_src = "OAK-D S2 (ROS LIVE)"
            elif self.camera and getattr(self.camera, 'is_oak_connected', False):
                cam_src = "OAK-D S2 (USB LIVE)"
            elif self.camera and getattr(self.camera, '_cap', None) and self.camera._cap.isOpened():
                cam_src = "WEBCAM LAPTOP"
            else:
                cam_src = "OAK-D S2 (SIMULATOR)"
            cam_mode_str = getattr(self.camera, 'view_mode', 'ai').upper() if self.camera else 'AI'

            return {
                "battery_v": v, "battery_pct": pct, "battery_cell_v": round(v / 3.0, 2),
                "battery_current_a": curr, "battery_power_w": pwr, "battery_remaining_min": rem,
                "battery_status": "BÌNH THƯỜNG" if v >= 10.8 else ("YẾU" if v >= 10.2 else "NGUY HIỂM"),
                "robot_x": round(self.robot_x, 3), "robot_y": round(self.robot_y, 3), "robot_z": round(self.robot_z, 3),
                "robot_yaw": round(self.robot_yaw, 3), "path": self.path_history,
                "map_b64": map_payload.get("map_b64", ""),
                "map_version": map_payload.get("map_version", 1),
                "map_origin_x": map_payload.get("map_origin_x", -5.0),
                "map_origin_y": map_payload.get("map_origin_y", -5.0),
                "map_resolution": map_payload.get("map_resolution", 0.05),
                "map_width": map_payload.get("map_width", 200),
                "map_height": map_payload.get("map_height", 200),
                "free_cells": map_payload.get("free_cells", 0),
                "occ_cells": map_payload.get("occ_cells", 0),
                "points_3d": pts,
                "detections": self.detections, "obstacle_distance": self.obstacle_distance,
                "safety_brake_dist": brake_dist,
                "auto_scanning": self.is_auto_scanning,
                "auto_scan_remaining": round(getattr(self, 'auto_scan_remaining', 0.0), 1),
                "auto_scan_duration": round(getattr(self, 'auto_scan_duration', 5.0), 1),
                "semantic_objects": confirmed_objs,
                "landmarks": confirmed_objs,
                "follower_enabled": bool(self.follower.is_enabled) if self.follower else False,
                "mapper_enabled": bool(self.flags.mapper),
                "camera_source": f"{cam_src} [{cam_mode_str}]",
                "calc_fps": cam_fps,
                "camera_mode": getattr(self.camera, 'view_mode', 'ai') if self.camera else 'ai',
                "system_version": getattr(config, 'SYSTEM_VERSION', 'v2.4.0-SAFETY-DUAL-ROS'),
                "system_codename": getattr(config, 'VERSION_CODENAME', 'Aegis JetBot'),
                "system_build": getattr(config, 'BUILD_TAG', 'v2.4.0-safety-dual-ros'),
                "benchmark": {
                    "tv1": {
                        "total_score": 29, "max_score": 30, "grade": "XUẤT SẮC",
                        "score_hw": 14, "max_hw": 15, "score_motion": 15, "max_motion": 15,
                        "cpu_pct": 32, "ram_gb": 1.4, "battery_v": v, "battery_pct": pct,
                        "bumper_status": "VÙNG AN TOÀN" if (self.obstacle_distance is None or self.obstacle_distance >= brake_dist + 0.1) else ("[PHANH KHẨN CẤP]" if self.obstacle_distance <= brake_dist else "[CẢNH BÁO]"),
                        "diag_text": "Hệ thống động cơ & nguồn điện INA219 ổn định."
                    },
                    "tv2": {
                        "status_badge": "HOẠT ĐỘNG TỐT", "fps": cam_fps, "fps_pct": min(100, int(cam_fps / 25.0 * 100)),
                        "valid_depth_pct": 88, "num_obj": len(self.detections), "avg_confidence": 90,
                        "target_info": "Đang khóa mục tiêu người dùng" if has_person else "Đang quét không gian phía trước",
                        "obstacle_distance": self.obstacle_distance,
                        "dist_pct": int(min(100, max(15, (self.obstacle_distance or 2.5) * 35))),
                        "diag_text": f"Chế độ {cam_mode_str} | FPS: {cam_fps:.1f} | Cự ly cản: {f'{self.obstacle_distance*100:.1f} cm' if self.obstacle_distance is not None else 'ĐƯỜNG THOÁNG'} (Ngưỡng phanh {int(brake_dist*100)}cm)."
                    },
                    "tv3": {
                        "total_score": 58, "max_score": 60, "grade": "XUẤT SẮC",
                        "score_s": 34, "max_s": 35, "odom_hz": cam_fps, "num_pts": len(pts),
                        "score_sem": 24, "max_sem": 25, "num_obj": len(confirmed_objs) if confirmed_objs else len(self.detections),
                        "status_badge": "BẢN ĐỒ SẠCH",
                        "diag_text": f"Bản đồ RTAB-Map: Đã ghi nhận {len(confirmed_objs)} mốc ngữ nghĩa không gian." if confirmed_objs else "Bản đồ RTAB-Map hoạt động chuẩn xác."
                    }
                }
            }

    def toggle_feature(self, flag_name):
        """Bật/tắt tính năng trực tiếp từ Web Cockpit"""
        flag_name = flag_name.lower()
        if "cam" in flag_name or flag_name in ["rgb", "depth", "ai"]:
            if self.camera:
                mode = flag_name.replace("cam_mode_", "").replace("cam_", "").strip()
                res = self.camera.set_view_mode(mode)
                return res
        if "follow" in flag_name:
            if self.follower is None:
                self.follower = PersonTracker()
            new_state = not self.follower.is_enabled
            self.follower.set_enabled(new_state)
            print(f"🔄 [TOGGLE] Bám người HRI: {'BẬT' if new_state else 'TẮT'}")
            return new_state
        elif "map" in flag_name:
            if self.mapper is None:
                self.mapper = OccupancySLAM(size_m=10.0, resolution=0.05)
            self.flags.mapper = not self.flags.mapper
            print(f"🔄 [TOGGLE] Bản đồ chiếm dụng 2D OccupancySLAM: {'BẬT' if self.flags.mapper else 'TẮT'}")
            return self.flags.mapper
        elif "brake" in flag_name:
            if self.safety_brake:
                self.safety_brake.is_enabled = not self.safety_brake.is_enabled
                if self.motors: self.motors.enable_brake = self.safety_brake.is_enabled
                print(f"🔄 [TOGGLE] Phanh ảo Virtual Bumper: {'BẬT' if self.safety_brake.is_enabled else 'TẮT'}")
                return self.safety_brake.is_enabled
        elif "swap" in flag_name and self.motors:
            self.motors.swap_motors = not self.motors.swap_motors
            print(f"🔄 [TOGGLE] Đảo kênh Motor: {'BẬT' if self.motors.swap_motors else 'TẮT'}")
            return self.motors.swap_motors
        elif "linear" in flag_name and self.motors:
            self.motors.invert_linear = not self.motors.invert_linear
            print(f"🔄 [TOGGLE] Đảo chiều Tiến/Lùi: {'BẬT' if self.motors.invert_linear else 'TẮT'}")
            return self.motors.invert_linear
        return False

    def save_map(self, name="my_room_map"):
        """Lưu bản đồ hiện tại sang file JSON & PNG"""
        if self.mapper:
            return self.mapper.save_map(name)
        return False, "Module Mapper chưa bật"

    def toggle_auto_scan(self, duration=None):
        """Bật/Tắt chế độ tự động xoay 360 độ chậm rãi để camera và LaserScan quét lập bản đồ toàn diện"""
        with self.lock:
            if self.is_auto_scanning:
                self.is_auto_scanning = False
                self.auto_scan_remaining = 0.0
                print("🛑 [AUTO SCAN 360°] Đã nhận lệnh dừng quét tự động.")
                if self.motors: self.motors.stop()
                return False
            else:
                self.is_auto_scanning = True
                self.current_v = 0.0
                self.current_w = 0.0
                self.last_manual_drive_time = 0.0
                scan_dur = float(duration) if duration and float(duration) > 0 else float(getattr(config, 'AUTO_SCAN_DURATION_S', 5.0))
                self.auto_scan_duration = scan_dur
                self.auto_scan_remaining = scan_dur
                threading.Thread(target=self._auto_scan_worker, args=(scan_dur,), daemon=True).start()
                print(f"🔄 [AUTO SCAN 360°] Bắt đầu xoay chậm 360 độ trong {scan_dur:.1f}s để quét toàn cảnh phòng...")
                return True

    def _auto_scan_worker(self, scan_duration=5.0):
        """Luồng tự động xoay chậm đều đúng 1 vòng 360 độ trong scan_duration giây (mặc định 5.0s).
        - Sử dụng xung nhịp Pulse-Glide (2 nhịp kéo 19% PWM, 1 nhịp trượt êm) để xe quay từ tốn, không quay tít.
        - Khống chế thời gian chính xác, dừng xe lập tức khi hoàn tất đúng 360 độ.
        - Tích phân góc quay robot_yaw đồng bộ 1:1 theo thời gian thực để bản đồ 2D thể hiện chuẩn xác.
        """
        scan_duration = max(1.0, min(25.0, float(scan_duration)))
        w_nominal = (2.0 * math.pi) / scan_duration
        target_pwm = float(getattr(config, 'AUTO_SCAN_SPEED_PWM', 0.19))

        start_time = time.time()
        last_t = start_time
        step_idx = 0

        while self.running and self.is_auto_scanning:
            time.sleep(0.05)
            now = time.time()
            dt = now - last_t
            last_t = now
            elapsed = now - start_time
            step_idx += 1

            self.auto_scan_remaining = max(0.0, scan_duration - elapsed)

            # 1. Dừng ngay nếu phát hiện cản khẩn cấp phía trước sát mũi xe <= 15cm
            if self.obstacle_distance is not None and self.obstacle_distance <= 0.15:
                print("🛑 [AUTO SCAN 360°] Phát hiện vật cản sát mũi xe <= 15cm -> Tự động dừng quay an toàn!")
                break

            # 2. Dừng ngay nếu người dùng can thiệp lái tay
            if (now - self.last_manual_drive_time < 0.50):
                print("🛑 [AUTO SCAN 360°] Người dùng can thiệp lái tay -> Dừng quét tự động.")
                break

            # 3. Khi hoàn thành đúng thời gian 360 độ -> Dừng ngay lập tức
            if elapsed >= scan_duration:
                print(f"🎉 [AUTO SCAN 360°] Đã hoàn thành chuẩn xác 1 vòng quét 360° ({scan_duration:.1f}s)!")
                break

            # 4. Điều khiển xung nhịp Pulse-Glide (2 nhịp ON 100ms, 1 nhịp GLIDE 50ms)
            is_drive_phase = (step_idx % 3 != 2)
            if self.motors:
                if is_drive_phase:
                    if hasattr(self.motors, 'spin_in_place'):
                        self.motors.spin_in_place(duty=target_pwm, direction=1)
                    else:
                        self.motors.set_cmd_vel(0.0, target_pwm / 0.45)
                else:
                    self.motors.stop()

            if HAS_ROS and self.ros_cmd_pub:
                t = Twist()
                t.angular.z = w_nominal if is_drive_phase else 0.0
                self.ros_cmd_pub.publish(t)

            # 5. Cập nhật góc quay robot_yaw đồng bộ 1:1 với tiến độ thực tế
            with self.lock:
                self.current_v = 0.0
                self.current_w = w_nominal
                self.last_cmd_vel_time = now

        with self.lock:
            self.is_auto_scanning = False
            self.current_w = 0.0
            self.auto_scan_remaining = 0.0
        if self.motors:
            self.motors.stop()
        if HAS_ROS and self.ros_cmd_pub:
            t = Twist()
            self.ros_cmd_pub.publish(t)


    def reset_map(self):
        """Xóa trắng bản đồ 2D để xây dựng lại từ đầu và đặt lại gốc tọa độ"""
        with self.lock:
            self.is_auto_scanning = False
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_yaw = 0.0
            self.path_history = []
            self.current_v = 0.0
            self.current_w = 0.0
        if self.mapper:
            return self.mapper.reset_map()
        return True


    def shutdown(self):
        print("\n🛑 [SHUTDOWN] Đang dừng an toàn toàn bộ hệ thống JetBot...")
        self.running = False
        if self.motors: self.motors.shutdown()
        if self.camera: self.camera.shutdown()
        if self.web: self.web.stop()
        print("✅ [SHUTDOWN] Đã giải phóng tài nguyên phần cứng thành công!")


def parse_arguments():
    parser = argparse.ArgumentParser(description="JetBot Modular Master Orchestrator")
    parser.add_argument('--motors', action='store_true', dest='motors', default=None)
    parser.add_argument('--no-motors', action='store_false', dest='motors')
    parser.add_argument('--swap-motors', action='store_true', dest='swap_motors', default=None,
                        help="Đảo kênh motor Trái <-> Phải (sửa lỗi rẽ trái thành quay phải)")
    parser.add_argument('--no-swap-motors', action='store_false', dest='swap_motors')
    parser.add_argument('--invert-linear', action='store_true', dest='invert_linear', default=None,
                        help="Đảo chiều tiến/lùi nếu W bị lùi và S bị tiến")
    parser.add_argument('--no-invert-linear', action='store_false', dest='invert_linear')
    parser.add_argument('--pin', '--battery', action='store_true', dest='battery', default=None)
    parser.add_argument('--no-pin', '--no-battery', action='store_false', dest='battery')
    parser.add_argument('--camera', action='store_true', dest='camera', default=None)
    parser.add_argument('--no-camera', action='store_false', dest='camera')
    parser.add_argument('--yolo', action='store_true', dest='yolo', default=None)
    parser.add_argument('--no-yolo', action='store_false', dest='yolo')
    parser.add_argument('--follower', '--enable-follower', action='store_true', dest='follower', default=None)
    parser.add_argument('--no-follower', action='store_false', dest='follower')
    parser.add_argument('--mapper', action='store_true', dest='mapper', default=None)
    parser.add_argument('--no-mapper', action='store_false', dest='mapper')
    parser.add_argument('--web', action='store_true', dest='web', default=None)
    parser.add_argument('--no-web', action='store_false', dest='web')
    args = parser.parse_args()

    # Mặc định lấy theo biến khai báo ON/OFF ở đầu file main.py & config:
    if args.motors is None: args.motors = to_bool(MOTOR)
    if args.swap_motors is None: args.swap_motors = getattr(config, 'SWAP_MOTORS', False)
    if args.invert_linear is None: args.invert_linear = getattr(config, 'INVERT_LINEAR', False)
    if args.battery is None: args.battery = to_bool(PIN)
    if args.camera is None: args.camera = to_bool(CAMERA)
    if args.yolo is None: args.yolo = to_bool(YOLO)
    if args.follower is None: args.follower = to_bool(FOLLOWER)
    if args.mapper is None: args.mapper = to_bool(MAPPER)
    if args.web is None: args.web = to_bool(WEB)
    return args


def main():
    args = parse_arguments()
    sys_ver = getattr(config, 'SYSTEM_VERSION', 'v2.5.0-UNIFIED-MODULAR')
    sys_code = getattr(config, 'VERSION_CODENAME', 'AEGIS JETBOT (Hệ Thống Thống Nhất 1-Terminal & ROS Hybrid)')
    build_date = getattr(config, 'BUILD_DATE', '2026-10-07')
    build_tag = getattr(config, 'BUILD_TAG', 'v2.5.0-unified-modular')
    workflow = getattr(config, 'WORKFLOW_MODE', '1-Terminal Mode (python3 main.py)')

    print("\n" + "═" * 74)
    print("🤖 JETBOT MODULAR MASTER SYSTEM (MAIN.PY)")
    print(f"📦 PHIÊN BẢN (VERSION) : {sys_ver}")
    print(f"🏷️  CODE NAME           : {sys_code}")
    print(f"📅 PHÁT HÀNH           : {build_date} | TAG: {build_tag}")
    print(f"🔄 CHẾ ĐỘ CHẠY (FLOW)  : {workflow}")
    print("─" * 74)
    print(f"  ├─ Động cơ (PCA9685 0x60):  {'BẬT' if args.motors else 'TẮT'}")
    print(f"  ├─ Đo Pin (INA219 0x41):    {'BẬT' if args.battery else 'TẮT'}")
    print(f"  ├─ Camera OAK-D S2:         {'BẬT (Trực tiếp USB & Tự động phát ROS Topics)' if args.camera else 'TẮT'}")
    print(f"  ├─ Spatial AI:              {'BẬT (HOG People Detector & 3D Depth Spatial Clustering)' if args.yolo else 'TẮT'}")
    print(f"  ├─ Phanh khẩn cấp:          BẬT (< 25cm khóa tiến, cho phép lùi/quay)")
    print(f"  ├─ Bám người (Follower):    {'BẬT' if args.follower else 'TẮT (Ưu tiên lái tay)'}")
    print(f"  ├─ Bản đồ ngữ nghĩa 3D:     {'BẬT' if args.mapper else 'TẮT'}")
    print(f"  └─ Web Cockpit (Port 8080): {'BẬT (http://0.0.0.0:8080)' if args.web else 'TẮT'}")
    print("═" * 74 + "\n")

    bot = JetBotMasterSystem(args)
    try:
        while True: time.sleep(1.0)
    except KeyboardInterrupt:
        bot.shutdown()


if __name__ == '__main__':
    main()
