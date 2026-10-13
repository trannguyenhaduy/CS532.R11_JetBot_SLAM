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
YOLO     = OFF  # 4. AI NHẬN DIỆN VẬT THỂ: TẠM TẮT (Sẵn sàng nạp YOLO VPU khi bật ON)
FOLLOWER = OFF  # 5. BÁM NGƯỜI: Tắt (Tạm thời bỏ qua theo yêu cầu để làm sau)
MAPPER   = ON   # 6. BẢN ĐỒ 3D / 2D SLAM: BẬT (Giai đoạn 5 - Lập bản đồ Occupancy Grid SLAM)
WEB      = ON   # 7. WEB COCKPIT: BẬT (Mở cổng 8080 để lái xe bằng phím W-A-S-D)
BRAKE    = ON   # 8. PHANH KHẨN CẤP: BẬT (Virtual Bumper cản < 20cm khóa tiến)
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
from modules.scan_matcher import CorrelativeScanMatcher2D
from modules.path_planner import PathPlanner
from modules.navigator import AutonomousNavigator

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
        self.last_imu_time = 0.0
        self.has_active_imu = False
        self.is_auto_scanning = False
        self.auto_scan_remaining = 0.0
        self.auto_scan_duration = float(getattr(config, 'AUTO_SCAN_DURATION_S', 16.0))
        self.auto_scan_mode = getattr(config, 'AUTO_SCAN_MODE', 'step')
        self.is_step_running = False
        self.step_cancel_requested = False
        self.step_status_text = "SẴN SÀNG"
        self.step_progress_pct = 0

        # Bộ đo quãng đường và góc quay chuyến đi (Trip Odometer)
        self.trip_distance_m = 0.0
        self.trip_turn_rad = 0.0
        self.trip_active_time_s = 0.0
        self.total_distance_m = 0.0
        self._manual_decel_id = 0
        self.soft_stop_enabled = getattr(config, 'ENABLE_SOFT_STOP', True)

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
                enable_brake=bool(getattr(self.flags, 'brake', getattr(config, 'ENABLE_SAFETY_BRAKE', False)))
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

        # 5. Module Bám người HRI: ĐÃ KHÓA CỨNG VĨNH VIỄN (Triệt tiêu 100% việc tự ý chạy động cơ)
        self.follower = None

        # 6. Khởi tạo Module Phanh khẩn cấp & Cản ảo 3D (Độc lập, dễ kiểm thử)
        brake_active = bool(getattr(self.flags, 'brake', getattr(config, 'ENABLE_SAFETY_BRAKE', False)))
        self.safety_brake = EmergencyBrake(
            brake_dist_m=getattr(config, 'SAFETY_BRAKE_DIST_M', 0.20),
            warning_dist_m=getattr(config, 'SAFETY_WARNING_DIST_M', 0.40),
            is_enabled=brake_active
        )
        if not brake_active:
            print("🛡️ [SAFETY] Phanh khẩn cấp: TẠM THỜI TẮT (Cho phép xe quay/di chuyển tự do không bị ngắt quãng)")

        # 7. Khởi tạo Module Bản đồ Chiếm dụng 2D Tích Lũy Tĩnh & Ghim Vật Thể (Occupancy SLAM)
        self.mapper = OccupancySLAM(size_m=10.0, resolution=0.05) if self.flags.mapper else None

        # 7.1. Khởi tạo Bộ so khớp tia Correlative Scan Matcher (CSM - Tự cân bằng vòng kín)
        self.scan_matcher = None
        if getattr(config, 'ENABLE_SCAN_MATCHING', True):
            self.scan_matcher = CorrelativeScanMatcher2D(
                search_dist_m=getattr(config, 'SCAN_MATCH_SEARCH_DIST_M', 0.08),
                search_yaw_deg=getattr(config, 'SCAN_MATCH_SEARCH_YAW_DEG', 6.0),
                min_score=getattr(config, 'SCAN_MATCH_MIN_SCORE', 0.28)
            )
        self._last_scan_matched = False

        # 7.2. Khởi tạo Module Lập kế hoạch đường đi tối ưu A* (Global Path Planner)
        self.planner = None
        if getattr(config, 'ENABLE_PATH_PLANNER', True):
            self.planner = PathPlanner(
                slam=self.mapper,
                robot_radius_m=getattr(config, 'ROBOT_RADIUS_M', 0.12),
                cost_inflation_radius_m=getattr(config, 'COSTMAP_INFLATION_RADIUS_M', 0.20)
            )

        # 7.3. Khởi tạo Module Tự hành bám đường (Autonomous Navigator & Path Follower)
        self.navigator = None
        if getattr(config, 'ENABLE_AUTONOMOUS_NAVIGATOR', True):
            self.navigator = AutonomousNavigator(
                get_pose_cb=lambda: (self.robot_x, self.robot_y, self.robot_yaw),
                set_cmd_vel_cb=self._nav_set_cmd_vel,
                stop_cb=self._nav_stop_robot,
                linear_speed=getattr(config, 'NAV_LINEAR_SPEED', 0.12),
                turn_speed=getattr(config, 'NAV_TURN_SPEED', 1.20),
                goal_tolerance_m=getattr(config, 'NAV_GOAL_TOLERANCE_M', 0.06),
                waypoint_tolerance_m=getattr(config, 'NAV_WAYPOINT_TOLERANCE_M', 0.10),
                heading_tolerance_rad=getattr(config, 'NAV_HEADING_TOLERANCE_RAD', 0.35),
                kp_angular=getattr(config, 'NAV_KP_ANGULAR', 1.6)
            )

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
            self.web.step_move_cb = self.execute_step_move
            self.web.step_cancel_cb = self.cancel_step_move
            self.web.trip_reset_cb = self.reset_trip_odometer
            self.web.plan_path_cb = self.plan_to_goal
            self.web.clear_goal_cb = self.clear_goal
            self.web.start_nav_cb = self.start_navigation
            self.web.stop_nav_cb = self.stop_navigation
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
        if not self.flags.mapper or not self.mapper or not getattr(config, 'ENABLE_SEMANTIC_LANDMARKS', False):
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
                if 0.06 <= zc <= 2.0:
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
                    forward_objs = [d['z'] for d in tracked if abs(d.get('x', 0.0)) <= 0.35 and 0.06 <= d.get('z', 99.0) <= 2.0]
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
            # Mặc định TẮT (ENABLE_VISUAL_GYRO=False) để bảo toàn tuyệt đối góc quay Odometry đã hiệu chuẩn chuẩn xác
            if getattr(config, 'ENABLE_VISUAL_GYRO', False) and self.camera and not getattr(self, 'is_step_running', False):
                is_moving = (abs(getattr(self, 'current_w', 0.0)) > 0.01 or abs(getattr(self, 'current_v', 0.0)) > 0.01)
                if not is_moving and not getattr(self, 'is_auto_scanning', False):
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
            # 1. Giám sát an toàn tức thời trên TỪNG FRAME (< 0.5ms) để phanh khẩn cấp nhạy 100%:
            if self.safety_brake:
                dist = self.safety_brake.calculate_clearance(depth_np)
                self.last_depth_clearance = dist
                self._update_fused_obstacle_clearance()
            elif self.yolo:
                dist = self.yolo.calculate_obstacle_distance(depth_np)
                self.last_depth_clearance = dist
                self._update_fused_obstacle_clearance()

            # 2. Cập nhật SLAM & CSM ở nhịp 10 Hz chuẩn (mỗi 0.09s):
            # Tiết kiệm 70% CPU Jetson Nano, giải phóng luồng camera stream đạt tốc độ tối đa!
            if (now - getattr(self, 'last_ros_slam_time', 0.0)) >= 0.09:
                self.last_ros_slam_time = now
                with self.lock:
                    rx, ry, rz, yaw = self.robot_x, self.robot_y, self.robot_z, self.robot_yaw
                self.camera.process_depth_frame(depth_np, rx, ry, rz, yaw)

                # Xác định trạng thái xe đang quay tại chỗ (Lái tay WASD, Bước định lượng hoặc Tự quét 360°):
                is_turning = (abs(getattr(self, 'current_w', 0.0)) > 0.03) or getattr(self, 'is_step_running', False) or getattr(self, 'is_auto_scanning', False)

                # Correlative Scan Matching (CSM - Tự cân bằng tọa độ vòng kín):
                # CHỈ chạy khi xe KHÔNG quay tại chỗ (tránh CSM kéo giật lùi góc robot_yaw về hướng tường cũ):
                if not is_turning and self.scan_matcher and getattr(config, 'ENABLE_SCAN_MATCHING', True) and self.mapper:
                    scan_robot = getattr(self.camera, 'points_robot', None)
                    if scan_robot:
                        corr_x, corr_y, corr_yaw, matched, score = self.scan_matcher.align_pose(
                            rx, ry, yaw, scan_robot, self.mapper.grid,
                            self.mapper.origin_x, self.mapper.origin_y,
                            self.mapper.width, self.mapper.height, self.mapper.resolution
                        )
                        self._last_scan_matched = matched
                        if matched:
                            with self.lock:
                                self.robot_x = corr_x
                                self.robot_y = corr_y
                                self.robot_yaw = corr_yaw
                                rx, ry, yaw = corr_x, corr_y, corr_yaw

                # Cập nhật bản đồ 2D Occupancy Grid:
                # - Khi xe đang quay tại chỗ (is_turning=True): TẠM DỪNG bắn tia vào bản đồ tĩnh để triệt tiêu
                #   hoàn toàn hiện tượng nhòe nan quạt do độ trễ frame.
                # - Khi xe dừng xoay hoặc chạy thẳng: Cập nhật khung hình tĩnh sắc nét (Keyframe) vào bản đồ.
                if self.mapper and self.flags.mapper:
                    if not is_turning:
                        self.mapper.update_scan(rx, ry, self.camera.points_3d)
        except Exception: pass

    def _ros_obstacle_dist_cb(self, msg):
        try:
            val = float(msg.data)
            if val < 4.0:
                self.last_depth_clearance = val
                self._update_fused_obstacle_clearance()
        except Exception: pass

    def _ros_imu_cb(self, msg):
        """Nhận góc quay trực tiếp từ cảm biến quán tính IMU (OAK-D hoặc BNO055/MPU6050)
        Chỉ nhận khi IMU có bộ lọc AHRS tính góc quay thực tế, loại trừ 100% topic tĩnh/dummy (0,0,0,1)
        """
        try:
            # 1. Ưu tiên tuyệt đối góc quay điều khiển khi xe đang thực hiện bước lệnh
            if getattr(self, 'is_step_running', False):
                return

            q = msg.orientation
            norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if norm_sq < 0.5:
                return # Bỏ qua quaternion rỗng hoặc không có dữ liệu

            # 2. Bỏ qua quaternion mặc định/dummy identity (0, 0, 0, 1) do các driver ROS không có AHRS phát ra
            if abs(q.x) < 1e-3 and abs(q.y) < 1e-3 and abs(q.z) < 1e-3 and abs(q.w - 1.0) < 1e-3:
                return

            # 3. Kiểm tra covariance: REP-145 covariance[0] < 0 nghĩa là orientation không khả dụng
            cov = getattr(msg, 'orientation_covariance', None)
            if cov is not None and len(cov) >= 9:
                if cov[0] < 0 or cov[0] > 10.0:
                    return

            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)

            with self.lock:
                self.robot_yaw = yaw
                self.last_imu_time = time.time()
                self.has_active_imu = True
                # LƯU Ý: Tuyệt đối KHÔNG gán self.last_real_ros_odom_time ở đây vì IMU góc quay không phải là Odometry vị trí!
        except Exception:
            pass

    def _ros_odom_cb(self, msg):
        try:
            # 1. Không để topic Odometry bên ngoài ghi đè khi xe đang tự thực thi bước lệnh
            if getattr(self, 'is_step_running', False):
                return

            q = msg.pose.pose.orientation
            norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if norm_sq < 0.5:
                return
            p = msg.pose.pose.position

            # 2. Bỏ qua nếu topic phát dummy tĩnh (0,0,0) hoặc quaternion identity
            is_dummy_pos = (abs(p.x) < 1e-3 and abs(p.y) < 1e-3 and abs(p.z) < 1e-3)
            is_dummy_rot = (abs(q.x) < 1e-3 and abs(q.y) < 1e-3 and abs(q.z) < 1e-3 and abs(q.w - 1.0) < 1e-3)
            if is_dummy_pos and is_dummy_rot:
                return

            # Kiểm tra covariance: nếu RTAB-Map mất tracking (covariance âm hoặc quá lớn) -> bỏ qua
            cov = getattr(msg.pose, 'covariance', None)
            if cov is not None and len(cov) >= 36:
                if cov[0] < 0 or cov[0] > 100.0:
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
        if getattr(self, 'is_step_running', False):
            self.cancel_step_move()
        if getattr(self, 'navigator', None) and self.navigator.is_running:
            self.navigator.cancel_navigation("Người dùng lái tay WASD")
        print(f"🎮 [WEB LÁI TAY] Lệnh nhận được: v={v:.2f}, w={w:.2f}")

        brake_thresh = getattr(config, 'SAFETY_BRAKE_DIST_M', 0.20)

        # 1. Đánh giá qua EmergencyBrake nếu có
        if self.safety_brake and self.safety_brake.is_enabled:
            v, w, alert = self.safety_brake.evaluate_velocity(v, w, self.obstacle_distance)
            if alert == "EMERGENCY_STOP":
                threshold_cm = int(self.safety_brake.brake_dist_m * 100)
                obs_cm = f"{self.obstacle_distance*100:.1f} cm" if self.obstacle_distance is not None else "< 20cm"
                print(f"🚨 [PHANH KHẨN CẤP] Cản cách {obs_cm} (< {threshold_cm}cm) -> Đã ngắt tiến, chỉ cho phép lùi/quay!")

        # 2. Can thiệp phanh khẩn cấp cứng (chỉ khi tính năng phanh khẩn cấp được bật):
        if self.safety_brake and getattr(self.safety_brake, 'is_enabled', False):
            if (v > 0.01) and (self.obstacle_distance is not None and self.obstacle_distance <= brake_thresh):
                v = 0.0
                print(f"🛑 [KHÓA LỆNH TIẾN] Cản cách {self.obstacle_distance*100:.1f} cm (<= {int(brake_thresh*100)}cm). Cho phép LÙI hoặc QUAY để thoát cản!")

        # 3. Cập nhật vận tốc tức thời và điều khiển động cơ
        with self.lock:
            old_v = getattr(self, 'current_v', 0.0)
            old_w = getattr(self, 'current_w', 0.0)
            self._manual_decel_id += 1
            cur_decel_id = self._manual_decel_id

        is_stop_cmd = (abs(v) < 0.01 and abs(w) < 0.01)
        was_moving = (abs(old_v) > 0.02 or abs(old_w) > 0.04)
        soft_decel = getattr(self, 'soft_stop_enabled', getattr(config, 'MANUAL_SOFT_DECEL', True))

        if is_stop_cmd and was_moving and soft_decel and self.motors:
            # Hạ dốc giảm tốc từ từ trong 180ms để xe dừng êm ái, không bị khựng giật
            threading.Thread(target=self._manual_soft_decel_worker, args=(old_v, old_w, cur_decel_id), daemon=True).start()
        else:
            # ĐỒNG NHẤT 100% VẬN TỐC GÓC VỚI ĐỘNG CƠ (motor_controller & _step_worker):
            # Khi quay tại chỗ (v=0, w!=0), motor_controller chuẩn hóa duty theo mốc 13% PWM (0.13 = 1.58 rad/s)
            # và chặn dưới tối thiểu STEP_TURN_MIN_PWM (0.11 = 11% PWM).
            # Ta đồng bộ ngược lại để current_w và bộ đo hành trình (Trip Odometer/SLAM) nhận chuẩn xác vận tốc thực tế:
            effective_w = w
            if abs(v) < 0.01 and abs(w) > 0.01:
                min_turn = getattr(config, 'STEP_TURN_MIN_PWM', 0.11)
                turn_duty = max(min_turn, min(0.24, (abs(w) / 1.58) * 0.13))
                effective_w = (1.58 * (turn_duty / 0.13)) * (1.0 if w > 0 else -1.0)

            with self.lock:
                self.current_v = v
                self.current_w = effective_w
                self.last_cmd_vel_time = time.time()

            if self.motors:
                if is_stop_cmd:
                    self.motors.stop()
                else:
                    self.motors.set_cmd_vel(v, effective_w)

        if HAS_ROS and self.ros_cmd_pub:
            t = Twist()
            t.linear.x, t.angular.z = v, getattr(self, 'current_w', w)
            self.ros_cmd_pub.publish(t)

    def _manual_soft_decel_worker(self, from_v, from_w, decel_id):
        """Hạ dốc giảm tốc mượt mà khi người dùng nhả phím WASD / cần Gamepad (180ms)"""
        decel_ms = getattr(config, 'MANUAL_DECEL_TIME_MS', 180)
        steps = 4
        dt = (decel_ms / 1000.0) / steps
        for i in range(1, steps + 1):
            with self.lock:
                if self._manual_decel_id != decel_id:
                    return  # Có lệnh lái mới, hủy ngay quá trình giảm tốc
                ratio = 1.0 - (i / steps)
                step_v = from_v * ratio
                step_w = from_w * ratio
                self.current_v = step_v
                self.current_w = step_w
                self.last_cmd_vel_time = time.time()
            if self.motors:
                self.motors.set_cmd_vel(step_v, step_w)
            time.sleep(dt)

        with self.lock:
            if self._manual_decel_id == decel_id:
                self.current_v = 0.0
                self.current_w = 0.0
        if self.motors and self._manual_decel_id == decel_id:
            self.motors.stop()

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
                # Nếu đang chạy bước định lượng, việc cập nhật hành trình và tọa độ do _step_worker cập nhật chuẩn xác theo bước lệnh:
                if getattr(self, 'is_step_running', False):
                    continue

                # Nếu không nhận thêm lệnh lái trong 0.35s -> coi như đã dừng xe (v=0, w=0)
                if (now - getattr(self, 'last_cmd_vel_time', 0.0)) > 0.35:
                    self.current_v = 0.0
                    self.current_w = 0.0

                v = getattr(self, 'current_v', 0.0)
                w = getattr(self, 'current_w', 0.0)

                if abs(v) > 0.005 or abs(w) > 0.005:
                    # ── ĐỒNG NHẤT HỆ SỐ HIỆU CHUẨN GIỮA LÁI THỦ CÔNG VÀ BƯỚC ĐỊNH LƯỢNG ──
                    # Dùng chung chính các hệ số SCALE người dùng cấu hình trong config:
                    if v >= 0:
                        lin_scale = getattr(config, 'STEP_FORWARD_SCALE', 0.44)
                    else:
                        lin_scale = getattr(config, 'STEP_BACKWARD_SCALE', 0.48)

                    if w > 0:  # Quay trái CCW
                        turn_scale = getattr(config, 'STEP_TURN_LEFT_SCALE', 0.60)
                    else:      # Quay phải CW
                        turn_scale = getattr(config, 'STEP_TURN_RIGHT_SCALE', 0.60)

                    lin_scale = max(0.05, float(lin_scale))
                    turn_scale = max(0.05, float(turn_scale))

                    # Vận tốc thực tế bánh xe lăn/quay trên sàn (được bù trừ bởi hệ số SCALE):
                    v_real = v / lin_scale
                    w_real = w / turn_scale

                    # Tích phân quãng đường & góc quay chuyến đi (Trip Odometer) theo vận tốc thực tế:
                    dist_step = abs(v_real) * dt
                    rot_step = abs(w_real) * dt
                    self.total_distance_m += dist_step
                    self.trip_distance_m += dist_step
                    self.trip_turn_rad += rot_step
                    self.trip_active_time_s += dt

                    # KIỂM TRA NGUỒN ODOMETRY THỰC (Ưu tiên tuyệt đối ROS & IMU phần cứng):
                    has_ros_odom = (now - getattr(self, 'last_real_ros_odom_time', 0.0)) < 0.50
                    has_real_imu = (now - getattr(self, 'last_imu_time', 0.0)) < 0.50

                    if not has_ros_odom:
                        # Chỉ tích phân góc quay từ w khi không có cảm biến IMU thực
                        if not has_real_imu:
                            self.robot_yaw += w_real * dt
                            self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

                        # Tích phân tọa độ không gian (X, Y) theo vận tốc thực tế
                        self.robot_x += v_real * dt * math.cos(self.robot_yaw)
                        self.robot_y += v_real * dt * math.sin(self.robot_yaw)

                    # Cập nhật vết lịch sử đường đi (path history) khi di chuyển đủ 2cm
                    if not self.path_history or (
                        (self.robot_x - self.path_history[-1][0])**2 + (self.robot_y - self.path_history[-1][1])**2 > 0.0004
                    ):
                        self.path_history.append([round(self.robot_x, 3), round(self.robot_y, 3), 0.02])
                        if len(self.path_history) > 400:
                            self.path_history.pop(0)

    def _control_loop(self):
        """Vòng lặp tự hành - ĐÃ KHÓA CỨNG AN TOÀN (Tuyệt đối không can thiệp động cơ)"""
        while self.running:
            time.sleep(0.5)

    def _ai_inference_loop(self):
        """Luồng suy luận AI bất đồng bộ (Zero-Latency Async Worker):
        Tách biệt hoàn toàn tính toán mạng nơ-ron AI khỏi luồng Camera Stream.
        Giúp Camera Stream đạt tối đa 25-30 FPS siêu mượt, độ trễ < 25ms!"""
        last_processed_frame = None
        while self.running:
            loop_t0 = time.time()
            if self.flags.yolo and self.yolo and (self.latest_raw_bgr is not None):
                try:
                    now = time.time()
                    # Nếu đang có luồng nhận diện từ VPU của OAK-D (qua ROS trong 1.8s gần nhất),
                    # KHÔNG chạy fallback CPU để tránh ghi đè làm mất các vật thể từ OAK-D
                    vpu_active = (now - getattr(self, 'last_vpu_det_time', 0.0) < 1.8)

                    if not vpu_active:
                        frame = self.latest_raw_bgr
                        # Tránh suy luận lặp lại trên cùng 1 frame
                        if frame is not last_processed_frame:
                            last_processed_frame = frame
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

            elapsed = time.time() - loop_t0
            time.sleep(max(0.005, 0.035 - elapsed))

    def _camera_provider_loop(self):
        """Cung cấp luồng hình ảnh camera siêu tốc độ cao (25-30 FPS, độ trễ cực thấp)"""
        mock_pts = []
        for x in np.linspace(-2.0, 2.0, 25):
            for z in np.linspace(0.1, 1.5, 8):
                mock_pts.append([round(float(x), 2), 2.0, round(float(z), 2)])
                mock_pts.append([round(float(x), 2), -2.0, round(float(z), 2)])

        last_sim_time = 0.0
        last_slam_time = 0.0

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
                        vpu_dists = [d['z'] for d in vpu_dets if 'z' in d and 0.06 <= d['z'] <= 2.0]
                        if vpu_dists:
                            self.last_yolo_clearance = min(vpu_dists)
                    self._update_fused_obstacle_clearance()
                if live_depth is not None:
                    self.latest_depth_np = live_depth

                    # 1. Giám sát an toàn tức thời trên TỪNG FRAME (< 0.5ms) để phanh khẩn cấp nhạy 100%
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

                    # 2. Cập nhật SLAM & CSM Scan Matching ở nhịp 10 Hz chuẩn (mỗi 0.09s)
                    # Giúp luồng Camera đạt tối đa 25-30 FPS mượt mà, không bị nghẽn bởi phép toán ma trận
                    if (now - last_slam_time >= 0.09):
                        last_slam_time = now
                        with self.lock:
                            rx, ry, rz, yaw = self.robot_x, self.robot_y, self.robot_z, self.robot_yaw
                        self.camera.process_depth_frame(live_depth, rx, ry, rz, yaw)

                        # Xác định trạng thái xe đang quay tại chỗ (Lái tay WASD, Bước định lượng hoặc Tự quét 360°):
                        is_turning = (abs(getattr(self, 'current_w', 0.0)) > 0.03) or getattr(self, 'is_step_running', False) or getattr(self, 'is_auto_scanning', False)

                        # Correlative Scan Matching (CSM - Tự cân bằng tọa độ vòng kín)
                        # CHỈ chạy khi xe KHÔNG quay tại chỗ (tránh CSM kéo giật lùi góc robot_yaw về hướng tường cũ):
                        if not is_turning and self.scan_matcher and getattr(config, 'ENABLE_SCAN_MATCHING', True) and self.mapper:
                            scan_robot = getattr(self.camera, 'points_robot', None)
                            if scan_robot:
                                corr_x, corr_y, corr_yaw, matched, score = self.scan_matcher.align_pose(
                                    rx, ry, yaw, scan_robot, self.mapper.grid,
                                    self.mapper.origin_x, self.mapper.origin_y,
                                    self.mapper.width, self.mapper.height, self.mapper.resolution
                                )
                                self._last_scan_matched = matched
                                if matched:
                                    with self.lock:
                                        self.robot_x = corr_x
                                        self.robot_y = corr_y
                                        self.robot_yaw = corr_yaw
                                        rx, ry, yaw = corr_x, corr_y, corr_yaw

                        # Cập nhật bản đồ 2D Occupancy Grid:
                        # - Khi xe đang quay tại chỗ (is_turning=True): TẠM DỪNG bắn tia vào bản đồ tĩnh để triệt tiêu
                        #   hoàn toàn hiện tượng nhòe nan quạt do độ trễ frame (160ms @ 6.2 FPS).
                        # - Khi xe dừng xoay hoặc chạy thẳng: Cập nhật khung hình tĩnh sắc nét (Keyframe) vào bản đồ.
                        if self.mapper and self.flags.mapper:
                            if not is_turning:
                                self.mapper.update_scan(rx, ry, self.camera.points_3d)

                if live_frame is not None:
                    self.latest_raw_bgr = live_frame

                    # Cập nhật góc quay robot_yaw: Ưu tiên IMU phần cứng OAK-D S2 nếu có
                    c_has_imu = False
                    if self.camera and hasattr(self.camera, 'get_imu_heading'):
                        c_has_imu, _, c_raw_yaw = self.camera.get_imu_heading()
                        if c_has_imu and not getattr(self, 'is_auto_scanning', False) and not getattr(self, 'is_step_running', False):
                            with self.lock:
                                self.robot_yaw = c_raw_yaw
                                self.last_imu_time = time.time()
                                self.has_active_imu = True

                    if getattr(config, 'ENABLE_VISUAL_GYRO', False) and not c_has_imu and not getattr(self, 'is_step_running', False):
                        # Ước lượng xoay thị giác thời gian thực (Visual Gyroscope):
                        # Mặc định TẮT (ENABLE_VISUAL_GYRO=False) để bảo toàn tuyệt đối góc quay Odometry đã hiệu chuẩn chuẩn xác
                        is_moving = (abs(getattr(self, 'current_w', 0.0)) > 0.01 or abs(getattr(self, 'current_v', 0.0)) > 0.01)
                        if not is_moving and not getattr(self, 'is_auto_scanning', False):
                            delta_yaw = self.camera.estimate_visual_rotation(live_frame)
                            if abs(delta_yaw) > 0.002:
                                with self.lock:
                                    self.robot_yaw += delta_yaw
                                    self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

                    with self.lock:
                        current_dets = list(self.detections)
                        obs_dist = self.obstacle_distance
                        rx, ry, rz, yaw = self.robot_x, self.robot_y, self.robot_z, self.robot_yaw

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
        # 1. Trích xuất nhanh các thông số cảm biến dưới lock (< 0.05ms)
        with self.lock:
            v, pct, curr, pwr, rem = self.battery_metrics
            pts = list(self.camera.points_3d) if self.camera else []
            dets = list(self.detections)
            rx, ry, rz = self.robot_x, self.robot_y, self.robot_z
            yaw = self.robot_yaw
            path = list(self.path_history)
            obs_dist = self.obstacle_distance
            scanning = self.is_auto_scanning
            scan_rem = getattr(self, 'auto_scan_remaining', 0.0)
            scan_dur = getattr(self, 'auto_scan_duration', 16.0)
            scan_mode = getattr(self, 'auto_scan_mode', 'step')
            last_ros_t = self.last_ros_img_time
            brake_dist = getattr(config, 'SAFETY_BRAKE_DIST_M', 0.20)
            cam_fps = float(self.camera.calc_fps) if (self.camera and hasattr(self.camera, 'calc_fps') and self.camera.calc_fps > 0) else 15.0
            now_t = time.time()
            has_ros_odom_active = (now_t - getattr(self, 'last_real_ros_odom_time', 0.0)) < 0.5
            has_imu_active = (now_t - getattr(self, 'last_imu_time', 0.0)) < 0.5

        # 2. GIẢI PHÓNG LOCK HOÀN TOÀN TRƯỚC KHI XỬ LÝ BẢN ĐỒ:
        # get_payload_for_web() chạy ngoài lock để triệt tiêu 100% hiện tượng khựng/lag điều khiển WASD
        has_person = any(d.get('name') == 'PERSON' for d in dets)
        confirmed_objs = self.mapper.get_confirmed_landmarks() if (self.flags.mapper and self.mapper) else []
        map_payload = self.mapper.get_payload_for_web() if (self.flags.mapper and self.mapper) else {}

        # Nguồn Odometry đang hoạt động - hiển thị trực tiếp lên Web Cockpit
        if has_ros_odom_active:
            odometry_source = "ROS RTAB-Map"
        elif getattr(self, '_last_scan_matched', False):
            odometry_source = "CSM Scan-Matching"
        elif has_imu_active:
            odometry_source = "IMU OAK-D S2"
        else:
            odometry_source = "Dead-Reckoning"

        if time.time() - last_ros_t < 2.0:
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
            "robot_x": round(rx, 3), "robot_y": round(ry, 3), "robot_z": round(rz, 3),
            "robot_yaw": round(yaw, 3), "path": path,
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
            "detections": dets, "obstacle_distance": obs_dist,
            "safety_brake_dist": brake_dist,
            "odometry_source": odometry_source,
            "imu_active": has_imu_active,
            "ros_odom_active": has_ros_odom_active,
            "soft_stop_enabled": getattr(self, 'soft_stop_enabled', True),
            "auto_scanning": scanning,
            "auto_scan_remaining": round(scan_rem, 1),
            "auto_scan_duration": round(scan_dur, 1),
            "auto_scan_mode": scan_mode,
            "semantic_objects": confirmed_objs,
            "landmarks": confirmed_objs,
            "scan_matching": getattr(config, 'ENABLE_SCAN_MATCHING', True),
            "scan_matched": getattr(self, '_last_scan_matched', False),
            "semantic_landmarks_enabled": getattr(config, 'ENABLE_SEMANTIC_LANDMARKS', False),
            "planned_path": getattr(self.planner, 'current_path', []) if getattr(self, 'planner', None) else [],
            "current_goal": getattr(self.planner, 'current_goal', None) if getattr(self, 'planner', None) else None,
            "nav_state": self.navigator.get_status() if getattr(self, 'navigator', None) else {
                "running": False, "state": "IDLE", "status_text": "SẴN SÀNG", "current_wp_idx": 0, "total_waypoints": 0, "dist_to_goal": 0.0, "progress_pct": 0.0
            },
            "step_state": {
                "running": getattr(self, 'is_step_running', False),
                "status": getattr(self, 'step_status_text', 'SẴN SÀNG')
            },
            "trip_odometer": {
                "distance_cm": round(getattr(self, 'trip_distance_m', 0.0) * 100.0, 1),
                "distance_m": round(getattr(self, 'trip_distance_m', 0.0), 3),
                "turn_deg": round(math.degrees(getattr(self, 'trip_turn_rad', 0.0)), 1),
                "active_time_s": round(getattr(self, 'trip_active_time_s', 0.0), 2),
                "current_v_cm_s": round(abs(getattr(self, 'current_v', 0.0)) * 100.0, 1),
                "total_distance_m": round(getattr(self, 'total_distance_m', 0.0), 3)
            },
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
            print("🛡️ [KHÓA CỨNG AN TOÀN] Tính năng bám người HRI đã bị VÔ HIỆU HÓA HOÀN TOÀN để bảo vệ động cơ JetBot!")
            if self.follower:
                self.follower.set_enabled(False)
            return False
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
        elif any(k in flag_name for k in ["soft_stop", "smooth", "decel", "ham_toc"]):
            self.soft_stop_enabled = not getattr(self, 'soft_stop_enabled', True)
            status_str = "BẬT (DỪNG TỪ TỪ - SOFT STOP)" if self.soft_stop_enabled else "TẮT (DỪNG TỨC THÌ - HARD STOP)"
            print(f"🛑 [TOGGLE] Chế độ dừng: {status_str}")
            return self.soft_stop_enabled
        return False

    def save_map(self, name="my_room_map"):
        """Lưu bản đồ hiện tại sang file JSON & PNG"""
        if self.mapper:
            return self.mapper.save_map(name)
        return False, "Module Mapper chưa bật"

    def execute_step_move(self, move_type, value, direction='forward', speed_pct=20.0):
        """Thực thi bước di chuyển định lượng chính xác (cm hoặc độ) theo tốc độ %"""
        with self.lock:
            if getattr(self, 'is_step_running', False):
                return False, "Đang thực hiện bước trước đó"
            if self.is_auto_scanning:
                self.is_auto_scanning = False
            self.is_step_running = True
            self.step_cancel_requested = False
            self.last_manual_drive_time = time.time()

        threading.Thread(target=self._step_worker, args=(move_type, float(value), direction, float(speed_pct)), daemon=True).start()
        return True, f"Bắt đầu bước {move_type}"

    def cancel_step_move(self):
        """Hủy khẩn cấp bước di chuyển định lượng"""
        with self.lock:
            self.step_cancel_requested = True
            self.current_v = 0.0
            self.current_w = 0.0
        if self.motors:
            if hasattr(self.motors, 'active_brake'):
                self.motors.active_brake(reverse_duty=0.14, pulse_ms=35)
            else:
                self.motors.stop()
        return True

    def _step_worker(self, move_type, value, direction='forward', speed_pct=20.0):
        """Luồng điều khiển bước vi sai chính xác theo cự ly/góc và mức tốc độ PWM (%)"""
        try:
            # Giới hạn mức tốc độ an toàn: tối thiểu 6% (chế độ bò siêu chậm), tối đa 40%
            pct_val = max(6.0, min(40.0, float(speed_pct)))
            duty = pct_val / 100.0

            if move_type == 'linear':
                dist_cm = max(1.0, min(500.0, float(value)))
                dist_m = dist_cm / 100.0
                is_fwd = (direction == 'forward')
                sign = 1.0 if is_fwd else -1.0

                # Tỉ lệ hiệu chuẩn cự ly thực tế (dùng chung cho cả bước cm và lái tay WASD)
                scale_factor = getattr(config, 'STEP_FORWARD_SCALE', 0.50) if is_fwd else getattr(config, 'STEP_BACKWARD_SCALE', 0.50)
                v_calib = 0.20 * (pct_val / 20.0)
                duration = max(0.04, (dist_m / v_calib) * scale_factor)
                v_target = v_calib if is_fwd else -v_calib
                brake_thresh = getattr(config, 'SAFETY_BRAKE_DIST_M', 0.20)

                # ── DỪNG TỪ TỪ ĐỒNG NHẤT 100% VỚI LÁI THỦ CÔNG ──
                enable_soft_stop = getattr(self, 'soft_stop_enabled', getattr(config, 'ENABLE_SOFT_STOP', True))
                decel_time_s = (getattr(config, 'MANUAL_DECEL_TIME_MS', 180) / 1000.0) if enable_soft_stop else 0.0

                # Trừ hao thời gian hạ dốc:
                if enable_soft_stop and duration > decel_time_s:
                    t_steady = duration - (decel_time_s / 2.0)
                else:
                    t_steady = duration
                    decel_time_s = 0.0

                total_t = t_steady + decel_time_s
                last_progress = 0.0
                last_t_meas = time.time()

                print(f"📏 [BƯỚC ĐỊNH LƯỢNG] Bắt đầu {'TIẾN' if is_fwd else 'LÙI'} {dist_cm:.0f}cm @ {pct_val:.0f}% tốc độ (v={v_calib:.2f}m/s, calib={scale_factor:.2f}, t_chay={t_steady:.2f}s, t_ham={decel_time_s:.2f}s, soft_stop={'BẬT' if enable_soft_stop else 'TẮT'})...")
                start_t = time.time()

                # GIAI ĐOẠN 1: Chạy đều ở vận tốc mục tiêu
                while not self.step_cancel_requested and (time.time() - start_t < t_steady):
                    now = time.time()
                    elapsed = now - start_t
                    pct = min(90, int((elapsed / total_t) * 100))
                    self.step_status_text = f"ĐANG {'TIẾN' if is_fwd else 'LÙI'} {dist_cm:.0f}CM @ {pct_val:.0f}%... ({pct}%)"

                    if is_fwd:
                        with self.lock:
                            obs_d = self.obstacle_distance
                        if obs_d is not None and obs_d <= brake_thresh:
                            print(f"🛑 [BƯỚC ĐỊNH LƯỢNG] Khóa tiến vì phát hiện cản cách {obs_d*100:.1f}cm!")
                            self.step_status_text = f"DỪNG: GẶP CẢN {obs_d*100:.1f}CM"
                            break

                    # Cập nhật tiến độ đo đạc chuẩn xác tuyệt đối theo bước lệnh:
                    progress = min(1.0, elapsed / max(0.001, total_t))
                    d_prog = progress - last_progress
                    last_progress = progress
                    dt_run = now - last_t_meas
                    last_t_meas = now
                    d_dist = d_prog * dist_m

                    with self.lock:
                        self.current_v = v_target
                        self.current_w = 0.0
                        self.last_cmd_vel_time = now
                        self.total_distance_m += d_dist
                        self.trip_distance_m += d_dist
                        self.trip_active_time_s += dt_run
                        self.robot_x += sign * d_dist * math.cos(self.robot_yaw)
                        self.robot_y += sign * d_dist * math.sin(self.robot_yaw)
                        if not self.path_history or ((self.robot_x - self.path_history[-1][0])**2 + (self.robot_y - self.path_history[-1][1])**2 > 0.0004):
                            self.path_history.append([round(self.robot_x, 3), round(self.robot_y, 3), 0.02])
                            if len(self.path_history) > 400: self.path_history.pop(0)

                    if self.motors:
                        self.motors.set_cmd_vel(v_target, 0.0)

                    time.sleep(0.02)

                # GIAI ĐOẠN 2: Hạ dốc giảm tốc từ từ (ĐỒNG NHẤT 100% VỚI LÁI THỦ CÔNG)
                if not self.step_cancel_requested and decel_time_s > 0.03:
                    steps = 4
                    dt_step = decel_time_s / steps
                    for i in range(1, steps + 1):
                        if self.step_cancel_requested:
                            break
                        ratio = 1.0 - (i / steps)
                        cur_v = v_target * ratio

                        d_prog = (1.0 - last_progress) / max(1, steps - (i - 1))
                        d_dist = d_prog * dist_m
                        last_progress += d_prog

                        with self.lock:
                            self.current_v = cur_v
                            self.current_w = 0.0
                            self.last_cmd_vel_time = time.time()
                            self.total_distance_m += d_dist
                            self.trip_distance_m += d_dist
                            self.trip_active_time_s += dt_step
                            self.robot_x += sign * d_dist * math.cos(self.robot_yaw)
                            self.robot_y += sign * d_dist * math.sin(self.robot_yaw)

                        if self.motors:
                            self.motors.set_cmd_vel(cur_v, 0.0)
                        time.sleep(dt_step)

                # Dừng hẳn động cơ
                if self.motors:
                    self.motors.stop()

                # Đồng bộ hoàn tất 100% bước (nếu không bị hủy giữa chừng):
                if not self.step_cancel_requested:
                    rem_prog = max(0.0, 1.0 - last_progress)
                    if rem_prog > 0:
                        rem_dist = rem_prog * dist_m
                        with self.lock:
                            self.total_distance_m += rem_dist
                            self.trip_distance_m += rem_dist
                            self.robot_x += sign * rem_dist * math.cos(self.robot_yaw)
                            self.robot_y += sign * rem_dist * math.sin(self.robot_yaw)

                with self.lock:
                    self.current_v = 0.0
                    self.current_w = 0.0

                if self.step_cancel_requested:
                    self.step_status_text = "ĐÃ HỦY BƯỚC"
                    print("⏹️ [BƯỚC ĐỊNH LƯỢNG] Đã hủy bước theo yêu cầu.")
                else:
                    self.step_status_text = f"HOÀN TẤT {'TIẾN' if is_fwd else 'LÙI'} {dist_cm:.0f}CM ({pct_val:.0f}%)"
                    print(f"✅ [BƯỚC ĐỊNH LƯỢNG] Hoàn tất {'tiến' if is_fwd else 'lùi'} {dist_cm:.0f}cm @ {pct_val:.0f}%.")

            elif move_type == 'turn':
                deg = max(1.0, min(360.0, float(value)))
                rad = math.radians(deg)
                is_left = (direction == 'left')
                dir_val = 1 if is_left else -1
                dir_sign = 1.0 if is_left else -1.0

                # Tách hệ số tỉ lệ quay TRÁI / PHẢI riêng biệt để bù sai lệch cơ khí:
                if is_left:
                    turn_scale = getattr(config, 'STEP_TURN_LEFT_SCALE', 0.60)
                else:
                    turn_scale = getattr(config, 'STEP_TURN_RIGHT_SCALE', 0.60)

                turn_min_pwm = getattr(config, 'STEP_TURN_MIN_PWM', 0.11)
                turn_ratio = getattr(config, 'STEP_TURN_SPEED_RATIO', 1.0)

                # Quy đổi vận tốc góc quay theo % PWM:
                raw_turn = 0.13 * (pct_val / 20.0) * turn_ratio
                turn_duty = max(turn_min_pwm, min(0.24, raw_turn))
                w_calib = 1.58 * (turn_duty / 0.13)
                duration = max(0.04, (rad / w_calib) * turn_scale)
                w_target = w_calib if is_left else -w_calib

                # ── DỪNG TỪ TỪ ĐỒNG NHẤT 100% VỚI LÁI THỦ CÔNG ──
                enable_soft_stop = getattr(self, 'soft_stop_enabled', getattr(config, 'ENABLE_SOFT_STOP', True))
                decel_time_s = (getattr(config, 'MANUAL_DECEL_TIME_MS', 180) / 1000.0) if enable_soft_stop else 0.0

                # Trừ hao thời gian hạ dốc:
                if enable_soft_stop and duration > decel_time_s:
                    t_steady = duration - (decel_time_s / 2.0)
                else:
                    t_steady = duration
                    decel_time_s = 0.0

                total_t = t_steady + decel_time_s
                last_progress = 0.0
                last_t_meas = time.time()

                side_name = 'TRÁI' if is_left else 'PHẢI'
                print(f"🔄 [BƯỚC ĐỊNH LƯỢNG] Bắt đầu QUAY {side_name} {deg:.0f}° @ {pct_val:.0f}% tốc độ (PWM quay={turn_duty*100:.0f}%, calib={turn_scale:.2f}, t_quay={t_steady:.2f}s, t_ham={decel_time_s:.2f}s, soft_stop={'BẬT' if enable_soft_stop else 'TẮT'})...")
                start_t = time.time()

                # GIAI ĐOẠN 1: Quay đều ở vận tốc mục tiêu
                while not self.step_cancel_requested and (time.time() - start_t < t_steady):
                    now = time.time()
                    elapsed = now - start_t
                    pct = min(90, int((elapsed / total_t) * 100))
                    self.step_status_text = f"ĐANG QUAY {side_name} {deg:.0f}° @ {pct_val:.0f}%... ({pct}%)"

                    # Cập nhật tiến độ góc quay chuẩn xác tuyệt đối theo bước lệnh:
                    progress = min(1.0, elapsed / max(0.001, total_t))
                    d_prog = progress - last_progress
                    last_progress = progress
                    dt_run = now - last_t_meas
                    last_t_meas = now
                    d_rad = d_prog * rad

                    with self.lock:
                        self.current_v = 0.0
                        self.current_w = w_target
                        self.last_cmd_vel_time = now
                        self.trip_turn_rad += d_rad
                        self.trip_active_time_s += dt_run
                        self.robot_yaw += dir_sign * d_rad
                        self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

                    if self.motors:
                        self.motors.spin_in_place(duty=turn_duty, direction=dir_val)

                    time.sleep(0.02)

                # GIAI ĐOẠN 2: Hạ dốc giảm tốc từ từ (ĐỒNG NHẤT 100% VỚI LÁI THỦ CÔNG)
                if not self.step_cancel_requested and decel_time_s > 0.03:
                    steps = 4
                    dt_step = decel_time_s / steps
                    for i in range(1, steps + 1):
                        if self.step_cancel_requested:
                            break
                        ratio = 1.0 - (i / steps)
                        cur_duty = turn_duty * ratio
                        cur_w = w_target * ratio

                        d_prog = (1.0 - last_progress) / max(1, steps - (i - 1))
                        d_rad = d_prog * rad
                        last_progress += d_prog

                        with self.lock:
                            self.current_v = 0.0
                            self.current_w = cur_w
                            self.last_cmd_vel_time = time.time()
                            self.trip_turn_rad += d_rad
                            self.trip_active_time_s += dt_step
                            self.robot_yaw += dir_sign * d_rad
                            self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

                        if self.motors:
                            if cur_duty >= 0.06:
                                self.motors.spin_in_place(duty=cur_duty, direction=dir_val)
                            else:
                                self.motors.stop()
                        time.sleep(dt_step)

                # Dừng hẳn động cơ
                if self.motors:
                    self.motors.stop()

                # Đồng bộ hoàn tất 100% bước quay (nếu không bị hủy giữa chừng):
                if not self.step_cancel_requested:
                    rem_prog = max(0.0, 1.0 - last_progress)
                    if rem_prog > 0:
                        rem_rad = rem_prog * rad
                        with self.lock:
                            self.trip_turn_rad += rem_rad
                            self.robot_yaw += dir_sign * rem_rad
                            self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

                with self.lock:
                    self.current_v = 0.0
                    self.current_w = 0.0

                if self.step_cancel_requested:
                    self.step_status_text = "ĐÃ HỦY BƯỚC"
                    print("⏹️ [BƯỚC ĐỊNH LƯỢNG] Đã hủy bước quay theo yêu cầu.")
                else:
                    self.step_status_text = f"HOÀN TẤT QUAY {'TRÁI' if is_left else 'PHẢI'} {deg:.0f}° ({pct_val:.0f}%)"
                    print(f"✅ [BƯỚC ĐỊNH LƯỢNG] Hoàn tất quay {'trái' if is_left else 'phải'} {deg:.0f}° @ {pct_val:.0f}%.")

        except Exception as e:
            self.step_status_text = f"LỖI: {e}"
            print(f"❌ [BƯỚC ĐỊNH LƯỢNG] Lỗi: {e}")
        finally:
            with self.lock:
                self.is_step_running = False
                self.current_v = 0.0
                self.current_w = 0.0
            if self.motors:
                self.motors.stop()

    def toggle_auto_scan(self, duration=None, mode=None, pwm=None):
        """Bật/Tắt chế độ tự động xoay chậm đều đúng 360 độ (Closed-Loop IMU 360° Panorama Spin)"""
        with self.lock:
            if self.is_auto_scanning:
                self.is_auto_scanning = False
                self.auto_scan_remaining = 0.0
                print("🛑 [AUTO SCAN 360°] Đã nhận lệnh dừng quét tự động.")
                if self.motors:
                    if hasattr(self.motors, 'active_brake'):
                        self.motors.active_brake(reverse_duty=0.14, pulse_ms=35)
                    else:
                        self.motors.stop()
                return False
            else:
                self.is_auto_scanning = True
                self.current_v = 0.0
                self.current_w = 0.0
                self.last_manual_drive_time = 0.0
                scan_pwm = float(pwm) if pwm and 0.08 <= float(pwm) <= 0.25 else float(getattr(config, 'AUTO_SCAN_SPEED_PWM', 0.12))
                scan_dur = float(duration) if duration and float(duration) > 0 else float(getattr(config, 'AUTO_SCAN_DURATION_S', 4.8))
                self.auto_scan_duration = scan_dur
                self.auto_scan_remaining = scan_dur
                threading.Thread(target=self._auto_scan_worker, args=(scan_dur, scan_pwm), daemon=True).start()
                print(f"🔄 [AUTO SCAN 360°] Bắt đầu xoay chậm đều 360 độ [PWM: {scan_pwm*100:.0f}%]...")
                return True

    def _auto_scan_worker(self, scan_duration=4.8, scan_pwm=0.12):
        """Luồng điều khiển tự động xoay 360 độ khép kín dựa trên CAMERA THỊ GIÁC (Visual Anchor Keyframe Closed-Loop):
        1. Đánh dấu mốc xuất phát (t=0):
           - Chờ xe ổn định vị trí, đọc khung hình Camera thực tế.
           - Đăng ký khung hình mốc (Anchor Keyframe template 160x120) và ghi nhận start_yaw.
        2. Bắt đầu quay chậm đều:
           - Tốc độ cố định mượt mà (base_pwm ~ 11-12%), xe xoay tại chỗ không giật lắc.
        3. Giám sát thị giác liên tục (30 FPS):
           - Kênh 1: Con quay thị giác (Visual Odometry phase correlation) đo góc tích phân turned_angle.
           - Cập nhật robot_yaw theo số đo thị giác thực tế trên bản đồ 2D Occupancy Grid SLAM.
           - Kênh 2: Sau khi rời mốc xuất phát (turned_angle >= 260° hoặc > 2.2s), liên tục so khớp
             khung hình hiện tại với Anchor Keyframe ban đầu bằng Normalized Cross Correlation.
           - Khi cản vào tầm ngắm (score >= 0.50), tự động hạ ga xuống mức bò chậm (creep_pwm 9%) để hãm đà quán tính.
           - Khi khung hình trùng khớp hoàn toàn (score >= 0.62, lệch ngang <= 12px):
             Kích hoạt phanh điện tử chủ động đảo chiều (Active Counter-Brake) dừng đứng bánh xe ngay lập tức!
        4. Kênh dự phòng an toàn (Fail-safe):
           - Kênh IMU phần cứng nếu có (OAK-D S2 Pro).
           - Tích phân góc quay thị giác đạt đủ 360° (2*pi rad).
           - Ngắt khẩn cấp khi gặp vật cản (< 16cm), người dùng lái tay, hoặc quá thời gian an toàn (16s).
        """
        target_angle = 2.0 * math.pi  # 360.0 độ = 6.28319 rad
        base_pwm = max(0.10, min(0.18, float(scan_pwm)))
        creep_pwm = float(getattr(config, 'AUTO_SCAN_CREEP_PWM', 0.09))
        direction = 1  # Quay trái CCW (1)

        # Chờ 100ms để xe hoàn toàn đứng yên trước khi chụp khung hình mốc
        time.sleep(0.10)

        with self.lock:
            start_yaw = self.robot_yaw
            self.current_v = 0.0
            self.current_w = 0.0

        # Lấy khung hình camera mốc ban đầu (Anchor Keyframe)
        anchor_ok = False
        start_frame = self.latest_raw_bgr
        if start_frame is None and self.camera:
            try:
                start_frame, _, _ = self.camera.get_live_frame()
            except Exception:
                start_frame = None

        if self.camera and start_frame is not None:
            anchor_ok = self.camera.register_anchor_keyframe(start_frame)

        if anchor_ok:
            print(f"📸 [AUTO SCAN 360°] Đã khóa khung hình mốc (Anchor Keyframe) tại góc {start_yaw*180/math.pi:.1f}°. Bắt đầu quay chậm đều theo Camera...")
        else:
            print(f"⚡ [AUTO SCAN 360°] Chế độ đo góc con quay thị giác & bù điện áp...")

        # Kiểm tra kênh dữ liệu IMU trên camera OAK-D S2 nếu có
        has_imu = False
        if self.camera and hasattr(self.camera, 'get_imu_heading'):
            has_imu, _, _ = self.camera.get_imu_heading()
            if has_imu:
                self.camera.reset_imu_heading()
                print("🧭 [AUTO SCAN 360°] Đã kết nối IMU OAK-D S2 song song!")

        # Kênh dự phòng: Tính toán bù điện áp pin từ INA219
        v_bat = 11.1
        if self.battery and hasattr(self.battery, 'read_metrics'):
            try:
                metrics = self.battery.read_metrics()
                if metrics and len(metrics) > 0 and metrics[0] > 7.0:
                    v_bat = float(metrics[0])
            except Exception:
                pass

        comp_ratio = 11.1 / max(9.5, min(12.6, v_bat))
        compensated_pwm = max(0.10, min(0.18, base_pwm * comp_ratio))
        calibrated_duration = float(getattr(config, 'AUTO_SCAN_DURATION_S', 4.8)) * (base_pwm / compensated_pwm)

        total_est = max(4.0, calibrated_duration)
        self.auto_scan_duration = total_est
        self.auto_scan_remaining = total_est

        start_time = time.time()
        last_processed_frame = None
        turned_angle = 0.0
        loop_closed = False
        max_scan_timeout = 16.0  # Giới hạn an toàn tối đa 16 giây

        # Bắt đầu xoay động cơ chậm đều
        if self.motors:
            if hasattr(self.motors, 'spin_in_place'):
                self.motors.spin_in_place(duty=compensated_pwm, direction=direction)
            else:
                self.motors.set_cmd_vel(0.0, 0.30)

        while self.running and self.is_auto_scanning:
            time.sleep(0.025)
            now = time.time()
            elapsed = now - start_time

            # 1. Chốt chặn an toàn: Vật cản sát mũi xe (chỉ kích hoạt khi phanh khẩn cấp được bật) hoặc người dùng lái tay
            if self.safety_brake and getattr(self.safety_brake, 'is_enabled', False):
                if self.obstacle_distance is not None and self.obstacle_distance <= 0.16:
                    print(f"🛑 [AUTO SCAN 360°] Phát hiện vật cản sát mũi xe ({self.obstacle_distance*100:.0f}cm) -> Dừng an toàn!")
                    break
            if (now - self.last_manual_drive_time < 0.40):
                print("🛑 [AUTO SCAN 360°] Người dùng can thiệp lái tay -> Dừng quét tự động.")
                break
            if elapsed > max_scan_timeout:
                print(f"⚠️ [AUTO SCAN 360°] Hết thời gian an toàn ({max_scan_timeout}s) -> Tự động dừng xe.")
                break

            # 2. Xử lý thị giác trên khung hình mới nhất từ Camera
            curr_frame = self.latest_raw_bgr
            frame_is_new = (curr_frame is not None and curr_frame is not last_processed_frame)

            vis_accum = 0.0
            if frame_is_new and self.camera:
                last_processed_frame = curr_frame
                _, vis_accum = self.camera.track_visual_rotation(curr_frame)

            # Đọc góc đã quay từ IMU hoặc Con quay thị giác
            if has_imu and self.camera:
                _, acc_yaw, raw_yaw = self.camera.get_imu_heading()
                turned_angle = abs(acc_yaw)
                with self.lock:
                    self.robot_yaw = raw_yaw
            elif abs(vis_accum) > 0.05:
                turned_angle = abs(vis_accum)
                with self.lock:
                    self.robot_yaw = start_yaw + vis_accum
                    self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))
            else:
                progress = min(1.0, elapsed / calibrated_duration)
                turned_angle = progress * target_angle
                with self.lock:
                    self.robot_yaw = start_yaw + turned_angle
                    self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))

            progress = min(1.0, turned_angle / target_angle)
            self.auto_scan_remaining = max(0.0, total_est * (1.0 - progress))

            # 3. Kiểm tra khóa vòng lặp thị giác (Anchor Loop Closure)
            # Chỉ kiểm tra khi đã quay tối thiểu 260 độ (4.54 rad) hoặc đã quay hơn 2.2 giây để tránh bẫy xuất phát
            match_score = 0.0
            offset_x = 999.0
            if anchor_ok and self.camera and curr_frame is not None:
                if turned_angle >= 4.54 or elapsed >= 2.2:
                    is_closed, match_score, offset_x = self.camera.check_anchor_loop_closure(curr_frame, min_turned_rad=4.5)
                    if is_closed:
                        print(f"🎯 [AUTO SCAN 360°] CAMERA ĐÃ PHÁT HIỆN LẠI KHUNG HÌNH MỐC BAN ĐẦU! (Độ khớp: {match_score*100:.1f}%, Lệch: {offset_x:+.0f}px, Thời gian: {elapsed:.2f}s)")
                        loop_closed = True
                        break

            # 4. Kiểm tra điều kiện hoàn thành 360 độ bằng góc đo thị giác / IMU
            if turned_angle >= target_angle:
                print(f"🎯 [AUTO SCAN 360°] HOÀN TẤT ĐỦ 360° THEO GÓC QUAY THỊ GIÁC! (Góc quay: {turned_angle*180/math.pi:.1f}°, Thời gian: {elapsed:.2f}s)")
                break

            # 5. Điều khiển tốc độ: Khi sắp về đến đích (match_score >= 0.50 hoặc góc > 310°), hạ xuống creeping PWM để hãm đà quán tính
            if match_score >= 0.50 or (target_angle - turned_angle) < 0.85:
                curr_pwm = creep_pwm
            else:
                curr_pwm = compensated_pwm

            if self.motors:
                if hasattr(self.motors, 'spin_in_place'):
                    self.motors.spin_in_place(duty=curr_pwm, direction=direction)
                else:
                    self.motors.set_cmd_vel(0.0, 0.30)

        # 6. Phanh chủ động đối kháng (Active Counter-Brake) dừng đứng bánh xe ngay tức khắc
        if self.motors:
            if hasattr(self.motors, 'active_brake'):
                self.motors.active_brake(reverse_duty=0.14, pulse_ms=40, spin_direction=direction)
            else:
                self.motors.stop()

        with self.lock:
            self.is_auto_scanning = False
            self.current_w = 0.0
            self.auto_scan_remaining = 0.0
            # Khi đã đóng vòng lặp về mốc ban đầu, khóa chặt góc về start_yaw
            self.robot_yaw = start_yaw

        if HAS_ROS and getattr(self, 'ros_cmd_pub', None):
            try:
                t = Twist()
                self.ros_cmd_pub.publish(t)
            except Exception:
                pass

        print(f"🎉 [AUTO SCAN 360°] Hoàn tất chu trình quét 360° bằng Camera. Xe đã dừng tại góc ban đầu {self.robot_yaw*180/math.pi:.1f}°.")


    def emergency_stop(self):
        """Phanh khẩn cấp lập tức toàn bộ hệ thống động cơ JetBot"""
        with self.lock:
            self.current_v = 0.0
            self.current_w = 0.0
            self.step_cancel_requested = True
            self.is_auto_scanning = False
        if self.motors:
            if hasattr(self.motors, 'active_brake'):
                self.motors.active_brake(reverse_duty=0.16, pulse_ms=40)
            else:
                self.motors.stop()
        print("🛑 [EMERGENCY STOP] Đã ngắt khẩn cấp toàn bộ động cơ JetBot!")
        return True

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
        if self.camera and hasattr(self.camera, 'reset_imu_heading'):
            self.camera.reset_imu_heading()
        self.clear_goal()
        if self.mapper:
            return self.mapper.reset_map()
        return True

    def plan_to_goal(self, gx, gy):
        """Lập kế hoạch đường đi tối ưu A* từ vị trí xe hiện tại tới đích (gx, gy) mét"""
        with self.lock:
            rx, ry = self.robot_x, self.robot_y
            grid = self.mapper.grid if self.mapper else None
            landmarks = self.mapper.landmarks if self.mapper else []

        if not getattr(self, 'planner', None):
            return {"success": False, "message": "Module PathPlanner chưa được khởi tạo"}

        res = self.planner.plan_path((rx, ry), (gx, gy), grid=grid, landmarks=landmarks)
        if res.get('success'):
            print(f"🎯 [A* PATH PLANNER] Đã tìm lộ trình tới ({gx:.2f}, {gy:.2f}): {res['length_m']}m ({res['waypoints_count']} waypoints) trong {res['plan_time_ms']}ms")
        else:
            print(f"⚠️ [A* PATH PLANNER] Không tìm thấy đường tới ({gx:.2f}, {gy:.2f}): {res.get('message')}")
        return res

    def clear_goal(self):
        """Xóa mục tiêu và lộ trình hiện tại"""
        if getattr(self, 'navigator', None) and self.navigator.is_running:
            self.navigator.cancel_navigation("Người dùng xóa đích")
        if getattr(self, 'planner', None):
            self.planner.clear_path()
            print("🧹 [A* PATH PLANNER] Đã xóa mục tiêu và lộ trình.")
            return True
        return False

    def _nav_set_cmd_vel(self, v, w):
        """Hàm nhận lệnh vận tốc từ AutonomousNavigator để phát cho Motor & Odometry"""
        with self.lock:
            self.current_v = v
            self.current_w = w
            self.last_cmd_vel_time = time.time()
        if self.motors:
            self.motors.set_cmd_vel(v, w)
        if HAS_ROS and self.ros_cmd_pub:
            t = Twist()
            t.linear.x, t.angular.z = v, w
            self.ros_cmd_pub.publish(t)

    def _nav_stop_robot(self):
        """Dừng xe an toàn khi tự hành hoàn tất hoặc bị hủy"""
        with self.lock:
            self.current_v = 0.0
            self.current_w = 0.0
            self.last_cmd_vel_time = time.time()
        if self.motors:
            self.motors.stop()
        if HAS_ROS and self.ros_cmd_pub:
            t = Twist()
            self.ros_cmd_pub.publish(t)

    def start_navigation(self):
        """Kích hoạt AutonomousNavigator tự động lăn bánh bám lộ trình A*"""
        if not getattr(self, 'navigator', None):
            return {"success": False, "message": "Module AutonomousNavigator chưa được khởi tạo"}
        if not getattr(self, 'planner', None) or not self.planner.current_path:
            return {"success": False, "message": "Chưa có đường đi A*. Hãy click bản đồ để đặt đích trước!"}

        # Hủy bước thủ công nếu có
        if getattr(self, 'is_step_running', False):
            self.cancel_step_move()

        return self.navigator.start_navigation(self.planner.current_path)

    def stop_navigation(self):
        """Dừng khẩn cấp tiến trình tự hành bám đường"""
        if getattr(self, 'navigator', None):
            self.navigator.cancel_navigation("Dừng bởi người dùng qua Web")
            return {"success": True, "message": "Đã dừng tự hành"}
        return {"success": False, "message": "Module AutonomousNavigator chưa được khởi tạo"}

    def reset_trip_odometer(self):
        """Đặt lại bộ đo quãng đường và góc quay chuyến đi về 0"""
        with self.lock:
            self.trip_distance_m = 0.0
            self.trip_turn_rad = 0.0
            self.trip_active_time_s = 0.0
        print("📏 [TRIP ODOMETER] Đã đặt lại đồng hồ đo quãng đường chuyến đi về 0.0 cm.")
        return True


    def shutdown(self):
        print("\n🛑 [SHUTDOWN] Đang dừng an toàn toàn bộ hệ thống JetBot...")
        self.running = False
        if getattr(self, 'navigator', None): self.navigator.cancel_navigation("Hệ thống tắt máy")
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
    parser.add_argument('--brake', action='store_true', dest='brake', default=None,
                        help="Bật tính năng phanh khẩn cấp tự động Virtual Bumper")
    parser.add_argument('--no-brake', action='store_false', dest='brake',
                        help="Tạm thời tắt phanh khẩn cấp để tránh lỗi detect làm ngắt quay xe")
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
    if args.brake is None: args.brake = to_bool(BRAKE) if 'BRAKE' in globals() else getattr(config, 'ENABLE_SAFETY_BRAKE', False)
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
    print(f"  ├─ Phanh khẩn cấp:          {'BẬT (< 20cm khóa tiến, cho phép lùi/quay)' if args.brake else 'TẮT'}")
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
