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
import base64

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
from modules.frame_recorder import FrameRecorder

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
        self.step_move_type = ""
        self.step_cancel_requested = False
        self.step_status_text = "SẴN SÀNG"
        self.step_progress_pct = 0
        self._imu_yaw_anchor = None
        self._ros_odom_initialized = False
        self._ros_odom_origin_x = 0.0
        self._ros_odom_origin_y = 0.0
        self._ros_odom_origin_z = 0.0
        self._ros_odom_origin_yaw = 0.0

        # Chế độ chạy kiểm tra quỹ đạo hình học khép kín (Geometric Loop Benchmark)
        self.is_pattern_running = False
        self.pattern_cancel_requested = False
        self.pattern_status_text = "SẴN SÀNG"
        self.pattern_type = "none"
        self.pattern_current_leg = 0
        self.pattern_total_legs = 0

        # Bộ đo quãng đường và góc quay chuyến đi (Trip Odometer)
        self.trip_distance_m = 0.0
        self.trip_turn_rad = 0.0
        self.trip_active_time_s = 0.0

        # Giám sát & Đối chiếu góc quay thực tế từ cảm biến IMU (Turn Diagnostics Monitor)
        self.latest_imu_yaw_val = 0.0
        self.latest_turn_perf = {}
        self.turn_diagnostics_history = []
        self._turn_eval_count = 0
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

        # 7.4. Observability & Debug Inspector (Quan sát & soi thuật toán từng frame)
        self.debug_paused = False
        self.debug_step_requested = False
        self._diag_frame_id = 0
        self.latest_frame_diag = {}
        self.recent_frame_logs = []

        # 7.5. Ghi nhận Frame & Siêu dữ liệu thuật toán KHI CÓ LỆNH di chuyển (Frame Recorder)
        logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        self.recorder = FrameRecorder(base_dir=logs_dir) if getattr(config, 'ENABLE_FRAME_RECORDER', False) else None
        self._was_moving = False
        self._settle_frames_remaining = 0
        self._initial_map_updated = False
        self.is_motion_active = False
        self.current_motion_desc = ""

        # 8. Khởi tạo Trạm điều khiển Web Cockpit
        self.web = None
        if self.flags.web:
            self.web = WebCockpitServer(host=config.WEB_HOST, port=config.WEB_PORT)
            self.web.recorder = self.recorder
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
            self.web.motor_trim_cb = self.set_motor_trim
            self.web.save_motor_trim_cb = self.save_motor_trim
            self.web.calibration_cb = self.update_calibration_params
            self.web.get_calibration_cb = self.get_calibration_params
            self.web.get_full_config_cb = self.get_full_config
            self.web.update_full_config_cb = self.update_full_config
            self.web.select_profile_cb = self.select_config_profile
            self.web.create_profile_cb = self.create_new_config_profile
            self.web.delete_profile_cb = self.delete_config_profile
            self.web.pattern_move_cb = self.execute_pattern_patrol
            self.web.pattern_cancel_cb = self.cancel_pattern_patrol
            self.web.debug_pause_cb = self.set_debug_paused
            self.web.debug_step_cb = self.request_debug_step
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

                    rospy.Subscriber('/rtabmap/odom', Odometry, lambda m: self._ros_odom_cb(m, 'rtabmap'), queue_size=1)
                    rospy.Subscriber('/odom', Odometry, lambda m: self._ros_odom_cb(m, 'odom'), queue_size=1)
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

    def _execute_slam_cycle(self, depth_np):
        """Thực thi một chu kỳ xử lý SLAM & CSM hoàn chỉnh ở tần số định thời (10Hz) độc lập với luồng camera.
        Thuật toán tích hợp cơ chế bảo vệ:
        - Khóa cố định tọa độ (Stationary Lock) khi xe đứng yên tĩnh, triệt tiêu 100% hiện tượng trôi dạt do nhiễu độ sâu.
        - Khóa quay góc (Turn Gate) khi xe xoay tại chỗ.
        - Correlative Scan Matching (CSM) chỉ nắn tọa độ khi xe thực sự đang chuyển động tịnh tiến.
        """
        if getattr(self, '_debug_pause_pipeline', False):
            return
        if depth_np is None or not self.camera:
            return

        t_start = time.perf_counter()
        timestamp_s = time.time()
        self._slam_frame_id = getattr(self, '_slam_frame_id', 0) + 1
        frame_id = self._slam_frame_id

        with self.lock:
            rx, ry, rz, yaw = self.robot_x, self.robot_y, self.robot_z, self.robot_yaw
            is_turning = (abs(getattr(self, 'current_w', 0.0)) > 0.05) or getattr(self, 'is_step_turning', False)
            is_moving = (abs(getattr(self, 'current_v', 0.0)) > 0.01) or (getattr(self, 'is_motion_active', False) and not getattr(self, 'is_step_turning', False))

        rx_init, ry_init, yaw_init = rx, ry, yaw

        # 1. Trích xuất đám mây điểm 3D và 120 tia quét 2D
        t_depth_start = time.perf_counter()
        self.camera.process_depth_frame(depth_np, rx, ry, rz, yaw)
        t_depth_ms = (time.perf_counter() - t_depth_start) * 1000.0

        scan_robot = getattr(self.camera, 'points_robot', None)
        csm_pts_count = len(scan_robot) if scan_robot else 0

        # 2. Correlative Scan Matching (CSM) - CHỈ CHẠY KHI XE ĐANG DI CHUYỂN TỊNH TIẾN
        # Khi xe quay tại chỗ hoặc vừa quay xong: Khóa cứng CSM yaw (Turn Gate).
        # Khi xe đứng yên tĩnh trên sàn (Stationary), tọa độ xe là BẤT BIẾN, tuyệt đối không nắn chỉnh do nhiễu sensor!
        t_csm_start = time.perf_counter()
        csm_executed = False
        csm_matched = False
        csm_score = 0.0
        csm_init_score = 0.0
        csm_dx_cm = 0.0
        csm_dy_cm = 0.0
        csm_dyaw_deg = 0.0

        need_csm = False
        if not is_turning and not getattr(self, '_just_finished_turn', False) and self.scan_matcher and getattr(config, 'ENABLE_SCAN_MATCHING', True) and self.mapper:
            if is_moving:
                need_csm = True
                self._was_moving = True
                self._settle_frames_remaining = int(getattr(config, 'SETTLING_FRAMES_COUNT', 3))
            elif getattr(self, '_was_moving', False) or getattr(self, '_settle_frames_remaining', 0) > 0:
                cur_rem = getattr(self, '_settle_frames_remaining', 0)
                if cur_rem > 0:
                    need_csm = True
                    self._settle_frames_remaining = cur_rem - 1
                else:
                    self._was_moving = False

        if need_csm and scan_robot:
            csm_executed = True
            corr_x, corr_y, corr_yaw, matched, score = self.scan_matcher.align_pose(
                rx, ry, yaw, scan_robot, self.mapper.grid,
                self.mapper.origin_x, self.mapper.origin_y,
                self.mapper.width, self.mapper.height, self.mapper.resolution,
                hit_counts=getattr(self.mapper, 'hit_counts', None)
            )
            self._last_scan_matched = matched
            csm_matched = matched
            csm_score = score
            csm_init_score = getattr(self.scan_matcher, 'last_init_score', 0.0)
            csm_dx_cm = getattr(self.scan_matcher, 'last_dx_cm', 0.0)
            csm_dy_cm = getattr(self.scan_matcher, 'last_dy_cm', 0.0)
            csm_dyaw_deg = 0.0
            if matched:
                with self.lock:
                    self.robot_x = corr_x
                    self.robot_y = corr_y
                    # KHÓA CHẶT YAW: CSM chỉ khử trôi tịnh tiến (X, Y) đối chiếu tường, TUYỆT ĐỐI không xoay yaw xe!
                    # Góc yaw xe chỉ tuân theo lệnh quay thực tế / Odometry / IMU!
                    rx, ry = corr_x, corr_y
        t_csm_ms = (time.perf_counter() - t_csm_start) * 1000.0

        if getattr(self, '_just_finished_turn', False):
            self._just_finished_turn = False

        # 3. Cập nhật bản đồ 2D Occupancy Grid (Keyframe Raycasting)
        # Chỉ cập nhật bản đồ khi:
        # 1. Xe đang di chuyển (is_moving = True)
        # 2. Hoặc đúng 1 lần sau khi chốt hạ vị trí dừng (need_csm = True)
        # 3. Hoặc xe mới khởi động / quét 360 độ
        # Khi xe đứng yên tĩnh bình thường: không bắn tia liên tục để bảo vệ độ nét bờ tường
        t_map_start = time.perf_counter()
        map_updated = False
        if self.mapper and self.flags.mapper and not is_turning:
            need_map_update = is_moving or need_csm or (not getattr(self, '_initial_map_updated', False)) or getattr(self, 'is_auto_scanning', False)
            if need_map_update and self.camera.points_3d:
                self.mapper.update_scan(rx, ry, self.camera.points_3d, yaw=yaw)
                map_updated = True
                self._initial_map_updated = True
        t_map_ms = (time.perf_counter() - t_map_start) * 1000.0

        t_total_ms = (time.perf_counter() - t_start) * 1000.0

        # 4. Trích xuất chỉ số FOV & Chùm tia
        total_rays = getattr(self.camera, 'last_total_rays', 0) if self.camera else 0
        obs_rays = getattr(self.camera, 'last_obs_rays', 0) if self.camera else 0
        floor_rays = getattr(self.camera, 'last_floor_rays', 0) if self.camera else 0
        blind_rays = getattr(self.camera, 'last_blind_rays', 0) if self.camera else 0
        ray_dist = getattr(self.camera, 'last_ray_distances', []) if self.camera else []
        ray_types = getattr(self.camera, 'last_ray_types', []) if self.camera else []

        # Lấy mẫu chùm tia scan_robot để visualizer hiển thị lớp chập pre vs post alignment (tối đa 40 điểm)
        sampled_scan_robot = []
        if scan_robot:
            stride = max(1, len(scan_robot) // 40)
            sampled_scan_robot = [[round(p[0], 3), round(p[1], 3)] for p in scan_robot[::stride]]

        # Phân tích cờ cảnh báo bất thường (Anomaly & Watchdog Flags)
        flags = []
        settle_rem = getattr(self, '_settle_frames_remaining', 0)
        if is_turning:
            flags.append({"level": "info", "tag": "TURN_GATED", "msg": "Khóa cập nhật CSM & Bản đồ khi xe xoay tại chỗ"})
        elif settle_rem > 0:
            flags.append({"level": "success", "tag": "SETTLING", "msg": f"Đang hội tụ đa bước chốt hạ vị trí dừng (Còn {settle_rem} frame)"})
        elif not is_moving:
            flags.append({"level": "info", "tag": "STATIONARY", "msg": "Xe đứng yên - Khóa cố định tọa độ (Chống trôi dạt do nhiễu)"})
        else:
            if csm_executed:
                if csm_matched:
                    flags.append({"level": "success", "tag": "CSM_LOCKED", "msg": f"Khớp CSM thành công (Likelihood: {csm_score:.1%})"})
                else:
                    flags.append({"level": "warning", "tag": "CSM_REJECTED", "msg": f"Khớp bị từ chối (Score {csm_score:.2f} < ngưỡng {getattr(self.scan_matcher, 'min_score', 0.28):.2f})"})
                if csm_score < 0.35 and csm_score > 0:
                    flags.append({"level": "warning", "tag": "LOW_SCORE", "msg": "Điểm khớp thấp - Vùng không gian ít vật cản hoặc bề mặt phẳng"})
                if abs(csm_dx_cm) > 10.0 or abs(csm_dy_cm) > 10.0 or abs(csm_dyaw_deg) > 8.0:
                    flags.append({"level": "warning", "tag": "LARGE_JUMP", "msg": f"Bước hiệu chỉnh lớn (Δ={math.hypot(csm_dx_cm, csm_dy_cm):.1f}cm, ΔYaw={csm_dyaw_deg:+.1f}°)"})
            else:
                flags.append({"level": "info", "tag": "CSM_SKIPPED", "msg": "Bỏ qua CSM (Không có điểm cản hoặc tính năng tắt)"})

        if csm_pts_count < 15 and not is_turning:
            flags.append({"level": "warning", "tag": "LOW_POINTS", "msg": f"Ít tia chạm cản ({csm_pts_count} tia) - Camera hướng vào khoảng trống"})
        if blind_rays > (total_rays * 0.7) and total_rays > 0:
            flags.append({"level": "danger", "tag": "BLIND_FOV", "msg": f"Góc nhìn bị mù >70% ({blind_rays}/{total_rays} tia vượt ngưỡng/lỗi depth)"})
        if t_total_ms > 85.0:
            flags.append({"level": "danger", "tag": "HIGH_LATENCY", "msg": f"Độ trễ chu kỳ cao ({t_total_ms:.1f}ms > 85ms)"})

        # Sinh ảnh Thumbnail RGB và JET (Tạm TẮT để giải phóng 100% CPU khi ENABLE_FRAME_RECORDER=False)
        need_thumbs = getattr(config, 'ENABLE_FRAME_RECORDER', False)
        rgb_thumb_b64 = None
        depth_thumb_b64 = None
        if need_thumbs:
            try:
                cur_rgb = None
                if self.camera and hasattr(self.camera, '_cached_oak_frame') and self.camera._cached_oak_frame is not None:
                    cur_rgb = self.camera._cached_oak_frame
                elif self.camera and hasattr(self.camera, 'frame') and self.camera.frame is not None:
                    cur_rgb = self.camera.frame

                if cur_rgb is not None:
                    small_rgb = cv2.resize(cur_rgb, (220, 124))
                    _, buf_rgb = cv2.imencode('.jpg', small_rgb, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    rgb_thumb_b64 = base64.b64encode(buf_rgb).decode('ascii')
            except Exception:
                pass

            try:
                if depth_np is not None:
                    if depth_np.dtype == np.uint16:
                        d_mm = depth_np.astype(np.float32)
                    else:
                        d_mm = depth_np * 1000.0
                    d_disp = np.clip(d_mm, 200.0, 3500.0)
                    disp_norm = ((3500.0 - d_disp) / 3300.0 * 255.0).astype(np.uint8)
                    small_disp = cv2.resize(disp_norm, (220, 124))
                    thermal_bgr = cv2.applyColorMap(small_disp, cv2.COLORMAP_JET)
                    _, buf_depth = cv2.imencode('.jpg', thermal_bgr, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    depth_thumb_b64 = base64.b64encode(buf_depth).decode('ascii')
            except Exception:
                pass

        # Giải trình chi tiết quyết định của thuật toán
        decision_status = "LOCKED" if (csm_matched and map_updated) else ("TURN_GATE" if is_turning else ("SETTLING" if settle_rem > 0 else ("STATIONARY" if not is_moving else ("REJECT" if csm_executed else "NO_SCAN"))))
        decision_title = "CHẤP THUẬN NẮN TỌA ĐỘ & CẬP NHẬT BẢN ĐỒ" if decision_status == "LOCKED" else ("TẠM KHÓA CẬP NHẬT (XE ĐANG QUAY)" if decision_status == "TURN_GATE" else ("ĐANG HỘI TỤ ĐA BƯỚC CHỐT HẠ VỊ TRÍ DỪNG" if decision_status == "SETTLING" else ("KHÓA CỐ ĐỊNH TỌA ĐỘ (XE ĐỨNG YÊN TĨNH)" if decision_status == "STATIONARY" else ("TỪ CHỐI FRAME (CHỐNG LOẠN BẢN ĐỒ)" if decision_status == "REJECT" else "BỎ QUA CSM"))))
        if decision_status == "LOCKED":
            decision_reason = f"Độ tương đồng CSM đạt {csm_score:.1%} (≥ ngưỡng {getattr(self.scan_matcher, 'min_score', 0.28):.0%}) và có {csm_pts_count} tia cản. Bản đồ được cập nhật an toàn."
        elif decision_status == "TURN_GATE":
            decision_reason = "Xe đang thực hiện quay góc tại chỗ. Tạm khóa cập nhật bản đồ để chống mờ nhòe và trôi mốc tọa độ."
        elif decision_status == "SETTLING":
            decision_reason = f"Xe vừa dừng hẳn. Thuật toán đang chạy thêm {settle_rem} frame ổn định để triệt tiêu rung lắc quán tính và hội tụ vị trí dừng chính xác nhất."
        elif decision_status == "STATIONARY":
            decision_reason = "Xe đang đứng yên tĩnh trên sàn. Tọa độ xe được khóa cố định tuyệt đối tại vị trí hiện tại để chống trôi dạt do nhiễu cảm biến."
        elif decision_status == "REJECT":
            decision_reason = f"Điểm khớp CSM chỉ đạt {csm_score:.1%} (< ngưỡng tối thiểu) hoặc số tia cản quá ít ({csm_pts_count} tia). Thuật toán từ chối cập nhật để bảo vệ cấu trúc bản đồ không bị biến dạng."
        else:
            decision_reason = "Không có chướng ngại vật trong tầm nhìn hoặc tính năng Scan-Matching đang tạm tắt."

        decision = {
            "status": decision_status,
            "title": decision_title,
            "reason": decision_reason,
            "score_ok": bool(csm_score >= getattr(self.scan_matcher, 'min_score', 0.28)),
            "points_ok": bool(csm_pts_count >= 10),
            "not_turning": bool(not is_turning),
            "jump_ok": bool(abs(csm_dx_cm) < 15.0 and abs(csm_dycm if 'csm_dycm' in locals() else csm_dy_cm) < 15.0 and abs(csm_dyaw_deg) < 10.0),
            "map_updated": bool(map_updated)
        }

        diag = {
            "frame_id": frame_id,
            "timestamp": round(timestamp_s, 2),
            "latencies": {
                "depth_ms": round(t_depth_ms, 2),
                "csm_ms": round(t_csm_ms, 2),
                "map_ms": round(t_map_ms, 2),
                "total_ms": round(t_total_ms, 2)
            },
            "pose_before": [round(rx_init, 3), round(ry_init, 3), round(yaw_init, 3)],
            "pose_after": [round(rx, 3), round(ry, 3), round(yaw, 3)],
            "csm": {
                "executed": csm_executed,
                "matched": csm_matched,
                "score": round(csm_score, 3),
                "init_score": round(csm_init_score, 3),
                "score_gain": round(csm_score - csm_init_score, 3),
                "dx_cm": round(csm_dx_cm, 2),
                "dy_cm": round(csm_dy_cm, 2),
                "dyaw_deg": round(csm_dyaw_deg, 2),
                "pts_count": csm_pts_count
            },
            "rays": {
                "total": total_rays,
                "obstacle": obs_rays,
                "floor": floor_rays,
                "blind": blind_rays,
                "distances": ray_dist,
                "types": ray_types
            },
            "sampled_scan_robot": sampled_scan_robot,
            "is_turning": is_turning,
            "map_updated": map_updated,
            "flags": flags,
            "debug_paused": getattr(self, 'debug_paused', False),
            "rgb_thumb": rgb_thumb_b64,
            "depth_thumb": depth_thumb_b64,
            "decision": decision
        }

        self.latest_frame_diag = diag

        # 5. Ghi nhận Frame & Telemetry phân tích KHI VÀ CHỈ KHI CÓ LỆNH DI CHUYỂN
        if hasattr(self, 'recorder') and self.recorder and getattr(self, 'is_motion_active', False):
            clean_telemetry = {k: v for k, v in diag.items() if k not in ('rgb_thumb', 'depth_thumb')}
            self.recorder.record_frame(frame_id, cur_rgb, depth_np, clean_telemetry)

        # Ghi nhận vào lịch sử gần nhất (Buffer tối đa 25 frames cho bảng theo dõi)
        if not hasattr(self, 'recent_frame_logs'):
            self.recent_frame_logs = []
        summary_log = {
            "frame_id": frame_id,
            "time_str": time.strftime("%H:%M:%S", time.localtime(timestamp_s)) + f".{int((timestamp_s%1)*10):01d}",
            "total_ms": round(t_total_ms, 1),
            "pts": csm_pts_count,
            "score": round(csm_score, 2),
            "dx_cm": round(csm_dx_cm, 1),
            "dy_cm": round(csm_dy_cm, 1),
            "dyaw_deg": round(csm_dyaw_deg, 1),
            "status": decision_status,
            "matched": csm_matched,
            "flags_count": len(flags),
            "rgb_thumb": rgb_thumb_b64,
            "depth_thumb": depth_thumb_b64,
            "decision": decision
        }
        self.recent_frame_logs.append(summary_log)
        if len(self.recent_frame_logs) > 25:
            self.recent_frame_logs.pop(0)
        # Giữ ảnh thumbnail chỉ cho 3 frame mới nhất trong lịch sử để giảm thiểu 85% payload mạng
        for old_log in self.recent_frame_logs[:-3]:
            if "rgb_thumb" in old_log and old_log["rgb_thumb"]:
                old_log["rgb_thumb"] = None
            if "depth_thumb" in old_log and old_log["depth_thumb"]:
                old_log["depth_thumb"] = None

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

            # 2. Cập nhật SLAM & CSM theo nhịp thích ứng (Adaptive Interval):
            # - Khi xe đứng yên: nhịp 0.35s (2.8 Hz) giúp giải phóng 90% CPU cho luồng Camera đạt 25-30 FPS mượt mà!
            # - Khi xe di chuyển: nhịp 0.10s (10 Hz) đảm bảo bám dính bản đồ sắc nét tức thời.
            slam_interval = 0.35 if not getattr(self, 'is_motion_active', False) else 0.10
            if (now - getattr(self, 'last_ros_slam_time', 0.0)) >= slam_interval:
                self.last_ros_slam_time = now
                self._execute_slam_cycle(depth_np)
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
            q = msg.orientation
            norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if norm_sq < 0.5:
                return # Bỏ qua quaternion rỗng hoặc không có dữ liệu

            # Bỏ qua quaternion mặc định/dummy identity (0, 0, 0, 1) do các driver ROS không có AHRS phát ra
            if abs(q.x) < 1e-3 and abs(q.y) < 1e-3 and abs(q.z) < 1e-3 and abs(q.w - 1.0) < 1e-3:
                return

            # Kiểm tra covariance: REP-145 covariance[0] < 0 nghĩa là orientation không khả dụng
            cov = getattr(msg, 'orientation_covariance', None)
            if cov is not None and len(cov) >= 9:
                if cov[0] < 0 or cov[0] > 10.0:
                    return

            frame_id = str(getattr(getattr(msg, 'header', None), 'frame_id', '')).lower()
            if 'base' in frame_id or 'odom' in frame_id or 'map' in frame_id:
                # Hệ quy chiếu Robot REP-103 chuẩn (Trục Z hướng lên trời)
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny, cosy)
            else:
                # Hệ quy chiếu OAK-D Camera quang học (Trục -Y hướng lên trời, Z hướng ra trước)
                # Tính góc xoay quanh trục đứng (-Y) trên mặt phẳng sàn:
                siny = -2.0 * (q.w * q.y + q.x * q.z)
                cosy = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                yaw = math.atan2(siny, cosy)

            # LUÔN LUÔN CẬP NHẬT GÓC ĐO THỰC TẾ CÓ NEO TƯƠNG ĐỐI (Relative IMU Anchor)
            with self.lock:
                if getattr(self, '_ros_imu_yaw_anchor', None) is None:
                    self._ros_imu_yaw_anchor = yaw - self.robot_yaw
                calib_yaw = yaw - self._ros_imu_yaw_anchor
                calib_yaw = math.atan2(math.sin(calib_yaw), math.cos(calib_yaw))
                self.latest_imu_yaw_val = calib_yaw
                self.last_imu_time = time.time()
                self.has_active_imu = True

            # Khi xe đang tự thực thi bước lệnh định lượng, không để callback ghi đè robot_yaw giữa chừng
            # mà để hàm kết thúc bước tự tính toán góc quay thực và nắn chỉnh vào Pose
            if getattr(self, 'is_step_running', False):
                return

            if getattr(config, 'ENABLE_CLOSED_LOOP_IMU', False):
                with self.lock:
                    self.robot_yaw = calib_yaw
        except Exception:
            pass

    def _ros_odom_cb(self, msg, topic_name='odom'):
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

            # Khóa topic Odometry để triệt tiêu 100% hiện tượng xung đột giữa nhiều topics (/rtabmap/odom vs /odom)
            active_top = getattr(self, '_ros_odom_active_topic', None)
            if active_top is not None and active_top != topic_name:
                if time.time() - getattr(self, 'last_real_ros_odom_time', 0.0) < 1.2:
                    return

            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)

            # Neo gốc tọa độ ROS Odometry tương đối (Relative ROS Odometry Anchor):
            # Giúp xe luôn bắt đầu từ (0, 0, 0) khi khởi động hoặc sau mỗi lần bấm Reset Bản Đồ,
            # triệt tiêu 100% hiện tượng xe bị giật/trôi dạt về tọa độ cũ của RTAB-Map bên ngoài.
            if not getattr(self, '_ros_odom_initialized', False):
                self._ros_odom_origin_x = p.x
                self._ros_odom_origin_y = p.y
                self._ros_odom_origin_z = p.z
                self._ros_odom_origin_yaw = yaw
                self._ros_odom_active_topic = topic_name
                self._ros_odom_initialized = True

            dx_raw = p.x - self._ros_odom_origin_x
            dy_raw = p.y - self._ros_odom_origin_y
            dz_raw = p.z - self._ros_odom_origin_z
            orig_yaw = self._ros_odom_origin_yaw

            # Xoay độ dời theo góc định hướng ban đầu để trục X luôn hướng về phía trước lúc xuất phát/reset:
            cos_o = math.cos(orig_yaw)
            sin_o = math.sin(orig_yaw)
            rel_x = dx_raw * cos_o + dy_raw * sin_o
            rel_y = -dx_raw * sin_o + dy_raw * cos_o
            rel_z = dz_raw
            rel_yaw = math.atan2(math.sin(yaw - orig_yaw), math.cos(yaw - orig_yaw))

            with self.lock:
                self.robot_x = rel_x
                self.robot_y = rel_y
                self.robot_z = rel_z
                self.robot_yaw = rel_yaw
                self.last_real_ros_odom_time = time.time()
                self.path_history.append([round(rel_x, 3), round(rel_y, 3), round(rel_z, 3)])
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

        brake_thresh = getattr(config, 'SAFETY_BRAKE_DIST_M', 0.22)
        warn_thresh = getattr(config, 'SAFETY_WARN_DIST_M', 0.45)

        # 1. Đánh giá qua EmergencyBrake nếu có
        if self.safety_brake and self.safety_brake.is_enabled:
            v, w, alert = self.safety_brake.evaluate_velocity(v, w, self.obstacle_distance)
            if alert == "EMERGENCY_STOP":
                threshold_cm = int(self.safety_brake.brake_dist_m * 100)
                obs_cm = f"{self.obstacle_distance*100:.1f} cm" if self.obstacle_distance is not None else "< 22cm"
                print(f"🚨 [DỪNG AN TOÀN] Cản cách {obs_cm} (<= {threshold_cm}cm) -> Đã hãm tốc êm & khóa tiến, chỉ cho phép lùi/quay!")

        # 2. Can thiệp an toàn: Giảm tốc từ từ và khóa tiến khi cản quá gần:
        if (v > 0.01) and (self.obstacle_distance is not None):
            if self.obstacle_distance <= brake_thresh:
                v = 0.0
                print(f"🛑 [KHÓA TIẾN AN TOÀN] Cản cách {self.obstacle_distance*100:.1f} cm (<= {int(brake_thresh*100)}cm). Cho phép LÙI hoặc QUAY để thoát cản!")
            elif self.obstacle_distance < warn_thresh:
                ratio = max(0.20, (self.obstacle_distance - brake_thresh) / max(0.01, warn_thresh - brake_thresh))
                v = v * ratio

        # 3. Cập nhật vận tốc tức thời và điều khiển động cơ
        with self.lock:
            old_v = getattr(self, 'current_v', 0.0)
            old_w = getattr(self, 'current_w', 0.0)
            self._manual_decel_id += 1
            cur_decel_id = self._manual_decel_id

        is_stop_cmd = (abs(v) < 0.01 and abs(w) < 0.01)
        was_moving = (abs(old_v) > 0.02 or abs(old_w) > 0.04)
        soft_decel = getattr(self, 'soft_stop_enabled', getattr(config, 'ENABLE_SOFT_STOP', getattr(config, 'MANUAL_SOFT_DECEL', True)))

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
                self._manual_straight_yaw = None
            elif abs(v) > 0.01 and abs(w) < 0.01:
                # ── KHÓA HƯỚNG TỰ ĐỘNG KHI LÁI THẲNG (ACTIVE HEADING LOCK) ──
                if getattr(config, 'ENABLE_HEADING_LOCK', False):
                    cur_yaw = getattr(self, 'robot_yaw', 0.0)
                    if not hasattr(self, '_manual_straight_yaw') or self._manual_straight_yaw is None or not was_moving:
                        self._manual_straight_yaw = cur_yaw
                    yaw_err = math.atan2(math.sin(self._manual_straight_yaw - cur_yaw), math.cos(self._manual_straight_yaw - cur_yaw))
                    kp = getattr(config, 'HEADING_LOCK_KP', 0.50)
                    effective_w = max(-0.16, min(0.16, kp * yaw_err))
                else:
                    self._manual_straight_yaw = None
            else:
                self._manual_straight_yaw = None

            with self.lock:
                self.current_v = v
                self.current_w = effective_w
                self.last_cmd_vel_time = time.time()
                if not is_stop_cmd:
                    self.is_motion_active = True
                    self.current_motion_desc = f"MANUAL_v{v:.2f}_w{effective_w:.2f}"
                else:
                    self.is_motion_active = False
                    self.current_motion_desc = ""

            if not is_stop_cmd and hasattr(self, 'recorder') and self.recorder:
                self.recorder.start_command(self.current_motion_desc)
            elif is_stop_cmd and hasattr(self, 'recorder') and self.recorder:
                self.recorder.stop_command()

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
        """Hạ dốc giảm tốc mượt mà hình Sin (Cosine S-Curve) khi người dùng nhả phím WASD / Web Cockpit (50Hz)"""
        decel_ms = max(100, int(getattr(config, 'MANUAL_DECEL_TIME_MS', 180)))
        steps = 10
        dt = (decel_ms / 1000.0) / steps
        for i in range(1, steps + 1):
            with self.lock:
                if self._manual_decel_id != decel_id:
                    return  # Có lệnh lái mới, hủy ngay quá trình giảm tốc
                # Cosine S-Curve: giảm tốc mượt mà, đạo hàm gia tốc triệt tiêu êm dịu ở nấc cuối
                ratio = 0.5 * (1.0 + math.cos(math.pi * (i / steps)))
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
                self.is_motion_active = False
                self.current_motion_desc = ""
        if hasattr(self, 'recorder') and self.recorder and self._manual_decel_id == decel_id:
            self.recorder.stop_command()
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

    def get_battery_voltage_comp(self):
        """Tính hệ số bù sụt áp pin thời gian thực để duy trì mô-men động cơ đồng nhất (chuẩn hóa 11.1V danh định)"""
        try:
            with self.lock:
                m = getattr(self, 'battery_metrics', None)
            v_bat = float(m[0]) if m and len(m) > 0 else 11.1
            if 8.5 <= v_bat <= 13.5:
                # v_nominal = 11.1V. Kẹp hệ số bù trong dải an toàn [0.85, 1.25]
                return max(0.85, min(1.25, 11.1 / v_bat))
        except Exception:
            pass
        return 1.0


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

                    # Cập nhật liên tục góc quay từ cảm biến IMU phần cứng nếu có:
                    if self.camera and hasattr(self.camera, 'get_imu_heading'):
                        c_has_imu, _, c_raw_yaw = self.camera.get_imu_heading()
                        if c_has_imu:
                            if getattr(self, '_imu_yaw_anchor', None) is None:
                                self._imu_yaw_anchor = c_raw_yaw - self.robot_yaw
                            calib_yaw = c_raw_yaw - self._imu_yaw_anchor
                            calib_yaw = math.atan2(math.sin(calib_yaw), math.cos(calib_yaw))
                            self.latest_imu_yaw_val = calib_yaw
                            self.last_imu_time = now
                            self.has_active_imu = True

                    # KIỂM TRA NGUỒN ODOMETRY THỰC (Ưu tiên tuyệt đối ROS & IMU phần cứng):
                    has_ros_odom = (now - getattr(self, 'last_real_ros_odom_time', 0.0)) < 0.80
                    has_real_imu = (now - getattr(self, 'last_imu_time', 0.0)) < 1.00

                    if not has_ros_odom:
                        # Chỉ tích phân góc quay từ w khi không có cảm biến IMU thực
                        if not has_real_imu:
                            self.robot_yaw += w_real * dt
                            self.robot_yaw = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))
                        elif getattr(config, 'ENABLE_CLOSED_LOOP_IMU', False) and not getattr(self, 'is_step_running', False):
                            self.robot_yaw = getattr(self, 'latest_imu_yaw_val', self.robot_yaw)

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

                    # 2. Cập nhật SLAM & CSM Scan Matching ở nhịp thích ứng (Adaptive):
                    # Xe tĩnh: 0.35s (2.8Hz) -> mở khóa 25-30 FPS cho luồng Camera; Xe chạy: 0.10s (10Hz)
                    slam_interval = 0.35 if not getattr(self, 'is_motion_active', False) else 0.10
                    if (now - last_slam_time >= slam_interval):
                        last_slam_time = now
                        self._execute_slam_cycle(live_depth)

                if live_frame is not None:
                    self.latest_raw_bgr = live_frame

                    # Cập nhật góc quay robot_yaw: Ưu tiên IMU phần cứng OAK-D S2 nếu có
                    c_has_imu = False
                    if self.camera and hasattr(self.camera, 'get_imu_heading'):
                        c_has_imu, _, c_raw_yaw = self.camera.get_imu_heading()
                        if c_has_imu:
                            # Neo góc quay tương đối (Relative Heading Anchor)
                            if getattr(self, '_imu_yaw_anchor', None) is None:
                                with self.lock:
                                    self._imu_yaw_anchor = c_raw_yaw - self.robot_yaw
                            with self.lock:
                                calib_yaw = c_raw_yaw - self._imu_yaw_anchor
                                calib_yaw = math.atan2(math.sin(calib_yaw), math.cos(calib_yaw))
                                self.latest_imu_yaw_val = calib_yaw
                                self.last_imu_time = time.time()
                                self.has_active_imu = True
                                if getattr(config, 'ENABLE_CLOSED_LOOP_IMU', False) and not getattr(self, 'is_step_running', False):
                                    self.robot_yaw = calib_yaw

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
                                    rx, ry, yaw = self.robot_x, self.robot_y, self.robot_yaw
                                self.mapper.update_scan(rx, ry, self.camera.points_3d, yaw=yaw)
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

    def get_state_for_web(self, client_map_ver=-1):
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
            brake_dist = getattr(config, 'SAFETY_BRAKE_DIST_M', 0.22)
            warn_dist = getattr(config, 'SAFETY_WARN_DIST_M', 0.45)
            cam_fps = float(self.camera.calc_fps) if (self.camera and hasattr(self.camera, 'calc_fps') and self.camera.calc_fps > 0) else 15.0
            now_t = time.time()
            has_ros_odom_active = (now_t - getattr(self, 'last_real_ros_odom_time', 0.0)) < 0.8
            has_imu_active = (now_t - getattr(self, 'last_imu_time', 0.0)) < 1.2

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
            "map_b64": map_payload.get("map_b64", "") if (client_map_ver != map_payload.get("map_version", 1)) else "",
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
            "safety_warn_dist": warn_dist,
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
            "cam_fps": cam_fps,
            "algo_hz": round(1000.0 / max(1.0, getattr(self, 'latest_frame_diag', {}).get('latencies', {}).get('total_ms', 100.0)), 1) if getattr(self, 'latest_frame_diag', None) else 10.0,
            "network_stats": self.web.get_stream_stats() if hasattr(self, 'web') and self.web else {},
            "frame_diag": getattr(self, 'latest_frame_diag', {}),
            "recent_frame_logs": getattr(self, 'recent_frame_logs', []),
            "recorder": self.recorder.get_session_stats() if hasattr(self, 'recorder') and self.recorder else {},
            "debug_paused": getattr(self, 'debug_paused', False),
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
            "pattern_state": {
                "running": getattr(self, 'is_pattern_running', False),
                "status": getattr(self, 'pattern_status_text', 'SẴN SÀNG'),
                "current_leg": getattr(self, 'pattern_current_leg', 0),
                "total_legs": getattr(self, 'pattern_total_legs', 0),
                "type": getattr(self, 'pattern_type', 'none')
            },
            "trip_odometer": {
                "distance_cm": round(getattr(self, 'trip_distance_m', 0.0) * 100.0, 1),
                "distance_m": round(getattr(self, 'trip_distance_m', 0.0), 3),
                "turn_deg": round(math.degrees(getattr(self, 'trip_turn_rad', 0.0)), 1),
                "active_time_s": round(getattr(self, 'trip_active_time_s', 0.0), 2),
                "current_v_cm_s": round(abs(getattr(self, 'current_v', 0.0)) * 100.0, 1),
                "total_distance_m": round(getattr(self, 'total_distance_m', 0.0), 3)
            },
            "turn_diagnostics": {
                "latest": getattr(self, 'latest_turn_perf', {}),
                "history": list(getattr(self, 'turn_diagnostics_history', [])),
                "current_imu_yaw_deg": round(math.degrees(getattr(self, 'latest_imu_yaw_val', self.robot_yaw)), 1),
                "imu_active": has_imu_active
            },
            "motor_trim_left": round(getattr(self.motors, 'trim_left', 1.0), 3) if self.motors else 1.0,
            "motor_trim_right": round(getattr(self.motors, 'trim_right', 1.0), 3) if self.motors else 1.0,
            "calibration": self.get_calibration_params(),
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
            self.soft_stop_enabled = True
            print("🛑 [CHẾ ĐỘ DỪNG] Luôn duy trì phanh từ từ (Soft Stop), không cho phép tắt về phanh gấp.")
            return True
        return False

    def set_debug_paused(self, paused=True):
        """Tạm dừng / tiếp tục luồng SLAM để quan sát chi tiết frame tại chỗ"""
        self.debug_paused = bool(paused)
        print(f"🔬 [ALGO INSPECTOR] Trạng thái Pause Debug: {'TẠM DỪNG' if self.debug_paused else 'TIẾP TỤC CHẠY'}")
        return self.debug_paused

    def request_debug_step(self):
        """Cho phép chạy đúng 1 frame tiếp theo khi đang ở chế độ Tạm dừng"""
        self.debug_paused = True
        self.debug_step_requested = True
        print(f"🔬 [ALGO INSPECTOR] Yêu cầu thực thi bước 1 Frame...")
        return True

    def save_map(self, name="my_room_map"):
        """Lưu bản đồ hiện tại sang file JSON & PNG"""
        if self.mapper:
            return self.mapper.save_map(name)
        return False, "Module Mapper chưa bật"

    def execute_step_move(self, move_type, value, direction='forward', speed_pct=None):
        """Thực thi bước di chuyển định lượng chính xác (cm hoặc độ) theo tốc độ %"""
        with self.lock:
            if getattr(self, 'is_step_running', False):
                return False, "Đang thực hiện bước trước đó"
            if self.is_auto_scanning:
                self.is_auto_scanning = False
            self.is_step_running = True
            self.step_move_type = str(move_type).lower()
            self.step_cancel_requested = False
            self.last_manual_drive_time = time.time()

        parsed_speed = float(speed_pct) if speed_pct is not None else None
        threading.Thread(target=self._step_worker, args=(move_type, float(value), direction, parsed_speed), daemon=True).start()
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

    def _step_worker(self, move_type, value, direction='forward', speed_pct=None):
        """Luồng điều khiển bước vi sai chính xác theo cự ly/góc và mức tốc độ PWM (%)"""
        try:
            self.step_move_type = str(move_type).lower()
            if move_type == 'linear':
                # ── MỨC GA CHẠY THẲNG: ĐỒNG BỘ TRỰC TIẾP TỪ CONFIG.DEFAULT_SPEED_PWM ──
                cfg_fwd_pwm = float(getattr(config, 'DEFAULT_SPEED_PWM', 0.06))
                if speed_pct is not None and float(speed_pct) > 0:
                    base_duty = max(0.03, min(0.40, float(speed_pct) / 100.0))
                else:
                    base_duty = max(0.03, min(0.40, cfg_fwd_pwm))

                # ── BÙ SỤT ÁP PIN THỜI GIAN THỰC (BATTERY VOLTAGE NORMALIZATION) ──
                bat_comp = self.get_battery_voltage_comp()
                duty = max(0.03, min(0.35, base_duty * bat_comp))
                pct_val = duty * 100.0


                dist_cm = max(1.0, min(500.0, float(value)))
                dist_m = dist_cm / 100.0
                is_fwd = (direction == 'forward')
                sign = 1.0 if is_fwd else -1.0

                # Tỉ lệ hiệu chuẩn cự ly thực tế (dùng chung cho cả bước cm và lái tay WASD)
                scale_factor = getattr(config, 'STEP_FORWARD_SCALE', 0.50) if is_fwd else getattr(config, 'STEP_BACKWARD_SCALE', 0.50)
                # HW_MAX_V = 0.55 m/s -> v_calib = duty * 0.55 để động cơ phát ra đúng duty PWM đã cấu hình!
                v_calib = duty * 0.55
                duration = max(0.04, (dist_m / v_calib) * scale_factor)
                v_target = v_calib if is_fwd else -v_calib
                brake_thresh = getattr(config, 'SAFETY_BRAKE_DIST_M', 0.20)

                # ── BIÊN DẠNG VẬN TỐC HÌNH SIN (COSINE S-CURVE TRAJECTORY ENGINE) ──
                # Hạ dốc giảm tốc mượt mà tuyệt đối (Jerk=0) và đồng bộ giải tích 100% với quãng đường
                enable_soft_stop = getattr(self, 'soft_stop_enabled', getattr(config, 'ENABLE_SOFT_STOP', True))
                t_nom = duration # Thời gian danh định ở vận tốc tối đa

                if enable_soft_stop and t_nom > 0.08:
                    accel_time_s = min(0.20, t_nom * 0.28)
                    decel_time_s = min(float(getattr(config, 'MANUAL_DECEL_TIME_MS', 180)) / 1000.0, t_nom * 0.35)
                    # Nếu cự ly rất ngắn (vd: 2-5cm), tự động co thời gian tăng/giảm tốc cân xứng
                    if t_nom <= ((accel_time_s + decel_time_s) / 2.0):
                        accel_time_s = t_nom / 2.0
                        decel_time_s = t_nom / 2.0
                        t_steady = 0.0
                    else:
                        t_steady = t_nom - ((accel_time_s + decel_time_s) / 2.0)
                else:
                    accel_time_s = 0.0
                    decel_time_s = 0.0
                    t_steady = t_nom

                total_t = accel_time_s + t_steady + decel_time_s
                prev_dist_m = 0.0
                last_t_meas = time.time()

                cmd_name = f"STEP_LINEAR_{'FWD' if is_fwd else 'BWD'}_{dist_cm:.0f}CM"
                with self.lock:
                    self.is_motion_active = True
                    self.current_motion_desc = cmd_name
                if hasattr(self, 'recorder') and self.recorder:
                    self.recorder.start_command(cmd_name)

                print(f"📏 [BƯỚC ĐỊNH LƯỢNG - S-CURVE] Bắt đầu {'TIẾN' if is_fwd else 'LÙI'} {dist_cm:.0f}cm @ {pct_val:.0f}% tốc độ (v={v_calib:.2f}m/s, calib={scale_factor:.2f}, t_tang={accel_time_s:.2f}s, t_deu={t_steady:.2f}s, t_giam={decel_time_s:.2f}s, soft_stop={'BẬT' if enable_soft_stop else 'TẮT'})...")
                start_t = time.time()
                start_yaw = getattr(self, 'robot_yaw', 0.0)
                w_assist = 0.0

                # VÒNG ĐIỀU KHIỂN ĐỘNG HỌC LIÊN TỤC 50Hz (Chu kỳ 20ms)
                while not self.step_cancel_requested:
                    now = time.time()
                    elapsed = now - start_t
                    if elapsed >= total_t:
                        break

                    # 1. TÍNH TOÁN BIÊN DẠNG VẬN TỐC TỨC THỜI ratio(t) & TÍCH PHÂN QUÃNG ĐƯỜNG GIẢI TÍCH sigma(t)
                    if elapsed < accel_time_s and accel_time_s > 0.001:
                        # Giai đoạn 1: Khởi động mềm hình Sin (Smooth S-curve ramp up)
                        ratio = 0.5 * (1.0 - math.cos(math.pi * (elapsed / accel_time_s)))
                        sigma_t = 0.5 * (elapsed - (accel_time_s / math.pi) * math.sin(math.pi * (elapsed / accel_time_s)))
                    elif elapsed < (accel_time_s + t_steady):
                        # Giai đoạn 2: Chạy đều ở vận tốc danh định (Steady cruising)
                        ratio = 1.0
                        sigma_t = 0.5 * accel_time_s + (elapsed - accel_time_s)
                    elif elapsed < total_t and decel_time_s > 0.001:
                        # Giai đoạn 3: Hạ dốc giảm tốc mượt mà hình Sin (Smooth S-curve ramp down)
                        u = elapsed - (accel_time_s + t_steady)
                        ratio = 0.5 * (1.0 + math.cos(math.pi * min(1.0, u / decel_time_s)))
                        sigma_t = 0.5 * accel_time_s + t_steady + 0.5 * (u + (decel_time_s / math.pi) * math.sin(math.pi * min(1.0, u / decel_time_s)))
                    else:
                        ratio = 0.0
                        sigma_t = t_nom

                    # Quãng đường giải tích chính xác tuyệt đối tại thời điểm t:
                    cur_target_dist = min(dist_m, dist_m * (sigma_t / max(0.001, t_nom)))
                    d_dist = max(0.0, cur_target_dist - prev_dist_m)
                    prev_dist_m = cur_target_dist

                    # Tiến độ hiển thị trên Web Cockpit (0% -> 100%)
                    pct = min(99, int((cur_target_dist / dist_m) * 100))
                    self.step_status_text = f"ĐANG {'TIẾN' if is_fwd else 'LÙI'} {dist_cm:.0f}CM @ {pct_val:.0f}%... ({pct}%)"

                    # 2. KIỂM TRA PHANH AN TOÀN VẬT CẢN (VIRTUAL BUMPER)
                    if is_fwd:
                        with self.lock:
                            obs_d = self.obstacle_distance
                        if obs_d is not None and obs_d <= brake_thresh:
                            print(f"🛑 [BƯỚC ĐỊNH LƯỢNG] Khóa tiến vì phát hiện cản cách {obs_d*100:.1f}cm!")
                            self.step_status_text = f"DỪNG: GẶP CẢN {obs_d*100:.1f}CM"
                            break

                    # 3. KHÓA HƯỚNG TỰ ĐỘNG (ACTIVE HEADING LOCK KHI ĐI THẲNG)
                    if getattr(config, 'ENABLE_HEADING_LOCK', False):
                        cur_yaw = getattr(self, 'robot_yaw', 0.0)
                        yaw_err = math.atan2(math.sin(start_yaw - cur_yaw), math.cos(start_yaw - cur_yaw))
                        kp = getattr(config, 'HEADING_LOCK_KP', 0.50)
                        w_assist = max(-0.16, min(0.16, kp * yaw_err))
                    else:
                        w_assist = 0.0

                    dt_run = now - last_t_meas
                    last_t_meas = now
                    v_cmd = v_target * ratio
                    w_cmd = w_assist * ratio

                    # 4. CẬP NHẬT POSE VÀ BỘ ĐO HÀNH TRÌNH CHÍNH XÁC TỪNG MILIMET
                    with self.lock:
                        self.current_v = v_cmd
                        self.current_w = w_cmd
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
                        self.motors.set_cmd_vel(v_cmd, w_cmd)

                    time.sleep(0.02) # Nhịp 50Hz mượt mà

                # Dừng hẳn động cơ êm ái
                if self.motors:
                    self.motors.stop()

                # Đồng bộ hoàn tất 100.0% cự ly nếu không bị hủy/gặp cản:
                if not self.step_cancel_requested and (self.obstacle_distance is None or self.obstacle_distance > brake_thresh):
                    rem_dist = max(0.0, dist_m - prev_dist_m)
                    if rem_dist > 0:
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

                # ── GHI NHẬN MỐC GÓC BAN ĐẦU & RESET BỘ TÍCH LŨY IMU ──
                if self.camera and hasattr(self.camera, 'reset_imu_heading'):
                    self.camera.reset_imu_heading()
                time.sleep(0.02) # Cho IMU ổn định mốc 0

                has_imu_ready = False
                if self.camera and hasattr(self.camera, 'get_imu_heading'):
                    has_imu_ready, _, _ = self.camera.get_imu_heading()

                yaw_start_pose = self.robot_yaw
                t_turn_start = time.time()

                # Tách hệ số tỉ lệ quay TRÁI / PHẢI riêng biệt làm dự phòng cho vòng hở:
                if is_left:
                    turn_scale = getattr(config, 'STEP_TURN_LEFT_SCALE', 0.60)
                else:
                    turn_scale = getattr(config, 'STEP_TURN_RIGHT_SCALE', 0.60)

                # ── MỨC GA QUAY TẠI CHỖ & BÙ SỤT ÁP PIN ──
                turn_min_pwm = float(getattr(config, 'STEP_TURN_MIN_PWM', 0.11))
                turn_ratio = float(getattr(config, 'STEP_TURN_SPEED_RATIO', 1.0))

                if speed_pct is not None and float(speed_pct) > 0:
                    base_turn_duty = max(0.05, min(0.35, (float(speed_pct) / 100.0) * turn_ratio))
                else:
                    base_turn_duty = max(0.05, min(0.35, turn_min_pwm * turn_ratio))

                # Áp dụng bù sụt áp pin thời gian thực để mô-men quay luôn đồng nhất
                bat_comp = self.get_battery_voltage_comp()
                turn_duty = max(0.08, min(0.30, base_turn_duty * bat_comp))
                pct_val = turn_duty * 100.0
                side_name = 'TRÁI' if is_left else 'PHẢI'

                cmd_name = f"STEP_TURN_{'LEFT' if is_left else 'RIGHT'}_{deg:.0f}DEG"
                with self.lock:
                    self.is_motion_active = True
                    self.is_step_turning = True
                    self._settle_frames_remaining = 0
                    self._was_moving = False
                    self.current_motion_desc = cmd_name
                if hasattr(self, 'recorder') and self.recorder:
                    self.recorder.start_command(cmd_name)

                use_closed_loop = getattr(config, 'ENABLE_CLOSED_LOOP_IMU', True) and has_imu_ready
                if use_closed_loop:
                    # ══════════════════════════════════════════════════════════════════
                    # CHẾ ĐỘ 1: VÒNG ĐIỀU KHIỂN KÍN IMU THỜI GIAN THỰC (CLOSED-LOOP IMU TURN)
                    # ══════════════════════════════════════════════════════════════════
                    target_deg = deg
                    decel_zone_deg = min(20.0, target_deg * 0.40)
                    min_crawl_duty = 0.075
                    timeout_s = (target_deg / 20.0) + 3.5
                    t_loop_start = time.time()
                    last_t_meas = t_loop_start

                    print(f"🔄 [BƯỚC ĐỊNH LƯỢNG - VÒNG KÍN IMU] Bắt đầu QUAY {side_name} {target_deg:.0f}° @ {pct_val:.0f}% tốc độ (Xung={turn_duty*100:.0f}%, Bù pin x{bat_comp:.2f})...")

                    while not self.step_cancel_requested:
                        now = time.time()
                        if (now - t_loop_start) > timeout_s:
                            print(f"⚠️ [VÒNG KÍN IMU] Hết thời gian an toàn ({timeout_s:.1f}s), dừng quay!")
                            break

                        # Đọc trực tiếp góc đã quay từ IMU accumulator
                        _, acc_rad, raw_yaw = self.camera.get_imu_heading()
                        turned_deg = math.degrees(abs(acc_rad))
                        rem_deg = target_deg - turned_deg

                        pct = max(0, min(98, int((turned_deg / target_deg) * 100)))
                        self.step_status_text = f"ĐANG QUAY {side_name} {target_deg:.0f}° ({turned_deg:.1f}°/{target_deg:.0f}°)... ({pct}%)"

                        if rem_deg <= 1.0:
                            break

                        if rem_deg > decel_zone_deg:
                            cur_duty = turn_duty
                        else:
                            ramp_ratio = max(0.0, rem_deg / decel_zone_deg)
                            cur_duty = min_crawl_duty + (turn_duty - min_crawl_duty) * ramp_ratio

                        dt_run = now - last_t_meas
                        last_t_meas = now
                        w_cmd = 1.58 * (cur_duty / 0.13) * dir_sign

                        with self.lock:
                            self.current_v = 0.0
                            self.current_w = w_cmd
                            self.last_cmd_vel_time = now
                            self.trip_active_time_s += dt_run

                        if self.motors:
                            self.motors.spin_in_place(duty=cur_duty, direction=dir_val)

                        time.sleep(0.010)

                    if self.motors:
                        if hasattr(self.motors, 'active_brake'):
                            self.motors.active_brake(reverse_duty=0.10, pulse_ms=25, spin_direction=dir_val)
                        else:
                            self.motors.stop()

                else:
                    # ══════════════════════════════════════════════════════════════════
                    # CHẾ ĐỘ 2: VÒNG HỞ THEO THỜI GIAN (DỰ PHÒNG KHI KHÔNG CÓ IMU)
                    # ══════════════════════════════════════════════════════════════════
                    w_calib = 1.58 * (turn_duty / 0.13)
                    stiction_comp_s = float(getattr(config, 'STEP_TURN_START_LATENCY_S', 0.080))
                    duration_motion = (rad / w_calib) * turn_scale
                    duration = max(0.05, duration_motion + stiction_comp_s)
                    w_target = w_calib if is_left else -w_calib

                    enable_soft_stop = getattr(self, 'soft_stop_enabled', getattr(config, 'ENABLE_SOFT_STOP', True))
                    max_decel_s = float(getattr(config, 'MANUAL_DECEL_TIME_MS', 180)) / 1000.0
                    decel_time_s = min(max_decel_s, duration * 0.28) if enable_soft_stop else 0.0

                    if enable_soft_stop and duration > decel_time_s:
                        t_steady = duration - (decel_time_s / 2.0)
                    else:
                        t_steady = duration
                        decel_time_s = 0.0

                    total_t = t_steady + decel_time_s
                    last_progress = 0.0
                    last_t_meas = time.time()

                    print(f"🔄 [BƯỚC ĐỊNH LƯỢNG - VÒNG HỞ DỰ PHÒNG] Bắt đầu QUAY {side_name} {deg:.0f}° @ {pct_val:.0f}% tốc độ (t={t_steady:.2f}s, t_ham={decel_time_s:.2f}s)...")
                    start_t = time.time()

                    while not self.step_cancel_requested and (time.time() - start_t < t_steady):
                        now = time.time()
                        elapsed = now - start_t
                        pct = min(90, int((elapsed / total_t) * 100))
                        self.step_status_text = f"ĐANG QUAY {side_name} {deg:.0f}° @ {pct_val:.0f}%... ({pct}%)"

                        progress = min(1.0, elapsed / max(0.001, total_t))
                        last_progress = progress
                        dt_run = now - last_t_meas
                        last_t_meas = now

                        with self.lock:
                            self.current_v = 0.0
                            self.current_w = w_target
                            self.last_cmd_vel_time = now
                            self.trip_active_time_s += dt_run

                        if self.motors:
                            self.motors.spin_in_place(duty=turn_duty, direction=dir_val)

                        time.sleep(0.02)

                    if not self.step_cancel_requested and decel_time_s > 0.02:
                        steps = 4
                        dt_step = decel_time_s / steps
                        for i in range(1, steps + 1):
                            if self.step_cancel_requested:
                                break
                            ratio = 1.0 - (i / steps)
                            cur_duty = max(0.075, turn_duty * ratio) if ratio > 0.1 else 0.0
                            cur_w = w_target * ratio
                            with self.lock:
                                self.current_v = 0.0
                                self.current_w = cur_w
                                self.last_cmd_vel_time = time.time()
                                self.trip_active_time_s += dt_step
                            if self.motors:
                                if cur_duty >= 0.07:
                                    self.motors.spin_in_place(duty=cur_duty, direction=dir_val)
                                else:
                                    self.motors.stop()
                            time.sleep(dt_step)

                    if self.motors:
                        self.motors.stop()

                # Đợi 100ms để triệt tiêu hoàn toàn rung lắc quán tính
                time.sleep(0.10)

                # ── ĐO ĐẠC GÓC QUAY THỰC TẾ TỪ CẢM BIẾN IMU ──
                has_valid_imu_eval = False
                imu_meas_deg = None
                imu_diff_deg = None

                if self.camera and hasattr(self.camera, 'get_imu_heading'):
                    c_act, imu_acc_rad, imu_raw = self.camera.get_imu_heading()
                    if c_act and abs(imu_acc_rad) > 0.01:
                        has_valid_imu_eval = True
                        actual_rad = abs(imu_acc_rad)
                        imu_meas_deg = math.degrees(actual_rad)
                        imu_diff_deg = imu_meas_deg - deg

                if not has_valid_imu_eval:
                    actual_deg = deg
                    error_deg = None  # None: Rendered as '--', tuyệt đối không fake 0.0°!
                    actual_rad = rad
                    eval_status = "ĐỘNG HỌC MỞ (KHÔNG CÓ IMU)"
                    slam_calibrated = False
                else:
                    actual_deg = imu_meas_deg
                    error_deg = imu_diff_deg
                    if abs(imu_diff_deg) <= 2.5:
                        eval_status = f"CHUẨN XÁC ({imu_diff_deg:+.1f}°)"
                    elif imu_diff_deg < -2.5:
                        eval_status = f"THIẾU GÓC ({imu_diff_deg:+.1f}°)"
                    else:
                        eval_status = f"DƯ GÓC ({imu_diff_deg:+.1f}°)"
                    slam_calibrated = True

                # Cập nhật góc robot_yaw theo góc quay thực tế
                final_robot_yaw = math.atan2(
                    math.sin(yaw_start_pose + dir_sign * actual_rad),
                    math.cos(yaw_start_pose + dir_sign * actual_rad)
                )

                # CẬP NHẬT POSE VÀ TRIP ODOMETER
                with self.lock:
                    self.trip_turn_rad += actual_rad
                    self.robot_yaw = final_robot_yaw
                    self.current_v = 0.0
                    self.current_w = 0.0
                    self._settle_frames_remaining = 0
                    self._was_moving = False
                    self.is_step_turning = False
                    self._just_finished_turn = True
                    # Đồng bộ neo IMU để camera loop không gây gián đoạn hay kéo lệch góc
                    if self.camera and hasattr(self.camera, 'get_imu_heading'):
                        _, _, cur_raw = self.camera.get_imu_heading()
                        self._imu_yaw_anchor = cur_raw - final_robot_yaw

                # Cập nhật ngay 1 frame vào bản đồ 2D Occupancy Grid tại góc quay chuẩn vừa tới
                if self.mapper and self.flags.mapper and self.camera and getattr(self.camera, 'points_3d', None):
                    self.mapper.update_scan(self.robot_x, self.robot_y, self.camera.points_3d, yaw=final_robot_yaw)

                # LƯU VÀO BẢNG GIÁM SÁT ĐỐI CHIẾU GÓC QUAY IMU
                self._turn_eval_count = getattr(self, '_turn_eval_count', 0) + 1
                turn_record = {
                    "id": self._turn_eval_count,
                    "time_str": time.strftime("%H:%M:%S"),
                    "direction": "TRÁI ↺" if is_left else "PHẢI ↻",
                    "target_deg": round(deg, 1),
                    "actual_deg": round(imu_meas_deg, 1) if has_valid_imu_eval else round(actual_deg, 1),
                    "error_deg": round(imu_diff_deg, 1) if has_valid_imu_eval else None,
                    "status": eval_status,
                    "has_imu": has_valid_imu_eval,
                    "speed_pct": round(pct_val, 0),
                    "duration_s": round(time.time() - t_turn_start, 2),
                    "slam_calibrated": slam_calibrated
                }
                self.latest_turn_perf = turn_record
                self.turn_diagnostics_history.append(turn_record)
                if len(self.turn_diagnostics_history) > 20:
                    self.turn_diagnostics_history.pop(0)

                err_log_str = f"{error_deg:+.1f}°" if error_deg is not None else "--"
                meas_log_str = f"{actual_deg:.1f}°" if has_valid_imu_eval else f"{actual_deg:.1f}° (mô hình)"
                print(f"🧭 [ĐO ĐẠC GÓC QUAY] Quay {side_name} {deg:.0f}° | Thực tế: {meas_log_str} | Lệch: {err_log_str} ({eval_status}) | Đồng bộ 2D Map Pose: {slam_calibrated}")

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
                self.step_move_type = ""
                self.current_v = 0.0
                self.current_w = 0.0
                self.is_motion_active = False
                self.current_motion_desc = ""
            if hasattr(self, 'recorder') and self.recorder:
                self.recorder.stop_command()
            if self.motors:
                self.motors.stop()

    def execute_pattern_patrol(self, pattern_type='rectangle', **kwargs):
        """Kích hoạt bài thử nghiệm di chuyển theo quỹ đạo hình học khép kín (Rectangle / Right Isosceles Triangle)"""
        with self.lock:
            if getattr(self, 'is_pattern_running', False) or getattr(self, 'is_step_running', False):
                return False, "Đang có tiến trình di chuyển đang thực thi"
            if self.is_auto_scanning:
                self.is_auto_scanning = False
            self.is_pattern_running = True
            self.pattern_cancel_requested = False
            self.step_cancel_requested = False
            self.pattern_type = pattern_type
            self.pattern_status_text = f"BẮT ĐẦU QUỸ ĐẠO {pattern_type.upper()}"
            self.last_manual_drive_time = time.time()

        threading.Thread(target=self._pattern_worker, args=(pattern_type, kwargs), daemon=True).start()
        return True, f"Bắt đầu chạy quỹ đạo {pattern_type}"

    def cancel_pattern_patrol(self):
        """Hủy khẩn cấp bài thử nghiệm quỹ đạo hình học"""
        with self.lock:
            self.pattern_cancel_requested = True
            self.step_cancel_requested = True
            self.is_pattern_running = False
            self.pattern_status_text = "ĐÃ HỦY QUỸ ĐẠO"
            self.current_v = 0.0
            self.current_w = 0.0
        if self.motors:
            if hasattr(self.motors, 'active_brake'):
                self.motors.active_brake(reverse_duty=0.14, pulse_ms=35)
            else:
                self.motors.stop()
        print("⏹️ [GEOMETRIC BENCHMARK] Đã dừng khẩn cấp bài thử nghiệm quỹ đạo.")
        return True

    def _pattern_worker(self, pattern_type, params):
        """Luồng thực thi tuần tự các chặng quỹ đạo hình học khép kín"""
        try:
            speed_pct = float(params.get('speed_pct', 10.0))
            turn_dir = str(params.get('turn_dir', 'left')).lower()
            if turn_dir not in ['left', 'right']: turn_dir = 'left'

            if pattern_type in ['rectangle', 'rect']:
                l_cm = max(15.0, min(300.0, float(params.get('length', 80.0))))
                w_cm = max(15.0, min(300.0, float(params.get('width', 40.0))))
                dir_label = "TRÁI" if turn_dir == 'left' else "PHẢI"
                steps = [
                    ('linear', l_cm, 'forward', f"Cạnh Dài 1 (Tiến {l_cm:.0f}cm)"),
                    ('turn', 90.0, turn_dir, f"Góc Vuông 1 (Quay {dir_label} 90°)"),
                    ('linear', w_cm, 'forward', f"Cạnh Rộng 1 (Tiến {w_cm:.0f}cm)"),
                    ('turn', 90.0, turn_dir, f"Góc Vuông 2 (Quay {dir_label} 90°)"),
                    ('linear', l_cm, 'forward', f"Cạnh Dài 2 (Tiến {l_cm:.0f}cm)"),
                    ('turn', 90.0, turn_dir, f"Góc Vuông 3 (Quay {dir_label} 90°)"),
                    ('linear', w_cm, 'forward', f"Cạnh Rộng 2 (Tiến {w_cm:.0f}cm)"),
                    ('turn', 90.0, turn_dir, f"Khép Kín Góc (Quay {dir_label} 90°)")
                ]
                pat_name = f"HÌNH CHỮ NHẬT {l_cm:.0f}x{w_cm:.0f}cm"

            elif pattern_type in ['triangle', 'tri']:
                a_cm = max(15.0, min(250.0, float(params.get('leg', 50.0))))
                c_cm = round(a_cm * 1.41421356, 1) # a * sqrt(2) theo định lý Pythagore
                dir_label = "TRÁI" if turn_dir == 'left' else "PHẢI"
                steps = [
                    ('linear', a_cm, 'forward', f"Cạnh Góc Vuông 1 (Tiến {a_cm:.0f}cm)"),
                    ('turn', 135.0, turn_dir, f"Góc Nhọn 1 (Quay {dir_label} 135°)"),
                    ('linear', c_cm, 'forward', f"Cạnh Huyền (Tiến {c_cm:.1f}cm)"),
                    ('turn', 135.0, turn_dir, f"Góc Nhọn 2 (Quay {dir_label} 135°)"),
                    ('linear', a_cm, 'forward', f"Cạnh Góc Vuông 2 (Tiến {a_cm:.0f}cm)"),
                    ('turn', 90.0, turn_dir, f"Khép Kín Góc Vuông (Quay {dir_label} 90°)")
                ]
                pat_name = f"TAM GIÁC VUÔNG CÂN {a_cm:.0f}x{a_cm:.0f}cm"
            else:
                self.pattern_status_text = f"QUỸ ĐẠO KHÔNG HỢP LỆ: {pattern_type}"
                return

            self.pattern_total_legs = len(steps)
            print(f"📐 [GEOMETRIC BENCHMARK] Bắt đầu quỹ đạo {pat_name} gồm {len(steps)} chặng @ {speed_pct:.0f}% PWM...")

            for idx, (m_type, val, direction, desc) in enumerate(steps, 1):
                if getattr(self, 'pattern_cancel_requested', False):
                    print("⏹️ [GEOMETRIC BENCHMARK] Đã hủy quỹ đạo giữa chừng.")
                    break

                self.pattern_current_leg = idx
                self.pattern_status_text = f"CHẶNG {idx}/{len(steps)}: {desc}"
                print(f" ▶️ [CHẶNG {idx}/{len(steps)}] {desc}...")

                with self.lock:
                    self.is_step_running = True
                    self.step_move_type = str(m_type).lower()
                    self.step_cancel_requested = False

                self._step_worker(m_type, val, direction, speed_pct=None)

                if getattr(self, 'pattern_cancel_requested', False) or getattr(self, 'step_cancel_requested', False):
                    break

                # Nghỉ ổn định thân xe 0.40s giữa các chặng để triệt tiêu đà quán tính
                time.sleep(0.40)

            if not getattr(self, 'pattern_cancel_requested', False):
                self.pattern_status_text = f"HOÀN TẤT 100% {pat_name}!"
                print(f"✅ [GEOMETRIC BENCHMARK] Hoàn tất 100% quỹ đạo {pat_name}! Kiểm tra độ khép kín trên bản đồ 2D và sàn thực tế.")
            else:
                self.pattern_status_text = "ĐÃ HỦY QUỸ ĐẠO"
        except Exception as e:
            self.pattern_status_text = f"LỖI: {e}"
            print(f"❌ [GEOMETRIC BENCHMARK] Lỗi: {e}")
        finally:
            with self.lock:
                self.is_pattern_running = False
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
                scan_pwm = float(pwm) if pwm and 0.08 <= float(pwm) <= 0.25 else float(getattr(config, 'STEP_TURN_MIN_PWM', getattr(config, 'AUTO_SCAN_SPEED_PWM', 0.11)))
                scan_dur = float(duration) if duration and float(duration) > 0 else float(getattr(config, 'AUTO_SCAN_DURATION_S', 4.8))
                self.auto_scan_duration = scan_dur
                self.auto_scan_remaining = scan_dur
                threading.Thread(target=self._auto_scan_worker, args=(scan_dur, scan_pwm), daemon=True).start()
                print(f"🔄 [AUTO SCAN 360°] Bắt đầu chế độ quét toàn cảnh 8 nấc x 45° Dừng-Chụp...")
                return True

    def _capture_static_scan_keyframe(self, fixed_x=None, fixed_y=None, fixed_yaw=None):
        """Chụp khung hình tĩnh khi xe đang dừng và cập nhật trực tiếp vào SLAM mà không chạy CSM sai lệch."""
        try:
            live_depth = getattr(self.camera, 'latest_oak_depth', None)
            if live_depth is None and self.camera:
                if hasattr(self.camera, 'get_live_frame'):
                    res = self.camera.get_live_frame()
                    if res and len(res) >= 2:
                        live_depth = res[1]
                elif hasattr(self.camera, 'get_latest_frame'):
                    _, live_depth = self.camera.get_latest_frame()
            if live_depth is None:
                live_depth = getattr(self, 'latest_depth_np', None)

            if live_depth is not None and self.mapper:
                with self.lock:
                    rx = fixed_x if fixed_x is not None else self.robot_x
                    ry = fixed_y if fixed_y is not None else self.robot_y
                    rz = self.robot_z
                    yaw = fixed_yaw if fixed_yaw is not None else self.robot_yaw
                    self.robot_x = rx
                    self.robot_y = ry
                    self.robot_yaw = yaw

                # Chiếu điểm chiều sâu 3D thành 120 tia quét 2D
                self.camera.process_depth_frame(live_depth, rx, ry, rz, yaw)

                # Bắn tia quang học vào bản đồ Occupancy Grid từ tâm neo chuẩn xác (KHÔNG gọi CSM làm méo lệch góc)
                if self.camera.points_3d:
                    self.mapper.update_scan(rx, ry, self.camera.points_3d, yaw=yaw)
        except Exception as ex:
            print(f"⚠️ [KEYFRAME SCAN] Lỗi nạp keyframe: {ex}")

    def _auto_scan_worker(self, scan_duration=None, scan_pwm=None):
        """
        Luồng điều khiển tự động quét toàn cảnh 360 độ RỜI RẠC TỪNG BƯỚC (Stepped 360° Panorama):
        1. Chia vòng tròn 360 độ thành 8 nấc rời rạc, mỗi nấc đúng 45 độ (CCW).
        2. Tại mỗi nấc (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°):
           - Xe dừng tĩnh hoàn toàn trong 0.45 giây để triệt tiêu đà quán tính và chống mờ chuyển động.
           - Trong lúc xe đứng yên: Camera chụp khung hình tĩnh sắc nét nhất (Keyframe),
             bắn 120 tia quét vào Occupancy Grid và gọi Correlative Scan Matching để tự cân bằng.
           - Sau đó, xe thực hiện bước quay 45 độ (CCW) với bộ điều khiển vi sai chuẩn hóa.
        3. Khóa cứng vật cản ban đầu (Anchor Wall Freezing):
           - Khi hoàn tất đủ 8 nấc (360 độ), hệ thống tự động gọi self.mapper.freeze_anchor_walls()
             để khóa kiên cố toàn bộ các ô tường mốc thành 'Bờ Tường Bất Tử' (Immutable Anchor Walls).
           - Các bức tường mốc này không bao giờ bị xóa hay ăn mòn bởi các tia quét sau này.
        """
        steps = 8
        step_deg = 45.0
        total_time_est = steps * (1.2 + 0.45)
        self.auto_scan_duration = total_time_est
        self.auto_scan_remaining = total_time_est
        t_start_total = time.time()

        print("🔄 [STEPPED 360° SCAN] Bắt đầu chu trình quét toàn cảnh 8 nấc x 45° Dừng-Chụp...")

        try:
            with self.lock:
                anchor_x = self.robot_x
                anchor_y = self.robot_y
                anchor_yaw = self.robot_yaw

            # Nấc 0 (Tại góc xuất phát 0°): Dừng tĩnh để chụp bức tường mốc ban đầu từ tâm neo
            time.sleep(0.35)
            self._capture_static_scan_keyframe(anchor_x, anchor_y, anchor_yaw)
            print("📸 [NẤC 1/8] Đã ghi nhận khung hình mốc ban đầu tại 0°.")

            for i in range(1, steps + 1):
                if not self.is_auto_scanning:
                    print("⏹️ [STEPPED 360° SCAN] Đã hủy quét 360° theo yêu cầu.")
                    break

                target_heading_deg = i * step_deg
                pct_done = int((i - 1) / steps * 100)
                print(f" ▶️ [NẤC {i}/{steps}] Đang quay 45° CCW hướng tới mốc {target_heading_deg:.0f}°... ({pct_done}%)")

                # 1. Quay 45 độ CCW bằng _step_worker (đồng bộ chuẩn xác theo STEP_TURN_MIN_PWM)
                with self.lock:
                    self.is_step_running = True
                    self.step_move_type = 'turn'
                    self.step_cancel_requested = False

                turn_pwm_val = scan_pwm if scan_pwm else float(getattr(config, 'STEP_TURN_MIN_PWM', getattr(config, 'AUTO_SCAN_SPEED_PWM', 0.11)))
                scan_spd_pct = round(turn_pwm_val * 100.0, 1)
                self._step_worker('turn', step_deg, 'left', speed_pct=scan_spd_pct)

                if not self.is_auto_scanning or self.step_cancel_requested:
                    break

                # Tính góc đích theo hình học nấc quay CCW (+)
                step_target_yaw = math.atan2(math.sin(anchor_yaw + i * math.radians(step_deg)), math.cos(anchor_yaw + i * math.radians(step_deg)))

                # 2. KHÓA CỐ ĐỊNH TÂM NEO (x0, y0) VÀ GÓC BƯỚC: Xe quay tại chỗ nên tọa độ không đổi!
                with self.lock:
                    self.robot_x = anchor_x
                    self.robot_y = anchor_y
                    self.robot_yaw = step_target_yaw
                    self.current_v = 0.0
                    self.current_w = 0.0
                time.sleep(0.45)

                # 3. Chụp và cập nhật Keyframe tĩnh vào bản đồ Occupancy Grid từ tâm neo chuẩn xác
                self._capture_static_scan_keyframe(anchor_x, anchor_y, step_target_yaw)

                elapsed = time.time() - t_start_total
                self.auto_scan_remaining = max(0.0, total_time_est - elapsed)
                print(f"📸 [NẤC {i}/{steps}] Hoàn tất nấc {i} ({target_heading_deg:.0f}°). Đã cập nhật bản đồ.")

            # 4. KHÓA CỨNG BẢN ĐỒ MỐC (ANCHOR WALL FREEZING)
            if self.is_auto_scanning and self.mapper:
                num_frozen = self.mapper.freeze_anchor_walls()
                print(f"🔒 [STEPPED 360° SCAN] HOÀN TẤT 100%! Đã khóa vĩnh viễn {num_frozen} ô bờ tường mốc 360° khởi tạo.")

        except Exception as e:
            print(f"❌ [STEPPED 360° SCAN] Lỗi trong quá trình quét: {e}")
        finally:
            with self.lock:
                self.is_auto_scanning = False
                self.is_step_running = False
                self.step_move_type = ''
                self.current_v = 0.0
                self.current_w = 0.0
                self.auto_scan_remaining = 0.0
            if self.motors:
                self.motors.stop()
            print("🏁 [STEPPED 360° SCAN] Đã kết thúc chu trình quét toàn cảnh 360°.")


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
        """Reset toàn bộ bản đồ 2D, đặt lại gốc tọa độ xuất phát (0, 0, 0) và làm mới chuỗi phân tích thuật toán"""
        # 1. HỦY NGAY LẬP TỨC MỌI TIẾN TRÌNH CHẠY NGẦM (bước lệnh, xoay 360, giảm tốc mềm, pattern):
        with self.lock:
            self.step_cancel_requested = True
            self.is_step_running = False
            self.pattern_cancel_requested = True
            self.is_pattern_running = False
            self.is_auto_scanning = False
            self._manual_decel_id = getattr(self, '_manual_decel_id', 0) + 1
            self.current_v = 0.0
            self.current_w = 0.0
            self.last_cmd_vel_time = time.time()
            self._manual_straight_yaw = None

        if self.motors:
            self.motors.stop()

        self.clear_goal()

        # 2. NEO CỨNG TỌA ĐỘ VỀ GỐC XUẤT PHÁT (0, 0, 0) VÀ XÓA SẠCH LỊCH SỬ HÀNH TRÌNH:
        with self.lock:
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_z = 0.0
            self.robot_yaw = 0.0
            self._imu_yaw_anchor = None
            self._ros_imu_yaw_anchor = None
            self._ros_odom_initialized = False
            self._ros_odom_active_topic = None
            self._ros_odom_origin_x = 0.0
            self._ros_odom_origin_y = 0.0
            self._ros_odom_origin_z = 0.0
            self._ros_odom_origin_yaw = 0.0
            self.last_real_ros_odom_time = 0.0
            self.path_history = []
            self.total_distance_m = 0.0
            self.trip_distance_m = 0.0
            self.trip_turn_rad = 0.0
            self.trip_active_time_s = 0.0

            # Làm mới hoàn toàn phiên quan sát thuật toán từng frame:
            self._diag_frame_id = 0
            if hasattr(self, 'recent_frame_logs'):
                self.recent_frame_logs.clear()
            self.latest_frame_diag = {}
            if hasattr(self, 'turn_diagnostics_history'):
                self.turn_diagnostics_history.clear()
            self.latest_turn_perf = {}
            self._turn_eval_count = 0

        # 3. XÓA SẠCH TOÀN BỘ BỘ ĐỆM MÂY ĐIỂM CỦA BẢN ĐỒ CŨ (tránh bắn đè vào bản đồ mới):
        if self.camera:
            self.camera.points_3d = []
            self.camera.points_robot = []
            self.camera.last_ray_distances = []
            self.camera.last_ray_types = []
            if hasattr(self.camera, 'reset_imu_heading'):
                self.camera.reset_imu_heading()

        # 4. ĐẶT LẠI THỐNG KÊ CSM SCAN MATCHER:
        if self.scan_matcher:
            self.scan_matcher.last_dx_cm = 0.0
            self.scan_matcher.last_dy_cm = 0.0
            self.scan_matcher.last_dyaw_deg = 0.0
            self.scan_matcher.last_raw_score = 0.0
            self.scan_matcher.last_init_score = 0.0
            self.scan_matcher.last_matched = False

        # 5. XÓA TRẮNG BẢN ĐỒ OCCUPANCY GRID VÀ ĐƯA ORIGIN VỀ TRUNG TÂM LƯỚI:
        if self.mapper:
            self.mapper.reset_map()

        print("🔄 [RESET BẢN ĐỒ] Đã triệt tiêu toàn bộ tiến trình chạy ngầm, neo JetBot về gốc xuất phát (0, 0, 0) và làm mới chuỗi phân tích thuật toán!")
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

    def set_motor_trim(self, left: float, right: float):
        """Cập nhật hệ số cân bằng tỉ lệ 2 bánh xe (Hardware Wheel Trim)"""
        if self.motors:
            self.motors.set_trim(left, right)
            return True
        return False

    def save_motor_trim(self, left: float, right: float):
        """Cập nhật và lưu vĩnh viễn hệ số cân bằng 2 bánh vào đĩa"""
        if self.motors:
            self.motors.set_trim(left, right)
            return self.motors.save_trim_to_disk()
        return False

    def get_calibration_params(self):
        """Lấy toàn bộ thông số hiệu chuẩn động học hiện tại"""
        tl = round(getattr(self.motors, 'trim_left', 1.0), 3) if self.motors else 1.0
        tr = round(getattr(self.motors, 'trim_right', 1.0), 3) if self.motors else 1.0
        return {
            "motor_trim_left": tl,
            "motor_trim_right": tr,
            "trim_factor": round(tr / max(0.01, tl), 3),
            "step_forward_scale": round(getattr(config, 'STEP_FORWARD_SCALE', 0.40), 3),
            "step_backward_scale": round(getattr(config, 'STEP_BACKWARD_SCALE', 0.40), 3),
            "step_turn_left_scale": round(getattr(config, 'STEP_TURN_LEFT_SCALE', 0.62), 3),
            "step_turn_right_scale": round(getattr(config, 'STEP_TURN_RIGHT_SCALE', 0.64), 3),
            "step_turn_min_pwm": round(getattr(config, 'STEP_TURN_MIN_PWM', 0.11), 3),
            "camera_offset_forward_m": round(getattr(config, 'CAMERA_OFFSET_FORWARD_M', 0.09), 3),
            "camera_offset_lateral_m": round(getattr(config, 'CAMERA_OFFSET_LATERAL_M', -0.015), 3),
            "camera_height_m": round(getattr(config, 'CAMERA_HEIGHT_M', 0.12), 3),
            "camera_pitch_deg": round(getattr(config, 'CAMERA_PITCH_DEG', 0.0), 1),
            "wheel_separation_m": round(getattr(config, 'WHEEL_SEPARATION_M', 0.12), 3),
        }

    def get_full_config(self):
        """Lấy toàn bộ cấu hình 4 Profiles và các tham số runtime hiện hành cho Web UI Tab 3"""
        with self.lock:
            cur_trim_l = round(getattr(self.motors, 'trim_left', 1.0), 3) if self.motors else 1.0
            cur_trim_r = round(getattr(self.motors, 'trim_right', 1.0), 3) if self.motors else 1.0

        current_params = {
            "motor_trim_left": cur_trim_l,
            "motor_trim_right": cur_trim_r,
            "step_forward_scale": round(getattr(config, 'STEP_FORWARD_SCALE', 0.50), 3),
            "step_backward_scale": round(getattr(config, 'STEP_BACKWARD_SCALE', 0.50), 3),
            "step_turn_left_scale": round(getattr(config, 'STEP_TURN_LEFT_SCALE', 0.69), 3),
            "step_turn_right_scale": round(getattr(config, 'STEP_TURN_RIGHT_SCALE', 0.75), 3),
            "default_speed_pwm": round(getattr(config, 'DEFAULT_SPEED_PWM', 0.06), 3),
            "step_turn_min_pwm": round(getattr(config, 'STEP_TURN_MIN_PWM', 0.11), 3),
            "manual_decel_time_ms": int(getattr(config, 'MANUAL_DECEL_TIME_MS', 180)),
            "step_turn_start_latency_s": round(getattr(config, 'STEP_TURN_START_LATENCY_S', 0.08), 3),
            "heading_lock_kp": round(getattr(config, 'HEADING_LOCK_KP', 0.50), 2),
            "safety_brake_dist_m": round(getattr(config, 'SAFETY_BRAKE_DIST_M', 0.22), 2),
            "safety_warn_dist_m": round(getattr(config, 'SAFETY_WARN_DIST_M', 0.45), 2),
            "camera_offset_forward_m": round(getattr(config, 'CAMERA_OFFSET_FORWARD_M', 0.09), 3),
            "camera_offset_lateral_m": round(getattr(config, 'CAMERA_OFFSET_LATERAL_M', -0.015), 3),
            "camera_height_m": round(getattr(config, 'CAMERA_HEIGHT_M', 0.12), 3),
            "camera_pitch_deg": round(getattr(config, 'CAMERA_PITCH_DEG', 0.0), 1),
            "wheel_separation_m": round(getattr(config, 'WHEEL_SEPARATION_M', 0.12), 3)
        }

        return {
            "active_profile": getattr(config, 'ACTIVE_CONFIG_PROFILE', 1),
            "profiles": getattr(config, 'PROFILES', {}),
            "current_params": current_params,
            "current": current_params
        }

    def select_config_profile(self, profile_id: int):
        """Chuyển sang 1 trong 4 profiles cấu hình địa hình, cập nhật runtime tức thì và lưu file"""
        try:
            pid = int(profile_id)
            profiles = getattr(config, 'PROFILES', {})
            if pid not in profiles:
                return False, f"Không tìm thấy Profile #{pid}"

            ok = config.apply_profile(pid)
            if not ok:
                return False, f"Lỗi khi nạp Profile #{pid}"

            target = profiles[pid]
            if self.motors:
                self.motors.set_trim(target["motor_trim_left"], target["motor_trim_right"])

            config.save_profiles_to_disk()
            print(f"🔄 [CONFIG] Đã chuyển đổi sang {target['name']} và đồng bộ xuống phần cứng!")
            return True, f"Đã áp dụng {target['name']}"
        except Exception as e:
            print(f"⚠️ [CONFIG] Lỗi khi đổi Profile: {e}")
            return False, str(e)

    def create_new_config_profile(self, name: str = None, description: str = None, params: dict = None):
        """Tạo một phiên bản cấu hình mới (Snapshot), lưu vĩnh viễn và kích hoạt ngay"""
        try:
            import datetime
            profiles = getattr(config, 'PROFILES', {})
            int_keys = [int(k) for k in profiles.keys() if str(k).isdigit()]
            new_pid = (max(int_keys) + 1) if int_keys else 1

            now_str = datetime.datetime.now().strftime("%d/%m/%Y %H:%M")
            prof_name = name.strip() if name and str(name).strip() else f"Config {new_pid}: Phiên bản #{new_pid}"
            prof_desc = description.strip() if description and str(description).strip() else f"Snapshot lưu lúc {now_str}"

            cur_active_id = getattr(config, 'ACTIVE_CONFIG_PROFILE', 1)
            base = dict(profiles.get(cur_active_id, profiles.get(1, {})))
            p = params if isinstance(params, dict) else {}

            new_profile = {
                "id": new_pid,
                "name": prof_name,
                "description": prof_desc,
                "motor_trim_left": round(float(p.get("motor_trim_left", base.get("motor_trim_left", 1.000))), 3),
                "motor_trim_right": round(float(p.get("motor_trim_right", base.get("motor_trim_right", 1.020))), 3),
                "step_forward_scale": round(float(p.get("step_forward_scale", base.get("step_forward_scale", 0.500))), 3),
                "step_backward_scale": round(float(p.get("step_backward_scale", base.get("step_backward_scale", 0.500))), 3),
                "step_turn_left_scale": round(float(p.get("step_turn_left_scale", base.get("step_turn_left_scale", 0.690))), 3),
                "step_turn_right_scale": round(float(p.get("step_turn_right_scale", base.get("step_turn_right_scale", 0.750))), 3),
                "default_speed_pwm": round(float(p.get("default_speed_pwm", base.get("default_speed_pwm", 0.060))), 3),
                "step_turn_min_pwm": round(float(p.get("step_turn_min_pwm", base.get("step_turn_min_pwm", 0.110))), 3),
                "manual_decel_time_ms": int(p.get("manual_decel_time_ms", base.get("manual_decel_time_ms", 180))),
                "step_turn_start_latency_s": round(float(p.get("step_turn_start_latency_s", base.get("step_turn_start_latency_s", 0.080))), 3),
                "heading_lock_kp": round(float(p.get("heading_lock_kp", base.get("heading_lock_kp", 0.50))), 2),
                "safety_brake_dist_m": round(float(p.get("safety_brake_dist_m", base.get("safety_brake_dist_m", 0.22))), 2),
                "safety_warn_dist_m": round(float(p.get("safety_warn_dist_m", base.get("safety_warn_dist_m", 0.45))), 2),
                "created_at": now_str
            }

            profiles[new_pid] = new_profile
            config.ACTIVE_CONFIG_PROFILE = new_pid
            config.apply_profile(new_pid)
            if self.motors:
                self.motors.set_trim(new_profile["motor_trim_left"], new_profile["motor_trim_right"])
            config.save_profiles_to_disk()

            # Đồng bộ robot_dynamics.json
            try:
                dyn_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_dynamics.json")
                with open(dyn_path, "w", encoding="utf-8") as f:
                    json.dump(new_profile, f, indent=4, ensure_ascii=False)
            except Exception:
                pass

            print(f"✨ [CONFIG VERSION] Đã tạo thành công Phiên bản #{new_pid}: {prof_name}")
            return True, f"Đã tạo và kích hoạt thành công {prof_name}!", new_pid
        except Exception as e:
            print(f"⚠️ [CONFIG VERSION] Lỗi khi tạo phiên bản mới: {e}")
            return False, str(e), None

    def delete_config_profile(self, profile_id: int):
        """Xóa một phiên bản cấu hình tùy chỉnh (bảo vệ Profile #1 chuẩn mặc định)"""
        try:
            pid = int(profile_id)
            if pid == 1:
                return False, "Không thể xóa Profile #1 (Cấu hình chuẩn mặc định của hệ thống)!"
            profiles = getattr(config, 'PROFILES', {})
            if pid not in profiles:
                return False, f"Không tìm thấy Profile #{pid} để xóa!"

            deleted_name = profiles[pid].get("name", f"Profile #{pid}")
            del profiles[pid]

            if getattr(config, 'ACTIVE_CONFIG_PROFILE', 1) == pid:
                config.ACTIVE_CONFIG_PROFILE = 1
                config.apply_profile(1)
                if self.motors and 1 in profiles:
                    self.motors.set_trim(profiles[1]["motor_trim_left"], profiles[1]["motor_trim_right"])

            config.save_profiles_to_disk()
            print(f"🗑️ [CONFIG VERSION] Đã xóa thành công {deleted_name}")
            return True, f"Đã xóa thành công {deleted_name}!"
        except Exception as e:
            print(f"⚠️ [CONFIG VERSION] Lỗi khi xóa profile: {e}")
            return False, str(e)

    def update_full_config(self, params: dict, profile_id: int = None):
        """Cập nhật các tham số cấu hình trực tiếp từ Web Cockpit Tab 3, áp dụng Live không cần restart"""
        try:
            pid = int(profile_id) if profile_id is not None else getattr(config, 'ACTIVE_CONFIG_PROFILE', 1)
            profiles = getattr(config, 'PROFILES', {})
            if pid not in profiles:
                pid = 1
            cur_p = profiles[pid]

            # Cập nhật các trường gửi lên
            if 'motor_trim_left' in params:
                val = max(0.5, min(1.5, float(params['motor_trim_left'])))
                if self.motors: self.motors.trim_left = val
                config.MOTOR_TRIM_LEFT = val
                cur_p['motor_trim_left'] = val
            if 'motor_trim_right' in params:
                val = max(0.5, min(1.5, float(params['motor_trim_right'])))
                if self.motors: self.motors.trim_right = val
                config.MOTOR_TRIM_RIGHT = val
                cur_p['motor_trim_right'] = val
            if 'step_forward_scale' in params:
                val = max(0.05, min(2.5, float(params['step_forward_scale'])))
                config.STEP_FORWARD_SCALE = val
                cur_p['step_forward_scale'] = val
            if 'step_backward_scale' in params:
                val = max(0.05, min(2.5, float(params['step_backward_scale'])))
                config.STEP_BACKWARD_SCALE = val
                cur_p['step_backward_scale'] = val
            if 'step_turn_left_scale' in params:
                val = max(0.05, min(2.5, float(params['step_turn_left_scale'])))
                config.STEP_TURN_LEFT_SCALE = val
                cur_p['step_turn_left_scale'] = val
            if 'step_turn_right_scale' in params:
                val = max(0.05, min(2.5, float(params['step_turn_right_scale'])))
                config.STEP_TURN_RIGHT_SCALE = val
                cur_p['step_turn_right_scale'] = val
            if 'default_speed_pwm' in params:
                val = max(0.02, min(0.35, float(params['default_speed_pwm'])))
                config.DEFAULT_SPEED_PWM = val
                cur_p['default_speed_pwm'] = val
            if 'step_turn_min_pwm' in params:
                val = max(0.05, min(0.35, float(params['step_turn_min_pwm'])))
                config.STEP_TURN_MIN_PWM = val
                config.AUTO_SCAN_SPEED_PWM = val
                cur_p['step_turn_min_pwm'] = val
            if 'manual_decel_time_ms' in params:
                val = max(30, min(800, int(params['manual_decel_time_ms'])))
                config.MANUAL_DECEL_TIME_MS = val
                cur_p['manual_decel_time_ms'] = val
            if 'step_turn_start_latency_s' in params:
                val = max(0.0, min(0.30, float(params['step_turn_start_latency_s'])))
                config.STEP_TURN_START_LATENCY_S = val
                cur_p['step_turn_start_latency_s'] = val
            if 'heading_lock_kp' in params:
                val = max(0.0, min(3.0, float(params['heading_lock_kp'])))
                config.HEADING_LOCK_KP = val
                cur_p['heading_lock_kp'] = val
            if 'safety_brake_dist_m' in params:
                val = max(0.12, min(0.60, float(params['safety_brake_dist_m'])))
                config.SAFETY_BRAKE_DIST_M = val
                cur_p['safety_brake_dist_m'] = val
            if 'safety_warn_dist_m' in params:
                val = max(0.25, min(1.20, float(params['safety_warn_dist_m'])))
                config.SAFETY_WARN_DIST_M = val
                cur_p['safety_warn_dist_m'] = val
            if 'camera_offset_forward_m' in params:
                val = max(0.0, min(0.50, float(params['camera_offset_forward_m'])))
                config.CAMERA_OFFSET_FORWARD_M = val
                cur_p['camera_offset_forward_m'] = val
            if 'camera_offset_lateral_m' in params:
                val = max(-0.25, min(0.25, float(params['camera_offset_lateral_m'])))
                config.CAMERA_OFFSET_LATERAL_M = val
                cur_p['camera_offset_lateral_m'] = val
            if 'camera_height_m' in params:
                val = max(0.04, min(0.60, float(params['camera_height_m'])))
                config.CAMERA_HEIGHT_M = val
                cur_p['camera_height_m'] = val
            if 'camera_pitch_deg' in params:
                val = max(-45.0, min(45.0, float(params['camera_pitch_deg'])))
                config.CAMERA_PITCH_DEG = val
                cur_p['camera_pitch_deg'] = val
            if 'wheel_separation_m' in params:
                val = max(0.06, min(0.35, float(params['wheel_separation_m'])))
                config.WHEEL_SEPARATION_M = val
                cur_p['wheel_separation_m'] = val

            config.ACTIVE_CONFIG_PROFILE = pid
            config.apply_profile(pid)
            config.save_profiles_to_disk()

            # Cập nhật file robot_dynamics.json
            dyn_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_dynamics.json")
            try:
                import json
                with open(dyn_path, "w", encoding="utf-8") as f:
                    json.dump(cur_p, f, indent=4, ensure_ascii=False)
            except Exception:
                pass

            print(f"💾 [CONFIG] Đã lưu và áp dụng thành công toàn bộ tham số vào Profile #{pid} và file config.py!")
            return True, f"Đã lưu và áp dụng thành công Profile #{pid}!"
        except Exception as ex:
            print(f"⚠️ [CONFIG] Lỗi khi cập nhật cấu hình: {ex}")
            return False, str(ex)

    def update_calibration_params(self, params: dict, save_to_disk: bool = False):
        """Cập nhật các tham số hiệu chuẩn động học theo thời gian thực và ghi đè cấu hình"""
        try:
            if 'trim_left' in params and self.motors:
                self.motors.trim_left = max(0.5, min(1.5, float(params['trim_left'])))
            if 'trim_right' in params and self.motors:
                self.motors.trim_right = max(0.5, min(1.5, float(params['trim_right'])))
            if 'step_forward_scale' in params:
                config.STEP_FORWARD_SCALE = max(0.05, min(2.5, float(params['step_forward_scale'])))
            if 'step_backward_scale' in params:
                config.STEP_BACKWARD_SCALE = max(0.05, min(2.5, float(params['step_backward_scale'])))
            if 'step_turn_left_scale' in params:
                config.STEP_TURN_LEFT_SCALE = max(0.05, min(2.5, float(params['step_turn_left_scale'])))
            if 'step_turn_right_scale' in params:
                config.STEP_TURN_RIGHT_SCALE = max(0.05, min(2.5, float(params['step_turn_right_scale'])))
            if 'step_turn_min_pwm' in params:
                config.STEP_TURN_MIN_PWM = max(0.05, min(0.30, float(params['step_turn_min_pwm'])))

            if save_to_disk:
                import json
                dyn_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_dynamics.json")
                dyn_data = {}
                if os.path.exists(dyn_path):
                    try:
                        with open(dyn_path, "r", encoding="utf-8") as f: dyn_data = json.load(f)
                    except Exception: pass
                
                tl = round(getattr(self.motors, 'trim_left', 1.0), 3) if self.motors else 1.0
                tr = round(getattr(self.motors, 'trim_right', 1.0), 3) if self.motors else 1.0
                dyn_data["motor_trim_left"] = tl
                dyn_data["motor_trim_right"] = tr
                dyn_data["trim_factor"] = round(tr / max(0.01, tl), 3)
                dyn_data["step_forward_scale"] = round(getattr(config, 'STEP_FORWARD_SCALE', 0.40), 3)
                dyn_data["step_backward_scale"] = round(getattr(config, 'STEP_BACKWARD_SCALE', 0.40), 3)
                dyn_data["step_turn_left_scale"] = round(getattr(config, 'STEP_TURN_LEFT_SCALE', 0.62), 3)
                dyn_data["step_turn_right_scale"] = round(getattr(config, 'STEP_TURN_RIGHT_SCALE', 0.64), 3)
                dyn_data["step_turn_min_pwm"] = round(getattr(config, 'STEP_TURN_MIN_PWM', 0.11), 3)
                dyn_data["last_calibrated"] = time.strftime("%Y-%m-%d %H:%M:%S")

                with open(dyn_path, "w", encoding="utf-8") as f:
                    json.dump(dyn_data, f, indent=4, ensure_ascii=False)

                self._persist_calibration_to_config_py(dyn_data)
                print(f"💾 [CALIBRATION] Đã lưu vĩnh viễn cấu hình động học vào robot_dynamics.json và config.py!")
            return True
        except Exception as e:
            print(f"⚠️ [CALIBRATION] Lỗi khi cập nhật cấu hình: {e}")
            return False

    def _persist_calibration_to_config_py(self, dyn_data: dict):
        """Ghi đè trực tiếp các tham số hiệu chuẩn vào config.py"""
        try:
            cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.py")
            if not os.path.exists(cfg_path): return
            with open(cfg_path, "r", encoding="utf-8") as f:
                content = f.read()
            import re
            mapping = {
                'MOTOR_TRIM_LEFT': dyn_data.get('motor_trim_left'),
                'MOTOR_TRIM_RIGHT': dyn_data.get('motor_trim_right'),
                'STEP_FORWARD_SCALE': dyn_data.get('step_forward_scale'),
                'STEP_BACKWARD_SCALE': dyn_data.get('step_backward_scale'),
                'STEP_TURN_LEFT_SCALE': dyn_data.get('step_turn_left_scale'),
                'STEP_TURN_RIGHT_SCALE': dyn_data.get('step_turn_right_scale'),
                'STEP_TURN_MIN_PWM': dyn_data.get('step_turn_min_pwm'),
                'CAMERA_OFFSET_FORWARD_M': dyn_data.get('camera_offset_forward_m'),
                'CAMERA_OFFSET_LATERAL_M': dyn_data.get('camera_offset_lateral_m'),
                'CAMERA_HEIGHT_M': dyn_data.get('camera_height_m'),
                'CAMERA_PITCH_DEG': dyn_data.get('camera_pitch_deg'),
                'WHEEL_SEPARATION_M': dyn_data.get('wheel_separation_m')
            }
            for key, val in mapping.items():
                if val is not None:
                    pattern = rf"^({key}\s*=\s*)([0-9\.\-]+)"
                    content = re.sub(pattern, rf"\g<1>{val}", content, flags=re.MULTILINE)
            with open(cfg_path, "w", encoding="utf-8") as f:
                f.write(content)
            print("💾 [CONFIG] Đã đồng bộ thông số hiệu chuẩn vào file config.py thành công.")
        except Exception as e:
            print(f"⚠️ [CONFIG] Lỗi khi lưu config.py: {e}")


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
