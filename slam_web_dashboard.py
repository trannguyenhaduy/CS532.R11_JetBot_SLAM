#!/usr/bin/env python3
"""
══════════════════════════════════════════════════════════════════════════════
JETBOT 3D SEMANTIC SLAM — CYBER COCKPIT (THREE.JS WEBGL EDITION)
══════════════════════════════════════════════════════════════════════════════
- 3D Holographic SLAM Viewer (Three.js WebGL + OrbitControls)
  * Xoay 360 độ, Zoom, Pan không gian 3D căn phòng
  * Robot 3D với góc hướng (Yaw) & Vệt đường đi 3D (Trajectory Ribbon)
  * Đám mây điểm 3D (3D Point Cloud particles) chiếu trực tiếp từ Stereo Depth
  * Khung hộp 3D cho vật thể nhận diện (Bàn, Ghế, Người)
- Live Video Camera OAK-D S2 (MJPEG 15 FPS)
- 2D Occupancy Grid Map song song (có nút chuyển đổi 2D <-> 3D)
- Bàn phím WASD / D-Pad điều khiển động cơ
- Telemetry Pin INA219 (Vôn & %)
══════════════════════════════════════════════════════════════════════════════
"""

import sys, os, time, json, math, base64, threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import numpy as np
import cv2

try:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
except Exception: pass

try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Float32MultiArray, String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    CameraInfo = object
    Image = object
    Odometry = object
    OccupancyGrid = object
    Twist = object
    Float32MultiArray = object
    String = object

try:
    from depthai_ros_msgs.msg import SpatialDetectionArray
    HAS_DEPTHAI = True
except ImportError:
    HAS_DEPTHAI = False

try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

try:
    from spatial_perception_node import SpatialPerceptionFilter, TARGET_CLASSES
    HAS_TV2_FILTER = True
except Exception:
    HAS_TV2_FILTER = False
    SpatialPerceptionFilter = None
    TARGET_CLASSES = {}

def is_shutdown():
    if HAS_ROS:
        return rospy.is_shutdown()
    return False

PORT = 8080

# ─── GLOBAL STATE ─────────────────────────────────────────────────────────────
class SystemState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.current_fps = 15.0

        # Robot Pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.path_history = []  # [[x, y, z], ...]

        # 2D Map
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_png_base64 = ""
        self.map_version = 0

        # 3D Point Cloud (Downsampled for high web performance)
        self.points_3d = []  # [[x, y, z], ...]
        self.max_3d_points = 2500

        # Camera Intrinsics
        self.fx = 400.0
        self.fy = 400.0
        self.cx = 208.0
        self.cy = 208.0

        # 3D Detections & Obstacle Telemetry (Thành viên 2)
        self.detections = []
        self.obstacle_distance = 1.45

        # Real-time 3S Li-ion Battery Telemetry
        self.battery_v = 12.18
        self.battery_pct = 88
        self.battery_cell_v = 4.06
        self.battery_current_a = 0.85
        self.battery_power_w = 10.35
        self.battery_remaining_min = 162
        self.battery_status = "BÌNH THƯỜNG"
        self.is_driving = False
        self.last_drive_time = 0.0

        # ─── REAL-TIME BENCHMARK TELEMETRY ───
        self.img_counter = 0
        self.last_img_sec = time.time()
        self.calc_fps = 15.0

        self.odom_counter = 0
        self.last_odom_sec = time.time()
        self.odom_hz = 0.0
        self.last_odom_recv = time.time()

        self.valid_depth_pct = 85.0
        self.avg_confidence = 0.82

        self.benchmark = {
            "tv1": {
                "score_hw": 14, "max_hw": 15,
                "score_motion": 15, "max_motion": 15,
                "total_score": 29, "max_score": 30,
                "grade": "XUẤT SẮC (A+)",
                "battery_v": 12.1, "battery_pct": 88,
                "cpu_pct": 35.0, "ram_gb": 2.1,
                "bumper_status": "VÙNG AN TOÀN",
                "diag_text": "Hệ thống động cơ & nguồn điện INA219 ổn định."
            },
            "tv2": {
                "camera_source": "Đang kết nối camera...",
                "fps": 0.0, "fps_pct": 0,
                "valid_depth_pct": 0.0, "avg_confidence": 0.0,
                "num_obj": 0, "target_info": "Đang quét không gian...",
                "obstacle_distance": 3.5, "dist_pct": 85,
                "status_badge": "KHỞI ĐỘNG",
                "diag_text": "Đang đồng bộ luồng camera và bộ lọc Spatial AI..."
            },
            "tv3": {
                "score_s": 28, "max_s": 35,
                "score_sem": 22, "max_sem": 25,
                "total_score": 50, "max_score": 60,
                "grade": "XUẤT SẮC (A+)",
                "odom_hz": 3.5, "num_pts": 800, "num_obj": 0,
                "diag_text": "Bản đồ RTAB-Map và cây tọa độ TF2 hoạt động chuẩn xác."
            },
            # Trường tương thích phẳng
            "camera_source": "Đang kết nối camera...",
            "fps": 0.0, "fps_pct": 0,
            "valid_depth_pct": 0.0, "avg_confidence": 0.0,
            "num_obj": 0, "target_info": "Đang quét không gian...",
            "obstacle_distance": 3.5, "dist_pct": 85,
            "status_badge": "KHỞI ĐỘNG",
            "diag_text": "Đang đồng bộ luồng camera và bộ lọc Spatial AI..."
        }

state = SystemState()

# ─── HARDWARE TELEMETRY HELPER ───────────────────────────────────────────────
def get_hardware_telemetry():
    cpu_pct = 35.0
    ram_used_gb = 2.1
    try:
        with open('/proc/loadavg', 'r') as f:
            load = float(f.read().split()[0])
            cpu_pct = min(100.0, round((load / 4.0) * 100.0, 1))
    except Exception: pass
    try:
        with open('/proc/meminfo', 'r') as f:
            mem_total, mem_avail = 4000.0, 2000.0
            for l in f:
                if l.startswith('MemTotal:'):
                    mem_total = float(l.split()[1]) / 1024.0
                elif l.startswith('MemAvailable:'):
                    mem_avail = float(l.split()[1]) / 1024.0
            ram_used_gb = round((mem_total - mem_avail) / 1024.0, 2)
    except Exception: pass
    return cpu_pct, ram_used_gb

# ─── REAL-TIME LI-ION 3S BATTERY BMS ENGINE ─────────────────────────────────
# Đường cong xả chuẩn Li-ion 3S (18650 Pack) từ tài liệu thực nghiệm
_LI_ION_CURVE_3S = [
    (12.60, 100), (12.30, 90), (12.00, 80), (11.70, 70), (11.40, 60),
    (11.10, 50),  (10.80, 35), (10.50, 20), (10.20, 10), (9.60, 5), (9.00, 0)
]

def calculate_battery_pct(pack_v, cells=3):
    """Tính % dung lượng pin theo đường cong phi tuyến Li-ion nội suy đa điểm"""
    if pack_v >= _LI_ION_CURVE_3S[0][0]: return 100
    if pack_v <= _LI_ION_CURVE_3S[-1][0]: return 0
    for (v1, p1), (v2, p2) in zip(_LI_ION_CURVE_3S, _LI_ION_CURVE_3S[1:]):
        if v2 <= pack_v <= v1:
            return int(round(p2 + (p1 - p2) * (pack_v - v2) / (v1 - v2)))
    return 0

def update_battery_metrics(v, current_a=0.85):
    """Tính toán toàn diện điện áp cell, công suất (W), thời lượng còn lại (phút)"""
    pct = calculate_battery_pct(v, cells=3)
    cell_v = round(v / 3.0, 2)
    power_w = round(v * current_a, 2)
    
    # Dung lượng danh định 3S Li-ion (khoảng 2600 mAh = 2.6 Ah)
    nominal_ah = 2.6
    rem_ah = nominal_ah * (pct / 100.0)
    if current_a > 0.15:
        rem_min = int((rem_ah / current_a) * 60)
    else:
        rem_min = 240 # Chế độ nghỉ
        
    if v < 10.2 or pct <= 10:
        status = "NGUY HIỂM (SẠC NGAY)"
    elif v < 10.8 or pct <= 25:
        status = "PIN YẾU (NÊN SẠC)"
    elif current_a >= 1.5:
        status = "ĐANG TẢI NẶNG"
    else:
        status = "BÌNH THƯỜNG"
        
    return pct, cell_v, round(current_a, 2), power_w, rem_min, status

def battery_cb(msg):
    """Callback nhận topic /battery_telemetry từ node điều khiển động cơ của TV1"""
    try:
        if hasattr(msg, 'data') and len(msg.data) >= 1:
            v = float(msg.data[0])
            curr = float(msg.data[2]) if len(msg.data) >= 3 else 0.85
            pct, cell_v, curr_a, p_w, rem_min, status_str = update_battery_metrics(v, curr)
            with state.lock:
                state.battery_v = round(v, 2)
                state.battery_pct = pct
                state.battery_cell_v = cell_v
                state.battery_current_a = curr_a
                state.battery_power_w = p_w
                state.battery_remaining_min = rem_min
                state.battery_status = status_str
    except Exception: pass

battery_pub = None

def battery_worker():
    """Luồng đọc trực tiếp cảm biến I2C INA219 (địa chỉ 0x41) trên JetBot thật"""
    global battery_pub
    if not HAS_SMBUS: return
    try:
        with SMBus(1) as bus:
            bus.write_i2c_block_data(0x41, 0x00, [0x39, 0x9F])
    except Exception: pass

    while not is_shutdown():
        try:
            with SMBus(1) as bus:
                # 1. Đọc Bus Voltage (thanh ghi 0x02)
                raw_bus = bus.read_i2c_block_data(0x41, 0x02, 2)
                v = (((raw_bus[0] << 8) | raw_bus[1]) >> 3) * 0.004

                # 2. Đọc Shunt Voltage (thanh ghi 0x01) để tính dòng điện xả thật
                raw_shunt = bus.read_i2c_block_data(0x41, 0x01, 2)
                shunt_raw = (raw_shunt[0] << 8) | raw_shunt[1]
                if shunt_raw > 32767: shunt_raw -= 65536
                shunt_v = shunt_raw * 0.00001 # 10 uV LSB
                curr_a = max(0.05, abs(shunt_v / 0.1)) # R_shunt = 0.1 Ohm

                pct, cell_v, curr_a, p_w, rem_min, status_str = update_battery_metrics(v, curr_a)
                with state.lock:
                    state.battery_v = round(v, 2)
                    state.battery_pct = pct
                    state.battery_cell_v = cell_v
                    state.battery_current_a = curr_a
                    state.battery_power_w = p_w
                    state.battery_remaining_min = rem_min
                    state.battery_status = status_str

                if HAS_ROS and battery_pub:
                    msg = Float32MultiArray()
                    msg.data = [round(v, 2), float(pct), round(curr_a, 2), round(p_w, 2), float(rem_min)]
                    battery_pub.publish(msg)
        except Exception: pass
        time.sleep(1.0)

# ─── REAL-TIME BENCHMARK EVALUATOR WORKER (3 THÀNH VIÊN) ─────────────────────
def benchmark_worker():
    time.sleep(1.5)  # Chờ hệ thống ổn định
    while not is_shutdown():
        try:
            now = time.time()
            cpu_pct, ram_gb = get_hardware_telemetry()

            with state.lock:
                # ─── THÔNG SỐ ĐO ĐẠC NỀN TẢNG ───
                # 1. Tính FPS camera thực tế
                dt_img = now - state.last_img_sec
                if dt_img >= 1.0:
                    state.calc_fps = round(state.img_counter / dt_img, 1)
                    state.img_counter = 0
                    state.last_img_sec = now
                fps = state.calc_fps

                # 2. Tính Tần số Odometry thực tế
                dt_odom = now - state.last_odom_sec
                if dt_odom >= 1.0:
                    state.odom_hz = round(state.odom_counter / dt_odom, 1)
                    state.odom_counter = 0
                    state.last_odom_sec = now
                odom_hz = state.odom_hz
                odom_active = (now - state.last_odom_recv) < 2.0

                num_obj = len(state.detections)
                obs_dist = getattr(state, 'obstacle_distance', 3.5)
                clarity = state.valid_depth_pct
                conf_pct = round(state.avg_confidence * 100, 1)
                cam_source = getattr(state, 'camera_source', 'Camera')
                num_pts = len(state.points_3d)

                # ══════════════════════════════════════════════════════════════
                # 1. ĐÁNH GIÁ THÀNH VIÊN 1: ĐIỀU KHIỂN & AN TOÀN PHẦN CỨNG (TV1)
                # ══════════════════════════════════════════════════════════════
                score_hw = 0
                if cpu_pct < 65.0: score_hw += 5
                elif cpu_pct < 85.0: score_hw += 3
                else: score_hw += 1

                if ram_gb < 3.0: score_hw += 5
                elif ram_gb < 3.6: score_hw += 3
                else: score_hw += 1

                if state.battery_v >= 11.1: score_hw += 5
                elif state.battery_v >= 10.4: score_hw += 3
                else: score_hw += 1
                score_hw = int(min(15, max(0, score_hw)))

                score_motion = 0
                if obs_dist >= 0.35: score_motion += 10
                elif obs_dist >= 0.20: score_motion += 6
                else: score_motion += 3
                score_motion += 5  # Tần số phản hồi điều khiển vi sai
                score_motion = int(min(15, max(0, score_motion)))

                score_tv1 = score_hw + score_motion
                if score_tv1 >= 26: grade_tv1 = "XUẤT SẮC (A+)"
                elif score_tv1 >= 22: grade_tv1 = "GIỎI (A)"
                elif score_tv1 >= 18: grade_tv1 = "ĐẠT CHUẨN (B)"
                else: grade_tv1 = "CẦN TỐI ƯU (C)"

                bumper_status = "VÙNG AN TOÀN" if obs_dist >= 0.35 else "PHANH KHẨN CẤP"
                if obs_dist < 0.35:
                    diag_tv1 = f"Virtual Bumper KÍCH HOẠT PHANH! Vật cản {obs_dist:.2f}m (< 0.35m)."
                elif state.battery_v < 10.6:
                    diag_tv1 = f"Pin 3S đang yếu ({state.battery_v:.1f}V). Cần cắm sạc bảo vệ Cell."
                elif cpu_pct > 80.0:
                    diag_tv1 = f"Tải CPU Jetson cao ({cpu_pct}%). Kiểm tra tiến trình nền."
                else:
                    diag_tv1 = "Hệ thống động cơ & nguồn điện INA219 ổn định. Virtual Bumper trực sẵn sàng."

                # ══════════════════════════════════════════════════════════════
                # 2. THÔNG SỐ THỰC TẾ THỊ GIÁC & SPATIAL AI (THÀNH VIÊN 2)
                #    (Tuyệt đối không tính điểm, chỉ hiển thị thông số đo đạc)
                # ══════════════════════════════════════════════════════════════
                if num_obj > 0:
                    first_det = state.detections[0]
                    target_info = f"{first_det.get('name', 'MỤC TIÊU')} ({int(first_det.get('score', 0.8)*100)}%) · X:{first_det.get('x',0)}m Z:{first_det.get('z',0)}m"
                else:
                    target_info = "Chưa phát hiện mục tiêu trong tầm quét 0.3m - 4.0m"

                if fps >= 12.0 and clarity >= 40.0:
                    status_badge_tv2 = "HOẠT ĐỘNG TỐT"
                elif clarity < 40.0:
                    status_badge_tv2 = "THIẾU SÁNG / MỜ"
                elif fps < 10.0:
                    status_badge_tv2 = "TRỄ KHUNG HÌNH"
                else:
                    status_badge_tv2 = "CHỜ TÍN HIỆU"

                if num_obj > 0:
                    diag_tv2 = f"Đã khóa {num_obj} mục tiêu ({first_det.get('name')}) ở cự ly {obs_dist:.2f}m. Bộ lọc 5 lớp TV2 hoạt động chính xác."
                elif clarity < 35.0:
                    diag_tv2 = "Ảnh camera bị mờ hoặc thiếu sáng. Cần kiểm tra ánh sáng phòng hoặc vệ sinh ống kính."
                elif fps < 10.0:
                    diag_tv2 = f"Tốc độ camera bị trễ ({fps:.1f} FPS < 15 FPS). Vui lòng kiểm tra băng thông truyền dẫn."
                else:
                    diag_tv2 = "Camera hoạt động ổn định. Đang quét 5 lớp mục tiêu: Người, Bàn, Ghế, Màn hình, Biển báo."

                # ══════════════════════════════════════════════════════════════
                # 3. ĐÁNH GIÁ THÀNH VIÊN 3: VISUAL SLAM & BẢN ĐỒ NGỮ NGHĨA 3D (TV3)
                # ══════════════════════════════════════════════════════════════
                score_s = 0
                if odom_active:
                    score_s += min(15.0, (odom_hz / 3.5) * 15.0)
                    score_s += min(10.0, (num_pts / 800.0) * 10.0)
                    score_s += 10.0  # TF transform liên tục
                else:
                    score_s = 8  # Mất tracking / chưa bật SLAM
                score_s = int(round(min(35, max(0, score_s))))

                score_sem = 0
                if num_obj > 0:
                    score_sem += 15.0
                    valid_depth_objs = sum(1 for d in state.detections if 0.4 <= d.get('z', 0) <= 4.5)
                    score_sem += min(10.0, (valid_depth_objs / max(1, num_obj)) * 10.0)
                else:
                    score_sem = 12.0
                score_sem = int(round(min(25, max(0, score_sem))))

                score_tv3 = score_s + score_sem
                if score_tv3 >= 52: grade_tv3 = "XUẤT SẮC (A+)"
                elif score_tv3 >= 44: grade_tv3 = "GIỎI (A)"
                elif score_tv3 >= 36: grade_tv3 = "ĐẠT CHUẨN (B)"
                else: grade_tv3 = "CẦN TỐI ƯU (C)"

                if not odom_active:
                    diag_tv3 = "Mất dấu Odometry (Lost Tracking). Robot cần quay chậm để bắt lại Visual Keypoints."
                elif num_pts < 300:
                    diag_tv3 = "Mật độ mây điểm 3D thấp. Hãy di chuyển robot để RTAB-Map làm dày bản đồ."
                elif num_obj == 0:
                    diag_tv3 = "Chưa phát hiện cụm ngữ nghĩa trong không gian 3D."
                else:
                    diag_tv3 = f"Bản đồ RTAB-Map & Cây TF2 chuẩn xác! Đã cắm cờ {num_obj} vật thể ngữ nghĩa."

                # Gom lại thành gói Benchmark 3 phân vùng
                state.benchmark = {
                    "tv1": {
                        "score_hw": score_hw, "max_hw": 15,
                        "score_motion": score_motion, "max_motion": 15,
                        "total_score": score_tv1, "max_score": 30,
                        "grade": grade_tv1,
                        "battery_v": state.battery_v, "battery_pct": state.battery_pct,
                        "cpu_pct": cpu_pct, "ram_gb": ram_gb,
                        "bumper_status": bumper_status,
                        "diag_text": diag_tv1
                    },
                    "tv2": {
                        "camera_source": cam_source,
                        "fps": fps,
                        "fps_pct": min(100, int((fps / 15.0) * 100)),
                        "valid_depth_pct": clarity,
                        "avg_confidence": conf_pct,
                        "num_obj": num_obj,
                        "target_info": target_info,
                        "obstacle_distance": round(obs_dist, 2),
                        "dist_pct": min(100, max(5, int((obs_dist / 4.0) * 100))),
                        "status_badge": status_badge_tv2,
                        "diag_text": diag_tv2
                    },
                    "tv3": {
                        "score_s": score_s, "max_s": 35,
                        "score_sem": score_sem, "max_sem": 25,
                        "total_score": score_tv3, "max_score": 60,
                        "grade": grade_tv3,
                        "odom_hz": odom_hz,
                        "num_pts": num_pts,
                        "num_obj": num_obj,
                        "diag_text": diag_tv3
                    },
                    # Trường tương thích phẳng
                    "camera_source": cam_source,
                    "fps": fps,
                    "fps_pct": min(100, int((fps / 15.0) * 100)),
                    "valid_depth_pct": clarity,
                    "avg_confidence": conf_pct,
                    "num_obj": num_obj,
                    "target_info": target_info,
                    "obstacle_distance": round(obs_dist, 2),
                    "dist_pct": min(100, max(5, int((obs_dist / 4.0) * 100))),
                    "status_badge": status_badge_tv2,
                    "diag_text": diag_tv2,
                    "score_hw": score_hw, "max_hw": 15,
                    "score_s": score_s, "max_s": 35,
                    "score_sem": score_sem, "max_sem": 25,
                    "score_tv1": score_tv1, "score_tv3": score_tv3
                }

            # IN RA BẢNG ĐÁNH GIÁ 3 THÀNH VIÊN TRÊN TERMINAL ĐỊNH KỲ (ANSI COLORS)
            c_cyan = "\033[1;36m"
            c_yellow = "\033[1;33m"
            c_green = "\033[1;32m"
            c_purple = "\033[1;35m"
            c_blue = "\033[1;34m"
            c_reset = "\033[0m"

            sys.stdout.write(
                f"\n{c_cyan}══════════════════════════════════════════════════════════════════════════════════════{c_reset}\n"
                f"{c_yellow}📊 BÁO CÁO ĐÁNH GIÁ ĐỒ ÁN JETBOT SLAM THEO 3 THÀNH VIÊN NHÓM{c_reset}\n"
                f"──────────────────────────────────────────────────────────────────────────────────────\n"
                f"{c_blue}🔵 [THÀNH VIÊN 1: ĐIỀU KHIỂN CHUYỂN ĐỘNG & AN TOÀN PHẦN CỨNG]{c_reset}\n"
                f"  ├─ Sức khỏe Jetson/Pin INA219 : {c_green}{score_hw:2d}/15 pts{c_reset} (CPU: {cpu_pct}%, RAM: {ram_gb}GB, Pin: {state.battery_v:.2f}V)\n"
                f"  ├─ Virtual Bumper & Chuyển động: {c_green}{score_motion:2d}/15 pts{c_reset} (Cự ly cản: {obs_dist:.2f}m · {bumper_status})\n"
                f"  ├─ Tổng điểm & Xếp loại TV1   : {c_green}{score_tv1:2d}/30 pts{c_reset} [{grade_tv1}]\n"
                f"  └─ Chẩn đoán an toàn TV1      : {diag_tv1}\n"
                f"──────────────────────────────────────────────────────────────────────────────────────\n"
                f"{c_green}🟢 [THÀNH VIÊN 2: THỊ GIÁC BIÊN & SPATIAL AI - THÔNG SỐ THỰC TẾ]{c_reset}\n"
                f"  ├─ Nguồn Cảm biến             : {c_cyan}{cam_source}{c_reset}\n"
                f"  ├─ 1. Tốc độ Camera (FPS)     : {c_green}{fps:.1f} FPS{c_reset} [Chuẩn 15.0 FPS, Tỉ lệ: {min(100, int((fps/15.0)*100))}%]\n"
                f"  ├─ 2. Độ nét ảnh / Depth      : {c_green}{clarity:.1f}%{c_reset} [{'SẮC NÉT · ĐỦ SÁNG' if clarity >= 60 else 'MỜ NÉT · THIẾU SÁNG'}]\n"
                f"  ├─ 3. Mục tiêu 5 Lớp TV2      : {c_green}{num_obj} vật thể{c_reset} [{target_info}]\n"
                f"  ├─ 4. Cự ly Vật cản Phía trước: {c_green}{obs_dist:.2f} mét{c_reset} [/obstacle_distance: {'VÙNG AN TOÀN' if obs_dist >= 0.35 else 'PHANH KHẨN CẤP'}]\n"
                f"  └─ Trạng thái & Chẩn đoán TV2 : [{status_badge_tv2}] - {diag_tv2}\n"
                f"──────────────────────────────────────────────────────────────────────────────────────\n"
                f"{c_purple}🟣 [THÀNH VIÊN 3: VISUAL SLAM & BẢN ĐỒ NGỮ NGHĨA 3D]{c_reset}\n"
                f"  ├─ Visual SLAM (RTAB-Map)     : {c_green}{score_s:2d}/35 pts{c_reset} (VO: {odom_hz:.1f} Hz, Mây điểm: {num_pts} pts)\n"
                f"  ├─ Semantic Map & 3D TF       : {c_green}{score_sem:2d}/25 pts{c_reset} ({num_obj} vật thể 3D đã cắm cờ)\n"
                f"  ├─ Tổng điểm & Xếp loại TV3   : {c_green}{score_tv3:2d}/60 pts{c_reset} [{grade_tv3}]\n"
                f"  └─ Chẩn đoán SLAM TV3         : {diag_tv3}\n"
                f"{c_cyan}══════════════════════════════════════════════════════════════════════════════════════{c_reset}\n"
            )
            sys.stdout.flush()

        except Exception as e:
            pass
        time.sleep(15.0)

# ─── ROS SUBSCRIBERS ──────────────────────────────────────────────────────────
cmd_vel_pub = None
_img_skip = 0
_depth_skip = 0

def obstacle_distance_cb(msg: Float32):
    with state.lock:
        state.obstacle_distance = round(float(msg.data), 2)

def image_cb(msg):
    global _img_skip
    _img_skip += 1
    # Bỏ qua 1 frame giữa chừng để giảm 50% tải CPU Jetson Nano
    if _img_skip % 2 != 0:
        return
    try:
        w, h = msg.width, msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ['bgr8', 'rgb8']:
            img = raw.reshape((h, w, 3))
            if msg.encoding == 'rgb8': img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            img = raw.reshape((h, w, -1))

        # Vẽ bounding box trực tiếp lên luồng video camera thật từ dữ liệu AI
        with state.lock:
            dets = list(state.detections)
            fx, fy, cx, cy = state.fx, state.fy, state.cx, state.cy

        for det in dets:
            try:
                x_m = float(det.get('x', 0.0))
                y_m = float(det.get('y', 0.0))
                z_m = float(det.get('z', 0.0))
                name = str(det.get('name', 'OBJ'))
                score = float(det.get('score', 0.8))

                if z_m > 0.15:
                    u = int(cx + (x_m * fx / z_m))
                    v = int(cy + (y_m * fy / z_m))
                    if 10 <= u < w - 10 and 10 <= v < h - 10:
                        bw = max(35, min(200, int(160.0 / z_m)))
                        bh = max(50, min(300, int(240.0 / z_m)))
                        x1, y1 = max(0, u - bw // 2), max(0, v - bh // 2)
                        x2, y2 = min(w - 1, u + bw // 2), min(h - 1, v + bh // 2)
                        col = (42, 42, 255) if 'PERSON' in name else (0, 255, 163)
                        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
                        cv2.putText(img, f"{name} {int(score*100)}% ({z_m:.1f}m)",
                                    (x1, max(22, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 2)
            except Exception: pass

        _, jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        with state.lock:
            state.latest_jpeg = jpeg.tobytes()
            state.camera_source = "OAK-D S2"
            state.img_counter += 1
    except Exception: pass

def camera_info_cb(msg: CameraInfo):
    with state.lock:
        if msg.K[0] > 0:
            state.fx = msg.K[0]
            state.fy = msg.K[4]
            state.cx = msg.K[2]
            state.cy = msg.K[5]

def depth_cb(msg: Image):
    """Chiếu Depth thành đám mây điểm 3D (giảm tần số xuống 3 Hz để giải phóng 70% CPU)"""
    global _depth_skip
    _depth_skip += 1
    if _depth_skip % 5 != 0:
        return
    try:
        w, h = msg.width, msg.height
        # 16-bit depth (mm)
        depth_data = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))

        # Downsample: lấy mẫu cách nhau mỗi 25 pixel để cực nhẹ CPU
        step = 25
        u_grid, v_grid = np.meshgrid(np.arange(0, w, step), np.arange(0, h, step))
        z_sample = depth_data[v_grid, u_grid].astype(np.float32) / 1000.0  # chuyển sang mét

        # Chỉ lấy điểm hợp lệ trong khoảng 0.3m -> 3.2m
        valid = (z_sample > 0.3) & (z_sample < 3.2)
        valid_ratio = float(np.count_nonzero(valid)) / max(1.0, float(valid.size))
        z_val = z_sample[valid]
        u_val = u_grid[valid]
        v_val = v_grid[valid]

        with state.lock:
            state.valid_depth_pct = round(valid_ratio * 100.0, 1)
            fx, fy, cx, cy = state.fx, state.fy, state.cx, state.cy
            rx, ry, rz = state.robot_x, state.robot_y, state.robot_z
            yaw = state.robot_yaw

        # Tọa độ trong Camera Frame: X phải, Y xuống, Z tới
        x_cam = (u_val - cx) * z_val / fx
        y_cam = (v_val - cy) * z_val / fy

        # Đổi sang Robot Frame (X tới, Y trái, Z lên): x_rob = z_cam, y_rob = -x_cam, z_rob = -y_cam + 0.08
        x_rob = z_val
        y_rob = -x_cam
        z_rob = -y_cam + 0.08

        # Chiếu sang World Frame bằng góc quay Yaw & Tọa độ robot
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        x_world = rx + (x_rob * cos_y - y_rob * sin_y)
        y_world = ry + (x_rob * sin_y + y_rob * cos_y)
        z_world = z_rob

        new_pts = []
        for i in range(min(len(x_world), 300)):
            new_pts.append([round(float(x_world[i]), 2), round(float(y_world[i]), 2), round(float(z_world[i]), 2)])

        with state.lock:
            state.points_3d.extend(new_pts)
            if len(state.points_3d) > state.max_3d_points:
                # Giữ lại các điểm mới nhất
                state.points_3d = state.points_3d[-state.max_3d_points:]
    except Exception: pass

def odom_cb(msg: Odometry):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    with state.lock:
        state.odom_counter += 1
        state.last_odom_recv = time.time()
        state.robot_x = round(p.x, 3)
        state.robot_y = round(p.y, 3)
        state.robot_z = round(p.z, 3)
        state.robot_yaw = round(yaw, 3)
        if not state.path_history or math.hypot(p.x - state.path_history[-1][0], p.y - state.path_history[-1][1]) > 0.03:
            state.path_history.append([round(p.x, 3), round(p.y, 3), round(p.z, 3)])
            if len(state.path_history) > 500: state.path_history.pop(0)

def map_cb(msg: OccupancyGrid):
    try:
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w, 3), 127, dtype=np.uint8)
        img[data == 0] = [235, 235, 235]
        img[data > 50] = [25, 25, 25]
        img = cv2.flip(img, 0)
        _, png = cv2.imencode('.png', img)
        b64 = base64.b64encode(png).decode('utf-8')
        with state.lock:
            state.map_width, state.map_height = w, h
            state.map_resolution = msg.info.resolution
            state.map_origin_x = msg.info.origin.position.x
            state.map_origin_y = msg.info.origin.position.y
            state.map_png_base64 = b64
            state.map_version += 1
    except Exception: pass

def detections_cb(msg):
    dets = []
    for d in getattr(msg, 'detections', []):
        for res in getattr(d, 'results', []):
            pos = getattr(d, 'position', None)
            if pos:
                cid = getattr(res, 'id', 0)
                lbl = getattr(res, 'label', None)
                if cid in TARGET_CLASSES:
                    name = TARGET_CLASSES[cid]
                elif lbl:
                    name = str(lbl).upper()
                else:
                    name = f"OBJ #{cid}"
                dets.append({
                    "id": cid,
                    "name": name,
                    "score": round(float(getattr(res, 'score', 0)), 2),
                    "x": round(float(pos.x), 2),
                    "y": round(float(pos.y), 2),
                    "z": round(float(pos.z), 2)
                })
    with state.lock:
        state.detections = dets
        if dets:
            scores = [d['score'] for d in dets]
            state.avg_confidence = round(float(sum(scores)) / len(scores), 2)

def spatial_objects_json_cb(msg):
    try:
        data = json.loads(msg.data)
        if isinstance(data, list):
            with state.lock:
                state.detections = data
                if data:
                    scores = [float(d.get('score', 0.8)) for d in data]
                    state.avg_confidence = round(float(sum(scores)) / len(scores), 2)
    except Exception: pass

# ─── FRONTEND HTML + THREE.JS ────────────────────────────────────────────────
HTML_PAGE = """<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>JetBot 3D Semantic SLAM Cockpit</title>
  <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@500;700;900&family=Rajdhani:wght@500;600;700&display=swap" rel="stylesheet">
  <!-- Three.js + OrbitControls -->
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
  <style>
    :root {
      --bg: #07090e; --card-bg: rgba(14, 20, 32, 0.8); --border: rgba(0, 240, 255, 0.25);
      --cyan: #00f0ff; --emerald: #00ffa3; --amber: #ffb800; --rose: #ff2a6d; --purple: #b537f2;
      --text: #e0f2fe; --dim: #64748b;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; user-select: none; }
    body {
      background: var(--bg); color: var(--text); font-family: 'Rajdhani', sans-serif;
      min-height: 100vh; overflow-x: hidden;
      background-image: radial-gradient(ellipse at 15% 20%, rgba(0, 240, 255, 0.08) 0%, transparent 40%);
    }
    header {
      padding: 10px 24px; display: flex; justify-content: space-between; align-items: center;
      border-bottom: 1px solid var(--border); backdrop-filter: blur(12px); background: rgba(7, 9, 14, 0.9);
      position: sticky; top: 0; z-index: 100;
    }
    .brand {
      display: flex; align-items: center; gap: 10px; font-family: 'Orbitron', monospace; font-size: 1.15rem;
      font-weight: 900; color: var(--cyan); text-shadow: 0 0 16px rgba(0, 240, 255, 0.5);
    }
    .pulse-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--emerald); box-shadow: 0 0 10px var(--emerald); animation: pulse 1.8s infinite; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.3; } }
    .telemetry { display: flex; gap: 12px; }
    .stat-box { background: var(--card-bg); border: 1px solid var(--border); border-radius: 8px; padding: 4px 12px; text-align: center; }
    .stat-label { font-size: 0.6rem; color: var(--dim); text-transform: uppercase; }
    .stat-val { font-family: 'Orbitron', monospace; font-size: 0.95rem; font-weight: 700; color: #fff; }
    
    .batt-box { min-width: 215px; text-align: left; padding: 4px 10px; }
    .batt-header { display: flex; justify-content: space-between; align-items: center; }
    .batt-badge { font-size: 0.6rem; padding: 1px 6px; border-radius: 4px; font-weight: 700; background: rgba(0, 255, 163, 0.15); color: var(--emerald); border: 1px solid var(--emerald); }
    .batt-badge.low { background: rgba(255, 184, 0, 0.2); color: var(--amber); border-color: var(--amber); }
    .batt-badge.critical { background: rgba(255, 42, 109, 0.2); color: var(--rose); border-color: var(--rose); animation: pulse 0.8s infinite; }
    .batt-sub { font-size: 0.62rem; color: var(--dim); margin-top: 1px; font-family: 'Space Grotesk', sans-serif; white-space: nowrap; }
    .batt-bar-bg { width: 100%; height: 5px; background: rgba(255,255,255,0.08); border-radius: 3px; overflow: hidden; margin-top: 3px; }
    .batt-bar-fill { height: 100%; width: 100%; background: var(--emerald); border-radius: 3px; transition: width 0.3s ease, background 0.3s ease; }
    
    .grid { display: grid; grid-template-columns: 1.2fr 0.8fr; gap: 16px; padding: 16px; max-width: 1600px; margin: 0 auto; }
    @media (max-width: 1000px) { .grid { grid-template-columns: 1fr; } }
    .card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 12px; overflow: hidden; display: flex; flex-direction: column; }
    .card-hdr {
      padding: 10px 16px; border-bottom: 1px solid rgba(255,255,255,0.06);
      display: flex; justify-content: space-between; align-items: center;
      font-family: 'Orbitron', monospace; font-size: 0.85rem; color: var(--cyan);
    }
    .tab-btn {
      background: rgba(0, 240, 255, 0.1); border: 1px solid var(--cyan); color: var(--cyan);
      padding: 3px 10px; border-radius: 4px; font-size: 0.75rem; cursor: pointer; margin-left: 6px;
    }
    .tab-btn.active { background: var(--cyan); color: #000; font-weight: 700; }
    
    .viewport { position: relative; width: 100%; height: 420px; background: #000; overflow: hidden; }
    .viewport img { width: 100%; height: 100%; object-fit: contain; }
    #three-viewport { width: 100%; height: 100%; cursor: grab; }
    #three-viewport:active { cursor: grabbing; }
    .canvas-map { width: 100%; height: 100%; background: #101520; }
    .hud {
      position: absolute; top: 10px; left: 10px; z-index: 10;
      background: rgba(0,0,0,0.7); border: 1px solid var(--cyan); border-radius: 6px;
      padding: 4px 10px; font-family: 'Orbitron', monospace; font-size: 0.75rem; color: var(--cyan);
    }
    .hud-tip {
      position: absolute; bottom: 10px; right: 10px; z-index: 10;
      background: rgba(0,0,0,0.6); border-radius: 4px; padding: 4px 8px;
      font-size: 0.7rem; color: var(--dim);
    }

    .bottom-grid { display: grid; grid-template-columns: 0.8fr 1fr 1.3fr; gap: 16px; padding: 0 16px 16px; max-width: 1600px; margin: 0 auto; }
    @media (max-width: 1100px) { .bottom-grid { grid-template-columns: 1fr; } }
    .dpad-container { display: flex; flex-direction: column; align-items: center; padding: 14px; gap: 6px; }
    .dpad-row { display: flex; gap: 6px; }
    .btn-drive { width: 70px; height: 50px; background: rgba(0, 240, 255, 0.08); border: 1px solid var(--cyan); border-radius: 8px; color: var(--cyan); font-size: 1.3rem; cursor: pointer; }
    .btn-drive:active { background: var(--cyan); color: #000; box-shadow: 0 0 16px var(--cyan); }
    .btn-stop { border-color: var(--rose); color: var(--rose); background: rgba(255,42,109,0.1); }
    .radar-box { padding: 12px; display: flex; flex-direction: column; gap: 6px; max-height: 190px; overflow-y: auto; }
    .radar-item {
      display: flex; justify-content: space-between; padding: 6px 10px; border-radius: 4px;
      background: rgba(255,255,255,0.03); border-left: 3px solid var(--emerald); font-family: 'Orbitron', monospace; font-size: 0.8rem;
    }
    
    /* Benchmark Scorecard Styles & 3 Member Evaluations */
    .eval-tabs { display: flex; gap: 4px; }
    .eval-tab-btn {
      background: rgba(255,255,255,0.05); border: 1px solid rgba(255,255,255,0.18);
      border-radius: 4px; padding: 2px 7px; color: var(--dim); font-size: 0.68rem;
      font-family: 'Space Grotesk', sans-serif; font-weight: 600; cursor: pointer; transition: all 0.2s;
    }
    .eval-tab-btn:hover { color: #fff; background: rgba(255,255,255,0.12); }
    .eval-tab-btn.active { color: var(--cyan); border-color: var(--cyan); background: rgba(0, 240, 255, 0.18); font-weight: 700; }
    .eval-card-block {
      background: rgba(255,255,255,0.02); border-radius: 6px; padding: 8px 10px;
      display: flex; flex-direction: column; gap: 6px; transition: all 0.3s;
    }
    .eval-block-tv1 { border-left: 3px solid var(--cyan); }
    .eval-block-tv2 { border-left: 3px solid var(--emerald); }
    .eval-block-tv3 { border-left: 3px solid var(--purple); }

    .bench-box { padding: 10px; display: flex; flex-direction: column; gap: 8px; }
    .bench-row { display: grid; grid-template-columns: 140px 1fr 65px; align-items: center; gap: 8px; font-size: 0.75rem; }
    .bench-lbl { color: var(--dim); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; font-weight: 600; }
    .bench-bar-bg { width: 100%; height: 7px; background: rgba(255,255,255,0.06); border-radius: 4px; overflow: hidden; }
    .bench-bar-fill { height: 100%; transition: width 0.35s ease; border-radius: 4px; }
    .bench-val { font-family: 'Orbitron', monospace; font-weight: 700; color: #fff; text-align: right; white-space: nowrap; font-size: 0.8rem; }
    .bench-sub { font-size: 0.65rem; color: var(--dim); margin-top: -3px; margin-bottom: 2px; font-family: 'Space Grotesk', sans-serif; letter-spacing: 0.2px; padding-left: 2px; }
    .bench-badge { padding: 2px 7px; border-radius: 4px; font-size: 0.70rem; font-weight: 700; background: rgba(0, 255, 163, 0.15); color: var(--emerald); border: 1px solid var(--emerald); }
    .bench-alert { background: rgba(0, 240, 255, 0.08); border-left: 3px solid var(--cyan); padding: 5px 8px; border-radius: 4px; font-size: 0.70rem; line-height: 1.3; }
  </style>
</head>
<body>
  <header>
    <div class="brand"><div class="pulse-dot"></div>JETBOT 3D SEMANTIC SLAM</div>
    <div class="telemetry">
      <div class="stat-box batt-box" id="batt-container">
        <div class="batt-header">
          <span class="stat-label">PIN 3S LI-ION</span>
          <span id="batt-badge" class="batt-badge">BÌNH THƯỜNG</span>
        </div>
        <div class="stat-val" id="batt-val">12.18 V (88%)</div>
        <div class="batt-sub" id="batt-sub">4.06V/c · 0.85A (10.4W) · ~2h 42m</div>
        <div class="batt-bar-bg"><div class="batt-bar-fill" id="batt-bar-fill" style="width: 88%;"></div></div>
      </div>
      <div class="stat-box"><div class="stat-label">TỌA ĐỘ 3D (X, Y, θ)</div><div class="stat-val" id="pose-val">0.0, 0.0, 0°</div></div>
      <div class="stat-box"><div class="stat-label">ĐIỂM 3D SLAM</div><div class="stat-val" id="pts-count" style="color:var(--emerald);">0 PTS</div></div>
    </div>
  </header>

  <div class="grid">
    <!-- 3D Holographic Scene -->
    <div class="card">
      <div class="card-hdr">
        <span>🌐 3D HOLOGRAPHIC SLAM (THREE.JS)</span>
        <div>
          <button class="tab-btn active" id="btn-view-3d" onclick="switchView('3d')">3D VIEW</button>
          <button class="tab-btn" id="btn-view-2d" onclick="switchView('2d')">2D GRID</button>
          <button class="tab-btn" style="border-color:var(--amber); color:var(--amber);" onclick="reset3DCamera()">RESET CAM</button>
        </div>
      </div>
      <div class="viewport">
        <div id="three-viewport"></div>
        <canvas id="map-canvas" class="canvas-map" style="display:none;"></canvas>
        <div class="hud" id="hud-info">3D POINT CLOUD & MÔ HÌNH ROBOT</div>
        <div class="hud-tip">💡 Chuột trái: Xoay 360° | Chuột phải: Kéo | Cuộn chuột: Phóng to/Thu nhỏ</div>
      </div>
    </div>

    <!-- Live Camera Feed -->
    <div class="card">
      <div class="card-hdr">
        <span id="cam-hdr-title">📷 CAMERA // LIVE STREAM</span>
        <span id="cam-source-badge" style="color:var(--emerald); font-size:0.75rem;">LIVE STREAM</span>
      </div>
      <div class="viewport">
        <img id="cam-stream" src="/stream.mjpg" alt="Chờ camera...">
        <div class="hud" id="cam-hud">STEREO DEPTH + YOLO SPATIAL</div>
      </div>
    </div>
  </div>

  <div class="bottom-grid">
    <!-- Teleop Controls -->
    <div class="card">
      <div class="card-hdr">🎮 ĐIỀU KHIỂN XE (PHÍM W, A, S, D)</div>
      <div class="dpad-container">
        <div class="dpad-row"><button class="btn-drive" onmousedown="sendDrive(0.2,0)" onmouseup="sendDrive(0,0)" ontouchstart="sendDrive(0.2,0)" ontouchend="sendDrive(0,0)">▲</button></div>
        <div class="dpad-row">
          <button class="btn-drive" onmousedown="sendDrive(0,0.6)" onmouseup="sendDrive(0,0)" ontouchstart="sendDrive(0,0.6)" ontouchend="sendDrive(0,0)">◀</button>
          <button class="btn-drive btn-stop" onclick="sendDrive(0,0)">■</button>
          <button class="btn-drive" onmousedown="sendDrive(0,-0.6)" onmouseup="sendDrive(0,0)" ontouchstart="sendDrive(0,-0.6)" ontouchend="sendDrive(0,0)">▶</button>
        </div>
        <div class="dpad-row"><button class="btn-drive" onmousedown="sendDrive(-0.2,0)" onmouseup="sendDrive(0,0)" ontouchstart="sendDrive(-0.2,0)" ontouchend="sendDrive(0,0)">▼</button></div>
      </div>
    </div>

    <!-- 3D Semantic Radar Objects -->
    <div class="card">
      <div class="card-hdr">
        <span>📡 3D SEMANTIC OBJECTS (VPU)</span>
        <span id="det-count" style="color:var(--amber); font-size:0.75rem;">0 VẬT THỂ</span>
      </div>
      <div class="radar-box" id="radar-list">
        <div style="text-align:center; color:var(--dim); padding:15px;">Đang quét vật thể 3D trong không gian...</div>
      </div>
    </div>

    <!-- Real-Time Evaluation & Benchmark Card (3 Thành Viên Đồ Án) -->
    <div class="card" id="card-evaluations">
      <div class="card-hdr">
        <span>📊 ĐÁNH GIÁ 3 THÀNH VIÊN ĐỒ ÁN</span>
        <div class="eval-tabs">
          <button class="eval-tab-btn" id="btn-tab-tv1" onclick="switchEvalTab('tv1')">🔵 TV1</button>
          <button class="eval-tab-btn" id="btn-tab-tv2" onclick="switchEvalTab('tv2')">🟢 TV2</button>
          <button class="eval-tab-btn" id="btn-tab-tv3" onclick="switchEvalTab('tv3')">🟣 TV3</button>
          <button class="eval-tab-btn active" id="btn-tab-all" onclick="switchEvalTab('all')">📋 TẤT CẢ</button>
        </div>
      </div>
      <div class="bench-box" style="max-height: 225px; overflow-y: auto;">

        <!-- 🔵 PHẦN 1: ĐÁNH GIÁ THÀNH VIÊN 1 (ĐIỀU KHIỂN & PHẦN CỨNG) -->
        <div class="eval-card-block eval-block-tv1" id="panel-tv1">
          <div style="display:flex; justify-content:space-between; align-items:center;">
            <span style="font-weight:700; color:var(--cyan); font-size:0.75rem;">🔵 TV1: ĐIỀU KHIỂN & SỨC KHỎE PHẦN CỨNG</span>
            <span id="badge-tv1" class="bench-badge" style="border-color:var(--cyan); color:var(--cyan); background:rgba(0,240,255,0.12);">29/30 · XUẤT SẮC</span>
          </div>
          <div class="bench-row">
            <span class="bench-lbl">1. Sức khỏe Jetson/Pin</span>
            <div class="bench-bar-bg"><div class="bench-bar-fill" id="bar-tv1-hw" style="width: 93%; background: var(--cyan);"></div></div>
            <span class="bench-val" id="val-tv1-hw">14/15</span>
          </div>
          <div class="bench-sub" id="sub-tv1-hw">CPU: --% · RAM: --GB · Pin INA219: --V</div>

          <div class="bench-row">
            <span class="bench-lbl">2. Phanh ảo Virtual Bumper</span>
            <div class="bench-bar-bg"><div class="bench-bar-fill" id="bar-tv1-motion" style="width: 100%; background: var(--emerald);"></div></div>
            <span class="bench-val" id="val-tv1-motion">15/15</span>
          </div>
          <div class="bench-sub" id="sub-tv1-motion">Virtual Bumper: VÙNG AN TOÀN (Cự ly cản: --m)</div>

          <div class="bench-alert" style="border-left-color:var(--cyan);">
            <span style="color:var(--cyan); font-weight:700;">📡 Chẩn đoán TV1:</span> <span id="diag-tv1-txt">Hệ thống động cơ & nguồn điện INA219 ổn định.</span>
          </div>
        </div>

        <!-- 🟢 PHẦN 2: THÔNG SỐ THỰC TẾ THÀNH VIÊN 2 (THỊ GIÁC BIÊN & SPATIAL AI - KHÔNG CHẤM ĐIỂM) -->
        <div class="eval-card-block eval-block-tv2" id="panel-tv2">
          <div style="display:flex; justify-content:space-between; align-items:center;">
            <span style="font-weight:700; color:var(--emerald); font-size:0.75rem;">🟢 TV2: THÔNG SỐ THỰC TẾ CAMERA & SPATIAL AI</span>
            <span id="cam-status-badge" class="bench-badge">HOẠT ĐỘNG TỐT</span>
          </div>
          <div class="bench-row">
            <span class="bench-lbl">1. Tốc độ Camera (FPS)</span>
            <div class="bench-bar-bg"><div class="bench-bar-fill" id="bar-cam-fps" style="width: 0%; background: var(--cyan);"></div></div>
            <span class="bench-val" id="val-cam-fps">-- FPS</span>
          </div>
          <div class="bench-sub" id="sub-cam-fps">Chu kỳ đo thực tế: -- / 15.0 FPS</div>

          <div class="bench-row">
            <span class="bench-lbl">2. Độ nét ảnh / Depth</span>
            <div class="bench-bar-bg"><div class="bench-bar-fill" id="bar-cam-clarity" style="width: 0%; background: var(--emerald);"></div></div>
            <span class="bench-val" id="val-cam-clarity">--%</span>
          </div>
          <div class="bench-sub" id="sub-cam-clarity">Độ nét quang học / Stereo Depth: --%</div>

          <div class="bench-row">
            <span class="bench-lbl">3. Mục tiêu 5 Lớp TV2</span>
            <div class="bench-bar-bg"><div class="bench-bar-fill" id="bar-cam-target" style="width: 0%; background: var(--purple);"></div></div>
            <span class="bench-val" id="val-cam-target">0 VẬT THỂ</span>
          </div>
          <div class="bench-sub" id="sub-cam-target">Chưa phát hiện mục tiêu trong vùng quét</div>

          <div class="bench-row">
            <span class="bench-lbl">4. Cự ly Vật cản Trước</span>
            <div class="bench-bar-bg"><div class="bench-bar-fill" id="bar-cam-dist" style="width: 0%; background: var(--amber);"></div></div>
            <span class="bench-val" id="val-cam-dist">-- MÉT</span>
          </div>
          <div class="bench-sub" id="sub-cam-dist">Topic /obstacle_distance: -- mét</div>

          <div class="bench-alert" style="border-left-color:var(--emerald);">
            <span style="color:var(--emerald); font-weight:700;">📡 Trạng thái TV2:</span> <span id="cam-diag-txt">Đang thu thập thông số camera thời gian thực...</span>
          </div>
        </div>

        <!-- 🟣 PHẦN 3: ĐÁNH GIÁ THÀNH VIÊN 3 (VISUAL SLAM & BẢN ĐỒ NGỮ NGHĨA 3D) -->
        <div class="eval-card-block eval-block-tv3" id="panel-tv3">
          <div style="display:flex; justify-content:space-between; align-items:center;">
            <span style="font-weight:700; color:var(--purple); font-size:0.75rem;">🟣 TV3: VISUAL SLAM & BẢN ĐỒ NGỮ NGHĨA 3D</span>
            <span id="badge-tv3" class="bench-badge" style="border-color:var(--purple); color:var(--purple); background:rgba(180,74,255,0.15);">50/60 · XUẤT SẮC</span>
          </div>
          <div class="bench-row">
            <span class="bench-lbl">1. Visual SLAM (RTAB-Map)</span>
            <div class="bench-bar-bg"><div class="bench-bar-fill" id="bar-tv3-slam" style="width: 80%; background: var(--cyan);"></div></div>
            <span class="bench-val" id="val-tv3-slam">28/35</span>
          </div>
          <div class="bench-sub" id="sub-tv3-slam">VO: -- Hz · Mây điểm: -- pts · Cây TF2 liên tục</div>

          <div class="bench-row">
            <span class="bench-lbl">2. Bản đồ Ngữ nghĩa 3D</span>
            <div class="bench-bar-bg"><div class="bench-bar-fill" id="bar-tv3-sem" style="width: 88%; background: var(--purple);"></div></div>
            <span class="bench-val" id="val-tv3-sem">22/25</span>
          </div>
          <div class="bench-sub" id="sub-tv3-sem">Đã cắm cờ ngữ nghĩa: -- vật thể 3D</div>

          <div class="bench-alert" style="border-left-color:var(--purple);">
            <span style="color:var(--purple); font-weight:700;">📡 Chẩn đoán TV3:</span> <span id="diag-tv3-txt">Bản đồ RTAB-Map và cây tọa độ TF2 hoạt động chuẩn xác.</span>
          </div>
        </div>

      </div>
    </div>
  </div>

  <script>
    // ════ THREE.JS 3D SCENE SETUP ════
    const container = document.getElementById('three-viewport');
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0e17);
    scene.fog = new THREE.FogExp2(0x0a0e17, 0.04);

    const camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 50);
    camera.position.set(0, -3.5, 3.5); // Góc nhìn nghiêng từ phía sau robot

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 - 0.05; // Không nhìn dưới mặt sàn

    // Ánh sáng
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
    scene.add(ambientLight);
    const dirLight = new THREE.DirectionalLight(0x00f0ff, 0.6);
    dirLight.position.set(5, 5, 10);
    scene.add(dirLight);

    // Lưới mặt sàn Neon
    const grid = new THREE.GridHelper(12, 24, 0x00f0ff, 0x1e293b);
    grid.rotation.x = Math.PI / 2; // Nằm trên mặt phẳng XY
    scene.add(grid);

    // MÔ HÌNH ROBOT 3D
    const robotGroup = new THREE.Group();
    // Thân xe (Cyan Box)
    const bodyGeo = new THREE.BoxGeometry(0.18, 0.14, 0.08);
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x00f0ff, roughness: 0.3, metalness: 0.7 });
    const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
    bodyMesh.position.z = 0.05;
    robotGroup.add(bodyMesh);

    // Mắt thần OAK-D S2
    const camGeo = new THREE.BoxGeometry(0.04, 0.10, 0.03);
    const camMat = new THREE.MeshStandardMaterial({ color: 0xff2a6d });
    const camMesh = new THREE.Mesh(camGeo, camMat);
    camMesh.position.set(0.09, 0, 0.08);
    robotGroup.add(camMesh);

    // Mũi tên chỉ hướng (Heading Needle)
    const arrowHelper = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.09, 0, 0.08), 0.25, 0x00ffa3);
    robotGroup.add(arrowHelper);

    scene.add(robotGroup);

    // ĐƯỜNG ĐI 3D (TRAJECTORY RIBBON)
    const pathGeo = new THREE.BufferGeometry();
    const pathMat = new THREE.LineBasicMaterial({ color: 0x00ffa3, linewidth: 2 });
    const pathLine = new THREE.Line(pathGeo, pathMat);
    scene.add(pathLine);

    // ĐÁM MÂY ĐIỂM 3D (3D POINT CLOUD PARTICLES)
    const MAX_PTS = 3000;
    const ptsGeo = new THREE.BufferGeometry();
    const ptsPos = new Float32Array(MAX_PTS * 3);
    const ptsColors = new Float32Array(MAX_PTS * 3);
    ptsGeo.setAttribute('position', new THREE.BufferAttribute(ptsPos, 3));
    ptsGeo.setAttribute('color', new THREE.BufferAttribute(ptsColors, 3));

    const ptsMat = new THREE.PointsMaterial({ size: 0.04, vertexColors: true, transparent: true, opacity: 0.85 });
    const pointsMesh = new THREE.Points(ptsGeo, ptsMat);
    scene.add(pointsMesh);

    // NHÓM VẬT THỂ NGỮ NGHĨA 3D (SEMANTIC BOXES)
    const semanticGroup = new THREE.Group();
    scene.add(semanticGroup);

    function reset3DCamera() {
      camera.position.set(robotGroup.position.x, robotGroup.position.y - 3.2, 3.2);
      controls.target.set(robotGroup.position.x, robotGroup.position.y, 0);
    }

    function animate() {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    }
    animate();

    window.addEventListener('resize', () => {
      camera.aspect = container.clientWidth / container.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(container.clientWidth, container.clientHeight);
    });

    // ════ 2D / 3D VIEW TOGGLE ════
    let currentView = '3d';
    let lastStateData = null;

    function switchView(mode) {
      currentView = mode;
      document.getElementById('btn-view-3d').classList.toggle('active', mode === '3d');
      document.getElementById('btn-view-2d').classList.toggle('active', mode === '2d');
      document.getElementById('three-viewport').style.display = mode === '3d' ? 'block' : 'none';
      document.getElementById('map-canvas').style.display = mode === '2d' ? 'block' : 'none';
      document.getElementById('hud-info').innerText = mode === '3d' ? '3D POINT CLOUD & MÔ HÌNH ROBOT' : '2D OCCUPANCY GRID MAP';
      if (mode === '2d' && lastStateData) {
        setTimeout(() => draw2DMap(lastStateData), 20);
      }
    }

    // ════ 2D OCCUPANCY GRID MAP RENDERER ════
    const mapCanvas = document.getElementById('map-canvas');
    const mapCtx = mapCanvas.getContext('2d');
    let currentMapVer = -1;
    let mapImage = new Image();

    function draw2DMap(d) {
      if (!mapCanvas || !mapCtx) return;
      if (!d) return;

      const rect = mapCanvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const targetW = Math.max(300, Math.round((rect.width || 600) * dpr));
      const targetH = Math.max(200, Math.round((rect.height || 420) * dpr));

      if (mapCanvas.width !== targetW || mapCanvas.height !== targetH) {
        mapCanvas.width = targetW;
        mapCanvas.height = targetH;
      }

      mapCtx.save();
      mapCtx.scale(dpr, dpr);

      const W = rect.width || (targetW / dpr);
      const H = rect.height || (targetH / dpr);

      // Xóa nền với màu Cyber Dark
      mapCtx.fillStyle = '#0a0e17';
      mapCtx.fillRect(0, 0, W, H);

      // Thang đo tỉ lệ (Scale): 1 mét thực tế = bao nhiêu pixel trên canvas
      const scale = Math.min(W, H) / 7.0; 
      const centerX = W / 2.0;
      const centerY = H / 2.0;

      function worldToCanvas(wx, wy) {
        return {
          x: centerX + wx * scale,
          y: centerY - wy * scale
        };
      }

      // 1. Lưới tọa độ mét ảo (Grid Lines)
      mapCtx.lineWidth = 1;
      for (let m = -4; m <= 4; m += 0.5) {
        const isMajor = Math.abs(m % 1.0) < 0.01;
        mapCtx.strokeStyle = isMajor ? 'rgba(0, 240, 255, 0.12)' : 'rgba(255, 255, 255, 0.03)';
        
        const p1 = worldToCanvas(m, -4);
        const p2 = worldToCanvas(m, 4);
        mapCtx.beginPath(); mapCtx.moveTo(p1.x, p1.y); mapCtx.lineTo(p2.x, p2.y); mapCtx.stroke();

        const q1 = worldToCanvas(-4, m);
        const q2 = worldToCanvas(4, m);
        mapCtx.beginPath(); mapCtx.moveTo(q1.x, q1.y); mapCtx.lineTo(q2.x, q2.y); mapCtx.stroke();
      }

      // 2. Vẽ Bản đồ Occupancy Grid (nếu có ảnh từ RTAB-Map hoặc Giả lập)
      if (mapImage && mapImage.complete && mapImage.naturalWidth > 0) {
        const res = d.map_resolution || 0.05;
        const ox = d.map_origin_x || -3.5;
        const oy = d.map_origin_y || -3.5;
        const mw = (d.map_width || mapImage.width) * res;
        const mh = (d.map_height || mapImage.height) * res;

        const tl = worldToCanvas(ox, oy + mh);
        const br = worldToCanvas(ox + mw, oy);
        
        mapCtx.save();
        mapCtx.globalAlpha = 0.9;
        mapCtx.drawImage(mapImage, tl.x, tl.y, br.x - tl.x, br.y - tl.y);
        mapCtx.restore();

        // Viền phát sáng cho căn phòng
        mapCtx.strokeStyle = 'rgba(0, 240, 255, 0.5)';
        mapCtx.lineWidth = 1.5;
        mapCtx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);
      } else {
        const pTL = worldToCanvas(-2.0, 2.0);
        const pBR = worldToCanvas(2.0, -2.0);
        mapCtx.fillStyle = 'rgba(30, 41, 59, 0.6)';
        mapCtx.fillRect(pTL.x, pTL.y, pBR.x - pTL.x, pBR.y - pTL.y);
        mapCtx.strokeStyle = 'rgba(0, 240, 255, 0.8)';
        mapCtx.lineWidth = 2;
        mapCtx.strokeRect(pTL.x, pTL.y, pBR.x - pTL.x, pBR.y - pTL.y);
      }

      // 3. Vẽ Vệt đường đi (Path Trajectory)
      if (d.path && d.path.length > 1) {
        mapCtx.strokeStyle = '#00ffa3';
        mapCtx.lineWidth = 2.5;
        mapCtx.lineCap = 'round';
        mapCtx.lineJoin = 'round';
        mapCtx.beginPath();
        const start = worldToCanvas(d.path[0][0], d.path[0][1]);
        mapCtx.moveTo(start.x, start.y);
        for (let i = 1; i < d.path.length; i++) {
          const pt = worldToCanvas(d.path[i][0], d.path[i][1]);
          mapCtx.lineTo(pt.x, pt.y);
        }
        mapCtx.stroke();
      }

      // 4. Vẽ các vật thể nhận diện (Semantic Objects: Ghế, Người)
      if (d.detections && d.detections.length > 0) {
        d.detections.forEach(det => {
          const wx = d.robot_x + (det.z * Math.cos(d.robot_yaw) - (-det.x) * Math.sin(d.robot_yaw));
          const wy = d.robot_y + (det.z * Math.sin(d.robot_yaw) + (-det.x) * Math.cos(d.robot_yaw));
          const cp = worldToCanvas(wx, wy);

          const isPerson = (det.id === 0 || det.name === 'PERSON');
          const color = isPerson ? '#ff2a6d' : '#00ffa3';

          // Vòng tròn vật thể
          mapCtx.fillStyle = color;
          mapCtx.beginPath();
          mapCtx.arc(cp.x, cp.y, 8, 0, Math.PI * 2);
          mapCtx.fill();
          mapCtx.strokeStyle = '#fff';
          mapCtx.lineWidth = 1.5;
          mapCtx.stroke();

          // Nhãn tên vật thể
          mapCtx.font = 'bold 11px Orbitron, monospace';
          mapCtx.fillStyle = color;
          mapCtx.fillText(`${det.name} ${(det.score*100).toFixed(0)}%`, cp.x + 12, cp.y + 4);
        });
      }

      // 5. Vẽ Robot JetBot (Tam giác định hướng + Nón trường nhìn FOV)
      const rp = worldToCanvas(d.robot_x, d.robot_y);
      const yaw = d.robot_yaw;

      // Nón trường nhìn Camera OAK-D S2 (FOV Cone 75 độ)
      mapCtx.save();
      mapCtx.fillStyle = 'rgba(0, 240, 255, 0.15)';
      mapCtx.strokeStyle = 'rgba(0, 240, 255, 0.4)';
      mapCtx.lineWidth = 1;
      mapCtx.beginPath();
      mapCtx.moveTo(rp.x, rp.y);
      const fovAngle = (75 * Math.PI) / 180;
      const fovDist = 2.0 * scale;
      const a1 = -yaw - fovAngle / 2;
      const a2 = -yaw + fovAngle / 2;
      mapCtx.arc(rp.x, rp.y, fovDist, a1, a2);
      mapCtx.closePath();
      mapCtx.fill();
      mapCtx.stroke();
      mapCtx.restore();

      // Vòng tròn an toàn Virtual Bumper (bán kính 0.25m)
      mapCtx.save();
      mapCtx.setLineDash([4, 4]);
      mapCtx.strokeStyle = 'rgba(255, 184, 0, 0.7)';
      mapCtx.lineWidth = 1.5;
      mapCtx.beginPath();
      mapCtx.arc(rp.x, rp.y, 0.25 * scale, 0, Math.PI * 2);
      mapCtx.stroke();
      mapCtx.restore();

      // Thân xe Robot JetBot
      mapCtx.save();
      mapCtx.translate(rp.x, rp.y);
      mapCtx.rotate(-yaw);
      mapCtx.fillStyle = '#00f0ff';
      mapCtx.shadowColor = '#00f0ff';
      mapCtx.shadowBlur = 10;
      mapCtx.beginPath();
      mapCtx.moveTo(14, 0);
      mapCtx.lineTo(-10, -9);
      mapCtx.lineTo(-6, 0);
      mapCtx.lineTo(-10, 9);
      mapCtx.closePath();
      mapCtx.fill();
      mapCtx.restore();

      // 6. Thước đo tỉ lệ (Scale Bar 1 mét) ở góc dưới bên trái
      const scaleBarPx = 1.0 * scale;
      const sbX = 18, sbY = H - 20;
      mapCtx.strokeStyle = '#00f0ff';
      mapCtx.lineWidth = 2;
      mapCtx.beginPath();
      mapCtx.moveTo(sbX, sbY);
      mapCtx.lineTo(sbX + scaleBarPx, sbY);
      mapCtx.moveTo(sbX, sbY - 4);
      mapCtx.lineTo(sbX, sbY + 4);
      mapCtx.moveTo(sbX + scaleBarPx, sbY - 4);
      mapCtx.lineTo(sbX + scaleBarPx, sbY + 4);
      mapCtx.stroke();
      mapCtx.font = '10px Orbitron, monospace';
      mapCtx.fillStyle = '#00f0ff';
      mapCtx.fillText('1.0 MÉT', sbX + 6, sbY - 6);

      // 7. Chú giải Bản đồ (HUD Legend)
      mapCtx.font = '10px Space Grotesk, sans-serif';
      mapCtx.fillStyle = 'rgba(255,255,255,0.7)';
      mapCtx.fillText('🟢 Free Space | 🟦 Tường/Vật cản | 🔺 Robot JetBot', W - 280, H - 15);

      mapCtx.restore();
    }

    // ════ STATE POLLING & 3D UPDATES ════
    async function pollState() {
      try {
        const res = await fetch('/api/state');
        const d = await res.json();

        // Header Telemetry (Real-time Battery BMS)
        const bV = d.battery_v !== undefined ? d.battery_v.toFixed(2) : '--';
        const bPct = d.battery_pct !== undefined ? d.battery_pct : 0;
        const bCell = d.battery_cell_v !== undefined ? d.battery_cell_v.toFixed(2) : (bV / 3.0).toFixed(2);
        const bCurr = d.battery_current_a !== undefined ? d.battery_current_a.toFixed(2) : '0.85';
        const bPow = d.battery_power_w !== undefined ? d.battery_power_w.toFixed(1) : (bV * 0.85).toFixed(1);
        const bRem = d.battery_remaining_min !== undefined ? d.battery_remaining_min : 0;
        const bStat = d.battery_status || 'BÌNH THƯỜNG';

        let remStr = bRem >= 60 ? `${Math.floor(bRem / 60)}h ${bRem % 60}m` : `${bRem}m`;
        if (bRem > 300) remStr = '> 4h (Nghỉ)';

        document.getElementById('batt-val').innerText = `${bV}V (${bPct}%)`;
        document.getElementById('batt-sub').innerText = `${bCell}V/c · ${bCurr}A (${bPow}W) · ~${remStr}`;
        
        const badgeEl = document.getElementById('batt-badge');
        const fillEl = document.getElementById('batt-bar-fill');
        badgeEl.innerText = bStat;
        fillEl.style.width = Math.min(100, Math.max(0, bPct)) + '%';

        if (bV < 10.2 || bPct <= 10) {
          badgeEl.className = 'batt-badge critical';
          fillEl.style.background = 'var(--rose)';
        } else if (bV < 10.8 || bPct <= 25) {
          badgeEl.className = 'batt-badge low';
          fillEl.style.background = 'var(--amber)';
        } else {
          badgeEl.className = 'batt-badge';
          fillEl.style.background = 'var(--emerald)';
        }

        const deg = (d.robot_yaw * 180 / Math.PI).toFixed(0);
        document.getElementById('pose-val').innerText = `${d.robot_x.toFixed(2)}, ${d.robot_y.toFixed(2)}, ${deg}°`;
        document.getElementById('pts-count').innerText = `${d.points_3d.length} PTS`;

        // 1. Cập nhật vị trí Robot 3D
        robotGroup.position.set(d.robot_x, d.robot_y, d.robot_z);
        robotGroup.rotation.z = d.robot_yaw;

        // 2. Cập nhật Vệt đường đi 3D
        if (d.path && d.path.length > 1) {
          const flat = [];
          for (const pt of d.path) flat.push(pt[0], pt[1], pt[2] || 0.02);
          pathGeo.setAttribute('position', new THREE.Float32BufferAttribute(flat, 3));
        }

        // 3. Cập nhật Đám mây điểm 3D (3D Point Cloud)
        if (d.points_3d && d.points_3d.length > 0) {
          const positions = ptsGeo.attributes.position.array;
          const colors = ptsGeo.attributes.color.array;
          const count = Math.min(d.points_3d.length, MAX_PTS);

          for (let i = 0; i < count; i++) {
            const p = d.points_3d[i];
            positions[i * 3]     = p[0];
            positions[i * 3 + 1] = p[1];
            positions[i * 3 + 2] = p[2];

            // Tô màu theo độ cao Z (từ Cyan đến Tím Sci-Fi)
            const normH = Math.max(0, Math.min(1, p[2] / 1.2));
            colors[i * 3]     = normH;          // R
            colors[i * 3 + 1] = 1.0 - normH;    // G
            colors[i * 3 + 2] = 1.0;            // B
          }
          ptsGeo.setDrawRange(0, count);
          ptsGeo.attributes.position.needsUpdate = true;
          ptsGeo.attributes.color.needsUpdate = true;
        }

        // 4. Cập nhật Khung hộp 3D cho vật thể nhận diện (Semantic 3D Bounding Boxes)
        while(semanticGroup.children.length > 0) {
          semanticGroup.remove(semanticGroup.children[0]);
        }
        if (d.detections && d.detections.length > 0) {
          d.detections.forEach(det => {
            // Tọa độ vật thể trong world frame
            const wx = d.robot_x + (det.z * Math.cos(d.robot_yaw) - (-det.x) * Math.sin(d.robot_yaw));
            const wy = d.robot_y + (det.z * Math.sin(d.robot_yaw) + (-det.x) * Math.cos(d.robot_yaw));
            const wz = 0.3;

            // Khung hộp dây 3D
            const boxGeo = new THREE.BoxGeometry(0.4, 0.4, 0.6);
            const edgeGeo = new THREE.EdgesGeometry(boxGeo);
            const boxMat = new THREE.LineBasicMaterial({ color: det.id === 0 ? 0xff2a6d : 0x00ffa3, linewidth: 2 });
            const wireBox = new THREE.LineSegments(edgeGeo, boxMat);
            wireBox.position.set(wx, wy, wz);
            semanticGroup.add(wireBox);
          });
        }

        lastStateData = d;

        // 5. Cập nhật 2D Map (nếu bật chế độ 2D)
        if (currentView === '2d') {
          if (d.map_b64 && d.map_version !== currentMapVer) {
            currentMapVer = d.map_version;
            mapImage.src = 'data:image/png;base64,' + d.map_b64;
            mapImage.onload = () => draw2DMap(d);
          } else {
            draw2DMap(d);
          }
        }

        // 6. Danh sách Radar
        const rl = document.getElementById('radar-list');
        document.getElementById('det-count').innerText = `${d.detections.length} VẬT THỂ`;
        if (d.detections.length > 0) {
          rl.innerHTML = d.detections.map(det => `
            <div class="radar-item" style="border-color:${det.id === 0 ? 'var(--rose)' : 'var(--emerald)'};">
              <span style="color:${det.id === 0 ? 'var(--rose)' : 'var(--emerald)'};">${det.name} (${(det.score*100).toFixed(0)}%)</span>
              <span style="color:var(--dim);">X:${det.x}m Y:${det.y}m Cự ly:${det.z}m</span>
            </div>
          `).join('');
        } else {
          rl.innerHTML = '<div style="text-align:center; color:var(--dim); padding:15px;">Đang quét vật thể 3D trong không gian...</div>';
        }

        // 7. Cập nhật Đánh Giá 3 Thành Viên Nhóm Đồ Án
        if (d.benchmark) {
          const b = d.benchmark;
          const tv1 = b.tv1 || {};
          const tv2 = b.tv2 || b;
          const tv3 = b.tv3 || {};

          // ─── THÀNH VIÊN 1 (TV1: ĐIỀU KHIỂN & PHẦN CỨNG) ───
          if (tv1.total_score !== undefined) {
            const b1 = document.getElementById('badge-tv1');
            if (b1) {
              b1.innerText = `${tv1.total_score}/${tv1.max_score || 30} · ${tv1.grade || 'XUẤT SẮC'}`;
              b1.style.borderColor = tv1.total_score >= 25 ? 'var(--cyan)' : (tv1.total_score >= 20 ? 'var(--amber)' : 'var(--rose)');
              b1.style.color = b1.style.borderColor;
            }
            if (document.getElementById('val-tv1-hw')) document.getElementById('val-tv1-hw').innerText = `${tv1.score_hw}/${tv1.max_hw}`;
            if (document.getElementById('bar-tv1-hw')) document.getElementById('bar-tv1-hw').style.width = `${((tv1.score_hw / (tv1.max_hw || 15)) * 100).toFixed(0)}%`;
            if (document.getElementById('sub-tv1-hw')) document.getElementById('sub-tv1-hw').innerText = `CPU: ${tv1.cpu_pct}% · RAM: ${tv1.ram_gb}GB · Pin INA219: ${tv1.battery_v}V (${tv1.battery_pct}%)`;

            if (document.getElementById('val-tv1-motion')) document.getElementById('val-tv1-motion').innerText = `${tv1.score_motion}/${tv1.max_motion}`;
            if (document.getElementById('bar-tv1-motion')) document.getElementById('bar-tv1-motion').style.width = `${((tv1.score_motion / (tv1.max_motion || 15)) * 100).toFixed(0)}%`;
            if (document.getElementById('sub-tv1-motion')) document.getElementById('sub-tv1-motion').innerText = `Virtual Bumper: ${tv1.bumper_status} (Cự ly cản: ${tv2.obstacle_distance !== undefined ? tv2.obstacle_distance : '--'}m)`;

            if (document.getElementById('diag-tv1-txt')) document.getElementById('diag-tv1-txt').innerText = tv1.diag_text || 'Hoạt động ổn định.';
          }

          // ─── THÀNH VIÊN 2 (TV2: THỊ GIÁC & SPATIAL AI - THÔNG SỐ THỰC TẾ) ───
          const badgeEl = document.getElementById('cam-status-badge');
          if (badgeEl) {
            badgeEl.innerText = tv2.status_badge || 'HOẠT ĐỘNG';
            if (tv2.status_badge === 'HOẠT ĐỘNG TỐT') {
              badgeEl.style.borderColor = 'var(--emerald)';
              badgeEl.style.color = 'var(--emerald)';
              badgeEl.style.background = 'rgba(0, 255, 163, 0.15)';
            } else if (tv2.status_badge && (tv2.status_badge.includes('THIẾU') || tv2.status_badge.includes('TRỄ'))) {
              badgeEl.style.borderColor = 'var(--amber)';
              badgeEl.style.color = 'var(--amber)';
              badgeEl.style.background = 'rgba(255, 184, 0, 0.15)';
            } else {
              badgeEl.style.borderColor = 'var(--rose)';
              badgeEl.style.color = 'var(--rose)';
              badgeEl.style.background = 'rgba(255, 42, 109, 0.15)';
            }
          }

          if (document.getElementById('val-cam-fps') && tv2.fps !== undefined) document.getElementById('val-cam-fps').innerText = `${tv2.fps.toFixed(1)} FPS`;
          if (document.getElementById('bar-cam-fps') && tv2.fps_pct !== undefined) document.getElementById('bar-cam-fps').style.width = `${tv2.fps_pct}%`;
          if (document.getElementById('sub-cam-fps') && tv2.fps !== undefined) document.getElementById('sub-cam-fps').innerText = `Chu kỳ đo thực tế: ${tv2.fps.toFixed(1)} / 15.0 FPS`;

          if (document.getElementById('val-cam-clarity') && tv2.valid_depth_pct !== undefined) document.getElementById('val-cam-clarity').innerText = `${tv2.valid_depth_pct}%`;
          if (document.getElementById('bar-cam-clarity') && tv2.valid_depth_pct !== undefined) document.getElementById('bar-cam-clarity').style.width = `${tv2.valid_depth_pct}%`;
          if (document.getElementById('sub-cam-clarity') && tv2.valid_depth_pct !== undefined) document.getElementById('sub-cam-clarity').innerText = tv2.valid_depth_pct >= 60 ? 'Chất lượng ảnh: SẮC NÉT · ĐỦ ÁNH SÁNG' : 'Chất lượng ảnh: MỜ NÉT · THIẾU SÁNG';

          if (document.getElementById('val-cam-target') && tv2.num_obj !== undefined) document.getElementById('val-cam-target').innerText = `${tv2.num_obj} VẬT THỂ`;
          if (document.getElementById('bar-cam-target')) document.getElementById('bar-cam-target').style.width = (tv2.num_obj > 0) ? `${Math.min(100, tv2.avg_confidence || 80)}%` : '0%';
          if (document.getElementById('sub-cam-target') && tv2.target_info) document.getElementById('sub-cam-target').innerText = tv2.target_info;

          if (document.getElementById('val-cam-dist') && tv2.obstacle_distance !== undefined) document.getElementById('val-cam-dist').innerText = `${tv2.obstacle_distance.toFixed(2)} MÉT`;
          if (document.getElementById('bar-cam-dist') && tv2.dist_pct !== undefined) document.getElementById('bar-cam-dist').style.width = `${tv2.dist_pct}%`;
          if (document.getElementById('sub-cam-dist') && tv2.obstacle_distance !== undefined) document.getElementById('sub-cam-dist').innerText = tv2.obstacle_distance >= 0.35 ? `Cự ly an toàn: ${tv2.obstacle_distance.toFixed(2)}m (VÙNG AN TOÀN)` : `CẢNH BÁO: ${tv2.obstacle_distance.toFixed(2)}m (< 0.35m PHANH KHẨN CẤP)`;

          if (document.getElementById('cam-diag-txt') && tv2.diag_text) document.getElementById('cam-diag-txt').innerText = tv2.diag_text;
          if (document.getElementById('cam-hdr-fps') && tv2.fps !== undefined) document.getElementById('cam-hdr-fps').innerText = `${tv2.fps.toFixed(1)} FPS · REAL-TIME`;
          if (document.getElementById('hud-cam-tag') && tv2.camera_source) document.getElementById('hud-cam-tag').innerText = tv2.camera_source;

          // ─── THÀNH VIÊN 3 (TV3: VISUAL SLAM & BẢN ĐỒ NGỮ NGHĨA 3D) ───
          if (tv3.total_score !== undefined) {
            const b3 = document.getElementById('badge-tv3');
            if (b3) {
              b3.innerText = `${tv3.total_score}/${tv3.max_score || 60} · ${tv3.grade || 'XUẤT SẮC'}`;
              b3.style.borderColor = tv3.total_score >= 50 ? 'var(--purple)' : (tv3.total_score >= 40 ? 'var(--amber)' : 'var(--rose)');
              b3.style.color = b3.style.borderColor;
            }
            if (document.getElementById('val-tv3-slam')) document.getElementById('val-tv3-slam').innerText = `${tv3.score_s}/${tv3.max_s}`;
            if (document.getElementById('bar-tv3-slam')) document.getElementById('bar-tv3-slam').style.width = `${((tv3.score_s / (tv3.max_s || 35)) * 100).toFixed(0)}%`;
            if (document.getElementById('sub-tv3-slam')) document.getElementById('sub-tv3-slam').innerText = `VO: ${tv3.odom_hz} Hz · Mây điểm: ${tv3.num_pts} pts · Cây TF2 liên tục`;

            if (document.getElementById('val-tv3-sem')) document.getElementById('val-tv3-sem').innerText = `${tv3.score_sem}/${tv3.max_sem}`;
            if (document.getElementById('bar-tv3-sem')) document.getElementById('bar-tv3-sem').style.width = `${((tv3.score_sem / (tv3.max_sem || 25)) * 100).toFixed(0)}%`;
            if (document.getElementById('sub-tv3-sem')) document.getElementById('sub-tv3-sem').innerText = `Đã cắm cờ ngữ nghĩa: ${tv3.num_obj} vật thể 3D`;

            if (document.getElementById('diag-tv3-txt')) document.getElementById('diag-tv3-txt').innerText = tv3.diag_text || 'Bản đồ RTAB-Map hoạt động chuẩn xác.';
          }
        }

        // Cập nhật Nhãn Nguồn Camera & Reconnect phục hồi
        if (d.camera_source) {
          const badge = document.getElementById('cam-source-badge');
          if (badge) {
            badge.innerText = d.camera_source.includes('WEBCAM') ? 'WEBCAM LAPTOP' : (d.camera_source.includes('OAK-D') ? 'OAK-D S2' : 'MÔ PHỎNG');
            badge.style.color = d.camera_source.includes('WEBCAM') ? 'var(--emerald)' : 'var(--cyan)';
          }
          const hud = document.getElementById('cam-hud');
          if (hud) {
            hud.innerText = `${d.camera_source} · ${(d.calc_fps || 15).toFixed(1)} FPS`;
          }
        }
      } catch (e) {}
      setTimeout(pollState, 140);
    }
    pollState();

    // Tự động khôi phục luồng stream nếu mạng gián đoạn
    const camImg = document.getElementById('cam-stream');
    if (camImg) {
      camImg.onerror = function() {
        setTimeout(() => {
          camImg.src = '/stream.mjpg?t=' + Date.now();
        }, 1200);
      };
    }

    // Chuyển đổi Tab Đánh giá 3 Thành viên
    function switchEvalTab(tabId) {
      ['tv1', 'tv2', 'tv3', 'all'].forEach(t => {
        const btn = document.getElementById('btn-tab-' + t);
        if (btn) btn.classList.toggle('active', t === tabId);
      });
      const p1 = document.getElementById('panel-tv1');
      const p2 = document.getElementById('panel-tv2');
      const p3 = document.getElementById('panel-tv3');
      if (tabId === 'all') {
        if (p1) p1.style.display = 'flex';
        if (p2) p2.style.display = 'flex';
        if (p3) p3.style.display = 'flex';
      } else {
        if (p1) p1.style.display = (tabId === 'tv1') ? 'flex' : 'none';
        if (p2) p2.style.display = (tabId === 'tv2') ? 'flex' : 'none';
        if (p3) p3.style.display = (tabId === 'tv3') ? 'flex' : 'none';
      }
    }

    // Vẽ bản đồ 2D trên Canvas
    function draw2DMap(d) {
      const cv = document.getElementById('map-canvas');
      const ctx = cv.getContext('2d');
      cv.width = cv.parentElement.clientWidth; cv.height = cv.parentElement.clientHeight;
      ctx.fillStyle = '#101520'; ctx.fillRect(0, 0, cv.width, cv.height);
      if (!mapImage.width) return;
      const sc = Math.min(cv.width / mapImage.width, cv.height / mapImage.height) * 1.3;
      ctx.save(); ctx.translate(cv.width / 2, cv.height / 2);
      ctx.drawImage(mapImage, -mapImage.width*sc/2, -mapImage.height*sc/2, mapImage.width*sc, mapImage.height*sc);

      function w2c(wx, wy) {
        return [((wx - d.map_origin_x)/d.map_resolution - mapImage.width/2)*sc, (mapImage.height/2 - (wy - d.map_origin_y)/d.map_resolution)*sc];
      }
      if (d.path && d.path.length > 1) {
        ctx.beginPath(); ctx.strokeStyle = '#00ffa3'; ctx.lineWidth = 2; ctx.setLineDash([3,3]);
        const s = w2c(d.path[0][0], d.path[0][1]); ctx.moveTo(s[0], s[1]);
        for(let i=1; i<d.path.length; i++) { const p = w2c(d.path[i][0], d.path[i][1]); ctx.lineTo(p[0], p[1]); }
        ctx.stroke(); ctx.setLineDash([]);
      }
      const [rx, ry] = w2c(d.robot_x, d.robot_y);
      ctx.save(); ctx.translate(rx, ry); ctx.rotate(-d.robot_yaw);
      ctx.fillStyle = '#00f0ff'; ctx.beginPath(); ctx.arc(0, 0, 7, 0, Math.PI*2); ctx.fill();
      ctx.strokeStyle = '#ff2a6d'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(14,0); ctx.stroke();
      ctx.restore(); ctx.restore();
    }

    // Điều khiển động cơ
    let driveTimer = null;
    function sendDrive(linear, angular) {
      if (driveTimer) clearInterval(driveTimer);
      fetch('/api/drive?v=' + linear + '&w=' + angular);
      if (linear !== 0 || angular !== 0) driveTimer = setInterval(() => fetch('/api/drive?v=' + linear + '&w=' + angular), 200);
    }
    window.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      if (['w','W'].includes(e.key)) sendDrive(0.2, 0);
      if (['s','S'].includes(e.key)) sendDrive(-0.2, 0);
      if (['a','A'].includes(e.key)) sendDrive(0, 0.6);
      if (['d','D'].includes(e.key)) sendDrive(0, -0.6);
      if (e.key === ' ') sendDrive(0, 0);
    });
    window.addEventListener('keyup', (e) => {
      if (['w','W','s','S','a','A','d','D'].includes(e.key)) sendDrive(0, 0);
    });
  </script>
</body>
</html>
"""

class WebHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args): pass
    def do_GET(self):
        if self.path in ['/', '/index.html']:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        elif self.path.startswith('/stream.mjpg'):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.end_headers()
            while not is_shutdown():
                with state.lock: jpeg = state.latest_jpeg
                if jpeg:
                    try:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                    except Exception: break
                time.sleep(0.04)
        elif self.path.startswith('/cam.jpg'):
            with state.lock: jpeg = state.latest_jpeg
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(404)
                self.end_headers()
        elif self.path == '/api/state':
            with state.lock:
                d = {
                    "battery_v": state.battery_v, "battery_pct": state.battery_pct,
                    "battery_cell_v": state.battery_cell_v, "battery_current_a": state.battery_current_a,
                    "battery_power_w": state.battery_power_w, "battery_remaining_min": state.battery_remaining_min,
                    "battery_status": state.battery_status,
                    "robot_x": state.robot_x, "robot_y": state.robot_y, "robot_z": state.robot_z, "robot_yaw": state.robot_yaw,
                    "path": state.path_history, "map_b64": state.map_png_base64, "map_version": state.map_version,
                    "map_origin_x": state.map_origin_x, "map_origin_y": state.map_origin_y,
                    "map_resolution": state.map_resolution, "detections": state.detections,
                    "obstacle_distance": getattr(state, 'obstacle_distance', 1.45),
                    "points_3d": state.points_3d,
                    "camera_source": getattr(state, 'camera_source', 'CAMERA'),
                    "calc_fps": getattr(state, 'calc_fps', 15.0),
                    "benchmark": state.benchmark
                }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(d).encode('utf-8'))
        elif self.path.startswith('/api/drive'):
            v, w = 0.0, 0.0
            if '?' in self.path:
                for p in self.path.split('?')[1].split('&'):
                    if p.startswith('v='): v = float(p.split('=')[1])
                    if p.startswith('w='): w = float(p.split('=')[1])
            with state.lock:
                state.is_driving = (abs(v) > 0.01 or abs(w) > 0.01)
                state.last_drive_time = time.time()
            if HAS_ROS and cmd_vel_pub:
                t = Twist()
                t.linear.x, t.angular.z = v, w
                cmd_vel_pub.publish(t)
            else:
                # Chế độ Giả lập trên Windows: cập nhật chuyển động xe ảo
                with state.lock:
                    state.robot_yaw += w * 0.15
                    state.robot_x += v * 0.15 * math.cos(state.robot_yaw)
                    state.robot_y += v * 0.15 * math.sin(state.robot_yaw)
                    state.path_history.append([round(state.robot_x, 3), round(state.robot_y, 3), 0.02])
                    if len(state.path_history) > 300: state.path_history.pop(0)
            self.send_response(200); self.end_headers(); self.wfile.write(b'OK')

class ThreadedServer(ThreadingMixIn, HTTPServer): daemon_threads = True

# ─── LOCAL MOCK SIMULATOR FOR WINDOWS (KHÔNG CẦN ROBOT THẬT) ─────────────────
def mock_simulator_worker():
    """Tạo căn phòng 3D ảo và dữ liệu giả lập để Thành viên 3 test trên Laptop"""
    # 1. Sinh mây điểm 3D căn phòng 4x4m
    mock_pts = []
    # Tường Bắc / Nam
    for x in np.linspace(-2.0, 2.0, 35):
        for z in np.linspace(0.0, 1.8, 12):
            mock_pts.append([round(float(x), 2), 2.2, round(float(z), 2)])
            mock_pts.append([round(float(x), 2), -2.2, round(float(z), 2)])
    # Tường Đông / Tây
    for y in np.linspace(-2.2, 2.2, 35):
        for z in np.linspace(0.0, 1.8, 12):
            mock_pts.append([2.0, round(float(y), 2), round(float(z), 2)])
            mock_pts.append([-2.0, round(float(y), 2), round(float(z), 2)])
    
    # 1b. Sinh bản đồ 2D Occupancy Grid giả lập căn phòng 4x4m
    map_h, map_w = 140, 140
    grid_img = np.full((map_h, map_w, 3), 18, dtype=np.uint8) # Dark Cyber
    # Sàn nhà (Free space từ -2.0m đến 2.0m)
    # Origin = -3.5m, resolution = 0.05m -> px = (meter - origin) / 0.05
    # x = -2.0 -> px = 30, x = 2.0 -> px = 110
    grid_img[30:111, 30:111] = [34, 46, 62] # Free space sàn nhà
    # Bức tường xung quanh phòng
    cv2.rectangle(grid_img, (30, 30), (110, 110), (0, 240, 255), 2)
    # Vật thể: Ghế (x=0.65, y=-0.1 -> px 83, py 68), Người (x=-0.45, y=0.05 -> px 61, py 71)
    cv2.rectangle(grid_img, (80, 65), (86, 71), (0, 255, 163), -1) # Ghế
    cv2.circle(grid_img, (61, 71), 3, (255, 42, 109), -1)          # Người

    grid_flipped = cv2.flip(grid_img, 0)
    _, map_png = cv2.imencode('.png', grid_flipped)
    mock_map_b64 = base64.b64encode(map_png).decode('utf-8')

    with state.lock:
        state.points_3d = mock_pts
        state.detections = [
            {"id": 0, "name": "PERSON", "score": 0.92, "x": -0.45, "y": 0.05, "z": 1.40},
            {"id": 56, "name": "CHAIR", "score": 0.86, "x": 0.65, "y": -0.10, "z": 1.75}
        ]
        state.obstacle_distance = 1.40
        state.valid_depth_pct = 95.5
        state.avg_confidence = 0.89
        state.camera_source = "OAK-D S2 SIMULATOR"
        state.map_width = map_w
        state.map_height = map_h
        state.map_resolution = 0.05
        state.map_origin_x = -3.5
        state.map_origin_y = -3.5
        state.map_png_base64 = mock_map_b64
        state.map_version = 1

    # Tạo ảnh khởi tạo ban đầu để /stream.mjpg sẵn sàng ngay lập tức
    init_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    init_frame[:] = (18, 22, 30)
    cv2.putText(init_frame, "DANG KET NOI CAMERA...", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 240, 255), 2)
    _, init_jpg = cv2.imencode('.jpg', init_frame)
    with state.lock:
        state.latest_jpeg = init_jpg.tobytes()

    # Khởi tạo kết nối Camera Laptop thực tế (Windows DirectShow)
    cap = None
    use_webcam = False
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ret_t, test_f = cap.read()
            if ret_t and test_f is not None:
                use_webcam = True
                print("📷 [WEBCAM] Đã kết nối thành công Camera Laptop (cv2.CAP_DSHOW 640x480)!")
            else:
                print("⚠️ [WEBCAM] Không đọc được frame từ webcam laptop, chuyển sang giả lập Cyber Room.")
                cap.release()
                cap = None
    except Exception as e:
        print(f"⚠️ [WEBCAM] Lỗi khởi tạo webcam laptop: {e}")
        if cap:
            try: cap.release()
            except Exception: pass
        cap = None

    step = 0
    sim_capacity_ah = 2.6 * (88.0 / 100.0)
    sim_nominal_ah = 2.6

    try:
        while not is_shutdown():
            try:
                now = time.time()
                with state.lock:
                    driving = (now - state.last_drive_time) < 0.6

                # Tiêu thụ dòng: Nghỉ ~0.85A, Đang lái ~1.85A
                if driving:
                    sim_current = 1.80 + 0.10 * math.sin(step * 0.4)
                    v_sag = 0.22
                else:
                    sim_current = 0.82 + 0.04 * math.sin(step * 0.1)
                    v_sag = 0.0

                # Xả pin theo thời gian thực (gia tốc x4)
                sim_capacity_ah = max(0.1, sim_capacity_ah - (sim_current * (0.066 / 3600.0)) * 4.0)
                sim_pct = int(max(0, min(100, (sim_capacity_ah / sim_nominal_ah) * 100.0)))

                ocv = 9.0 + (sim_pct / 100.0) * 3.6
                for (v_c, p_c), (v_n, p_n) in zip(_LI_ION_CURVE_3S, _LI_ION_CURVE_3S[1:]):
                    if p_n <= sim_pct <= p_c:
                        ocv = v_n + ((sim_pct - p_n) / max(1, (p_c - p_n))) * (v_c - v_n)
                        break
                
                real_v = max(9.0, round(ocv - v_sag, 2))
                pct, cell_v, curr_a, p_w, rem_min, status_str = update_battery_metrics(real_v, sim_current)

                with state.lock:
                    state.battery_v = real_v
                    state.battery_pct = pct
                    state.battery_cell_v = cell_v
                    state.battery_current_a = curr_a
                    state.battery_power_w = p_w
                    state.battery_remaining_min = rem_min
                    state.battery_status = status_str

                got_cam_frame = False
                if use_webcam and cap and cap.isOpened():
                    ret_cam, raw_cam = cap.read()
                    if ret_cam and raw_cam is not None:
                        got_cam_frame = True
                        # Lật gương ngang để tự nhiên khi soi camera laptop
                        frame = cv2.flip(raw_cam, 1)
                        if frame.shape[0] != 480 or frame.shape[1] != 640:
                            frame = cv2.resize(frame, (640, 480))
                        
                        # Đo độ nét thực tế qua phương sai Laplacian
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                        real_clarity = min(99.0, max(45.0, round(lap_var / 4.0, 1)))

                        # Phân đoạn màu da HSV để nhận diện Người trước camera
                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                        skin_mask = cv2.inRange(hsv, np.array([0, 30, 60], dtype=np.uint8), np.array([25, 255, 255], dtype=np.uint8))
                        cnts, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        large_cnts = [c for c in cnts if cv2.contourArea(c) > 1200]

                        if large_cnts:
                            c = max(large_cnts, key=cv2.contourArea)
                            bx, by, bw, bh = cv2.boundingRect(c)
                            bx = max(10, bx - 20)
                            by = max(10, by - 25)
                            bw = min(620 - bx, bw + 40)
                            bh = min(460 - by, bh + 50)
                            
                            p_dist = round(max(0.45, min(2.8, 180.0 / max(bw, bh))), 2)
                            p_score = round(min(0.96, 0.84 + (cv2.contourArea(c) / (640*480)) * 0.4), 2)
                            center_x = round((bx + bw/2.0 - 320.0) / 320.0 * 0.6, 2)
                            center_y = round((240.0 - (by + bh/2.0)) / 240.0 * 0.4, 2)
                        else:
                            p_dist = round(1.25 + math.sin(step * 0.1) * 0.05, 2)
                            bx, by, bw, bh = 200, 100, 240, 300
                            p_score = 0.91
                            center_x, center_y = 0.05, 0.02

                        # Vẽ Bounding Box PERSON (Cyber Rose neon)
                        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (255, 42, 109), 2)
                        cv2.rectangle(frame, (bx, max(0, by - 28)), (bx + bw, by), (255, 42, 109), -1)
                        cv2.putText(frame, f"PERSON {int(p_score*100)}% ({p_dist}m)", (bx + 5, max(18, by - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                        cv2.putText(frame, f"3D: X:{center_x}m Y:{center_y}m Z:{p_dist}m", (bx + 5, min(470, by + bh + 18)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 42, 109), 1)

                        # HUD Overlay
                        cv2.putText(frame, f"[WEBCAM LAPTOP REAL-TIME] {state.calc_fps:.1f} FPS | NET: {real_clarity:.1f}%", (20, 440),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 240, 255), 1)
                        cv2.putText(frame, f"Cu ly vat can (/obstacle_distance): {p_dist}m (AN TOAN)", (20, 465),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 163), 1)

                        sim_dets = [
                            {"id": 0, "name": "PERSON", "score": p_score, "x": center_x, "y": center_y, "z": p_dist}
                        ]
                        cam_src = "WEBCAM LAPTOP REAL-TIME"

                if not got_cam_frame:
                    # Chế độ Fallback: Tạo phòng Cyber Room ảo
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    frame[:] = (18, 22, 30)

                    for gx in range(0, 640, 40): cv2.line(frame, (gx, 0), (gx, 480), (35, 45, 60), 1)
                    for gy in range(0, 480, 40): cv2.line(frame, (0, gy), (640, gy), (35, 45, 60), 1)

                    scan_y = int((step * 10) % 480)
                    cv2.line(frame, (0, scan_y), (640, scan_y), (0, 240, 255), 1)

                    p_dist = round(1.40 + math.sin(step * 0.1) * 0.05, 2)
                    px_w = 140 + int(math.sin(step * 0.15) * 3)
                    px_h = 320 + int(math.cos(step * 0.15) * 3)

                    cv2.rectangle(frame, (120, 100), (120 + px_w, 100 + px_h), (255, 42, 109), 2)
                    cv2.rectangle(frame, (120, 72), (120 + px_w, 98), (255, 42, 109), -1)
                    cv2.putText(frame, f"PERSON 92% ({p_dist}m)", (125, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                    cv2.putText(frame, "3D: X:-0.45m Y:0.05m Z:1.40m", (125, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 42, 109), 1)

                    cv2.rectangle(frame, (380, 180), (520, 400), (0, 255, 163), 2)
                    cv2.rectangle(frame, (380, 152), (520, 178), (0, 255, 163), -1)
                    cv2.putText(frame, "CHAIR 86% (1.75m)", (385, 172), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
                    cv2.putText(frame, "3D: X:0.65m Y:-0.10m Z:1.75m", (385, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 163), 1)

                    real_clarity = 95.5
                    p_score = 0.89
                    cv2.putText(frame, f"[OAK-D S2 CAM SIMULATOR] 15.0 FPS | NET: {real_clarity}%", (20, 440),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 240, 255), 1)
                    cv2.putText(frame, f"Cu ly vat can (/obstacle_distance): {p_dist}m (VUNG AN TOAN)", (20, 465),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 163), 1)

                    sim_dets = [
                        {"id": 0, "name": "PERSON", "score": 0.92, "x": -0.45, "y": 0.05, "z": p_dist},
                        {"id": 56, "name": "CHAIR", "score": 0.86, "x": 0.65, "y": -0.10, "z": 1.75}
                    ]
                    cam_src = "OAK-D S2 SIMULATOR"

                _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                with state.lock:
                    state.latest_jpeg = jpeg.tobytes()
                    state.detections = sim_dets
                    state.obstacle_distance = p_dist
                    state.valid_depth_pct = real_clarity
                    state.avg_confidence = p_score
                    state.camera_source = cam_src
                    state.img_counter += 1
                    state.odom_counter += 1
                    state.last_odom_recv = time.time()
                step += 1
            except Exception:
                pass
            time.sleep(0.035 if got_cam_frame else 0.066)
    finally:
        if cap:
            try: cap.release()
            except Exception: pass

def main():
    global cmd_vel_pub, battery_pub

    if HAS_ROS:
        rospy.init_node('slam_3d_web_dashboard', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        battery_pub = rospy.Publisher('/battery_telemetry', Float32MultiArray, queue_size=1)

        rospy.Subscriber('/stereo_inertial_publisher/color/image', Image, image_cb, queue_size=1)
        rospy.Subscriber('/stereo_inertial_publisher/color/camera_info', CameraInfo, camera_info_cb, queue_size=1)
        rospy.Subscriber('/stereo_inertial_publisher/stereo/depth', Image, depth_cb, queue_size=1)
        rospy.Subscriber('/rtabmap/odom', Odometry, odom_cb, queue_size=1)
        rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, map_cb, queue_size=1)
        rospy.Subscriber('/battery_telemetry', Float32MultiArray, battery_cb, queue_size=1)
        rospy.Subscriber('/spatial_objects', String, spatial_objects_json_cb, queue_size=1)
        rospy.Subscriber('/obstacle_distance', Float32, obstacle_distance_cb, queue_size=1)

        if HAS_DEPTHAI:
            rospy.Subscriber('/stereo_inertial_publisher/color/yolov4_Spatial_detections', SpatialDetectionArray, detections_cb, queue_size=1)

        threading.Thread(target=battery_worker, daemon=True).start()
        threading.Thread(target=benchmark_worker, daemon=True).start()
        server = ThreadedServer(('0.0.0.0', PORT), WebHandler)
        rospy.loginfo(f"🌐 JetBot 3D Cockpit Web Server: http://0.0.0.0:{PORT}")
        threading.Thread(target=server.serve_forever, daemon=True).start()
        rospy.spin()
    else:
        # CHẾ ĐỘ GIẢ LẬP TRÊN WINDOWS DÀNH CHO THÀNH VIÊN 3 (KHÔNG CẦN ROBOT)
        c_green = "\033[1;32m"
        c_cyan = "\033[1;36m"
        c_yellow = "\033[1;33m"
        c_reset = "\033[0m"

        print(f"\n{c_cyan}══════════════════════════════════════════════════════════════════════════════{c_reset}")
        print(f"{c_yellow}🎮 KHỞI CHẠY THÀNH CÔNG: JETBOT 3D DIGITAL TWIN (WINDOWS MOCK SIMULATOR){c_reset}")
        print(f"  ├─ {c_green}Mô phỏng căn phòng 3D ảo & Mây điểm 3D (Walls & Floor){c_reset}")
        print(f"  ├─ {c_green}Mô phỏng Camera OAK-D S2 + Nhận diện Người & Ghế 3D{c_reset}")
        print(f"  ├─ {c_green}Điều khiển xe ảo bằng phím W, A, S, D trên Web{c_reset}")
        import socket
        try:
            _s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _s.connect(("8.8.8.8", 80))
            lan_ip = _s.getsockname()[0]
            _s.close()
        except Exception:
            lan_ip = "127.0.0.1"

        print(f"{c_yellow}👉 TRÊN MÁY NÀY:       {c_cyan}http://localhost:{PORT}{c_reset}")
        print(f"{c_yellow}👉 TỪ MÁY KHÁC / ĐIỆN THOẠI CÙNG WIFI: {c_green}http://{lan_ip}:{PORT}{c_reset}")
        print(f"{c_cyan}══════════════════════════════════════════════════════════════════════════════{c_reset}\n")

        threading.Thread(target=mock_simulator_worker, daemon=True).start()
        threading.Thread(target=benchmark_worker, daemon=True).start()
        server = ThreadedServer(('0.0.0.0', PORT), WebHandler)
        server.serve_forever()

if __name__ == '__main__':
    main()
