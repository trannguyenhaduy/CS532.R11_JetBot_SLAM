#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cấu hình tập trung cho toàn bộ hệ thống JetBot (Centralized Configuration)
Chứa tất cả các cờ tính năng (Feature Flags) và thông số kỹ thuật.
"""

# ─── THÔNG TIN PHIÊN BẢN HỆ THỐNG (SYSTEM VERSIONING) ─────────────────────────
SYSTEM_VERSION     = "v2.4.0-SAFETY-DUAL-ROS"
VERSION_CODENAME   = "AEGIS JETBOT (Bảo Vệ Toàn Diện & Phanh Tự Hành)"
BUILD_TAG          = "v2.4.0-safety-dual-ros"
WORKFLOW_MODE      = "1-Terminal Mode (python3 main.py - Chạy trực tiếp đơn giản, nhẹ CPU)"

# ─── CỜ BẬT / TẮT TÍNH NĂNG (FEATURE FLAGS) ───────────────────────────────────
ENABLE_MOTORS       = True   # Bật/Tắt module động cơ (PCA9685 I2C 0x60)
ENABLE_BATTERY      = True   # Bật/Tắt module đọc pin INA219 (I2C 0x41)
ENABLE_CAMERA       = True   # Bật/Tắt module Camera OAK-D S2
ENABLE_IMU          = True   # Bật/Tắt cảm biến quán tính IMU phần cứng trên OAK-D S2
ENABLE_CLOSED_LOOP_IMU = True # Bật vòng điều khiển quay kín theo góc IMU thời gian thực (chuẩn xác ±1°)
ENABLE_YOLO         = True   # BẬT AI nhận diện vật thể (MobileNet-SSD trên chip VPU OAK-D)
AI_MODEL            = "mobilenet" # "mobilenet" (MobileNet-SSD 20 lớp, nhẹ, chuẩn JetBot)
ENABLE_FOLLOWER     = False  # Bật/Tắt module tự hành bám người (MẶC ĐỊNH TẮT ĐỂ ƯU TIÊN LÁI TAY)
ENABLE_MAPPER             = True   # Bật/Tắt module bản đồ 2D Occupancy Grid SLAM
ENABLE_SEMANTIC_LANDMARKS = False  # Mặc định TẮT nhãn AI & Bounding Box trên bản đồ 2D (chỉ hiện tường & vị trí xe thực tế)
ENABLE_SCAN_MATCHING      = True   # Bật thuật toán Correlative Scan Matching (CSM) tự cân bằng tọa độ vòng kín
ENABLE_VISUAL_GYRO        = False  # TẮT con quay thị giác (tránh tự cộng dồn góc xoay ngẫu nhiên khi dừng xe)
SCAN_MATCH_SEARCH_DIST_M  = 0.08   # Cửa sổ tìm kiếm vị trí (±8cm)
SCAN_MATCH_SEARCH_YAW_DEG = 0.0    # KHÓA CỨNG GÓC YAW (0.0 độ): CSM chỉ nắn vị trí (X, Y), không xoay yaw xe gây trôi dạt
SCAN_MATCH_MIN_SCORE      = 0.28   # Ngưỡng tin cậy tối thiểu để hít tọa độ (28% số tia khớp tường)
ENABLE_WEB                = True   # Bật/Tắt trạm điều khiển Web Cockpit (Port 8080)
ENABLE_SAFETY_BRAKE       = True   # Bật tính năng phanh khẩn cấp Virtual Bumper (< 20cm)
ENABLE_PATH_PLANNER       = True   # Bật module tìm đường tối ưu A* (Global Path Planner)
ROBOT_RADIUS_M            = 0.12   # Bán kính thân xe JetBot (12cm)
COSTMAP_INFLATION_RADIUS_M = 0.20  # Bán kính vùng đệm an toàn chống cạ tường (20cm)
ENABLE_HEADING_LOCK       = True   # Bật/Tắt khóa hướng đi thẳng chủ động bằng IMU (Active Heading Lock)
HEADING_LOCK_KP           = 0.50   # Hệ số P khóa hướng mượt mà (0.45 - 0.55), không gây rung lắc giật cục
MOTOR_TRIM_LEFT           = 1.000  # Cân bằng tĩnh bánh trái (Hardware Wheel Trim)
MOTOR_TRIM_RIGHT          = 1.020  # Cân bằng tĩnh bánh phải (+2% bù ma sát hộp số)

# ─── THÔNG SỐ TRẠM ĐIỀU KHIỂN WEB ─────────────────────────────────────────────
WEB_PORT = 8080
WEB_HOST = '0.0.0.0'

# ─── THÔNG SỐ PHẦN CỨNG XE (JETBOT BASELINE) ──────────────────────────────────
I2C_BUS = 1
PCA9685_ADDR = 0x60
INA219_ADDR  = 0x41

# ══════════════════════════════════════════════════════════════════════════════
# 📐 THÔNG SỐ HÌNH HỌC KHUNG XE & VỊ TRÍ GẮN CAMERA OAK-D (ROBOT & CAMERA EXTRINSICS)
# ══════════════════════════════════════════════════════════════════════════════
# Gốc tọa độ chuẩn của Robot (0, 0, 0) nằm tại mặt sàn, ngay chính giữa đường nối tâm 2 bánh xe chủ động.
WHEEL_SEPARATION_M       = 0.12   # Khoảng cách giữa 2 tâm vệt bánh xe vi sai (mét, ví dụ 0.12 = 12cm)
MAX_LINEAR_SPEED         = 0.20   # Vận tốc tiến tối đa (m/s) - Giảm xuống 0.20 m/s để phanh kịp thời
MAX_ANGULAR_SPEED        = 1.60   # Vận tốc quay tối đa (rad/s) ~ 91.7 deg/s

# ── Vị trí không gian 3 chiều của Camera OAK-D so với gốc tọa độ xe (Tâm trục bánh, mặt sàn) ──
CAMERA_OFFSET_FORWARD_M  = 0.050  # [Trục X] Độ nhô ra phía trước của thấu kính OAK-D so với trục bánh (m, VD: 0.09 = 9cm)
CAMERA_OFFSET_LATERAL_M  = -0.015 # [Trục Y] Độ lệch ngang của camera so với trục dọc giữa xe (m, Âm: LỆCH PHẢI, Dương: LỆCH TRÁI. VD: -0.015 = lệch phải 1.5cm)
CAMERA_HEIGHT_M          = 0.120  # [Trục Z] Chiều cao từ mặt sàn lên đến tâm thấu kính OAK-D (m, VD: 0.12 = 12cm)
CAMERA_PITCH_DEG         = 0.0    # [Góc Pitch] Độ nghiêng thấu kính camera (độ, Dương: cúp xuống sàn, Âm: ngửa lên trời, 0.0: song song sàn)
CAMERA_MAX_MAP_DIST_M    = 2.80   # Cự ly tin cậy tối đa cho Stereo Depth 2D (tránh nhiễu tán xạ > 2.8m)
RAY_SAMPLE_COUNT         = 120    # Số tia quét phân bố đều trên FOV ngang của OAK-D
MAPPING_DEFAULT_SPEED_V  = 0.08   # Tốc độ tiến chuẩn khi lập bản đồ (8% PWM) - Rất êm dịu, triệt tiêu nhòe camera
MAPPING_DEFAULT_SPEED_W  = 0.60   # Tốc độ quay chuẩn khi lập bản đồ (~34 deg/s) - Cho điểm quét sắc nét, không trượt bánh

# ─── CẤU HÌNH ĐẢO KÊNH ĐỘNG CƠ (MOTOR ORIENTATION) ────────────────────────────
SWAP_MOTORS        = False  # False: Khắc phục lỗi xoay trái/phải bị ngược (chuẩn theo lệnh phím A/D)
INVERT_LINEAR      = False  # False: Chiều tiến/lùi đã đồng bộ theo chuẩn Adafruit_MotorHAT Waveshare
INVERT_LEFT_MOTOR  = False  # True: Đảo cực tính bánh trái nếu bị quay lùi khi tiến
INVERT_RIGHT_MOTOR = False  # True: Đảo cực tính bánh phải nếu bị quay lùi khi tiến

# ─── THÔNG SỐ AN TOÀN (HARNESS SAFETY) ────────────────────────────────────────
SAFETY_BRAKE_DIST_M   = 0.20  # Ngưỡng phanh Virtual Bumper: 0.20m (20cm) khóa tiến tức thì
SAFETY_WARNING_DIST_M = 0.40  # Ngưỡng cảnh báo giảm tốc: 0.40m (40cm)
WATCHDOG_TIMEOUT_S    = 0.50  # Thời gian tự ngắt động cơ khi mất lệnh lái

# ─── THÔNG SỐ TỐI ƯU HÓA TÀI NGUYÊN (CPU THROTTLING) ──────────────────────────
DEPTH_SKIP_FRAMES = 1       # Cập nhật mây điểm 3D tức thời (đã vector hóa NumPy siêu tốc < 0.8ms)
DEPTH_DOWNSAMPLE_STEP = 25  # Bước nhảy lấy mẫu ma trận điểm ảnh (pixel)
IMAGE_SKIP_FRAMES = 1       # Phát stream liên tục mượt mà 20-30 FPS (không drop frame ảo)

# ─── THÔNG SỐ TỰ QUÉT 360 ĐỘ & ĐỊNH HƯỚNG THỊ GIÁC CAMERA (VISUAL ANCHOR 360° SCAN) ───
AUTO_SCAN_MODE        = "visual_anchor" # Tự động đánh dấu khung hình mốc & quay chậm đều khép vòng lặp Camera
AUTO_SCAN_SPEED_PWM   = 0.085           # Mức xung PWM quay đều êm dịu (8.5% PWM) chống nhòe camera
AUTO_SCAN_CREEP_PWM   = 0.065           # Mức xung PWM bò chậm khi đến gần 360° (score >= 0.50) để hãm đà quán tính
AUTO_SCAN_TARGET_DEG  = 360.0           # Góc mục tiêu chuẩn 1 vòng tròn 360 độ (2*pi rad)
AUTO_SCAN_DURATION_S  = 6.5             # Thời gian quay từ tốn (tự bù áp theo INA219)

# ─── THÔNG SỐ HIỆU CHUẨN ĐỒNG NHẤT HỆ THỐNG (UNIFIED MOTION CALIBRATION) ────────────
# [DÙNG CHUNG DUY NHẤT 1 BỘ THAM SỐ CHO CẢ 2 CHẾ ĐỘ: BƯỚC ĐỊNH LƯỢNG & LÁI THỦ CÔNG / ODOMETRY]
# Tỉ lệ bù trừ giữa thế giới thực tế và mô hình lý thuyết:
#   - Nếu xe thực tế đi xa gấp 2 lần giá trị set -> đặt SCALE = 0.50
#   - Ở chế độ Bước: Thời gian chạy tự động nhân với SCALE để dừng đúng cự ly mong muốn.
#   - Ở chế độ Thủ công (WASD): Bộ đo hành trình & Bản đồ SLAM tự động quy đổi vận tốc thực (v / SCALE)
#     để đo đúng 100% khoảng cách xe đã di chuyển trên sàn thực tế!
STEP_FORWARD_SCALE       = 0.42  # Hệ số hiệu chuẩn TIẾN (dùng chung cho cả Bước cm và Lái tay WASD)
STEP_BACKWARD_SCALE      = 0.48  # Hệ số hiệu chuẩn LÙI (bù lực cản bánh mắt trâu khi bị kéo ngược)
STEP_TURN_LEFT_SCALE     = 0.60  # Hệ số hiệu chuẩn QUAY TRÁI (CCW - dự phòng vòng hở khi không có IMU)
STEP_TURN_RIGHT_SCALE    = 0.60  # Hệ số hiệu chuẩn QUAY PHẢI (CW - dự phòng vòng hở khi không có IMU)
DEFAULT_SPEED_PWM        = 0.06  # Mức ga cơ bản chạy thẳng (6% PWM - chạy êm, đầm, không trượt bánh, không nhòe cam)

# ─── BÙ MÔ-MEN XOAY TẠI CHỖ (IN-PLACE TURN TORQUE COMPENSATION) ────────────────
STEP_TURN_MIN_PWM        = 0.11  # Mức xung PWM tối thiểu khi quay tại chỗ (11% đủ thắng ma sát trượt ngang)
STEP_TURN_SPEED_RATIO    = 1.0   # Tỉ lệ điều chỉnh tốc độ quay so với tốc độ tiến

# ─── GIẢM TỐC TỪ TỪ ĐỒNG NHẤT 100% (UNIFIED SMOOTH DECELERATION & SOFT STOP) ───
ENABLE_SOFT_STOP         = True  # Cờ tổng thể: True = Dừng êm từ từ | False = Cắt điện tức thì
MANUAL_SOFT_DECEL        = True  # Giữ tương thích: Cho phép hạ dốc mượt mà khi dừng xe
MANUAL_DECEL_TIME_MS     = 180   # Thời gian hạ dốc mượt mà (180ms) dùng chung cho cả Lái tay và Bước cm/độ
STEP_USE_ACTIVE_BRAKE    = False # False: Ngắt êm tự nhiên (không giật ngược). True: Dùng xung phanh ngược cứng
# ─── THÔNG SỐ TỰ HÀNH BÁM ĐƯỜNG (AUTONOMOUS NAVIGATOR & PATH FOLLOWER) ───────────
ENABLE_AUTONOMOUS_NAVIGATOR = True  # Bật bộ điều khiển bám đường tự hành A*
NAV_LINEAR_SPEED            = 0.10  # Vận tốc tiến tự hành (m/s) đầm xe, ổn định camera
NAV_TURN_SPEED              = 0.70  # Vận tốc quay tại chỗ (rad/s ~ 40 deg/s) êm dịu, không giật mất đường
NAV_GOAL_TOLERANCE_M        = 0.06  # Bán kính chạm đích dừng xe (6cm)
NAV_WAYPOINT_TOLERANCE_M    = 0.10  # Bán kính chuyển waypoint tiếp theo (10cm)
NAV_HEADING_TOLERANCE_RAD   = 0.35  # Dung sai góc lệch (~20 độ) trước khi chuyển từ quay sang tiến
NAV_KP_ANGULAR              = 1.4   # Hệ số P chỉnh lái bám đường mượt mà
ENABLE_FRAME_RECORDER       = False # True: Ghi frame chẩn đoán xuống thư mục logs khi di chuyển

# ══════════════════════════════════════════════════════════════════════════════
# 🗂️ HỆ THỐNG PROFILES CẤU HÌNH ĐỘNG HỌC & ĐỊA HÌNH (RUNTIME PROFILE MANAGEMENT)
# ══════════════════════════════════════════════════════════════════════════════
ACTIVE_CONFIG_PROFILE = 1
PROFILES = {}

def load_profiles_from_disk():
    global ACTIVE_CONFIG_PROFILE, PROFILES
    import json, os
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    prof_file = os.path.join(cur_dir, "config_profiles.json")
    if os.path.exists(prof_file):
        try:
            with open(prof_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            ACTIVE_CONFIG_PROFILE = int(data.get("active_profile", 1))
            PROFILES = {int(k): v for k, v in data.get("profiles", {}).items()}
        except Exception:
            pass

    if not PROFILES:
        PROFILES = {}

    # Profile 1 luôn là Chuẩn Mặc Định kế thừa 100% từ các giá trị khai báo trực tiếp trong config.py:
    # (Đảm bảo người dùng sửa trực tiếp trong config.py là hệ thống nhận ngay lập tức, không bao giờ bị ghi đè)
    PROFILES[1] = {
        "id": 1,
        "name": "Config 1: Chuẩn Mặc Định (Giáp Master)",
        "description": "Hiệu chuẩn chuẩn xác: Bù ga L:1.000 R:1.020, Bước Tiến 0.42 / Lùi 0.48, Khóa hướng Kp=0.50, Ga 8%",
        "motor_trim_left": MOTOR_TRIM_LEFT,
        "motor_trim_right": MOTOR_TRIM_RIGHT,
        "step_forward_scale": STEP_FORWARD_SCALE,
        "step_backward_scale": STEP_BACKWARD_SCALE,
        "step_turn_left_scale": STEP_TURN_LEFT_SCALE,
        "step_turn_right_scale": STEP_TURN_RIGHT_SCALE,
        "default_speed_pwm": DEFAULT_SPEED_PWM,
        "step_turn_min_pwm": STEP_TURN_MIN_PWM,
        "manual_decel_time_ms": MANUAL_DECEL_TIME_MS,
        "step_turn_start_latency_s": 0.08,
        "heading_lock_kp": HEADING_LOCK_KP,
        "safety_brake_dist_m": SAFETY_BRAKE_DIST_M,
        "safety_warn_dist_m": SAFETY_WARNING_DIST_M,
        "camera_offset_forward_m": CAMERA_OFFSET_FORWARD_M,
        "camera_offset_lateral_m": CAMERA_OFFSET_LATERAL_M,
        "camera_height_m": CAMERA_HEIGHT_M,
        "camera_pitch_deg": CAMERA_PITCH_DEG,
        "wheel_separation_m": WHEEL_SEPARATION_M,
        "created_at": "config.py (Gốc chuẩn)"
    }

    # Nạp trim động cơ từ robot_dynamics.json nếu có (chỉ kế thừa trim cân bằng 2 bánh, không đè lên tham số config.py)
    dyn_file = os.path.join(cur_dir, "robot_dynamics.json")
    if os.path.exists(dyn_file):
        try:
            with open(dyn_file, "r", encoding="utf-8") as f:
                dyn = json.load(f)
            if 1 in PROFILES:
                if "motor_trim_left" in dyn: PROFILES[1]["motor_trim_left"] = float(dyn["motor_trim_left"])
                if "motor_trim_right" in dyn: PROFILES[1]["motor_trim_right"] = float(dyn["motor_trim_right"])
        except Exception:
            pass

def apply_profile(profile_id: int):
    global ACTIVE_CONFIG_PROFILE
    global MOTOR_TRIM_LEFT, MOTOR_TRIM_RIGHT
    global STEP_FORWARD_SCALE, STEP_BACKWARD_SCALE
    global STEP_TURN_LEFT_SCALE, STEP_TURN_RIGHT_SCALE
    global DEFAULT_SPEED_PWM, STEP_TURN_MIN_PWM
    global MANUAL_DECEL_TIME_MS, STEP_TURN_START_LATENCY_S
    global HEADING_LOCK_KP, SAFETY_BRAKE_DIST_M, SAFETY_WARN_DIST_M
    global CAMERA_OFFSET_FORWARD_M, CAMERA_OFFSET_LATERAL_M, CAMERA_HEIGHT_M, CAMERA_PITCH_DEG, WHEEL_SEPARATION_M

    pid = int(profile_id)
    if pid not in PROFILES:
        return False
    ACTIVE_CONFIG_PROFILE = pid
    cur = PROFILES[pid]
    MOTOR_TRIM_LEFT           = cur.get("motor_trim_left", 1.000)
    MOTOR_TRIM_RIGHT          = cur.get("motor_trim_right", 1.020)
    STEP_FORWARD_SCALE        = cur.get("step_forward_scale", 0.42)
    STEP_BACKWARD_SCALE       = cur.get("step_backward_scale", 0.48)
    STEP_TURN_LEFT_SCALE      = cur.get("step_turn_left_scale", 0.60)
    STEP_TURN_RIGHT_SCALE     = cur.get("step_turn_right_scale", 0.60)
    DEFAULT_SPEED_PWM         = cur.get("default_speed_pwm", 0.08)
    STEP_TURN_MIN_PWM         = cur.get("step_turn_min_pwm", 0.11)
    MANUAL_DECEL_TIME_MS      = cur.get("manual_decel_time_ms", 180)
    STEP_TURN_START_LATENCY_S = cur.get("step_turn_start_latency_s", 0.08)
    HEADING_LOCK_KP           = cur.get("heading_lock_kp", 0.50)
    SAFETY_BRAKE_DIST_M       = cur.get("safety_brake_dist_m", 0.20)
    SAFETY_WARN_DIST_M        = cur.get("safety_warn_dist_m", 0.45)
    CAMERA_OFFSET_FORWARD_M   = cur.get("camera_offset_forward_m", 0.090)
    CAMERA_OFFSET_LATERAL_M   = cur.get("camera_offset_lateral_m", -0.015)
    CAMERA_HEIGHT_M           = cur.get("camera_height_m", 0.120)
    CAMERA_PITCH_DEG          = cur.get("camera_pitch_deg", 0.0)
    WHEEL_SEPARATION_M        = cur.get("wheel_separation_m", 0.120)
    return True

def save_profiles_to_disk():
    import json, os
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    prof_file = os.path.join(cur_dir, "config_profiles.json")
    try:
        data = {
            "active_profile": ACTIVE_CONFIG_PROFILE,
            "profiles": {str(k): v for k, v in PROFILES.items()}
        }
        with open(prof_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return True
    except Exception:
        return False

# Nạp profiles và áp dụng ngay khi khởi tạo
load_profiles_from_disk()
apply_profile(ACTIVE_CONFIG_PROFILE)
