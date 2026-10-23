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
ENABLE_YOLO         = True   # BẬT AI nhận diện vật thể & đo khoảng cách 3D
AI_MODEL            = "yolo" # "yolo" (YOLOv8/v6/v5/v4-Tiny 80 lớp COCO, bám khít) hoặc "mobilenet" (MobileNet-SSD 20 lớp)
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
PATH_PLAN_MIN_CONFIDENCE_HITS = 3  # Ngưỡng số lần quét xác nhận tối thiểu để A* coi là vật cản LETHAL (loại bỏ nhiễu đỏ)
ENABLE_HEADING_LOCK       = False  # Mặc định TẮT theo yêu cầu để đi thẳng và lùi thuần cơ khí 100%
HEADING_LOCK_KP           = 0.50   # Hệ số P khóa hướng mượt mà
MOTOR_TRIM_LEFT           = 0.995  # Cân bằng bánh trái khi TIẾN (Forward Wheel Trim)
MOTOR_TRIM_RIGHT          = 1.015  # Cân bằng bánh phải khi TIẾN (Forward Wheel Trim: +1.5% bù ma sát hộp số)
MOTOR_TRIM_BACKWARD_LEFT  = 1.01  # Cân bằng bánh trái khi LÙI (Backward Wheel Trim)
MOTOR_TRIM_BACKWARD_RIGHT = 0.99  # Cân bằng bánh phải khi LÙI (Backward Wheel Trim: tinh chỉnh độc lập khi lùi)

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
MAPPING_DEFAULT_SPEED_V  = 0.15   # Tốc độ tiến chuẩn khi lập bản đồ (15% PWM) - Triệt tiêu nhòe camera
MAPPING_DEFAULT_SPEED_W  = 0.85   # Tốc độ quay chuẩn khi lập bản đồ (~48 deg/s) - Cho điểm quét sắc nét

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
AUTO_SCAN_SPEED_PWM   = 0.12            # Mức xung PWM quay đều êm dịu (12% PWM) chống nhòe camera
AUTO_SCAN_CREEP_PWM   = 0.09            # Mức xung PWM bò chậm khi đến gần 360° (score >= 0.50) để hãm đà quán tính
AUTO_SCAN_TARGET_DEG  = 360.0           # Góc mục tiêu chuẩn 1 vòng tròn 360 độ (2*pi rad)
AUTO_SCAN_DURATION_S  = 4.8             # Thời gian dự phòng khi pin 11.1V (tự bù áp theo INA219)

# ─── THÔNG SỐ HIỆU CHUẨN ĐỒNG NHẤT HỆ THỐNG (UNIFIED MOTION CALIBRATION) ────────────
# [DÙNG CHUNG DUY NHẤT 1 BỘ THAM SỐ CHO CẢ 2 CHẾ ĐỘ: BƯỚC ĐỊNH LƯỢNG & LÁI THỦ CÔNG / ODOMETRY]
# Tỉ lệ bù trừ giữa thế giới thực tế và mô hình lý thuyết:
#   - Nếu xe thực tế đi xa gấp 2 lần giá trị set -> đặt SCALE = 0.50
#   - Ở chế độ Bước: Thời gian chạy tự động nhân với SCALE để dừng đúng cự ly mong muốn.
#   - Ở chế độ Thủ công (WASD): Bộ đo hành trình & Bản đồ SLAM tự động quy đổi vận tốc thực (v / SCALE)
#     để đo đúng 100% khoảng cách xe đã di chuyển trên sàn thực tế!
STEP_FORWARD_SCALE       = 0.43  # Hệ số hiệu chuẩn TIẾN (dùng chung cho cả Bước cm và Lái tay WASD)
STEP_BACKWARD_SCALE      = 0.48  # Hệ số hiệu chuẩn LÙI (0.48 theo cấu hình thực tế của bạn)
STEP_TURN_LEFT_SCALE     = 0.70  # Hệ số hiệu chuẩn QUAY TRÁI (0.70 theo cấu hình thực tế)
STEP_TURN_RIGHT_SCALE    = 0.70  # Hệ số hiệu chuẩn QUAY PHẢI (0.70 theo cấu hình thực tế)
DEFAULT_SPEED_PWM        = 0.15  # Mức ga cơ bản chạy thẳng (15% PWM)

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
NAV_MODE                    = "stepped" # "stepped" (Sense-Plan-Act 30cm) hoặc "continuous" (Pure Pursuit)
NAV_STEP_DIST_CM            = 30.0  # Cự ly mỗi bước tiến/lùi (30cm)
NAV_INIT_SCAN_ROUNDS        = 2     # Số vòng quét 360° khởi tạo ban đầu (2 vòng x 8 nấc 45° = 16 nấc)
NAV_OBSERVE_SECTOR_DEG      = 45.0  # Góc quay mỗi nấc quan sát (45 độ)
NAV_OBSERVE_STEPS           = 8     # Số nấc quay quan sát mỗi khi dừng (8 nấc x 45° = 360°)
NAV_LINEAR_SPEED            = 0.12  # Vận tốc tiến tự hành (m/s) ~15-20% PWM đầm xe
NAV_TURN_SPEED              = 1.20  # Vận tốc quay tại chỗ (rad/s) ~11-13% PWM êm ái
NAV_GOAL_TOLERANCE_M        = 0.06  # Bán kính chạm đích dừng xe (6cm)
NAV_WAYPOINT_TOLERANCE_M    = 0.10  # Bán kính chuyển waypoint tiếp theo (10cm)
NAV_HEADING_TOLERANCE_RAD   = 0.35  # Dung sai góc lệch (~20 độ) trước khi chuyển từ quay sang tiến
NAV_KP_ANGULAR              = 1.6   # Hệ số P chỉnh lái bám đường
ENABLE_FRAME_RECORDER       = True  # True: Ghi frame chẩn đoán xuống thư mục logs khi di chuyển

# ══════════════════════════════════════════════════════════════════════════════
# 🗂️ HỆ THỐNG PROFILES CẤU HÌNH ĐỘNG HỌC & ĐỊA HÌNH (RUNTIME PROFILE MANAGEMENT)
# ══════════════════════════════════════════════════════════════════════════════
ACTIVE_CONFIG_PROFILE = 1
PROFILES = {}

def load_profiles_from_disk():
    global ACTIVE_CONFIG_PROFILE, PROFILES
    ACTIVE_CONFIG_PROFILE = 1
    # Profile 1 kế thừa 100% từ các giá trị khai báo trực tiếp trong config.py (Nguồn chuẩn duy nhất)
    PROFILES = {
        1: {
            "id": 1,
            "name": "Config 1: Chuẩn Mặc Định (config.py)",
            "description": "Hiệu chuẩn chuẩn xác: Bù ga Tiến L:1.000 R:1.015, Lùi L:1.000 R:1.000, Bước Tiến 0.42 / Lùi 0.48, Ga 15%",
            "motor_trim_left": MOTOR_TRIM_LEFT,
            "motor_trim_right": MOTOR_TRIM_RIGHT,
            "motor_trim_backward_left": MOTOR_TRIM_BACKWARD_LEFT,
            "motor_trim_backward_right": MOTOR_TRIM_BACKWARD_RIGHT,
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
            "created_at": "config.py (Gốc chuẩn duy nhất)"
        }
    }

def apply_profile(profile_id: int):
    # Đồng bộ Profile 1 từ các biến config.py
    load_profiles_from_disk()
    return True

def save_profiles_to_disk():
    # config.py là nguồn chuẩn duy nhất, không ghi file JSON độc lập
    return True

# Nạp profiles và áp dụng ngay khi khởi tạo
load_profiles_from_disk()
apply_profile(ACTIVE_CONFIG_PROFILE)
