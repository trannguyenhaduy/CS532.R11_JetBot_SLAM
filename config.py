#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hệ thống cấu hình tập trung (Centralized Configuration System)
Quản lý cờ tính năng (Feature Flags), thông số động học, hình học xe và thuật toán SLAM/A*.
"""

# ==============================================================================
# KHỐI 1: THÔNG TIN PHIÊN BẢN HỆ THỐNG & CHẾ ĐỘ VẬN HÀNH
# ==============================================================================
SYSTEM_VERSION     = "v2.4.0-SAFETY-DUAL-ROS"
VERSION_CODENAME   = "AEGIS JETBOT"
BUILD_TAG          = "v2.4.0-safety-dual-ros"
WORKFLOW_MODE      = "1-Terminal Mode"


# ==============================================================================
# KHỐI 2: CỜ BẬT / TẮT CÁC PHÂN HỆ TÍNH NĂNG (FEATURE FLAGS)
# ==============================================================================
ENABLE_MOTORS               = True   # Kích hoạt module điều khiển động cơ (I2C PCA9685 0x60)
ENABLE_BATTERY              = True   # Kích hoạt module đo điện áp pin (I2C INA219 0x41)
ENABLE_CAMERA               = True   # Kích hoạt module Camera OAK-D S2
ENABLE_IMU                  = True   # Kích hoạt cảm biến quán tính IMU phần cứng trên OAK-D S2
ENABLE_CLOSED_LOOP_IMU      = False  # Điều khiển quay vòng hở theo thời gian, IMU đo góc thực tế
ENABLE_YOLO                 = True   # Kích hoạt module nhận diện vật thể AI (VPU / Spatial Detector)
AI_CONFIDENCE_THRESHOLD     = 0.40   # Ngưỡng độ tin cậy tối thiểu để hiển thị đối tượng nhận diện
ENABLE_FOLLOWER             = False  # Module bám người (mặc định tắt, ưu tiên điều hướng tự hành)
ENABLE_MAPPER               = True   # Kích hoạt module bản đồ 2D Occupancy Grid SLAM
ENABLE_SEMANTIC_LANDMARKS   = True   # Kích hoạt gán nhãn mốc vật thể ngữ nghĩa trên bản đồ 2D
SEMANTIC_PIN_MIN_CONFIDENCE = 0.70   # Ngưỡng tin cậy tối thiểu để ghim nhãn mốc lên bản đồ
SEMANTIC_MIN_SEEN_CONFIRM   = 2      # Số lần nhận diện tối thiểu để xác nhận vị trí mốc
SEMANTIC_MERGE_DIST_M       = 0.50   # Bán kính gom cụm khử trùng lặp mốc không gian (mét)
SEMANTIC_NMS_IOU_THRESH     = 0.38   # Ngưỡng IoU Non-Maximum Suppression trên mặt phẳng ảnh
SEMANTIC_DRAW_BBOX_MAP      = False  # Hiển thị mốc dạng điểm tọa độ và nhãn trên bản đồ 2D
ENABLE_SCAN_MATCHING        = True   # Kích hoạt thuật toán Correlative Scan Matching (CSM) 2D
ENABLE_VISUAL_GYRO          = False  # Con quay thị giác (Visual Gyroscope)
SCAN_MATCH_SEARCH_DIST_M    = 0.08   # Bán kính cửa sổ tìm kiếm vị trí của CSM (mét)
SCAN_MATCH_SEARCH_YAW_DEG   = 0.0    # Góc tìm kiếm hướng của CSM (0.0: khóa góc Yaw, chỉ nắn vị trí)
SCAN_MATCH_MIN_SCORE        = 0.28   # Ngưỡng điểm số khớp tối thiểu để chấp nhận hiệu chỉnh vị trí
ENABLE_WEB                  = True   # Kích hoạt trạm điều khiển Web Cockpit
ENABLE_SAFETY_BRAKE         = True   # Kích hoạt hệ thống phanh an toàn Virtual Bumper
ENABLE_PATH_PLANNER         = True   # Kích hoạt thuật toán lập kế hoạch đường đi toàn cục A*
ENABLE_AUTONOMOUS_NAVIGATOR = True   # Kích hoạt bộ điều khiển bám đường tự hành


# ==============================================================================
# KHỐI 3: THÔNG SỐ HÌNH HỌC KHUNG XE & NGOẠI TẠI CẢM BIẾN (EXTRINSICS)
# ==============================================================================
# Gốc tọa độ xe (0, 0, 0) đặt tại mặt sàn, chính giữa đường nối tâm hai bánh xe chủ động
ROBOT_LENGTH_M              = 0.180  # Chiều dài thân xe (mét)
ROBOT_WIDTH_M               = 0.130  # Khoảng cách cơ học giữa hai bánh xe (mét)
ROBOT_FRONT_FROM_AXLE_M     = 0.080  # Khoảng cách từ trục bánh đến mũi trước xe (mét)
ROBOT_REAR_FROM_AXLE_M      = 0.100  # Khoảng cách từ trục bánh đến đuôi sau xe (mét)
ROBOT_RIGHT_EXTRUSION_M     = 0.000  # Độ nhô phần cứng bên sườn phải xe (mét)
ROBOT_LEFT_EXTRUSION_M      = 0.070  # Độ nhô cáp kết nối bên sườn trái xe (mét)
NAV_SAFE_ROTATION_RADIUS_M  = 0.168  # Bán kính an toàn quét xoay tại chỗ: sqrt(0.10^2 + 0.135^2)
ROBOT_RADIUS_M              = 0.110  # Bán kính thân xe cơ sở cho vùng LETHAL A* (mét)
OBSTACLE_ALERT_MARGIN_M     = 0.060  # Bán kính vùng cảnh giác cận cản (Alert Cushion) quanh vật cản (mét)
ROBOT_CLEAR_PADDING_M       = 0.035  # Vùng đệm xóa footprint thân xe trên bản đồ SLAM (mét)
COSTMAP_INFLATION_RADIUS_M  = 0.220  # Bán kính phình to vùng đệm an toàn trên Costmap (mét)
ENABLE_ASYMMETRIC_CLEARANCE = True   # Bù lệch hướng sang phải để bảo vệ cáp nhô bên sườn trái
LEFT_CABLE_SAFETY_BIAS_M    = 0.035  # Độ lệch an toàn sang phải khi qua hành lang hẹp (mét)

# Vị trí không gian của camera OAK-D S2 so với gốc tọa độ xe
WHEEL_SEPARATION_M          = 0.120  # Khoảng cách giữa 2 tâm vệt tiếp xúc bánh xe (mét)
CAMERA_OFFSET_FORWARD_M     = 0.050  # Khoảng cách từ trục bánh trước đến thấu kính camera (mét)
CAMERA_OFFSET_LATERAL_M     = -0.015 # Độ lệch ngang thấu kính (âm: lệch phải, dương: lệch trái)
CAMERA_HEIGHT_M             = 0.120  # Chiều cao từ mặt sàn đến tâm thấu kính (mét)
CAMERA_PITCH_DEG            = 3.5    # Góc nghiêng thấu kính so với phương ngang (độ)
CAMERA_MAX_MAP_DIST_M       = 2.80   # Cự ly đo đạc hiệu dụng tối đa cho bản đồ 2D (mét)
RAY_SAMPLE_COUNT            = 120    # Số tia quét lấy mẫu từ mây điểm Stereo Depth


# ==============================================================================
# KHỐI 4: HIỆU CHUẨN ĐỘNG CƠ & CHUYỂN ĐỘNG VI SAI
# ==============================================================================
I2C_BUS                     = 1
PCA9685_ADDR                = 0x60
INA219_ADDR                 = 0x41
SWAP_MOTORS                 = False  # Đảo kênh động cơ trái / phải
INVERT_LINEAR               = False  # Đảo chiều tiến / lùi tổng thể
INVERT_LEFT_MOTOR           = True   # Đảo chiều cực tính động cơ trái
INVERT_RIGHT_MOTOR          = True   # Đảo chiều cực tính động cơ phải

# Vận tốc giới hạn
MAX_LINEAR_SPEED            = 0.20   # Vận tốc tiến tối đa (m/s)
MAX_ANGULAR_SPEED           = 1.60   # Vận tốc quay tối đa (rad/s)
DEFAULT_SPEED_PWM           = 0.15   # Mức xung PWM cơ bản khi chạy thẳng
MAPPING_DEFAULT_SPEED_V     = 0.15   # Vận tốc tiến khi lập bản đồ
MAPPING_DEFAULT_SPEED_W     = 0.85   # Vận tốc quay khi lập bản đồ

# Hệ số hiệu chuẩn quãng đường và góc quay thực tế
STEP_FORWARD_SCALE          = 0.43   # Tỉ lệ hiệu chuẩn bước tiến
STEP_BACKWARD_SCALE         = 0.48   # Tỉ lệ hiệu chuẩn bước lùi
STEP_TURN_LEFT_SCALE        = 0.70   # Tỉ lệ hiệu chuẩn quay trái
STEP_TURN_RIGHT_SCALE       = 0.70   # Tỉ lệ hiệu chuẩn quay phải
STEP_TURN_MIN_PWM           = 0.11   # Mức xung PWM tối thiểu thắng ma sát tĩnh khi quay tại chỗ
STEP_TURN_SPEED_RATIO       = 1.0    # Tỉ số tốc độ quay so với tốc độ tiến
MOTOR_TRIM_LEFT             = 0.995  # Bù trừ hệ số động cơ trái khi tiến
MOTOR_TRIM_RIGHT            = 1.015  # Bù trừ hệ số động cơ phải khi tiến
MOTOR_TRIM_BACKWARD_LEFT    = 1.010  # Bù trừ hệ số động cơ trái khi lùi
MOTOR_TRIM_BACKWARD_RIGHT   = 0.990  # Bù trừ hệ số động cơ phải khi lùi
ENABLE_HEADING_LOCK         = False  # Khóa hướng chuyển động bằng bộ điều khiển phản hồi
HEADING_LOCK_KP             = 0.50   # Hệ số khuếch đại tỉ lệ P khóa hướng
ENABLE_SOFT_STOP            = True   # Dừng động cơ có kiểm soát gia tốc hạ dốc
MANUAL_SOFT_DECEL           = True   # Giảm tốc mượt mà khi dừng lái thủ công
MANUAL_DECEL_TIME_MS        = 180    # Thời gian hạ dốc gia tốc dừng (ms)
STEP_USE_ACTIVE_BRAKE       = False  # Xung phanh ngược tích cực khi dừng bước


# ==============================================================================
# KHỐI 5: HỆ THỐNG AN TOÀN & PHANH TỰ HÀNH (VIRTUAL BUMPER)
# ==============================================================================
SAFETY_BRAKE_DIST_M         = 0.140  # Khoảng cách dừng khẩn cấp Virtual Bumper (mét)
SAFETY_WARNING_DIST_M       = 0.280  # Khoảng cách kích hoạt cảnh báo cận cản (mét)
LANE_MARGIN_M               = 0.010  # Dung sai bề rộng vệt di chuyển thân xe (mét)
OBSTACLE_MIN_HEIGHT_M       = 0.070  # Độ cao tối thiểu so với sàn để phân loại cản tĩnh (mét)
SAFETY_BRAKE_MIN_HEIGHT_M   = 0.035  # Độ cao tối thiểu kích hoạt phanh khẩn cấp (mét)
WATCHDOG_TIMEOUT_S          = 0.50   # Thời gian ngắt động cơ khi mất tín hiệu điều khiển (giây)


# ==============================================================================
# KHỐI 6: CẤU HÌNH STEREO DEPTH & QUÉT BẢN ĐỒ
# ==============================================================================
DEPTH_SKIP_FRAMES           = 1      # Bước nhảy khung hình độ sâu
DEPTH_DOWNSAMPLE_STEP       = 25     # Bước lấy mẫu không gian trên ma trận điểm ảnh
IMAGE_SKIP_FRAMES           = 1      # Bước nhảy khung hình stream RGB
AUTO_SCAN_MODE              = "visual_anchor" # Chế độ quay quét lập bản đồ
AUTO_SCAN_SPEED_PWM         = 0.12   # Mức xung PWM quay quét quan sát
AUTO_SCAN_CREEP_PWM         = 0.09   # Mức xung PWM hãm đà quán tính trước khi dừng
AUTO_SCAN_TARGET_DEG        = 360.0  # Góc quét mục tiêu (độ)
AUTO_SCAN_DURATION_S        = 4.8    # Thời gian ước lượng một chu kỳ quét (giây)


# ==============================================================================
# KHỐI 7: BẢN ĐỒ 2D OCCUPANCY SLAM & ĐIỀU HƯỚNG
# ==============================================================================
PATH_PLAN_MIN_CONFIDENCE_HITS = 3    # Số lần quét trúng tối thiểu để xác nhận tường cố định
PATH_PLAN_UNKNOWN_SPACE_PENALTY = 4.5 # Trọng số phạt chi phí khi đi qua vùng chưa khám phá
NAV_ALLOW_UNKNOWN_EXPLORATION   = True # Cho phép tìm đường qua vùng chưa khám phá khi cần
PATH_PLAN_ALLOW_TIGHT_GAPS  = True   # Cho phép lập đường đi qua khe hẹp an toàn (>= 28cm)
PATH_PLAN_BUFFER_PENALTY_SCALE = 0.015 # Hệ số tỉ lệ phạt vùng đệm an toàn mềm
PATH_PLAN_TIGHT_LETHAL_RADIUS_M = 0.090 # Bán kính cản LETHAL khi qua khe hẹp thích nghi (mét)
PATH_PLAN_TIGHT_INFLATION_RADIUS_M = 0.130 # Bán kính phình to khi qua khe hẹp thích nghi (mét)
PATH_PLAN_DOUGLAS_PEUCKER_EPSILON_M = 0.060 # Ngưỡng tối giản điểm mốc trung gian (mét)


# ==============================================================================
# KHỐI 8: BỘ ĐIỀU KHIỂN TỰ HÀNH & CƠ CHẾ CẢNH GIÁC CẬN CẢN
# ==============================================================================
NAV_MODE                    = "stepped" # Chế độ tự hành từng bước (Sense-Plan-Act)
NAV_STEP_DIST_CM            = 40.0   # Độ dài bước tiến cơ sở (cm)
NAV_DYNAMIC_STEP_ENABLED    = True   # Tự động điều chỉnh bước tiến theo độ thoáng không gian
NAV_STEP_MAX_DIST_CM        = 55.0   # Độ dài bước tiến tối đa khi không gian thoáng (cm)
NAV_STEP_MIN_DIST_CM        = 25.0   # Độ dài bước tiến tối thiểu khi gần chướng ngại vật (cm)
NAV_PROXIMITY_ALERT_DIST_M  = 0.260  # Cự ly kích hoạt trạng thái Cảnh Giác Cận Cản (mét)
NAV_PROXIMITY_STEP_MAX_CM   = 15.0   # Độ dài bước tối đa khi trong vùng cảnh giác (cm)
NAV_PROXIMITY_LOOK_AGAIN    = True   # Kích hoạt cơ chế nhìn lại cản trước khi bước qua
NAV_PROXIMITY_PEEK_DEG      = 22.0   # Góc liếc nhìn lại mép cản bên sườn (độ)
NAV_PROXIMITY_SAFE_CLEARANCE_M = 0.180 # Cự ly tối thiểu sau khi nhìn lại để cho phép bước (mét)
NAV_COSTMAP_AWARE_PEEK      = True   # Bỏ qua liếc góc cua nếu vùng rẽ đã được xác thực thoáng
VIRTUAL_OBSTACLE_RADIUS_M   = 0.090  # Bán kính cản ảo tạm thời khi gặp tắc đường (mét)
VIRTUAL_OBSTACLE_DURATION_S = 14.0   # Thời gian duy trì cản ảo tạm thời (giây)
NAV_STUCK_ZONE_RADIUS_M     = 0.250  # Bán kính gom cụm nhận diện vùng kẹt lặp lại (mét)
NAV_STUCK_ZONE_ESCALATE_COUNT = 2    # Số lần kẹt lặp lại để kích hoạt phục hồi nâng cấp
NAV_STUCK_ZONE_ESCALATE_DURATION_S = 60.0 # Thời gian phong tỏa vùng kẹt lặp lại (giây)
NAV_MAX_REPLAN_FAIL_STREAK  = 5      # Số lần thử tìm đường lại tối đa trước khi báo kẹt
NAV_INIT_SCAN_ROUNDS        = 0      # Số vòng quay quan sát khởi tạo khi bắt đầu hành trình
NAV_OBSERVE_SECTOR_DEG      = 45.0   # Góc quay mỗi nấc quan sát (độ)
NAV_OBSERVE_STEPS           = 8      # Số nấc quan sát hoàn thành một chu kỳ 360 độ
NAV_OBSERVE_MODE            = "event_driven" # Chiến thuật quan sát theo sự kiện
NAV_KNOWN_FREE_MAX_STEP_CM  = 80.0   # Bước tiến trên hành lang đã xác nhận an toàn (cm)
NAV_MAX_CROSS_TRACK_ERR_M   = 0.280  # Ngưỡng lệch hành lang tối đa trước khi replan (mét)
NAV_PEEK_ANGLE_DEG          = 40.0   # Góc liếc quan sát góc cua thông thường (độ)
NAV_PEEK_TRIGGER_ANGLE_DEG  = 30.0   # Ngưỡng góc rẽ kích hoạt liếc quan sát (độ)
NAV_PEEK_TRIGGER_DIST_M     = 0.650  # Cự ly cản trước mặt kích hoạt liếc quan sát (mét)
NAV_SWEEP_ANGLE_DEG         = 55.0   # Góc quét bán nguyệt mở rộng tầm nhìn (độ)
NAV_SWEEP_INTERVAL_M        = 1.50   # Quãng đường chạy thẳng tích lũy kích hoạt quét bán nguyệt (mét)
NAV_SWEEP_INTERVAL_STEPS    = 5      # Số bước chạy thẳng tích lũy kích hoạt quét bán nguyệt
NAV_FINAL_SPRINT_DIST_M     = 1.20   # Cự ly bắt đầu giai đoạn tiếp cận đích cuối (mét)
NAV_LINEAR_SPEED            = 0.120  # Vận tốc tiến khi tự hành (m/s)
NAV_TURN_SPEED              = 1.200  # Vận tốc quay tại chỗ khi tự hành (rad/s)
NAV_GOAL_TOLERANCE_M        = 0.060  # Sai số vị trí chấp nhận đến đích (mét)
NAV_WAYPOINT_TOLERANCE_M    = 0.100  # Bán kính chuyển tiếp điểm mốc waypoint (mét)
NAV_HEADING_TOLERANCE_RAD   = 0.350  # Dung sai góc hướng trước khi chuyển sang tiến (rad)
NAV_KP_ANGULAR              = 1.600  # Hệ số khuếch đại tỉ lệ P bám hướng


# ==============================================================================
# KHỐI 9: CẤU HÌNH TRẠM ĐIỀU KHIỂN WEB & TRUYỀN STREAM
# ==============================================================================
WEB_PORT                    = 8080
WEB_HOST                    = '0.0.0.0'
ENABLE_FRAME_RECORDER       = False  # Ghi khung hình và dữ liệu xuống đĩa lưu trữ
STREAM_MAX_FPS              = 30     # Tần số khung hình phát luồng video (FPS)
STREAM_JPEG_QUALITY         = 55     # Chất lượng nén ảnh JPEG (1-100)
STREAM_WIDTH                = 640    # Chiều rộng khung hình truyền phát (pixel)
STREAM_HEIGHT               = 480    # Chiều cao khung hình truyền phát (pixel)


# ==============================================================================
# KHỐI 10: HỆ THỐNG HỒ SƠ CẤU HÌNH ĐỘNG HỌC (RUNTIME CONFIG PROFILES)
# ==============================================================================
ACTIVE_CONFIG_PROFILE = 1
PROFILES = {}

def load_profiles_from_disk():
    global ACTIVE_CONFIG_PROFILE, PROFILES
    ACTIVE_CONFIG_PROFILE = 1
    PROFILES = {
        1: {
            "id": 1,
            "name": "Config 1: Mac Dinh (config.py)",
            "description": "Thong so tieu chuan cho khung xe JetBot va camera OAK-D S2",
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
            "created_at": "config.py"
        }
    }

def apply_profile(profile_id: int):
    """Áp dụng hồ sơ cấu hình vào bộ nhớ runtime."""
    global ACTIVE_CONFIG_PROFILE
    if not PROFILES:
        load_profiles_from_disk()
    pid = int(profile_id)
    if pid not in PROFILES:
        return False
    ACTIVE_CONFIG_PROFILE = pid
    p = PROFILES[pid]
    global MOTOR_TRIM_LEFT, MOTOR_TRIM_RIGHT, MOTOR_TRIM_BACKWARD_LEFT, MOTOR_TRIM_BACKWARD_RIGHT
    global STEP_FORWARD_SCALE, STEP_BACKWARD_SCALE, STEP_TURN_LEFT_SCALE, STEP_TURN_RIGHT_SCALE
    global DEFAULT_SPEED_PWM, STEP_TURN_MIN_PWM, MANUAL_DECEL_TIME_MS, HEADING_LOCK_KP
    global SAFETY_BRAKE_DIST_M, SAFETY_WARNING_DIST_M
    global CAMERA_OFFSET_FORWARD_M, CAMERA_OFFSET_LATERAL_M, CAMERA_HEIGHT_M, CAMERA_PITCH_DEG, WHEEL_SEPARATION_M
    MOTOR_TRIM_LEFT = float(p.get("motor_trim_left", MOTOR_TRIM_LEFT))
    MOTOR_TRIM_RIGHT = float(p.get("motor_trim_right", MOTOR_TRIM_RIGHT))
    MOTOR_TRIM_BACKWARD_LEFT = float(p.get("motor_trim_backward_left", MOTOR_TRIM_BACKWARD_LEFT))
    MOTOR_TRIM_BACKWARD_RIGHT = float(p.get("motor_trim_backward_right", MOTOR_TRIM_BACKWARD_RIGHT))
    STEP_FORWARD_SCALE = float(p.get("step_forward_scale", STEP_FORWARD_SCALE))
    STEP_BACKWARD_SCALE = float(p.get("step_backward_scale", STEP_BACKWARD_SCALE))
    STEP_TURN_LEFT_SCALE = float(p.get("step_turn_left_scale", STEP_TURN_LEFT_SCALE))
    STEP_TURN_RIGHT_SCALE = float(p.get("step_turn_right_scale", STEP_TURN_RIGHT_SCALE))
    DEFAULT_SPEED_PWM = float(p.get("default_speed_pwm", DEFAULT_SPEED_PWM))
    STEP_TURN_MIN_PWM = float(p.get("step_turn_min_pwm", STEP_TURN_MIN_PWM))
    MANUAL_DECEL_TIME_MS = int(p.get("manual_decel_time_ms", MANUAL_DECEL_TIME_MS))
    HEADING_LOCK_KP = float(p.get("heading_lock_kp", HEADING_LOCK_KP))
    SAFETY_BRAKE_DIST_M = float(p.get("safety_brake_dist_m", SAFETY_BRAKE_DIST_M))
    SAFETY_WARNING_DIST_M = float(p.get("safety_warn_dist_m", SAFETY_WARNING_DIST_M))
    CAMERA_OFFSET_FORWARD_M = float(p.get("camera_offset_forward_m", CAMERA_OFFSET_FORWARD_M))
    CAMERA_OFFSET_LATERAL_M = float(p.get("camera_offset_lateral_m", CAMERA_OFFSET_LATERAL_M))
    CAMERA_HEIGHT_M = float(p.get("camera_height_m", CAMERA_HEIGHT_M))
    CAMERA_PITCH_DEG = float(p.get("camera_pitch_deg", CAMERA_PITCH_DEG))
    WHEEL_SEPARATION_M = float(p.get("wheel_separation_m", WHEEL_SEPARATION_M))
    return True

def save_profiles_to_disk():
    return True

load_profiles_from_disk()
apply_profile(ACTIVE_CONFIG_PROFILE)
