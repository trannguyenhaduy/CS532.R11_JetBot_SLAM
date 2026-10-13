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
BUILD_DATE         = "2026-10-07"
WORKFLOW_MODE      = "2-Terminal Mode (Terminal 1: camera_ai.launch | Terminal 2: main.py)"

# ─── CỜ BẬT / TẮT TÍNH NĂNG (FEATURE FLAGS) ───────────────────────────────────
ENABLE_MOTORS       = True   # Bật/Tắt module động cơ (PCA9685 I2C 0x60)
ENABLE_BATTERY      = True   # Bật/Tắt module đọc pin INA219 (I2C 0x41)
ENABLE_CAMERA       = True   # Bật/Tắt module Camera OAK-D S2
ENABLE_YOLO         = False  # Tạm TẮT (sẵn sàng nạp YOLO VPU khi bật True)
ENABLE_FOLLOWER     = False  # Bật/Tắt module tự hành bám người (MẶC ĐỊNH TẮT ĐỂ ƯU TIÊN LÁI TAY)
ENABLE_MAPPER             = True   # Bật/Tắt module bản đồ 2D Occupancy Grid SLAM
ENABLE_SEMANTIC_LANDMARKS = False  # Mặc định TẮT nhãn AI & Bounding Box trên bản đồ 2D (chỉ hiện tường & vị trí xe thực tế)
ENABLE_SCAN_MATCHING      = True   # Bật thuật toán Correlative Scan Matching (CSM) tự cân bằng tọa độ vòng kín
ENABLE_VISUAL_GYRO        = False  # TẮT con quay thị giác (tránh tự cộng dồn góc xoay ngẫu nhiên khi dừng xe)
SCAN_MATCH_SEARCH_DIST_M  = 0.08   # Cửa sổ tìm kiếm vị trí (±8cm)
SCAN_MATCH_SEARCH_YAW_DEG = 6.0    # Cửa sổ tìm kiếm góc quay (±6 độ)
SCAN_MATCH_MIN_SCORE      = 0.28   # Ngưỡng tin cậy tối thiểu để hít tọa độ (28% số tia khớp tường)
ENABLE_WEB                = True   # Bật/Tắt trạm điều khiển Web Cockpit (Port 8080)
ENABLE_SAFETY_BRAKE       = True   # Bật tính năng phanh khẩn cấp Virtual Bumper (< 20cm)
ENABLE_PATH_PLANNER       = True   # Bật module tìm đường tối ưu A* (Global Path Planner)
ROBOT_RADIUS_M            = 0.12   # Bán kính thân xe JetBot (12cm)
COSTMAP_INFLATION_RADIUS_M = 0.20  # Bán kính vùng đệm an toàn chống cạ tường (20cm)

# ─── THÔNG SỐ TRẠM ĐIỀU KHIỂN WEB ─────────────────────────────────────────────
WEB_PORT = 8080
WEB_HOST = '0.0.0.0'

# ─── THÔNG SỐ PHẦN CỨNG XE (JETBOT BASELINE) ──────────────────────────────────
I2C_BUS = 1
PCA9685_ADDR = 0x60
INA219_ADDR  = 0x41

WHEEL_SEPARATION_M = 0.12  # Khoảng cách 2 bánh vi sai (mét)
MAX_LINEAR_SPEED   = 0.20  # Vận tốc tiến tối đa (m/s) - Giảm xuống 0.20 m/s để phanh kịp thời
MAX_ANGULAR_SPEED  = 1.60  # Vận tốc quay tối đa (rad/s) ~ 91.7 deg/s - Đồng bộ 1:1 với mô-men động cơ JetBot

# ─── THÔNG SỐ CẢNH TAY ĐÒN & CẢM BIẾN MAPPING (SLAM PERCEPTION STANDARDS) ──────
CAMERA_OFFSET_FORWARD_M  = 0.09  # Cự ly tâm cụm OAK-D S2 đặt nhô phía trước trục bánh xe vi sai (9cm)
CAMERA_MAX_MAP_DIST_M    = 2.80  # Cự ly tin cậy tối đa cho Stereo Depth 2D (tránh nhiễu tán xạ > 2.8m)
MAPPING_DEFAULT_SPEED_V  = 0.15  # Tốc độ tiến chuẩn khi lập bản đồ (15% PWM) - Triệt tiêu nhòe camera
MAPPING_DEFAULT_SPEED_W  = 0.85  # Tốc độ quay chuẩn khi lập bản đồ (~48 deg/s) - Cho điểm quét sắc nét

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
STEP_FORWARD_SCALE       = 0.40  # Hệ số hiệu chuẩn TIẾN (dùng chung cho cả Bước cm và Lái tay WASD)
STEP_BACKWARD_SCALE      = 0.40  # Hệ số hiệu chuẩn LÙI (dùng chung cho cả Bước cm và Lái tay WASD)
STEP_TURN_LEFT_SCALE     = 0.62  # Hệ số hiệu chuẩn QUAY TRÁI (CCW)
STEP_TURN_RIGHT_SCALE    = 0.64  # Hệ số hiệu chuẩn QUAY PHẢI (CW) - Tách riêng để bù lệch cơ khí 2 bên

# ─── BÙ MÔ-MEN XOAY TẠI CHỖ (IN-PLACE TURN TORQUE COMPENSATION) ────────────────
STEP_TURN_MIN_PWM        = 0.11  # Mức xung PWM tối thiểu khi quay tại chỗ (11% đủ thắng ma sát trượt ngang)
STEP_TURN_SPEED_RATIO    = 1.0   # Tỉ lệ điều chỉnh tốc độ quay so với tốc độ tiến

# ─── GIẢM TỐC TỪ TỪ ĐỒNG NHẤT 100% (UNIFIED SMOOTH DECELERATION & SOFT STOP) ───
# Cả 2 chế độ (Bước định lượng và Lái tay WASD) đều dùng CHUNG cơ chế hạ dốc tuyến tính:
#   - Khi sắp đến đích hoặc khi nhả phím, xe hạ dốc mượt mà về 0 trong đúng thời gian MANUAL_DECEL_TIME_MS.
#   - Thời gian hạ dốc được tự động trừ hao chuẩn xác trong công thức động học vi sai,
#     giúp góc quay (180°) và quãng đường (cm) luôn đạt độ chính xác tuyệt đối trên cả xe thật và bản đồ!
ENABLE_SOFT_STOP         = True  # Cờ tổng thể: True = Dừng êm từ từ | False = Cắt điện tức thì
MANUAL_SOFT_DECEL        = True  # Giữ tương thích: Cho phép hạ dốc mượt mà khi dừng xe
MANUAL_DECEL_TIME_MS     = 180   # Thời gian hạ dốc mượt mà (180ms) dùng chung cho cả Lái tay và Bước cm/độ
STEP_USE_ACTIVE_BRAKE    = False # False: Ngắt êm tự nhiên (không giật ngược). True: Dùng xung phanh ngược cứng

# ─── THÔNG SỐ TỰ HÀNH BÁM ĐƯỜNG (AUTONOMOUS NAVIGATOR & PATH FOLLOWER) ───────────
ENABLE_AUTONOMOUS_NAVIGATOR = True  # Bật bộ điều khiển bám đường tự hành A*
NAV_LINEAR_SPEED            = 0.12  # Vận tốc tiến tự hành (m/s) ~15-20% PWM đầm xe
NAV_TURN_SPEED              = 1.20  # Vận tốc quay tại chỗ (rad/s) ~11-13% PWM êm ái
NAV_GOAL_TOLERANCE_M        = 0.06  # Bán kính chạm đích dừng xe (6cm)
NAV_WAYPOINT_TOLERANCE_M    = 0.10  # Bán kính chuyển waypoint tiếp theo (10cm)
NAV_HEADING_TOLERANCE_RAD   = 0.35  # Dung sai góc lệch (~20 độ) trước khi chuyển từ quay sang tiến
NAV_KP_ANGULAR              = 1.6   # Hệ số P chỉnh lái bám đường

