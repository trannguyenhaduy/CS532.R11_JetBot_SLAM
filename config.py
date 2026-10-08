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
ENABLE_YOLO         = True   # Bật/Tắt module lọc nhận diện 3D Spatial AI
ENABLE_FOLLOWER     = False  # Bật/Tắt module tự hành bám người (MẶC ĐỊNH TẮT ĐỂ ƯU TIÊN LÁI TAY)
ENABLE_MAPPER       = True   # Bật/Tắt module bản đồ ngữ nghĩa 3D
ENABLE_WEB          = True   # Bật/Tắt trạm điều khiển Web Cockpit (Port 8080)
ENABLE_SAFETY_BRAKE = True   # Bật tính năng phanh khẩn cấp Virtual Bumper (< 18cm)

# ─── THÔNG SỐ TRẠM ĐIỀU KHIỂN WEB ─────────────────────────────────────────────
WEB_PORT = 8080
WEB_HOST = '0.0.0.0'

# ─── THÔNG SỐ PHẦN CỨNG XE (JETBOT BASELINE) ──────────────────────────────────
I2C_BUS = 1
PCA9685_ADDR = 0x60
INA219_ADDR  = 0x41

WHEEL_SEPARATION_M = 0.12  # Khoảng cách 2 bánh vi sai (mét)
MAX_LINEAR_SPEED   = 0.20  # Vận tốc tiến tối đa (m/s) - Giảm xuống 0.20 m/s để phanh kịp thời
MAX_ANGULAR_SPEED  = 0.60  # Vận tốc quay tối đa (rad/s) - Giảm xuống 0.60 rad/s để xe quay êm ái

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
DEPTH_SKIP_FRAMES = 5       # Chỉ tính mây điểm 3D 1 trong 5 frame (~3 Hz)
DEPTH_DOWNSAMPLE_STEP = 25  # Bước nhảy lấy mẫu ma trận điểm ảnh (pixel)
IMAGE_SKIP_FRAMES = 2       # Bỏ qua 1/2 frame video để nhẹ CPU encode JPEG

