#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
KỊCH BẢN CHẠY TRỰC TIẾP AI NÉ VẬT CẢN (NVIDIA JETBOT LIVE COLLISION AVOIDANCE)
══════════════════════════════════════════════════════════════════════════════
Cách chạy:
  python scripts/run_collision_avoidance.py --speed 0.12 --turn 1.10
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import argparse
import time
import cv2

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from modules.collision_manager import CollisionManager
try:
    from modules.motor_controller import MotorController
except ImportError:
    MotorController = None


def main():
    parser = argparse.ArgumentParser(description="Live Collision Avoidance Runner")
    parser.add_argument("--speed", type=float, default=0.11, help="Vận tốc tiến cơ bản m/s (default: 0.11)")
    parser.add_argument("--turn", type=float, default=0.45, help="Vận tốc quay góc khi gặp cản rad/s (default: 0.45)")
    parser.add_argument("--camera", type=int, default=0, help="ID camera /dev/video* (default: 0)")
    parser.add_argument("--stream_url", type=str, default="", help="MJPEG stream URL nếu lấy từ main.py (ví dụ: http://127.0.0.1:8080/stream.mjpg)")
    args = parser.parse_args()

    print("\n" + "═" * 70)
    print("🤖 JETBOT AI LIVE COLLISION AVOIDANCE AUTOPILOT")
    print("═" * 70)

    mgr = CollisionManager(workspace_dir=ROOT_DIR)

    if not mgr.model:
        print(f"❌ Không tìm thấy model đã huấn luyện tại: {mgr.model_path}")
        print("💡 Hãy chạy 'python scripts/train_collision.py' để train model trước!")
        return

    # Kết nối Động cơ
    motors = None
    if MotorController is not None:
        try:
            motors = MotorController()
            print("🚗 Đã kết nối Motor Controller phần cứng JetBot thành công.")
        except Exception as e:
            print(f"⚠️ Không thể mở động cơ trực tiếp (có thể đang được main.py chiếm): {e}")

    # Mở Camera
    cap = None
    if args.stream_url:
        print(f"📡 Đang kết nối luồng camera stream: {args.stream_url}")
        cap = cv2.VideoCapture(args.stream_url)
    else:
        # Thử kết nối MJPEG localhost trước nếu main.py đang chạy
        cap = cv2.VideoCapture("http://127.0.0.1:8080/stream.mjpg")
        if not cap.isOpened():
            print(f"📷 Mở Camera phần cứng thiết bị ID: {args.camera}")
            cap = cv2.VideoCapture(args.camera)

    if not cap or not cap.isOpened():
        print("❌ Không thể mở luồng camera! Hãy kiểm tra kết nối camera hoặc chạy main.py.")
        return

    mgr.is_avoiding = True
    print("\n⚡ KÍCH HOẠT CHẾ ĐỘ NÉ VẬT CẢN TỰ HÀNH! (Bấm Ctrl+C hoặc phím 'q' để dừng)\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.02)
                continue

            # Dự đoán xác suất
            p_free, p_blocked = mgr.predict(frame)
            v, w, desc = mgr.compute_avoidance_cmd(p_blocked, base_speed=args.speed, turn_speed=args.turn)

            # Gửi lệnh điều khiển xuống động cơ
            if motors and v is not None and w is not None:
                motors.set_cmd_vel(v, w)

            # In trạng thái thời gian thực
            bar_len = 20
            b_cnt = int(p_blocked * bar_len)
            f_cnt = bar_len - b_cnt
            bar_str = "[" + "█" * b_cnt + "░" * f_cnt + "]"
            sys.stdout.write(f"\rAI: {bar_str} Blocked: {p_blocked*100:4.1f}% | Free: {p_free*100:4.1f}% | Hành động: {desc}   ")
            sys.stdout.flush()

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n🛑 Người dùng yêu cầu dừng tự hành.")
    finally:
        if motors:
            motors.stop()
        if cap:
            cap.release()
        print("🏁 Đã dừng hoàn toàn xe an toàn.")


if __name__ == "__main__":
    main()
