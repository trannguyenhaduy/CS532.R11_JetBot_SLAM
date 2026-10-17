#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
YAHBOOM TRANSBOT VISUAL COURSE SUITE (ADAPTED FOR JETBOT & OAK-D CAMERA)
Tham khảo & Tích hợp trọn bộ Chương 9 (ROS Depth Camera Course - Yahboom):
  - Bài 9.2: Astra Color Tracking (Bám theo màu sắc HSV)
  - Bài 9.3: Astra Object Tracking (Bám theo vật thể bất kỳ bằng Tracker)
  - Bài 9.4: Visual Tracking Autopilot (Tự hành bám mục tiêu bằng đo sâu 3D)
══════════════════════════════════════════════════════════════════════════════
Hỗ trợ 2 chế độ chạy:
  1. Chạy trực tiếp trên Robot JetBot: python3 scripts/yahboom_visual_course.py
  2. Chạy từ xa trên Laptop (Windows/Ubuntu) qua WiFi:
     python scripts/yahboom_visual_course.py --ip 192.168.1.11 --mode color
══════════════════════════════════════════════════════════════════════════════
"""

import sys
import os
import time
import argparse
import urllib.request
import numpy as np
import cv2

# Dải màu HSV chuẩn cho bài 9.2 Color Tracking
COLOR_RANGES = {
    'red': [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255]))
    ],
    'green': [
        (np.array([35, 80, 60]), np.array([85, 255, 255]))
    ],
    'blue': [
        (np.array([100, 100, 60]), np.array([130, 255, 255]))
    ],
    'yellow': [
        (np.array([20, 100, 100]), np.array([34, 255, 255]))
    ]
}


class YahboomVisualSuite:
    def __init__(self, mode='color', color_name='red', robot_ip=None, port=8080):
        self.mode = mode.lower()
        self.color_name = color_name.lower()
        self.robot_ip = robot_ip
        self.port = int(port)
        self.is_remote = (self.robot_ip is not None and len(self.robot_ip.strip()) > 0)
        self.last_cmd_time = 0.0

        # Tham số bộ điều khiển PID bám mục tiêu (Chuẩn Yahboom Transbot)
        self.kp_ang = 0.85
        self.kd_ang = 0.12
        self.kp_dist = 0.35
        self.last_ex = 0.0

        # Tracker cho bài 9.3 (Object Tracking)
        self.tracker = None
        self.tracking_box = None
        self.is_tracking = False

        # Khởi tạo luồng Camera
        if self.is_remote:
            self.base_url = f"http://{self.robot_ip.strip()}:{self.port}"
            self.stream_url = f"{self.base_url}/stream.mjpg"
            print(f"📡 Chế độ: Điều khiển từ xa qua WiFi -> JetBot IP: {self.robot_ip}")
            self.cap = cv2.VideoCapture(self.stream_url)
        else:
            print("🤖 Chế độ: Chạy trực tiếp trên JetBot (Local Hardware)")
            self.base_url = None
            self.stream_url = None
            self.cap = cv2.VideoCapture(0)

        # Kết nối động cơ nếu chạy local
        self.motors = None
        if not self.is_remote:
            try:
                from modules.motor_controller import MotorController
                self.motors = MotorController()
                print("✅ Đã kết nối Driver động cơ JetBot thành công.")
            except Exception as e:
                print(f"ℹ️ Không nạp được driver động cơ local: {e}")

    def send_drive(self, v: float, w: float):
        """Phát lệnh vận tốc tới JetBot (Hỗ trợ cả Local và WiFi API)"""
        now = time.time()
        # Giới hạn tần số gửi lệnh tối đa 20Hz (mỗi 50ms)
        if now - self.last_cmd_time < 0.045:
            return
        self.last_cmd_time = now

        if self.is_remote:
            url = f"{self.base_url}/api/drive?v={v:.2f}&w={w:.2f}"
            try:
                req = urllib.request.Request(url, headers={'User-Agent': 'YahboomVisualSuite'})
                with urllib.request.urlopen(req, timeout=0.15) as resp:
                    pass
            except Exception:
                pass
        else:
            if self.motors:
                self.motors.set_cmd_vel(v, w)

    def process_color_tracking(self, frame):
        """Bài 9.2: Yahboom Astra Color Tracking"""
        h, w = frame.shape[:2]
        center_x = w // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Lấy dải màu
        ranges = COLOR_RANGES.get(self.color_name, COLOR_RANGES['red'])
        mask = None
        for (lower, upper) in ranges:
            m = cv2.inRange(hsv, lower, upper)
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        # Lọc nhiễu hình thái học (Morphology)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_found = False
        action_text = "TÌM MỤC TIÊU..."
        action_color = (0, 200, 255)
        v_cmd, w_cmd = 0.0, 0.0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 450:  # Ngưỡng diện tích tối thiểu
                ((cx, cy), radius) = cv2.minEnclosingCircle(largest)
                cx, cy, radius = int(cx), int(cy), int(radius)

                # Vẽ vòng tròn bao quanh mục tiêu màu
                cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                # Tính độ lệch góc so với tâm khung hình
                ex = (cx - center_x) / float(center_x)  # Chuẩn hóa về [-1.0, 1.0]
                dex = ex - self.last_ex
                self.last_ex = ex

                # Bộ điều khiển PD góc quay
                w_cmd = -self.kp_ang * ex - self.kd_ang * dex
                w_cmd = max(-1.0, min(1.0, w_cmd))

                # Bộ điều khiển tiến lùi theo bán kính (Target radius ~ 45 pixel)
                target_r = 45
                r_diff = target_r - radius
                if abs(r_diff) > 8 and abs(ex) < 0.35:
                    v_cmd = np.clip(r_diff * 0.005, -0.12, 0.15)
                else:
                    v_cmd = 0.0

                target_found = True
                action_text = f"ĐANG BÁM MÀU [{self.color_name.upper()}]: R={radius}px | Offset={ex:.2f}"
                action_color = (0, 255, 100)

        if not target_found:
            v_cmd, w_cmd = 0.0, 0.0

        self.send_drive(v_cmd, w_cmd)
        return frame, action_text, action_color

    def init_object_tracker(self, frame, bbox):
        """Khởi tạo Tracker cho Bài 9.3"""
        self.tracking_box = bbox
        try:
            if hasattr(cv2, 'TrackerMIL_create'):
                self.tracker = cv2.TrackerMIL_create()
            elif hasattr(cv2, 'TrackerKCF_create'):
                self.tracker = cv2.TrackerKCF_create()
            elif hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
                self.tracker = cv2.legacy.TrackerKCF_create()
            else:
                self.tracker = cv2.TrackerMIL_create()

            self.tracker.init(frame, bbox)
            self.is_tracking = True
            print(f"🎯 Đã khóa mục tiêu theo dõi: {bbox}")
        except Exception as e:
            print(f"⚠️ Lỗi khởi tạo Tracker: {e}")
            self.is_tracking = False

    def process_object_tracking(self, frame):
        """Bài 9.3: Yahboom Astra Object Tracking (KCF/MIL Tracker)"""
        h, w = frame.shape[:2]
        center_x = w // 2
        action_text = "HÃY BẤM [S] HOẶC DÙNG CHUỘT KÉO HỘP ĐỂ CHỌN MỤC TIÊU"
        action_color = (0, 240, 255)
        v_cmd, w_cmd = 0.0, 0.0

        if self.is_tracking and self.tracker is not None:
            ok, bbox = self.tracker.update(frame)
            if ok:
                bx, by, bw, bh = [int(v) for v in bbox]
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 255, 255), 2)
                cx = bx + bw // 2
                cy = by + bh // 2
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                ex = (cx - center_x) / float(center_x)
                w_cmd = -self.kp_ang * ex
                w_cmd = max(-0.9, min(0.9, w_cmd))

                action_text = f"ĐANG BÁM ĐỐI TƯỢNG (KCF): Tâm=({cx},{cy}) | w={w_cmd:.2f}"
                action_color = (0, 255, 120)
            else:
                action_text = "⚠️ MẤT DẤU ĐỐI TƯỢNG! Hãy bấm [S] để chọn lại"
                action_color = (0, 0, 255)
                v_cmd, w_cmd = 0.0, 0.0

        self.send_drive(v_cmd, w_cmd)
        return frame, action_text, action_color

    def process_visual_autopilot(self, frame):
        """Bài 9.4: Yahboom Visual Tracking Autopilot (Tự hành bám khoảng cách)"""
        h, w = frame.shape[:2]
        center_x = w // 2
        # Giả lập phát hiện người / vật cản phía trước bằng contour hoặc face
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4, minSize=(30, 30))

        v_cmd, w_cmd = 0.0, 0.0
        action_text = "AUTOPILOT: ĐANG TÌM MỤC TIÊU ĐỂ BÁM..."
        action_color = (0, 200, 255)

        if len(faces) > 0:
            (fx, fy, fw, fh) = max(faces, key=lambda f: f[2] * f[3])
            cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
            fcx = fx + fw // 2
            fcy = fy + fh // 2
            cv2.circle(frame, (fcx, fcy), 4, (0, 0, 255), -1)

            # Ước lượng cự ly theo kích thước khuôn mặt (Khuôn mặt ~ 75px ở 0.85m)
            est_dist_m = 75.0 / max(1.0, float(fw)) * 0.85

            ex = (fcx - center_x) / float(center_x)
            w_cmd = -self.kp_ang * ex
            w_cmd = max(-0.9, min(0.9, w_cmd))

            target_dist = 0.85
            ez = est_dist_m - target_dist
            if abs(ez) > 0.10:
                v_cmd = np.clip(self.kp_dist * ez, -0.12, 0.16)
            else:
                v_cmd = 0.0

            # Phanh an toàn nếu quá gần (< 0.40m)
            if est_dist_m < 0.40:
                v_cmd = 0.0
                action_text = f"🛑 CẢN QUÁ GẦN ({est_dist_m:.2f}m < 0.40m) -> PHANH DỪNG!"
                action_color = (0, 0, 255)
            else:
                action_text = f"🎯 AUTOPILOT BÁM MỤC TIÊU: Cự ly={est_dist_m:.2f}m | v={v_cmd:.2f}m/s, w={w_cmd:.2f}rad/s"
                action_color = (0, 255, 100)

        self.send_drive(v_cmd, w_cmd)
        return frame, action_text, action_color

    def run(self):
        """Vòng lặp chính của chương trình thị giác Yahboom"""
        print("═" * 70)
        print("🚀 KHỞI ĐỘNG BỘ THỊ GIÁC YAHBOOM TRANSBOT CHO JETBOT")
        print(f"📌 Chế độ hoạt động: {self.mode.upper()}")
        print("🎮 Phím tắt: [Q]: Thoát | [SPACE]: Dừng khẩn | [S]: Chọn vùng bám (Chế độ Object)")
        print("═" * 70)

        win_name = f"Yahboom Visual Course [{self.mode.upper()}]"
        cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)

        fps_count = 0
        fps_start = time.time()
        fps_disp = 0.0

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    print("⚠️ Mất tín hiệu camera... Đang kết nối lại...")
                    time.sleep(0.5)
                    if self.is_remote:
                        self.cap = cv2.VideoCapture(self.stream_url)
                    continue

                fps_count += 1
                if time.time() - fps_start >= 1.0:
                    fps_disp = fps_count / (time.time() - fps_start)
                    fps_count = 0
                    fps_start = time.time()

                h, w = frame.shape[:2]

                # Xử lý theo chế độ được chọn
                if self.mode == 'color':
                    frame, act_text, act_col = self.process_color_tracking(frame)
                elif self.mode == 'object':
                    frame, act_text, act_col = self.process_object_tracking(frame)
                elif self.mode == 'autopilot':
                    frame, act_text, act_col = self.process_visual_autopilot(frame)
                else:
                    act_text, act_col = "CHẾ ĐỘ KHÔNG HỢP LỆ", (0, 0, 255)

                # Vẽ OSD HUD thanh trạng thái
                cv2.rectangle(frame, (0, 0), (w, 32), (10, 14, 23), -1)
                cv2.putText(frame, f"YAHBOOM TRANSBOT ROS COURSE | MODE: {self.mode.upper()} | FPS: {fps_disp:.1f}",
                            (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 240, 255), 2)

                cv2.rectangle(frame, (0, h - 35), (w, h), (10, 14, 23), -1)
                cv2.putText(frame, act_text, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.48, act_col, 2)

                cv2.imshow(win_name, frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q') or key == 27:
                    break
                elif key == 32:  # Phím SPACE
                    self.send_drive(0.0, 0.0)
                    print("■ PHANH DỪNG KHẨN CẤP!")
                elif key == ord('s') and self.mode == 'object':
                    # Cho phép người dùng kéo chuột chọn vùng bám
                    bbox = cv2.selectROI(win_name, frame, fromCenter=False, showCrosshair=True)
                    if bbox[2] > 10 and bbox[3] > 10:
                        self.init_object_tracker(frame, bbox)

        except KeyboardInterrupt:
            print("\n🛑 Ngắt bởi bàn phím.")
        finally:
            self.send_drive(0.0, 0.0)
            self.cap.release()
            cv2.destroyAllWindows()
            print("🏁 Đã dừng an toàn chương trình thị giác.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Yahboom Transbot Visual Course Runner for JetBot")
    parser.add_argument('--mode', type=str, default='color', choices=['color', 'object', 'autopilot'],
                        help='Chế độ: color (9.2), object (9.3), autopilot (9.4)')
    parser.add_argument('--color', type=str, default='red', choices=['red', 'green', 'blue', 'yellow'],
                        help='Màu cần bám (dành cho chế độ color)')
    parser.add_argument('--ip', type=str, default=None,
                        help='Địa chỉ IP của JetBot nếu chạy remote qua WiFi (bỏ trống nếu chạy local)')
    parser.add_argument('--port', type=int, default=8080, help='Cổng Web JetBot')
    args = parser.parse_args()

    suite = YahboomVisualSuite(mode=args.mode, color_name=args.color, robot_ip=args.ip, port=args.port)
    suite.run()
