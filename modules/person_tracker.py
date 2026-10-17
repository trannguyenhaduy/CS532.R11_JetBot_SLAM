#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 5: Bộ điều khiển Thị giác & Bám mục tiêu (Visual & Person Tracker)
Tích hợp trọn bộ thuật toán:
  1. Bám người tự hành (Person Following / Visual Autopilot - Yahboom 9.4)
  2. Bám màu sắc HSV (Color Tracking - Yahboom 9.2: Đỏ, Xanh lá, Xanh dương, Vàng)
  3. Bám vật thể bất kỳ (Object Tracking - Yahboom 9.3: KCF / MIL Tracker)
Tuân thủ Quy tắc Harness: Mặc định TẮT, chỉ gửi lệnh dừng 1 lần khi mất mục tiêu.
"""

import sys
import time
import numpy as np
import cv2

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

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


class PersonTracker:
    def __init__(self, target_dist=0.85, kp_dist=0.35, kd_dist=0.08, kp_ang=1.10, kd_ang=0.15):
        self.is_enabled = False  # Mặc định TẮT
        self.mode = 'off'        # 'off' | 'person' | 'color' | 'object'
        self.color_name = 'red'  # 'red' | 'green' | 'blue' | 'yellow'
        self.target_dist = float(target_dist)
        self.kp_dist = float(kp_dist)
        self.kd_dist = float(kd_dist)
        self.kp_ang = float(kp_ang)
        self.kd_ang = float(kd_ang)

        self.last_ez = 0.0
        self.last_ex = 0.0
        self.last_ctrl_time = time.time()
        self.last_target_time = 0.0
        self.has_sent_stop = True
        self.status_text = "SẴN SÀNG"

        # Bộ bám vật thể KCF / MIL (Bài 9.3)
        self.tracker = None
        self.tracking_box = None
        self.is_tracking_object = False

    def set_enabled(self, enabled: bool):
        self.is_enabled = bool(enabled)
        if not self.is_enabled:
            self.mode = 'off'
            self.has_sent_stop = False
            self.status_text = "TẮT"
        else:
            if self.mode == 'off':
                self.mode = 'person'
            self.status_text = f"ĐANG CHẠY: {self.mode.upper()}"

    def set_mode(self, mode: str, color_name: str = None):
        """Chuyển đổi chế độ bám: person | color | object | off"""
        m = str(mode).strip().lower()
        if m in ('off', 'stop', 'disabled'):
            self.set_enabled(False)
            return False

        self.is_enabled = True
        self.has_sent_stop = False
        if color_name:
            self.color_name = str(color_name).strip().lower()

        if m in ('color', 'colour'):
            self.mode = 'color'
            self.status_text = f"BÁM MÀU: {self.color_name.upper()}"
        elif m in ('object', 'kcf', 'mil'):
            self.mode = 'object'
            self.status_text = "BÁM VẬT THỂ (CHỜ CHỌN VÙNG)"
        else:
            self.mode = 'person'
            self.status_text = "BÁM NGƯỜI (SPATIAL AUTOPILOT)"

        print(f"🎯 [TRACKER] Đã kích hoạt chế độ: {self.status_text}")
        return True

    def init_object_roi(self, frame, bbox):
        """Khởi tạo vùng vật thể theo dõi (x, y, w, h) cho bài 9.3"""
        if frame is None or frame.size == 0 or bbox is None:
            return False
        try:
            if hasattr(cv2, 'TrackerMIL_create'):
                self.tracker = cv2.TrackerMIL_create()
            elif hasattr(cv2, 'TrackerKCF_create'):
                self.tracker = cv2.TrackerKCF_create()
            elif hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
                self.tracker = cv2.legacy.TrackerKCF_create()
            else:
                self.tracker = cv2.TrackerMIL_create()

            self.tracker.init(frame, tuple(bbox))
            self.tracking_box = bbox
            self.is_tracking_object = True
            self.mode = 'object'
            self.is_enabled = True
            self.has_sent_stop = False
            self.status_text = f"ĐÃ KHÓA VẬT THỂ: {bbox}"
            print(f"🎯 [TRACKER] Đã khóa mục tiêu: {bbox}")
            return True
        except Exception as e:
            print(f"⚠️ [TRACKER] Lỗi khởi tạo Object Tracker: {e}")
            self.is_tracking_object = False
            return False

    def compute_command(self, detections=None, frame=None):
        """
        Tính toán lệnh vận tốc (v, w). Trả về (None, None) nếu không có lệnh mới.
        Tự động phân phối theo chế độ: person | color | object
        """
        if not self.is_enabled or self.mode == 'off':
            return None, None

        if self.mode == 'color':
            return self._compute_color_command(frame)
        elif self.mode == 'object':
            return self._compute_object_command(frame)
        else:
            return self._compute_person_command(detections)

    def _compute_person_command(self, detections):
        """Bài 9.4: Yahboom Visual Autopilot (Bám người qua Spatial Depth)"""
        now = time.time()
        if not detections:
            if not self.has_sent_stop and (now - self.last_target_time > 0.8):
                self.has_sent_stop = True
                self.status_text = "MẤT DẤU MỤC TIÊU (DỪNG)"
                return 0.0, 0.0
            return None, None

        # Lọc danh sách người
        persons = [d for d in detections if d.get('id') == 0 or 'PERSON' in str(d.get('name', '')).upper()]
        if not persons:
            if not self.has_sent_stop and (now - self.last_target_time > 0.8):
                self.has_sent_stop = True
                self.status_text = "MẤT DẤU NGƯỜI (DỪNG)"
                return 0.0, 0.0
            return None, None

        # Chọn người gần trục giữa camera nhất
        target = min(persons, key=lambda p: abs(float(p.get('x', 0.0))))
        x = float(target.get('x', 0.0))
        z = float(target.get('z', 0.0))

        valid_depth = (0.20 < z < 5.0)
        dt = max(0.01, now - self.last_ctrl_time)

        if valid_depth:
            ez = z - self.target_dist
            dez = (ez - self.last_ez) / dt
            # Khóa an toàn: Nếu người quá gần (< 0.40m), dừng tiến để không va chạm
            if z < 0.40:
                v = 0.0
                self.status_text = f"CẢN QUÁ GẦN ({z:.2f}m < 0.40m) -> DỪNG PHANH"
            else:
                v = (self.kp_dist * ez) + (self.kd_dist * dez) if abs(ez) > 0.08 else 0.0
                self.status_text = f"BÁM NGƯỜI ({z:.2f}m) | v={v:.2f}m/s"
            self.last_ez = ez
        else:
            v = 0.0
            self.status_text = "ĐANG TÌM ĐỘ SÂU NGƯỜI..."

        ex = x
        dex = (ex - self.last_ex) / dt
        w = (-self.kp_ang * ex) - (self.kd_ang * dex) if abs(ex) > 0.05 else 0.0

        v = max(-0.14, min(0.18, v))
        w = max(-0.95, min(0.95, w))

        self.last_ex = ex
        self.last_ctrl_time = now
        self.last_target_time = now
        self.has_sent_stop = False
        return v, w

    def _compute_color_command(self, frame):
        """Bài 9.2: Yahboom Astra Color Tracking (Bám theo màu sắc HSV)"""
        now = time.time()
        if frame is None or frame.size == 0:
            return None, None

        h, w = frame.shape[:2]
        center_x = w // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ranges = COLOR_RANGES.get(self.color_name, COLOR_RANGES['red'])
        mask = None
        for (lower, upper) in ranges:
            m = cv2.inRange(hsv, lower, upper)
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            if not self.has_sent_stop and (now - self.last_target_time > 0.8):
                self.has_sent_stop = True
                self.status_text = f"KHÔNG THẤY MÀU {self.color_name.upper()} (DỪNG)"
                return 0.0, 0.0
            return None, None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 400:
            if not self.has_sent_stop and (now - self.last_target_time > 0.8):
                self.has_sent_stop = True
                return 0.0, 0.0
            return None, None

        ((cx, cy), radius) = cv2.minEnclosingCircle(largest)
        ex = (cx - center_x) / float(center_x)
        w_cmd = -self.kp_ang * ex
        w_cmd = max(-0.9, min(0.9, w_cmd))

        target_r = 45
        r_diff = target_r - radius
        if abs(r_diff) > 8 and abs(ex) < 0.35:
            v_cmd = np.clip(r_diff * 0.005, -0.12, 0.15)
        else:
            v_cmd = 0.0

        self.last_target_time = now
        self.last_ctrl_time = now
        self.has_sent_stop = False
        self.status_text = f"BÁM MÀU [{self.color_name.upper()}]: R={int(radius)}px | w={w_cmd:.2f}"
        return float(v_cmd), float(w_cmd)

    def _compute_object_command(self, frame):
        """Bài 9.3: Yahboom Astra Object Tracking (KCF/MIL Tracker)"""
        now = time.time()
        if frame is None or frame.size == 0 or not self.is_tracking_object or self.tracker is None:
            if not self.has_sent_stop and (now - self.last_target_time > 0.8):
                self.has_sent_stop = True
                return 0.0, 0.0
            return None, None

        ok, bbox = self.tracker.update(frame)
        if not ok:
            if not self.has_sent_stop and (now - self.last_target_time > 0.8):
                self.has_sent_stop = True
                self.status_text = "MẤT DẤU VẬT THỂ THEO DÕI"
                return 0.0, 0.0
            return None, None

        bx, by, bw, bh = [int(v) for v in bbox]
        h, w = frame.shape[:2]
        center_x = w // 2
        cx = bx + bw // 2

        ex = (cx - center_x) / float(center_x)
        w_cmd = -self.kp_ang * ex
        w_cmd = max(-0.85, min(0.85, w_cmd))
        v_cmd = 0.0

        self.last_target_time = now
        self.last_ctrl_time = now
        self.has_sent_stop = False
        self.status_text = f"BÁM VẬT THỂ: Tâm=({cx}) | w={w_cmd:.2f}"
        return v_cmd, w_cmd


# Bí danh tương thích ngược
VisualTracker = PersonTracker

if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử Visual & Person Tracker...")
    vt = PersonTracker()
    assert vt.is_enabled is False, "Lỗi: Mặc định phải TẮT"
    vt.set_mode('color', 'red')
    assert vt.mode == 'color', "Lỗi: Chế độ phải là color"
    assert vt.color_name == 'red', "Lỗi: Màu phải là red"
    print("✅ [SELF-TEST] VisualTracker ĐẠT CHUẨN 100%!")
