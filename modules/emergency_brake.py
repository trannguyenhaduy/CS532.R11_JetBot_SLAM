#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 6: Hệ thống Phanh khẩn cấp & Cản ảo 3D (Emergency Brake & Virtual Bumper)
Chuyên trách đo đạc cự ly vật cản từ Camera Stereo Depth, chống va chạm thời gian thực.

Tính năng:
- Lọc nhiễu phân vị 5% (5th Percentile) bắt chính xác mép cản gần nhất.
- Chống báo động giả (False Alarm Filter): Không bị khóa phanh oan trên sàn gạch nhẵn.
- Khóa chiều tiến (Lock Forward), luôn cho phép lùi (Reverse) và quay đầu (Turn) để thoát hiểm.
- Độc lập 100%, có self-test riêng: python3 -m modules.emergency_brake
"""

import sys
import time
import numpy as np

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass


class EmergencyBrake:
    """Bộ giám sát an toàn và Phanh khẩn cấp Virtual Bumper cho JetBot"""

    def __init__(self, brake_dist_m=0.18, warning_dist_m=0.40, min_pts_threshold=35):
        """
        :param brake_dist_m: Ngưỡng cự ly phanh cứng khẩn cấp (mặc định 18cm = 180mm)
        :param warning_dist_m: Ngưỡng cảnh báo giảm tốc (mặc định 40cm)
        :param min_pts_threshold: Số điểm ảnh cản tối thiểu để xác nhận (chống nhiễu hạt)
        """
        self.brake_dist_m = float(brake_dist_m)
        self.warning_dist_m = float(warning_dist_m)
        self.min_pts_threshold = int(min_pts_threshold)

        self.last_clearance_m = 99.0
        self.last_alert_level = "SAFE"
        self.is_emergency_active = False

    def calculate_clearance(self, depth_frame):
        """
        Trích xuất cự ly vật cản gần nhất ở vùng trung tâm phía trước mũi xe.
        :param depth_frame: Ma trận ảnh độ sâu numpy (uint16 mm hoặc float mét)
        :return: cự ly vật cản tính bằng mét (float)
        """
        if depth_frame is None or not isinstance(depth_frame, np.ndarray):
            self.last_clearance_m = 99.0
            return 99.0

        h, w = depth_frame.shape[:2]
        if h < 20 or w < 20:
            self.last_clearance_m = 99.0
            return 99.0

        # Vùng kiểm soát trung tâm (Middle ROI: 1/3 chiều cao, 1/3 chiều rộng)
        h_start, h_end = int(h * 0.33), int(h * 0.67)
        w_start, w_end = int(w * 0.30), int(w * 0.70)
        roi = depth_frame[h_start:h_end, w_start:w_end]

        # Chuẩn hóa đơn vị về mét
        if roi.dtype == np.uint16 or roi.max() > 100.0:
            roi_m = roi.astype(np.float32) / 1000.0
        else:
            roi_m = roi.astype(np.float32)

        # Lọc các điểm đo vật lý hợp lệ trong cự ly từ 8cm đến 4.0m
        valid = roi_m[(roi_m >= 0.08) & (roi_m <= 4.0)]

        # Phải có đủ số lượng điểm tối thiểu để tránh nhiễu do điểm ảnh chết hoặc sàn trơn
        if len(valid) >= self.min_pts_threshold:
            # Lấy phân vị 5% (bắt mép trước của vật cản nhạy hơn min tuyệt đối)
            clearance = float(np.percentile(valid, 5))
            self.last_clearance_m = round(clearance, 2)
        else:
            # Đường thoáng (hoặc sàn nhà phẳng không có vật cản)
            self.last_clearance_m = 99.0

        return self.last_clearance_m

    def evaluate_velocity(self, target_v: float, target_w: float, clearance_m: float = None):
        """
        Đánh giá và can thiệp lệnh vận tốc để bảo vệ phần cứng.
        :param target_v: Vận tốc dài yêu cầu (m/s)
        :param target_w: Vận tốc góc yêu cầu (rad/s)
        :param clearance_m: Cự ly cản đo được (nếu None sẽ lấy cự ly gần nhất)
        :return: (safe_v, safe_w, alert_level)
        """
        if clearance_m is None:
            clearance_m = self.last_clearance_m

        # 1. TRƯỜNG HỢP NGUY CẤP: Vật cản áp sát dưới ngưỡng phanh
        if clearance_m < self.brake_dist_m:
            if target_v > 0.0:
                # KHÓA CHẶT LỆNH TIẾN — PHANH KHẨN CẤP
                self.is_emergency_active = True
                self.last_alert_level = "EMERGENCY_STOP"
                # Vẫn giữ target_w để cho phép robot xoay tìm hướng thoát!
                return 0.0, target_w, "EMERGENCY_STOP"
            else:
                # Cho phép lùi (v < 0) hoặc quay đầu (w != 0) để lùi xa vật cản
                self.is_emergency_active = False
                self.last_alert_level = "ESCAPE_MANEUVER"
                return target_v, target_w, "ESCAPE_MANEUVER"

        # 2. TRƯỜNG HỢP CẢNH BÁO: Vật cản đang tới gần (< warning_dist)
        elif clearance_m < self.warning_dist_m:
            self.is_emergency_active = False
            self.last_alert_level = "WARNING"
            # Giảm tốc độ tiến còn tối đa 0.15 m/s để tiếp cận êm ái
            capped_v = min(0.15, target_v) if target_v > 0.0 else target_v
            return capped_v, target_w, "WARNING"

        # 3. TRƯỜNG HỢP AN TOÀN TUYỆT ĐỐI
        else:
            self.is_emergency_active = False
            self.last_alert_level = "SAFE"
            return target_v, target_w, "SAFE"


if __name__ == '__main__':
    print("\n" + "═" * 70)
    print("🧪 [SELF-TEST] BẮT ĐẦU KIỂM THỬ ĐỘC LẬP MODULE PHANH KHẨN CẤP (EMERGENCY BRAKE)")
    print("═" * 70)

    brake = EmergencyBrake(brake_dist_m=0.18, warning_dist_m=0.40)
    print(f"  ├─ Ngưỡng phanh khẩn cấp: {brake.brake_dist_m}m (18cm)")
    print(f"  ├─ Ngưỡng cảnh báo giảm tốc: {brake.warning_dist_m}m (40cm)")

    # KỊCH BẢN 1: Đường thoáng (không có cản)
    fake_empty_depth = np.ones((360, 480), dtype=np.uint16) * 1500 # 1.5m
    c1 = brake.calculate_clearance(fake_empty_depth)
    v1, w1, alert1 = brake.evaluate_velocity(0.25, 0.0, c1)
    print(f"  ├─ Test 1 (Đường 1.5m): Cự ly={c1}m -> Lệnh v={v1}m/s [{alert1}]")
    assert alert1 == "SAFE" and v1 == 0.25, "Lỗi: Kịch bản 1 không an toàn!"

    # KỊCH BẢN 2: Có vật cản nguy hiểm ở cự ly 14cm (0.14m)
    fake_danger_depth = np.ones((360, 480), dtype=np.uint16) * 140 # 140mm
    c2 = brake.calculate_clearance(fake_danger_depth)
    v2, w2, alert2 = brake.evaluate_velocity(0.20, 0.5, c2)
    print(f"  ├─ Test 2 (Vật cản 14cm khi đang bấm TIẾN): Cự ly={c2}m -> Lệnh v={v2}m/s [{alert2}]")
    assert alert2 == "EMERGENCY_STOP" and v2 == 0.0, "Lỗi: Không phanh khẩn cấp khi gặp cản!"

    # KỊCH BẢN 3: Đang kẹt sát cản (14cm) nhưng bấm LÙI hoặc QUAY để thoát
    v3, w3, alert3 = brake.evaluate_velocity(-0.20, 0.6, c2)
    print(f"  ├─ Test 3 (Thoát hiểm: Bấm LÙI v=-0.2 khi sát cản): Lệnh v={v3}m/s [{alert3}]")
    assert v3 == -0.20, "Lỗi: Bị chặn lệnh lùi thoát hiểm!"

    # KỊCH BẢN 4: Sàn gạch phẳng không có texture (nhiều điểm rỗng)
    fake_floor_zeros = np.zeros((360, 480), dtype=np.uint16)
    c4 = brake.calculate_clearance(fake_floor_zeros)
    v4, w4, alert4 = brake.evaluate_velocity(0.20, 0.0, c4)
    print(f"  ├─ Test 4 (Sàn nhẵn ít vân - Điểm 0): Cự ly={c4}m -> Lệnh v={v4}m/s [{alert4}]")
    assert alert4 == "SAFE" and v4 == 0.20, "Lỗi: Bị báo động giả trên sàn gạch!"

    print("═" * 70)
    print("✅ [SELF-TEST] TẤT CẢ 4 KỊCH BẢN PHANH KHẨN CẤP ĐỀU ĐẠT CHUẨN 100%!")
    print("═" * 70 + "\n")
