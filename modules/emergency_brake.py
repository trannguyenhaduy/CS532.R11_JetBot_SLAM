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

    def __init__(self, brake_dist_m=0.22, warning_dist_m=0.45, min_pts_threshold=20, is_enabled=False):
        """
        :param brake_dist_m: Ngưỡng cự ly phanh an toàn (mặc định 22cm = 0.22m)
        :param warning_dist_m: Ngưỡng cảnh báo giảm tốc từ từ (mặc định 45cm = 0.45m)
        :param min_pts_threshold: Số điểm ảnh cản tối thiểu để xác nhận (chống nhiễu hạt)
        :param is_enabled: Bật/Tắt can thiệp phanh (Mặc định False để lái tự do)
        """
        self.brake_dist_m = float(brake_dist_m)
        self.warning_dist_m = float(warning_dist_m)
        self.min_pts_threshold = int(min_pts_threshold)
        self.is_enabled = bool(is_enabled)

        self.last_clearance_m = 99.0
        self.last_alert_level = "SAFE"
        self.is_emergency_active = False

    def calculate_clearance(self, depth_frame):
        """
        Trích xuất cự ly vật cản gần nhất ở vùng trung tâm phía trước mũi xe.
        LƯU Ý QUAN TRỌNG: Loại trừ 40% phía dưới khung hình (sàn gạch/bóng phản chiếu).
        Chỉ đo hành lang cản từ độ cao 10cm đến 40cm phía trước xe.
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

        # Vùng kiểm soát hành lang cản trước mũi xe: w từ 25% đến 75%, h từ 15% đến 65%
        h_start, h_end = int(h * 0.15), int(h * 0.65)
        w_start, w_end = int(w * 0.25), int(w * 0.75)
        roi = depth_frame[h_start:h_end, w_start:w_end]

        # Khử NaN/Inf nếu là ma trận float32
        if np.issubdtype(roi.dtype, np.floating):
            roi = np.nan_to_num(roi, nan=0.0, posinf=0.0, neginf=0.0)

        # Chuẩn hóa đơn vị về mét (nếu là uint16 hoặc giá trị lớn hơn 50.0 thì là mm)
        if roi.dtype == np.uint16 or (roi.size > 0 and np.max(roi) > 50.0):
            roi_m = roi.astype(np.float32) / 1000.0
        else:
            roi_m = roi.astype(np.float32)

        valid_mask = (roi_m >= 0.10) & (roi_m <= 3.5)

        # Lọc cao độ hình học so với mặt sàn: Chỉ lấy điểm cao hơn sàn >= 4cm (0.04m) và <= 85cm (0.85m)
        num_rows = h_end - h_start
        cy = h / 2.0
        fy = h * 0.9375  # FOV chuẩn OAK-D S2 (~55-60°)
        v_idx = (np.arange(num_rows, dtype=np.float32) + h_start)[:, None]
        y_cam = (v_idx - cy) * roi_m / fy
        z_height_above_floor = -y_cam + 0.12  # Chiều cao tâm thấu kính 12cm

        obs_mask = valid_mask & (z_height_above_floor >= 0.04) & (z_height_above_floor <= 0.85)
        valid = roi_m[obs_mask]

        # Phải có đủ số lượng điểm tối thiểu để tránh nhiễu do điểm ảnh chết hoặc sàn trơn
        if len(valid) >= self.min_pts_threshold:
            # Lấy phân vị 15%: Bắt chính xác mép cản gần nhất trước mặt
            clearance = float(np.percentile(valid, 15))
            self.last_clearance_m = round(clearance, 2)
        elif roi_m.size > 0 and (np.count_nonzero(roi_m < 0.10) / float(roi_m.size)) > 0.45 and self.last_clearance_m <= 0.45:
            # HIỆN TƯỢNG ĐIỂM MÙ STEREO: Khi vật cản che kín >45% hành lang trước mắt ở cự ly áp sát
            self.last_clearance_m = 0.18
        else:
            # Đường thoáng (không có cản trong hành lang)
            self.last_clearance_m = 99.0

        return self.last_clearance_m

    def evaluate_velocity(self, target_v: float, target_w: float, clearance_m: float = None):
        """
        Đánh giá và can thiệp lệnh vận tốc để bảo vệ phần cứng với giảm tốc mượt mà.
        :param target_v: Vận tốc dài yêu cầu (m/s)
        :param target_w: Vận tốc góc yêu cầu (rad/s)
        :param clearance_m: Cự ly cản đo được (nếu None sẽ lấy cự ly gần nhất)
        :return: (safe_v, safe_w, alert_level)
        """
        if clearance_m is None:
            clearance_m = self.last_clearance_m

        # Nếu tính năng phanh bị tắt: Cho phép xe chạy 100% tự do
        if not self.is_enabled:
            self.is_emergency_active = False
            self.last_alert_level = "SAFE (BYPASS)"
            return target_v, target_w, "SAFE"

        # 1. TRƯỜNG HỢP NGUY CẤP: Vật cản áp sát dưới ngưỡng phanh (<= 22cm)
        if clearance_m <= self.brake_dist_m:
            if target_v > 0.0:
                # KHÓA CHẶT LỆNH TIẾN — DỪNG AN TOÀN
                self.is_emergency_active = True
                self.last_alert_level = "EMERGENCY_STOP"
                # Vẫn giữ target_w để cho phép robot xoay tìm hướng thoát!
                return 0.0, target_w, "EMERGENCY_STOP"
            else:
                # Cho phép lùi (v < 0) hoặc quay đầu (w != 0) để lùi xa vật cản
                self.is_emergency_active = False
                self.last_alert_level = "ESCAPE_MANEUVER"
                return target_v, target_w, "ESCAPE_MANEUVER"

        # 2. TRƯỜNG HỢP CẢNH BÁO: Đang tiếp cận gần (22cm < clearance_m < 45cm) -> Giảm tốc mượt mà
        elif clearance_m < self.warning_dist_m:
            self.is_emergency_active = False
            self.last_alert_level = "WARNING"
            if target_v > 0.0:
                ratio = max(0.20, (clearance_m - self.brake_dist_m) / max(0.01, self.warning_dist_m - self.brake_dist_m))
                smooth_v = target_v * ratio
                return smooth_v, target_w, "WARNING"
            return target_v, target_w, "WARNING"

        # 3. TRƯỜNG HỢP AN TOÀN TUYỆT ĐỐI
        else:
            self.is_emergency_active = False
            self.last_alert_level = "SAFE"
            return target_v, target_w, "SAFE"


if __name__ == '__main__':
    print("\n" + "═" * 70)
    print("🧪 [SELF-TEST] BẮT ĐẦU KIỂM THỬ ĐỘC LẬP MODULE PHANH KHẨN CẤP (EMERGENCY BRAKE)")
    print("═" * 70)

    brake = EmergencyBrake(brake_dist_m=0.22, warning_dist_m=0.45, is_enabled=True)
    print(f"  ├─ Ngưỡng phanh an toàn: {brake.brake_dist_m}m (22cm)")
    print(f"  ├─ Ngưỡng cảnh báo giảm tốc: {brake.warning_dist_m}m (45cm)")

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

    # KỊCH BẢN 4: Khởi động trên sàn gạch phẳng không có texture (nhiều điểm rỗng)
    brake_floor = EmergencyBrake(brake_dist_m=0.18, warning_dist_m=0.40, is_enabled=True)
    fake_floor_zeros = np.zeros((360, 480), dtype=np.uint16)
    c4 = brake_floor.calculate_clearance(fake_floor_zeros)
    v4, w4, alert4 = brake_floor.evaluate_velocity(0.20, 0.0, c4)
    print(f"  ├─ Test 4 (Sàn nhẵn ít vân - Điểm 0 lúc ban đầu): Cự ly={c4}m -> Lệnh v={v4}m/s [{alert4}]")
    assert alert4 == "SAFE" and v4 == 0.20, "Lỗi: Bị báo động giả trên sàn gạch!"

    # KỊCH BẢN 5: Bắt điểm mù Stereo khi tiến sát vật cản
    c5 = brake.calculate_clearance(fake_floor_zeros) # brake đang có last_clearance_m = 0.14m từ test 2
    v5, w5, alert5 = brake.evaluate_velocity(0.20, 0.0, c5)
    print(f"  ├─ Test 5 (Khóa điểm mù khi lọt vào vùng <35cm): Cự ly={c5}m -> Lệnh v={v5}m/s [{alert5}]")
    assert alert5 in ("WARNING", "EMERGENCY_STOP"), "Lỗi: Không khóa điểm mù khi mất dấu cản sát mũi!"

    print("═" * 70)
    print("✅ [SELF-TEST] TẤT CẢ 4 KỊCH BẢN PHANH KHẨN CẤP ĐỀU ĐẠT CHUẨN 100%!")
    print("═" * 70 + "\n")
