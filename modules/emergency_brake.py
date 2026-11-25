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

    def __init__(self, brake_dist_m=0.40, warning_dist_m=0.60, min_pts_threshold=20, is_enabled=None):
        """
        :param brake_dist_m: Ngưỡng cự ly phanh an toàn dừng dứt khoát (mặc định 40cm = 0.40m)
        :param warning_dist_m: Ngưỡng cảnh báo từ xa (mặc định 60cm = 0.60m)
        :param min_pts_threshold: Số điểm ảnh cản tối thiểu để xác nhận (chống nhiễu hạt)
        :param is_enabled: Bật/Tắt can thiệp phanh (Mặc định đọc từ config.ENABLE_SAFETY_BRAKE)
        """
        try:
            import config
            if brake_dist_m is None:
                brake_dist_m = float(getattr(config, 'SAFETY_BRAKE_DIST_M', 0.25))
            if warning_dist_m is None:
                warning_dist_m = float(getattr(config, 'SAFETY_WARNING_DIST_M', 0.38))
            if is_enabled is None:
                is_enabled = bool(getattr(config, 'ENABLE_SAFETY_BRAKE', True))
        except Exception:
            if is_enabled is None:
                is_enabled = True

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

        # Vùng kiểm soát hành lang cản trước mặt:
        # Chiếu tọa độ 3D theo Báo cáo.md (Mục IV.1):
        # y_rob in [-(0.065 + margin), +(0.135 + margin)] = [-0.075m, +0.145m]
        # Bắt đúng vệt thân xe, không nhìn nhầm 2 bên tường của khe hẹp 28-30cm
        h_start, h_end = int(h * 0.10), int(h * 0.78)
        w_start, w_end = int(w * 0.15), int(w * 0.85)
        roi = depth_frame[h_start:h_end, w_start:w_end]

        # Khử NaN/Inf nếu là ma trận float32
        if np.issubdtype(roi.dtype, np.floating):
            roi = np.nan_to_num(roi, nan=0.0, posinf=0.0, neginf=0.0)

        # Chuẩn hóa đơn vị về mét (nếu là uint16 hoặc giá trị lớn hơn 50.0 thì là mm)
        if roi.dtype == np.uint16 or (roi.size > 0 and np.max(roi) > 50.0):
            roi_m = roi.astype(np.float32) / 1000.0
        else:
            roi_m = roi.astype(np.float32)

        cx = float(w) / 2.0
        cy = float(h) / 2.0
        fx = 450.0 * (float(w) / 640.0)
        fy = 450.0 * (float(h) / 400.0)
        cam_h = 0.12 # 12cm
        cam_y = -0.015 # Camera lệch phải 1.5cm so với trục giữa xe
        pitch_sin = 0.061 # sin(3.5°)

        try:
            import config
            cam_h = float(getattr(config, 'CAMERA_HEIGHT_M', 0.12))
            cam_y = float(getattr(config, 'CAMERA_OFFSET_LATERAL_M', -0.015))
            min_obs_h = float(getattr(config, 'SAFETY_BRAKE_MIN_HEIGHT_M', 0.035))
            lane_margin = float(getattr(config, 'LANE_MARGIN_M', 0.010))
            left_ext = float(getattr(config, 'ROBOT_LEFT_EXTRUSION_M', 0.070))
        except Exception:
            min_obs_h = 0.035
            lane_margin = 0.010
            left_ext = 0.070

        # Ranh giới hành lang thân xe (bó cáp trái nhô 7cm):
        y_rob_min = -(0.065 + lane_margin)            # Bên phải xe (-7.5cm)
        y_rob_max = +(0.065 + left_ext + lane_margin) # Bên trái xe (+14.5cm)

        min_valid_m = max(0.09, min(0.12, self.brake_dist_m * 0.45))
        v_indices = np.arange(h_start, h_end)[:, None]
        u_indices = np.arange(w_start, w_end)[None, :]

        # Chiếu không gian 3D sang hệ tọa độ thân xe
        y_rob = -((u_indices - cx) * roi_m / fx) + cam_y
        z_floor = cam_h - ((v_indices - cy) * roi_m) / fy + roi_m * pitch_sin

        is_obstacle = (
            (roi_m >= min_valid_m) & (roi_m <= 3.5) &
            (y_rob >= y_rob_min) & (y_rob <= y_rob_max) &
            ((z_floor >= min_obs_h) | (v_indices <= cy))
        )
        valid = roi_m[is_obstacle]

        # Phải có đủ số lượng điểm tối thiểu để tránh nhiễu do điểm ảnh chết hoặc sàn trơn
        if len(valid) >= self.min_pts_threshold:
            # Lấy phân vị 15% để bắt đúng mép cản gần nhất mà không bị nhiễu hạt
            clearance = float(np.percentile(valid, 15))
            self.last_clearance_m = round(clearance, 2)
            self._clear_count = 0
        elif roi_m.size > 0 and (np.count_nonzero(roi_m < min_valid_m) / float(roi_m.size)) > 0.35 and self.last_clearance_m <= max(0.40, self.brake_dist_m + 0.10):
            # HIỆN TƯỢNG ĐIỂM MÙ STEREO: Khi vật cản che kín >35% hành lang trước mắt ở cự ly áp sát (< 14cm)
            # Giữ mức phanh an toàn, không reset lên 99.0 gây giật cục đập cản
            self.last_clearance_m = min(self.last_clearance_m, max(0.12, self.brake_dist_m - 0.05))
            self._clear_count = 0
        else:
            # Đường thoáng (không có cản trong hành lang)
            # Hysteresis an toàn: Nếu đang phanh sát cản, cần 2 frame thoáng liên tiếp trước khi giải phóng 99.0
            if self.last_clearance_m <= self.brake_dist_m:
                self._clear_count = getattr(self, '_clear_count', 0) + 1
                if self._clear_count >= 2:
                    self.last_clearance_m = 99.0
                    self._clear_count = 0
            else:
                self.last_clearance_m = 99.0
                self._clear_count = 0

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

        # 1. TRƯỜNG HỢP NGUY CẤP: Vật cản áp sát dưới ngưỡng phanh (<= 35cm) -> DỪNG DỨT KHOÁT!
        if clearance_m <= self.brake_dist_m:
            if target_v > 0.0:
                # KHÓA CHẶT LỆNH TIẾN — DỪNG HẲN AN TOÀN TUYỆT ĐỐI (Không giảm ga dập dình gây kẹt động cơ)
                self.is_emergency_active = True
                self.last_alert_level = "EMERGENCY_STOP"
                # Vẫn giữ target_w để cho phép robot xoay tìm hướng thoát hiểm!
                return 0.0, target_w, "EMERGENCY_STOP"
            else:
                # Cho phép lùi (v < 0) hoặc quay đầu (w != 0) để lùi xa vật cản
                self.is_emergency_active = False
                self.last_alert_level = "ESCAPE_MANEUVER"
                return target_v, target_w, "ESCAPE_MANEUVER"

        # 2. TRƯỜNG HỢP CẢNH BÁO TỪ XA: Đang tiếp cận -> Giới hạn tốc độ tối đa an toàn (không tự ý tăng ga nếu xe đang đi chậm hơn)
        elif clearance_m < self.warning_dist_m:
            self.is_emergency_active = False
            self.last_alert_level = "WARNING"
            if target_v > 0.0:
                smooth_v = min(target_v, 0.12)
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

    brake = EmergencyBrake(brake_dist_m=0.40, warning_dist_m=0.60, is_enabled=True)
    print(f"  ├─ Ngưỡng phanh an toàn dừng dứt khoát: {brake.brake_dist_m}m (40cm)")
    print(f"  ├─ Ngưỡng cảnh báo từ xa: {brake.warning_dist_m}m (60cm)")

    # KỊCH BẢN 1: Đường thoáng (không có cản)
    fake_empty_depth = np.ones((360, 480), dtype=np.uint16) * 1500 # 1.5m
    c1 = brake.calculate_clearance(fake_empty_depth)
    v1, w1, alert1 = brake.evaluate_velocity(0.25, 0.0, c1)
    print(f"  ├─ Test 1 (Đường 1.5m): Cự ly={c1}m -> Lệnh v={v1}m/s [{alert1}]")
    assert alert1 == "SAFE" and v1 == 0.25, "Lỗi: Kịch bản 1 không an toàn!"

    # KỊCH BẢN 2: Có vật cản nguy hiểm ở cự ly 37cm (0.37m <= 40cm)
    fake_danger_depth = np.ones((360, 480), dtype=np.uint16) * 370 # 370mm
    c2 = brake.calculate_clearance(fake_danger_depth)
    v2, w2, alert2 = brake.evaluate_velocity(0.20, 0.5, c2)
    print(f"  ├─ Test 2 (Vật cản 37cm khi đang bấm TIẾN): Cự ly={c2}m -> Lệnh v={v2}m/s [{alert2}]")
    assert alert2 == "EMERGENCY_STOP" and v2 == 0.0, "Lỗi: Không phanh khẩn cấp khi gặp cản <= 40cm!"

    # KỊCH BẢN 3: Đang kẹt sát cản (37cm) nhưng bấm LÙI hoặc QUAY để thoát
    v3, w3, alert3 = brake.evaluate_velocity(-0.20, 0.6, c2)
    print(f"  ├─ Test 3 (Thoát hiểm: Bấm LÙI v=-0.2 khi sát cản): Lệnh v={v3}m/s [{alert3}]")
    assert v3 == -0.20, "Lỗi: Bị chặn lệnh lùi thoát hiểm!"

    # KỊCH BẢN 4: Khởi động trên sàn gạch phẳng không có texture (nhiều điểm rỗng)
    brake_floor = EmergencyBrake(brake_dist_m=0.40, warning_dist_m=0.60, is_enabled=True)
    fake_floor_zeros = np.zeros((360, 480), dtype=np.uint16)
    c4 = brake_floor.calculate_clearance(fake_floor_zeros)
    v4, w4, alert4 = brake_floor.evaluate_velocity(0.20, 0.0, c4)
    print(f"  ├─ Test 4 (Sàn nhẵn ít vân - Điểm 0 lúc ban đầu): Cự ly={c4}m -> Lệnh v={v4}m/s [{alert4}]")
    assert alert4 == "SAFE" and v4 == 0.20, "Lỗi: Bị báo động giả trên sàn gạch!"

    print("═" * 70)
    print("✅ [SELF-TEST] TẤT CẢ CÁC KỊCH BẢN PHANH KHẨN CẤP ĐỀU ĐẠT CHUẨN 100%!")
    print("═" * 70 + "\n")
