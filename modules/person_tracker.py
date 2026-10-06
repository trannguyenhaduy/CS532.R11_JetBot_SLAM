#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 5: Bộ điều khiển bám người HRI (Person Tracker)
Tuân thủ Quy tắc Harness 4: Mặc định TẮT, chỉ gửi lệnh dừng 1 lần khi mất mục tiêu.
"""

import sys
import time

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

class PersonTracker:
    def __init__(self, target_dist=0.85, kp_dist=0.40, kd_dist=0.08, kp_ang=1.20, kd_ang=0.15):
        self.is_enabled = False  # BẮT BUỘC TẮT MẶC ĐỊNH
        self.target_dist = target_dist
        self.kp_dist = kp_dist
        self.kd_dist = kd_dist
        self.kp_ang = kp_ang
        self.kd_ang = kd_ang

        self.last_ez = 0.0
        self.last_ex = 0.0
        self.last_ctrl_time = time.time()
        self.last_person_time = 0.0
        self.has_sent_stop = True

    def set_enabled(self, enabled: bool):
        self.is_enabled = bool(enabled)
        if not self.is_enabled:
            self.has_sent_stop = False

    def compute_command(self, detections):
        """Tính toán lệnh vận tốc (v, w). Trả về (None, None) nếu không có lệnh mới"""
        if not self.is_enabled:
            return None, None

        # Lọc danh sách người
        persons = [d for d in detections if d.get('id') == 0 or 'PERSON' in str(d.get('name', '')).upper()]

        now = time.time()
        if not persons:
            # Nếu mất người quá 0.8s: Chỉ phát lệnh dừng 1 LẦN DUY NHẤT
            if not self.has_sent_stop and (now - self.last_person_time > 0.8):
                self.has_sent_stop = True
                return 0.0, 0.0
            return None, None

        # Chọn người gần trục giữa camera nhất
        target = min(persons, key=lambda p: abs(float(p.get('x', 0.0))))
        x = float(target.get('x', 0.0))
        z = float(target.get('z', 0.0))

        dt = max(0.01, now - self.last_ctrl_time)
        ez = z - self.target_dist
        dez = (ez - self.last_ez) / dt

        ex = x
        dex = (ex - self.last_ex) / dt

        # Bộ điều khiển PD
        v = (self.kp_dist * ez) + (self.kd_dist * dez) if abs(ez) > 0.08 else 0.0
        w = (-self.kp_ang * ex) - (self.kd_ang * dex) if abs(ex) > 0.05 else 0.0

        # Giới hạn an toàn
        v = max(-0.20, min(0.30, v))
        w = max(-1.00, min(1.00, w))

        self.last_ez = ez
        self.last_ex = ex
        self.last_ctrl_time = now
        self.last_person_time = now
        self.has_sent_stop = False

        return v, w


if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử PersonTracker...")
    pt = PersonTracker()
    assert pt.is_enabled is False, "Lỗi: is_enabled phải TẮT mặc định!"
    print(f"  ├─ Trạng thái kích hoạt mặc định: {pt.is_enabled} (Chuẩn)")
    pt.set_enabled(True)
    v, w = pt.compute_command([{"id": 0, "name": "PERSON", "x": 0.2, "z": 1.5}])
    print(f"  ├─ Lệnh lái khi thấy người ở 1.5m: v={v:.2f} m/s, w={w:.2f} rad/s")
    assert v > 0, "Lỗi: Xe phải có xu hướng tiến khi người ở xa hơn 0.85m!"
    print("✅ [SELF-TEST] PersonTracker ĐẠT CHUẨN!")
