#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 3: Quản lý luồng Camera OAK-D S2 & Mây điểm 3D (Camera Streamer)
Tuân thủ Quy tắc Harness 3: Giảm tải CPU bằng cách skip frames và downsampling.
"""

import sys
import time
import math
import numpy as np
import cv2

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

class CameraStreamer:
    def __init__(self, depth_step=25, depth_skip=5, img_skip=2):
        self.depth_step = depth_step
        self.depth_skip = depth_skip
        self.img_skip = img_skip

        self._img_counter = 0
        self._depth_counter = 0

        self.latest_jpeg = None
        self.points_3d = []
        self.valid_depth_pct = 0.0

        # Thông số camera mặc định OAK-D S2
        self.fx = 450.0
        self.fy = 450.0
        self.cx = 320.0
        self.cy = 200.0

    def process_color_frame(self, bgr_img, detections_annotator=None):
        """Xử lý frame màu và nén thành JPEG phục vụ Web MJPEG stream"""
        if bgr_img is None:
            return self.latest_jpeg

        self._img_counter += 1
        # Frame đầu tiên luôn nén ngay, sau đó mới áp dụng skip để giảm tải CPU
        if self.latest_jpeg is not None and self._img_counter % self.img_skip != 0:
            return self.latest_jpeg

        annotated = bgr_img.copy()
        if detections_annotator:
            annotated = detections_annotator(annotated)

        _, jpeg = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        self.latest_jpeg = jpeg.tobytes()
        return self.latest_jpeg

    def process_depth_frame(self, depth_uint16_mm, rx=0.0, ry=0.0, rz=0.0, yaw=0.0):
        """Chiếu ma trận độ sâu thành đám mây điểm 3D với tần số ~3 Hz"""
        self._depth_counter += 1
        if self._depth_counter % self.depth_skip != 0:
            return self.points_3d

        if depth_uint16_mm is None or not isinstance(depth_uint16_mm, np.ndarray):
            return self.points_3d

        h, w = depth_uint16_mm.shape[:2]
        u_grid, v_grid = np.meshgrid(np.arange(0, w, self.depth_step), np.arange(0, h, self.depth_step))
        z_sample = depth_uint16_mm[v_grid, u_grid].astype(np.float32) / 1000.0

        valid = (z_sample > 0.3) & (z_sample < 3.2)
        self.valid_depth_pct = round(float(np.count_nonzero(valid)) / max(1.0, float(valid.size)) * 100.0, 1)

        z_val = z_sample[valid]
        u_val = u_grid[valid]
        v_val = v_grid[valid]

        x_cam = (u_val - self.cx) * z_val / self.fx
        y_cam = (v_val - self.cy) * z_val / self.fy

        x_rob = z_val
        y_rob = -x_cam
        z_rob = -y_cam + 0.08

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        x_world = rx + (x_rob * cos_y - y_rob * sin_y)
        y_world = ry + (x_rob * sin_y + y_rob * cos_y)
        z_world = z_rob

        new_pts = []
        for i in range(min(len(x_world), 300)):
            new_pts.append([round(float(x_world[i]), 2), round(float(y_world[i]), 2), round(float(z_world[i]), 2)])

        self.points_3d.extend(new_pts)
        if len(self.points_3d) > 2500:
            self.points_3d = self.points_3d[-2500:]

        return self.points_3d


if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử CameraStreamer...")
    cs = CameraStreamer()
    # Tạo frame ảnh giả lập
    fake_img = np.zeros((400, 640, 3), dtype=np.uint8)
    cv2.putText(fake_img, "TEST STREAM", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    # Xử lý 2 frame để qua bộ lọc skip
    cs.process_color_frame(fake_img)
    jpeg = cs.process_color_frame(fake_img)
    assert jpeg is not None and len(jpeg) > 1000, "Lỗi: Không tạo được ảnh JPEG!"
    print(f"  ├─ Kích thước JPEG tạo ra: {len(jpeg)} bytes")
    print("✅ [SELF-TEST] CameraStreamer ĐẠT CHUẨN!")
