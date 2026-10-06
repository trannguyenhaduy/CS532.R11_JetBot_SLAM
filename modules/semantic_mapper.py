#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 6: Bộ tạo Bản đồ Ngữ nghĩa 3D (Semantic Mapper)
Tuân thủ Quy tắc Harness 2: Chiếu tọa độ thuần hình học vi sai, miễn nhiễm lỗi PyInit__tf2.
"""

import sys
import math
import time

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass

class SemanticObject:
    def __init__(self, obj_id, name, x, y, z, score):
        self.obj_id = obj_id
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.score = score
        self.seen_count = 1
        self.last_seen = time.time()

    def update(self, x, y, z, score):
        alpha = 0.3
        self.x = (1 - alpha) * self.x + alpha * x
        self.y = (1 - alpha) * self.y + alpha * y
        self.z = (1 - alpha) * self.z + alpha * z
        self.score = max(self.score, score)
        self.seen_count += 1
        self.last_seen = time.time()


class SemanticMapper:
    def __init__(self, cluster_dist=0.40, min_seen=3):
        self.cluster_dist = cluster_dist
        self.min_seen = min_seen
        self.catalog = []

    def transform_cam_to_world(self, xc, yc, zc, rx, ry, rz, yaw):
        """
        Chiếu điểm từ Camera Frame sang World Frame bằng phép toán vi sai.
        Camera: X phải, Y xuống, Z tới -> Robot: X tới (zc), Y trái (-xc), Z lên (-yc + 0.12)
        """
        x_rob = zc
        y_rob = -xc
        z_rob = -yc + 0.12

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        wx = rx + (x_rob * cos_y - y_rob * sin_y)
        wy = ry + (x_rob * sin_y + y_rob * cos_y)
        wz = rz + z_rob
        return round(wx, 2), round(wy, 2), round(wz, 2)

    def add_detection(self, obj_id, name, xc, yc, zc, score, rx=0.0, ry=0.0, rz=0.0, yaw=0.0):
        wx, wy, wz = self.transform_cam_to_world(xc, yc, zc, rx, ry, rz, yaw)

        # Gom cụm Euclidean Clustering
        for obj in self.catalog:
            if obj.name == name or (obj.obj_id == obj_id and obj_id != 0):
                d = math.hypot(wx - obj.x, wy - obj.y)
                if d < self.cluster_dist:
                    obj.update(wx, wy, wz, score)
                    return obj

        new_obj = SemanticObject(obj_id, name, wx, wy, wz, score)
        self.catalog.append(new_obj)
        return new_obj

    def get_confirmed_objects(self):
        """Chỉ trả về các vật thể đã thấy đủ số lần (chống nhấp nháy)"""
        return [
            {
                "id": o.obj_id, "name": o.name, "score": o.score,
                "x": round(o.x, 2), "y": round(o.y, 2), "z": round(o.z, 2),
                "seen": o.seen_count
            }
            for o in self.catalog if o.seen_count >= self.min_seen
        ]


if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử SemanticMapper...")
    sm = SemanticMapper(cluster_dist=0.40, min_seen=2)
    # Giả lập thấy cùng 1 cái ghế 3 lần ở các góc nhìn khác nhau
    sm.add_detection(56, "CHAIR", 0.1, 0.0, 1.5, 0.85, rx=0.0, ry=0.0, rz=0.0, yaw=0.0)
    sm.add_detection(56, "CHAIR", 0.12, 0.0, 1.52, 0.88, rx=0.05, ry=0.0, rz=0.0, yaw=0.0)
    sm.add_detection(56, "CHAIR", 0.09, 0.0, 1.49, 0.90, rx=0.1, ry=0.0, rz=0.0, yaw=0.0)

    confirmed = sm.get_confirmed_objects()
    print(f"  ├─ Số cụm vật thể xác thực: {len(confirmed)}")
    print(f"  └─ Thông tin cụm: {confirmed}")
    assert len(confirmed) == 1, "Lỗi: Không gom được 3 lần thấy thành 1 vật thể duy nhất!"
    print("✅ [SELF-TEST] SemanticMapper ĐẠT CHUẨN!")
