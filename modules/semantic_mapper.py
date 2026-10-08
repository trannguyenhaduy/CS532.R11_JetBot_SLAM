#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 6: Bộ tạo Bản đồ Ngữ nghĩa & Bản đồ Chiếm dụng 2D (Semantic & Occupancy SLAM Mapper)
Tuân thủ Quy tắc Harness 2: Chiếu tọa độ thuần hình học vi sai, miễn nhiễm lỗi PyInit__tf2.
Tích hợp Bản đồ Chiếm dụng 2D (Occupancy Grid Map) thời gian thực từ Stereo Depth OAK-D S2.
"""

import sys
import math
import time
import base64
import numpy as np
import cv2

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


class OccupancyGridMap2D:
    """
    Bản đồ chiếm dụng 2D (Occupancy Grid Map) thời gian thực.
    Kích thước 8m x 8m, độ phân giải 0.05m (5cm/ô), ma trận 160x160.
    Giá trị ô:
      0: Chưa khám phá (Unknown)
      128: Vùng thoáng đã quan sát (Free space)
      255: Vật cản / Tường (Occupied)
    """
    def __init__(self, size_m=8.0, resolution=0.05):
        self.size_m = float(size_m)
        self.resolution = float(resolution)
        self.origin_x = -size_m / 2.0
        self.origin_y = -size_m / 2.0
        self.width = int(round(size_m / resolution))
        self.height = int(round(size_m / resolution))
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.version = 1
        self.last_encode_time = 0.0
        self.cached_b64 = ""
        self.num_free_cells = 0
        self.num_occ_cells = 0

    def world_to_grid(self, wx, wy):
        """Chuyển tọa độ thế giới (m) sang pixel bản đồ (col, row)"""
        col = int(round((wx - self.origin_x) / self.resolution))
        row = int(round((self.origin_y + self.size_m - wy) / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None, None

    def update_from_scan(self, rx, ry, points_3d, max_dist=3.5):
        """
        Cập nhật ma trận chiếm dụng từ mây điểm 3D OAK-D S2.
        Chiếu các điểm vật cản 3D xuống 2D và vẽ tia quan sát (Ray-casting C++ cv2.line).
        """
        r_col, r_row = self.world_to_grid(rx, ry)
        if r_col is None:
            return

        # Dọn sạch vùng xung quanh robot (bán kính 0.20m an toàn)
        cv2.circle(self.grid, (r_col, r_row), int(round(0.20 / self.resolution)), 128, -1)

        updated = False
        if points_3d:
            # Lấy mẫu tối đa 40 điểm cản để tối ưu tốc độ CPU < 1ms
            sampled = points_3d[::max(1, len(points_3d) // 40)] if len(points_3d) > 40 else points_3d
            for pt in sampled:
                try:
                    wx, wy = float(pt[0]), float(pt[1])
                    wz = float(pt[2]) if len(pt) > 2 else 0.2
                    # Chỉ lấy các điểm trong tầm cao thân xe/vật cản (-0.08m đến 0.80m)
                    if not (-0.08 <= wz <= 0.80):
                        continue

                    dist = math.hypot(wx - rx, wy - ry)
                    if 0.15 <= dist <= max_dist:
                        o_col, o_row = self.world_to_grid(wx, wy)
                        if o_col is not None:
                            # Vẽ tia quét quang học: Free space từ robot tới vật cản
                            cv2.line(self.grid, (r_col, r_row), (o_col, o_row), 128, 1)
                            # Đánh dấu ô vật cản
                            self.grid[o_row, o_col] = 255
                            updated = True
                except Exception:
                    pass

        if updated:
            self.version += 1

    def update_from_ros_occupancy_grid(self, msg):
        """
        Cập nhật trực tiếp từ ROS nav_msgs/OccupancyGrid (RTAB-Map hoặc Cartographer).
        msg.data: int8[] (-1: Unknown, 0: Free space, 100: Obstacle)
        """
        try:
            w = int(msg.info.width)
            h = int(msg.info.height)
            if w <= 0 or h <= 0 or len(msg.data) < w * h:
                return

            res = float(msg.info.resolution)
            ox = float(msg.info.origin.position.x)
            oy = float(msg.info.origin.position.y)

            raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))

            # Chuyển đổi chuẩn hóa sang định dạng nội bộ:
            # -1 -> 0 (unknown)
            # 0 -> 128 (free space)
            # > 40 -> 255 (occupied)
            new_grid = np.zeros((h, w), dtype=np.uint8)
            new_grid[raw_data == 0] = 128
            new_grid[raw_data > 40] = 255

            # ROS OccupancyGrid có hàng 0 ở dưới cùng, lật trục Y cho đúng hệ pixel chuẩn
            self.grid = np.flipud(new_grid)
            self.width = w
            self.height = h
            self.resolution = res
            self.origin_x = ox
            self.origin_y = oy
            self.size_m = max(w * res, h * res)
            self.version += 1
            self.cached_b64 = ""
        except Exception:
            pass

    def get_png_base64(self, force=False):
        """Tạo ảnh PNG 4-kênh BGRA phát sáng Cyberpunk cho Web Cockpit (tần số ~3 Hz)"""
        now = time.time()
        if not force and self.cached_b64 and (now - self.last_encode_time < 0.30):
            return self.cached_b64, self.version

        bgra = np.zeros((self.height, self.width, 4), dtype=np.uint8)
        mask_free = (self.grid == 128)
        mask_occ = (self.grid == 255)

        # Free space: Xanh lục neon dịu, bán trong suốt (alpha=65)
        bgra[mask_free] = [180, 255, 60, 65]
        # Occupied: Cyan rực rỡ (alpha=255)
        bgra[mask_occ] = [255, 230, 0, 255]

        # Giãn nhẹ 2px để vệt tường liền mạch sắc nét
        kernel = np.ones((2, 2), np.uint8)
        dilated_occ = cv2.dilate(mask_occ.astype(np.uint8), kernel)
        bgra[dilated_occ > 0] = [255, 240, 0, 255]

        success, buf = cv2.imencode('.png', bgra)
        if success:
            self.cached_b64 = base64.b64encode(buf).decode('ascii')
            self.last_encode_time = now
            self.num_free_cells = int(np.count_nonzero(mask_free))
            self.num_occ_cells = int(np.count_nonzero(mask_occ))

        return self.cached_b64, self.version


try:
    from modules.occupancy_slam import OccupancySLAM, PinnedLandmark
except ImportError:
    from occupancy_slam import OccupancySLAM, PinnedLandmark


class SemanticMapper(OccupancySLAM):
    """
    SemanticMapper kế thừa trực tiếp OccupancySLAM:
    Bản đồ chiếm dụng 2D tích lũy vĩnh viễn, ghim vật thể ngữ nghĩa cố định,
    hỗ trợ lưu/nạp/xóa bản đồ.
    """
    def __init__(self, cluster_dist=0.40, min_seen=2, grid_size=8.0, resolution=0.05, save_dir="maps"):
        super().__init__(size_m=grid_size, resolution=resolution, save_dir=save_dir)
        self.cluster_dist = cluster_dist
        self.min_seen = min_seen
        # Alias occupancy_grid trỏ tới chính self để tương thích ngược
        self.occupancy_grid = self

    def get_confirmed_objects(self):
        """Chỉ trả về các vật thể đã thấy đủ số lần (chống nhấp nháy)"""
        return [lm.to_dict() for lm in self.landmarks if lm.seen_count >= self.min_seen]



if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử SemanticMapper & OccupancyGridMap2D...")
    sm = SemanticMapper(cluster_dist=0.40, min_seen=2)
    # Giả lập thấy cùng 1 cái ghế 3 lần ở các góc nhìn khác nhau
    sm.add_detection(56, "CHAIR", 0.1, 0.0, 1.5, 0.85, rx=0.0, ry=0.0, rz=0.0, yaw=0.0)
    sm.add_detection(56, "CHAIR", 0.12, 0.0, 1.52, 0.88, rx=0.05, ry=0.0, rz=0.0, yaw=0.0)
    sm.add_detection(56, "CHAIR", 0.09, 0.0, 1.49, 0.90, rx=0.1, ry=0.0, rz=0.0, yaw=0.0)

    confirmed = sm.get_confirmed_objects()
    print(f"  ├─ Số cụm vật thể xác thực: {len(confirmed)}")
    assert len(confirmed) == 1, "Lỗi: Không gom được 3 lần thấy thành 1 vật thể duy nhất!"

    # Giả lập quét 2D
    mock_scan = [[1.5, y, 0.2] for y in np.linspace(-0.8, 0.8, 20)]
    sm.update_scan(0.0, 0.0, mock_scan)
    payload = sm.get_map_payload()
    print(f"  ├─ Kích thước bản đồ: {payload['map_width']}x{payload['map_height']} ô ({payload['map_resolution']}m/ô)")
    print(f"  ├─ Chuỗi Base64 PNG: {len(payload['map_b64'])} ký tự (Phiên bản {payload['map_version']})")
    assert len(payload['map_b64']) > 500, "Lỗi: Không mã hóa được ảnh PNG bản đồ 2D!"
    print("✅ [SELF-TEST] SemanticMapper & OccupancyGridMap2D ĐẠT CHUẨN XUẤT SẮC!")
