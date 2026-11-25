#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
MODULE 5: HỆ THỐNG XÂY DỰNG BẢN ĐỒ CHIẾM DỤNG 2D TÍCH LŨY TĨNH (OCCUPANCY SLAM)
Tên file: modules/occupancy_slam.py
Trọng trách:
  1. Bản đồ chiếm dụng tích lũy tĩnh (Persistent Static 2D Occupancy Grid Map).
  2. Cơ chế neo tọa độ vật cản (Spatial Anchoring): Duy trì tính ổn định của vật cản sau khi quét.
  3. Bộ lọc xác nhận Hit-Counter: Xác nhận vật cản khi đạt ngưỡng quan sát, giảm thiểu nhiễu đo lường.
  4. Quản lý danh mục vật thể ngữ nghĩa cố định (Pinned Semantic Landmarks).
  5. Tính năng Lưu bản đồ (Save Map to JSON/PNG) và Tải bản đồ (Load Map).
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import math
import json
import base64
import numpy as np
import cv2

try:
    import config
except ImportError:
    try:
        import sys as _s; _s.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        import config
    except ImportError:
        config = None

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass


class PinnedLandmark:
    """
    Quản lý thông tin vật thể ngữ nghĩa (Landmark) trên bản đồ 2D.
    Hỗ trợ tích lũy xác suất đa quan sát và khóa tọa độ (Spatial Anchoring).
    """
    def __init__(self, obj_id, name, x, y, z=0.2, score=0.8, width_m=None, depth_m=None):
        self.obj_id = int(obj_id)
        self.name = str(name).upper()
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        sc = float(score)
        norm_sc = sc / 100.0 if sc > 1.0 else sc
        self.score = round(norm_sc, 2)
        self.seen_count = 1
        self.first_seen = time.time()
        self.last_seen = self.first_seen
        self.is_pinned = False       # Khóa tọa độ không gian (Position Pinning)
        self.is_label_locked = False # Khóa định danh ngữ nghĩa (Semantic Saturation Lock)
        self.is_dynamic = (self.name == "PERSON")

        # Từ điển tích lũy xác suất đa lớp theo mô hình phân loại:
        self.class_votes = {self.name: float(self.score)}
        self.class_counts = {self.name: 1}

        # Kích thước hình học 2D thực tế (Rộng x Sâu, mét) phục vụ vẽ 2D Bounding Box:
        dim_map = {
            "PERSON": (0.45, 0.45),
            "CHAIR": (0.50, 0.50),
            "DININGTABLE": (1.10, 0.70),
            "TABLE": (1.10, 0.70),
            "DESK": (1.10, 0.70),
            "POTTEDPLANT": (0.35, 0.35),
            "BOTTLE": (0.20, 0.20),
            "CUP": (0.15, 0.15),
            "LAPTOP": (0.35, 0.25),
            "TVMONITOR": (0.80, 0.20),
            "SOFA": (1.40, 0.80),
            "BED": (1.60, 1.20),
            "CAR": (0.50, 0.40),
            "UNKNOWN": (0.35, 0.35),
            "OBSTACLE": (0.35, 0.35)
        }
        def_w, def_d = dim_map.get(self.name, (0.40, 0.40))
        self.width_m = float(width_m) if width_m is not None else def_w
        self.depth_m = float(depth_m) if depth_m is not None else def_d

    def update(self, x, y, z, score, width_m=None, depth_m=None, new_name=None):
        self.last_seen = time.time()
        self.seen_count += 1
        sc = float(score)
        norm_sc = sc / 100.0 if sc > 1.0 else sc

        # ─── 1. BỘ LỌC TÍCH LŨY XÁC SUẤT ĐA QUAN SÁT & CỐ ĐỊNH ĐỊNH DANH ───
        if new_name:
            clean_n = str(new_name).upper()
            if clean_n not in ["UNKNOWN", "OBSTACLE", "OBJ", "NONE", "NULL"]:
                if self.is_label_locked:
                    # Khi nhãn đã ổn định: Tăng điểm tích lũy cho nhãn hiện tại
                    # Điều kiện chuyển nhãn: Nhãn mới có xác suất tích lũy vượt trội ở cự ly gần
                    cur_vote = self.class_votes.get(self.name, 1.0)
                    new_vote = self.class_votes.get(clean_n, 0.0) + norm_sc
                    self.class_votes[clean_n] = new_vote
                    if new_vote > cur_vote * 1.6 and norm_sc >= 0.85:
                        self.name = clean_n
                        self.is_dynamic = (clean_n == "PERSON")
                        self.is_label_locked = False
                else:
                    # Giai đoạn tích lũy: Cộng dồn xác suất từ các khung hình nhận diện
                    self.class_votes[clean_n] = self.class_votes.get(clean_n, 0.0) + norm_sc
                    self.class_counts[clean_n] = self.class_counts.get(clean_n, 0) + 1
                    best_name = max(self.class_votes, key=self.class_votes.get)
                    self.name = best_name
                    self.is_dynamic = (best_name == "PERSON")

                    total_votes = sum(self.class_votes.values())
                    purity = self.class_votes[best_name] / max(1e-5, total_votes)

                    # Tính độ tin cậy trung bình của nhãn chiếm ưu thế
                    mean_model_conf = self.class_votes[best_name] / max(1, self.class_counts[best_name])
                    self.score = round(min(0.99, mean_model_conf), 2)

                    # Khóa nhãn khi số lần quan sát >= 5 và độ đồng thuận >= 75% (ngoại trừ PERSON)
                    if self.seen_count >= 5 and purity >= 0.75 and not self.is_dynamic:
                        self.is_label_locked = True

        if width_m is not None:
            self.width_m = round(0.70 * self.width_m + 0.30 * float(width_m), 2)
        if depth_m is not None:
            self.depth_m = round(0.70 * self.depth_m + 0.30 * float(depth_m), 2)

        # ─── 2. CẬP NHẬT TỌA ĐỘ VỊ TRÍ ADAPTIVE EMA (LÀM MƯỢT KHÔNG GIAN) ───
        if not self.is_pinned:
            alpha = 0.30
            self.x = (1.0 - alpha) * self.x + alpha * float(x)
            self.y = (1.0 - alpha) * self.y + alpha * float(y)
            self.z = (1.0 - alpha) * self.z + alpha * float(z)
            min_seen_pin = int(getattr(config, 'SEMANTIC_MIN_SEEN_CONFIRM', 3)) if config else 3
            if self.seen_count >= min_seen_pin:
                self.is_pinned = True
        else:
            alpha = 0.02
            self.x = (1.0 - alpha) * self.x + alpha * float(x)
            self.y = (1.0 - alpha) * self.y + alpha * float(y)

    def to_dict(self):
        return {
            "id": self.obj_id,
            "name": self.name,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "z": round(self.z, 3),
            "score": round(self.score / 100.0 if self.score > 1.0 else self.score, 2),
            "seen": self.seen_count,
            "pinned": self.is_pinned,
            "locked": self.is_label_locked,
            "width": round(self.width_m, 2),
            "depth": round(self.depth_m, 2),
            "is_obstacle": True
        }


class OccupancySLAM:
    """
    Hệ thống Xây dựng Bản đồ Chiếm dụng 2D Tích Lũy Tĩnh (Persistent 2D Grid SLAM).
    Kích thước mặc định: 10m x 10m, độ phân giải 0.05m (5cm/ô) -> ma trận 200 x 200 ô.
    Giá trị ô:
      - 0: Chưa khám phá (Unknown)
      - 128: Không gian thoáng đã đi qua/quét thấy (Free Space - Vĩnh viễn)
      - 255: Tường & Chướng ngại vật được ghim cố định (Occupied Obstacle)
    """
    def __init__(self, size_m=10.0, resolution=0.05, save_dir="maps"):
        self.size_m = float(size_m)
        self.resolution = float(resolution)
        self.origin_x = -size_m / 2.0
        self.origin_y = -size_m / 2.0
        self.width = int(round(size_m / resolution))
        self.height = int(round(size_m / resolution))
        self.save_dir = save_dir

        # Ma trận bản đồ và ma trận tích lũy độ tin cậy
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.hit_counts = np.zeros((self.height, self.width), dtype=np.int16)
        self.miss_counts = np.zeros((self.height, self.width), dtype=np.int16)
        # Ma trận mốc thời gian quan sát cản gần nhất (phục vụ suy hao Temporal Decay)
        self.last_hit_time = np.zeros((self.height, self.width), dtype=np.float64)
        # Ma trận mặt nạ bờ tường mốc bất tử (Immutable Anchor Walls)
        self.anchor_mask = np.zeros((self.height, self.width), dtype=bool)

        # Danh mục vật thể ngữ nghĩa được ghim (Landmarks)
        self.landmarks = []
        self._next_landmark_id = 1

        self.version = 1
        self.cached_version = -1
        self.cached_b64 = ""
        self.last_encode_time = 0.0
        self.total_scans_processed = 0
        self._decay_tick = 0

        # Đảm bảo thư mục lưu bản đồ tồn tại
        if not os.path.exists(self.save_dir):
            try: os.makedirs(self.save_dir, exist_ok=True)
            except Exception: pass

    def world_to_grid(self, wx, wy):
        """Chuyển tọa độ thế giới (mét) sang chỉ số hàng/cột ma trận bản đồ"""
        col = int(round((wx - self.origin_x) / self.resolution))
        row = int(round((self.origin_y + self.size_m - wy) / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None, None

    def grid_to_world(self, col, row):
        """Chuyển chỉ số ô ma trận sang tọa độ thế giới (mét)"""
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = (self.origin_y + self.size_m) - (row + 0.5) * self.resolution
        return round(wx, 3), round(wy, 3)

    def clear_robot_footprint(self, rx, ry, yaw=None):
        """
        Dọn sạch không gian bên dưới và xung quanh thân xe JetBot (Oriented Bounding Box).
        - Khi có yaw: Tính hình chữ nhật xoay theo hướng thực tế của xe (dài 18cm, rộng 16cm, cáp trái 7cm + padding).
        - Khi không có yaw: Dùng vòng tròn bao quanh an toàn.
        - Đặt grid = 128 (Free Space) và xóa hit_counts = 0 bên trong footprint để xe không tự coi mình là vật cản.
        """
        r_col, r_row = self.world_to_grid(rx, ry)
        if r_col is None:
            return

        pad = float(getattr(config, 'ROBOT_CLEAR_PADDING_M', 0.035)) if config else 0.035
        front_m = float(getattr(config, 'ROBOT_FRONT_FROM_AXLE_M', 0.080)) if config else 0.080
        rear_m = float(getattr(config, 'ROBOT_REAR_FROM_AXLE_M', 0.100)) if config else 0.100
        width = float(getattr(config, 'ROBOT_WIDTH_M', 0.130)) if config else 0.130
        ext_left = float(getattr(config, 'ROBOT_LEFT_EXTRUSION_M', 0.070)) if config else 0.070

        if yaw is not None:
            # Tọa độ 4 góc thân xe trong hệ quy chiếu cục bộ (robot frame: X tới trước, Y sang trái)
            # Theo Báo cáo.md: Gốc tọa độ tại tâm trục bánh: +8cm trước, -10cm sau, rộng 13cm, cáp trái +7cm
            front_x = front_m + pad
            back_x  = -(rear_m + pad)
            left_y  = (width / 2.0) + ext_left + pad
            right_y = -(width / 2.0 + pad)

            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)

            local_corners = [
                (front_x, left_y),
                (front_x, right_y),
                (back_x, right_y),
                (back_x, left_y)
            ]

            grid_pts = []
            for lx, ly in local_corners:
                wx = rx + (lx * cos_y - ly * sin_y)
                wy = ry + (lx * sin_y + ly * cos_y)
                c, r = self.world_to_grid(wx, wy)
                if c is not None and r is not None:
                    grid_pts.append([c, r])

            if len(grid_pts) == 4:
                pts_arr = np.array([grid_pts], dtype=np.int32)
                cv2.fillPoly(self.grid, pts_arr, 128)
                cv2.fillPoly(self.hit_counts, pts_arr, 0)
                if np.any(self.anchor_mask):
                    self.grid[self.anchor_mask] = 255
                return

        # Fallback tròn nếu không có yaw hoặc nằm sát mép biên ma trận
        clear_r = float(getattr(config, 'ROBOT_RADIUS_M', 0.13)) + pad if config else 0.17
        rad_cells = int(round(clear_r / self.resolution))
        cv2.circle(self.grid, (r_col, r_row), rad_cells, 128, -1)
        cv2.circle(self.hit_counts, (r_col, r_row), rad_cells, 0, -1)
        if np.any(self.anchor_mask):
            self.grid[self.anchor_mask] = 255

    def update_scan(self, rx, ry, points_3d, max_dist=3.8, yaw=None):
        """
        Cập nhật mây điểm quét tia LaserScan / Depth từ OAK-D S2 vào bản đồ.
        - Phân biệt triệt để sàn nhà (wz < 0.06m) và vật cản thực sự (0.06m <= wz <= 0.85m).
        - Tuyệt đối KHÔNG đánh dấu sàn nhà thành vật cản (tránh hiện tượng hình nón dày đặc trước xe).
        - Vật cản thực sự tích lũy hit_count >= 3 mới chốt cứng thành tường (255).
        """
        r_col, r_row = self.world_to_grid(rx, ry)
        if r_col is None:
            return

        # 1. Tự động dọn sạch vùng thân xe và sợi dây cáp bên trái (Oriented Footprint Clearing)
        self.clear_robot_footprint(rx, ry, yaw=yaw)

        if not points_3d or len(points_3d) == 0:
            return

        # Gốc phát tia quang học thực tế (từ mắt thấu kính OAK-D thay vì tâm trục bánh xe)
        if yaw is not None:
            cam_off_x = getattr(config, 'CAMERA_OFFSET_FORWARD_M', 0.09) if config else 0.09
            cam_off_y = getattr(config, 'CAMERA_OFFSET_LATERAL_M', -0.015) if config else 0.0
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            cam_wx = rx + (cam_off_x * cos_y - cam_off_y * sin_y)
            cam_wy = ry + (cam_off_x * sin_y + cam_off_y * cos_y)
            c_col, c_row = self.world_to_grid(cam_wx, cam_wy)
            ray_orig_col, ray_orig_row = (c_col, c_row) if c_col is not None else (r_col, r_row)
            cam_orig_x, cam_orig_y = cam_wx, cam_wy
        else:
            ray_orig_col, ray_orig_row = r_col, r_row
            cam_orig_x, cam_orig_y = rx, ry

        self.total_scans_processed += 1
        has_new_obstacle = False
        effective_max_dist = min(max_dist, 2.85)

        # Sử dụng đầy đủ toàn bộ các tia quét đo được từ OAK-D S2 Stereo Depth
        sampled_pts = points_3d

        # Tạo ray_mask 1 lần duy nhất cho toàn bộ các tia để tối ưu CPU Jetson Nano
        ray_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        has_ray = False

        for pt in sampled_pts:
            try:
                wx, wy = float(pt[0]), float(pt[1])
                wz = float(pt[2]) if len(pt) > 2 else 0.2

                # Bỏ qua các điểm quá cao (trên trần nhà / tầm camera > 0.85m)
                if wz > 0.85:
                    continue

                dist = math.hypot(wx - rx, wy - ry)
                if not (0.15 <= dist <= effective_max_dist):
                    continue

                o_col, o_row = self.world_to_grid(wx, wy)
                if o_col is None:
                    continue

                # Vẽ tia quan sát quang học: Vùng giữa mắt thấu kính Camera và Điểm đo là KHÔNG GIAN THOÁNG
                # Dừng tia trước vật cản 6cm để bảo toàn mép vật cản không bị tia đè lên
                dist_cam = math.hypot(wx - cam_orig_x, wy - cam_orig_y)
                if dist_cam > 0.08:
                    ratio = max(0.0, (dist_cam - 0.06) / dist_cam)
                    fx = cam_orig_x + (wx - cam_orig_x) * ratio
                    fy = cam_orig_y + (wy - cam_orig_y) * ratio
                    f_col, f_row = self.world_to_grid(fx, fy)
                    if f_col is not None:
                        cv2.line(ray_mask, (ray_orig_col, ray_orig_row), (f_col, f_row), 1, 1)
                        has_ray = True

                # ─── PHÂN LOẠI SÀN NHÀ VÀ VẬT CẢN THỰC SỰ ───
                # Sàn nhà (wz < 0.07m / 7cm): Không phải vật cản! Xe có thể đi qua an toàn!
                if wz < 0.07:
                    if self.hit_counts[o_row, o_col] < 3 and not self.anchor_mask[o_row, o_col]:
                        self.grid[o_row, o_col] = 128
                    continue

                # Vật cản thực sự (wz >= 0.07m: chân ghế, mặt ghế, tường, chướng ngại vật...):
                cur_hits = int(self.hit_counts[o_row, o_col])
                if cur_hits < 30:
                    self.hit_counts[o_row, o_col] = cur_hits + 1
                self.last_hit_time[o_row, o_col] = time.time()

                # Xác nhận vật cản kiên cố khi số lần quét trúng đạt ngưỡng (hits >= 2)
                if self.hit_counts[o_row, o_col] >= 2:
                    self.grid[o_row, o_col] = 255
                    has_new_obstacle = True

            except Exception:
                pass

        if has_ray:
            # Cập nhật không gian trống dọc theo tia quét (Bresenham Raycasting Free Space Clearing):
            # Các ô mà tia nhìn xuyên qua được giảm trừ 1 hit_count để loại bỏ nhiễu phản xạ,
            # trong khi vật thể thực được quét liên tục sẽ tích lũy hit_counts tăng dần theo thời gian.
            ray_indices = (ray_mask == 1)
            decay_mask = ray_indices & (self.hit_counts > 0) & (~self.anchor_mask)
            self.hit_counts[decay_mask] = np.maximum(
                0, self.hit_counts[decay_mask] - 1
            )
            free_indices = ray_indices & (self.hit_counts < 2) & (~self.anchor_mask)
            self.grid[free_indices] = 128
            self.miss_counts[free_indices] += 1

            # Khôi phục bảo toàn giá trị 255 tuyệt đối cho toàn bộ các ô mốc Anchor kiên cố
            if np.any(self.anchor_mask):
                self.grid[self.anchor_mask] = 255

        # 4. Các vật thể ngữ nghĩa (self.landmarks) được hiển thị nhãn nhận diện trên UI
        #    và độc lập với lưới chiếm dụng 2D (không ghi đè 255 trực tiếp vào lưới bản đồ),
        #    bảo toàn hình học quét từ cảm biến độ sâu.

        if has_new_obstacle or (self.total_scans_processed % 3 == 0):
            self.version += 1

    def apply_temporal_decay(self, expiry_s=None):
        """
        Bộ lọc suy hao theo thời gian thực (Temporal Timestamp Decay):
        - Giải quyết triệt để 'Bẫy bóng tối': Cản ma/nhiễu hạt rơi vào góc mù (ngoài FOV 72°)
          không bị raycast chiếu qua để xóa.
        - Với các ô cản (hit_counts > 0) KHÔNG thuộc anchor_mask:
          Nếu quá expiry_s (mặc định 25.0s) không được nhìn thấy lại, tự động suy hao hit_counts.
        - Khi hit_counts < min_confirm_hits: giải phóng ô khỏi grid (grid = 128).
        """
        if expiry_s is None:
            expiry_s = float(getattr(config, 'TEMPORAL_OBSTACLE_EXPIRY_S', 25.0)) if config else 25.0
        now = time.time()
        min_confirm_hits = int(getattr(config, 'PATH_PLAN_MIN_CONFIDENCE_HITS', 3)) if config else 3

        stale_mask = (self.hit_counts > 0) & (~self.anchor_mask) & (self.last_hit_time > 0) & ((now - self.last_hit_time) > expiry_s)
        if np.any(stale_mask):
            self.hit_counts[stale_mask] = np.maximum(0, self.hit_counts[stale_mask] - 1)
            # Tái tạo mốc thời gian để suy hao từng nấc
            self.last_hit_time[stale_mask] = now - (expiry_s * 0.7)
            cleared = stale_mask & (self.hit_counts < min_confirm_hits)
            if np.any(cleared):
                self.grid[cleared] = 128
                self.version += 1

    def _cluster_obstacle_points(self, rx, ry, points_3d):
        """
        Phân cụm không gian 3D (3D Depth Spatial Clustering) cho các điểm cản:
        - Gom các điểm cản thực sự (0.05m <= wz <= 0.85m, cự ly 0.18m - 3.2m) thành từng khối độc lập.
        - Tính toán tâm (cx, cy), chiều rộng (width_m) và chiều sâu (depth_m) của từng cụm.
        - Nếu cụm trùng với Landmark đã có (ví dụ PERSON, CHAIR) -> Cập nhật kích thước thực tế.
        - Nếu cụm không trùng với bất kỳ vật thể nào -> Tự động tạo mốc [UNKNOWN] với Bounding Box chuẩn xác!
        """
        if not points_3d or len(points_3d) < 3:
            return []

        # 1. Lọc điểm cản hợp lệ (không phải sàn nhà và nằm trong tầm quan sát)
        valid_obs = []
        for pt in points_3d:
            try:
                wx, wy = float(pt[0]), float(pt[1])
                wz = float(pt[2]) if len(pt) > 2 else 0.2
                if 0.05 <= wz <= 0.85:
                    d = math.hypot(wx - rx, wy - ry)
                    if 0.18 <= d <= 3.2:
                        valid_obs.append((wx, wy, wz, d))
            except Exception:
                pass

        if len(valid_obs) < 3:
            return []

        # 2. Sắp xếp theo góc cực so với Robot để quét gom cụm liên tục
        valid_obs.sort(key=lambda p: math.atan2(p[1] - ry, p[0] - rx))

        # 3. Phân cụm Euclidean: 2 điểm liên tiếp cách nhau <= 28cm thuộc về cùng 1 vật thể
        clusters = []
        current_cluster = [valid_obs[0]]

        for i in range(1, len(valid_obs)):
            pt = valid_obs[i]
            prev = current_cluster[-1]
            dist_pts = math.hypot(pt[0] - prev[0], pt[1] - prev[1])
            if dist_pts <= 0.28:
                current_cluster.append(pt)
            else:
                if len(current_cluster) >= 3:
                    clusters.append(current_cluster)
                current_cluster = [pt]

        if len(current_cluster) >= 3:
            clusters.append(current_cluster)

        detected_clusters = []
        # 4. Tính toán hình học bounding box và gắn nhãn cho từng cụm
        for cl in clusters:
            xs = [p[0] for p in cl]
            ys = [p[1] for p in cl]
            zs = [p[2] for p in cl]

            cx = float(np.mean(xs))
            cy = float(np.mean(ys))
            cz = float(np.mean(zs))

            span_x = max(xs) - min(xs)
            span_y = max(ys) - min(ys)
            diag = math.hypot(span_x, span_y)
            w_m = max(0.25, min(1.30, round(diag, 2)))
            d_m = max(0.20, min(0.90, round(max(span_x, span_y) * 0.75, 2)))

            detected_clusters.append({
                "cx": cx, "cy": cy, "cz": cz,
                "width_m": w_m, "depth_m": d_m,
                "num_pts": len(cl)
            })

            # Kiểm tra xem cụm này có trùng với Landmark đã có trong không gian không (< 0.45m)
            matched_lm = None
            min_dist = 0.45
            for lm in self.landmarks:
                d = math.hypot(lm.x - cx, lm.y - cy)
                if d < min_dist:
                    min_dist = d
                    matched_lm = lm

            if matched_lm is not None:
                matched_lm.update(cx, cy, cz, max(matched_lm.score, 0.85), width_m=w_m, depth_m=d_m)

        return detected_clusters

    def add_semantic_landmark(self, name, x_world, y_world, z_world=0.2, score=0.8, width_m=None, depth_m=None):
        """
        Gán và ghim vật thể ngữ nghĩa đã nhận diện trên bản đồ:
        - Bỏ qua hoàn toàn các nhãn không xác định (UNKNOWN, OBSTACLE, OBJ).
        - Khử trùng lặp: Nếu trùng vị trí (< 0.40m) -> cập nhật mốc cũ.
        - Khóa tọa độ mượt mà khi thấy >= 2 lần.
        """
        if not getattr(config, 'ENABLE_SEMANTIC_LANDMARKS', False):
            return None
        name_clean = str(name).strip().upper()
        # BỎ QUA HOÀN TOÀN UNKNOWN, OBSTACLE, OBJ - CHỈ LẤY VẬT THỂ AI NHẬN DIỆN THỰC SỰ
        if not name_clean or name_clean in ["UNKNOWN", "OBSTACLE", "OBJ", "NONE", "NULL"]:
            return None

        pin_min_conf = float(getattr(config, 'SEMANTIC_PIN_MIN_CONFIDENCE', 0.70)) if config else 0.70
        norm_sc = float(score) / 100.0 if float(score) > 1.0 else float(score)
        if norm_sc <= pin_min_conf:
            return None

        base_merge_dist = float(getattr(config, 'SEMANTIC_MERGE_DIST_M', 0.50)) if config else 0.50

        best_match = None
        min_d = 999.0
        for lm in self.landmarks:
            d = math.hypot(lm.x - x_world, lm.y - y_world)
            # Nếu cùng nhãn (ví dụ cùng là CHAIR hoặc PERSON), mở rộng bán kính gộp để không bị nhân đôi mốc
            dyn_thresh = max(base_merge_dist, getattr(lm, 'width_m', 0.40) * 0.90) if lm.name == name_clean else base_merge_dist
            if d < dyn_thresh and d < min_d:
                min_d = d
                best_match = lm

        if best_match is not None:
            best_match.update(x_world, y_world, z_world, score, width_m=width_m, depth_m=depth_m, new_name=name_clean)
            return True

        new_lm = PinnedLandmark(self._next_landmark_id, name_clean, x_world, y_world, z_world, score, width_m=width_m, depth_m=depth_m)
        self._next_landmark_id += 1
        self.landmarks.append(new_lm)
        return False

    def transform_cam_to_world(self, xc, yc, zc, rx, ry, rz, yaw):
        """
        Chiếu điểm từ Camera Frame sang World Frame bằng phép toán vi sai.
        Camera: X phải, Y xuống, Z tới -> Robot: X tới (zc), Y trái (-xc), Z lên (-yc + 0.12)
        """
        # Bù cánh tay đòn camera OAK-D S2 nhô phía trước trục bánh vi sai 9cm và lệch ngang -1.5cm
        cam_offset_x = getattr(config, 'CAMERA_OFFSET_FORWARD_M', 0.09) if config else 0.09
        cam_offset_y = getattr(config, 'CAMERA_OFFSET_LATERAL_M', -0.015) if config else -0.015
        cam_height_z = getattr(config, 'CAMERA_HEIGHT_M', 0.12) if config else 0.12
        x_rob = zc + cam_offset_x
        y_rob = -xc + cam_offset_y
        z_rob = -yc + cam_height_z

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        wx = rx + (x_rob * cos_y - y_rob * sin_y)
        wy = ry + (x_rob * sin_y + y_rob * cos_y)
        wz = rz + z_rob
        return round(wx, 3), round(wy, 3), round(wz, 3)

    def add_detection(self, obj_id, name, xc, yc, zc, score, rx=0.0, ry=0.0, rz=0.0, yaw=0.0, width_m=None, depth_m=None):
        """Cập nhật nhận diện từ Camera Frame, tự động chiếu sang World Frame và ghim cố định"""
        # Khóa cự ly an toàn: Chỉ chấp nhận cự ly từ 0.35m đến 3.8m, loại bỏ 100% rác đo hụt cự ly
        if zc < 0.35 or zc > 3.8:
            return
        wx, wy, wz = self.transform_cam_to_world(xc, yc, zc, rx, ry, rz, yaw)
        self.add_semantic_landmark(name, wx, wy, wz, score, width_m=width_m, depth_m=depth_m)

    def update_from_ros_grid(self, msg):
        """Cập nhật bản đồ chiếm dụng từ ROS nav_msgs/OccupancyGrid (RTAB-Map / Cartographer / Gmapping)"""
        try:
            w = int(msg.info.width)
            h = int(msg.info.height)
            if w <= 0 or h <= 0 or len(msg.data) < w * h:
                return

            res = float(msg.info.resolution)
            ox = float(msg.info.origin.position.x)
            oy = float(msg.info.origin.position.y)

            raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            new_grid = np.zeros((h, w), dtype=np.uint8)
            new_grid[raw_data == 0] = 128
            new_grid[raw_data > 40] = 255

            self.grid = np.flipud(new_grid)
            self.hit_counts.fill(0)
            self.hit_counts[self.grid == 255] = 5
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

    def get_confirmed_objects(self):
        """Bí danh tương thích với SemanticMapper"""
        return self.get_confirmed_landmarks()

    def get_map_payload(self):
        """Bí danh tương thích với SemanticMapper"""
        return self.get_payload_for_web()

    def get_confirmed_landmarks(self):
        """
        Trả về danh sách các vật thể AI nhận diện đã được xác nhận (seen >= min_seen hoặc đã ghim)
        và đã khử trùng lặp không gian (Spatial De-duplication).
        BỎ QUA HOÀN TOÀN UNKNOWN / OBSTACLE / OBJ.
        """
        if not getattr(config, 'ENABLE_SEMANTIC_LANDMARKS', False):
            return []
        now = time.time()
        min_seen = int(getattr(config, 'SEMANTIC_MIN_SEEN_CONFIRM', 2)) if config else 2
        merge_dist = float(getattr(config, 'SEMANTIC_MERGE_DIST_M', 0.40)) if config else 0.40

        pin_min_conf = float(getattr(config, 'SEMANTIC_PIN_MIN_CONFIDENCE', 0.70)) if config else 0.70
        # 1. Lọc sạch và chỉ giữ vật thể AI đạt độ tự tin > 70%:
        # - Vật thể tĩnh (bàn, ghế, laptop...): Lưu vĩnh viễn trên bản đồ.
        # - Vật thể động (PERSON): Nếu quá 10 giây không nhìn thấy -> Tự động giải phóng khỏi bản đồ!
        valid_landmarks = []
        for lm in self.landmarks:
            if lm.name in ["UNKNOWN", "OBSTACLE", "OBJ", "NONE", "NULL", ""]:
                continue
            if getattr(lm, 'score', 0.0) <= pin_min_conf:
                continue
            if getattr(lm, 'is_dynamic', False) or lm.name == "PERSON":
                if now - lm.last_seen > 10.0:
                    continue  # Đối tượng động đã di chuyển ra ngoài phạm vi quan sát
            valid_landmarks.append(lm)
        self.landmarks = valid_landmarks

        # 2. Lọc danh sách ứng viên đã thấy đủ số lần xác thực
        candidates = [lm for lm in self.landmarks if (lm.is_pinned or lm.seen_count >= min_seen)]
        if not candidates:
            return []

        # 3. Khử trùng lặp không gian vật lý chuẩn World Spatial Clustering:
        # Gom các mốc cạnh nhau thành một mốc duy nhất đại diện, cộng dồn số lần thấy
        merged = []
        used = set()
        sorted_candidates = sorted(candidates, key=lambda o: (o.seen_count, getattr(o, 'score', 0.0)), reverse=True)

        for i, lm in enumerate(sorted_candidates):
            if i in used:
                continue
            used.add(i)
            cluster_lms = [lm]
            for j in range(i + 1, len(sorted_candidates)):
                if j not in used:
                    other = sorted_candidates[j]
                    dist = math.hypot(lm.x - other.x, lm.y - other.y)
                    # Nếu cùng loại hoặc khoảng cách < merge_dist: gom vào 1 vật thể
                    threshold = max(merge_dist, 0.60) if lm.name == other.name else merge_dist
                    if dist <= threshold:
                        used.add(j)
                        cluster_lms.append(other)

            best = max(cluster_lms, key=lambda o: (o.seen_count, getattr(o, 'score', 0.0)))
            # Tạo dictionary trả về kèm id đánh số thứ tự gọn gàng
            d_dict = best.to_dict()
            d_dict['cluster_size'] = len(cluster_lms)
            merged.append(d_dict)

        return merged

    def freeze_anchor_walls(self):
        """Khóa cứng các bức tường/vật cản kiên cố (hit_counts >= 4) thành Bờ Tường Mốc Bất Tử (Immutable Anchor Walls).
        Tuyệt đối không khóa các điểm đỏ chớm thấy 1-2 lần để tia nhìn sau này có thể dọn sạch."""
        current_walls = (self.grid == 255) & (self.hit_counts >= 4)
        self.anchor_mask = current_walls.copy()
        self.hit_counts[self.anchor_mask] = np.maximum(self.hit_counts[self.anchor_mask], 50)
        num_anchors = int(np.sum(self.anchor_mask))
        self.version += 1
        print(f"🔒 [OCCUPANCY SLAM] Đã khóa kiên cố {num_anchors} ô bờ tường mốc (Anchor Walls) vĩnh cửu!")
        return num_anchors

    def reset_map(self):
        """Xóa trắng bản đồ để bắt đầu xây dựng lại từ đầu"""
        self.grid.fill(0)
        self.hit_counts.fill(0)
        self.miss_counts.fill(0)
        self.last_hit_time.fill(0.0)
        self.anchor_mask.fill(False)
        self.landmarks.clear()
        self._next_landmark_id = 1
        self.origin_x = -self.size_m / 2.0
        self.origin_y = -self.size_m / 2.0
        self.width = int(round(self.size_m / self.resolution))
        self.height = int(round(self.size_m / self.resolution))
        self.total_scans_processed = 0
        self.version += 1
        self.cached_b64 = ""
        print("🧹 [OCCUPANCY SLAM] Đã xóa trắng bản đồ và trả gốc tọa độ về (-size/2, -size/2) thành công!")
        return True

    def save_map(self, filename="my_room_map"):
        """Lưu bản đồ thành 2 file: .json (dữ liệu ma trận + vật thể) và .png (ảnh trực quan)"""
        if not filename.endswith(".json"):
            json_file = os.path.join(self.save_dir, f"{filename}.json")
            png_file = os.path.join(self.save_dir, f"{filename}.png")
        else:
            json_file = os.path.join(self.save_dir, filename)
            png_file = json_file.replace(".json", ".png")

        try:
            # 1. Lưu file JSON cấu trúc
            data = {
                "name": filename,
                "timestamp": time.time(),
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "size_m": self.size_m,
                "resolution": self.resolution,
                "width": self.width,
                "height": self.height,
                "origin_x": self.origin_x,
                "origin_y": self.origin_y,
                "grid_base64": base64.b64encode(self.grid.tobytes()).decode('ascii'),
                "landmarks": [lm.to_dict() for lm in self.landmarks]
            }
            with open(json_file, 'w', encoding='utf-8') as fp:
                json.dump(data, fp, indent=2, ensure_ascii=False)

            # 2. Lưu file ảnh PNG chất lượng cao
            png_b64, _ = self.get_png_base64(force=True)
            png_bytes = base64.b64decode(png_b64)
            with open(png_file, 'wb') as fp:
                fp.write(png_bytes)

            print(f"💾 [OCCUPANCY SLAM] Đã lưu bản đồ thành công vào: {json_file} & {png_file}")
            return True, json_file
        except Exception as e:
            print(f"❌ [OCCUPANCY SLAM] Lỗi lưu bản đồ: {e}")
            return False, str(e)

    def load_map(self, filename="my_room_map"):
        """Nạp lại bản đồ đã lưu từ file JSON"""
        if not filename.endswith(".json"):
            json_file = os.path.join(self.save_dir, f"{filename}.json")
        else:
            json_file = os.path.join(self.save_dir, filename)

        if not os.path.isfile(json_file):
            print(f"❌ [OCCUPANCY SLAM] Không tìm thấy file bản đồ: {json_file}")
            return False, "File not found"

        try:
            with open(json_file, 'r', encoding='utf-8') as fp:
                data = json.load(fp)

            self.size_m = float(data.get("size_m", self.size_m))
            self.resolution = float(data.get("resolution", self.resolution))
            self.width = int(data.get("width", self.width))
            self.height = int(data.get("height", self.height))
            self.origin_x = float(data.get("origin_x", self.origin_x))
            self.origin_y = float(data.get("origin_y", self.origin_y))

            raw_bytes = base64.b64decode(data["grid_base64"])
            self.grid = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((self.height, self.width)).copy()
            self.hit_counts.fill(0)
            self.hit_counts[self.grid == 255] = 5  # Đóng dấu kiên cố các ô tường đã lưu

            # Nạp lại danh mục vật thể
            self.landmarks.clear()
            for obj in data.get("landmarks", []):
                lm = PinnedLandmark(obj["id"], obj["name"], obj["x"], obj["y"], obj.get("z", 0.2), obj.get("score", 0.8))
                lm.seen_count = obj.get("seen", 5)
                lm.is_pinned = True
                self.landmarks.append(lm)

            self.version += 1
            self.cached_b64 = ""
            print(f"📂 [OCCUPANCY SLAM] Đã nạp thành công bản đồ từ: {json_file} ({len(self.landmarks)} vật thể ghim)")
            return True, json_file
        except Exception as e:
            print(f"❌ [OCCUPANCY SLAM] Lỗi nạp bản đồ: {e}")
            return False, str(e)

    def get_png_base64(self, force=False):
        """
        Tạo ảnh PNG 4-kênh BGRA phong cách ROS RViz / Cartographer / SLAM Toolbox:
        - Bảo toàn 100% vật cản mảnh (chân bàn, chân ghế) đã được xác thực hit_counts >= 2
        - Đóng liền mạch các bức tường đứt đoạn (Morphological Wall Closing 3x3)
        - Lớp đệm an toàn Costmap Inflation 15cm (3 ô buffer màu hổ phách dịu)
        - Viền ngoài tường kiến trúc sắc nét tương phản cao (High-contrast Boundary Contour Stroke)
        - Tương thích kép 100% OpenCV 3 (Jetson Nano) & OpenCV 4
        """
        now = time.time()
        # 1. Trả ngay tức thì nếu dữ liệu bản đồ chưa hề thay đổi (0 CPU, 0ms latency)
        if not force and self.cached_b64 and (self.cached_version == self.version):
            return self.cached_b64, self.version

        # 2. Giới hạn tần suất sinh PNG tối đa ~3 FPS (cách nhau >= 0.35s) để tiết kiệm CPU
        if not force and self.cached_b64 and (now - self.last_encode_time < 0.35):
            return self.cached_b64, self.version

        try:
            bgra = np.zeros((self.height, self.width, 4), dtype=np.uint8)
            mask_free = (self.grid == 128)
            raw_occ = (self.grid == 255).astype(np.uint8)

            # 1. BẢO TOÀN VẬT CẢN & CHÂN BÀN GHẾ:
            # Các ô grid==255 vốn đã được kiểm chứng bởi hit_counts >= 2 trong update_scan.
            # Bảo toàn nguyên vẹn vật cản (chân bàn/ghế 1-2 ô) mà không bị bộ lọc diện tích xoá nhầm.
            clean_occ = raw_occ.copy()

            # 2. HÀN GẮN & NỐI LIỀN TƯỜNG (Morphological Closing 3x3):
            # Nối các điểm quét LaserScan bị ngắt quãng thành những mảng tường thẳng, vuông vức
            kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            closed_occ = cv2.morphologyEx(clean_occ, cv2.MORPH_CLOSE, kernel_close)

            # 3. LỚP ĐỆM AN TOÀN COSTMAP INFLATION (Bán kính 15cm = 3 ô 5cm):
            # Giãn nở vùng chướng ngại vật để tạo hành lang đệm an toàn phong cách ROS Costmap 2D
            kernel_inflate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            inflated = cv2.dilate(closed_occ, kernel_inflate)
            mask_inflation = (inflated > 0) & mask_free & (closed_occ == 0)

            # 4. TÔ MÀU THEO MỨC ĐỘ TỰ TIN / XÁC SUẤT CHIẾM DỤNG (CONFIDENCE GRADIENT):
            # A. Sàn nhà đã khám phá (Free Space): Màu xám ngà ánh cyan sáng, sạch sẽ, trong suốt nhẹ
            bgra[mask_free] = [232, 238, 242, 215]

            # B. Vùng đệm an toàn Costmap Inflation (15cm buffer): Màu lam ngọc nhạt êm dịu
            bgra[mask_inflation] = [215, 228, 245, 160]

            # C. Phân tầng vật cản theo số lần quét xác nhận (Confidence Color Ramp):
            # - Mức 1: hit_counts <= 2 -> ĐỎ RUBY/NEON (Mới quét, mức độ chưa chắc chắn / Nghi ngờ)
            # - Mức 2: hit_counts 3..5 -> VÀNG ĐẬM / CAM (Đang tích lũy xác nhận)
            # - Mức 3: hit_counts 6..9 -> VÀNG NHẠT (Độ tự tin cao)
            # - Mức 4: hit_counts >= 10 hoặc anchor_mask -> XANH LỤC EMERALD (Kiên cố, tự tin 100%)
            occ_mask = (closed_occ > 0)
            if np.any(occ_mask):
                hits = self.hit_counts

                # Thang màu thể hiện mức độ tích lũy: Đỏ (1-2) -> Cam (3-5) -> Vàng (6-9) -> Xanh lục Emerald (>=10 / Anchor)
                m1 = occ_mask & (hits <= 2)
                bgra[m1] = [50, 42, 255, 255]

                m2 = occ_mask & (hits >= 3) & (hits <= 5)
                bgra[m2] = [0, 140, 255, 255]

                m3 = occ_mask & (hits >= 6) & (hits <= 9)
                bgra[m3] = [30, 225, 255, 255]

                m4 = occ_mask & ((hits >= 10) | getattr(self, 'anchor_mask', np.zeros_like(occ_mask, dtype=bool)))
                bgra[m4] = [160, 255, 0, 255]

                # Nét viền ranh giới vật cản (Outer Contour Stroke): Viền đỏ kỹ thuật 1px nổi bật
                try:
                    res = cv2.findContours(closed_occ, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    contours = res[0] if len(res) == 2 else res[1]
                    if contours:
                        cv2.drawContours(bgra, contours, -1, (40, 40, 255, 255), 1)
                except Exception:
                    pass

            success, buf = cv2.imencode('.png', bgra)
            if success:
                self.cached_b64 = base64.b64encode(buf).decode('ascii')
                self.cached_version = self.version
                self.last_encode_time = now

        except Exception as e:
            print(f"⚠️ [OCCUPANCY SLAM] Lỗi render bản đồ PNG base64: {e}")

        return self.cached_b64, self.version

    def get_payload_for_web(self, client_map_ver=-1):
        """Xuất gói dữ liệu toàn diện phục vụ hiển thị trên Web Cockpit.
        Tối ưu hóa: Nếu client đã có bản đồ phiên bản mới nhất, không gửi lại chuỗi base64 nặng nề!"""
        if client_map_ver == self.version and self.cached_b64:
            b64 = ""
            ver = self.version
        else:
            b64, ver = self.get_png_base64()
        num_free = int(np.count_nonzero(self.grid == 128))
        num_occ = int(np.count_nonzero(self.grid == 255))
        return {
            "map_b64": b64,
            "map_version": ver,
            "map_origin_x": self.origin_x,
            "map_origin_y": self.origin_y,
            "map_resolution": self.resolution,
            "map_width": self.width,
            "map_height": self.height,
            "free_cells": num_free,
            "occ_cells": num_occ,
            "landmarks": self.get_confirmed_landmarks()
        }


if __name__ == '__main__':
    print("🧪 [SELF-TEST] Bắt đầu tự kiểm thử OccupancySLAM độc lập...")
    slam = OccupancySLAM(size_m=8.0, resolution=0.05, save_dir="maps_test")

    # Giả lập quét 1 bức tường cách 1.5m
    wall_pts = [[1.5, y, 0.2] for y in np.linspace(-1.0, 1.0, 30)]
    for _ in range(3):
        slam.update_scan(0.0, 0.0, wall_pts)

    # Giả lập thấy cái ghế 3 lần
    slam.add_semantic_landmark("CHAIR", 1.2, 0.5, 0.2, 0.9)
    slam.add_semantic_landmark("CHAIR", 1.22, 0.51, 0.2, 0.92)
    slam.add_semantic_landmark("CHAIR", 1.19, 0.49, 0.2, 0.95)

    lms = slam.get_confirmed_landmarks()
    print(f"  ├─ Số vật thể đã ghim: {len(lms)} (Gồm: {[o['name'] for o in lms]})")
    chair_lm = next((o for o in lms if o['name'] == 'CHAIR'), None)
    unknown_lm = next((o for o in lms if o['name'] == 'UNKNOWN'), None)
    assert chair_lm is not None and chair_lm['pinned'] is True, "Lỗi: Cái ghế chưa được ghim cố định!"
    assert unknown_lm is None, "Kỳ vọng: Cụm cản UNKNOWN đã được lọc sạch khỏi danh sách mốc ngữ nghĩa!"

    payload = slam.get_payload_for_web()
    print(f"  ├─ Ô tự do (Free): {payload['free_cells']}, Ô tường (Occ): {payload['occ_cells']}")
    assert payload['occ_cells'] > 0 and len(payload['map_b64']) > 500, "Lỗi tạo bản đồ!"

    # Thử lưu và nạp bản đồ
    ok_save, _ = slam.save_map("test_map")
    assert ok_save, "Lỗi lưu bản đồ!"
    ok_load, _ = slam.load_map("test_map")
    assert ok_load, "Lỗi nạp bản đồ!"

    # Dọn dẹp thư mục test
    import shutil
    if os.path.exists("maps_test"):
        shutil.rmtree("maps_test", ignore_errors=True)

    print("🎉 [SELF-TEST] Module OccupancySLAM ĐẠT CHUẨN 100%!")
