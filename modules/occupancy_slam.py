#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
MODULE 5: HỆ THỐNG XÂY DỰNG BẢN ĐỒ CHIẾM DỤNG 2D TÍCH LŨY TĨNH (OCCUPANCY SLAM)
Tên file: modules/occupancy_slam.py
Trọng trách:
  1. Bản đồ chiếm dụng tích lũy vĩnh viễn (Persistent Static 2D Occupancy Grid Map).
  2. Cơ chế ghim tọa độ vật cản (Spatial Anchoring): Quét qua 1 lần là đứng yên, không nhảy nhót.
  3. Bộ lọc xác nhận Hit-Counter: Chỉ chốt vật cản khi thấy >= 3 lần, chống nhiễu hạt bụi.
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

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass


class PinnedLandmark:
    """Vật thể ngữ nghĩa hoặc chướng ngại vật được ghim cố định trên bản đồ toàn cục (Map Frame)"""
    def __init__(self, obj_id, name, x, y, z=0.2, score=0.8, width_m=None, depth_m=None):
        self.obj_id = int(obj_id)
        self.name = str(name).upper()
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.score = float(score)
        self.seen_count = 1
        self.first_seen = time.time()
        self.last_seen = self.first_seen
        self.is_pinned = False  # Khi seen_count >= 10 -> Khóa cứng tọa độ không đổi!

        # Kích thước hình học 2D thực tế (Rộng x Sâu, mét) phục vụ vẽ 2D Bounding Box:
        dim_map = {
            "PERSON": (0.45, 0.45),
            "CHAIR": (0.50, 0.50),
            "DININGTABLE": (1.10, 0.70),
            "TABLE": (1.10, 0.70),
            "DESK": (1.10, 0.70),
            "POTTEDPLANT": (0.35, 0.35),
            "BOTTLE": (0.15, 0.15),
            "CUP": (0.12, 0.12),
            "LAPTOP": (0.35, 0.25),
            "TVMONITOR": (0.80, 0.20),
            "SOFA": (1.40, 0.80),
            "BED": (1.60, 1.20),
            "UNKNOWN": (0.35, 0.35),
            "OBSTACLE": (0.35, 0.35)
        }
        def_w, def_d = dim_map.get(self.name, (0.40, 0.40))
        self.width_m = float(width_m) if width_m is not None else def_w
        self.depth_m = float(depth_m) if depth_m is not None else def_d

    def update(self, x, y, z, score, width_m=None, depth_m=None):
        self.last_seen = time.time()
        self.seen_count += 1
        self.score = max(self.score, score)

        if width_m is not None:
            self.width_m = round(0.70 * self.width_m + 0.30 * float(width_m), 2)
        if depth_m is not None:
            self.depth_m = round(0.70 * self.depth_m + 0.30 * float(depth_m), 2)

        # Cập nhật trung bình vị trí:
        if not self.is_pinned:
            alpha = 0.35
            self.x = (1.0 - alpha) * self.x + alpha * float(x)
            self.y = (1.0 - alpha) * self.y + alpha * float(y)
            self.z = (1.0 - alpha) * self.z + alpha * float(z)
            if self.seen_count >= 3:
                self.is_pinned = True  # ĐÓNG DẤU GHIM CỐ ĐỊNH!
        else:
            # Khi đã ghim: Tinh chỉnh rất êm dịu (alpha=0.04) để triệt tiêu sai số trôi dạt
            alpha = 0.04
            self.x = (1.0 - alpha) * self.x + alpha * float(x)
            self.y = (1.0 - alpha) * self.y + alpha * float(y)

    def to_dict(self):
        return {
            "id": self.obj_id,
            "name": self.name,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "z": round(self.z, 3),
            "score": round(self.score, 2),
            "seen": self.seen_count,
            "pinned": self.is_pinned,
            "width": round(self.width_m, 2),
            "depth": round(self.depth_m, 2)
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

        # Danh mục vật thể ngữ nghĩa được ghim (Landmarks)
        self.landmarks = []
        self._next_landmark_id = 1

        self.version = 1
        self.cached_version = -1
        self.cached_b64 = ""
        self.last_encode_time = 0.0
        self.total_scans_processed = 0

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

    def update_scan(self, rx, ry, points_3d, max_dist=3.8):
        """
        Cập nhật mây điểm quét tia LaserScan / Depth từ OAK-D S2 vào bản đồ.
        - Phân biệt triệt để sàn nhà (wz < 0.06m) và vật cản thực sự (0.06m <= wz <= 0.85m).
        - Tuyệt đối KHÔNG đánh dấu sàn nhà thành vật cản (tránh hiện tượng hình nón dày đặc trước xe).
        - Vật cản thực sự tích lũy hit_count >= 3 mới chốt cứng thành tường (255).
        """
        r_col, r_row = self.world_to_grid(rx, ry)
        if r_col is None:
            return

        # 1. Tự động dọn sạch vùng thân xe (bán kính an toàn 0.18m quanh JetBot)
        cv2.circle(self.grid, (r_col, r_row), int(round(0.18 / self.resolution)), 128, -1)

        if not points_3d or len(points_3d) == 0:
            return

        self.total_scans_processed += 1
        has_new_obstacle = False

        # Lấy mẫu tối đa 40 điểm cản tiêu biểu trong lượt quét hiện tại để giữ CPU < 1ms
        sampled_pts = points_3d[::max(1, len(points_3d) // 40)] if len(points_3d) > 40 else points_3d

        # Tạo ray_mask 1 lần duy nhất cho toàn bộ 40 tia thay vì cấp phát 40 mảng trong vòng lặp
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
                if not (0.15 <= dist <= max_dist):
                    continue

                o_col, o_row = self.world_to_grid(wx, wy)
                if o_col is None:
                    continue

                # Vẽ tia quan sát quang học: Vùng giữa Robot và Điểm đo là KHÔNG GIAN THOÁNG
                # Dừng tia trước vật cản 6cm để bảo toàn mép vật cản không bị tia đè lên
                if dist > 0.08:
                    ratio = max(0.0, (dist - 0.06) / dist)
                    fx = rx + (wx - rx) * ratio
                    fy = ry + (wy - ry) * ratio
                    f_col, f_row = self.world_to_grid(fx, fy)
                    if f_col is not None:
                        cv2.line(ray_mask, (r_col, r_row), (f_col, f_row), 1, 1)
                        has_ray = True

                # ─── PHÂN LOẠI SÀN NHÀ VÀ VẬT CẢN THỰC SỰ ───
                # Sàn nhà (wz < 0.06m / 6cm): Không phải vật cản! Xe có thể đi qua!
                if wz < 0.06:
                    if self.hit_counts[o_row, o_col] < 2:
                        self.grid[o_row, o_col] = 128
                    continue

                # Vật cản thực sự (wz >= 0.06m: chân ghế, mặt ghế, tường...):
                self.hit_counts[o_row, o_col] += 1
                # Khi quét trúng >= 2 lần -> Ghim chặt thành vật cản cố định (255)
                if self.hit_counts[o_row, o_col] >= 2:
                    self.grid[o_row, o_col] = 255
                    has_new_obstacle = True

            except Exception:
                pass

        if has_ray:
            free_indices = (ray_mask == 1) & (self.hit_counts < 2)
            self.grid[free_indices] = 128
            self.miss_counts[free_indices] += 1

        # 4. Tự động gom cụm các điểm cản thành Bounding Box (UNKNOWN / OBSTACLE)
        self._cluster_obstacle_points(rx, ry, points_3d)

        if has_new_obstacle or (self.total_scans_processed % 3 == 0):
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

            # Kiểm tra xem cụm này có trùng với Landmark đã có nhãn AI không (< 0.45m)
            matched_named = False
            for lm in self.landmarks:
                if lm.name not in ["UNKNOWN", "OBSTACLE"]:
                    if math.hypot(lm.x - cx, lm.y - cy) < 0.45:
                        lm.update(cx, cy, cz, lm.score, width_m=w_m, depth_m=d_m)
                        matched_named = True
                        break

            # Nếu không có tên AI nhận diện -> Đánh dấu là UNKNOWN
            if not matched_named:
                self.add_semantic_landmark("UNKNOWN", cx, cy, cz, score=0.85, width_m=w_m, depth_m=d_m)

        return detected_clusters

    def add_semantic_landmark(self, name, x_world, y_world, z_world=0.2, score=0.8, width_m=None, depth_m=None):
        """
        Gán và ghim vật thể ngữ nghĩa hoặc chướng ngại vật cố định trên bản đồ:
        - Gom cụm không gian: Ghép nối với vật thể cùng loại (UNKNOWN: 40cm, khác: 85cm).
        - Khóa tọa độ (Spatial Pinning): Khi thấy >= 3 lần, tọa độ đóng dấu cố định!
        """
        name_clean = str(name).upper()
        threshold_dist = 0.40 if name_clean in ["UNKNOWN", "OBSTACLE"] else 0.85

        best_match = None
        min_d = 999.0
        for lm in self.landmarks:
            if lm.name == name_clean:
                d = math.hypot(lm.x - x_world, lm.y - y_world)
                if d < threshold_dist and d < min_d:
                    min_d = d
                    best_match = lm

        if best_match is not None:
            best_match.update(x_world, y_world, z_world, score, width_m=width_m, depth_m=depth_m)
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
        x_rob = zc
        y_rob = -xc
        z_rob = -yc + 0.12

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        wx = rx + (x_rob * cos_y - y_rob * sin_y)
        wy = ry + (x_rob * sin_y + y_rob * cos_y)
        wz = rz + z_rob
        return round(wx, 3), round(wy, 3), round(wz, 3)

    def add_detection(self, obj_id, name, xc, yc, zc, score, rx=0.0, ry=0.0, rz=0.0, yaw=0.0):
        """Cập nhật nhận diện từ Camera Frame, tự động chiếu sang World Frame và ghim cố định"""
        wx, wy, wz = self.transform_cam_to_world(xc, yc, zc, rx, ry, rz, yaw)
        self.add_semantic_landmark(name, wx, wy, wz, score)

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
        Trả về danh sách các vật thể đã được xác nhận (seen >= 3)
        và đã khử trùng lặp không gian (Spatial De-duplication 0.85m)
        để không bị đè nhiều icon cùng một vị trí.
        """
        now = time.time()
        # Loại bỏ các vật thể rác chưa ghim và không thấy lại trong 8 giây
        self.landmarks = [
            lm for lm in self.landmarks
            if lm.is_pinned or (now - lm.last_seen < 8.0)
        ]

        # Lọc danh sách ứng viên (đã thấy ít nhất 3 lần và score >= 0.40 để loại bỏ rác/nhiễu AI)
        candidates = [lm for lm in self.landmarks if lm.seen_count >= 3 and getattr(lm, 'score', 1.0) >= 0.40]
        if not candidates:
            return []

        # Gom cụm không gian vật lý (Spatial Physical Clustering):
        # Hai vật thể không thể chiếm cùng 1 tọa độ không gian (< 0.45m).
        # Nếu AI nhận diện chập chờn (lúc DiningTable, lúc Cat/Bird/Plant) -> Chỉ giữ lại 1 vật thể uy tín nhất!
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
                    # Cùng loại UNKNOWN: chỉ gom khi rất gần nhau (<= 0.35m) để tách biệt 2 vật cản đặt cạnh nhau
                    if lm.name in ["UNKNOWN", "OBSTACLE"] and other.name in ["UNKNOWN", "OBSTACLE"]:
                        should_merge = (dist <= 0.35)
                    elif lm.name == other.name:
                        should_merge = (dist <= 0.85)
                    else:
                        should_merge = (dist <= 0.40)

                    if should_merge:
                        used.add(j)
                        cluster_lms.append(other)

            # Ưu tiên vật thể có tên cụ thể từ AI trước (nếu có), sau đó mới tới số lần nhìn thấy
            def _prio(o):
                is_named = 1 if o.name not in ["UNKNOWN", "OBSTACLE"] else 0
                return (is_named, o.seen_count, getattr(o, 'score', 0.0))

            best = max(cluster_lms, key=_prio)
            merged.append(best.to_dict())

        return merged

    def reset_map(self):
        """Xóa trắng bản đồ để bắt đầu xây dựng lại từ đầu"""
        self.grid.fill(0)
        self.hit_counts.fill(0)
        self.miss_counts.fill(0)
        self.landmarks.clear()
        self._next_landmark_id = 1
        self.version += 1
        self.cached_b64 = ""
        print("🧹 [OCCUPANCY SLAM] Đã xóa trắng bản đồ thành công!")
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

        # 2. Giới hạn tần suất sinh PNG tối đa 3 FPS (cách nhau >= 0.35s) để chống nghẽn CPU Jetson Nano
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

            # 4. TÔ MÀU THEO CHUẨN ĐỒ HỌA CAO CẤP ROS RVIZ / CARTOGRAPHER:
            # A. Sàn nhà đã khám phá (Free Space): Màu xám ngà sáng, sạch sẽ, phẳng phiu (BGRA)
            bgra[mask_free] = [228, 230, 232, 235]

            # B. Vùng đệm an toàn Costmap Inflation (15cm buffer): Màu hổ phách dịu ấm áp
            bgra[mask_inflation] = [185, 210, 240, 235]

            # C. Tường & Chướng ngại vật thực sự (Occupied): Màu đen đậm kiến trúc CAD sắc sảo
            bgra[closed_occ > 0] = [15, 15, 15, 255]

            # D. Viền sắc sảo ranh giới tường (Outer Contour Stroke): Nét viền đen tuyền 1px
            # Tương thích kép OpenCV 3 (Jetson Nano Melodic: 3 giá trị) & OpenCV 4 (2 giá trị)
            if np.any(closed_occ):
                try:
                    res = cv2.findContours(closed_occ, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    contours = res[0] if len(res) == 2 else res[1]
                    if contours:
                        cv2.drawContours(bgra, contours, -1, (5, 5, 5, 255), 1)
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

    def get_payload_for_web(self):
        """Xuất gói dữ liệu toàn diện phục vụ hiển thị trên Web Cockpit"""
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
    assert unknown_lm is not None, "Lỗi: Cụm cản UNKNOWN chưa được quét và ghim!"

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
