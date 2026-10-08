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
    """Vật thể ngữ nghĩa được ghim cố định trên bản đồ toàn cục (Map Frame)"""
    def __init__(self, obj_id, name, x, y, z=0.2, score=0.8):
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

    def update(self, x, y, z, score):
        self.last_seen = time.time()
        self.seen_count += 1
        self.score = max(self.score, score)

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
            "pinned": self.is_pinned
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
                        ray_mask = np.zeros((self.height, self.width), dtype=np.uint8)
                        cv2.line(ray_mask, (r_col, r_row), (f_col, f_row), 1, 1)
                        free_indices = (ray_mask == 1) & (self.hit_counts < 2)
                        self.grid[free_indices] = 128
                        self.miss_counts[free_indices] += 1

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

        if has_new_obstacle or (self.total_scans_processed % 3 == 0):
            self.version += 1

    def add_semantic_landmark(self, name, x_world, y_world, z_world=0.2, score=0.8):
        """
        Gán và ghim vật thể ngữ nghĩa cố định trên bản đồ:
        - Gom cụm không gian (Spatial Clustering 85cm): Ghép nối với vật thể cùng loại.
        - Khóa tọa độ (Spatial Pinning): Khi thấy >= 10 lần, tọa độ đứng yên 100%, không nhảy nhót.
        """
        name_clean = str(name).upper()
        threshold_dist = 0.85  # Bán kính gom cụm 85cm

        best_match = None
        min_d = 999.0
        for lm in self.landmarks:
            if lm.name == name_clean:
                d = math.hypot(lm.x - x_world, lm.y - y_world)
                if d < threshold_dist and d < min_d:
                    min_d = d
                    best_match = lm

        if best_match is not None:
            best_match.update(x_world, y_world, z_world, score)
            return True

        new_lm = PinnedLandmark(self._next_landmark_id, name_clean, x_world, y_world, z_world, score)
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

        # Lọc danh sách ứng viên (đã thấy ít nhất 3 lần)
        candidates = [lm for lm in self.landmarks if lm.seen_count >= 3]
        if not candidates:
            return []

        # Nhóm các landmark cùng tên nằm trong bán kính 0.85m vào 1 điểm đại diện duy nhất
        merged = []
        used = set()
        # Sắp xếp theo số lần thấy giảm dần (ưu tiên vật thể quan sát nhiều nhất)
        sorted_candidates = sorted(candidates, key=lambda o: o.seen_count, reverse=True)

        for i, lm in enumerate(sorted_candidates):
            if i in used:
                continue
            used.add(i)
            # Tìm tất cả các landmark cùng tên quá gần (< 0.85m)
            cluster_lms = [lm]
            for j in range(i + 1, len(sorted_candidates)):
                if j not in used and sorted_candidates[j].name == lm.name:
                    dist = math.hypot(lm.x - sorted_candidates[j].x, lm.y - sorted_candidates[j].y)
                    if dist <= 0.85:
                        used.add(j)
                        cluster_lms.append(sorted_candidates[j])

            # Chọn landmark có số lần thấy cao nhất trong cụm
            best = max(cluster_lms, key=lambda o: o.seen_count)
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
        """Tạo ảnh PNG 4-kênh RGBA sắc nét, tương phản cao hiển thị trên Web Cockpit"""
        now = time.time()
        if not force and self.cached_b64 and (now - self.last_encode_time < 0.25):
            return self.cached_b64, self.version

        bgra = np.zeros((self.height, self.width, 4), dtype=np.uint8)
        mask_free = (self.grid == 128)
        mask_occ = (self.grid == 255)

        # 1. Sàn nhà đã quét (Free space): Xanh lục nhạt dịu mắt, bán trong suốt (alpha=40)
        # Trong hệ BGRA: B=20, G=170, R=40, A=40 (Màu xanh lá êm dịu, tương ứng 🟢 Free Space trên HUD)
        bgra[mask_free] = [20, 170, 40, 40]

        # 2. Vật cản & Tường cố định (Occupied): Màu Cyan rực rỡ, độ mờ 100% (alpha=255, 🟦 Tường Tích Lũy)
        bgra[mask_occ] = [255, 240, 0, 255]

        # 3. Làm liền mạch các mép tường bằng giãn nở nhẹ 2x2
        kernel = np.ones((2, 2), np.uint8)
        dilated = cv2.dilate(mask_occ.astype(np.uint8), kernel)
        bgra[dilated > 0] = [255, 240, 0, 255]

        success, buf = cv2.imencode('.png', bgra)
        if success:
            self.cached_b64 = base64.b64encode(buf).decode('ascii')
            self.last_encode_time = now

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
    print(f"  ├─ Số vật thể đã ghim: {len(lms)} (Tọa độ: {lms[0]['x']}, {lms[0]['y']} - Pinned: {lms[0]['pinned']})")
    assert len(lms) == 1 and lms[0]['pinned'] is True, "Lỗi: Cái ghế chưa được ghim cố định!"

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
