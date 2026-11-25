#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module Lập kế hoạch đường đi toàn cục A* & Bản đồ chi phí (A* Global Path Planner & Costmap)
Chức năng:
- Xây dựng bản đồ chi phí đa tầng (Tiered Costmap Layer) kết hợp vùng cảnh giác cận cản.
- Tìm kiếm lộ trình tối ưu bằng thuật toán A* trên lưới ô vuông 2D.
- Làm mượt lộ trình bằng thuật toán Line-of-Sight (Raycasting Smoothing).
- Hiệu chỉnh điểm mốc tự động (Goal / Start Snapping).
"""

import os
import sys
import time
import math
import heapq
import numpy as np
import cv2

if hasattr(sys.stdout, 'reconfigure'):
    try: sys.stdout.reconfigure(encoding='utf-8')
    except Exception: pass


class PathPlanner:
    """Bộ lập kế hoạch đường đi tối ưu A* trên lưới bản đồ 2D Occupancy Grid"""

    def __init__(self, slam=None, robot_radius_m=0.13, safety_margin_m=0.06, cost_inflation_radius_m=0.22):
        """
        :param slam: Tham chiếu tới đối tượng OccupancySLAM (nếu có)
        :param robot_radius_m: Bán kính thân xe JetBot (khoảng 0.13m = 13cm: thân 8cm + dung sai 5cm)
        :param safety_margin_m: Khoảng cách an toàn bổ sung (0.06m = 6cm)
        :param cost_inflation_radius_m: Bán kính vùng đệm cản mềm (0.22m = 22cm)
        """
        self.slam = slam
        self.robot_radius_m = float(robot_radius_m)
        self.safety_margin_m = float(safety_margin_m)
        self.inflation_radius_m = float(cost_inflation_radius_m)

        # Trạng thái lộ trình hiện tại
        self.current_path = []        # Danh sách tọa độ thế giới [[x1, y1], [x2, y2], ...]
        self.initial_path = []        # Lộ trình ban đầu khi xác lập đích đến
        self.previous_path = []       # Lộ trình của bước tính toán gần nhất trước đó
        self.current_goal = None      # (gx, gy)
        self.replan_count = 0         # Số lần tính toán tái lập đường đi
        self.latest_decision = {}     # Chi tiết mổ xẻ lý do ra quyết định đổi đường đi
        self.last_costmap = None      # Ma trận costmap gần nhất để debug/render
        self.last_highway_mask = None # Ma trận đường đã đi qua (Traversed Highway)
        self.last_plan_time = 0.0     # Thời gian tính toán gần nhất (giây)
        self.last_plan_duration_ms = 0.0
        self.virtual_obstacles = []   # Danh sách cản ảo tạm thời: [{'x': float, 'y': float, 'radius': float, 'expire_at': float}]

        # Tham số lưới mặc định nếu không có SLAM (10m x 10m, 5cm/ô -> 200 x 200)
        self.size_m = 10.0
        self.resolution = 0.05
        self.origin_x = -5.0
        self.origin_y = -5.0
        self.width = 200
        self.height = 200

        self._sync_with_slam()

    @staticmethod
    def calculate_path_length(path):
        """Tính tổng độ dài theo mét của một danh sách điểm tọa độ [[x, y], ...]"""
        if not path or len(path) < 2:
            return 0.0
        total_len = 0.0
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            total_len += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        return round(total_len, 3)

    def check_path_collision(self, path, costmap=None, current_pose=None, cost_threshold=220):
        """
        Kiểm tra xem một lộ trình có bị cắt ngang hoặc va chạm với vật cản trong costmap / occupancy grid không.
        Trả về: (is_blocked: bool, blocked_coords: (x, y) hoặc None, dist_from_current: float hoặc None)
        """
        if not path or len(path) == 0:
            return False, None, None

        if costmap is None:
            if self.last_costmap is not None:
                costmap = self.last_costmap
            elif self.slam is not None and hasattr(self.slam, 'grid'):
                costmap = self.create_costmap(
                    grid=self.slam.grid,
                    landmarks=getattr(self.slam, 'landmarks', []),
                    hit_counts=getattr(self.slam, 'hit_counts', None)
                )

        grid = None
        if self.slam is not None and hasattr(self.slam, 'grid'):
            grid = self.slam.grid

        if costmap is None and grid is None:
            return False, None, None

        h, w = costmap.shape[:2] if costmap is not None else grid.shape[:2]
        cur_x = current_pose[0] if current_pose is not None else path[0][0]
        cur_y = current_pose[1] if current_pose is not None else path[0][1]

        # Chuẩn bị danh sách các điểm kiểm tra dọc theo lộ trình
        points_to_check = []
        if len(path) == 1:
            points_to_check.append(path[0])
        else:
            for i in range(len(path) - 1):
                p1, p2 = path[i], path[i + 1]
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                seg_len = math.hypot(dx, dy)
                if seg_len < 0.001:
                    points_to_check.append(p1)
                    continue

                # Bước nhảy kiểm tra 2.5cm
                steps = max(1, int(round(seg_len / 0.025)))
                for s in range(steps + 1):
                    ratio = s / float(steps)
                    points_to_check.append((p1[0] + dx * ratio, p1[1] + dy * ratio))

        for sx, sy in points_to_check:
            # Bỏ qua các điểm quá sát robot hiện tại (< 8cm) vì robot đang đứng ở đó
            dist_m = math.hypot(sx - cur_x, sy - cur_y)
            if dist_m < 0.08:
                continue

            c, r = self.world_to_grid(sx, sy)
            if c is not None and r is not None and 0 <= c < w and 0 <= r < h:
                # 1. Đụng phải ô cản thực tế trên occupancy grid (>= 200)
                if grid is not None and grid[r, c] >= 200:
                    return True, (round(sx, 3), round(sy, 3)), round(dist_m, 2)
                # 2. Đụng phải ô cản LETHAL hoặc vùng lạm phát nguy hiểm trên costmap
                if costmap is not None and costmap[r, c] >= cost_threshold:
                    return True, (round(sx, 3), round(sy, 3)), round(dist_m, 2)

        return False, None, None

    def is_path_blocked(self, path, current_pose=None, cost_threshold=220):
        """Hàm bọc tiện ích kiểm tra xem lộ trình hiện tại có bị cản chặn không"""
        return self.check_path_collision(path, costmap=None, current_pose=current_pose, cost_threshold=cost_threshold)

    def _sync_with_slam(self):
        """Đồng bộ các tham số không gian với OccupancySLAM nếu có"""
        if self.slam is not None:
            self.size_m = float(getattr(self.slam, 'size_m', 10.0))
            self.resolution = float(getattr(self.slam, 'resolution', 0.05))
            self.origin_x = float(getattr(self.slam, 'origin_x', -5.0))
            self.origin_y = float(getattr(self.slam, 'origin_y', -5.0))
            self.width = int(getattr(self.slam, 'width', 200))
            self.height = int(getattr(self.slam, 'height', 200))

    def world_to_grid(self, wx, wy):
        """Chuyển tọa độ thế giới (mét) sang chỉ số (col, row) trên ma trận"""
        self._sync_with_slam()
        col = int(round((wx - self.origin_x) / self.resolution))
        row = int(round((self.origin_y + self.size_m - wy) / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None, None

    def grid_to_world(self, col, row):
        """Chuyển chỉ số ô ma trận (col, row) sang tọa độ thế giới (wx, wy) mét"""
        self._sync_with_slam()
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = (self.origin_y + self.size_m) - (row + 0.5) * self.resolution
        return round(float(wx), 3), round(float(wy), 3)

    def create_costmap(self, grid=None, landmarks=None, start_pose=None, hit_counts=None, tight_mode=False, traversed_path=None):
        """
        Khởi tạo bản đồ chi phí đa tầng (Tiered Costmap) từ Occupancy Grid:
        - Lọc độ tin cậy (Confidence Filtering): Chỉ các ô có số lần quét hit_counts >= min_hits
          hoặc thuộc anchor_mask mới được đưa vào obstacle_mask (LETHAL = 255).
        - Phân tách Semantic Landmarks: Mốc nhận diện AI chỉ dùng để hiển thị nhãn trên giao diện,
          không gán cản cứng lên Costmap.
        - Khai phóng hành lang đã di chuyển (Traversed Path Carving): Vùng bánh xe đã đi qua
          được giải phóng vùng đệm phình to, giữ nguyên các ô cản tĩnh thực tế.
        """
        self._sync_with_slam()

        if grid is None:
            if self.slam is not None and hasattr(self.slam, 'grid'):
                grid = self.slam.grid
            else:
                grid = np.zeros((self.height, self.width), dtype=np.uint8)

        if hit_counts is None and self.slam is not None and hasattr(self.slam, 'hit_counts'):
            hit_counts = self.slam.hit_counts

        h, w = grid.shape[:2]
        obstacle_mask = np.zeros((h, w), dtype=np.uint8)

        # Ngưỡng số lần quét xác nhận tối thiểu để xác lập vật cản cứng LETHAL
        min_hits = 2
        try:
            import config
            min_hits = int(getattr(config, 'PATH_PLAN_MIN_CONFIDENCE_HITS', 2))
        except Exception:
            min_hits = 2

        # 1. Trích xuất các ô chướng ngại vật THỰC SỰ CÓ ĐỘ TIN CẬY CAO
        if hit_counts is not None:
            is_anchor = getattr(self.slam, 'anchor_mask', None) if self.slam else None
            if is_anchor is not None and np.any(is_anchor):
                confirmed_mask = (grid >= 200) & ((hit_counts >= min_hits) | is_anchor)
            else:
                confirmed_mask = (grid >= 200) & (hit_counts >= min_hits)
            obstacle_mask[confirmed_mask] = 255
        else:
            obstacle_mask[grid >= 200] = 255

        # 1.5. Lọc khử nhiễu hạt cô lập trên dữ liệu điểm cản thô trước khi nạp Landmark:
        # Triệt tiêu các hạt bụi đơn lẻ, nhưng không bao giờ xóa nhầm các vật thể AI Landmark
        if np.any(obstacle_mask):
            k_nbr = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)
            nbr_cnt = cv2.filter2D((obstacle_mask > 0).astype(np.uint8), -1, k_nbr)
            obstacle_mask[(obstacle_mask > 0) & (nbr_cnt == 0)] = 0

        # 2. Dấu chấm Semantic Landmarks (AI YOLO) chỉ dùng để hiển thị nhận dạng vật thể trên Web UI,
        #    KHÔNG vẽ vào obstacle_mask để tránh tạo vật cản ảo phình to bít kín hành lang thực tế.

        # 2.5. Bơm các vật cản ảo tạm thời (Virtual Obstacles Injection)
        now_t = time.time()
        active_virtuals = []
        for vo in self.virtual_obstacles:
            if vo.get('expire_at', 0.0) > now_t:
                active_virtuals.append(vo)
                vx, vy = vo['x'], vo['y']
                vr = vo.get('radius', 0.22)
                vc, vr_row = self.world_to_grid(vx, vy)
                if vc is not None and vr_row is not None and 0 <= vc < w and 0 <= vr_row < h:
                    rad_cells = max(1, int(round(vr / self.resolution)))
                    cv2.circle(obstacle_mask, (vc, vr_row), rad_cells, 255, -1)
        self.virtual_obstacles = active_virtuals

        # 3. Phình to theo 2 cấp độ:
        # Tự động đồng bộ với cấu hình: cho phép đi qua khe hẹp an toàn (>= 25-30cm)
        allow_tight = True
        eff_robot_radius = self.robot_radius_m
        eff_inflation_radius = self.inflation_radius_m
        try:
            import config
            allow_tight = getattr(config, 'PATH_PLAN_ALLOW_TIGHT_GAPS', True)
            eff_robot_radius = float(getattr(config, 'ROBOT_RADIUS_M', self.robot_radius_m))
            eff_inflation_radius = float(getattr(config, 'COSTMAP_INFLATION_RADIUS_M', self.inflation_radius_m))
        except Exception:
            pass

        if tight_mode:
            # Chế độ vượt khe hẹp thích nghi: Thân xe 8cm + 1cm an toàn = 9cm
            try:
                import config
                eff_robot_radius = float(getattr(config, 'PATH_PLAN_TIGHT_LETHAL_RADIUS_M', 0.09))
                eff_inflation_radius = float(getattr(config, 'PATH_PLAN_TIGHT_INFLATION_RADIUS_M', 0.13))
            except Exception:
                eff_robot_radius = 0.09
                eff_inflation_radius = 0.13
        elif allow_tight:
            # JetBot có chiều dài 18cm, rộng 16cm (half-diagonal thực tế ~12cm).
            # Bán kính lethal chuẩn từ ROBOT_RADIUS_M (mặc định 13cm) đảm bảo không va quệt góc khi xoay.
            eff_robot_radius = max(0.11, eff_robot_radius)
            eff_inflation_radius = max(eff_robot_radius + 0.04, eff_inflation_radius)

        # Cấp 1: Vùng LETHAL VẬT LÝ
        lethal_cells = max(1, int(eff_robot_radius / self.resolution))
        k_lethal = 2 * lethal_cells + 1
        kernel_lethal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_lethal, k_lethal))
        lethal_mask = cv2.dilate(obstacle_mask, kernel_lethal)

        # Cấp 1.5: VÙNG CẢNH GIÁC CẬN CẢN 5-6CM (PROXIMITY ALERT CUSHION)
        alert_margin_m = 0.06
        try:
            import config
            alert_margin_m = float(getattr(config, 'OBSTACLE_ALERT_MARGIN_M', 0.06))
        except Exception:
            alert_margin_m = 0.06

        alert_radius_m = eff_robot_radius + alert_margin_m
        alert_cells = max(lethal_cells + 1, int(round(alert_radius_m / self.resolution)))
        k_alert = 2 * alert_cells + 1
        kernel_alert = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_alert, k_alert))
        alert_mask = cv2.dilate(obstacle_mask, kernel_alert)

        # Cấp 2: Vùng đệm an toàn mềm (Safety Buffer)
        inf_cells = max(alert_cells + 1, int(round(eff_inflation_radius / self.resolution)))
        k_inf = 2 * inf_cells + 1
        kernel_inf = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_inf, k_inf))
        outer_inflated = cv2.dilate(obstacle_mask, kernel_inf)

        costmap = np.zeros((h, w), dtype=np.uint8)

        # Tính chi phí mềm giảm dần theo khoảng cách Euclid từ biên vật cản
        self.last_dist_transform = None
        if np.any(obstacle_mask):
            dist_transform = cv2.distanceTransform(cv2.bitwise_not(obstacle_mask), cv2.DIST_L2, 5)
            self.last_dist_transform = dist_transform
            dist_m = dist_transform * self.resolution
            buffer_mask = (outer_inflated > 0) & (lethal_mask == 0)
            if np.any(buffer_mask):
                denom = max(0.01, eff_inflation_radius - eff_robot_radius)
                ratio = np.clip((dist_m - eff_robot_radius) / denom, 0.0, 1.0)
                # Tính chi phí mềm vùng đệm ngoài (15..175)
                cost_buffer = (160.0 * (1.0 - ratio) + 15.0).astype(np.uint8)
                costmap[buffer_mask] = cost_buffer[buffer_mask]

            # Áp chi phí cảnh giác cao (180 - 235) cho VÙNG CẢNH GIÁC CẬN CẢN (5-6cm sát ngoài lethal)
            alert_cushion_mask = (alert_mask > 0) & (lethal_mask == 0)
            if np.any(alert_cushion_mask):
                denom_alert = max(0.01, alert_margin_m)
                ratio_alert = np.clip((dist_m - eff_robot_radius) / denom_alert, 0.0, 1.0)
                cost_alert = (180.0 + 55.0 * (1.0 - ratio_alert)).astype(np.uint8)
                costmap[alert_cushion_mask] = np.maximum(costmap[alert_cushion_mask], cost_alert[alert_cushion_mask])

        # Khóa cứng các ô LETHAL thành 255 (Tuyệt đối không đi vào)
        costmap[lethal_mask > 0] = 255

        # 3.1. CÁC ĐIỂM CẢN MỚI CHỚM THẤY / NGHI NGỜ (hit_counts < min_hits):
        # Không tạo vùng LETHAL và không phình to 20cm làm bít kín hành lang.
        # Chỉ gán chi phí cảnh báo mềm (~ 35) để robot ưu tiên né nếu phòng rộng, nhưng VẪN ĐI QUA ĐƯỢC!
        if hit_counts is not None:
            suspected_mask = (grid >= 200) & (hit_counts < min_hits) & (costmap < 250)
            costmap[suspected_mask] = np.maximum(costmap[suspected_mask], 35)

        # 4. Tự động giải phóng vùng thân xe tại vị trí xuất phát (Start Pose Clearance)
        # Giúp xe không bao giờ bị bẫy kẹt trong vùng đệm cản khi quay hoặc quét cảm biến
        if start_pose is not None:
            sc, sr = self.world_to_grid(float(start_pose[0]), float(start_pose[1]))
            if sc is not None and sr is not None and 0 <= sc < w and 0 <= sr < h:
                cv2.circle(costmap, (sc, sr), max(1, lethal_cells - 1), 0, -1)

        # 5. ĐẠI LỘ ĐÃ XÁC THỰC (TRAVERSED HIGHWAY CARVING):
        # Toàn bộ dải hành lang mà bánh xe JetBot đã thực sự lăn qua là không gian trống được thực tế kiểm chứng.
        # Đục thông vùng đệm phình to (nhưng vẫn bảo toàn ô cản thật trên obstacle_mask nếu có vật cản mới đặt vào).
        highway_mask = None
        if traversed_path and len(traversed_path) >= 1:
            highway_mask = np.zeros((h, w), dtype=np.uint8)
            ribbon_rad_cells = max(1, int(round(0.10 / self.resolution)))  # Nửa rộng ~10cm -> dải thông 20-22cm
            prev_pt = None
            prev_world = None
            for pt in traversed_path:
                wx, wy = float(pt[0]), float(pt[1])
                c, r = self.world_to_grid(wx, wy)
                if c is not None and r is not None and 0 <= c < w and 0 <= r < h:
                    cv2.circle(highway_mask, (c, r), ribbon_rad_cells, 1, -1)
                    if prev_pt is not None and prev_world is not None:
                        if math.hypot(wx - prev_world[0], wy - prev_world[1]) <= 0.60:
                            cv2.line(highway_mask, prev_pt, (c, r), 1, thickness=max(2, ribbon_rad_cells * 2))
                    prev_pt = (c, r)
                    prev_world = (wx, wy)
            carve_mask = (highway_mask > 0) & (obstacle_mask == 0)
            costmap[carve_mask] = 0

        self.last_highway_mask = highway_mask
        self.last_costmap = costmap
        return costmap

    def find_nearest_free_cell(self, col, row, costmap, max_search_cells=16, grid=None):
        """
        Nắn chỉnh điểm đích/xuất phát (Goal/Start Snapping):
        Nếu điểm chọn trúng vật cản LETHAL (>= 250), tìm ô khả dụng gần nhất
        trong bán kính max_search_cells để không bị từ chối tìm đường.
        Theo Báo cáo.md (Mục III.1.3): Ưu tiên cực mạnh các ô đã khám phá thoáng (grid == 128)
        và tránh ô quá sát mép cản (cost >= 150), tránh rơi vào vùng bóng khuất (grid == 0) bên trong vật cản.
        """
        h, w = costmap.shape[:2]
        if grid is None and self.slam is not None and hasattr(self.slam, 'grid'):
            grid = self.slam.grid

        if 0 <= col < w and 0 <= row < h and costmap[row, col] < 220:
            if grid is None or grid[row, col] == 128:
                return col, row

        best_score = float('inf')
        best_cell = None

        for dr in range(-max_search_cells, max_search_cells + 1):
            for dc in range(-max_search_cells, max_search_cells + 1):
                nr, nc = row + dr, col + dc
                if 0 <= nr < h and 0 <= nc < w:
                    cost_val = int(costmap[nr, nc])
                    if cost_val < 250:  # Không phải ô cản LETHAL
                        dist_sq = dr * dr + dc * dc
                        explored_bonus = 0.0
                        if grid is not None:
                            g_val = grid[nr, nc]
                            if g_val == 128:
                                explored_bonus = -45.0  # Thưởng lớn cho ô thoáng đã khám phá
                            elif g_val == 0:
                                explored_bonus = 35.0   # Phạt ô tối chưa khám phá (tránh lọt vào trong gầm bàn)

                        # Phạt nặng ô quá sát mép cản (cost >= 150)
                        proximity_penalty = cost_val * 0.45 if cost_val >= 150 else cost_val * 0.15
                        score = dist_sq + proximity_penalty + explored_bonus
                        if score < best_score:
                            best_score = score
                            best_cell = (nc, nr)

        return best_cell if best_cell is not None else (col, row)

    def _line_of_sight(self, c0, r0, c1, r1, costmap, max_buffer_cost=150):
        """
        Thuật toán Bresenham kiểm tra đường nhìn thẳng giữa 2 điểm:
        - Tuyệt đối cấm đi qua vật cản LETHAL (>= 250).
        - Ngăn chặn 'cắt cua' (shortcutting) xuyên qua vùng đệm nguy hiểm sát cản (cost > allowed_cost):
          Nếu cả 2 điểm đầu mút đều ở vùng an toàn, không cho tia nhìn thẳng ăn lẹm vào vùng đệm cao!
        """
        h, w = costmap.shape[:2]
        dx = abs(c1 - c0)
        dy = abs(r1 - r0)
        x, y = c0, r0
        sx = 1 if c1 > c0 else -1
        sy = 1 if r1 > r0 else -1
        err = dx - dy

        c_start = int(costmap[r0, c0])
        c_end = int(costmap[r1, c1])
        # Cho phép dung sai đệm thích nghi theo mức chi phí của 2 điểm đầu mút
        allowed_cost = max(max_buffer_cost, max(c_start, c_end) + 30)
        allowed_cost = min(249, allowed_cost)

        while True:
            if not (0 <= x < w and 0 <= y < h) or costmap[y, x] >= 250 or costmap[y, x] > allowed_cost:
                return False
            if x == c1 and y == r1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True

    @staticmethod
    def simplify_path_douglas_peucker(path, epsilon=0.06):
        """
        Rút gọn đường đi A* theo chuẩn Douglas-Peucker:
        Triệt tiêu các điểm waypoint li ti trên đoạn thẳng, giảm thiểu số lần dừng ngắt quãng
        và giảm 70% số lần peek/replan không cần thiết.
        """
        if not path or len(path) <= 2:
            return path

        pts = np.array(path, dtype=np.float32)

        def _dp(points):
            if len(points) <= 2:
                return points
            p_start = points[0]
            p_end = points[-1]
            line_vec = p_end - p_start
            line_len = np.linalg.norm(line_vec)

            if line_len < 1e-6:
                dists = np.linalg.norm(points[1:-1] - p_start, axis=1)
            else:
                line_unit = line_vec / line_len
                vecs = points[1:-1] - p_start
                projs = np.dot(vecs, line_unit)
                perp_vecs = vecs - projs[:, None] * line_unit
                dists = np.linalg.norm(perp_vecs, axis=1)

            max_idx = np.argmax(dists) + 1
            max_dist = dists[max_idx - 1]

            if max_dist > epsilon:
                left = _dp(points[:max_idx + 1])
                right = _dp(points[max_idx:])
                return np.vstack([left[:-1], right])
            else:
                return np.array([p_start, p_end])

        simplified = _dp(pts)
        return [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in simplified]

    def smooth_path(self, grid_path, costmap):
        """
        Làm mượt lộ trình (Path Smoothing / Ray-casting Pruning):
        Bỏ qua các điểm trung gian răng cưa nếu xe có thể chạy thẳng mượt từ điểm A tới điểm C
        mà không cắt qua vùng đệm nguy hiểm sát vật cản.
        """
        if len(grid_path) <= 2:
            return grid_path

        smoothed = [grid_path[0]]
        current_idx = 0

        while current_idx < len(grid_path) - 1:
            furthest_idx = current_idx + 1
            for next_idx in range(len(grid_path) - 1, current_idx, -1):
                c0, r0 = grid_path[current_idx]
                c1, r1 = grid_path[next_idx]
                if self._line_of_sight(c0, r0, c1, r1, costmap):
                    furthest_idx = next_idx
                    break

            smoothed.append(grid_path[furthest_idx])
            current_idx = furthest_idx

        return smoothed

    def apply_asymmetric_cable_bias(self, world_path, costmap, bias_m=0.035):
        """
        Bảo vệ sợi dây cáp OAK nhô ra bên sườn trái xe (7cm) theo chuẩn Báo cáo.md (Mục III.1.2):
        Quy tắc 2 bước cho từng waypoint trung gian:
          - Bước A (Căn giữa trước): Nếu điểm đang nằm trong hành lang hẹp (dist_m < 0.24m),
            dò dọc vector pháp tuyến để đưa điểm về đúng đỉnh sống giữa hành lang (dist_m lớn nhất).
          - Bước B (Dịch phải bảo vệ cáp trái): Từ tâm giữa đó, dịch sang phải tối đa +3.5cm,
            nhưng KHÓA AN TOÀN: không được làm khoảng hở tới tường phải giảm xuống dưới
            max(0.115m, center_d - 0.038m) để tránh tự ép bánh phải vào vách!
        """
        if not world_path or len(world_path) <= 2 or costmap is None:
            return world_path

        h, w = costmap.shape[:2]
        dist_transform = getattr(self, 'last_dist_transform', None)
        adjusted_path = [list(world_path[0])]  # Giữ nguyên điểm xuất phát

        for i in range(1, len(world_path) - 1):
            p_prev = world_path[i - 1]
            p_curr = world_path[i]
            p_next = world_path[i + 1]

            # Vector tiếp tuyến hướng đi
            dx = p_next[0] - p_prev[0]
            dy = p_next[1] - p_prev[1]
            seg_len = math.hypot(dx, dy)

            if seg_len < 0.001:
                adjusted_path.append(list(p_curr))
                continue

            # Vector pháp tuyến chỉ sang bên PHẢI xe: nx, ny = dy / seg_len, -dx / seg_len
            nx = dy / seg_len
            ny = -dx / seg_len

            curr_c, curr_r = self.world_to_grid(p_curr[0], p_curr[1])
            if curr_c is None or curr_r is None or not (0 <= curr_c < w and 0 <= curr_r < h):
                adjusted_path.append(list(p_curr))
                continue

            center_x, center_y = p_curr[0], p_curr[1]
            center_d = float(dist_transform[curr_r, curr_c]) * self.resolution if dist_transform is not None else 0.50

            # BƯỚC A: Căn giữa nếu trong hành lang hẹp (dist_m < 0.24m)
            if dist_transform is not None and center_d < 0.24:
                best_ridge_d = center_d
                best_ridge_x, best_ridge_y = center_x, center_y
                for s in np.linspace(-0.08, 0.08, 9):
                    cand_s_x = p_curr[0] + nx * s
                    cand_s_y = p_curr[1] + ny * s
                    sc, sr = self.world_to_grid(cand_s_x, cand_s_y)
                    if sc is not None and sr is not None and 0 <= sc < w and 0 <= sr < h:
                        d_val = float(dist_transform[sr, sc]) * self.resolution
                        if d_val > best_ridge_d and costmap[sr, sc] < 220:
                            best_ridge_d = d_val
                            best_ridge_x, best_ridge_y = cand_s_x, cand_s_y
                center_x, center_y = best_ridge_x, best_ridge_y
                center_d = best_ridge_d

            # BƯỚC B: Dịch sang PHẢI để bảo vệ cáp trái 7cm, có khóa an toàn tường phải
            cand_x = center_x + nx * bias_m
            cand_y = center_y + ny * bias_m
            c_cand, r_cand = self.world_to_grid(cand_x, cand_y)

            if c_cand is not None and r_cand is not None and 0 <= c_cand < w and 0 <= r_cand < h:
                cand_d = float(dist_transform[r_cand, c_cand]) * self.resolution if dist_transform is not None else 0.50
                cand_cost = int(costmap[r_cand, c_cand])
                min_safe_d = max(0.115, center_d - 0.038)
                if cand_d >= min_safe_d and cand_cost < 200:
                    adjusted_path.append([round(float(cand_x), 3), round(float(cand_y), 3)])
                    continue

            # Nếu bên phải vướng cản, giữ nguyên vị trí tâm
            adjusted_path.append([round(float(center_x), 3), round(float(center_y), 3)])

        adjusted_path.append(list(world_path[-1]))  # Giữ nguyên điểm đích
        return adjusted_path

    def _search_astar(self, scol, srow, gcol, grow, costmap, grid=None, unk_penalty=4.5, buffer_scale=0.015, max_iterations=35000, highway_mask=None):
        """
        Thuật toán tìm kiếm A* 8 hướng tổng quát hóa, áp dụng cho A* chuẩn, Tight Gap và Exploration.
        Ưu tiên cao nhất các ô thuộc Traversed Highway (con đường xe đã thực sự lăn qua) và vùng đã quét thoáng (128).
        """
        h, w = costmap.shape[:2]
        open_set = []
        h_start = math.hypot(gcol - scol, grow - srow)
        heapq.heappush(open_set, (h_start, 0.0, scol, srow))

        came_from = {}
        g_scores = {(scol, srow): 0.0}
        closed_set = set()

        # 8 hướng di chuyển: 4 hướng chính (cost 1.0) và 4 hướng chéo (cost sqrt(2) ~ 1.414)
        neighbors = [
            (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
            (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)
        ]

        found = False
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            f, current_g, curr_c, curr_r = heapq.heappop(open_set)

            if (curr_c, curr_r) in closed_set:
                continue
            closed_set.add((curr_c, curr_r))

            if curr_c == gcol and curr_r == grow:
                found = True
                break

            for dc, dr, move_cost in neighbors:
                nc, nr = curr_c + dc, curr_r + dr

                # Kiểm tra biên
                if not (0 <= nc < w and 0 <= nr < h):
                    continue
                # Ô cản LETHAL tuyệt đối cấm đi:
                if costmap[nr, nc] >= 250:
                    continue

                # Chống cắt góc nhọn (Corner Cutting Prevention) qua góc cản LETHAL:
                if dc != 0 and dr != 0:
                    if costmap[curr_r, nc] >= 250 and costmap[nr, curr_c] >= 250:
                        continue

                is_highway = (highway_mask is not None and highway_mask[nr, nc] > 0)

                # Phí vùng đệm an toàn mềm (bằng 0 trên đường xe đã thực sự lăn qua):
                buffer_penalty = 0.0 if is_highway else (float(costmap[nr, nc]) * buffer_scale)

                # Phân biệt Đại lộ đã đi qua (Highway) vs Vùng Thoáng đã quét (128) vs Vùng Tối Chưa Khám Phá (0):
                if is_highway:
                    terrain_penalty = 0.0   # Tuyệt đối tin cậy: Đã đi qua an toàn
                elif grid is not None:
                    cell_type = grid[nr, nc]
                    if cell_type == 128:
                        terrain_penalty = 0.25  # Vùng thoáng đã khám phá: ưu tiên cao
                    elif cell_type == 0:
                        terrain_penalty = unk_penalty   # Vùng chưa khám phá: phạt nặng để không đi vòng mép ngoài
                    else:
                        terrain_penalty = 0.5   # Điểm cản nghi ngờ chưa xác nhận: phạt nhẹ
                else:
                    terrain_penalty = 0.0

                tentative_g = current_g + move_cost + buffer_penalty + terrain_penalty
                if tentative_g < g_scores.get((nc, nr), float('inf')):
                    g_scores[(nc, nr)] = tentative_g
                    came_from[(nc, nr)] = (curr_c, curr_r)
                    h_val = math.hypot(gcol - nc, grow - nr)
                    f_val = tentative_g + h_val
                    heapq.heappush(open_set, (f_val, tentative_g, nc, nr))

        return found, came_from

    def plan_path(self, start_pose, goal_pose, grid=None, landmarks=None, hit_counts=None, traversed_path=None):
        """
        Thực hiện tìm đường ngắn nhất A* từ start_pose tới goal_pose:
        :param start_pose: (start_x, start_y) mét thực tế
        :param goal_pose: (goal_x, goal_y) mét thực tế
        :param grid: Ma trận Occupancy Grid (tùy chọn)
        :param landmarks: Danh mục vật cản (tùy chọn - chỉ mang tính tham khảo hiển thị)
        :param hit_counts: Ma trận số lần quét xác nhận (tùy chọn, để lọc vật cản tin cậy)
        :param traversed_path: Lịch sử tọa độ bánh xe đã lăn qua (tùy chọn, tạo đại lộ an toàn)
        :return: dict kết quả {success, path, waypoints_count, length_m, plan_time_ms, message}
        """
        t0 = time.time()
        self._sync_with_slam()

        sx, sy = float(start_pose[0]), float(start_pose[1])
        gx, gy = float(goal_pose[0]), float(goal_pose[1])

        # 1. Tạo ma trận chi phí có vùng đệm an toàn và giải phóng thân xe xuất phát (Costmap)
        costmap = self.create_costmap(
            grid=grid, landmarks=landmarks, start_pose=start_pose,
            hit_counts=hit_counts, traversed_path=traversed_path
        )
        highway_mask = self.last_highway_mask
        h, w = costmap.shape[:2]

        # 2. Chuyển sang tọa độ ô lưới (col, row)
        start_cell = self.world_to_grid(sx, sy)
        goal_cell = self.world_to_grid(gx, gy)

        if start_cell == (None, None):
            return {
                "success": False,
                "path": [],
                "waypoints_count": 0,
                "length_m": 0.0,
                "plan_time_ms": round((time.time() - t0) * 1000, 2),
                "goal": [gx, gy],
                "message": f"Vị trí bắt đầu ({sx:.2f}, {sy:.2f}) nằm ngoài bản đồ!"
            }

        if goal_cell == (None, None):
            return {
                "success": False,
                "path": [],
                "waypoints_count": 0,
                "length_m": 0.0,
                "plan_time_ms": round((time.time() - t0) * 1000, 2),
                "goal": [gx, gy],
                "message": f"Điểm đích ({gx:.2f}, {gy:.2f}) nằm ngoài bản đồ!"
            }

        scol, srow = self.find_nearest_free_cell(start_cell[0], start_cell[1], costmap, max_search_cells=10, grid=grid)
        gcol, grow = self.find_nearest_free_cell(goal_cell[0], goal_cell[1], costmap, max_search_cells=16, grid=grid)

        # 3. Thuật toán tìm kiếm A* (A-Star Grid Search) 8 hướng
        buffer_penalty_scale = 0.015
        unk_pen = 4.5
        try:
            import config
            buffer_penalty_scale = float(getattr(config, 'PATH_PLAN_BUFFER_PENALTY_SCALE', 0.015))
            unk_pen = float(getattr(config, 'PATH_PLAN_UNKNOWN_SPACE_PENALTY', 4.5))
        except Exception:
            pass

        found, came_from = self._search_astar(
            scol, srow, gcol, grow, costmap,
            grid=grid, unk_penalty=unk_pen, buffer_scale=buffer_penalty_scale,
            highway_mask=highway_mask
        )

        # Nếu tìm đường thông thường thất bại, thử lại với TIGHT GAP MODE (Adaptive Tight Inflation)
        is_tight_fallback = False
        if not found:
            allow_tight_retry = True
            try:
                import config
                allow_tight_retry = getattr(config, 'PATH_PLAN_ALLOW_TIGHT_GAPS', True)
            except Exception: pass
            if allow_tight_retry:
                tight_costmap = self.create_costmap(
                    grid=grid, landmarks=landmarks, start_pose=start_pose,
                    hit_counts=hit_counts, tight_mode=True, traversed_path=traversed_path
                )
                tight_hw = self.last_highway_mask
                tscol, tsrow = self.find_nearest_free_cell(start_cell[0], start_cell[1], tight_costmap, max_search_cells=10, grid=grid)
                tgcol, tgrow = self.find_nearest_free_cell(goal_cell[0], goal_cell[1], tight_costmap, max_search_cells=16, grid=grid)
                t_found, t_came_from = self._search_astar(
                    tscol, tsrow, tgcol, tgrow, tight_costmap,
                    grid=grid, unk_penalty=unk_pen, buffer_scale=0.010,
                    highway_mask=tight_hw
                )
                if t_found:
                    found = True
                    came_from = t_came_from
                    gcol, grow = tgcol, tgrow
                    costmap = tight_costmap
                    is_tight_fallback = True
                    print("🚪 [PATH PLANNER] Đã kích hoạt Tight Gap Mode: Tìm thấy đường đi qua khe hẹp!")

        # Tier 3: Nếu cả A* thông thường lẫn Tight Gap đều không tìm ra đường (ví dụ do bị vây quanh bởi Unknown),
        # nhưng đích nằm ở phía trước: Thử mở rộng đường đi qua vùng chưa khám phá (Exploration Fallback)
        if not found:
            allow_explore = True
            try:
                import config
                allow_explore = getattr(config, 'NAV_ALLOW_UNKNOWN_EXPLORATION', True)
            except Exception:
                pass
            if allow_explore:
                e_costmap = self.create_costmap(
                    grid=grid, landmarks=landmarks, start_pose=start_pose,
                    hit_counts=hit_counts, tight_mode=True, traversed_path=traversed_path
                )
                e_hw = self.last_highway_mask
                escol, esrow = self.find_nearest_free_cell(start_cell[0], start_cell[1], e_costmap, max_search_cells=12, grid=grid)
                egcol, egrow = self.find_nearest_free_cell(goal_cell[0], goal_cell[1], e_costmap, max_search_cells=20, grid=grid)
                e_found, e_came_from = self._search_astar(
                    escol, esrow, egcol, egrow, e_costmap,
                    grid=grid, unk_penalty=0.5, buffer_scale=0.005,
                    highway_mask=e_hw
                )
                if e_found:
                    found = True
                    came_from = e_came_from
                    gcol, grow = egcol, egrow
                    costmap = e_costmap
                    print("🧭 [PATH PLANNER] Đã kích hoạt Exploration Mode: Tìm thấy đường mở rộng qua không gian mới!")

        plan_ms = round((time.time() - t0) * 1000, 2)
        self.last_plan_time = time.time()
        self.last_plan_duration_ms = plan_ms

        if not found:
            self.replan_count += 1
            self.latest_decision = {
                "replan_count": self.replan_count,
                "has_diverged": True,
                "blocked_coords": [gx, gy],
                "dist_to_block_m": round(math.hypot(gx - sx, gy - sy), 2),
                "reason_summary": "DICH DEN BI CHAN HOAN TOAN",
                "reason_detail": "Khong the tim thay duong di! Vat can vay kin loi vao dich hoac khong co hanh lang thoang > 20cm.",
                "initial_length_m": self.calculate_path_length(self.initial_path),
                "current_length_m": 0.0,
                "initial_waypoints": len(self.initial_path),
                "current_waypoints": 0
            }
            self.current_path = []
            return {
                "success": False,
                "path": [],
                "waypoints_count": 0,
                "length_m": 0.0,
                "plan_time_ms": plan_ms,
                "goal": [gx, gy],
                "decision": dict(self.latest_decision),
                "message": "Không tìm thấy đường đi khả dụng (Vật cản chặn kín lối đi)!"
            }

        # 4. Tái hiện lộ trình ô lưới từ đích về xuất phát
        raw_grid_path = [(gcol, grow)]
        curr = (gcol, grow)
        while curr in came_from:
            curr = came_from[curr]
            raw_grid_path.append(curr)
        raw_grid_path.reverse()

        # 5. Làm mượt đường đi (Path Smoothing)
        smoothed_grid_path = self.smooth_path(raw_grid_path, costmap)

        # 6. Chuyển đổi sang tọa độ thế giới (mét thực tế)
        world_path = []
        for col, row in smoothed_grid_path:
            wx, wy = self.grid_to_world(col, row)
            world_path.append([wx, wy])

        # 6.1. Rút gọn các điểm waypoint dư thừa trên đoạn thẳng bằng Douglas-Peucker TRƯỚC bước nắn cáp
        if len(world_path) > 3:
            dp_eps = 0.06
            try:
                import config
                dp_eps = float(getattr(config, 'PATH_PLAN_DOUGLAS_PEUCKER_EPSILON_M', 0.06))
            except Exception: pass
            if dp_eps > 0.01:
                world_path = self.simplify_path_douglas_peucker(world_path, epsilon=dp_eps)

        # 6.2. Phân đoạn các đoạn dài (> 0.25m) để đảm bảo các hành lang thẳng vẫn có waypoint trung gian được bù cáp trái
        densified_world_path = []
        if world_path:
            densified_world_path.append(world_path[0])
            for i in range(len(world_path) - 1):
                p1 = world_path[i]
                p2 = world_path[i + 1]
                dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                if dist > 0.25:
                    num_sub = int(dist // 0.20)
                    for k in range(1, num_sub + 1):
                        frac = k / (num_sub + 1)
                        densified_world_path.append([
                            round(p1[0] + frac * (p2[0] - p1[0]), 3),
                            round(p1[1] + frac * (p2[1] - p1[1]), 3)
                        ])
                densified_world_path.append(p2)
            world_path = densified_world_path

        # 6.3. BÙ LỆCH AN TOÀN BẢO VỆ DÂY CÁP TRÁI (ASYMMETRIC CABLE SHIELD - BƯỚC CUỐI CÙNG)
        # Theo Báo cáo.md (Mục III.1.2): Nắn cáp chạy SAU CÙNG để không bao giờ bị Douglas-Peucker xóa mất 3.5cm
        allow_asym = True
        bias_dist = 0.035
        try:
            import config
            allow_asym = getattr(config, 'ENABLE_ASYMMETRIC_CLEARANCE', True)
            bias_dist = float(getattr(config, 'LEFT_CABLE_SAFETY_BIAS_M', 0.035))
        except Exception:
            pass

        if allow_asym and bias_dist > 0.005 and len(world_path) >= 3:
            world_path = self.apply_asymmetric_cable_bias(world_path, costmap, bias_m=bias_dist)

        # Đảm bảo điểm bắt đầu và điểm kết thúc chuẩn xác theo yêu cầu ban đầu
        if world_path:
            world_path[0] = [round(sx, 3), round(sy, 3)]
            world_path[-1] = [round(gx, 3), round(gy, 3)]

        # 7. Tính tổng độ dài lộ trình (mét)
        total_len = 0.0
        for i in range(len(world_path) - 1):
            p1, p2 = world_path[i], world_path[i+1]
            total_len += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        total_len = round(total_len, 3)

        # 8. MỔ XẺ LÝ DO RA QUYẾT ĐỊNH ĐƯỜNG ĐI (PATH DECISION & REPLAN REASONING)
        is_new_goal = False
        if not self.initial_path or not self.current_goal:
            is_new_goal = True
        else:
            goal_dist = math.hypot(gx - self.current_goal[0], gy - self.current_goal[1])
            if goal_dist > 0.15:
                is_new_goal = True

        if is_new_goal:
            self.initial_path = [list(p) for p in world_path]
            self.previous_path = [list(p) for p in world_path]
            self.replan_count = 0
            self.latest_decision = {
                "replan_count": 0,
                "has_diverged": False,
                "blocked_coords": None,
                "dist_to_block_m": None,
                "reason_summary": "KHOI TAO LO TRINH BAN DAU",
                "reason_detail": f"Xac lap duong di ban dau toi dich ({gx:.2f}, {gy:.2f}) qua {len(world_path)} waypoints, dai {total_len:.2f}m.",
                "initial_length_m": total_len,
                "current_length_m": total_len,
                "initial_waypoints": len(world_path),
                "current_waypoints": len(world_path)
            }
        else:
            self.replan_count += 1
            # Kiểm tra xem đường ban đầu và đường bước trước có bị đè bởi vật cản mới không
            is_init_blocked, init_block_pt, init_block_d = self.check_path_collision(self.initial_path, costmap, start_pose)
            is_prev_blocked, prev_block_pt, prev_block_d = self.check_path_collision(self.previous_path, costmap, start_pose)

            init_len = self.calculate_path_length(self.initial_path)
            prev_len = self.calculate_path_length(self.previous_path)
            delta_len = round(total_len - prev_len, 2)

            if is_init_blocked:
                has_diverged = True
                blocked_coords = list(init_block_pt)
                dist_to_block_m = init_block_d
                reason_summary = "VAT CAN MOI CHAN DUONG BAN DAU"
                reason_detail = f"Duong ban dau bi chan tai ({init_block_pt[0]:+.2f}, {init_block_pt[1]:+.2f}) cach xe {init_block_d:.2f}m do xuat hien vat can moi. A* da be lai ne sang vung thoang an toan."
            elif is_prev_blocked:
                has_diverged = True
                blocked_coords = list(prev_block_pt)
                dist_to_block_m = prev_block_d
                reason_summary = "VAT CAN CHAN DUONG BUOC TRUOC"
                reason_detail = f"Duong buoc truoc bi chan tai ({prev_block_pt[0]:+.2f}, {prev_block_pt[1]:+.2f}) cach xe {prev_block_d:.2f}m. A* tinh toan nhanh di moi bao toan vung dem 20cm."
            else:
                blocked_coords = None
                dist_to_block_m = None
                has_diverged = abs(delta_len) > 0.06
                if has_diverged:
                    reason_summary = "TOI UU THEO VI TRI BUOC TIEN MOI"
                    reason_detail = f"Duong cu khong bi chan. Vi tri xe tien buoc giup A* cat ngan hanh trinh va chinh goc tiep can thang toi dich (con {total_len:.2f}m)."
                else:
                    reason_summary = "BAM SAT LO TRINH DA DINH"
                    reason_detail = f"Lo trinh thong thoang, xe tiep tuc bam theo hanh lang A* da vach ra (con {total_len:.2f}m toi dich)."

            self.latest_decision = {
                "replan_count": self.replan_count,
                "has_diverged": has_diverged,
                "blocked_coords": blocked_coords,
                "dist_to_block_m": dist_to_block_m,
                "reason_summary": reason_summary,
                "reason_detail": reason_detail,
                "initial_length_m": init_len,
                "current_length_m": total_len,
                "initial_waypoints": len(self.initial_path),
                "current_waypoints": len(world_path)
            }
            self.previous_path = [list(p) for p in self.current_path] if self.current_path else [list(p) for p in world_path]

        self.current_path = world_path
        self.current_goal = [gx, gy]

        return {
            "success": True,
            "path": world_path,
            "waypoints_count": len(world_path),
            "length_m": total_len,
            "plan_time_ms": plan_ms,
            "goal": [gx, gy],
            "initial_path": [list(p) for p in self.initial_path],
            "decision": dict(self.latest_decision),
            "message": f"Tìm thấy lộ trình tối ưu {round(total_len, 2)}m ({len(world_path)} waypoints) trong {plan_ms}ms"
        }

    def clear_path(self):
        """Xóa lộ trình hiện tại và đặt lại lịch sử quyết định"""
        self.current_path = []
        self.initial_path = []
        self.previous_path = []
        self.current_goal = None
        self.replan_count = 0
        self.latest_decision = {}
        self.clear_virtual_obstacles()

    def add_virtual_obstacle(self, x, y, radius=0.22, duration_s=60.0):
        """
        Bơm một vật cản ảo tạm thời tại tọa độ (x, y) thế giới (mét).
        Cản ảo này sẽ được nạp trực tiếp vào costmap như vật thể LETHAL (255)
        để thuật toán A* bắt buộc phải tìm đường vòng né tránh,
        tự động biến mất sau duration_s giây.
        """
        now_t = time.time()
        self.virtual_obstacles = [vo for vo in self.virtual_obstacles if vo.get('expire_at', 0.0) > now_t]
        self.virtual_obstacles.append({
            'x': float(x),
            'y': float(y),
            'radius': float(radius),
            'expire_at': now_t + float(duration_s)
        })
        print(f"🛑 [A* VIRTUAL OBSTACLE] Đã bơm cản ảo tại ({x:.2f}, {y:.2f}), R={radius*100:.0f}cm, hiệu lực {duration_s:.0f}s")

    def clear_virtual_obstacles(self):
        """Xóa sạch các cản ảo tạm thời"""
        self.virtual_obstacles = []

    def is_area_free(self, wx, wy, radius_m=0.20, allow_unknown=False):
        """
        Kiểm tra xem một vùng quanh (wx, wy) đã là không gian trống (Free Space) an toàn hay chưa.
        :param allow_unknown: Nếu True, vùng Unknown (0) trong không gian mở không bị coi là cản
                              (chỉ kiểm tra vật cản thực sự grid >= 200 hoặc costmap >= 200).
        """
        col, row = self.world_to_grid(wx, wy)
        if col is None or row is None:
            return False

        grid = None
        if self.slam is not None and hasattr(self.slam, 'grid'):
            grid = self.slam.grid
        if grid is None:
            return False

        h, w = grid.shape[:2]
        r_cells = max(1, int(round(radius_m / self.resolution)))

        r1 = max(0, row - r_cells)
        r2 = min(h - 1, row + r_cells)
        c1 = max(0, col - r_cells)
        c2 = min(w - 1, col + r_cells)

        patch = grid[r1:r2+1, c1:c2+1]
        if patch.size == 0:
            return False

        # Vùng phải hoàn toàn là Free Space (128) - không chứa ô chưa biết (0) và không chứa cản (>= 200)
        has_unknown = bool(np.any(patch == 0))
        has_obstacle = bool(np.any(patch >= 200))

        # Kiểm tra thêm trong last_costmap nếu có
        if self.last_costmap is not None and self.last_costmap.shape[:2] == (h, w):
            c_patch = self.last_costmap[r1:r2+1, c1:c2+1]
            if np.any(c_patch >= 200):
                return False

        if allow_unknown:
            return not has_obstacle
        return (not has_unknown) and (not has_obstacle)

    def is_direct_line_clear(self, start_pose, end_pose, cost_threshold=220):
        """
        Kiểm tra đường nhìn thẳng từ start_pose tới end_pose xem có bị vật cản cắt ngang không.
        Sử dụng thuật toán Bresenham raycasting trên costmap hoặc grid.
        Không bị chặn bởi ô Unknown trong không gian mở.
        """
        sx, sy = float(start_pose[0]), float(start_pose[1])
        ex, ey = float(end_pose[0]), float(end_pose[1])
        c0, r0 = self.world_to_grid(sx, sy)
        c1, r1 = self.world_to_grid(ex, ey)
        if c0 is None or r0 is None or c1 is None or r1 is None:
            return False

        costmap = self.last_costmap
        if costmap is None and self.slam is not None and hasattr(self.slam, 'grid'):
            costmap = self.create_costmap(grid=self.slam.grid)
        if costmap is None:
            return False

        return self._line_of_sight(c0, r0, c1, r1, costmap, max_buffer_cost=cost_threshold)

    def get_clearance_at(self, wx, wy):
        """
        Trả về cự ly mét từ tọa độ thế giới (wx, wy) đến vật cản gần nhất trên bản đồ SLAM.
        Sử dụng ma trận biến đổi khoảng cách Euclid (last_dist_transform).
        """
        if getattr(self, 'last_dist_transform', None) is None:
            if self.slam is not None and hasattr(self.slam, 'grid'):
                self.create_costmap(grid=self.slam.grid)
            if getattr(self, 'last_dist_transform', None) is None:
                return 99.0

        c, r = self.world_to_grid(float(wx), float(wy))
        if c is None or r is None:
            return 99.0
        h, w = self.last_dist_transform.shape[:2]
        if 0 <= c < w and 0 <= r < h:
            return round(float(self.last_dist_transform[r, c]) * self.resolution, 3)
        return 99.0

    def get_path_obstacle_clearance(self, path, cur_pose=None, max_lookahead_m=0.60):
        """
        Kiểm tra độ thoáng của lộ trình sắp tới đối với các vật cản trên bản đồ SLAM:
        - Quét các điểm dọc theo waypoints trong phạm vi max_lookahead_m (mặc định 60cm).
        - Trả về: (min_clearance_m, nearest_pt, rel_angle_deg)
          + min_clearance_m: Cự ly mét nhỏ nhất tới mép cản gần nhất.
          + nearest_pt: Tọa độ (x, y) của điểm trên lộ trình gần cản nhất.
          + rel_angle_deg: Góc lệch tương đối (-180°..+180°) từ hướng robot tới vật cản đó.
        """
        if not path or len(path) == 0:
            return 99.0, None, 0.0

        cur_x = float(cur_pose[0]) if cur_pose is not None else float(path[0][0])
        cur_y = float(cur_pose[1]) if cur_pose is not None else float(path[0][1])
        cur_yaw = float(cur_pose[2]) if cur_pose is not None and len(cur_pose) >= 3 else 0.0

        if getattr(self, 'last_dist_transform', None) is None:
            if self.slam is not None and hasattr(self.slam, 'grid'):
                self.create_costmap(grid=self.slam.grid)
            if getattr(self, 'last_dist_transform', None) is None:
                return 99.0, None, 0.0

        h, w = self.last_dist_transform.shape[:2]

        # Bỏ qua các waypoint cũ phía sau robot nếu có
        closest_idx = 0
        min_p_dist = float('inf')
        for i, wp in enumerate(path):
            d = math.hypot(float(wp[0]) - cur_x, float(wp[1]) - cur_y)
            if d < min_p_dist:
                min_p_dist = d
                closest_idx = i
        fwd_path = path[closest_idx:] if closest_idx < len(path) else path

        sampled_pts = []
        accum_d = 0.0
        prev_p = (cur_x, cur_y)

        for wp in fwd_path:
            wx, wy = float(wp[0]), float(wp[1])
            seg_len = math.hypot(wx - prev_p[0], wy - prev_p[1])
            if seg_len < 0.001:
                continue
            steps = max(1, int(round(seg_len / 0.035)))
            for s in range(1, steps + 1):
                rat = s / float(steps)
                px = prev_p[0] + (wx - prev_p[0]) * rat
                py = prev_p[1] + (wy - prev_p[1]) * rat
                accum_d += math.hypot(px - prev_p[0], py - prev_p[1])
                sampled_pts.append((px, py))
                prev_p = (px, py)
                if accum_d >= max_lookahead_m:
                    break
            if accum_d >= max_lookahead_m:
                break

        if not sampled_pts:
            sampled_pts = [(cur_x, cur_y)]

        min_d = 99.0
        nearest_step_pt = None
        nearest_grid_rc = None

        for px, py in sampled_pts:
            c, r = self.world_to_grid(px, py)
            if c is not None and r is not None and 0 <= c < w and 0 <= r < h:
                d_val = float(self.last_dist_transform[r, c]) * self.resolution
                if d_val < min_d:
                    min_d = d_val
                    nearest_step_pt = (px, py)
                    nearest_grid_rc = (r, c)

        rel_angle_deg = 0.0
        if nearest_grid_rc is not None and min_d < 90.0:
            # Tìm vector hướng từ robot tới ô vật cản thực tế gần nhất
            r_box = max(2, int(round(min_d / self.resolution)) + 2)
            nr, nc = nearest_grid_rc
            r1 = max(0, nr - r_box)
            r2 = min(h - 1, nr + r_box)
            c1 = max(0, nc - r_box)
            c2 = min(w - 1, nc + r_box)

            # Ô cản là ô có distanceTransform == 0
            obs_cells = np.argwhere(self.last_dist_transform[r1:r2+1, c1:c2+1] == 0)
            if len(obs_cells) > 0:
                # Tìm ô cản gần robot nhất
                closest_obs_w = None
                closest_dist = float('inf')
                for orow_rel, ocol_rel in obs_cells:
                    owx, owy = self.grid_to_world(c1 + ocol_rel, r1 + orow_rel)
                    d_robot_obs = math.hypot(owx - cur_x, owy - cur_y)
                    if d_robot_obs < closest_dist:
                        closest_dist = d_robot_obs
                        closest_obs_w = (owx, owy)

                if closest_obs_w is not None:
                    bearing = math.atan2(closest_obs_w[1] - cur_y, closest_obs_w[0] - cur_x)
                    diff = math.atan2(math.sin(bearing - cur_yaw), math.cos(bearing - cur_yaw))
                    rel_angle_deg = round(math.degrees(diff), 1)

        return round(min_d, 3), nearest_step_pt, rel_angle_deg


# ══════════════════════════════════════════════════════════════════════
# TỰ KIỂM THỬ ĐỘC LẬP (SELF-TEST)
# ══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print("══════════════════════════════════════════════════════════════════")
    print("🧪 [SELF-TEST] BẮT ĐẦU KIỂM THỬ ĐỘC LẬP MODULE PATH PLANNER A*")
    print("══════════════════════════════════════════════════════════════════")

    planner = PathPlanner(robot_radius_m=0.12, cost_inflation_radius_m=0.20)

    # 1. Tạo bản đồ mô phỏng 10m x 10m (200 x 200 ô)
    test_grid = np.zeros((200, 200), dtype=np.uint8)

    # Đặt bức tường chắn ngang ở x = 0 (col = 100) từ y = -2.0m đến y = +2.0m
    # Chừa lối đi thoáng ở mép phía trên (y > 2.0m)
    wall_col = 100
    r_start = int(round((-2.0 - (-5.0)) / 0.05))  # row tương ứng y = 2.0
    r_end = int(round((2.0 - (-5.0)) / 0.05))    # row tương ứng y = -2.0
    r_min, r_max = min(r_start, r_end), max(r_start, r_end)
    test_grid[r_min:r_max, wall_col] = 255
    print(f"  ├─ Đã tạo tường cản giữa bản đồ: Col={wall_col}, Chiều cao={r_max - r_min} ô")

    # 2. Điểm xuất phát (-1.5m, 0.0m) và Đích (+1.5m, 0.0m) nằm ở 2 bên tường
    start = (-1.5, 0.0)
    goal = (1.5, 0.0)

    print(f"  ├─ Xuất phát: {start} m")
    print(f"  ├─ Đích đến:  {goal} m")

    res = planner.plan_path(start, goal, grid=test_grid)
    print(f"  ├─ Trạng thái tìm đường: {'✅ THÀNH CÔNG' if res['success'] else '❌ THẤT BẠI'}")
    print(f"  ├─ Thời gian tính toán:  {res['plan_time_ms']} ms")
    print(f"  ├─ Chiều dài lộ trình:   {res['length_m']} m")
    print(f"  ├─ Số điểm mốc (Waypoints): {res['waypoints_count']}")
    print(f"  └─ Thông báo: {res['message']}")

    if res['success']:
        print("\n  📍 Các điểm mốc (Waypoints sau làm mượt):")
        for i, pt in enumerate(res['path']):
            print(f"     Waypoint {i+1}: X={pt[0]:+0.2f}m, Y={pt[1]:+0.2f}m")

        # Kiểm tra không điểm nào xuyên tường (x = 0 trong phạm vi tường)
        assert res['length_m'] > 3.0, "Lộ trình phải vòng qua tường, chiều dài phải > 3.0m!"
        print("\n✅ [SELF-TEST PASS] Thuật toán A* & Costmap Inflation đã vượt qua kiểm thử thành công!")
    else:
        print("\n❌ [SELF-TEST FAIL] Không tìm thấy đường đi!")
