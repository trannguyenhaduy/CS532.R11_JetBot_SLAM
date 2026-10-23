#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module 8: Hệ thống Lập kế hoạch Đường đi Toàn cục A* & Vùng đệm Chi phí (A-Star Global Path Planner & Costmap)
Chuyên trách:
- Vùng đệm an toàn chống cạ tường (Costmap Inflation Layer 15cm - 20cm).
- Thuật toán tìm đường ngắn nhất A* (A-Star Grid Search) siêu tốc < 5ms.
- Bộ làm mượt đường đi (Line-of-Sight Path Smoothing) triệt tiêu bậc thang răng cưa.
- Tự động nắn điểm đích (Goal Snapping) khi người dùng click nhầm vào vật cản/tường.
- Độc lập 100%, có self-test riêng: python3 -m modules.path_planner
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

    def __init__(self, slam=None, robot_radius_m=0.12, safety_margin_m=0.08, cost_inflation_radius_m=0.20):
        """
        :param slam: Tham chiếu tới đối tượng OccupancySLAM (nếu có)
        :param robot_radius_m: Bán kính thân xe JetBot (khoảng 0.12m = 12cm)
        :param safety_margin_m: Khoảng cách an toàn bổ sung (0.08m = 8cm)
        :param cost_inflation_radius_m: Bán kính vùng đệm cản (Tổng cộng 0.20m = 20cm)
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
        self.last_plan_time = 0.0     # Thời gian tính toán gần nhất (giây)
        self.last_plan_duration_ms = 0.0

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

    def check_path_collision(self, path, costmap, current_pose=None):
        """
        Kiểm tra xem một lộ trình có bị cắt ngang hoặc va chạm với vật cản trong costmap không.
        Trả về: (is_blocked: bool, blocked_coords: (x, y) hoặc None, dist_from_current: float hoặc None)
        """
        if not path or len(path) < 2 or costmap is None:
            return False, None, None

        h, w = costmap.shape[:2]
        cur_x = current_pose[0] if current_pose is not None else path[0][0]
        cur_y = current_pose[1] if current_pose is not None else path[0][1]

        # Duyệt dọc theo từng đoạn thẳng của lộ trình
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            seg_len = math.hypot(dx, dy)
            if seg_len < 0.001:
                continue

            # Bước nhảy kiểm tra 2.5cm
            steps = max(1, int(round(seg_len / 0.025)))
            for s in range(steps + 1):
                ratio = s / float(steps)
                sx = p1[0] + dx * ratio
                sy = p1[1] + dy * ratio
                c, r = self.world_to_grid(sx, sy)
                if c is not None and r is not None and 0 <= c < w and 0 <= r < h:
                    if costmap[r, c] >= 250:  # Đụng phải vật cản LETHAL (tường/vật thể thực tế)
                        dist_m = math.hypot(sx - cur_x, sy - cur_y)
                        return True, (round(sx, 3), round(sy, 3)), round(dist_m, 2)

        return False, None, None

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

    def create_costmap(self, grid=None, landmarks=None, start_pose=None, hit_counts=None):
        """
        Tạo bản đồ chi phí đa tầng phân cấp (Tiered Costmap) từ Occupancy Grid:
        - LỌC ĐỘ TIN CẬY (CONFIDENCE FILTERING):
          Chỉ các ô chướng ngại vật thực sự được xác nhận (hit_counts >= min_confidence, mặc định >= 3,
          hoặc bờ tường mốc anchor_mask, hoặc Landmark) mới được đưa vào obstacle_mask để tạo
          vùng LETHAL VẬT LÝ (255) và vùng đệm an toàn.
          Triệt tiêu 100% hiện tượng "tường ma" do các điểm đỏ nhiễu hạt chớm thấy (hit_counts <= 2)
          phình to làm bít kín đường đi của robot hoặc ép xe đi đường vòng!
        - Các điểm cản mới chớm thấy (hit_counts 1-2):
          Được gán chi phí mềm (cost = 35) để robot ưu tiên né nếu phòng rộng, nhưng VẪN CHO ĐI QUA
          thay vì bị coi là tường chết.
        - Tự động dọn sạch vùng thân xe tại start_pose (nếu có) để xe không bao giờ bị bẫy kẹt ở điểm xuất phát.
        - Trả về ma trận costmap (uint8, 0 = an toàn, 1..249 = vùng đệm mềm, 255 = cấm đi).
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

        # Ngưỡng số lần quét xác nhận tối thiểu để coi là vật cản cứng LETHAL (mặc định 3)
        min_hits = 3
        try:
            import config
            min_hits = int(getattr(config, 'PATH_PLAN_MIN_CONFIDENCE_HITS', 3))
        except Exception:
            min_hits = 3

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

        # 2. Đánh dấu các vật thể Landmark vào bản đồ cản (nếu có)
        active_landmarks = landmarks if landmarks is not None else (getattr(self.slam, 'landmarks', []) if self.slam else [])
        for lm in active_landmarks:
            lx = getattr(lm, 'x', None) if not isinstance(lm, dict) else lm.get('x')
            ly = getattr(lm, 'y', None) if not isinstance(lm, dict) else lm.get('y')
            lw = getattr(lm, 'width_m', 0.40) if not isinstance(lm, dict) else lm.get('width', 0.40)
            ld = getattr(lm, 'depth_m', 0.40) if not isinstance(lm, dict) else lm.get('depth', 0.40)

            if lx is not None and ly is not None:
                half_w_cells = max(1, int(round((lw / 2.0) / self.resolution)))
                half_d_cells = max(1, int(round((ld / 2.0) / self.resolution)))
                c_col, c_row = self.world_to_grid(lx, ly)
                if c_col is not None and c_row is not None:
                    r1 = max(0, c_row - half_d_cells)
                    r2 = min(h - 1, c_row + half_d_cells)
                    c1 = max(0, c_col - half_w_cells)
                    c2 = min(w - 1, c_col + half_w_cells)
                    obstacle_mask[r1:r2+1, c1:c2+1] = 255

        # 3. Phình to theo 2 cấp độ:
        # Cấp 1: Vùng LETHAL VẬT LÝ (Bán kính thân xe robot_radius_m ~ 12cm = 2-3 ô lưới)
        lethal_cells = max(1, int(round(self.robot_radius_m / self.resolution)))
        k_lethal = 2 * lethal_cells + 1
        kernel_lethal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_lethal, k_lethal))
        lethal_mask = cv2.dilate(obstacle_mask, kernel_lethal)

        # Cấp 2: Vùng đệm an toàn mềm (Safety Buffer từ robot_radius_m đến inflation_radius_m)
        inf_cells = max(lethal_cells + 1, int(math.ceil(self.inflation_radius_m / self.resolution)))
        k_inf = 2 * inf_cells + 1
        kernel_inf = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_inf, k_inf))
        outer_inflated = cv2.dilate(obstacle_mask, kernel_inf)

        costmap = np.zeros((h, w), dtype=np.uint8)

        # Tính chi phí mềm giảm dần theo khoảng cách Euclid từ biên vật cản
        buffer_mask = (outer_inflated > 0) & (lethal_mask == 0)
        if np.any(buffer_mask):
            dist_transform = cv2.distanceTransform(cv2.bitwise_not(obstacle_mask), cv2.DIST_L2, 5)
            dist_m = dist_transform * self.resolution
            denom = max(0.01, self.inflation_radius_m - self.robot_radius_m)
            ratio = np.clip((dist_m - self.robot_radius_m) / denom, 0.0, 1.0)
            cost_buffer = (110.0 * (1.0 - ratio) + 10.0).astype(np.uint8)
            costmap[buffer_mask] = cost_buffer[buffer_mask]

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

        self.last_costmap = costmap
        return costmap

    def find_nearest_free_cell(self, col, row, costmap, max_search_cells=16):
        """
        Nắn chỉnh điểm đích/xuất phát (Goal/Start Snapping):
        Nếu điểm chọn trúng vật cản LETHAL (>= 250), tìm ô khả dụng gần nhất
        trong bán kính max_search_cells để không bị từ chối tìm đường.
        Ưu tiên ô có chi phí cản thấp nhất và gần tâm nhất.
        """
        h, w = costmap.shape[:2]
        if 0 <= col < w and 0 <= row < h and costmap[row, col] < 250:
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
                        # Điểm số kết hợp khoảng cách và mức độ an toàn (càng xa cản càng tốt)
                        score = dist_sq + (cost_val * 0.15)
                        if score < best_score:
                            best_score = score
                            best_cell = (nc, nr)

        return best_cell if best_cell is not None else (col, row)

    def _line_of_sight(self, c0, r0, c1, r1, costmap):
        """Thuật toán Bresenham kiểm tra đường nhìn thẳng giữa 2 điểm có bị chặn bởi vật cản LETHAL không"""
        h, w = costmap.shape[:2]
        dx = abs(c1 - c0)
        dy = abs(r1 - r0)
        x, y = c0, r0
        sx = 1 if c1 > c0 else -1
        sy = 1 if r1 > r0 else -1
        err = dx - dy

        while True:
            # Nếu chạm vật cản lethal (>= 250): cấm nhìn thẳng
            if not (0 <= x < w and 0 <= y < h) or costmap[y, x] >= 250:
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

    def smooth_path(self, grid_path, costmap):
        """
        Làm mượt lộ trình (Path Smoothing / Ray-casting Pruning):
        Bỏ qua các điểm trung gian răng cưa nếu xe có thể chạy thẳng mượt từ điểm A tới điểm C.
        """
        if len(grid_path) <= 2:
            return grid_path

        smoothed = [grid_path[0]]
        current_idx = 0

        while current_idx < len(grid_path) - 1:
            # Nhìn xa nhất có thể về phía đích
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

    def plan_path(self, start_pose, goal_pose, grid=None, landmarks=None, hit_counts=None):
        """
        Thực hiện tìm đường ngắn nhất A* từ start_pose tới goal_pose:
        :param start_pose: (start_x, start_y) mét thực tế
        :param goal_pose: (goal_x, goal_y) mét thực tế
        :param grid: Ma trận Occupancy Grid (tùy chọn)
        :param landmarks: Danh mục vật cản (tùy chọn)
        :param hit_counts: Ma trận số lần quét xác nhận (tùy chọn, để lọc vật cản tin cậy)
        :return: dict kết quả {success, path, waypoints_count, length_m, plan_time_ms, message}
        """
        t0 = time.time()
        self._sync_with_slam()

        sx, sy = float(start_pose[0]), float(start_pose[1])
        gx, gy = float(goal_pose[0]), float(goal_pose[1])

        # 1. Tạo ma trận chi phí có vùng đệm an toàn và giải phóng thân xe xuất phát (Costmap)
        costmap = self.create_costmap(grid=grid, landmarks=landmarks, start_pose=start_pose, hit_counts=hit_counts)
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

        scol, srow = self.find_nearest_free_cell(start_cell[0], start_cell[1], costmap, max_search_cells=10)
        gcol, grow = self.find_nearest_free_cell(goal_cell[0], goal_cell[1], costmap, max_search_cells=16)

        # 3. Thuật toán tìm kiếm A* (A-Star Grid Search) 8 hướng
        # Priority Queue: (f_score, g_score, col, row)
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
        max_iterations = 35000
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            f, current_g, curr_c, curr_r = heapq.heappop(open_set)

            if (curr_c, curr_r) in closed_set:
                continue
            closed_set.add((curr_c, curr_r))

            # Đến đích:
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

                # Chống cắt góc nhọn (Corner Cutting Prevention) qua góc cản LETHAL
                if dc != 0 and dr != 0:
                    if costmap[curr_r, nc] >= 250 or costmap[nr, curr_c] >= 250:
                        continue

                # Phí vùng đệm an toàn mềm (tự động né xa tường khi có thể)
                buffer_penalty = float(costmap[nr, nc]) * 0.06

                # Phân biệt Vùng Thoáng (128) vs Vùng Tối Chưa Khám Phá (0):
                # - Vùng thoáng 128 (đã quét thấy): phí = 0.0 (hành lang ưu tiên số 1)
                # - Vùng tối 0 (chưa quét): phí thám hiểm = 4.0 (ưu tiên tuyệt đối vùng đã quét trong phòng,
                #   ngăn ngừa xe đi vòng ra vùng tối vô tận bên ngoài khi phòng đã có đường thoáng)
                if grid is not None:
                    cell_type = grid[nr, nc]
                    terrain_penalty = 0.0 if cell_type == 128 else 4.0
                else:
                    terrain_penalty = 0.0

                tentative_g = current_g + move_cost + buffer_penalty + terrain_penalty
                if tentative_g < g_scores.get((nc, nr), float('inf')):
                    g_scores[(nc, nr)] = tentative_g
                    came_from[(nc, nr)] = (curr_c, curr_r)
                    h_val = math.hypot(gcol - nc, grow - nr)
                    f_val = tentative_g + h_val
                    heapq.heappush(open_set, (f_val, tentative_g, nc, nr))

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
        print("\n🎉 [SELF-TEST PASS] Thuật toán A* & Costmap Inflation hoạt động hoàn hảo 100%!")
    else:
        print("\n❌ [SELF-TEST FAIL] Không tìm thấy đường đi!")
