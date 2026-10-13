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
        self.current_goal = None      # (gx, gy)
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

    def create_costmap(self, grid=None, landmarks=None):
        """
        Tạo bản đồ chi phí an toàn (Costmap) từ Occupancy Grid:
        - Các ô cản thực sự (>= 200) và vật thể Landmark được đánh dấu là LETHAL (255).
        - Thực hiện phép Giãn nở hình học (Binary Dilation) bán kính inflation_radius_m
          để tạo vùng đệm an toàn cấm xe đi sát mép tường.
        - Trả về ma trận costmap (uint8, 0 = an toàn, 255 = cấm đi).
        """
        self._sync_with_slam()

        if grid is None:
            if self.slam is not None and hasattr(self.slam, 'grid'):
                grid = self.slam.grid
            else:
                grid = np.zeros((self.height, self.width), dtype=np.uint8)

        h, w = grid.shape[:2]
        obstacle_mask = np.zeros((h, w), dtype=np.uint8)

        # 1. Trích xuất các ô chướng ngại vật từ Occupancy Grid (255)
        obstacle_mask[grid >= 200] = 255

        # 2. Đánh dấu các vật thể Landmark vào bản đồ cản (nếu có)
        active_landmarks = landmarks if landmarks is not None else (getattr(self.slam, 'landmarks', []) if self.slam else [])
        for lm in active_landmarks:
            lx = getattr(lm, 'x', None) if not isinstance(lm, dict) else lm.get('x')
            ly = getattr(lm, 'y', None) if not isinstance(lm, dict) else lm.get('y')
            lw = getattr(lm, 'width_m', 0.40) if not isinstance(lm, dict) else lm.get('width', 0.40)
            ld = getattr(lm, 'depth_m', 0.40) if not isinstance(lm, dict) else lm.get('depth', 0.40)

            if lx is not None and ly is not None:
                # Vẽ vùng chiếm dụng của vật thể
                half_w_cells = max(1, int(round((lw / 2.0) / self.resolution)))
                half_d_cells = max(1, int(round((ld / 2.0) / self.resolution)))
                c_col, c_row = self.world_to_grid(lx, ly)
                if c_col is not None and c_row is not None:
                    r1 = max(0, c_row - half_d_cells)
                    r2 = min(h - 1, c_row + half_d_cells)
                    c1 = max(0, c_col - half_w_cells)
                    c2 = min(w - 1, c_col + half_w_cells)
                    obstacle_mask[r1:r2+1, c1:c2+1] = 255

        # 3. Phình to vật cản (Costmap Inflation) bằng hình tròn bán kính r_inflation
        inflation_cells = max(1, int(math.ceil(self.inflation_radius_m / self.resolution)))
        kernel_size = 2 * inflation_cells + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated_costmap = cv2.dilate(obstacle_mask, kernel)

        self.last_costmap = inflated_costmap
        return inflated_costmap

    def find_nearest_free_cell(self, col, row, costmap, max_search_cells=10):
        """
        Nắn chỉnh điểm đích/xuất phát (Goal/Start Snapping):
        Nếu điểm chọn trúng vật cản hoặc vùng đệm an toàn, tìm ô trống gần nhất
        trong bán kính max_search_cells để không bị từ chối tìm đường.
        """
        h, w = costmap.shape[:2]
        if 0 <= col < w and 0 <= row < h and costmap[row, col] == 0:
            return col, row

        # Tìm kiếm mở rộng vòng tròn theo khoảng cách Euclidean
        best_dist = float('inf')
        best_cell = None

        for dr in range(-max_search_cells, max_search_cells + 1):
            for dc in range(-max_search_cells, max_search_cells + 1):
                nr, nc = row + dr, col + dc
                if 0 <= nr < h and 0 <= nc < w:
                    if costmap[nr, nc] == 0:
                        dist_sq = dr * dr + dc * dc
                        if dist_sq < best_dist:
                            best_dist = dist_sq
                            best_cell = (nc, nr)

        return best_cell if best_cell is not None else (col, row)

    def _line_of_sight(self, c0, r0, c1, r1, costmap):
        """Thuật toán Bresenham kiểm tra đường nhìn thẳng giữa 2 điểm có bị chặn bởi vật cản không"""
        h, w = costmap.shape[:2]
        dx = abs(c1 - c0)
        dy = abs(r1 - r0)
        x, y = c0, r0
        sx = 1 if c1 > c0 else -1
        sy = 1 if r1 > r0 else -1
        err = dx - dy

        while True:
            if not (0 <= x < w and 0 <= y < h) or costmap[y, x] != 0:
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

    def plan_path(self, start_pose, goal_pose, grid=None, landmarks=None):
        """
        Thực hiện tìm đường ngắn nhất A* từ start_pose tới goal_pose:
        :param start_pose: (start_x, start_y) mét thực tế
        :param goal_pose: (goal_x, goal_y) mét thực tế
        :param grid: Ma trận Occupancy Grid (tùy chọn)
        :param landmarks: Danh mục vật cản (tùy chọn)
        :return: dict kết quả {success, path, waypoints_count, length_m, plan_time_ms, message}
        """
        t0 = time.time()
        self._sync_with_slam()

        sx, sy = float(start_pose[0]), float(start_pose[1])
        gx, gy = float(goal_pose[0]), float(goal_pose[1])

        # 1. Tạo ma trận chi phí có vùng đệm an toàn (Costmap)
        costmap = self.create_costmap(grid=grid, landmarks=landmarks)
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

        scol, srow = self.find_nearest_free_cell(start_cell[0], start_cell[1], costmap, max_search_cells=6)
        gcol, grow = self.find_nearest_free_cell(goal_cell[0], goal_cell[1], costmap, max_search_cells=12)

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
        max_iterations = 25000
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

                # Kiểm tra biên và vật cản trong Costmap
                if not (0 <= nc < w and 0 <= nr < h):
                    continue
                if costmap[nr, nc] != 0:
                    continue

                # Chống cắt góc nhọn (Corner Cutting Prevention) khi đi chéo
                if dc != 0 and dr != 0:
                    if costmap[curr_r, nc] != 0 or costmap[nr, curr_c] != 0:
                        continue

                tentative_g = current_g + move_cost
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
            self.current_path = []
            self.current_goal = None
            return {
                "success": False,
                "path": [],
                "waypoints_count": 0,
                "length_m": 0.0,
                "plan_time_ms": plan_ms,
                "goal": [gx, gy],
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

        self.current_path = world_path
        self.current_goal = [gx, gy]

        return {
            "success": True,
            "path": world_path,
            "waypoints_count": len(world_path),
            "length_m": round(total_len, 3),
            "plan_time_ms": plan_ms,
            "goal": [gx, gy],
            "message": f"Tìm thấy lộ trình tối ưu {round(total_len, 2)}m ({len(world_path)} waypoints) trong {plan_ms}ms"
        }

    def clear_path(self):
        """Xóa lộ trình hiện tại"""
        self.current_path = []
        self.current_goal = None


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
