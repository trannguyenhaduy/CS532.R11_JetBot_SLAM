#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
MODULE AUTONOMOUS NAVIGATOR & PATH FOLLOWER (NAVIGATOR.PY)
Bộ điều khiển bám đường tự hành đa điểm (Multi-Waypoint Trajectory Tracker)
Tuân thủ tiêu chuẩn Harness Robotics & Differential Drive Kinematics
══════════════════════════════════════════════════════════════════════════════
Tính năng chính:
  1. Theo dõi lộ trình A* (Waypoints Tracking) từ PathPlanner.
  2. Thuật toán Pure Pursuit 2 pha (Turn-In-Place -> Smooth Drive).
  3. Bù trễ, giới hạn gia tốc và khử trượt cho động cơ vi sai JetBot.
  4. Cơ chế an toàn Watchdog: Tự động dừng khẩn cấp khi người dùng can thiệp
     lái tay (WASD) hoặc kẹt xe quá thời gian (Stall Detection).
══════════════════════════════════════════════════════════════════════════════
"""

import sys
import math
import time
import threading
from typing import List, Tuple, Optional, Callable, Dict, Any

try:
    import config
except ImportError:
    config = None

if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
if hasattr(sys.stderr, 'reconfigure'):
    try:
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass


class AutonomousNavigator:
    """Bộ điều khiển bám đường tự hành bám danh sách Waypoints A*"""

    # Các trạng thái vận hành của Navigator
    STATE_IDLE = "IDLE"
    STATE_TURNING = "TURNING"
    STATE_DRIVING = "DRIVING"
    STATE_GOAL_REACHED = "GOAL_REACHED"
    STATE_STALLED = "STALLED"
    STATE_CANCELLED = "CANCELLED"
    STATE_OBSTACLE_BLOCKED = "OBSTACLE_BLOCKED"
    STATE_INIT_SCAN = "INIT_SCAN_360"
    STATE_OBSERVING = "OBSERVING_360"
    STATE_PEEKING = "PEEKING_SCAN"
    STATE_SWEEPING = "SWEEPING_180"
    STATE_STATIC_SCAN = "STATIC_SCAN"
    STATE_REPLANNING = "REPLANNING"
    STATE_RECOVERING = "RECOVERING"
    STATE_PROXIMITY_CAUTIOUS = "PROXIMITY_CAUTIOUS"
    STATE_PROXIMITY_LOOK_AGAIN = "PROXIMITY_LOOK_AGAIN"

    def __init__(
        self,
        get_pose_cb: Callable[[], Tuple[float, float, float]],
        set_cmd_vel_cb: Callable[[float, float], None],
        stop_cb: Callable[[], None],
        linear_speed: float = 0.12,         # m/s (~15-20% PWM an toàn trong nhà)
        turn_speed: float = 1.20,           # rad/s (~11-13% PWM quay êm)
        goal_tolerance_m: float = 0.06,     # Dung sai chạm đích (6cm)
        waypoint_tolerance_m: float = 0.10, # Dung sai lướt qua waypoint (10cm)
        heading_tolerance_rad: float = 0.35,# Dung sai góc lệch (~20 độ)
        kp_angular: float = 1.6,            # Hệ số P chỉnh lái khi tiến
        stalled_timeout_s: float = 20.0,    # Thời gian tối đa cho 1 waypoint trước khi báo kẹt
        step_move_sync_cb: Optional[Callable[..., Any]] = None,
        observe_scan_cb: Optional[Callable[..., Any]] = None,
        replan_cb: Optional[Callable[..., Any]] = None,
        cancel_step_cb: Optional[Callable[..., Any]] = None,
        mode: str = "stepped",              # "stepped" (Sense-Plan-Act) hoặc "continuous" (Pure Pursuit)
        step_dist_cm: float = 40.0,         # Cự ly mỗi bước tiến/lùi (tối đa 40cm)
        init_scan_rounds: int = 1,          # Số vòng quay 360 độ ban đầu để khởi tạo mốc bản đồ
        peek_scan_cb: Optional[Callable[..., Any]] = None,
        static_scan_cb: Optional[Callable[..., Any]] = None,
        sweep_scan_cb: Optional[Callable[..., Any]] = None,
        observe_mode: str = "smart_active", # "smart_active": chế độ quan sát chủ động kết hợp di chuyển từng bước
        peek_angle_deg: float = 40.0,       # Góc liếc trái/phải khi sắp rẽ (độ)
        peek_trigger_angle_deg: float = 30.0, # Ngưỡng góc cua để kích hoạt liếc quan sát
        peek_interval_steps: int = 4,       # Chu kỳ liếc định kỳ
        sweep_angle_deg: float = 55.0,      # Góc quét bán nguyệt 180° bù đắp vùng mù (±55°)
        sweep_interval_m: float = 0.90,     # Quãng đường thẳng tích lũy (90cm) -> quét 180°
        sweep_interval_steps: int = 3,      # Hoặc sau mỗi 3 bước thẳng không xoay
        get_clearance_cb: Optional[Callable[[], Optional[float]]] = None,
        is_area_free_cb: Optional[Callable[[float, float], bool]] = None,
        is_path_blocked_cb: Optional[Callable[..., Any]] = None,
        add_virtual_obstacle_cb: Optional[Callable[[float, float], None]] = None,
        step_max_dist_cm: float = 65.0,
        step_min_dist_cm: float = 25.0,
        dynamic_step_enabled: bool = True,
        costmap_aware_peek: bool = True,
        look_again_cb: Optional[Callable[..., Any]] = None,
        get_path_clearance_cb: Optional[Callable[..., Any]] = None,
        proximity_alert_dist_m: float = 0.26,
        proximity_step_max_cm: float = 15.0,
        proximity_peek_deg: float = 22.0,
        proximity_safe_clearance_m: float = 0.18
    ):
        self.get_pose_cb = get_pose_cb
        self.set_cmd_vel_cb = set_cmd_vel_cb
        self.stop_cb = stop_cb

        self.linear_speed = float(linear_speed)
        self.turn_speed = float(turn_speed)
        self.goal_tolerance_m = float(goal_tolerance_m)
        self.waypoint_tolerance_m = float(waypoint_tolerance_m)
        self.heading_tolerance_rad = float(heading_tolerance_rad)
        self.kp_angular = float(kp_angular)
        self.stalled_timeout_s = float(stalled_timeout_s)

        # Tham số Tự hành Từng bước (Stepped Sense-Plan-Act 35cm)
        self.step_move_sync_cb = step_move_sync_cb
        self.observe_scan_cb = observe_scan_cb
        self.replan_cb = replan_cb
        self.cancel_step_cb = cancel_step_cb
        self.mode = str(mode).lower()
        self.step_dist_cm = float(step_dist_cm)
        self.init_scan_rounds = int(init_scan_rounds)

        # Chiến thuật quan sát tối ưu (Smart Active Perception)
        self.peek_scan_cb = peek_scan_cb
        self.static_scan_cb = static_scan_cb
        self.sweep_scan_cb = sweep_scan_cb
        self.observe_mode = str(observe_mode).lower()
        self.peek_angle_deg = float(peek_angle_deg)
        self.peek_trigger_angle_deg = float(peek_trigger_angle_deg)
        self.peek_interval_steps = max(1, int(peek_interval_steps))
        self.sweep_angle_deg = float(sweep_angle_deg)
        self.sweep_interval_m = float(sweep_interval_m)
        self.sweep_interval_steps = max(1, int(sweep_interval_steps))

        # Dynamic Step Scaling & Costmap-Aware Perception
        self.get_clearance_cb = get_clearance_cb
        self.is_area_free_cb = is_area_free_cb
        self.is_path_blocked_cb = is_path_blocked_cb
        self.add_virtual_obstacle_cb = add_virtual_obstacle_cb
        self.step_max_dist_cm = float(step_max_dist_cm)
        self.step_min_dist_cm = float(step_min_dist_cm)
        self.dynamic_step_enabled = bool(dynamic_step_enabled)
        self.costmap_aware_peek = bool(costmap_aware_peek)
        self.look_again_cb = look_again_cb
        self.get_path_clearance_cb = get_path_clearance_cb
        self.proximity_alert_dist_m = float(proximity_alert_dist_m)
        self.proximity_step_max_cm = float(proximity_step_max_cm)
        self.proximity_peek_deg = float(proximity_peek_deg)
        self.proximity_safe_clearance_m = float(proximity_safe_clearance_m)
        self.last_look_again_time = 0.0

        # Trạng thái runtime
        self.is_running = False
        self.state = self.STATE_IDLE
        self.consecutive_stuck_count = 0
        self.stuck_zone_memory: List[Dict[str, Any]] = []
        self.status_text = "SẴN SÀNG"
        self.waypoints: List[Tuple[float, float]] = []
        self.goal: Optional[Tuple[float, float]] = None
        self.current_wp_idx = 0
        self.dist_to_goal = 0.0
        self.progress_pct = 0.0

        self.lock = threading.RLock()
        self._nav_thread: Optional[threading.Thread] = None
        self._cancel_requested = False

    def _record_and_check_stuck_zone(self, cur_x: float, cur_y: float) -> Tuple[bool, int]:
        """
        Ghi nhớ và kiểm tra vị trí bị kẹt lặp lại (Spatial Stuck Memory).
        - Gom cụm các điểm kẹt trong bán kính NAV_STUCK_ZONE_RADIUS_M (mặc định 0.30m).
        - Trả về (is_escalated, count): True nếu bị kẹt lặp lại >= NAV_STUCK_ZONE_ESCALATE_COUNT (2 lần).
        """
        now = time.time()
        # Dọn dẹp các điểm kẹt cũ hơn 600s
        self.stuck_zone_memory = [z for z in self.stuck_zone_memory if (now - z.get('last_time', 0.0)) < 600.0]

        radius = float(getattr(config, 'NAV_STUCK_ZONE_RADIUS_M', 0.30)) if config else 0.30
        escalate_thresh = int(getattr(config, 'NAV_STUCK_ZONE_ESCALATE_COUNT', 2)) if config else 2

        for zone in self.stuck_zone_memory:
            dist = math.hypot(cur_x - zone['x'], cur_y - zone['y'])
            if dist <= radius:
                zone['count'] += 1
                zone['last_time'] = now
                is_esc = (zone['count'] >= escalate_thresh)
                return is_esc, zone['count']

        self.stuck_zone_memory.append({
            'x': cur_x,
            'y': cur_y,
            'count': 1,
            'last_time': now
        })
        return False, 1

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Chuẩn hóa góc về khoảng [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def start_navigation(self, path: List[Tuple[float, float]], goal: Optional[Tuple[float, float]] = None) -> Dict[str, Any]:
        """Bắt đầu điều khiển xe tự hành bám theo danh sách waypoints hoặc đến điểm đích"""
        with self.lock:
            if not path or len(path) < 1:
                return {"success": False, "message": "Lộ trình rỗng, không có điểm waypoint nào"}

            # Hủy tiến trình cũ nếu đang chạy và chờ kết thúc sạch sẽ
            old_thread = self._nav_thread
            self.cancel_navigation()
        if old_thread is not None and old_thread.is_alive() and old_thread is not threading.current_thread():
            old_thread.join(timeout=1.0)
        with self.lock:

            self.waypoints = [tuple(p) for p in path]
            if goal is not None:
                self.goal = (float(goal[0]), float(goal[1]))
            else:
                self.goal = tuple(self.waypoints[-1])

            self.is_running = True
            self._cancel_requested = False
            self.current_wp_idx = 0
            self.progress_pct = 0.0

            # Nếu chế độ là stepped và có step_move_sync_cb -> chạy luồng Tự hành từng bước Sense-Plan-Act 30cm
            if self.mode == "stepped" and self.step_move_sync_cb is not None:
                self.state = self.STATE_INIT_SCAN
                self.status_text = f"BẮT ĐẦU: Quét 360° khởi tạo ({self.init_scan_rounds} vòng)..."
                self._nav_thread = threading.Thread(target=self._stepped_navigation_loop, daemon=True)
            else:
                self.state = self.STATE_TURNING
                self.status_text = f"BẮT ĐẦU: Tiến tới WP 1/{len(self.waypoints)}"
                self._nav_thread = threading.Thread(target=self._navigation_loop, daemon=True)

            self._nav_thread.start()

            print(f"🚀 [NAVIGATOR] Bắt đầu tự hành ({self.mode.upper()}) tới đích ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
            return {
                "success": True,
                "message": f"Đã kích hoạt tự hành ({self.mode})",
                "total_waypoints": len(self.waypoints),
                "goal": self.goal,
                "mode": self.mode
            }

    def cancel_navigation(self, reason: str = "Người dùng hủy") -> None:
        """Hủy ngay lập tức chu trình tự hành và dừng xe an toàn"""
        with self.lock:
            if self.is_running:
                self._cancel_requested = True
                self.is_running = False
                self.state = self.STATE_CANCELLED
                self.status_text = f"ĐÃ DỪNG: {reason}"
                print(f"🛑 [NAVIGATOR] {self.status_text}")

        # Hủy khẩn cấp bước di chuyển định lượng nếu đang chạy
        if self.cancel_step_cb:
            try:
                self.cancel_step_cb()
            except Exception:
                pass

        # Dừng động cơ ngay lập tức
        try:
            self.stop_cb()
        except Exception as e:
            print(f"⚠️ [NAVIGATOR] Lỗi khi gọi stop_cb: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Lấy thông tin trạng thái tự hành hiện tại để gửi về Web Cockpit"""
        with self.lock:
            total_wp = len(self.waypoints)
            return {
                "running": self.is_running,
                "state": self.state,
                "status_text": self.status_text,
                "current_wp_idx": self.current_wp_idx,
                "total_waypoints": total_wp,
                "dist_to_goal": round(self.dist_to_goal, 3),
                "progress_pct": round(self.progress_pct, 1),
                "target_wp": self.waypoints[self.current_wp_idx] if (0 <= self.current_wp_idx < total_wp) else None
            }

    def _is_path_direct_to_goal(self, cur_x: float, cur_y: float, goal_x: float, goal_y: float) -> bool:
        """
        Kiểm tra xem lộ trình A* hiện tại có phải là đường thẳng trực tiếp tới đích không:
          1. Tổng chiều dài các đoạn waypoints không vượt quá 1.15x khoảng cách thẳng tới đích.
          2. Không có góc ngoặt cua lớn (> 20°) giữa các đoạn waypoint.
          3. Không có vật cản cắt ngang giữa vị trí hiện tại và đích.
        """
        dist_direct = math.hypot(goal_x - cur_x, goal_y - cur_y)
        if dist_direct < 0.08:
            return True
        if not self.waypoints or len(self.waypoints) == 0:
            return False

        path_len = 0.0
        prev_pt = (cur_x, cur_y)
        prev_heading = None
        for wp in self.waypoints:
            seg_dx = wp[0] - prev_pt[0]
            seg_dy = wp[1] - prev_pt[1]
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len > 0.02:
                path_len += seg_len
                cur_h = math.atan2(seg_dy, seg_dx)
                if prev_heading is not None:
                    turn_deg = math.degrees(abs(self.normalize_angle(cur_h - prev_heading)))
                    if turn_deg > 20.0:
                        return False
                prev_heading = cur_h
            prev_pt = wp

        if path_len > dist_direct * 1.15:
            return False

        # Kiểm tra không gian giữa xe và đích có an toàn không nếu có callback
        if self.is_area_free_cb:
            num_samples = max(2, int(dist_direct / 0.15))
            for s in range(1, num_samples):
                frac = s / num_samples
                check_x = cur_x + frac * (goal_x - cur_x)
                check_y = cur_y + frac * (goal_y - cur_y)
                try:
                    is_free = self.is_area_free_cb(check_x, check_y, radius_m=0.12, allow_unknown=True)
                except TypeError:
                    is_free = self.is_area_free_cb(check_x, check_y, radius_m=0.12)
                if not is_free:
                    return False

        # Kiểm tra thêm callback va chạm lộ trình thẳng trực tiếp nếu có
        if self.is_path_blocked_cb:
            blocked, _, _ = self.is_path_blocked_cb([(goal_x, goal_y)], (cur_x, cur_y))
            if blocked:
                return False

        return True

    def _goal_in_direct_sight(self, cur_x: float, cur_y: float, cur_yaw: float,
                              goal_x: float, goal_y: float,
                              angle_threshold_deg: float = 20.0) -> bool:
        """
        Kiểm tra đích có đang thẳng phía trước và đường đi thoáng không:
          - Góc lệch giữa hướng xe hiện tại và hướng tới đích <= angle_threshold_deg.
        """
        dx = goal_x - cur_x
        dy = goal_y - cur_y
        dist = math.hypot(dx, dy)
        if dist < 0.01:  # Đã chạm đích
            return True
        heading_to_goal = math.atan2(dy, dx)
        angle_err_deg = math.degrees(abs(self.normalize_angle(heading_to_goal - cur_yaw)))
        return angle_err_deg <= angle_threshold_deg

    def _safe_backward_step(self, dist_cm: float, cur_x: float, cur_y: float, cur_yaw: float) -> float:
        """
        Lùi an toàn có kiểm tra bản đồ phía sau (Look-Behind trên 2D Occupancy Grid/Costmap):
        - Không lùi mù khi phía sau xe là tường hoặc vật cản.
        - Tự động cắt ngắn khoảng cách lùi nếu phát hiện cản sau lưng.
        - Trả về số cm thực tế đã lùi an toàn.
        """
        if not self.step_move_sync_cb or dist_cm <= 0.0 or self._cancel_requested:
            return 0.0

        desired_m = dist_cm / 100.0
        safe_m = desired_m

        # Kiểm tra dọc theo hướng lùi trên bản đồ 2D SLAM
        if self.is_area_free_cb:
            num_samples = max(2, int(desired_m / 0.04))
            for s in range(1, num_samples + 1):
                d = s * (desired_m / num_samples)
                # Điểm lùi sau lưng xe: -d theo vector hướng yaw
                bx = cur_x - d * math.cos(cur_yaw)
                by = cur_y - d * math.sin(cur_yaw)
                try:
                    is_free = self.is_area_free_cb(bx, by, radius_m=0.10, allow_unknown=True)
                except TypeError:
                    is_free = self.is_area_free_cb(bx, by, radius_m=0.10)

                if not is_free:
                    # Giữ vùng đệm an toàn 6cm trước vật cản
                    safe_m = max(0.0, d - 0.06)
                    break

        safe_cm = round(safe_m * 100.0, 1)
        if safe_cm < 3.0:
            print(f"🛑 [SAFE REVERSE] Sau lưng có vật cản sát đuôi xe ({safe_cm:.0f}cm < 3cm) -> Bỏ qua lệnh lùi để tránh cấn đuôi!")
            return 0.0

        if safe_cm < dist_cm:
            print(f"🛡️ [SAFE REVERSE] Cắt ngắn bước lùi ({dist_cm:.0f}cm -> {safe_cm:.0f}cm) do phía sau có tường/cản!")
        else:
            print(f"🛡️ [SAFE REVERSE] Lùi an toàn {safe_cm:.0f}cm (sau lưng thông thoáng)...")

        self.step_move_sync_cb('linear', safe_cm, 'backward', None)
        time.sleep(0.12)
        return safe_cm

    def _ensure_safe_turn_bubble(self, cur_x: float, cur_y: float, cur_yaw: float,
                                 clearance: Optional[float], turn_angle_deg: float) -> Tuple[float, float, float]:
        """
        Bảo vệ bán kính xoay an toàn (Safe Turn Bubble 18cm):
        - Trước khi xoay tại chỗ góc lớn (> 15°):
          Nếu cự ly mặt trước < 20cm hoặc trong bán kính quét xoay 16.8cm có vật cản trên map:
          Tự động nhích lùi nhẹ 6-8cm vào vùng an toàn trước khi xoay,
          tránh quẹt mũi/đuôi/cáp trái vào vật cản!
        """
        if turn_angle_deg <= 15.0 or self._cancel_requested:
            return cur_x, cur_y, cur_yaw

        rot_radius_m = float(getattr(config, 'NAV_SAFE_ROTATION_RADIUS_M', 0.168)) if config else 0.168
        needs_space = False
        reason = ""

        # 1. Kiểm tra cảm biến cự ly trước mặt (mũi xe 8cm + đệm an toàn)
        if clearance is not None and clearance < (rot_radius_m + 0.03):
            needs_space = True
            reason = f"mũi trước quá sát cản ({clearance*100:.0f}cm < {(rot_radius_m+0.03)*100:.0f}cm)"
        # 2. Kiểm tra bản đồ 2D xung quanh vị trí hiện tại
        elif self.is_area_free_cb:
            try:
                is_free = self.is_area_free_cb(cur_x, cur_y, radius_m=rot_radius_m, allow_unknown=True)
            except TypeError:
                is_free = self.is_area_free_cb(cur_x, cur_y, radius_m=rot_radius_m)
            if not is_free:
                needs_space = True
                reason = f"trong bán kính quét xoay {rot_radius_m*100:.1f}cm có vật cản trên map"

        if needs_space:
            print(f"🛡️ [SAFE TURN BUBBLE] Cảnh báo xoay ({reason}) -> Nhích lùi an toàn để tạo khoảng đệm quay!")
            backed_cm = self._safe_backward_step(7.0, cur_x, cur_y, cur_yaw)
            if backed_cm > 0.0:
                cur_x, cur_y, cur_yaw = self.get_pose_cb()

        return cur_x, cur_y, cur_yaw

    def _stepped_navigation_loop(self) -> None:
        """
        Vòng lặp tự hành từng bước chuẩn công nghiệp (Stepped Sense-Plan-Act, tối đa 40cm):
        1. KHỞI TẠO: Quay 360° theo yêu cầu (1 vòng x 8 nấc 45°) để nạp bản đồ ban đầu.
        2. VÒNG LẶP ĐIỀU HƯỚNG TỪNG BƯỚC:
           a. TÁI LẬP KẾ HOẠCH (REPLAN): Chạy A* từ vị trí robot tới Đích.
           b. KIỂM TRA ĐÍCH: Nếu <= goal_tolerance_m (6cm) -> GOAL_REACHED!
           c. ĐỊNH HƯỚNG: Tính góc lệch Δψ, quay tại chỗ nếu |Δψ| > 12°.
           d. TIẾN BƯỚC: S = min(step_dist_cm=40cm, dist_to_target).
           e. KIỂM TRA LẠI ĐÍCH sau bước tiến.
           f. QUAN SÁT THÔNG MINH:
              - Nếu đích thẳng phía trước (góc <= 20°): BỎ QUA HOÀN TOÀN peek/sweep,
                tiến thẳng liên tục. Chỉ chụp 1 static frame nhanh phía trước.
              - Nếu cần rẽ góc lớn (>= 30°): liếc góc cua (peek).
              - Nếu đi thẳng lâu tích lũy vùng mù (>= 90cm): quét bán nguyệt 180°.
           g. Lặp lại bước a.
        """
        try:
            cur_x, cur_y, cur_yaw = self.get_pose_cb()
            goal_x, goal_y = self.goal if self.goal else self.waypoints[-1]

            # ── 1. KHỞI TẠO: QUAY 360° (CHỈ QUÉT KHI CẤU HÌNH YÊU CẦU) ──
            init_rounds = getattr(self, 'init_scan_rounds', 0)
            if init_rounds > 0 and self.observe_scan_cb and not self._cancel_requested:
                print(f"🔄 [NAV STEPPED] Bắt đầu chu trình quét 360° khởi tạo {init_rounds} vòng...")
                for r in range(1, init_rounds + 1):
                    if self._cancel_requested:
                        break
                    with self.lock:
                        self.state = self.STATE_INIT_SCAN
                        self.status_text = f"QUÉT 360° KHỞI TẠO: VÒNG {r}/{init_rounds}..."
                    print(f"📸 [NAV STEPPED] Quét khởi tạo vòng {r}/{init_rounds}...")
                    is_last_init = (r == init_rounds)
                    self.observe_scan_cb(rounds=1, is_initial=is_last_init)
                    time.sleep(0.2)
            else:
                print("🚀 [NAV STEPPED] Bỏ qua quét 360° khởi tạo: Ưu tiên tiến thẳng về phía đích tức thời!")

            if self._cancel_requested:
                return

            step_count = 0
            max_steps = 150  # Giới hạn an toàn chống lặp vô tận
            replan_fail_streak = 0
            straight_dist_accum = 0.0
            straight_steps_accum = 0
            force_replan = True  # Luôn tính toán A* ở bước đầu tiên
            steps_since_last_replan = 0

            while self.is_running and not self._cancel_requested and step_count < max_steps:
                cur_x, cur_y, cur_yaw = self.get_pose_cb()
                dist_to_goal = math.hypot(goal_x - cur_x, goal_y - cur_y)
                self.dist_to_goal = dist_to_goal

                # Kiểm tra cự ly chạm đích cuối cùng (<= 6cm)
                if dist_to_goal <= self.goal_tolerance_m:
                    self.progress_pct = 100.0
                    self._complete_navigation(f"ĐÃ ĐẾN ĐÍCH! (Sai số: {dist_to_goal*100:.1f}cm)")
                    break

                step_count += 1
                max_step_m = self.step_dist_cm / 100.0  # Tối đa 40cm
                is_direct = self._is_path_direct_to_goal(cur_x, cur_y, goal_x, goal_y)

                # ── a. CẮT TỈA WAYPOINTS ĐÃ ĐI QUA & XÁC ĐỊNH NHU CẦU REPLAN ──
                min_dist_to_path = float('inf')
                closest_wp_idx = 0
                if self.waypoints:
                    for idx, wp in enumerate(self.waypoints):
                        d = math.hypot(wp[0] - cur_x, wp[1] - cur_y)
                        if d < min_dist_to_path:
                            min_dist_to_path = d
                            closest_wp_idx = idx

                    # Bỏ qua các waypoints cũ phía sau xe đã đi qua
                    if closest_wp_idx > 0:
                        self.waypoints = self.waypoints[closest_wp_idx:]

                    # Loại bỏ waypoint hiện tại nếu xe đã đứng sát (< 8cm) và vẫn còn các điểm tiếp theo
                    while self.waypoints and math.hypot(self.waypoints[0][0] - cur_x, self.waypoints[0][1] - cur_y) < 0.08 and len(self.waypoints) > 1:
                        self.waypoints.pop(0)

                # ── KIỂM TRA VA CHẠM LỘ TRÌNH VỚI BẢN ĐỒ THỜI GIAN THỰC (ACTIVE PATH COLLISION CHECK) ──
                is_path_blocked = False
                block_pt = None
                block_d = None
                if self.is_path_blocked_cb and self.waypoints:
                    is_path_blocked, block_pt, block_d = self.is_path_blocked_cb(self.waypoints, (cur_x, cur_y))

                # ── KIỂM TRA ĐỘ THOÁNG ĐỐI VỚI VẬT CẢN (PATH OBSTACLE CLEARANCE & PROXIMITY ALERT) ──
                is_in_proximity = False
                min_path_clearance = 99.0
                nearest_obs_pt = None
                obs_rel_angle = 0.0

                if self.get_path_clearance_cb and self.waypoints:
                    try:
                        min_path_clearance, nearest_obs_pt, obs_rel_angle = self.get_path_clearance_cb(
                            self.waypoints, (cur_x, cur_y, cur_yaw)
                        )
                        if min_path_clearance <= self.proximity_alert_dist_m:
                            is_in_proximity = True
                    except Exception as e:
                        print(f"⚠️ [PROXIMITY] Lỗi kiểm tra cự ly cản: {e}")

                # ── FINAL SPRINT MODE (BỨT TỐC VỀ ĐÍCH KHÔNG CHẦN CHỪ) ──
                clearance = self.get_clearance_cb() if self.get_clearance_cb else None
                sprint_dist_thresh = float(getattr(config, 'NAV_FINAL_SPRINT_DIST_M', 1.20)) if config else 1.20
                front_safe = (clearance is None or clearance >= min(0.40, dist_to_goal - 0.05))
                goal_in_front_sight = self._goal_in_direct_sight(cur_x, cur_y, cur_yaw, goal_x, goal_y, angle_threshold_deg=35.0)
                # Khi trong vùng cảnh giác cận cản: Không kích hoạt bứt tốc trực tiếp
                is_final_sprint = (dist_to_goal <= sprint_dist_thresh) and (not is_path_blocked) and front_safe and (is_direct or goal_in_front_sight) and (not is_in_proximity)

                # Xác định xem có thực sự cần gọi thuật toán A* không (Event-Driven Replanning)
                need_replan = False
                replan_reason = ""

                if is_path_blocked:
                    need_replan = True
                    b_str = f" tại ({block_pt[0]:.2f}, {block_pt[1]:.2f}) cách {block_d*100:.0f}cm" if block_pt and block_d is not None else ""
                    replan_reason = f"phát hiện vật cản mới cắt ngang lộ trình{b_str}"
                elif force_replan:
                    need_replan = True
                    replan_reason = "khởi tạo hoặc sau khi gặp cản cần né"
                elif not self.waypoints or len(self.waypoints) == 0:
                    need_replan = True
                    replan_reason = "hết waypoints phía trước"
                elif min_dist_to_path > 0.25 and not is_direct and not is_final_sprint:
                    need_replan = True
                    replan_reason = f"lệch khỏi lộ trình ({min_dist_to_path*100:.0f}cm > 25cm)"
                elif steps_since_last_replan >= 3 and not is_direct and not is_final_sprint:
                    need_replan = True
                    replan_reason = f"chu kỳ định kỳ sau {steps_since_last_replan} bước cập nhật cản tĩnh mới"

                # Nếu đích thẳng trước mắt HOẶC đang trong Final Sprint VÀ đường đi thực sự không có cản:
                if (is_direct or is_final_sprint) and not force_replan and not is_path_blocked:
                    need_replan = False

                # ── b. TÁI LẬP KẾ HOẠCH ĐƯỜNG ĐI A* (NẾU CÓ SỰ KIỆN KÍCH HOẠT) ──
                if need_replan and self.replan_cb:
                    with self.lock:
                        self.state = self.STATE_REPLANNING
                        self.status_text = f"TÍNH TOÁN ĐƯỜNG A* ({replan_reason})... CÒN {dist_to_goal:.2f}M"
                    print(f"🧠 [NAV STEPPED] BƯỚC {step_count}: Tính toán lộ trình A* do {replan_reason}...")

                    new_path = self.replan_cb(goal_x, goal_y)
                    if new_path and len(new_path) >= 1:
                        self.waypoints = [tuple(p) for p in new_path]
                        self.current_wp_idx = 0
                        replan_fail_streak = 0
                        force_replan = False
                        steps_since_last_replan = 0
                    else:
                        replan_fail_streak += 1
                        max_replan_streak = int(getattr(config, 'NAV_MAX_REPLAN_FAIL_STREAK', 5)) if config else 5
                        print(f"⚠️ [NAV STEPPED] Replan A* chưa tìm được đường mới (thử lần {replan_fail_streak}/{max_replan_streak})...")
                        
                        # Cơ chế thông minh: Nếu tắc đường, kích hoạt quét 360° dự phòng để mở rộng bản đồ tìm lối thoát
                        if replan_fail_streak == 1 and self.observe_scan_cb and not self._cancel_requested:
                            print("🔄 [NAV STEPPED] Nghi ngờ bị chắn góc nhìn hẹp -> Kích hoạt Quét 360° khẩn cấp tìm lối thoát...")
                            with self.lock:
                                self.state = self.STATE_OBSERVING
                                self.status_text = "TẮC ĐƯỜNG: QUÉT 360° MỞ RỘNG TẦM NHÌN TÌM LỐI THOÁT..."
                            self.observe_scan_cb(rounds=1, is_initial=False)
                            time.sleep(0.15)
                            # Thử replan lại ngay lập tức
                            recovered_path = self.replan_cb(goal_x, goal_y)
                            if recovered_path and len(recovered_path) >= 1:
                                self.waypoints = [tuple(p) for p in recovered_path]
                                self.current_wp_idx = 0
                                replan_fail_streak = 0
                                force_replan = False
                                steps_since_last_replan = 0
                                print(f"🎉 [NAV STEPPED] ĐÃ TÌM THẤY LỐI THOÁT MỚI ({len(recovered_path)} waypoints) sau khi quét 360°!")

                        # Kiểm tra xem lộ trình hiện hữu có còn điểm nào phía trước không
                        if replan_fail_streak > 0:
                            has_valid_future_wp = (not is_path_blocked) and any(math.hypot(wp[0] - cur_x, wp[1] - cur_y) >= 0.08 for wp in self.waypoints)
                            max_replan_streak = int(getattr(config, 'NAV_MAX_REPLAN_FAIL_STREAK', 5)) if config else 5
                            if has_valid_future_wp and replan_fail_streak < max_replan_streak:
                                print("ℹ️ [NAV STEPPED] Tiếp tục duy trì lộ trình hiện hữu để thoát khỏi vùng hẹp!")
                            else:
                                # Nếu đường cũ bị cản chặn, hủy bỏ và thực hiện lùi an toàn để tìm hướng mới
                                if is_path_blocked and self.step_move_sync_cb and replan_fail_streak <= 2:
                                    print("⚠️ [NAV STEPPED] Lộ trình cũ đã bị cản chặn -> Lùi an toàn 12cm để mở rộng không gian tìm đường mới!")
                                    with self.lock:
                                        self.state = self.STATE_RECOVERING
                                        self.status_text = "LỘ TRÌNH BỊ CHẶN: LÙI 12CM MỞ GÓC TÌM ĐƯỜNG MỚI..."
                                    backed_cm = self._safe_backward_step(12.0, cur_x, cur_y, cur_yaw)
                                    if backed_cm > 0.0:
                                        cur_x, cur_y, cur_yaw = self.get_pose_cb()
                                    if self.add_virtual_obstacle_cb and block_pt:
                                        self.add_virtual_obstacle_cb(block_pt[0], block_pt[1], radius=0.18, duration_s=25.0)
                                    force_replan = True
                                    continue
                                else:
                                    print(f"⚠️ [NAV STEPPED] DỪNG TỰ HÀNH: Không tìm thấy đường đi khả dụng sau {replan_fail_streak} lần thử!")
                                    with self.lock:
                                        self.is_running = False
                                        self.state = self.STATE_OBSTACLE_BLOCKED
                                        self.status_text = "DỪNG: BỊ CẢN CHẶN HOÀN TOÀN KHÔNG CÓ ĐƯỜNG ĐI"
                                    break
                else:
                    steps_since_last_replan += 1
                    print(f"⏩ [NAV STEPPED] BƯỚC {step_count}: Tiếp tục bám lộ trình ({len(self.waypoints)} waypoints, thẳng đích: {is_direct})...")

                if self._cancel_requested:
                    break

                # ── c. XÁC ĐỊNH MỤC TIÊU TIẾP THEO (LOOKAHEAD WAYPOINT & DIRECT GOAL) ──
                if is_final_sprint or is_direct:
                    target_wp = (goal_x, goal_y)
                    dist_to_wp = dist_to_goal
                else:
                    # Đi theo waypoints: tìm waypoint xa nhất phía trước trên đoạn thẳng
                    # Cho phép lookahead dài lên tới step_max_dist_cm (65cm) nếu đường thẳng thoáng
                    lookahead_limit_m = (self.step_max_dist_cm / 100.0) if (clearance is None or clearance >= 1.0) else (self.step_dist_cm / 100.0)
                    target_wp = None
                    for wp in self.waypoints:
                        d_wp = math.hypot(wp[0] - cur_x, wp[1] - cur_y)
                        if d_wp < 0.08:
                            continue
                        if target_wp is None:
                            target_wp = wp
                        elif d_wp <= lookahead_limit_m:
                            h_prev = math.atan2(target_wp[1] - cur_y, target_wp[0] - cur_x)
                            h_curr = math.atan2(wp[1] - cur_y, wp[0] - cur_x)
                            if math.degrees(abs(self.normalize_angle(h_curr - h_prev))) < 20.0:
                                target_wp = wp
                            else:
                                break
                        else:
                            break
                    if target_wp is None:
                        target_wp = (goal_x, goal_y)
                    dist_to_wp = math.hypot(target_wp[0] - cur_x, target_wp[1] - cur_y)

                # ── d. ĐỊNH HƯỚNG VỀ PHÍA WAYPOINT (TURN TO HEADING) ──
                dx_target = target_wp[0] - cur_x
                dy_target = target_wp[1] - cur_y
                dist_to_wp = math.hypot(dx_target, dy_target)
                target_heading = math.atan2(dy_target, dx_target)
                angle_err = self.normalize_angle(target_heading - cur_yaw)
                abs_angle_deg = math.degrees(abs(angle_err))

                # Giao thức vượt khe hẹp 28-30cm thích ứng (Tight Gap Protocol):
                clearance = self.get_clearance_cb() if self.get_clearance_cb else None
                is_in_tight = (clearance is not None and 0.12 <= clearance <= 0.32)
                approaching_tight = (clearance is not None and 0.32 < clearance <= 0.60)

                if is_in_tight:
                    turn_thresh_deg = 22.0 # Nới lỏng trong khe
                elif approaching_tight:
                    turn_thresh_deg = 8.0  # Siết chặt căn thẳng mũi trước khi chui khe
                else:
                    turn_thresh_deg = 12.0 # Chuẩn ở sân rộng

                if abs_angle_deg > turn_thresh_deg:
                    # Trong Final Sprint và góc lệch nhỏ (<= 18°), bỏ qua xoay tại chỗ để bứt tốc thẳng
                    if is_final_sprint and abs_angle_deg <= 18.0:
                        print(f"🎯 [FINAL SPRINT] Góc lệch nhỏ {abs_angle_deg:.1f}° <= 18° -> Bỏ qua xoay tại chỗ, tiến thẳng bứt tốc!")
                    elif is_in_tight and abs_angle_deg <= 30.0:
                        print(f"🚪 [NAV TIGHT GAP] Đang giữa khe hẹp, bỏ qua xoay tại chỗ {abs_angle_deg:.1f}°! Khóa thân lướt thẳng an toàn.")
                    else:
                        # ── SAFE TURN BUBBLE: Đảm bảo khoảng cách an toàn trước khi xoay ──
                        cur_x, cur_y, cur_yaw = self._ensure_safe_turn_bubble(
                            cur_x, cur_y, cur_yaw, clearance, abs_angle_deg
                        )
                        dx_target = target_wp[0] - cur_x
                        dy_target = target_wp[1] - cur_y
                        dist_to_wp = math.hypot(dx_target, dy_target)
                        target_heading = math.atan2(dy_target, dx_target)
                        angle_err = self.normalize_angle(target_heading - cur_yaw)
                        abs_angle_deg = math.degrees(abs(angle_err))

                        turn_dir = 'left' if angle_err > 0 else 'right'
                        with self.lock:
                            self.state = self.STATE_TURNING
                            self.status_text = f"XOAY {'TRÁI' if turn_dir == 'left' else 'PHẢI'} {abs_angle_deg:.0f}° HƯỚNG VỀ MỤC TIÊU..."
                        print(f"🔄 [NAV STEPPED] BƯỚC {step_count}: Xoay {turn_dir} {abs_angle_deg:.1f}° theo IMU...")

                        if self.step_move_sync_cb:
                            self.step_move_sync_cb('turn', abs_angle_deg, turn_dir, None)
                        time.sleep(0.12)

                    # Nếu xoay góc đáng kể (>= 20°), reset quãng đường thẳng tích lũy
                    if abs_angle_deg >= 20.0:
                        straight_dist_accum = 0.0
                        straight_steps_accum = 0

                    # Cập nhật lại tọa độ và hướng sau khi xoay để bước tiến đi thẳng chuẩn xác
                    cur_x, cur_y, cur_yaw = self.get_pose_cb()
                    dx_target = target_wp[0] - cur_x
                    dy_target = target_wp[1] - cur_y
                    dist_to_wp = math.hypot(dx_target, dy_target)

                if self._cancel_requested:
                    break

                # ── e. TIẾN BƯỚC ĐỊNH LƯỢNG (DYNAMIC STEP SCALING) ──
                allowed_step_m = self.step_dist_cm / 100.0

                if self.dynamic_step_enabled:
                    clearance = self.get_clearance_cb() if self.get_clearance_cb else None

                    # Trường hợp ĐẶC BIỆT: VÙNG CẢNH GIÁC CẬN CẢN (5-6cm Alert Bubble)
                    if is_in_proximity:
                        # Khống chế bước tiến tối đa thận trọng (10 - 15cm)
                        prox_max_m = self.proximity_step_max_cm / 100.0
                        allowed_step_m = min(allowed_step_m, prox_max_m)
                        print(f"🛡️ [PROXIMITY ALERT] Bước {step_count}: Trong Vùng Cảnh Giác Cận Cản ({min_path_clearance*100:.0f}cm <= {self.proximity_alert_dist_m*100:.0f}cm) -> Khống chế bước ngắn {allowed_step_m*100:.0f}cm!")

                        # Cơ chế nhìn lại vật cản (Look-Again Re-Inspection) trước khi bước qua
                        now_t = time.time()
                        abs_obs_angle = abs(obs_rel_angle)
                        if self.look_again_cb and (15.0 <= abs_obs_angle <= 75.0) and (now_t - getattr(self, 'last_look_again_time', 0.0) >= 0.8) and not self._cancel_requested:
                            self.last_look_again_time = now_t
                            peek_side = 'left' if obs_rel_angle > 0 else 'right'
                            peek_ang = min(30.0, max(15.0, abs_obs_angle * 0.8))
                            with self.lock:
                                self.state = self.STATE_PROXIMITY_LOOK_AGAIN
                                self.status_text = f"CẢNH GIÁC CẬN CẢN ({min_path_clearance*100:.0f}CM): NHÌN LẠI CẢN BÊN {'TRÁI' if peek_side=='left' else 'PHẢI'}..."
                            print(f"👀 [LOOK-AGAIN] Bước {step_count}: Cận cản ({min_path_clearance*100:.0f}cm) -> Liếc nhìn lại mép cản bên {peek_side} ({peek_ang:.0f}°)...")
                            self.look_again_cb(peek_side, peek_ang)
                            time.sleep(0.08)

                            cur_x, cur_y, cur_yaw = self.get_pose_cb()
                            clearance = self.get_clearance_cb() if self.get_clearance_cb else None

                            # Thẩm định an toàn: nếu mép cản ngoài đời áp sát nguy hiểm (< proximity_safe_clearance_m, vd 18cm)
                            if clearance is not None and clearance < self.proximity_safe_clearance_m:
                                print(f"🛑 [LOOK-AGAIN DANGER] Mép cản thực tế áp sát ({clearance*100:.0f}cm < {self.proximity_safe_clearance_m*100:.0f}cm)! Dừng bước, nhích lùi và Replan né cản!")
                                with self.lock:
                                    self.state = self.STATE_RECOVERING
                                    self.status_text = f"MÉP CẢN QUÁ GẦN ({clearance*100:.0f}CM): LÙI 6CM NÉ CẢN..."
                                if self.add_virtual_obstacle_cb:
                                    rad_angle = cur_yaw + math.radians(obs_rel_angle)
                                    vox = cur_x + clearance * math.cos(rad_angle)
                                    voy = cur_y + clearance * math.sin(rad_angle)
                                    self.add_virtual_obstacle_cb(vox, voy, radius=0.16, duration_s=20.0)
                                backed_cm = self._safe_backward_step(6.0, cur_x, cur_y, cur_yaw)
                                if backed_cm > 0.0:
                                    cur_x, cur_y, cur_yaw = self.get_pose_cb()
                                force_replan = True
                                continue

                    # Trường hợp 0: FINAL SPRINT - Đích trong tầm ngắm (dist_to_goal <= 1.2m)
                    elif is_final_sprint:
                        max_sprint_m = min(dist_to_goal, self.step_max_dist_cm / 100.0)
                        if clearance is not None:
                            max_sprint_m = min(max_sprint_m, max(0.12, clearance - 0.20))
                        allowed_step_m = max_sprint_m
                        print(f"🎯 [FINAL SPRINT] Đích trong tầm ngắm ({dist_to_goal*100:.0f}cm) -> Phóng dứt khoát {allowed_step_m*100:.0f}cm!")

                    # Trường hợp 1: Đường thẳng & thoáng rộng (clearance > 1.2m hoặc không thấy cản)
                    # Sải bước dài lên tới step_max_dist_cm (65cm) để giảm giật cục, tăng tốc hành trình
                    elif (is_direct or abs_angle_deg <= 12.0) and (clearance is None or clearance >= 1.20) and dist_to_wp >= 0.45:
                        max_possible_m = self.step_max_dist_cm / 100.0
                        if clearance is not None:
                            max_possible_m = min(max_possible_m, max(0.35, clearance - 0.40))
                        allowed_step_m = max_possible_m
                        clr_str = f"{clearance*100:.0f}cm" if clearance is not None else "THOÁNG"
                        print(f"🚀 [DYNAMIC STEP] Đường thẳng thoáng ({clr_str}) -> Sải bước dài {allowed_step_m*100:.0f}cm!")

                    # Trường hợp 2: Gần chướng ngại vật (clearance < 0.75m)
                    # Thu ngắn bước để tiếp cận an toàn, không bao giờ phóng 40cm đâm vào cản
                    elif clearance is not None and clearance < 0.75:
                        # Nếu cản quá gần (<= 32cm) và xe đang hướng thẳng vào cản: Chủ động dừng tiến & Né cản từ xa!
                        if clearance <= 0.32 and abs_angle_deg <= 35.0:
                            print(f"🛑 [DYNAMIC STEP] Cản trực diện quá gần ({clearance*100:.0f}cm <= 32cm) -> Dừng tiến, chủ động kích hoạt Replan né cản!")
                            if self.add_virtual_obstacle_cb:
                                v_x = cur_x + clearance * math.cos(cur_yaw)
                                v_y = cur_y + clearance * math.sin(cur_yaw)
                                self.add_virtual_obstacle_cb(v_x, v_y, radius=0.18, duration_s=25.0)
                            force_replan = True
                            continue

                        # Thu ngắn bước an toàn: giữ khoảng đệm tối thiểu 22cm trước mũi xe
                        allowed_step_m = min(self.step_min_dist_cm / 100.0, max(0.06, clearance - 0.22))
                        print(f"⚠️ [DYNAMIC STEP] Cận cản ({clearance*100:.0f}cm) -> Thu ngắn bước an toàn {allowed_step_m*100:.0f}cm (vùng đệm >= 22cm)")

                    # Trường hợp 3: Chuẩn bị rẽ góc lớn (> 25°) sau waypoint này
                    # Chỉ tiến tối đa đến đúng waypoint đó, không chạy lố cua
                    elif dist_to_wp < allowed_step_m:
                        allowed_step_m = dist_to_wp

                step_dist_m = min(allowed_step_m, dist_to_wp, dist_to_goal)
                step_cm = max(5.0, round(step_dist_m * 100.0, 1))

                with self.lock:
                    self.state = self.STATE_DRIVING
                    if is_in_proximity:
                        self.status_text = f"🛡️ [CẢNH GIÁC CẬN CẢN] TIẾN {step_cm:.0f}CM (BƯỚC {step_count})... CÒN {dist_to_goal:.2f}M"
                    else:
                        self.status_text = f"TIẾN BƯỚC {step_cm:.0f}CM (BƯỚC {step_count})... CÒN {dist_to_goal:.2f}M"
                print(f"📏 [NAV STEPPED] BƯỚC {step_count}: Tiến {step_cm:.0f}cm bằng Cosine S-Curve...")

                step_success = True
                if self.step_move_sync_cb:
                    res = self.step_move_sync_cb('linear', step_cm, 'forward', None)
                    if res is False:
                        step_success = False
                time.sleep(0.12)

                if self._cancel_requested:
                    break

                # PROGRESSIVE OBSTACLE RECOVERY SEQUENCE (Chuẩn ROS2 Nav2 Phase 4):
                # Thay vì quét 360° mỗi lần gặp cản, phân 3 cấp độ:
                # - Level 1: Lùi 12cm, nạp static frame, Replan A* ngay (KHÔNG XOAY XE!)
                # - Level 2: Lùi 18cm, liếc ±45° tìm lối thoát, Replan A*
                # - Level 3: Lùi 25cm, quét bán nguyệt 180°, Replan A*
                if not step_success:
                    self.consecutive_stuck_count += 1
                    stuck_lvl = self.consecutive_stuck_count

                    # Kiểm tra bộ nhớ vùng kẹt không gian (Spatial Stuck Memory)
                    is_esc_stuck, zone_k_cnt = self._record_and_check_stuck_zone(cur_x, cur_y)
                    esc_duration = float(getattr(config, 'NAV_STUCK_ZONE_ESCALATE_DURATION_S', 60.0)) if config else 60.0
                    vo_radius = 0.20 if is_esc_stuck else float(getattr(config, 'VIRTUAL_OBSTACLE_RADIUS_M', 0.14)) if config else 0.14
                    vo_dur = esc_duration if is_esc_stuck else float(getattr(config, 'VIRTUAL_OBSTACLE_DURATION_S', 18.0)) if config else 18.0

                    if is_esc_stuck:
                        print(f"⚠️ [NAV STUCK ESCALATE] PHÁT HIỆN VÙNG KẸT LẶP LẠI (Lần {zone_k_cnt}) tại ({cur_x:.2f}, {cur_y:.2f}) -> Bơm cản ảo R={vo_radius*100:.0f}cm trong {vo_dur:.0f}s!")

                    print(f"[NAV RECOVERY] Bước {step_count}: Phanh dừng xe! (Lần kẹt thứ {stuck_lvl}) Thực hiện PROGRESSIVE RECOVERY LEVEL {min(stuck_lvl, 3)}...")
                    with self.lock:
                        self.state = self.STATE_OBSTACLE_BLOCKED
                        self.status_text = f"GẶP CẢN (LẦN {stuck_lvl}) - ĐANG PHỤC HỒI LEVEL {min(stuck_lvl, 3)}..."

                    def _inject_recovery_vobs(fwd_m):
                        if self.add_virtual_obstacle_cb and not self._cancel_requested:
                            ovx = cur_x + fwd_m * math.cos(cur_yaw)
                            ovy = cur_y + fwd_m * math.sin(cur_yaw)
                            try:
                                self.add_virtual_obstacle_cb(ovx, ovy, radius=vo_radius, duration_s=vo_dur)
                            except TypeError:
                                self.add_virtual_obstacle_cb(ovx, ovy)

                    # Cấp 1 (Lần đầu kẹt): Lùi 12cm an toàn, static frame, Replan ngay (0 xoay)
                    if stuck_lvl == 1:
                        if self.static_scan_cb and not self._cancel_requested:
                            self.static_scan_cb()
                            time.sleep(0.08)
                        _inject_recovery_vobs(0.28)
                        if self.step_move_sync_cb and not self._cancel_requested:
                            backed_cm = self._safe_backward_step(12.0, cur_x, cur_y, cur_yaw)
                            if backed_cm > 0.0:
                                cur_x, cur_y, cur_yaw = self.get_pose_cb()

                    # Cấp 2 (Lần 2 liên tiếp): Lùi 18cm an toàn, liếc ±45°
                    elif stuck_lvl == 2:
                        if self.static_scan_cb and not self._cancel_requested:
                            self.static_scan_cb()
                        _inject_recovery_vobs(0.26)
                        if self.step_move_sync_cb and not self._cancel_requested:
                            backed_cm = self._safe_backward_step(18.0, cur_x, cur_y, cur_yaw)
                            if backed_cm > 0.0:
                                cur_x, cur_y, cur_yaw = self.get_pose_cb()
                        if self.peek_scan_cb and not self._cancel_requested:
                            with self.lock:
                                self.state = self.STATE_PEEKING
                                self.status_text = "LIẾC ±45° TÌM HƯỚNG THOÁT CẢN (LEVEL 2)..."
                            self.peek_scan_cb(45.0)
                            time.sleep(0.12)

                    # Cấp 3 (Lần 3+ liên tiếp): Lùi 25cm an toàn, quét 180°
                    else:
                        _inject_recovery_vobs(0.25)
                        if self.step_move_sync_cb and not self._cancel_requested:
                            backed_cm = self._safe_backward_step(25.0, cur_x, cur_y, cur_yaw)
                            if backed_cm > 0.0:
                                cur_x, cur_y, cur_yaw = self.get_pose_cb()
                        if (self.sweep_scan_cb or self.observe_scan_cb) and not self._cancel_requested:
                            with self.lock:
                                self.state = self.STATE_SWEEPING
                                self.status_text = "QUÉT BÁN NGUYỆT 180° TÌM ĐƯỜNG VÒNG (LEVEL 3)..."
                            if self.sweep_scan_cb:
                                self.sweep_scan_cb(55.0)
                            elif self.observe_scan_cb:
                                self.observe_scan_cb(rounds=1, is_initial=False)
                            time.sleep(0.15)

                    straight_dist_accum = 0.0
                    straight_steps_accum = 0
                    force_replan = True
                    with self.lock:
                        self.state = self.STATE_REPLANNING
                        self.status_text = f"ĐÃ GIẢI PHÓNG PHANH L{min(stuck_lvl, 3)} - TÁI LẬP A* TÌM ĐƯỜNG..."
                    continue

                # KHI TIẾN BƯỚC THÀNH CÔNG: Xóa bộ đếm kẹt
                self.consecutive_stuck_count = 0
                straight_dist_accum += (step_cm / 100.0)
                straight_steps_accum += 1

                # Kiểm tra sau khi bước: nếu đã chạm đích (<= 6cm) thì hoàn thành luôn!
                cur_x, cur_y, cur_yaw = self.get_pose_cb()
                dist_after = math.hypot(goal_x - cur_x, goal_y - cur_y)
                self.dist_to_goal = dist_after
                if dist_after <= self.goal_tolerance_m:
                    self.progress_pct = 100.0
                    self._complete_navigation(f"ĐÃ ĐẾN ĐÍCH! (Sai số: {dist_after*100:.1f}cm)")
                    break

                # ── d. QUAN SÁT TỐI ƯU CẬP NHẬT BẢN ĐỒ (ACTIVE PERCEPTION & SMART OBSERVATION) ──
                # Kiểm tra nếu đích thẳng thoáng hoặc nằm trong tầm nhìn trực diện hoặc đang Final Sprint
                goal_in_sight = (is_direct or is_final_sprint or (dist_to_goal <= 1.20) or self._goal_in_direct_sight(
                    cur_x, cur_y, cur_yaw, goal_x, goal_y, angle_threshold_deg=25.0
                )) and (not is_in_proximity)

                if goal_in_sight:
                    # ── ĐÍCH THẲNG TRƯỚC MẮT / SPRINT: BỎ QUA HOÀN TOÀN peek/sweep, TIẾN DỨT KHOÁT! ──
                    if self.static_scan_cb and not self._cancel_requested:
                        with self.lock:
                            self.state = self.STATE_STATIC_SCAN
                            self.status_text = f"ĐÍCH THẲNG - NẠP FRAME PHÍA TRƯỚC (BƯỚC {step_count})..."
                        self.static_scan_cb()
                        time.sleep(0.03)  # Siêu nhanh chỉ 30ms
                    print(f"⏩ [NAV ACTIVE] BƯỚC {step_count}: ĐÍCH TRONG TẦM NHÌN "
                          f"(còn {dist_to_goal*100:.0f}cm) - BỎ QUA HOÀN TOÀN peek/sweep, TIẾN DỨT KHOÁT!")
                    straight_dist_accum = 0.0
                    straight_steps_accum = 0

                else:
                    # ── ĐÍCH KHÔNG THẲNG: Xử lý theo chiến thuật quan sát đã cấu hình ──

                    # 1. Chụp frame tĩnh phía trước (chỉ ~0.06s, nạp cản trực diện)
                    if self.static_scan_cb and not self._cancel_requested:
                        with self.lock:
                            self.state = self.STATE_STATIC_SCAN
                            self.status_text = f"NẠP FRAME TĨNH PHÍA TRƯỚC (BƯỚC {step_count})..."
                        self.static_scan_cb()
                        time.sleep(0.06)

                    # 2. Xử lý theo chiến thuật quan sát đã cấu hình
                    if self.observe_mode == "event_driven":
                        # CHẾ ĐỘ QUAN SÁT SỰ KIỆN CHUẨN ROS2 (EVENT-DRIVEN NAVIGATION):
                        # Triết lý: TIẾN VỀ ĐÍCH LÀ ƯU TIÊN SỐ 1. Bỏ qua hoàn toàn các nhịp quét định kỳ.
                        # Chỉ kiểm tra xem waypoint cua tiếp theo có vào vùng Unknown không.
                        next_turn_deg = 0.0
                        next_wp_ahead = None
                        for wp in self.waypoints:
                            if math.hypot(wp[0] - cur_x, wp[1] - cur_y) >= 0.12:
                                next_wp_ahead = wp
                                break
                        if next_wp_ahead is not None:
                            ahead_heading = math.atan2(next_wp_ahead[1] - cur_y, next_wp_ahead[0] - cur_x)
                            next_turn_deg = math.degrees(abs(self.normalize_angle(ahead_heading - cur_yaw)))

                        # CHỈ peek nếu góc cua gắt (>= 45°) VÀ vùng đó chưa phải là Free Space
                        if next_turn_deg >= self.peek_trigger_angle_deg and self.peek_scan_cb and not self._cancel_requested:
                            is_turn_area_free = False
                            if self.costmap_aware_peek and self.is_area_free_cb and next_wp_ahead is not None:
                                is_turn_area_free = self.is_area_free_cb(next_wp_ahead[0], next_wp_ahead[1])
                            if not is_turn_area_free:
                                with self.lock:
                                    self.state = self.STATE_PEEKING
                                    self.status_text = f"LIẾC GÓC CUA ±{self.peek_angle_deg:.0f}° (Vùng chưa xác thực)..."
                                print(f"👀 [NAV EVENT] Bước {step_count}: Liếc ±{self.peek_angle_deg:.0f}° do cua {next_turn_deg:.0f}° vào vùng chưa thám hiểm...")
                                self.peek_scan_cb(self.peek_angle_deg)
                                time.sleep(0.08)
                            else:
                                print(f"⏩ [NAV EVENT] Bước {step_count}: Góc cua {next_turn_deg:.0f}° đã là Free Space -> TIẾP TỤC TIẾN DỨT KHOÁT!")
                        else:
                            print(f"⏩ [NAV EVENT] Bước {step_count}: Hướng đi thông suốt (góc cua {next_turn_deg:.0f}°), TIẾP TỤC TIẾN TỚI ĐÍCH!")

                    elif self.observe_mode == "full_360":
                        with self.lock:
                            self.state = self.STATE_OBSERVING
                            self.status_text = f"QUAN SÁT 360° CẬP NHẬT CẢN (BƯỚC {step_count})..."
                        print(f"📸 [NAV STEPPED] BƯỚC {step_count}: Dừng xe tĩnh, quay 45° 8 lần quét vật cản...")
                        if self.observe_scan_cb and not self._cancel_requested:
                            self.observe_scan_cb(rounds=1, is_initial=False)
                            time.sleep(0.15)

                    elif self.observe_mode in ("smart_active", "smart_peek", "peek_45"):
                        # Xác định waypoint tiếp theo để tính góc cua
                        next_turn_deg = 0.0
                        next_wp_ahead = None
                        for wp in self.waypoints:
                            if math.hypot(wp[0] - cur_x, wp[1] - cur_y) >= 0.12:
                                next_wp_ahead = wp
                                break
                        if next_wp_ahead is not None:
                            ahead_heading = math.atan2(next_wp_ahead[1] - cur_y, next_wp_ahead[0] - cur_x)
                            next_turn_deg = math.degrees(abs(self.normalize_angle(ahead_heading - cur_yaw)))

                        # Sự kiện 1: Sắp rẽ góc khuất (>= 30°) -> Liếc góc cua
                        need_peek = False
                        peek_reason = ""
                        if self.observe_mode == "peek_45":
                            need_peek = True
                            peek_reason = "chế độ cố định peek_45"
                        elif next_turn_deg >= self.peek_trigger_angle_deg:
                            # Kiểm tra xem khu vực rẽ tới đã là Free Space chưa (Costmap-Aware Peek)
                            is_turn_area_free = False
                            if self.costmap_aware_peek and self.is_area_free_cb and next_wp_ahead is not None:
                                is_turn_area_free = self.is_area_free_cb(next_wp_ahead[0], next_wp_ahead[1])

                            if is_turn_area_free:
                                need_peek = False
                                print(f"⏩ [NAV ACTIVE] BƯỚC {step_count}: Góc rẽ {next_turn_deg:.0f}° đã là vùng Free Space thông thoáng -> BỎ QUA LIẾC, TIẾP TỤC ĐI!")
                            else:
                                need_peek = True
                                peek_reason = f"sắp rẽ góc cua {next_turn_deg:.0f}° >= {self.peek_trigger_angle_deg:.0f}° vào vùng chưa xác thực"

                        # Sự kiện 2: Đi thẳng tích lũy vùng mù hai bên sườn -> Quét bán nguyệt
                        # CHỈ kích hoạt khi xe còn cách đích xa (> 1.50m) và tích lũy đủ quãng đường thẳng
                        need_sweep = False
                        sweep_reason = ""
                        if dist_to_goal > 1.50 and (straight_dist_accum >= self.sweep_interval_m or
                                straight_steps_accum >= self.sweep_interval_steps):
                            need_sweep = True
                            sweep_reason = (f"chạy thẳng liên tục {straight_dist_accum:.2f}m "
                                           f"({straight_steps_accum} bước) tích lũy vùng mù xa đích")

                        # THỰC THI QUAN SÁT THEO ĐỘ ƯU TIÊN:
                        if need_peek and self.peek_scan_cb and not self._cancel_requested:
                            with self.lock:
                                self.state = self.STATE_PEEKING
                                self.status_text = f"LIẾC GÓC CUA ±{self.peek_angle_deg:.0f}° ({peek_reason})..."
                            print(f"👀 [NAV ACTIVE] BƯỚC {step_count}: Liếc góc cua ±{self.peek_angle_deg:.0f}° do {peek_reason}...")
                            self.peek_scan_cb(self.peek_angle_deg)
                            straight_dist_accum = 0.0
                            straight_steps_accum = 0
                            time.sleep(0.12)

                        elif need_sweep and (self.sweep_scan_cb or self.peek_scan_cb) and not self._cancel_requested:
                            with self.lock:
                                self.state = self.STATE_SWEEPING
                                self.status_text = f"QUÉT BÁN NGUYỆT 180° BÙ ĐẮP VÙNG MÙ ({sweep_reason})..."
                            print(f"📡 [NAV ACTIVE] BƯỚC {step_count}: Quét bán nguyệt 180° do {sweep_reason}...")
                            if self.sweep_scan_cb:
                                self.sweep_scan_cb(self.sweep_angle_deg)
                            else:
                                self.peek_scan_cb(self.peek_angle_deg)
                            straight_dist_accum = 0.0
                            straight_steps_accum = 0
                            time.sleep(0.12)

                        else:
                            print(f"⏩ [NAV ACTIVE] BƯỚC {step_count}: Thẳng ({straight_dist_accum:.2f}m/"
                                  f"{self.sweep_interval_m:.2f}m, góc rẽ {next_turn_deg:.0f}°), bỏ qua quét!")
                    else:
                        # Chế độ "none" hoặc không quét
                        pass

            if step_count >= max_steps and self.is_running:
                self.cancel_navigation("Hết số bước tối đa cho phép (150 bước)")

        except Exception as e:
            print(f"❌ [NAV STEPPED] Lỗi trong luồng tự hành từng bước: {e}")
            self.cancel_navigation(f"Lỗi: {e}")
        finally:
            with self.lock:
                self.is_running = False
            self.stop_cb()

    def _navigation_loop(self) -> None:
        """Vòng lặp điều khiển thời gian thực bám đường liên tục (Pure Pursuit Fallback)"""
        rate_hz = 20.0
        dt = 1.0 / rate_hz

        last_wp_progress_time = time.time()
        last_wp_idx = self.current_wp_idx

        try:
            while self.is_running and not self._cancel_requested:
                loop_start = time.time()

                rx, ry, ryaw = self.get_pose_cb()
                total_wps = len(self.waypoints)

                if self.current_wp_idx >= total_wps:
                    self._complete_navigation("ĐÃ ĐẾN ĐÍCH AN TOÀN")
                    break

                target_x, target_y = self.waypoints[self.current_wp_idx]
                goal_x, goal_y = self.waypoints[-1]

                dx_target = target_x - rx
                dy_target = target_y - ry
                dist_to_target = math.hypot(dx_target, dy_target)

                dx_goal = goal_x - rx
                dy_goal = goal_y - ry
                self.dist_to_goal = math.hypot(dx_goal, dy_goal)

                if total_wps > 1:
                    self.progress_pct = min(99.0, (self.current_wp_idx / (total_wps - 1)) * 100.0)

                # 1. KIỂM TRA ĐẠT ĐÍCH CUỐI CÙNG (GOAL REACHED)
                is_last_wp = (self.current_wp_idx == total_wps - 1)
                if is_last_wp and (dist_to_target <= self.goal_tolerance_m or self.dist_to_goal <= self.goal_tolerance_m):
                    self.progress_pct = 100.0
                    self._complete_navigation(f"ĐÃ ĐẾN ĐÍCH! (Sai số: {self.dist_to_goal*100:.1f}cm)")
                    break

                # 2. KIỂM TRA ĐẠT WAYPOINT TRUNG GIAN
                if not is_last_wp and dist_to_target <= self.waypoint_tolerance_m:
                    self.current_wp_idx += 1
                    last_wp_progress_time = time.time()
                    print(f"📍 [NAVIGATOR] Đã đạt WP {self.current_wp_idx}/{total_wps}. Chuyển sang WP {self.current_wp_idx + 1}")
                    continue

                # 3. KIỂM TRA KẸT XE / KHÔNG TIẾN TRIỂN (STALL DETECTION)
                if self.current_wp_idx != last_wp_idx:
                    last_wp_idx = self.current_wp_idx
                    last_wp_progress_time = time.time()
                elif (time.time() - last_wp_progress_time) > self.stalled_timeout_s:
                    self.cancel_navigation("CẢNH BÁO: Kẹt xe hoặc trượt bánh quá lâu tại một waypoint")
                    self.state = self.STATE_STALLED
                    break

                # 4. TÍNH TOÁN GÓC HƯỚNG VÀ VẬN TỐC ĐIỀU KHIỂN
                target_heading = math.atan2(dy_target, dx_target)
                angle_err = self.normalize_angle(target_heading - ryaw)
                abs_angle_err = abs(angle_err)

                if abs_angle_err > self.heading_tolerance_rad:
                    self.state = self.STATE_TURNING
                    deg_err = math.degrees(abs_angle_err)
                    turn_dir_str = "TRÁI" if angle_err > 0 else "PHẢI"
                    self.status_text = f"QUAY {turn_dir_str} ({deg_err:.0f}°) -> WP {self.current_wp_idx + 1}/{total_wps}"
                    turn_sign = 1.0 if angle_err > 0 else -1.0
                    cmd_v = 0.0
                    cmd_w = turn_sign * self.turn_speed
                else:
                    self.state = self.STATE_DRIVING
                    self.status_text = f"TIẾN VỀ WP {self.current_wp_idx + 1}/{total_wps} (còn {self.dist_to_goal:.2f}m)"
                    if is_last_wp and dist_to_target < 0.25:
                        drive_v = max(0.06, self.linear_speed * (dist_to_target / 0.25))
                    else:
                        drive_v = self.linear_speed
                    cmd_v = drive_v
                    cmd_w = max(-1.0, min(1.0, self.kp_angular * angle_err))

                self.set_cmd_vel_cb(cmd_v, cmd_w)
                elapsed = time.time() - loop_start
                sleep_t = max(0.005, dt - elapsed)
                time.sleep(sleep_t)

        except Exception as e:
            print(f"❌ [NAVIGATOR] Lỗi trong vòng lặp điều khiển: {e}")
            self.cancel_navigation(f"Lỗi: {e}")
        finally:
            with self.lock:
                self.is_running = False
            self.stop_cb()

    def _complete_navigation(self, message: str) -> None:
        """Xử lý khi xe đã hoàn thành toàn bộ lộ trình thành công"""
        with self.lock:
            self.is_running = False
            self.state = self.STATE_GOAL_REACHED
            self.status_text = message
            print(f"🏁 [NAVIGATOR] {message}")
        self.stop_cb()


if __name__ == '__main__':
    print("\n" + "═" * 70)
    print("🧪 [SELF-TEST] KIỂM THỬ ĐỘC LẬP AUTONOMOUS NAVIGATOR")
    print("═" * 70)

    # Giả lập robot pose
    sim_pose = [0.0, 0.0, 0.0]  # x, y, yaw

    def fake_get_pose():
        return sim_pose[0], sim_pose[1], sim_pose[2]

    def fake_set_cmd_vel(v, w):
        dt = 0.05
        sim_pose[2] += w * dt
        sim_pose[2] = AutonomousNavigator.normalize_angle(sim_pose[2])
        sim_pose[0] += v * dt * math.cos(sim_pose[2])
        sim_pose[1] += v * dt * math.sin(sim_pose[2])

    def fake_stop():
        pass

    nav = AutonomousNavigator(
        get_pose_cb=fake_get_pose,
        set_cmd_vel_cb=fake_set_cmd_vel,
        stop_cb=fake_stop,
        linear_speed=0.20,
        turn_speed=1.5,
        goal_tolerance_m=0.05,
        waypoint_tolerance_m=0.08
    )

    test_path = [(0.0, 0.0), (0.3, 0.0), (0.3, 0.3), (0.0, 0.3)]
    res = nav.start_navigation(test_path)
    print(f"Khởi chạy tự hành: {res}")

    start_t = time.time()
    while nav.is_running and (time.time() - start_t < 10.0):
        st = nav.get_status()
        x, y, yaw = fake_get_pose()
        print(f"  Trạng thái: {st['state']} | WP {st['current_wp_idx'] + 1}/{st['total_waypoints']} | Robot: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) | Đích còn: {st['dist_to_goal']:.2f}m")
        time.sleep(0.4)

    final_st = nav.get_status()
    print("═" * 70)
    print(f"Kết quả cuối: State={final_st['state']} | Status={final_st['status_text']}")
    print("✅ TEST THÀNH CÔNG!" if final_st['state'] == AutonomousNavigator.STATE_GOAL_REACHED else "❌ TEST THẤT BẠI")
    print("═" * 70 + "\n")
