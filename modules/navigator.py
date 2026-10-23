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
    STATE_REPLANNING = "REPLANNING"

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
        mode: str = "stepped",              # "stepped" (Sense-Plan-Act 30cm) hoặc "continuous" (Pure Pursuit)
        step_dist_cm: float = 30.0,         # Cự ly mỗi bước tiến/lùi (30cm)
        init_scan_rounds: int = 2           # Số vòng quay 360° ban đầu (2 vòng theo yêu cầu người dùng)
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

        # Tham số Tự hành Từng bước (Stepped Sense-Plan-Act 30cm)
        self.step_move_sync_cb = step_move_sync_cb
        self.observe_scan_cb = observe_scan_cb
        self.replan_cb = replan_cb
        self.cancel_step_cb = cancel_step_cb
        self.mode = str(mode).lower()
        self.step_dist_cm = float(step_dist_cm)
        self.init_scan_rounds = int(init_scan_rounds)

        # Trạng thái runtime
        self.is_running = False
        self.state = self.STATE_IDLE
        self.status_text = "SẴN SÀNG"
        self.waypoints: List[Tuple[float, float]] = []
        self.goal: Optional[Tuple[float, float]] = None
        self.current_wp_idx = 0
        self.dist_to_goal = 0.0
        self.progress_pct = 0.0

        self.lock = threading.RLock()
        self._nav_thread: Optional[threading.Thread] = None
        self._cancel_requested = False

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

            # Hủy tiến trình cũ nếu đang chạy
            self.cancel_navigation()

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

    def _stepped_navigation_loop(self) -> None:
        """
        Vòng lặp tự hành từng bước chuẩn công nghiệp (Stepped Sense-Plan-Act 30cm):
        1. KHỞI TẠO: Quay 360° đúng 2 lần (2 vòng x 8 nấc 45° = 16 nấc) theo yêu cầu người dùng để nạp bản đồ ban đầu cực kỳ vững chắc.
        2. VÒNG LẶP ĐIỀU HƯỚNG TỪNG BƯỚC:
           a. TÁI LẬP KẾ HOẠCH (REPLAN):
              - Chạy A* từ vị trí robot hiện tại tới Đích (Goal).
              - Nếu không tìm thấy đường (bị cản chặn hoàn toàn) -> báo OBSTACLE_BLOCKED và dừng.
              - Cập nhật waypoints mới.
           b. KIỂM TRA ĐÍCH (GOAL CHECK):
              - Nếu khoảng cách tới Đích <= goal_tolerance_m (6cm) -> GOAL_REACHED! Hoàn thành!
           c. ĐỊNH HƯỚNG (TURN TO HEADING):
              - Chọn waypoint mục tiêu phía trước.
              - Tính góc lệch hướng xe: Δψ = normalize_angle(ψ_target - ryaw).
              - Nếu |Δψ| > 12°:
                Quay tại chỗ đúng góc |Δψ| (Trái/Phải) bằng bước quay định lượng có IMU bảo hộ.
           d. TIẾN BƯỚC (STEP MOVE 30CM):
              - Tính cự ly tới waypoint / đích.
              - Bước tiến: S = min(self.step_dist_cm, dist_to_target_cm).
              - Thực thi bước tiến S cm bằng S-Curve 50Hz.
           e. KIỂM TRA LẠI ĐÍCH:
              - Nếu đã tới đích -> GOAL_REACHED!
           f. QUAN SÁT 360° CẬP NHẬT VẬT CẢN (OBSERVE SURROUNDINGS):
              - Dừng tĩnh, quay 45° đúng 8 lần (1 vòng 360°) để nạp toàn bộ các vật cản mới xuất hiện vào bản đồ.
           g. Lặp lại bước a.
        """
        try:
            cur_x, cur_y, cur_yaw = self.get_pose_cb()
            goal_x, goal_y = self.goal if self.goal else self.waypoints[-1]

            # ── 1. KHỞI TẠO: QUAY 360° ĐÚNG 2 LẦN THEO YÊU CẦU NGƯỜI DÙNG ──
            init_rounds = getattr(self, 'init_scan_rounds', 2)
            if init_rounds > 0 and self.observe_scan_cb and not self._cancel_requested:
                print(f"🔄 [NAV STEPPED] Bắt đầu chu trình quét 360° khởi tạo {init_rounds} vòng (mỗi vòng 8 nấc 45°)...")
                for r in range(1, init_rounds + 1):
                    if self._cancel_requested:
                        break
                    with self.lock:
                        self.state = self.STATE_INIT_SCAN
                        self.status_text = f"QUÉT 360° KHỞI TẠO: VÒNG {r}/{init_rounds} (8 nấc 45°)..."
                    print(f"📸 [NAV STEPPED] Quét khởi tạo vòng {r}/{init_rounds}...")
                    is_last_init = (r == init_rounds)
                    self.observe_scan_cb(rounds=1, is_initial=is_last_init)
                    time.sleep(0.2)

            if self._cancel_requested:
                return

            step_count = 0
            max_steps = 150  # Giới hạn an toàn chống lặp vô tận
            replan_fail_streak = 0

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

                # ── a. TÁI LẬP KẾ HOẠCH ĐƯỜNG ĐI A* (REPLANNING) ──
                with self.lock:
                    self.state = self.STATE_REPLANNING
                    self.status_text = f"TÍNH TOÁN ĐƯỜNG A* (BƯỚC {step_count})... CÒN {dist_to_goal:.2f}M"
                print(f"🧠 [NAV STEPPED] BƯỚC {step_count}: Tính toán lộ trình A* tới đích ({goal_x:.2f}, {goal_y:.2f})...")

                if self.replan_cb:
                    new_path = self.replan_cb(goal_x, goal_y)
                    if new_path and len(new_path) >= 1:
                        self.waypoints = [tuple(p) for p in new_path]
                        self.current_wp_idx = 0
                        replan_fail_streak = 0
                    else:
                        replan_fail_streak += 1
                        print(f"⚠️ [NAV STEPPED] Replan A* chưa tìm được đường mới (thử lần {replan_fail_streak}/3)...")
                        # Kiểm tra xem lộ trình hiện hữu có còn điểm nào phía trước không
                        has_valid_future_wp = any(math.hypot(wp[0] - cur_x, wp[1] - cur_y) >= 0.08 for wp in self.waypoints)
                        if has_valid_future_wp and replan_fail_streak < 3:
                            print("ℹ️ [NAV STEPPED] Tiếp tục duy trì lộ trình hiện hữu để thoát khỏi vùng hẹp!")
                        else:
                            print(f"⚠️ [NAV STEPPED] DỪNG TỰ HÀNH: Không tìm thấy đường đi khả dụng sau {replan_fail_streak} lần thử!")
                            self.state = self.STATE_OBSTACLE_BLOCKED
                            self.status_text = "DỪNG: BỊ CẢN CHẶN HOÀN TOÀN KHÔNG CÓ ĐƯỜNG ĐI"
                            break

                if self._cancel_requested:
                    break

                # Tìm waypoint mục tiêu phía trước (cách xe ít nhất 8cm)
                target_wp = None
                for wp in self.waypoints:
                    d_wp = math.hypot(wp[0] - cur_x, wp[1] - cur_y)
                    if d_wp >= 0.08:
                        target_wp = wp
                        break
                if target_wp is None:
                    target_wp = (goal_x, goal_y)

                dx_target = target_wp[0] - cur_x
                dy_target = target_wp[1] - cur_y
                dist_to_wp = math.hypot(dx_target, dy_target)

                # ── b. ĐỊNH HƯỚNG VỀ PHÍA WAYPOINT (TURN TO HEADING) ──
                target_heading = math.atan2(dy_target, dx_target)
                angle_err = self.normalize_angle(target_heading - cur_yaw)
                abs_angle_deg = math.degrees(abs(angle_err))

                if abs_angle_deg > 12.0:
                    turn_dir = 'left' if angle_err > 0 else 'right'
                    with self.lock:
                        self.state = self.STATE_TURNING
                        self.status_text = f"XOAY {'TRÁI' if turn_dir == 'left' else 'PHẢI'} {abs_angle_deg:.0f}° HƯỚNG VỀ MỤC TIÊU..."
                    print(f"🔄 [NAV STEPPED] BƯỚC {step_count}: Xoay {turn_dir} {abs_angle_deg:.1f}° theo IMU...")

                    if self.step_move_sync_cb:
                        self.step_move_sync_cb('turn', abs_angle_deg, turn_dir, None)
                    time.sleep(0.12)
                    # Cập nhật lại tọa độ và hướng sau khi xoay để bước tiến đi thẳng chuẩn xác
                    cur_x, cur_y, cur_yaw = self.get_pose_cb()
                    dx_target = target_wp[0] - cur_x
                    dy_target = target_wp[1] - cur_y
                    dist_to_wp = math.hypot(dx_target, dy_target)

                if self._cancel_requested:
                    break

                # ── c. TIẾN BƯỚC ĐỊNH LƯỢNG (STEP ADVANCE MAX 30CM) ──
                step_dist_m = min(self.step_dist_cm / 100.0, dist_to_wp)
                step_dist_m = min(step_dist_m, dist_to_goal)
                step_cm = max(5.0, round(step_dist_m * 100.0, 1))

                with self.lock:
                    self.state = self.STATE_DRIVING
                    self.status_text = f"TIẾN BƯỚC {step_cm:.0f}CM (BƯỚC {step_count})... CÒN {dist_to_goal:.2f}M"
                print(f"📏 [NAV STEPPED] BƯỚC {step_count}: Tiến {step_cm:.0f}cm bằng Cosine S-Curve...")

                if self.step_move_sync_cb:
                    self.step_move_sync_cb('linear', step_cm, 'forward', None)
                time.sleep(0.12)

                if self._cancel_requested:
                    break

                # Kiểm tra sau khi bước: nếu đã chạm đích (<= 6cm) thì hoàn thành luôn!
                cur_x, cur_y, cur_yaw = self.get_pose_cb()
                dist_after = math.hypot(goal_x - cur_x, goal_y - cur_y)
                self.dist_to_goal = dist_after
                if dist_after <= self.goal_tolerance_m:
                    self.progress_pct = 100.0
                    self._complete_navigation(f"ĐÃ ĐẾN ĐÍCH! (Sai số: {dist_after*100:.1f}cm)")
                    break

                # ── d. DỪNG LẠI QUAN SÁT XUNG QUANH BẰNG CÁCH QUAY 45° 8 LẦN (OBSERVE 360°) ──
                with self.lock:
                    self.state = self.STATE_OBSERVING
                    self.status_text = f"QUAN SÁT 360° CẬP NHẬT CẢN (BƯỚC {step_count})..."
                print(f"📸 [NAV STEPPED] BƯỚC {step_count}: Dừng xe tĩnh, quay 45° 8 lần quét vật cản...")

                if self.observe_scan_cb and not self._cancel_requested:
                    self.observe_scan_cb(rounds=1, is_initial=False)
                    time.sleep(0.15)

            if step_count >= max_steps and self.is_running:
                self.cancel_navigation("Hết số bước tối đa cho phép (150 bước)")

        except Exception as e:
            print(f"❌ [NAV STEPPED] Lỗi trong luồng tự hành từng bước: {e}")
            self.cancel_navigation(f"Lỗi: {e}")
        finally:
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
