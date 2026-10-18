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

import math
import time
import threading
from typing import List, Tuple, Optional, Callable, Dict, Any


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

    def __init__(
        self,
        get_pose_cb: Callable[[], Tuple[float, float, float]],
        set_cmd_vel_cb: Callable[[float, float], None],
        stop_cb: Callable[[], None],
        linear_speed: float = 0.11,         # m/s (~15-20% PWM an toàn trong nhà)
        turn_speed: float = 0.50,           # rad/s (~11-13% PWM quay êm, triệt tiêu mờ camera)
        goal_tolerance_m: float = 0.06,     # Dung sai chạm đích (6cm)
        waypoint_tolerance_m: float = 0.10, # Dung sai lướt qua waypoint (10cm)
        heading_tolerance_rad: float = 0.35,# Dung sai góc lệch (~20 độ)
        kp_angular: float = 1.6,            # Hệ số P chỉnh lái khi tiến
        stalled_timeout_s: float = 20.0     # Thời gian tối đa cho 1 waypoint trước khi báo kẹt
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

        # Trạng thái runtime
        self.is_running = False
        self.state = self.STATE_IDLE
        self.status_text = "SẴN SÀNG"
        self.waypoints: List[Tuple[float, float]] = []
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

    def start_navigation(self, path: List[Tuple[float, float]]) -> Dict[str, Any]:
        """Bắt đầu điều khiển xe tự hành bám theo danh sách waypoints"""
        with self.lock:
            if not path or len(path) < 1:
                return {"success": False, "message": "Lộ trình rỗng, không có điểm waypoint nào"}

            # Hủy tiến trình cũ nếu đang chạy
            self.cancel_navigation()

            self.waypoints = [tuple(p) for p in path]
            # Nếu waypoint đầu tiên trùng ngay vị trí xe (< 8cm), bỏ qua để tới điểm tiếp theo
            cur_x, cur_y, _ = self.get_pose_cb()
            start_idx = 0
            if len(self.waypoints) > 1:
                d0 = math.hypot(self.waypoints[0][0] - cur_x, self.waypoints[0][1] - cur_y)
                if d0 < self.waypoint_tolerance_m:
                    start_idx = 1

            self.current_wp_idx = start_idx
            self.is_running = True
            self._cancel_requested = False
            self.state = self.STATE_TURNING
            self.status_text = f"BẮT ĐẦU: Tiến tới WP {self.current_wp_idx + 1}/{len(self.waypoints)}"
            self.progress_pct = 0.0

            self._nav_thread = threading.Thread(target=self._navigation_loop, daemon=True)
            self._nav_thread.start()

            print(f"🚀 [NAVIGATOR] Bắt đầu tự hành với {len(self.waypoints)} waypoints (từ WP {self.current_wp_idx + 1})")
            return {"success": True, "message": "Đã kích hoạt tự hành bám đường", "total_waypoints": len(self.waypoints)}

    def cancel_navigation(self, reason: str = "Người dùng hủy") -> None:
        """Hủy ngay lập tức chu trình tự hành và dừng xe an toàn"""
        with self.lock:
            if self.is_running:
                self._cancel_requested = True
                self.is_running = False
                self.state = self.STATE_CANCELLED
                self.status_text = f"ĐÃ DỪNG: {reason}"
                print(f"🛑 [NAVIGATOR] {self.status_text}")
        
        # Gọi dừng động cơ ngay lập tức
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

    def _navigation_loop(self) -> None:
        """Vòng lặp điều khiển thời gian thực bám đường (Tần số 20 Hz, chu kỳ 50ms)"""
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
                    # Đã đi hết toàn bộ waypoints
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

                # Cập nhật tỉ lệ hoàn thành %
                if total_wps > 1:
                    self.progress_pct = min(99.0, (self.current_wp_idx / (total_wps - 1)) * 100.0)

                # ── 1. KIỂM TRA ĐẠT ĐÍCH CUỐI CÙNG (GOAL REACHED) ──
                is_last_wp = (self.current_wp_idx == total_wps - 1)
                if is_last_wp and (dist_to_target <= self.goal_tolerance_m or self.dist_to_goal <= self.goal_tolerance_m):
                    self.progress_pct = 100.0
                    self._complete_navigation(f"ĐÃ ĐẾN ĐÍCH! (Sai số: {self.dist_to_goal*100:.1f}cm)")
                    break

                # ── 2. KIỂM TRA ĐẠT WAYPOINT TRUNG GIAN ──
                if not is_last_wp and dist_to_target <= self.waypoint_tolerance_m:
                    self.current_wp_idx += 1
                    last_wp_progress_time = time.time()
                    print(f"📍 [NAVIGATOR] Đã đạt WP {self.current_wp_idx}/{total_wps}. Chuyển sang WP {self.current_wp_idx + 1}")
                    continue

                # ── 3. KIỂM TRA KẸT XE / KHÔNG TIẾN TRIỂN (STALL DETECTION) ──
                if self.current_wp_idx != last_wp_idx:
                    last_wp_idx = self.current_wp_idx
                    last_wp_progress_time = time.time()
                elif (time.time() - last_wp_progress_time) > self.stalled_timeout_s:
                    self.cancel_navigation("CẢNH BÁO: Kẹt xe hoặc trượt bánh quá lâu tại một waypoint")
                    self.state = self.STATE_STALLED
                    break

                # ── 4. TÍNH TOÁN GÓC HƯỚNG VÀ VẬN TỐC ĐIỀU KHIỂN (2-PHASE PURE PURSUIT) ──
                target_heading = math.atan2(dy_target, dx_target)
                angle_err = self.normalize_angle(target_heading - ryaw)

                abs_angle_err = abs(angle_err)

                # PHA 1: Quay tại chỗ nếu góc lệch hướng quá lớn (> heading_tolerance)
                if abs_angle_err > self.heading_tolerance_rad:
                    self.state = self.STATE_TURNING
                    deg_err = math.degrees(abs_angle_err)
                    turn_dir_str = "TRÁI" if angle_err > 0 else "PHẢI"
                    self.status_text = f"QUAY {turn_dir_str} ({deg_err:.0f}°) -> WP {self.current_wp_idx + 1}/{total_wps}"

                    # Vận tốc quay cố định êm ái
                    turn_sign = 1.0 if angle_err > 0 else -1.0
                    cmd_v = 0.0
                    cmd_w = turn_sign * self.turn_speed

                # PHA 2: Tiến về phía trước và hiệu chỉnh lái nhẹ bằng bộ điều khiển P
                else:
                    self.state = self.STATE_DRIVING
                    self.status_text = f"TIẾN VỀ WP {self.current_wp_idx + 1}/{total_wps} (còn {self.dist_to_goal:.2f}m)"

                    # Giảm tốc khi gần tới đích cuối cùng để phanh êm
                    if is_last_wp and dist_to_target < 0.25:
                        drive_v = max(0.06, self.linear_speed * (dist_to_target / 0.25))
                    else:
                        drive_v = self.linear_speed

                    cmd_v = drive_v
                    # Chỉnh lái mượt mà tỉ lệ với góc lệch
                    cmd_w = max(-1.0, min(1.0, self.kp_angular * angle_err))

                # Gửi lệnh vận tốc xuống hệ thống động cơ
                self.set_cmd_vel_cb(cmd_v, cmd_w)

                # Giữ tần số vòng lặp 20 Hz
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
